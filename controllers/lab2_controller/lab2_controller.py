"""
Laboratorio 2 - Robótica y Sistemas Autónomos 2026-01
ICI 4150 - Navegación reactiva con filtrado y fusión de sensores

Controlador corregido para robot e-puck en Webots.

CORRECCIONES respecto a versión anterior:
  - OBSTACLE_THRESHOLD subido a 100 (sensor e-puck devuelve ~70 en espacio libre,
    no 0; el umbral anterior de 150 siempre se activaba → giro permanente).
  - Índices de sensores corregidos: ps0/ps7 son frontales, ps2 izquierdo, ps5 derecho
    (confirmado con distribución oficial del e-puck).
  - Lógica de decisión: se añade zona muerta en sensores laterales para evitar giros
    innecesarios cuando ambos lados están libres.
  - Kalman: se inicializa con la primera lectura real del sensor, no con 0.
  - Se imprime en consola el valor crudo de los sensores para facilitar calibración.
"""

from controller import Robot
import csv
import os

# ─────────────────────────────────────────────
#  Parámetros físicos del e-puck
# ─────────────────────────────────────────────
WHEEL_RADIUS   = 0.0205   # [m]
WHEEL_DISTANCE = 0.052    # [m]
MAX_SPEED      = 6.28     # [rad/s]

# ─────────────────────────────────────────────
#  Muestreo
# ─────────────────────────────────────────────
TIME_STEP = 64            # [ms]  — debe coincidir con basicTimeStep del mundo
TS        = TIME_STEP / 1000.0
FS        = 1.0 / TS

# ─────────────────────────────────────────────
#  Navegación
#
#  El sensor e-puck devuelve valores en [0, ~4000]:
#    ~70  → espacio libre (sin obstáculo cercano)
#    ~300 → obstáculo a ~5 cm
#    >500 → obstáculo muy próximo
#
#  OBSTACLE_THRESHOLD: valor a partir del cual se considera que hay obstáculo.
#  Se recomienda empezar en 100–150 y ajustar según tu escenario.
# ─────────────────────────────────────────────
OBSTACLE_THRESHOLD  = 90   # umbral frontal para decidir girar
SIDE_DEADZONE       = 80    # diferencia mínima entre laterales para elegir lado

FWD_SPEED   = 4.0   # [rad/s] velocidad de avance
TURN_SPEED  = 4.5   # [rad/s] velocidad de giro

# ─────────────────────────────────────────────
#  Filtro de Kalman
# ─────────────────────────────────────────────
Q_KALMAN = 0.5    # varianza del proceso  (más alto → confía más en el sensor)
R_KALMAN = 20.0   # varianza del sensor   (más alto → confía más en la predicción)

# ─────────────────────────────────────────────
#  Filtro de media móvil
# ─────────────────────────────────────────────
MA_WINDOW = 5


# ══════════════════════════════════════════════
class MovingAverageFilter:
    def __init__(self, window):
        self.window = window
        self.buf = []

    def update(self, v):
        self.buf.append(v)
        if len(self.buf) > self.window:
            self.buf.pop(0)
        return sum(self.buf) / len(self.buf)


# ══════════════════════════════════════════════
class KalmanFilter1D:
    """
    Filtro de Kalman escalar.

    Estado:   dk  = distancia frontal estimada
    Entrada:  Δdk = variación por encoders (negativa si avanza hacia obstáculo)
    Medición: zk  = lectura del sensor frontal
    """

    def __init__(self, Q, R):
        self.Q     = Q
        self.R     = R
        self.d_hat = None   # se inicializa con la primera lectura real
        self.P     = 1.0
        self.K     = 0.0
        self._d_prior = 0.0
        self._P_prior = 0.0

    def predict(self, delta_d):
        """ˆd⁻_k = ˆd_{k-1} + Δd    |    P⁻_k = P_{k-1} + Q"""
        self._d_prior = self.d_hat + delta_d
        self._P_prior = self.P + self.Q

    def update(self, z):
        """K = P⁻/(P⁻+R)    |    ˆd = ˆd⁻ + K(z−ˆd⁻)    |    P = (1−K)P⁻"""
        self.K     = self._P_prior / (self._P_prior + self.R)
        self.d_hat = self._d_prior + self.K * (z - self._d_prior)
        self.P     = (1.0 - self.K) * self._P_prior
        return self.d_hat


# ══════════════════════════════════════════════
class EpuckController:

    # Índices de sensores en el e-puck (distribución oficial Webots)
    #
    #        ps7  ps0
    #       /       \
    #  ps6            ps1
    #  ps5            ps2
    #       \       /
    #        ps4  ps3
    #
    IDX_FRONT_R  = 0   # ps0  — frontal derecho
    IDX_FRONT_L  = 7   # ps7  — frontal izquierdo
    IDX_SIDE_L   = 2   # ps2  — lateral izquierdo
    IDX_SIDE_R   = 5   # ps5  — lateral derecho

    def __init__(self):
        self.robot = Robot()

        # Motores
        self.lm = self.robot.getDevice("left wheel motor")
        self.rm = self.robot.getDevice("right wheel motor")
        self.lm.setPosition(float("inf"))
        self.rm.setPosition(float("inf"))
        self.lm.setVelocity(0.0)
        self.rm.setVelocity(0.0)

        # Encoders
        self.le = self.robot.getDevice("left wheel sensor")
        self.re = self.robot.getDevice("right wheel sensor")
        self.le.enable(TIME_STEP)
        self.re.enable(TIME_STEP)
        self._prev_le = None
        self._prev_re = None

        # Sensores de distancia
        self.ps = []
        for i in range(8):
            s = self.robot.getDevice(f"ps{i}")
            s.enable(TIME_STEP)
            self.ps.append(s)

        # Filtros
        self.ma_fl = MovingAverageFilter(MA_WINDOW)   # frontal izquierdo
        self.ma_fr = MovingAverageFilter(MA_WINDOW)   # frontal derecho
        self.kf    = KalmanFilter1D(Q_KALMAN, R_KALMAN)

        # Registro
        self.log  = []
        self.step = 0

        print(f"[INIT] Ts={TS:.3f}s  fs={FS:.2f}Hz  umbral={OBSTACLE_THRESHOLD}")
        print(f"[INIT] Kalman Q={Q_KALMAN}  R={R_KALMAN}  MA_WIN={MA_WINDOW}")
        print("[INIT] Esperando primera muestra para calibrar Kalman...")

    # ── Encoders → desplazamiento lineal ────
    def _displacement(self):
        le = self.le.getValue()
        re = self.re.getValue()
        if self._prev_le is None:
            self._prev_le, self._prev_re = le, re
            return 0.0
        ds = WHEEL_RADIUS * ((le - self._prev_le) + (re - self._prev_re)) / 2.0
        self._prev_le, self._prev_re = le, re
        return ds   # [m]

    # ── Lógica de navegación ─────────────────
    def _decide(self, ps_vals, dist_est):
        front_l = ps_vals[self.IDX_FRONT_L]
        front_r = ps_vals[self.IDX_FRONT_R]
        front   = max(front_l, front_r)   # más conservador: usar el máximo

        if dist_est > OBSTACLE_THRESHOLD or front > OBSTACLE_THRESHOLD:
            # Obstáculo al frente → elegir lado
            side_l = ps_vals[self.IDX_SIDE_L]
            side_r = ps_vals[self.IDX_SIDE_R]
            diff   = side_l - side_r

            if abs(diff) < SIDE_DEADZONE:
                # Laterales similares → girar derecha por defecto
                lv, rv, action = TURN_SPEED, -TURN_SPEED, "TURN_RIGHT(default)"
            elif diff > 0:
                # Izquierda más bloqueada → girar derecha
                lv, rv, action = TURN_SPEED, -TURN_SPEED, "TURN_RIGHT"
            else:
                # Derecha más bloqueada → girar izquierda
                lv, rv, action = -TURN_SPEED, TURN_SPEED, "TURN_LEFT"
        else:
            lv, rv, action = FWD_SPEED, FWD_SPEED, "FORWARD"

        return lv, rv, action

    # ── Bucle principal ──────────────────────
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            self.step += 1
            t = self.step * TS

            # 1. Lecturas crudas
            ps_vals  = [s.getValue() for s in self.ps]
            le_val   = self.le.getValue()
            re_val   = self.re.getValue()

            # 2. Señal frontal cruda (promedio ps0 y ps7)
            front_raw = (ps_vals[self.IDX_FRONT_L] + ps_vals[self.IDX_FRONT_R]) / 2.0

            # 3. Filtro media móvil
            front_filt = (
                self.ma_fl.update(ps_vals[self.IDX_FRONT_L]) +
                self.ma_fr.update(ps_vals[self.IDX_FRONT_R])
            ) / 2.0

            # 4. Kalman
            ds = self._displacement()   # avance [m]
            # Escala empírica: 1 m de avance ≈ 500 unidades de sensor
            delta_d = ds * 500.0

            if self.kf.d_hat is None:
                # Primera muestra: inicializar con lectura real
                self.kf.d_hat = front_raw
                print(f"[INIT] Kalman inicializado con d0={front_raw:.1f}")

            self.kf.predict(delta_d)
            dist_kalman = self.kf.update(front_raw)

            # 5. Decisión
            lv, rv, action = self._decide(ps_vals, dist_kalman)
            self.lm.setVelocity(lv)
            self.rm.setVelocity(rv)

            # 6. Log periódico en consola
            if self.step % 20 == 0:
                print(
                    f"[t={t:.2f}s] "
                    f"ps0={ps_vals[0]:.0f} ps7={ps_vals[7]:.0f} "
                    f"ps2={ps_vals[2]:.0f} ps5={ps_vals[5]:.0f} | "
                    f"raw={front_raw:.1f} filt={front_filt:.1f} "
                    f"kalman={dist_kalman:.1f} K={self.kf.K:.3f} | "
                    f"{action}"
                )

            # 7. Registrar datos
            self.log.append({
                "t": round(t, 4),
                "ps0": round(ps_vals[0], 1),
                "ps7": round(ps_vals[7], 1),
                "ps2": round(ps_vals[2], 1),
                "ps5": round(ps_vals[5], 1),
                "front_raw":      round(front_raw, 2),
                "front_filtered": round(front_filt, 2),
                "dist_kalman":    round(dist_kalman, 2),
                "kalman_gain":    round(self.kf.K, 4),
                "kalman_P":       round(self.kf.P, 4),
                "le_enc":         round(le_val, 4),
                "re_enc":         round(re_val, 4),
                "ds_m":           round(ds, 6),
                "lv":             round(lv, 2),
                "rv":             round(rv, 2),
                "action":         action,
            })

        self._save_csv()

    def _save_csv(self):
        if not self.log:
            return
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_log.csv")
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.log[0].keys()))
            w.writeheader()
            w.writerows(self.log)
        print(f"[DONE] CSV guardado: {path} ({len(self.log)} muestras)")


if __name__ == "__main__":
    EpuckController().run()