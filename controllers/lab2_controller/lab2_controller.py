"""
Laboratorio 2 - Robótica y Sistemas Autónomos 2026-01
ICI 4150 - Navegación reactiva con filtrado y fusión de sensores

CORRECCIONES en esta versión:
  1. Signo de delta_d CORREGIDO: avanzar hacia un obstáculo REDUCE la distancia
     estimada (delta_d = -ds * SENSOR_SCALE). La versión anterior sumaba, lo que
     hacía que Kalman divergiera hacia valores altos y nunca detectara obstáculos.
  2. OBSTACLE_THRESHOLD unificado en 90 u.a. (coherente con el código; el README
     decía 150 pero eso era un valor desactualizado).
  3. Lógica de escape de esquina: si el robot lleva más de MAX_TURN_STEPS pasos
     girando y el obstáculo sigue presente, fuerza un giro prolongado para
     garantizar que se aleje de la pared.
  4. Detección de pared trasera/lateral extrema: si ps3/ps4 (traseros) o ps1/ps6
     (laterales interiores) superan el umbral, el robot retrocede brevemente.
  5. Nombres de columnas en CSV corregidos para coincidir con plot_signals.py:
     left_enc → left_enc  (sin cambio)
     re_enc → right_enc   (renombrado)
     ds_m → linear_adv_m  (renombrado)
     ps2 → ps2_raw, ps5 → ps5_raw  (renombrados)
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
TIME_STEP = 64            # [ms]
TS        = TIME_STEP / 1000.0
FS        = 1.0 / TS

# ─────────────────────────────────────────────
#  Navegación
#
#  El sensor e-puck devuelve valores en [0, ~4000]:
#    ~70  → espacio libre
#    ~300 → obstáculo a ~5 cm
#    >500 → obstáculo muy próximo
# ─────────────────────────────────────────────
OBSTACLE_THRESHOLD  = 100    # umbral frontal para decidir girar [u.a.]
SIDE_DEADZONE       = 80    # diferencia mínima entre laterales para elegir lado
REAR_THRESHOLD      = 120   # umbral para sensores traseros (retroceso de emergencia)

FWD_SPEED    = 4.0    # [rad/s] velocidad de avance
TURN_SPEED   = 4.5    # [rad/s] velocidad de giro
BACK_SPEED   = 2.5    # [rad/s] velocidad de retroceso

# Pasos máximos en modo giro antes de forzar escape
MAX_TURN_STEPS = 25   # 25 × 64 ms ≈ 1.6 s

# ─────────────────────────────────────────────
#  Filtro de Kalman
#  (Q pequeño → confía más en el modelo;
#   R grande  → sensor muy ruidoso)
# ─────────────────────────────────────────────
Q_KALMAN = 0.5
R_KALMAN = 20.0

# ─────────────────────────────────────────────
#  Filtro de media móvil
# ─────────────────────────────────────────────
MA_WINDOW = 5

# ─────────────────────────────────────────────
#  Escala empírica encoder → unidades sensor
#  Ajustar si el Kalman converge demasiado lento
#  o demasiado rápido.
# ─────────────────────────────────────────────
SENSOR_SCALE = 500.0   # [u.a. / m]


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
    Filtro de Kalman escalar para distancia frontal.

    Estado:    d̂_k  = distancia frontal estimada [u.a.]
    Entrada:   Δd_k  = variación predicha por encoders
                       NEGATIVA cuando el robot avanza hacia el obstáculo
    Medición:  z_k   = lectura del sensor frontal [u.a.]
    """

    def __init__(self, Q, R):
        self.Q      = Q
        self.R      = R
        self.d_hat  = None   # inicializado con la primera lectura real
        self.P      = 1.0
        self.K      = 0.0
        self._d_prior = 0.0
        self._P_prior = 0.0

    def predict(self, delta_d):
        """
        ˆd⁻_k = ˆd_{k-1} + Δd_k
        P⁻_k  = P_{k-1} + Q

        IMPORTANTE: delta_d debe ser NEGATIVO cuando el robot avanza
        (se acerca al obstáculo), POSITIVO cuando retrocede.
        """
        self._d_prior = self.d_hat + delta_d
        self._P_prior = self.P + self.Q

    def update(self, z):
        """K = P⁻/(P⁻+R)  |  ˆd = ˆd⁻ + K(z−ˆd⁻)  |  P = (1−K)P⁻"""
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
    IDX_SIDE_L   = 6   # ps6  — lateral izquierdo (más alejado del frente)
    IDX_SIDE_R   = 1   # ps1  — lateral derecho
    IDX_DIAG_L   = 1   # ps2  — diagonal izquierdo (45°)  ← usado para giro
    IDX_DIAG_R   = 6   # ps5  — diagonal derecho  (45°)   ← usado para giro
    IDX_REAR_L   = 3   # ps3  — trasero izquierdo
    IDX_REAR_R   = 4   # ps4  — trasero derecho

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

        # Filtros (uno por sensor frontal)
        self.ma_fl = MovingAverageFilter(MA_WINDOW)
        self.ma_fr = MovingAverageFilter(MA_WINDOW)
        self.kf    = KalmanFilter1D(Q_KALMAN, R_KALMAN)

        # Estado de navegación para escape de esquina
        self._turn_steps    = 0        # pasos consecutivos girando
        self._last_turn_dir = "RIGHT"  # último lado de giro
        self._back_steps    = 0        # pasos restantes de retroceso forzado

        # Registro
        self.log  = []
        self.step = 0

        print(f"[INIT] Ts={TS:.3f}s  fs={FS:.2f}Hz  umbral={OBSTACLE_THRESHOLD}")
        print(f"[INIT] Kalman Q={Q_KALMAN}  R={R_KALMAN}  MA_WIN={MA_WINDOW}")
        print(f"[INIT] SENSOR_SCALE={SENSOR_SCALE}  MAX_TURN_STEPS={MAX_TURN_STEPS}")

    # ── Encoders → desplazamiento lineal ────────
    def _displacement(self):
        le = self.le.getValue()
        re = self.re.getValue()
        if self._prev_le is None:
            self._prev_le, self._prev_re = le, re
            return 0.0
        ds = WHEEL_RADIUS * ((le - self._prev_le) + (re - self._prev_re)) / 2.0
        self._prev_le, self._prev_re = le, re
        return ds   # [m], positivo = avance

    # ── Lógica de navegación ─────────────────────
    def _decide(self, ps_vals, dist_est):
        front_l = ps_vals[self.IDX_FRONT_L]
        front_r = ps_vals[self.IDX_FRONT_R]
        front   = max(front_l, front_r)

        # --- Retroceso de emergencia activo ---
        if self._back_steps > 0:
            self._back_steps -= 1
            self._turn_steps   = 0
            return -BACK_SPEED, -BACK_SPEED, "BACKWARD"

        # --- Pared trasera detectada → retroceso de emergencia ---
        rear = max(ps_vals[self.IDX_REAR_L], ps_vals[self.IDX_REAR_R])
        if rear > REAR_THRESHOLD and front < OBSTACLE_THRESHOLD:
            # Atrapado entre paredes → retroceder y luego girar
            self._back_steps = 8
            return -BACK_SPEED, -BACK_SPEED, "BACKWARD"

        # --- Hay obstáculo al frente ---
        if dist_est > OBSTACLE_THRESHOLD or front > OBSTACLE_THRESHOLD:
            self._turn_steps += 1

            diag_l = ps_vals[self.IDX_DIAG_L]
            diag_r = ps_vals[self.IDX_DIAG_R]
            diff   = diag_l - diag_r   # >0 → izquierda más bloqueada

            # Escape de esquina: si lleva demasiado tiempo girando sin éxito,
            # retroceder un poco para liberarse antes de seguir girando.
            if self._turn_steps > MAX_TURN_STEPS:
                self._back_steps   = 10
                self._turn_steps   = 0
                return -BACK_SPEED, -BACK_SPEED, "BACKWARD"

            if abs(diff) < SIDE_DEADZONE:
                # Laterales similares → mantener último lado o usar derecha por defecto
                if self._last_turn_dir == "LEFT":
                    lv, rv, action = -TURN_SPEED, TURN_SPEED, "TURN_LEFT"
                else:
                    lv, rv, action = TURN_SPEED, -TURN_SPEED, "TURN_RIGHT(default)"
            elif diff > 0:
                # Izquierda más bloqueada → girar a la derecha
                lv, rv, action = TURN_SPEED, -TURN_SPEED, "TURN_RIGHT"
                self._last_turn_dir = "RIGHT"
            else:
                # Derecha más bloqueada → girar a la izquierda
                lv, rv, action = -TURN_SPEED, TURN_SPEED, "TURN_LEFT"
                self._last_turn_dir = "LEFT"

        else:
            # Camino libre → avanzar
            self._turn_steps = 0
            lv, rv, action = FWD_SPEED, FWD_SPEED, "FORWARD"

        return lv, rv, action

    # ── Bucle principal ──────────────────────────
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            self.step += 1
            t = self.step * TS

            # 1. Lecturas crudas
            ps_vals = [s.getValue() for s in self.ps]
            le_val  = self.le.getValue()
            re_val  = self.re.getValue()

            # 2. Señal frontal cruda
            front_raw = (ps_vals[self.IDX_FRONT_L] + ps_vals[self.IDX_FRONT_R]) / 2.0

            # 3. Filtro media móvil
            front_filt = (
                self.ma_fl.update(ps_vals[self.IDX_FRONT_L]) +
                self.ma_fr.update(ps_vals[self.IDX_FRONT_R])
            ) / 2.0

            # 4. Kalman
            ds = self._displacement()   # [m], positivo = avanza

            # BUG CORREGIDO: avanzar REDUCE la distancia al obstáculo → delta_d negativo.
            # La versión anterior usaba +ds*SCALE, lo que hacía subir el estimado
            # indefinidamente y nunca superaba el umbral.
            delta_d = -ds * SENSOR_SCALE

            if self.kf.d_hat is None:
                self.kf.d_hat = front_raw
                print(f"[INIT] Kalman inicializado con d0={front_raw:.1f}")

            self.kf.predict(delta_d)
            dist_kalman = self.kf.update(front_raw)

            # 5. Decisión
            lv, rv, action = self._decide(ps_vals, front_raw)
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
                    f"turn_steps={self._turn_steps} | {action}"
                )

            # 7. Registrar datos (columnas compatibles con plot_signals.py)
            self.log.append({
                "t":              round(t, 4),
                "ps0":            round(ps_vals[0], 1),
                "ps7":            round(ps_vals[7], 1),
                "ps2_raw":        round(ps_vals[2], 1),
                "ps5_raw":        round(ps_vals[5], 1),
                "front_raw":      round(front_raw, 2),
                "front_filtered": round(front_filt, 2),
                "dist_kalman":    round(dist_kalman, 2),
                "kalman_gain":    round(self.kf.K, 4),
                "kalman_P":       round(self.kf.P, 4),
                "left_enc":       round(le_val, 4),
                "right_enc":      round(re_val, 4),
                "linear_adv_m":   round(ds, 6),
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