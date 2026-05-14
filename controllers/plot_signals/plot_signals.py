"""
plot_signals.py
Genera gráficos de análisis de señales a partir del CSV producido
por el controlador de Webots (sensor_log.csv).

Uso:
    python plot_signals.py [ruta_csv]

Si no se indica ruta, busca sensor_log.csv en el mismo directorio.
"""

import sys
import os
import csv
import math

# ─────────────────────────────────────────────
#  Intentar importar matplotlib; si no está
#  disponible, informar al usuario.
# ─────────────────────────────────────────────
try:
    import matplotlib
    matplotlib.use("Agg")          # backend sin pantalla (sirve en Webots)
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


def load_csv(path: str):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) if k != "action" else v
                         for k, v in row.items()})
    return rows


def extract(rows, key):
    return [r[key] for r in rows]


def plot_all(rows, out_dir: str):
    t               = extract(rows, "t")
    front_raw       = extract(rows, "front_raw")
    front_filtered  = extract(rows, "front_filtered")
    dist_kalman     = extract(rows, "dist_kalman")
    kalman_gain     = extract(rows, "kalman_gain")
    ps2_raw         = extract(rows, "ps2_raw")
    ps5_raw         = extract(rows, "ps5_raw")
    left_enc        = extract(rows, "left_enc")
    right_enc       = extract(rows, "right_enc")
    linear_adv      = extract(rows, "linear_adv_m")

    # ── Paleta de colores ──────────────────────
    C_RAW     = "#e05c5c"
    C_FILT    = "#f5a623"
    C_KALMAN  = "#4a90d9"
    C_ENC     = "#7ed321"
    C_GAIN    = "#9b59b6"

    fig = plt.figure(figsize=(14, 18))
    fig.patch.set_facecolor("#1a1a2e")
    gs  = gridspec.GridSpec(4, 2, figure=fig,
                            hspace=0.55, wspace=0.35,
                            left=0.08, right=0.96,
                            top=0.93, bottom=0.05)

    style = dict(facecolor="#16213e", edgecolor="#0f3460")

    def make_ax(row, col, title):
        ax = fig.add_subplot(gs[row, col])
        ax.set_facecolor("#16213e")
        for spine in ax.spines.values():
            spine.set_edgecolor("#0f3460")
        ax.tick_params(colors="#aaaacc", labelsize=8)
        ax.set_title(title, color="#e0e0ff", fontsize=9, fontweight="bold")
        ax.set_xlabel("Tiempo [s]", color="#aaaacc", fontsize=8)
        ax.grid(True, color="#0f3460", linewidth=0.6, linestyle="--")
        return ax

    # ── Gráfico 1: Señal frontal cruda vs filtrada ──
    ax1 = make_ax(0, 0, "Sensor frontal: cruda vs media móvil")
    ax1.plot(t, front_raw,      color=C_RAW,  lw=1.2, label="Cruda", alpha=0.8)
    ax1.plot(t, front_filtered, color=C_FILT, lw=1.8, label="Filtrada (MA)")
    ax1.set_ylabel("Valor sensor [u.a.]", color="#aaaacc", fontsize=8)
    ax1.legend(fontsize=7, facecolor="#0f3460", labelcolor="white", framealpha=0.8)

    # ── Gráfico 2: Kalman vs señal cruda ────────────
    ax2 = make_ax(0, 1, "Estimación Kalman vs señal cruda")
    ax2.plot(t, front_raw,   color=C_RAW,    lw=1.0, label="Cruda", alpha=0.6)
    ax2.plot(t, dist_kalman, color=C_KALMAN, lw=2.0, label="Kalman")
    ax2.set_ylabel("Distancia frontal [u.a.]", color="#aaaacc", fontsize=8)
    ax2.legend(fontsize=7, facecolor="#0f3460", labelcolor="white", framealpha=0.8)

    # ── Gráfico 3: Comparación triple ───────────────
    ax3 = make_ax(1, 0, "Comparación: cruda / filtrada / Kalman")
    ax3.plot(t, front_raw,      color=C_RAW,    lw=1.0, alpha=0.6, label="Cruda")
    ax3.plot(t, front_filtered, color=C_FILT,   lw=1.4, alpha=0.9, label="Media Móvil")
    ax3.plot(t, dist_kalman,    color=C_KALMAN, lw=2.0,             label="Kalman")
    ax3.set_ylabel("Distancia frontal [u.a.]", color="#aaaacc", fontsize=8)
    ax3.legend(fontsize=7, facecolor="#0f3460", labelcolor="white", framealpha=0.8)

    # ── Gráfico 4: Ganancia de Kalman ────────────────
    ax4 = make_ax(1, 1, "Ganancia de Kalman (Kk)")
    ax4.plot(t, kalman_gain, color=C_GAIN, lw=1.5)
    ax4.set_ylabel("Kk", color="#aaaacc", fontsize=8)
    ax4.set_ylim(0, 1)

    # ── Gráfico 5: Sensores laterales ───────────────
    ax5 = make_ax(2, 0, "Sensores laterales (ps2 izq / ps5 der)")
    ax5.plot(t, ps2_raw, color="#2ecc71", lw=1.4, label="ps2 (izq)")
    ax5.plot(t, ps5_raw, color="#e67e22", lw=1.4, label="ps5 (der)")
    ax5.set_ylabel("Valor sensor [u.a.]", color="#aaaacc", fontsize=8)
    ax5.legend(fontsize=7, facecolor="#0f3460", labelcolor="white", framealpha=0.8)

    # ── Gráfico 6: Encoders ─────────────────────────
    ax6 = make_ax(2, 1, "Encoders de rueda (ángulo acumulado [rad])")
    ax6.plot(t, left_enc,  color="#1abc9c", lw=1.4, label="Encoder izq")
    ax6.plot(t, right_enc, color="#e74c3c", lw=1.4, label="Encoder der")
    ax6.set_ylabel("Ángulo [rad]", color="#aaaacc", fontsize=8)
    ax6.legend(fontsize=7, facecolor="#0f3460", labelcolor="white", framealpha=0.8)

    # ── Gráfico 7: Avance estimado ──────────────────
    ax7 = make_ax(3, 0, "Avance lineal estimado por encoders")
    ax7.plot(t, linear_adv, color=C_ENC, lw=1.4)
    ax7.set_ylabel("Δs [m]", color="#aaaacc", fontsize=8)
    ax7.axhline(0, color="#555577", lw=0.8, linestyle="--")

    # ── Gráfico 8: Avance acumulado ──────────────────
    cumulative = [sum(linear_adv[:i+1]) for i in range(len(linear_adv))]
    ax8 = make_ax(3, 1, "Desplazamiento acumulado")
    ax8.plot(t, cumulative, color="#f39c12", lw=1.4)
    ax8.set_ylabel("Σs [m]", color="#aaaacc", fontsize=8)

    # ── Título general ───────────────────────────────
    fig.suptitle(
        "Laboratorio 2 – Análisis de Señales\n"
        "ICI 4150 · Robótica y Sistemas Autónomos 2026-01",
        color="#e0e0ff", fontsize=13, fontweight="bold"
    )

    out_path = os.path.join(out_dir, "signal_analysis.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print(f"[OK] Gráfico guardado en {out_path}")
    plt.close()


def main():
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        csv_path = os.path.join(os.path.dirname(__file__), "sensor_log.csv")

    if not os.path.exists(csv_path):
        print(f"[ERROR] No se encontró {csv_path}")
        print("  Ejecuta primero la simulación en Webots para generar el CSV.")
        sys.exit(1)

    if not HAS_MATPLOTLIB:
        print("[ERROR] matplotlib no está instalado.")
        print("  Instalar con: pip install matplotlib")
        sys.exit(1)

    rows = load_csv(csv_path)
    print(f"[INFO] {len(rows)} muestras cargadas.")
    out_dir = os.path.dirname(os.path.abspath(csv_path))
    plot_all(rows, out_dir)


if __name__ == "__main__":
    main()
