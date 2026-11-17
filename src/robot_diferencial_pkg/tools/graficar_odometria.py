#!/usr/bin/env python3
"""
Gráficas para el pitch a partir de los CSV de cronometro_vueltas (requisito 2).

    python3 tools/graficar_odometria.py ~/resultados_entrega2/propia_auto_<fecha>

(se pasa el prefijo común de los dos CSV, sin '_vueltas.csv' / '_trayectoria.csv')
Genera <prefijo>_odometria.png: trayectoria real vs estimada (vía nominal y, si existe, vía
calibrada), error de posición contra el tiempo y tabla de tiempos y errores de cierre por vuelta.
"""
import csv
import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def leer(ruta):
    """CSV -> dict columna: np.array (float)."""
    with open(ruta) as f:
        filas = list(csv.DictReader(f))
    return {k: np.array([float(fila[k]) for fila in filas]) for k in filas[0]}


def main():
    pre = sys.argv[1]
    t = leer(pre + '_trayectoria.csv')
    v = leer(pre + '_vueltas.csv')
    ok = ~np.isnan(t['x_calc'])
    t = {k: a[ok] for k, a in t.items()}
    hay_cal = 'x_cal' in t and not np.isnan(t['x_cal']).all()
    t0 = t['t_sim'][0]

    fig, ax = plt.subplots(1, 3, figsize=(19, 5.8), gridspec_kw={'width_ratios': [1.3, 1, 1.1]})
    ax[0].plot(t['x_real'], t['y_real'], 'g-', lw=1.8, label='real (Gazebo)')
    ax[0].plot(t['x_calc'], t['y_calc'], '-', color='orangered', lw=1.0,
               label='odometría, b = 0.134 m')
    if hay_cal:
        ax[0].plot(t['x_cal'], t['y_cal'], '-', color='royalblue', lw=1.0,
                   label='odometría calibrada, b_ef = 0.153 m')
    ax[0].plot([0, 0], [-0.4, 0.4], 'k-', lw=3, label='salida/meta')
    ax[0].set_aspect('equal')
    ax[0].legend(fontsize=8, loc='best')
    ax[0].grid(True, lw=0.3)
    ax[0].set_title('Trayectoria: real vs estimada (3 vueltas)')
    ax[0].set_xlabel('x [m]')
    ax[0].set_ylabel('y [m]')

    tt = t['t_sim'] - t0
    ax[1].plot(tt, np.hypot(t['x_real'] - t['x_calc'], t['y_real'] - t['y_calc']),
               '-', color='orangered', label='b = 0.134 m')
    if hay_cal:
        ax[1].plot(tt, np.hypot(t['x_real'] - t['x_cal'], t['y_real'] - t['y_cal']),
                   '-', color='royalblue', label='b_ef = 0.153 m')
    for ts in v['t_sim'][1:]:
        ax[1].axvline(ts - t0, color='k', ls='--', lw=0.7)
    ax[1].set_title('Error de posición de la odometría (líneas: cruces de meta)')
    ax[1].set_xlabel('tiempo de simulación [s]')
    ax[1].set_ylabel('error [m]')
    ax[1].legend(fontsize=8)
    ax[1].grid(True, lw=0.3)

    ax[2].axis('off')
    filas = []
    for i in range(1, len(v['cruce'])):
        fila = [int(v['cruce'][i]), f"{v['t_vuelta'][i]:.2f}", f"{v['error_pos'][i]:.3f}",
                f"{math.degrees(v['error_psi'][i]):.1f}"]
        if hay_cal:
            fila += [f"{v['error_pos_cal'][i]:.3f}", f"{math.degrees(v['error_psi_cal'][i]):.1f}"]
        filas.append(fila)
    cols = ['vuelta', 'tiempo\n[s]', 'e_pos\n[m]', 'e_psi\n[°]']
    if hay_cal:
        cols += ['e_pos cal.\n[m]', 'e_psi cal.\n[°]']
    if filas:
        filas.append(['total', f"{v['t_sim'][-1] - v['t_sim'][0]:.2f}"] + [''] * (len(cols) - 2))
        tabla = ax[2].table(cellText=filas, loc='center', colLabels=cols)
        tabla.auto_set_font_size(False)
        tabla.set_fontsize(9)
        tabla.scale(1, 2.0)
        for (fila, _), celda in tabla.get_celld().items():
            if fila == 0:
                celda.set_height(celda.get_height() * 1.6)
    ax[2].set_title('Tiempos (/clock) y error de cierre por vuelta')
    fig.savefig(pre + '_odometria.png', dpi=110, bbox_inches='tight')
    print('guardado', pre + '_odometria.png')


if __name__ == '__main__':
    main()
