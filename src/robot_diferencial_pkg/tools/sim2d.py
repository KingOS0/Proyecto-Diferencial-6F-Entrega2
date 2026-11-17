#!/usr/bin/env python3
"""
Simulador 2D SIN ROS para ajustar el seguidor de pared y el AEB antes de Gazebo.

    python3 tools/sim2d.py --pista models/pista_competencia/meshes/pista_competencia.stl \
        --vueltas 3 --png out.png
    python3 tools/sim2d.py --pista ... --kp 8 --kd 1 --l 0.35 --v_max 1.0 --obstaculo 4.0 0.0

Qué modela: cinemática del robot con los límites del enunciado (v<=1.0, w<=4.0, a<=2.0),
LiDAR de 450 rayos a 10 Hz con ruido sigma=0.01, y la MISMA lógica de nucleo.py que usan
los nodos. Qué NO modela: dinámica, deslizamiento real del skid-steer (se aproxima con
--k_giro: fracción del giro comandado que realmente se logra). Gazebo manda: esto solo da
un punto de partida para las ganancias.
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from robot_diferencial_pkg.nucleo import AEB, Barrido, cruza_linea, SeguidorPared  # noqa: E402
from tools.stl_a_segmentos import segmentos_de_pared  # noqa: E402

LARGO, ANCHO = 0.1986, 0.153


def lidar(seg, x, y, psi, n=450, rmin=0.03, rmax=12.0, sigma=0.01, rng=None):
    ang = -math.pi + 2 * math.pi * np.arange(n) / n
    d = np.stack([np.cos(ang + psi), np.sin(ang + psi)], 1)            # (n, 2)
    p1, p2 = seg[:, :2], seg[:, 2:]
    e = p2 - p1                                                        # (m, 2)
    w = p1 - np.array([x, y])                                          # (m, 2)
    den = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (w[None, :, 0] * e[None, :, 1] - w[None, :, 1] * e[None, :, 0]) / den
        s = (w[None, :, 0] * d[:, None, 1] - w[None, :, 1] * d[:, None, 0]) / den
    t[~((t > 0) & (s >= 0) & (s <= 1))] = np.inf
    r = t.min(1)
    if rng is not None:
        r = r + rng.normal(0, sigma, n)
    r[(r < rmin) | (r > rmax)] = np.inf
    return Barrido(r, -math.pi, 2 * math.pi / n, rmin, rmax)


def dist_a_muros(seg, x, y):
    p1, p2 = seg[:, :2], seg[:, 2:]
    e = p2 - p1
    t = np.clip(((np.array([x, y]) - p1) * e).sum(1) / np.maximum((e * e).sum(1), 1e-12), 0, 1)
    c = p1 + t[:, None] * e
    return float(np.hypot(c[:, 0] - x, c[:, 1] - y).min())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pista', required=True)
    ap.add_argument('--vueltas', type=int, default=3)
    ap.add_argument('--x0', type=float, default=-0.6)
    ap.add_argument('--y0', type=float, default=0.0)
    ap.add_argument('--yaw0', type=float, default=0.0)
    ap.add_argument('--y_linea', type=float, nargs=2, default=[-0.4, 0.4])
    ap.add_argument('--k_giro', type=float, default=0.8,
                    help='fracción del giro comandado que se logra')
    ap.add_argument('--obstaculo', type=float, nargs=2, default=None,
                    help='x y de una caja de 0.3 m')
    ap.add_argument('--aeb', default='corredor', choices=['corredor', 'ittc', 'off'])
    ap.add_argument('--t_max', type=float, default=400.0)
    ap.add_argument('--semilla', type=int, default=1)
    ap.add_argument('--png', default=None)
    defaults = {'d_des': 0.25, 'l': 0.35, 'kp': 12.0, 'ki': 0.0, 'kd': 1.0, 'v_max': 1.0,
                'v_min': 0.35, 'w_ref': 4.0, 'theta': 50.0, 't_lat': 0.20, 'tau_0': 0.30,
                'margen': 0.03, 'd_frente_lento': 0.9, 'd_frente_min': 0.30, 'gamma': 20.0}
    for k, v in defaults.items():
        ap.add_argument('--' + k, type=float, default=v)
    ap.add_argument('--lado', default='izquierda')
    a = ap.parse_args()

    seg = segmentos_de_pared(a.pista)
    if a.obstaculo:
        ox, oy = a.obstaculo
        h = 0.15
        c = [(ox - h, oy - h), (ox + h, oy - h), (ox + h, oy + h), (ox - h, oy + h)]
        seg = np.vstack([seg] + [[*c[i], *c[(i + 1) % 4]] for i in range(4)])
    rng = np.random.default_rng(a.semilla)
    wf = SeguidorPared(lado=a.lado, d_des=a.d_des, theta=math.radians(a.theta), l=a.l, kp=a.kp,
                       ki=a.ki, kd=a.kd, v_max=a.v_max, v_min=a.v_min, w_ref=a.w_ref,
                       d_frente_lento=a.d_frente_lento, d_frente_min=a.d_frente_min,
                       gamma=math.radians(a.gamma))
    aeb = AEB(LARGO, ANCHO, t_lat=a.t_lat, tau_0=a.tau_0, margen=a.margen,
              modo='corredor' if a.aeb == 'off' else a.aeb)

    x, y, psi, v, w = a.x0, a.y0, a.yaw0, 0.0, 0.0
    v_cmd = w_cmd = 0.0
    dt, t, k = 0.01, 0.0, 0
    cruces, tray, frenadas, t_quieto = [], [], 0, 0.0
    min_muro, e_log, activo_prev = 9.9, [], False
    while t < a.t_max and len(cruces) < a.vueltas + 1:
        if k % 10 == 0:                                   # LiDAR a 10 Hz
            b = lidar(seg, x, y, psi, rng=rng)
            v_auto, w_auto = wf.paso(b, 0.1, v, activo_prev if a.aeb != 'off' else None)
            if a.aeb == 'off':
                v_cmd, w_cmd, activo = v_auto, w_auto, False
            else:
                v_cmd, w_cmd, activo, _ = aeb.decidir(b, v_auto, w_auto, v)
            if activo and not activo_prev:
                frenadas += 1
                print('  AEB t=%.1f x=%.2f y=%.2f v=%.2f' % (t, x, y, v))
            activo_prev = activo
            e_log.append(wf.info['e'])
        # límites del enunciado (plugin DiffDrive): aceleración lineal 2.0 m/s^2
        v += max(-2.0 * dt, min(2.0 * dt, max(-1.0, min(1.0, v_cmd)) - v))
        w_obj = a.k_giro * max(-4.0, min(4.0, w_cmd))
        w += max(-25 * dt, min(25 * dt, w_obj - w))
        xp, yp = x, y
        x += v * math.cos(psi) * dt
        y += v * math.sin(psi) * dt
        psi += w * dt
        t += dt
        k += 1
        ok, f = cruza_linea((xp, yp), (x, y), 0.0, *a.y_linea)
        if ok:
            cruces.append(t - dt + f * dt)
        d = dist_a_muros(seg, x, y)
        min_muro = min(min_muro, d)
        tray.append((x, y, v, w, d))
        t_quieto = t_quieto + dt if abs(v) < 0.02 and t > 1.0 else 0.0
        if d < ANCHO / 2:
            print('CHOQUE en t=%.2f  x=%.2f y=%.2f' % (t, x, y))
            break
        if t_quieto > 10.0:
            info = {k_: round(v_, 2) for k_, v_ in wf.info.items()}
            print('DETENIDO >10 s en t=%.2f  x=%.2f y=%.2f psi=%.2f (AEB activo=%s) '
                  'cmd=(%.2f, %.2f) info=%s' % (t, x, y, psi, activo_prev, v_cmd, w_cmd, info))
            break
    tray = np.array(tray)
    vueltas = np.diff(cruces)
    print('vueltas:', np.round(vueltas, 2),
          ' total=%.2f s' % vueltas.sum() if len(vueltas) else '')
    print('holgura mínima centro-muro = %.3f m (choque lateral < %.3f, diagonal %.3f)' % (
        min_muro, ANCHO / 2, math.hypot(LARGO, ANCHO) / 2))
    print('v media = %.2f m/s   activaciones AEB = %d   RMS e = %.3f m' % (
        tray[:, 2].mean(), frenadas, float(np.sqrt(np.mean(np.square(e_log[20:]))))))
    if a.png:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(14, 6))
        for s in seg:
            ax.plot(s[[0, 2]], s[[1, 3]], 'k-', lw=0.6)
        sc = ax.scatter(tray[::5, 0], tray[::5, 1], c=tray[::5, 2], s=2, cmap='viridis')
        plt.colorbar(sc, label='v [m/s]')
        ax.set_aspect('equal')
        fig.savefig(a.png, dpi=70, bbox_inches='tight')


if __name__ == '__main__':
    main()
