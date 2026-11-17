#!/usr/bin/env python3
"""
Trazado de la PISTA PROPIA (requisito 5): contornos de los muros (DXF) + plano PNG.

    python3 tools/generar_pista.py            # paso 1: trazado
    freecadcmd tools/pista_cad_freecad.py     # paso 2: modelo CAD (.FCStd) y STL en metros

La pista se define por su línea central (rectas y arcos) y un perfil de ancho. De ahí
salen los contornos de los dos muros (interior y exterior):
  - DXF  -> models/pista_propia/meshes/pista_propia.dxf: lo lee el modelo de FreeCAD, que
            extruye los muros 300 mm y exporta el STL (en metros) que carga Gazebo.
  - PNG  -> plano con secciones y cotas para el pitch.
Convenciones iguales a la pista de competencia: sentido antihorario, línea de salida/meta
en x = 0 (avance en +x), robot en x = -0.60, y = 0, yaw = 0.
"""
import math
import os

import numpy as np

ANCHO = 0.80          # ancho nominal del carril [m]
ANCHO_MIN = 0.40      # estrechamiento (mínimo permitido por el enunciado)
MURO = 0.10           # espesor de muro [m]
PASO = 0.02           # muestreo de la línea central [m]

# Chicana: tres arcos (derecha, izquierda, derecha) de radio R_CH y ángulo A_CH/2A_CH/A_CH
R_CH, A_CH = 1.5, 35.0
R_FINAL = 1.2         # última curva (media) antes de la recta principal
# -35°, +70°, -35° es un 'bulto': sale 2R(1-cos35°) = 0.54 m y vuelve a la misma recta
DESPLAZ_CH = 0.0
AVANCE_CH = 4 * R_CH * math.sin(math.radians(A_CH))            # avance de la chicana
X_FIN_S7 = -3.0       # x de la recta de la chicana antes de desplazarse
Y_INI_S7 = 6.3        # y donde arranca esa recta (bajando)
X0 = X_FIN_S7 - DESPLAZ_CH + R_FINAL          # inicio de la recta principal (cierra el circuito)
RECTA_ANTES_CH = 0.5
RECTA_DESPUES_CH = Y_INI_S7 - RECTA_ANTES_CH - AVANCE_CH - R_FINAL

# (nombre, tipo, valor...)  recta: longitud [m]   arco: radio [m], ángulo [deg] (+ izquierda)
SECCIONES = [
    ('S1 recta principal (salida/meta)', 'recta', 4.0 - X0),
    ('S2 horquilla 180° R=0.9', 'arco', 0.9, 180),
    ('S3 recta corta', 'recta', 1.5),
    ('S4 curva cerrada a la derecha 90° R=1.0', 'arco', 1.0, -90),
    ('S5 recta con estrechamiento a 0.40 m', 'recta', 3.0),
    ('S6 curva amplia 90° R=2.0', 'arco', 2.0, 90),
    ('S7 recta', 'recta', 1.0),
    ('S8 curva media 90° R=1.5', 'arco', 1.5, 90),
    ('S9 recta', 'recta', RECTA_ANTES_CH),
    ('S10 chicana (R=1.5: -35°, +70°, -35°)', 'arco', R_CH, -A_CH),
    ('S10b', 'arco', R_CH, 2 * A_CH),
    ('S10c', 'arco', R_CH, -A_CH),
    ('S11 recta', 'recta', RECTA_DESPUES_CH),
    ('S12 curva media 90° R=1.2', 'arco', R_FINAL, 90),
]
# estrechamiento dentro de S5: (inicio, fin) de cada tramo medido desde el inicio de S5 [m]
ESTRECHO = {'seccion': 4, 'rampa_in': (0.5, 1.0), 'angosto': (1.0, 2.5), 'rampa_out': (2.5, 3.0)}


def linea_central():
    """Devuelve puntos (N,2), rumbo (N,), ancho (N,) y los límites de cada sección."""
    x, y, psi = X0, 0.0, 0.0
    P, PSI, W, limites = [], [], [], []
    for k, sec in enumerate(SECCIONES):
        nombre, tipo = sec[0], sec[1]
        if tipo == 'recta':
            L = sec[2]
            n = max(int(round(L / PASO)), 1)
            for i in range(n):
                s = L * i / n
                w = ANCHO
                if k == ESTRECHO['seccion']:
                    (a, b), (c, d) = ESTRECHO['rampa_in'], ESTRECHO['rampa_out']
                    if a <= s < b:
                        w = ANCHO + (ANCHO_MIN - ANCHO) * (s - a) / (b - a)
                    elif b <= s < c:
                        w = ANCHO_MIN
                    elif c <= s < d:
                        w = ANCHO_MIN + (ANCHO - ANCHO_MIN) * (s - c) / (d - c)
                P.append((x + s * math.cos(psi), y + s * math.sin(psi)))
                PSI.append(psi)
                W.append(w)
            x, y = x + L * math.cos(psi), y + L * math.sin(psi)
        else:
            R, ang = sec[2], math.radians(sec[3])
            sg = 1.0 if ang > 0 else -1.0
            cx, cy = x - sg * R * math.sin(psi), y + sg * R * math.cos(psi)
            n = max(int(round(R * abs(ang) / PASO)), 2)
            for i in range(n):
                a = ang * i / n
                P.append((cx + sg * R * math.sin(psi + a), cy - sg * R * math.cos(psi + a)))
                PSI.append(psi + a)
                W.append(ANCHO)
            x, y = cx + sg * R * math.sin(psi + ang), cy - sg * R * math.cos(psi + ang)
            psi += ang
        limites.append((nombre, len(P)))
    cierre = math.hypot(x - X0, y) + abs(math.remainder(psi, 2 * math.pi))
    assert cierre < 1e-6, f'la pista no cierra: error {cierre}'
    return np.array(P), np.array(PSI), np.array(W), limites


def muros(P, PSI, W):
    """Cuatro polilíneas cerradas: borde interno/externo del muro izquierdo y del derecho."""
    n = np.stack([-np.sin(PSI), np.cos(PSI)], 1)          # normal a la izquierda
    izq_in = P + n * (W / 2)[:, None]
    izq_out = P + n * (W / 2 + MURO)[:, None]
    der_in = P - n * (W / 2)[:, None]
    der_out = P - n * (W / 2 + MURO)[:, None]
    return (izq_in, izq_out), (der_in, der_out)


def escribir_dxf(ruta, polilineas):
    with open(ruta, 'w') as f:
        f.write('0\nSECTION\n2\nENTITIES\n')
        for p in polilineas:
            f.write('0\nPOLYLINE\n8\nMUROS\n66\n1\n70\n1\n')
            for x, y in p:
                f.write(f'0\nVERTEX\n8\nMUROS\n10\n{x:.5f}\n20\n{y:.5f}\n30\n0.0\n')
            f.write('0\nSEQEND\n')
        f.write('0\nENDSEC\n0\nEOF\n')


def main():
    raiz = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
    P, PSI, W, limites = linea_central()
    (ii, io), (di, do) = muros(P, PSI, W)
    d_mesh = os.path.join(raiz, 'models', 'pista_propia', 'meshes')
    os.makedirs(d_mesh, exist_ok=True)
    escribir_dxf(os.path.join(d_mesh, 'pista_propia.dxf'), [ii[::3], io[::3], di[::3], do[::3]])
    longitud = float(np.hypot(*np.diff(np.vstack([P, P[:1]]), axis=0).T).sum())
    todo = np.vstack([io, do])
    print(f'longitud línea central: {longitud:.2f} m (3 vueltas {3*longitud:.1f} m)')
    print('siguiente paso (CAD): freecadcmd tools/pista_cad_freecad.py  -> .FCStd y STL en metros')
    print('caja: x[%.2f, %.2f]  y[%.2f, %.2f]   ancho mínimo %.2f m' % (
        todo[:, 0].min(), todo[:, 0].max(), todo[:, 1].min(), todo[:, 1].max(), W.min()))
    print(f'inicio de la recta principal x0 = {X0:.3f}   '
          f'recta tras la chicana = {RECTA_DESPUES_CH:.3f} m')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(11, 10))
        for a, b in ((ii, io), (di, do)):
            pol = np.vstack([a, a[:1], b[:1][::-1], b[::-1]])
            ax.fill(pol[:, 0], pol[:, 1], color='0.25', lw=0)
        ax.plot(P[:, 0], P[:, 1], '--', color='0.6', lw=0.8)
        ax.plot([0, 0], [-ANCHO / 2, ANCHO / 2], 'r-', lw=3)
        ax.annotate('salida/meta x=0', (0, -0.75), ha='center', color='r')
        ax.plot(-0.6, 0, 'bs')
        ax.annotate('spawn', (-0.6, 0.12), ha='center', color='b', fontsize=8)
        ini = 0
        for nombre, fin in limites:
            if not nombre[-1] in 'bc':
                m = (ini + fin) // 2
                nrm = np.array([-math.sin(PSI[m]), math.cos(PSI[m])])
                ax.annotate(nombre, P[m] - nrm * 0.9, ha='center', va='center', fontsize=7,
                            bbox={'fc': 'w', 'ec': '0.7', 'lw': 0.5})
            ini = fin
        ax.set_aspect('equal')
        ax.grid(True, lw=0.3)
        ax.set_title(f'Pista propia — {longitud:.1f} m por vuelta, sentido antihorario, '
                     f'carril {ANCHO} m')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        fig.savefig(os.path.join(d_mesh, '..', 'plano_pista_propia.png'), dpi=110,
                    bbox_inches='tight')
    except ImportError:
        print('(sin matplotlib: no se generó el plano PNG)')


if __name__ == '__main__':
    main()
