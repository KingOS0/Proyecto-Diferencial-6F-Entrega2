#!/usr/bin/env python3
"""
Convierte un STL de pista (muros extruidos en z) en segmentos 2D de pared.

Uso:  python3 tools/stl_a_segmentos.py <pista.stl> salida.npy

Idea: las caras superiores (normal +z) forman la "huella" de los muros. Las aristas
que pertenecen a un solo triángulo de esa huella son el contorno del muro: esos
son los segmentos contra los que choca un rayo del LiDAR en 2D.
Solo usa numpy (sirve para el simulador 2D sin ROS: tools/sim2d.py).
"""
from collections import Counter
import struct
import sys

import numpy as np


def leer_stl_binario(ruta):
    d = open(ruta, 'rb').read()
    n = struct.unpack('<I', d[80:84])[0]
    if 84 + 50 * n != len(d):
        raise ValueError('El STL no es binario (o está corrupto): expórtelo como STL binario.')
    tipo = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    t = np.frombuffer(d[84:], dtype=tipo)
    return t['n'].astype(float), t['v'].astype(float)


def segmentos_de_pared(ruta, dec=4):
    normales, tri = leer_stl_binario(ruta)
    zmax = tri[:, :, 2].max()
    arriba = (np.abs(tri[:, :, 2] - zmax) < 1e-4).all(axis=1)   # triángulos de la tapa superior
    cuenta = Counter()
    for t in tri[arriba]:
        p = [tuple(np.round(v[:2], dec)) for v in t]
        for i in range(3):
            a, b = p[i], p[(i + 1) % 3]
            if a != b:
                cuenta[(a, b) if a < b else (b, a)] += 1
    seg = np.array([[a[0], a[1], b[0], b[1]] for (a, b), c in cuenta.items() if c == 1])
    return seg


if __name__ == '__main__':
    seg = segmentos_de_pared(sys.argv[1])
    print('segmentos:', len(seg), ' x:[%.2f, %.2f]  y:[%.2f, %.2f]' % (
        seg[:, [0, 2]].min(), seg[:, [0, 2]].max(), seg[:, [1, 3]].min(), seg[:, [1, 3]].max()))
    if len(sys.argv) > 2:
        np.save(sys.argv[2], seg)
