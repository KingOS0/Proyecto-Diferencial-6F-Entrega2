"""
Pruebas de la lógica pura (no necesitan ROS ni Gazebo).

Desde la raíz del paquete:  python3 -m pytest test/test_nucleo.py -q
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from robot_diferencial_pkg.nucleo import (AEB, Barrido, cruza_linea,  # noqa: E402
                                          geometria_pared, OdometriaDiferencial,
                                          SeguidorPared)

N = 450
ANG = -math.pi + 2 * math.pi * np.arange(N) / N


def barrido(fn):
    return Barrido([fn(a) for a in ANG], -math.pi, 2 * math.pi / N, 0.03, 12.0)


def pared_al_frente(d):
    """Pared infinita perpendicular al eje x, a distancia d del LiDAR."""
    return barrido(lambda a: d / math.cos(a) if math.cos(a) > 1e-3 else math.inf)


def pasillo(izq, der):
    """Dos paredes paralelas al robot: a 'izq' m a la izquierda y 'der' m a la derecha."""
    def f(a):
        s = math.sin(a)
        if s > 1e-3:
            return izq / s
        if s < -1e-3:
            return der / -s
        return math.inf
    return barrido(f)


def test_odometria_recta_y_giro():
    o = OdometriaDiferencial(r=0.0245, b=0.134)
    for _ in range(1000):
        o.paso(10.0, 10.0, 0.001)                       # 1 s a u = r*w = 0.245 m/s
    assert abs(o.eta[0] - 0.245) < 1e-6 and abs(o.eta[1]) < 1e-9
    o = OdometriaDiferencial(r=0.0245, b=0.134)
    for _ in range(1000):
        o.paso(1.0, -1.0, 0.001)                        # gira en el sitio a 2r/b rad/s
    assert abs(o.eta[2] - 2 * 0.0245 / 0.134) < 1e-6 and abs(o.eta[0]) < 1e-9


def test_geometria_dos_rayos():
    theta = math.radians(45)
    # robot paralelo a la pared a 0.30 m: b = 0.30, a = 0.30 / cos(45°)
    alpha, d = geometria_pared(0.30 / math.cos(theta), 0.30, theta)
    assert abs(alpha) < 1e-9 and abs(d - 0.30) < 1e-9
    # robot girado 10° alejándose de la pared izquierda (pared a 0.30 m perpendicular)
    g = math.radians(10)
    b_ = 0.30 / math.cos(g)
    a_ = 0.30 / math.cos(theta + g)                       # el rayo a queda más rasante
    alpha, d = geometria_pared(a_, b_, theta)
    assert abs(alpha - g) < 1e-9 and abs(d - 0.30) < 1e-9


def test_ttc_ejemplo_de_clase():
    """Clase lidar_aeb: pared a 1.0 m, v = 0.5 m/s -> iTTC = 2.0 s (sin descuentos)."""
    aeb = AEB(largo=0.0, ancho=0.0, d_seg=0.0, modo='ittc')
    assert abs(aeb.ttc(pared_al_frente(1.0), 0.5) - 2.0) < 0.01
    assert aeb.ttc(pared_al_frente(1.0), 0.0) == math.inf
    assert aeb.ttc(pared_al_frente(1.0), -0.5) == math.inf      # retrocede: nada atrás


def test_aeb_frena_y_deja_retroceder():
    aeb = AEB(largo=0.1986, ancho=0.153)
    b = pared_al_frente(0.40)
    v, w, activo, _ = aeb.decidir(b, 1.0, 0.0, 1.0)
    assert activo and v == 0.0
    v, w, activo, _ = aeb.decidir(b, -0.3, 0.0, 0.0)
    assert not activo and v == -0.3
    v, w, activo, _ = aeb.decidir(pared_al_frente(5.0), 1.0, 0.2, 1.0)
    assert not activo and v == 1.0 and w == 0.2


def test_corredor_no_frena_en_estrechamiento_e_ittc_si():
    b = pasillo(0.20, 0.20)                    # estrechamiento de 0.40 m, robot centrado
    assert not AEB(0.1986, 0.153, modo='corredor').decidir(b, 1.0, 0.0, 1.0)[2]
    assert AEB(0.1986, 0.153, modo='ittc').decidir(b, 1.0, 0.0, 1.0)[2]


def test_seguidor_signos():
    wf = SeguidorPared(lado='izquierda', d_des=0.20)
    v, w = wf.paso(pasillo(0.20, 0.60), 0.1)
    assert abs(w) < 0.05 and abs(v - wf.v_max) < 0.02  # en la distancia deseada: recto y rápido
    wf = SeguidorPared(lado='izquierda', d_des=0.20)
    assert wf.paso(pasillo(0.35, 0.45), 0.1)[1] > 0    # lejos de la pared izq. -> gira a la izq.
    wf = SeguidorPared(lado='izquierda', d_des=0.20)
    assert wf.paso(pasillo(0.12, 0.68), 0.1)[1] < 0        # muy cerca -> gira a la derecha
    wf = SeguidorPared(lado='derecha', d_des=0.20)
    assert wf.paso(pasillo(0.45, 0.35), 0.1)[1] < 0    # pared der. lejos -> gira a la derecha


def test_cruce_de_linea():
    assert cruza_linea((-0.01, 0.1), (0.01, 0.1))[0]
    assert not cruza_linea((0.01, 0.1), (-0.01, 0.1))[0]    # en sentido contrario no cuenta
    assert not cruza_linea((-0.01, 3.0), (0.01, 3.0))[0]    # fuera de la línea no cuenta


def test_retenido_por_el_aeb_no_gira_desde_el_primer_instante():
    """
    Con /aeb/activo = True el seguidor pide (v_min, 0) de inmediato.

    Si mandara la omega del PID, el AEB la dejaría pasar (robot quieto) y el robot giraría
    frente al obstáculo.
    """
    wf = SeguidorPared(lado='izquierda')
    wf.paso(pasillo(0.20, 0.60), 0.1, v_med=0.5, frenado=False)
    for _ in range(100):                                   # 10 s retenido: nunca gira
        v, w = wf.paso(pasillo(0.12, 0.68), 0.1, v_med=0.0, frenado=True)
        assert w == 0.0 and v == wf.v_min
    v, w = wf.paso(pasillo(0.12, 0.68), 0.1, v_med=0.0, frenado=False)
    assert w < 0                                           # lo sueltan: vuelve a controlar


def test_retenido_deducido_sin_ros():
    """Sin /aeb/activo (sim2d): retenido = quieto aunque venía pidiendo avanzar."""
    wf = SeguidorPared(lado='izquierda')
    wf.paso(pasillo(0.20, 0.60), 0.1, v_med=0.5)
    assert wf.paso(pasillo(0.12, 0.68), 0.1, v_med=0.0) == (wf.v_min, 0.0)
