"""
Lógica pura (sin ROS) de la Entrega 2: odometría, AEB (TTC) y seguimiento de pared.

Está separada de los nodos para poder probarla sin Gazebo:
  - test/test_nucleo.py   (pytest)
  - tools/sim2d.py        (simulador 2D que usa el STL de la pista)
Los nodos (nodo_cinematica, aeb_node, wall_follow) solo traducen mensajes ROS a
estas funciones. Ecuaciones: enunciado "Reporte Robot Diferencial: AEB, seguimiento
de pared y competencia" (sec. 3.2) y clases lidar_aeb.pdf / gap_wall.pdf (M. Ortiz, EIA).
"""
import math

import numpy as np


def envolver(a):
    """Lleva un ángulo a (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


# --------------------------------------------------------------------------
# Odometría diferencial / skid-steer
# --------------------------------------------------------------------------
class OdometriaDiferencial:
    """
    Integra la pose eta = [x, y, psi] a partir de las velocidades de rueda.

    Skid-steer de 4 ruedas tratado como diferencial equivalente (enunciado, req. 2):
    w_D y w_I son el PROMEDIO de las dos ruedas de cada lado.
        u     = r (w_D + w_I) / 2          (avance)
        psi_p = r (w_D - w_I) / b          (giro; b = vía, centro a centro)
    """

    def __init__(self, r, b, x0=0.0, y0=0.0, psi0=0.0):
        self.r, self.b = float(r), float(b)
        self.eta = np.array([x0, y0, psi0], dtype=float)
        self.distancia = 0.0

    def matriz(self, psi):
        r, b = self.r, self.b
        return np.array([[r / 2 * math.cos(psi), r / 2 * math.cos(psi)],
                         [r / 2 * math.sin(psi), r / 2 * math.sin(psi)],
                         [r / b, -r / b]])

    def paso(self, w_D, w_I, dt):
        """Euler hacia adelante (igual que en la Entrega 1). Devuelve (u, psi_p)."""
        d_eta = self.matriz(self.eta[2]) @ np.array([w_D, w_I])
        self.eta += d_eta * dt
        self.eta[2] = envolver(self.eta[2])
        u = self.r * (w_D + w_I) / 2
        self.distancia += abs(u) * dt
        return u, d_eta[2]


# --------------------------------------------------------------------------
# Utilidades de LiDAR
# --------------------------------------------------------------------------
class Barrido:
    """Copia mínima de sensor_msgs/LaserScan (para no depender de ROS aquí)."""

    def __init__(self, ranges, angle_min, angle_increment, range_min, range_max):
        self.r = np.asarray(ranges, dtype=float)
        self.angle_min = float(angle_min)
        self.inc = float(angle_increment)
        self.range_min, self.range_max = float(range_min), float(range_max)
        self.ang = self.angle_min + self.inc * np.arange(len(self.r))
        # inf / nan / fuera de rango = "no hay nada", NO es un obstáculo
        self.valido = np.isfinite(self.r) & (self.r > self.range_min) & (self.r < self.range_max)

    def rango_en(self, angulo, vecinos=2, sin_dato=None):
        """Distancia en un ángulo: promedio de los haces vecinos válidos (filtra ruido)."""
        n = len(self.r)
        i = int(round((envolver(angulo) - self.angle_min) / self.inc))
        idx = np.arange(i - vecinos, i + vecinos + 1) % n
        ok = self.valido[idx]
        if not ok.any():
            return self.range_max if sin_dato is None else sin_dato
        return float(self.r[idx][ok].mean())

    def hay_salto(self, ang_ini, ang_fin, umbral=0.20):
        """
        Indica si el rango CAE de golpe entre dos rayos seguidos, de ang_ini a ang_fin.

        True si cae más de 'umbral'. Una pared continua no hace eso; un objeto suelto
        delante de la pared (una caja), sí.
        """
        n = len(self.r)
        i0 = int(round((envolver(ang_ini) - self.angle_min) / self.inc))
        i1 = int(round((envolver(ang_fin) - self.angle_min) / self.inc))
        paso = 1 if i1 >= i0 else -1
        previo = None
        for i in range(i0, i1 + paso, paso):
            if not self.valido[i % n]:
                continue
            if previo is not None and self.r[i % n] < previo - umbral:
                return True
            previo = self.r[i % n]
        return False

    def lateral_min(self, ang_ini, ang_fin, sin_dato=None):
        """Mínima distancia LATERAL (r |sin(ang)|) de los rayos válidos entre dos ángulos."""
        a0, a1 = sorted((envolver(ang_ini), envolver(ang_fin)))
        ok = self.valido & (self.ang >= a0) & (self.ang <= a1)
        if not ok.any():
            return self.range_max if sin_dato is None else sin_dato
        return float((self.r[ok] * np.abs(np.sin(self.ang[ok]))).min())

    def minimo_en(self, centro, semiancho):
        """Distancia mínima válida en el sector [centro - semiancho, centro + semiancho]."""
        d = np.abs(np.arctan2(np.sin(self.ang - centro), np.cos(self.ang - centro)))
        ok = self.valido & (d <= semiancho)
        return float(self.r[ok].min()) if ok.any() else self.range_max


# --------------------------------------------------------------------------
# AEB: frenado autónomo de emergencia por tiempo a colisión
# --------------------------------------------------------------------------
class AEB:
    """
    TTC por rayo y decisión de frenado.

    Por cada rayo i (enunciado, req. 3):
        r_p_i = -v cos(theta_i)            (el giro w no cambia el rango)
        TTC_i = r_i / max(-r_p_i, 0)       (infinito si el rayo no se acerca)
    Frena si min_i TTC_i < tau(v),  con  tau(v) = max(tau_0, t_lat + |v| / (2 a_max))
    (clase lidar_aeb, "distancia de frenado y umbral dinámico").

    Dos detalles para que no frene en falso ni tarde:
      * descuento de carrocería: el LiDAR está en el centro, el robot choca con el
        parachoques. Se resta a cada rango la distancia del LiDAR al borde del robot
        en esa dirección.
      * colchón d_seg: distancia a la que se quiere quedar del obstáculo al detenerse.
      * modo 'corredor': solo cuentan los rayos cuyo punto de impacto cae dentro de
        la franja que barre el robot al avanzar (|y_i| < ancho/2 + margen). En modo
        'ittc' cuentan todos (sirve para mostrar por qué frena en el estrechamiento).
    """

    def __init__(self, largo, ancho, x_lidar=0.0, a_max=2.0, t_lat=0.20, tau_0=0.30,
                 margen=0.03, d_seg=0.05, modo='corredor', v_quieto=0.05):
        self.hx_del = largo / 2 - x_lidar      # LiDAR -> parachoques delantero
        self.hx_tra = largo / 2 + x_lidar      # LiDAR -> parachoques trasero
        self.hy = ancho / 2
        self.a_max, self.t_lat, self.tau_0 = a_max, t_lat, tau_0
        self.margen, self.d_seg, self.modo, self.v_quieto = margen, d_seg, modo, v_quieto

    def tau(self, v):
        return max(self.tau_0, self.t_lat + abs(v) / (2 * self.a_max))

    def distancia_frenado(self, v):
        return abs(v) * self.t_lat + v * v / (2 * self.a_max)

    def ttc(self, b, v):
        """min_i TTC_i [s] para una velocidad lineal v (medida o comandada)."""
        if abs(v) < 1e-3:
            return math.inf
        c, s = np.cos(b.ang), np.sin(b.ang)
        acercamiento = v * c                      # = -r_p_i
        ok = b.valido & (acercamiento > 1e-6)
        if self.modo == 'corredor':
            y_imp = np.where(b.valido, b.r, 0.0) * s          # y del punto de impacto
            ok &= np.abs(y_imp) < (self.hy + self.margen)
        if not ok.any():
            return math.inf
        hx = self.hx_del if v > 0 else self.hx_tra
        # distancia del LiDAR al borde del robot (rectángulo) en la dirección de cada rayo
        casco = np.minimum(hx / np.maximum(np.abs(c), 1e-9), self.hy / np.maximum(np.abs(s), 1e-9))
        # distancia libre real: rango - carrocería - colchón de seguridad d_seg
        libre = np.maximum(np.where(b.valido, b.r, 0.0) - casco - self.d_seg, 0.0)
        return float((libre[ok] / acercamiento[ok]).min())

    def decidir(self, b, v_cmd, w_cmd, v_med):
        """
        Devuelve (v_out, w_out, activo, ttc_minimo).

        - emergencia: con la velocidad MEDIDA ya voy hacia algo.
        - comando inseguro: el comando me llevaría hacia algo (aunque esté quieto).
        - retroceder siempre se puede evaluar con los rayos de atrás (v < 0).
        """
        ttc_med = self.ttc(b, v_med)
        ttc_cmd = self.ttc(b, v_cmd)
        emergencia = ttc_med < self.tau(v_med)
        inseguro = ttc_cmd < self.tau(v_cmd)
        activo = inseguro or (emergencia and v_cmd * v_med > 0)
        if not activo:
            return v_cmd, w_cmd, False, min(ttc_med, ttc_cmd)
        # Frenando: el giro solo pasa cuando el robot ya está casi quieto (para poder
        # salir girando en el sitio); a velocidad, girar puede meter una esquina al muro.
        w_out = w_cmd if abs(v_med) < self.v_quieto else 0.0
        return 0.0, w_out, True, min(ttc_med, ttc_cmd)


# --------------------------------------------------------------------------
# Seguimiento de pared (dos rayos + PID)
# --------------------------------------------------------------------------
def geometria_pared(a, b, theta):
    """
    Ángulo robot-pared (alpha) y distancia perpendicular (D_t) con dos rayos.

    b: rayo perpendicular al robot (90° al lado de la pared)
    a: rayo adelantado un ángulo theta respecto a b
        alpha = arctan((a cos(theta) - b) / (a sin(theta)))   D_t = b cos(alpha)
    alpha > 0: el robot se aleja de la pared; alpha < 0: se acerca.
    """
    alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
    return alpha, b * math.cos(alpha)


class SeguidorPared:
    """
    PID sobre la distancia a una pared; salida omega; v según la curvatura.

        D_{t+1} = D_t + l sin(alpha)        (l: distancia de anticipación)
        D_{t+1} = min(D_{t+1}, c sin(gamma)) (tercer rayo c: ve si la pared se cierra adelante)
        e       = D_des - D_{t+1}           (convención del enunciado)
        D_des efectivo = min(D_des, ancho del carril / 2)   (para caber en el estrechamiento)
        omega   = -signo_lado * (Kp e + Ki int(e) + Kd de/dt)
    Pared izquierda (signo_lado = +1): e < 0 (lejos) -> omega > 0 (gira a la izquierda).
    """

    def __init__(self, lado='izquierda', d_des=0.25, theta=math.radians(50), l=0.35,  # noqa: E741
                 kp=12.0, ki=0.0, kd=1.0, v_max=1.0, v_min=0.35, w_max=4.0, w_ref=4.0,
                 d_frente_lento=0.9, d_frente_min=0.30, pared_max=1.2, tau_d=0.10,
                 gamma=math.radians(20), d_des_min=0.20):
        self.signo = 1.0 if lado == 'izquierda' else -1.0
        self.d_des, self.theta, self.l = d_des, theta, l  # noqa: E741 (l del enunciado)
        self.kp, self.ki, self.kd = kp, ki, kd
        self.v_max, self.v_min, self.w_max, self.w_ref = v_max, v_min, w_max, w_ref
        self.d_frente_lento, self.d_frente_min = d_frente_lento, d_frente_min
        self.pared_max, self.tau_d = pared_max, tau_d
        self.gamma, self.d_des_min = gamma, d_des_min
        self.integral, self.e_prev, self.de_f = 0.0, None, 0.0
        self.retenido, self.v_cmd_prev = False, 0.0
        self.info = {}

    def _retenido(self, v_med, frenado):
        """
        Qué hacer cuando el AEB está reteniendo al robot (lo frenó frente a algo).

        frenado: estado del AEB (/aeb/activo). Si es None se deduce: v_med ~ 0 aunque se
        pidió avanzar (sirve sin ROS, en tools/sim2d.py).
        Retenido: pide (v_min, 0), o sea sigue pidiendo avanzar despacio para que el AEB lo
        suelte apenas se quite el obstáculo, pero SIN girar. Con el robot quieto el AEB deja
        pasar el giro (para poder salir girando en teleop); si aquí se mandara la omega del
        PID, el robot giraría en el sitio frente al obstáculo (medido en Gazebo).
        Devuelve (v, w) o None si no está retenido.
        """
        if frenado is None:
            frenado = v_med is not None and abs(v_med) < 0.03 and self.v_cmd_prev > 0.1
        if not frenado:
            if self.retenido:                       # lo soltaron: el PID arranca limpio
                self.integral, self.e_prev, self.de_f = 0.0, None, 0.0
                self.retenido = False
            return None
        self.retenido = True
        return self.v_min, 0.0

    def paso(self, b, dt, v_med=None, frenado=None):
        """
        Un paso del seguidor; devuelve (v, omega).

        b: Barrido. dt: tiempo entre barridos [s]. v_med: velocidad medida (odometría).
        frenado: True si el AEB está reteniendo al robot (/aeb/activo).
        """
        ret = self._retenido(v_med, frenado)
        if ret is not None:
            self.info = {'retenido': 1.0, 'e': 0.0, 'v': ret[0], 'w': ret[1]}
            return ret
        ang_b = self.signo * math.pi / 2
        ang_a = self.signo * (math.pi / 2 - self.theta)
        # si la pared "desaparece" (hueco, esquina), se limita la lectura (sin volantazos)
        rb = min(b.rango_en(ang_b), self.pared_max)
        ra = min(b.rango_en(ang_a), self.pared_max / math.cos(self.theta))
        alpha, d_t = geometria_pared(ra, rb, self.theta)
        d_t1 = d_t + self.l * math.sin(alpha)
        # Tercer rayo (anticipación): c mira a gamma del frente, del lado de la pared. Su punto
        # de impacto está a c*sin(gamma) de lado. En recta eso vale D_t; si la pared se cierra
        # adelante (curva hacia el lado contrario, rampa del estrechamiento) vale menos, y se
        # usa esa distancia para empezar a girar antes. gamma = 0 lo desactiva.
        # Solo vale si c toca la MISMA pared: si entre a y c el rango cae de golpe, lo que ve c
        # es un objeto suelto (obstáculo). Esquivarlo no es trabajo del seguidor: se sigue
        # derecho y el AEB detiene el robot.
        d_c = d_t1
        if self.gamma > 0 and not b.hay_salto(ang_a, self.signo * self.gamma):
            rc = b.rango_en(self.signo * self.gamma)
            d_c = rc * math.sin(self.gamma)
            d_t1 = min(d_t1, d_c)
        # Consigna efectiva: nunca más lejos de la pared que el centro del carril, para pasar
        # centrado por el estrechamiento. El ancho se mide con la pared del OTRO lado desde el
        # través (90°) hasta theta hacia adelante (distancia lateral r |sin|): así el robot se
        # corre al centro ANTES de que el carril se cierre, no cuando ya lo tiene al lado
        # (medido en Gazebo: con solo el rayo a 90° entraba al estrechamiento a 0.056 m del muro
        # y el AEB frenaba). Piso d_des_min = 0.20 m: la mitad del ancho mínimo del enunciado.
        lado_opuesto = b.lateral_min(-ang_b, -ang_a, sin_dato=self.pared_max)
        ancho_carril = rb + min(lado_opuesto, self.pared_max)
        d_des_ef = max(min(self.d_des, ancho_carril / 2), min(self.d_des, self.d_des_min))
        e = d_des_ef - d_t1

        dt = max(dt, 1e-3)
        de = 0.0 if self.e_prev is None else (e - self.e_prev) / dt
        k = dt / (self.tau_d + dt)        # pasa-bajos de la derivada (ruido del LiDAR)
        self.de_f += k * (de - self.de_f)
        self.e_prev = e
        u = self.kp * e + self.ki * self.integral + self.kd * self.de_f
        w = -self.signo * u
        w_sat = max(-self.w_max, min(self.w_max, w))
        if w == w_sat:                                   # anti-windup: no integra saturado
            self.integral += e * dt

        # velocidad: más lento cuanto más fuerte gira ...
        v = self.v_max - (self.v_max - self.v_min) * min(abs(w_sat) / self.w_ref, 1.0)
        # ... y más lento si hay muro cerca al frente (entrada a curva cerrada)
        d_frente = b.minimo_en(0.0, math.radians(10))
        if d_frente < self.d_frente_lento:
            f = (d_frente - self.d_frente_min) / (self.d_frente_lento - self.d_frente_min)
            v = min(v, self.v_min + (self.v_max - self.v_min) * max(0.0, min(1.0, f)))
        self.info = {'a': ra, 'b': rb, 'alpha': alpha, 'd_t': d_t, 'd_t1': d_t1, 'e': e,
                     'w': w_sat, 'v': v, 'd_frente': d_frente, 'd_c': d_c,
                     'd_des_ef': d_des_ef, 'retenido': 0.0}
        self.v_cmd_prev = v
        return v, w_sat


# --------------------------------------------------------------------------
# Cruce de la línea de salida/meta
# --------------------------------------------------------------------------
def cruza_linea(p_prev, p_act, x_linea=0.0, y_min=-0.4, y_max=0.4):
    """
    Detecta si el robot pasó de x < x_linea a x >= x_linea (avance en +x) por la línea.

    Devuelve (cruzo, fraccion): fraccion en [0, 1] para interpolar el instante del cruce.
    """
    x0, y0 = p_prev
    x1, y1 = p_act
    if x0 < x_linea <= x1 and (x1 - x0) < 1.0:
        f = (x_linea - x0) / (x1 - x0)
        y = y0 + f * (y1 - y0)
        if y_min <= y <= y_max:
            return True, f
    return False, 0.0
