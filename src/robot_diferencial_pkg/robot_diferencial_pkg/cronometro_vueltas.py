#!/usr/bin/env python3
"""
Cronómetro de vueltas y error de cierre de la odometría (requisitos 2 y 6).

- Detecta el cruce de la línea de salida/meta (x = 0, avance en +x) con la pose REAL
  (/odom_real). El tiempo es el sello del mensaje = tiempo de simulación (/clock).
- En cada cruce compara la pose estimada por nodo_cinematica (/odom_calc) con la real:
  ese es el error de cierre de la vuelta. Si existe /odom_calc_cal (vía calibrada), también
  guarda su error, para comparar antes/después de calibrar.
- Guarda dos CSV en la carpeta 'carpeta' (por defecto ~/resultados_entrega2):
    vueltas_<fecha>.csv      una fila por cruce (tiempo de vuelta, error de cierre)
    trayectoria_<fecha>.csv  pose real vs estimada a 10 Hz (para tools/graficar_odometria.py)
"""
import csv
import math
import os
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from robot_diferencial_pkg.nodo_cinematica import yaw_de
from robot_diferencial_pkg.nucleo import cruza_linea, envolver
from std_msgs.msg import Bool, String


class Cronometro(Node):
    def __init__(self):
        super().__init__('cronometro_vueltas')
        self.declare_parameter('x_linea', 0.0)
        self.declare_parameter('y_min', -0.4)
        self.declare_parameter('y_max', 0.4)
        self.declare_parameter('vueltas', 3)
        self.declare_parameter('carpeta', os.path.expanduser('~/resultados_entrega2'))
        self.declare_parameter('etiqueta', 'corrida')
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.x_linea, self.y_min, self.y_max = g('x_linea'), g('y_min'), g('y_max')
        self.n_vueltas = int(g('vueltas'))

        os.makedirs(g('carpeta'), exist_ok=True)
        sello = time.strftime('%Y%m%d_%H%M%S')
        base = os.path.join(g('carpeta'), f"{g('etiqueta')}_{sello}")
        self.f_v = open(base + '_vueltas.csv', 'w', newline='')
        self.f_t = open(base + '_trayectoria.csv', 'w', newline='')
        self.csv_v, self.csv_t = csv.writer(self.f_v), csv.writer(self.f_t)
        self.csv_v.writerow(['cruce', 't_sim', 't_vuelta', 'x_real', 'y_real', 'psi_real',
                             'x_calc', 'y_calc', 'psi_calc', 'error_pos', 'error_psi',
                             'error_pos_cal', 'error_psi_cal', 'frenadas_aeb'])
        self.csv_t.writerow(['t_sim', 'x_real', 'y_real', 'psi_real', 'x_calc', 'y_calc',
                             'psi_calc', 'x_cal', 'y_cal', 'psi_cal', 'vuelta'])
        self.get_logger().info(f'Resultados en {base}_*.csv')

        self.prev = None            # (t, x, y) real anterior
        self.calc = None            # (x, y, psi) estimada
        self.cal = None             # (x, y, psi) estimada con la vía calibrada
        self.cruces = []            # tiempos de cruce
        self.t_csv = -1.0
        self.frenadas, self.aeb_prev = 0, False
        self.create_subscription(Odometry, '/odom_real', self.cb_real, 50)
        self.create_subscription(Odometry, '/odom_calc', self.cb_calc, 50)
        self.create_subscription(Odometry, '/odom_calc_cal', self.cb_cal, 50)
        self.create_subscription(Bool, '/aeb/activo', self.cb_aeb, 10)
        self.pub = self.create_publisher(String, '/vueltas', 10)

    def cb_calc(self, msg):
        p = msg.pose.pose
        self.calc = (p.position.x, p.position.y, yaw_de(p.orientation))

    def cb_cal(self, msg):
        p = msg.pose.pose
        self.cal = (p.position.x, p.position.y, yaw_de(p.orientation))

    def cb_aeb(self, msg):
        if msg.data and not self.aeb_prev:
            self.frenadas += 1
        self.aeb_prev = msg.data

    def cb_real(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        p = msg.pose.pose
        real = (p.position.x, p.position.y, yaw_de(p.orientation))
        nan3 = (math.nan, math.nan, math.nan)
        calc, cal = self.calc or nan3, self.cal or nan3
        if t - self.t_csv >= 0.1:
            self.t_csv = t
            self.csv_t.writerow([f'{t:.3f}'] + [f'{v:.4f}' for v in real + calc + cal]
                                + [len(self.cruces)])
        if self.prev is not None:
            ok, f = cruza_linea(self.prev[1:], real[:2], self.x_linea, self.y_min, self.y_max)
            if ok:
                self.registrar(self.prev[0] + f * (t - self.prev[0]), real, calc, cal)
        self.prev = (t, real[0], real[1])

    @staticmethod
    def error(est, real):
        """Error de cierre (posición [m], rumbo [rad]) de una estimación frente a la pose real."""
        return math.hypot(est[0] - real[0], est[1] - real[1]), envolver(est[2] - real[2])

    def registrar(self, t, real, calc, cal):
        n = len(self.cruces)
        t_vuelta = t - self.cruces[-1] if n else 0.0
        self.cruces.append(t)
        e_pos, e_psi = self.error(calc, real)
        e_pos_c, e_psi_c = self.error(cal, real)
        self.csv_v.writerow([n, f'{t:.3f}', f'{t_vuelta:.3f}']
                            + [f'{v:.4f}' for v in real + calc + (e_pos, e_psi, e_pos_c, e_psi_c)]
                            + [self.frenadas])
        self.f_v.flush()
        self.f_t.flush()
        if n == 0:
            txt = f'SALIDA en t_sim={t:.2f} s'
        else:
            txt = (f'VUELTA {n}: {t_vuelta:.2f} s | error de cierre: {e_pos:.3f} m, '
                   f'{math.degrees(e_psi):.1f} deg')
            if not math.isnan(e_pos_c):
                txt += f' (calibrada: {e_pos_c:.3f} m, {math.degrees(e_psi_c):.1f} deg)'
            txt += f' | frenadas AEB acumuladas: {self.frenadas}'
            if n == self.n_vueltas:
                txt += f' | TOTAL {self.n_vueltas} vueltas: {t - self.cruces[0]:.2f} s'
        self.get_logger().info(txt)
        self.pub.publish(String(data=txt))

    def destroy_node(self):
        self.f_v.close()
        self.f_t.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = Cronometro()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                                    # Ctrl+C: cierre limpio, sin traza de error
    except RuntimeError:                        # RCLError si la señal llega esperando mensajes
        if rclpy.ok():
            raise
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
