#!/usr/bin/env python3
"""
AEB: frenado autónomo de emergencia por tiempo a colisión (requisito 3).

Es una COMPUERTA: todo comando de velocidad (teleop o autónomo) pasa por aquí antes de
llegar a /cmd_vel. Nadie más publica en /cmd_vel.

    /cmd_vel_mux --> [aeb_node] --> /cmd_vel          (además /aeb/ttc y /aeb/activo)
                      ^      ^
                    /scan   /odom (velocidad medida)

Por cada rayo:  r_p_i = -v cos(theta_i),  TTC_i = r_i / max(-r_p_i, 0)
Frena si  min TTC_i < tau(v) = max(tau_0, t_lat + |v| / (2 a_max)).  Ver nucleo.AEB.
"""
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_diferencial_pkg.nucleo import AEB, Barrido
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class AebNode(Node):
    def __init__(self):
        super().__init__('aeb_node')
        self.declare_parameter('modo_ttc', 'corredor')   # corredor | ittc
        self.declare_parameter('largo', 0.1986)
        self.declare_parameter('ancho', 0.153)
        self.declare_parameter('x_lidar', 0.0)           # posición del LiDAR respecto al centro
        self.declare_parameter('a_max', 2.0)             # desaceleración disponible [m/s^2]
        self.declare_parameter('t_lat', 0.20)            # latencia sensor -> actuador [s]
        self.declare_parameter('tau_0', 0.30)            # umbral mínimo [s]
        self.declare_parameter('margen', 0.03)           # margen lateral del corredor [m]
        self.declare_parameter('d_seg', 0.05)            # colchón al detenerse [m]
        self.declare_parameter('t_scan_viejo', 0.5)      # sin LiDAR reciente -> no se mueve
        self.declare_parameter('habilitado', True)       # SOLO para demostrar qué pasa sin AEB
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.aeb = AEB(g('largo'), g('ancho'), g('x_lidar'), g('a_max'), g('t_lat'), g('tau_0'),
                       g('margen'), g('d_seg'), g('modo_ttc'))
        self.t_scan_viejo = float(g('t_scan_viejo'))

        self.barrido, self.t_scan = None, None
        self.cmd = Twist()
        self.v_med = 0.0
        self.activo_prev = False

        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel_mux', self.cb_cmd, 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 20)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_ttc = self.create_publisher(Float32, '/aeb/ttc', 10)
        self.pub_activo = self.create_publisher(Bool, '/aeb/activo', 10)
        self.create_timer(0.05, self.cb_timer)    # 20 Hz: decide aunque el comando no cambie
        self.get_logger().info(
            f"AEB listo: modo={g('modo_ttc')}, tau(1.0 m/s)={self.aeb.tau(1.0):.2f} s, "
            f'd_frenado(1.0 m/s)={self.aeb.distancia_frenado(1.0):.2f} m')

    def cb_scan(self, msg):
        self.barrido = Barrido(msg.ranges, msg.angle_min, msg.angle_increment,
                               msg.range_min, msg.range_max)
        self.t_scan = self.get_clock().now()

    def cb_cmd(self, msg):
        self.cmd = msg

    def cb_odom(self, msg):
        self.v_med = msg.twist.twist.linear.x

    def cb_timer(self):
        out = Twist()
        v_cmd, w_cmd = self.cmd.linear.x, self.cmd.angular.z
        sin_lidar = (self.t_scan is None or
                     (self.get_clock().now() - self.t_scan).nanoseconds * 1e-9 > self.t_scan_viejo)
        if not self.get_parameter('habilitado').value:
            out.linear.x, out.angular.z = v_cmd, w_cmd
            activo, ttc = False, math.inf
        elif sin_lidar:
            activo, ttc = True, 0.0                      # sin ojos no se avanza (el giro tampoco)
        else:
            v, w, activo, ttc = self.aeb.decidir(self.barrido, v_cmd, w_cmd, self.v_med)
            out.linear.x, out.angular.z = float(v), float(w)
        self.pub_cmd.publish(out)
        self.pub_ttc.publish(Float32(data=float(min(ttc, 99.0))))
        self.pub_activo.publish(Bool(data=bool(activo)))
        if activo and not self.activo_prev:
            tau = self.aeb.tau(max(abs(v_cmd), abs(self.v_med)))
            self.get_logger().warn(
                f'AEB ACTIVO: TTC={ttc:.2f} s < tau={tau:.2f} s '
                f'(v_med={self.v_med:.2f}, v_cmd={v_cmd:.2f})' if not sin_lidar
                else 'AEB ACTIVO: no llega /scan')
        self.activo_prev = activo


def main(args=None):
    rclpy.init(args=args)
    nodo = AebNode()
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
