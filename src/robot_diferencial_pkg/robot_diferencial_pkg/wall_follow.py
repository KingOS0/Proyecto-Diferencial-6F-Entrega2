#!/usr/bin/env python3
"""
Seguimiento de pared con LiDAR (requisito 4): PID sobre la distancia a una pared.

Reactivo: decide solo con el último barrido de /scan (sin mapa, waypoints ni trayectorias).
Publica en /cmd_vel_auto; el comando pasa por mux_cmd y por el AEB antes de llegar al robot.

    alpha = arctan((a cos(theta) - b) / (a sin(theta)))
    D_t = b cos(alpha),  D_{t+1} = D_t + l sin(alpha),  e = D_des - D_{t+1}
    omega = -(Kp e + Ki int(e) + Kd de/dt)   (pared izquierda);  v baja cuando |omega| sube
Tercer rayo c (a gamma del frente): D_{t+1} = min(D_{t+1}, c sin(gamma)) para anticipar
cuando la pared se cierra adelante. Ver nucleo.SeguidorPared.

Depuración (rqt_plot): /wall/error, /wall/distancia, /wall/alpha
"""
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from robot_diferencial_pkg.nucleo import Barrido, SeguidorPared
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

# nombre del parámetro ROS -> valor por defecto (ajustados en tools/sim2d.py; ver docs)
PARAMS = {'lado': 'izquierda', 'd_des': 0.25, 'theta_deg': 50.0, 'l': 0.35, 'kp': 12.0,
          'ki': 0.0, 'kd': 1.0, 'v_max': 1.0, 'v_min': 0.35, 'w_max': 4.0, 'w_ref': 4.0,
          'd_frente_lento': 0.9, 'd_frente_min': 0.30, 'pared_max': 1.2, 'tau_d': 0.10,
          'gamma_deg': 20.0, 'd_des_min': 0.20}


class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow')
        for nombre, valor in PARAMS.items():
            self.declare_parameter(nombre, valor)
        self.seguidor = self._crear()
        self.add_post_set_parameters_callback(self.cb_params)
        self.v_med, self.t_prev, self.frenado = None, None, None
        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 20)
        # estado del AEB: si está reteniendo al robot, el seguidor deja de mandar giro
        self.create_subscription(Bool, '/aeb/activo', self.cb_aeb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self.pub_e = self.create_publisher(Float32, '/wall/error', 10)
        self.pub_d = self.create_publisher(Float32, '/wall/distancia', 10)
        self.pub_a = self.create_publisher(Float32, '/wall/alpha', 10)

    def _crear(self):
        p = {n: self.get_parameter(n).value for n in PARAMS}
        p['theta'] = math.radians(p.pop('theta_deg'))
        p['gamma'] = math.radians(p.pop('gamma_deg'))
        self.get_logger().info(f'wall_follow: {p}')
        return SeguidorPared(**p)

    def cb_params(self, _params):
        # permite sintonizar en vivo:  ros2 param set /wall_follow kp 10.0
        self.seguidor = self._crear()

    def cb_aeb(self, msg):
        self.frenado = bool(msg.data)

    def cb_odom(self, msg):
        self.v_med = msg.twist.twist.linear.x

    def cb_scan(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        dt = 0.1 if self.t_prev is None or not (0.0 < t - self.t_prev < 1.0) else t - self.t_prev
        self.t_prev = t
        b = Barrido(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        v, w = self.seguidor.paso(b, dt, self.v_med, self.frenado)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.pub.publish(cmd)
        info = self.seguidor.info
        self.pub_e.publish(Float32(data=float(info.get('e', 0.0))))
        self.pub_d.publish(Float32(data=float(info.get('d_t', 0.0))))
        self.pub_a.publish(Float32(data=float(info.get('alpha', 0.0))))


def main(args=None):
    rclpy.init(args=args)
    nodo = WallFollow()
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
