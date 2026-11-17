#!/usr/bin/env python3
"""
Nodo de cinemática (Entrega 1) extendido para la Entrega 2 (requisito 2: odometría).

Qué hace:
  - Lee /joint_states y toma w_D y w_I como el PROMEDIO de las dos ruedas de cada lado
    (skid-steer de 4 ruedas tratado como diferencial equivalente).
  - Integra la pose eta = [x, y, psi] (Euler, igual que en la Entrega 1), arrancando en la
    pose de spawn para que quede en coordenadas del mundo.
  - La compara con la pose REAL de Gazebo (/odom_real, plugin OdometryPublisher) y con la
    odometría del plugin DiffDrive (/odom).
  - Publica /odom_calc (Odometry), /trayectoria_calc y /trayectoria_real (Path, para RViz).
  - Calibración (opcional, parámetro via_calibrada > 0): integra en paralelo la misma
    odometría con una vía EFECTIVA b_ef = b / k_giro y la publica en /odom_calc_cal. En
    skid-steer el robot gira menos de lo que predice el modelo diferencial (las ruedas
    patinan de lado); k_giro se midió aparte, girando en el sitio (ver docs/BITACORA.md).

Entradas:  /joint_states (sensor_msgs/JointState), /odom_real y /odom (nav_msgs/Odometry)
Salidas:   /odom_calc, /odom_calc_cal, /trayectoria_calc, /trayectoria_real
"""
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from robot_diferencial_pkg.nucleo import envolver, OdometriaDiferencial
from sensor_msgs.msg import JointState


def yaw_de(q):
    """Yaw (psi) de un cuaternión."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class CinematicaDiferencial(Node):
    def __init__(self):
        super().__init__('nodo_cinematica')
        # Parámetros del robot (deben coincidir con urdf/robot.urdf.xacro)
        self.declare_parameter('radio_rueda', 0.0245)
        self.declare_parameter('via', 0.134)
        self.declare_parameter('via_calibrada', 0.153)   # b / k_giro = 0.134 / 0.876; 0 = no
        self.declare_parameter('x0', -0.60)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('yaw0', 0.0)
        self.declare_parameter('juntas_izq', ['front_left_wheel_joint', 'rear_left_wheel_joint'])
        self.declare_parameter('juntas_der', ['front_right_wheel_joint', 'rear_right_wheel_joint'])
        self.declare_parameter('marco', 'mundo')
        self.declare_parameter('periodo_log', 2.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.odo = OdometriaDiferencial(g('radio_rueda'), g('via'), g('x0'), g('y0'), g('yaw0'))
        self.odo_cal = None
        if g('via_calibrada') > 0.0:
            self.odo_cal = OdometriaDiferencial(g('radio_rueda'), g('via_calibrada'),
                                                g('x0'), g('y0'), g('yaw0'))
        self.juntas_izq, self.juntas_der = list(g('juntas_izq')), list(g('juntas_der'))
        self.marco = g('marco')
        self.t_prev = None
        self.real = None            # [x, y, psi] real (Gazebo)
        self.plugin = None          # [x, y, psi] del plugin DiffDrive (relativa al spawn)
        self.spawn = (g('x0'), g('y0'), g('yaw0'))
        self.avisado = False

        self.create_subscription(JointState, '/joint_states', self.cb_juntas, 50)
        self.create_subscription(Odometry, '/odom_real', self.cb_real, 50)
        self.create_subscription(Odometry, '/odom', self.cb_plugin, 50)
        self.pub_odom = self.create_publisher(Odometry, '/odom_calc', 50)
        self.pub_odom_cal = self.create_publisher(Odometry, '/odom_calc_cal', 50)
        self.pub_path_calc = self.create_publisher(Path, '/trayectoria_calc', 5)
        self.pub_path_real = self.create_publisher(Path, '/trayectoria_real', 5)
        self.path_calc, self.path_real = Path(), Path()
        self.path_calc.header.frame_id = self.path_real.header.frame_id = self.marco
        self.create_timer(0.2, self.cb_paths)
        self.create_timer(float(g('periodo_log')), self.cb_log)

    # ---- entradas -------------------------------------------------------
    def cb_real(self, msg):
        p = msg.pose.pose
        self.real = (p.position.x, p.position.y, yaw_de(p.orientation))

    def cb_plugin(self, msg):
        # /odom del DiffDrive arranca en (0,0,0): se lleva al mundo con la pose de spawn
        p = msg.pose.pose
        x0, y0, yaw0 = self.spawn
        c, s = math.cos(yaw0), math.sin(yaw0)
        self.plugin = (x0 + c * p.position.x - s * p.position.y,
                       y0 + s * p.position.x + c * p.position.y,
                       envolver(yaw0 + yaw_de(p.orientation)))

    def cb_juntas(self, msg):
        # dt con el sello de tiempo del mensaje (tiempo de simulación), no con el reloj de pared
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        if self.t_prev is None:
            self.t_prev = t
            return
        dt = t - self.t_prev
        self.t_prev = t
        if dt <= 0.0 or dt > 1.0:
            return
        try:
            v = msg.velocity
            w_I = sum(v[msg.name.index(j)] for j in self.juntas_izq) / len(self.juntas_izq)
            w_D = sum(v[msg.name.index(j)] for j in self.juntas_der) / len(self.juntas_der)
        except (ValueError, IndexError):
            if not self.avisado:
                self.get_logger().warn(
                    f'/joint_states no trae las juntas esperadas: {list(msg.name)}')
                self.avisado = True
            return

        self.pub_odom.publish(self._mensaje(self.odo, w_D, w_I, dt, msg.header.stamp))
        if self.odo_cal is not None:
            self.pub_odom_cal.publish(self._mensaje(self.odo_cal, w_D, w_I, dt, msg.header.stamp))

    def _mensaje(self, odo, w_D, w_I, dt, stamp):
        u, psi_p = odo.paso(w_D, w_I, dt)
        x, y, psi = odo.eta
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.marco
        o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = float(x), float(y)
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(psi / 2), math.cos(psi / 2)
        o.twist.twist.linear.x, o.twist.twist.angular.z = float(u), float(psi_p)
        return o

    # ---- salidas periódicas --------------------------------------------
    def _agregar(self, path, x, y):
        ps = PoseStamped()
        ps.header.frame_id = self.marco
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
        if len(path.poses) > 5000:
            del path.poses[:1000]

    def cb_paths(self):
        ahora = self.get_clock().now().to_msg()
        self._agregar(self.path_calc, self.odo.eta[0], self.odo.eta[1])
        self.path_calc.header.stamp = ahora
        self.pub_path_calc.publish(self.path_calc)
        if self.real is not None:
            self._agregar(self.path_real, self.real[0], self.real[1])
            self.path_real.header.stamp = ahora
            self.pub_path_real.publish(self.path_real)

    def cb_log(self):
        x, y, psi = self.odo.eta
        txt = (f'\nCalc  : x={x:.3f}, y={y:.3f}, psi={psi:.3f}'
               f'  (recorrido {self.odo.distancia:.1f} m)')
        for nombre, ref in (('Real  ', self.real), ('Plugin', self.plugin)):
            if ref is not None:
                e = math.hypot(x - ref[0], y - ref[1])
                txt += (f'\n{nombre}: x={ref[0]:.3f}, y={ref[1]:.3f}, psi={ref[2]:.3f}'
                        f'  | error calc: pos={e:.3f} m, psi={envolver(psi - ref[2]):.3f} rad')
        self.get_logger().info(txt)


def main(args=None):
    rclpy.init(args=args)
    nodo = CinematicaDiferencial()
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
