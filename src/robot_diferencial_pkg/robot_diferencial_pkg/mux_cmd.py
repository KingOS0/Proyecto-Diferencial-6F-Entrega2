#!/usr/bin/env python3
"""
Selector de comandos (requisito 7): deja pasar teleop o autónomo hacia el AEB.

Teleop y autónomo publican en topics separados y este nodo deja pasar solo el del modo activo.

    /cmd_vel_teleop --+
                      +--> [mux_cmd] --> /cmd_vel_mux --> [aeb_node] --> /cmd_vel --> DiffDrive
    /cmd_vel_auto ----+

Parámetro 'modo': teleop | auto. Se puede cambiar en caliente:
    ros2 param set /mux_cmd modo auto
En modo auto el teclado NO interviene (regla de la competencia).
"""
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class MuxCmd(Node):
    def __init__(self):
        super().__init__('mux_cmd')
        self.declare_parameter('modo', 'teleop')
        self.declare_parameter('t_sin_auto', 0.5)      # s sin /cmd_vel_auto -> se detiene
        self.modo = self.get_parameter('modo').value
        self.t_sin_auto = float(self.get_parameter('t_sin_auto').value)
        self.ultimo = {'teleop': Twist(), 'auto': Twist()}
        self.t_auto = None
        self.add_on_set_parameters_callback(self.cb_param)
        self.create_subscription(Twist, '/cmd_vel_teleop', lambda m: self.cb('teleop', m), 10)
        self.create_subscription(Twist, '/cmd_vel_auto', lambda m: self.cb('auto', m), 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_mux', 10)
        self.create_timer(0.05, self.cb_timer)          # 20 Hz
        self.get_logger().info(f'mux_cmd en modo: {self.modo}')

    def cb_param(self, params):
        for p in params:
            if p.name == 'modo':
                if p.value not in ('teleop', 'auto'):
                    return SetParametersResult(successful=False,
                                               reason='modo debe ser teleop o auto')
                self.modo = p.value
                # al cambiar de modo arranca quieto
                self.ultimo = {'teleop': Twist(), 'auto': Twist()}
                self.get_logger().info(f'mux_cmd cambia a modo: {self.modo}')
        return SetParametersResult(successful=True)

    def cb(self, fuente, msg):
        self.ultimo[fuente] = msg
        if fuente == 'auto':
            self.t_auto = self.get_clock().now()

    def cb_timer(self):
        out = self.ultimo[self.modo]
        if self.modo == 'auto':
            # el seguidor publica a 10 Hz; si se cae, el robot no debe seguir con el último comando
            viejo = (self.t_auto is None or
                     (self.get_clock().now() - self.t_auto).nanoseconds * 1e-9 > self.t_sin_auto)
            if viejo:
                out = Twist()
        # teleop_twist_keyboard publica solo al oprimir tecla: se mantiene el último comando
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    nodo = MuxCmd()
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
