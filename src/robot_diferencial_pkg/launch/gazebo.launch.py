"""
Launch único de la Entrega 2 (requisito 7: integración).

    ros2 launch robot_diferencial_pkg gazebo.launch.py modo:=teleop pista:=propia
    ros2 launch robot_diferencial_pkg gazebo.launch.py modo:=auto   pista:=competencia

Argumentos:
    modo   teleop | auto          quién manda: teclado o seguimiento de pared
    pista  propia | competencia | entrega1
    rviz   true | false
    gui    true | false           false = Gazebo sin ventana (solo servidor)
    teclado true | false          false = no abre la ventana de teleop_twist_keyboard
    x, y, yaw                     pose de spawn (por defecto la de la competencia: -0.60, 0, 0)
    mundo                         ruta a un .sdf externo (si el profesor entrega otro world)
    aeb_modo  corredor | ittc     cómo filtra los rayos el AEB
    lado, d_des, v_max            ajustes rápidos del seguidor de pared

Cadena de comandos (nadie más publica en /cmd_vel):
    teleop_twist_keyboard -> /cmd_vel_teleop --+
                                               +-> mux_cmd -> /cmd_vel_mux -> aeb_node -> /cmd_vel
    wall_follow (solo modo auto) -> /cmd_vel_auto --+
"""
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'robot_diferencial_pkg'
MUNDOS = {'propia': 'pista_propia.sdf', 'competencia': 'pista_competencia.sdf',
          'entrega1': 'pista.sdf'}


def terminal_para_teleop():
    """
    Prefijo para abrir teleop_twist_keyboard en su propia ventana (lee el teclado).

    Usa el emulador de terminal que esté instalado; None si no hay ninguno.
    """
    opciones = [('xterm', 'xterm -fa Monospace -fs 12 -T TELEOP -e'),
                ('gnome-terminal', 'gnome-terminal --wait --title=TELEOP --'),
                ('terminator', 'terminator -T TELEOP -x'),
                ('konsole', 'konsole -e')]
    for binario, prefijo in opciones:
        if shutil.which(binario):
            return prefijo
    return None


def configurar(context):
    cfg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    pkg = get_package_share_directory(PKG)
    modo, pista = cfg('modo'), cfg('pista')
    if modo not in ('teleop', 'auto'):
        raise RuntimeError(f'modo:={modo} no existe (use teleop o auto)')
    if pista not in MUNDOS:
        raise RuntimeError(f"pista:={pista} no existe (use {', '.join(MUNDOS)})")
    mundo = cfg('mundo') or os.path.join(pkg, 'worlds', MUNDOS[pista])
    x, y, yaw = cfg('x'), cfg('y'), cfg('yaw')
    gui = cfg('gui').lower() == 'true'
    gz_args = f'-r {mundo}' + ('' if gui else ' -s --headless-rendering')
    sim = {'use_sim_time': True}

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg, 'urdf', 'robot.urdf.xacro')]), value_type=str)

    acciones = [
        LogInfo(msg=f'[Entrega 2] modo={modo}  pista={pista}  mundo={mundo}  '
                    f'spawn=({x}, {y}, {yaw})'),

        # --- Simulación -------------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args}.items()),

        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}, sim]),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'robot_diferencial', '-topic', 'robot_description',
                        '-x', x, '-y', y, '-z', '0.05', '-Y', yaw]),

        # ]  ROS -> Gazebo        [  Gazebo -> ROS
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[sim],
             arguments=[
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                 '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                 # odometría del DiffDrive (de ruedas) / pose real (OdometryPublisher)
                 '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                 '/odom_real@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                 # TF mundo -> base_footprint (pose real)
                 '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                 '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                 '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                 '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
             ]),

        # --- Cadena de comandos: mux -> AEB -> /cmd_vel ----------------------------------
        Node(package=PKG, executable='mux_cmd', output='screen',
             parameters=[sim, {'modo': modo}]),

        Node(package=PKG, executable='aeb_node', output='screen',
             parameters=[sim, {'modo_ttc': cfg('aeb_modo')}]),

        # --- Odometría (req. 2) y cronómetro de vueltas (req. 2 y 6) -------------------------
        Node(package=PKG, executable='nodo_cinematica', output='screen',
             parameters=[sim, {'x0': float(x), 'y0': float(y), 'yaw0': float(yaw)}]),

        Node(package=PKG, executable='cronometro_vueltas', output='screen',
             parameters=[sim, {'etiqueta': f'{pista}_{modo}'}]),
    ]

    # --- Modo autónomo: seguimiento de pared ---------------------------------------------
    if modo == 'auto':
        acciones.append(Node(
            package=PKG, executable='wall_follow', output='screen',
            parameters=[sim, {'lado': cfg('lado'), 'd_des': float(cfg('d_des')),
                              'v_max': float(cfg('v_max'))}]))

    # --- Teleoperación: siempre disponible en su ventana, pero el mux solo la deja pasar
    #     en modo teleop (en la categoría autónoma no se pueden intervenir los comandos) ---
    if modo == 'teleop' and cfg('teclado').lower() == 'true':
        prefijo = terminal_para_teleop()
        if prefijo:
            acciones.append(Node(
                package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
                name='teleop', output='screen', prefix=prefijo,
                remappings=[('/cmd_vel', '/cmd_vel_teleop')]))
        else:
            acciones.append(LogInfo(msg=(
                '[Entrega 2] No hay xterm/gnome-terminal/terminator: abra otra terminal y corra\n'
                '  ros2 run teleop_twist_keyboard teleop_twist_keyboard '
                '--ros-args -r /cmd_vel:=/cmd_vel_teleop')))

    if cfg('rviz').lower() == 'true':
        acciones.append(Node(
            package='rviz2', executable='rviz2', output='log', parameters=[sim],
            arguments=['-d', os.path.join(pkg, 'config', 'competencia.rviz')]))
    return acciones


def generate_launch_description():
    pkg = get_package_share_directory(PKG)
    return LaunchDescription([
        DeclareLaunchArgument('modo', default_value='teleop', description='teleop | auto'),
        DeclareLaunchArgument('pista', default_value='propia',
                              description='propia | competencia | entrega1'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('teclado', default_value='true',
                              description='false: sin ventana de teleop (pruebas por topics)'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false: Gazebo sin ventana (corridas largas)'),
        DeclareLaunchArgument('x', default_value='-0.60'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('mundo', default_value='',
                              description='ruta a un .sdf externo (opcional)'),
        DeclareLaunchArgument('aeb_modo', default_value='corredor', description='corredor | ittc'),
        DeclareLaunchArgument('lado', default_value='izquierda',
                              description='pared que sigue: izquierda | derecha'),
        DeclareLaunchArgument('d_des', default_value='0.25',
                              description='distancia deseada a la pared [m]'),
        DeclareLaunchArgument('v_max', default_value='1.0',
                              description='velocidad máxima del seguidor [m/s]'),
        # Gazebo busca los modelos (model://pista_...) en esta carpeta
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models')),
        OpaqueFunction(function=configurar),
    ])
