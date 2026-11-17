# Robótica móvil — Entrega 2

Proyecto de un robot móvil diferencial de cuatro ruedas (skid-steer) desarrollado con **ROS 2 Jazzy** y **Gazebo Harmonic**. Incluye teleoperación, seguimiento autónomo de pared, frenado automático de emergencia (AEB), odometría y cronometraje de vueltas en dos pistas.
Integrantes: Idekel Plaza, Cristian Ramirez, Matteo Paganessi, Samuel Patiño

## 1. Requisitos y entorno

- Ubuntu 24.04, ROS 2 Jazzy y Gazebo Harmonic.
- Paquetes ROS utilizados: `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `xacro`, `rviz2` y `teleop_twist_keyboard`.
- Abrir los comandos en **Terminator o la terminal de Ubuntu**. No se recomienda la terminal integrada de VS Code cuando este está instalado mediante Snap, debido a posibles conflictos de bibliotecas gráficas.


## 2. Estructura del workspace

```text
robotica_movil_entrega2_github/
├── README.md                     # Manual general
├── .gitignore                    # Ignora compilaciones y archivos temporales
└── src/
    └── robot_diferencial_pkg/
        ├── robot_diferencial_pkg/ # Nodos Python
        ├── launch/               # Lanzadores ROS 2
        ├── config/               # Configuración RViz
        ├── models/               # Modelos de Gazebo, STL y CAD
        ├── urdf/                 # Descripción del robot
        ├── worlds/               # Mundos de Gazebo
        ├── tools/                # Herramientas auxiliares
        ├── test/                 # Pruebas de lógica y estilo
        ├── docs/                 # Enunciado y documentación técnica
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```


## 3. Compilar el workspace

```bash
cd /RUTA/AL/WORKSPACE
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Verificar que ROS reconozca el paquete:

```bash
ros2 pkg list | grep robot_diferencial_pkg
```

**Importante:** cada terminal nueva que ejecute comandos ROS debe cargar `/opt/ros/jazzy/setup.bash` y el `install/setup.bash` del workspace. Si ya se está en la carpeta raíz:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 4. Ejecutar Gazebo: modo autónomo

El robot sigue una pared mediante su controlador y mantiene activo el AEB. Ejecutar **solo una de estas pruebas a la vez**:

**Pista de competencia:**

```bash
ros2 launch robot_diferencial_pkg gazebo.launch.py modo:=auto pista:=competencia
```

**Pista propia:**

```bash
ros2 launch robot_diferencial_pkg gazebo.launch.py modo:=auto pista:=propia
```

En la terminal de lanzamiento, el nodo de cronometraje informa los cruces y tiempos por vuelta. Para detener Gazebo, usar `Ctrl+C` en la terminal de lanzamiento.

## 5. Ejecutar Gazebo: teleoperación

En este modo se utiliza `teleop_twist_keyboard`, que el lanzamiento intenta abrir en una ventana de terminal aparte.

**Pista de competencia:**

```bash
ros2 launch robot_diferencial_pkg gazebo.launch.py modo:=teleop pista:=competencia
```

**Pista propia:**

```bash
ros2 launch robot_diferencial_pkg gazebo.launch.py modo:=teleop pista:=propia
```

Para controlar el robot, seleccionar la ventana de teleoperación y seguir las indicaciones de teclas mostradas allí. Presionar `k` para detener el movimiento. El AEB sigue interviniendo sobre la orden de velocidad.

**Si no se abre la ventana de teleoperación:** iniciar el launch con `teclado:=false` y, en una segunda terminal con el entorno ROS cargado, ejecutar:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop
```

## 6. Ejecutar el mundo de la Entrega 1

```bash
ros2 launch robot_diferencial_pkg gazebo.launch.py modo:=teleop pista:=entrega1
```

## 7. Opciones del lanzamiento


**Ejemplo: autónomo sin RViz:**

```bash
ros2 launch robot_diferencial_pkg gazebo.launch.py modo:=auto pista:=competencia rviz:=false
```

**Ejemplo: cambiar la velocidad máxima:**

```bash
ros2 launch robot_diferencial_pkg gazebo.launch.py modo:=auto pista:=propia v_max:=0.6
```

## 8. Frenado automático de emergencia (AEB)

El nodo `aeb_node` aplica la protección de seguridad a las órdenes que recibe, tanto en teleoperación como en modo autónomo. Para consultar los temas disponibles, abrir otra terminal y cargar el entorno:

```bash
source /opt/ros/jazzy/setup.bash
source /RUTA/AL/WORKSPACE/install/setup.bash
ros2 topic list
ros2 topic echo /aeb/activo
```

**Prueba de obstáculo** (con Gazebo ejecutándose en la pista de competencia):

```bash
ros2 run ros_gz_sim create -name caja -x 4.0 -y 0.0 -z 0 -file \
  $(ros2 pkg prefix robot_diferencial_pkg)/share/robot_diferencial_pkg/models/obstaculo/model.sdf
```

Para eliminarla del mundo `pista_competencia`:

```bash
gz service -s /world/pista_competencia/remove --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "caja" type: MODEL'
```

La posición de la caja se utiliza como ensayo; solo habrá frenado cuando el obstáculo esté en la trayectoria del robot y dentro de los criterios del AEB.

## 9. Odometría y resultados

El proyecto publica estimaciones de odometría e información de las vueltas. Para revisar los temas:

```bash
ros2 topic echo /odom_calc
ros2 topic echo /odom_calc_cal
```

Ejecutar los comandos en terminales diferentes, porque `ros2 topic echo` permanece activo hasta `Ctrl+C`.

El cronómetro guarda archivos CSV en la carpeta del usuario:

```bash
ls -lh ~/resultados_entrega2/
```

Los archivos de vueltas contienen los cruces y tiempos; los de trayectoria contienen las poses necesarias para comparar las estimaciones con la referencia simulada. Para generar una gráfica a partir de una captura existente (desde la raíz del workspace):

```bash
python3 src/robot_diferencial_pkg/tools/graficar_odometria.py \
  src/robot_diferencial_pkg/docs/resultados_finales/propia_auto_20261113_134710
```

## 10. Ejecutar las pruebas del paquete

Desde la raíz del workspace, tras compilar y cargar el entorno:

```bash
colcon test --packages-select robot_diferencial_pkg
colcon test-result --verbose
```

Para ejecutar solo la prueba de la lógica principal:

```bash
python3 -m pytest src/robot_diferencial_pkg/test/test_nucleo.py -q
```


