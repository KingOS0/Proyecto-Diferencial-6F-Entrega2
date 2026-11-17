# Robot diferencial en ROS 2 — Entrega 2: AEB, seguimiento de pared y competencia

**Institución:** Universidad EIA — Ingeniería Mecatrónica — Robótica 2026-2

### Integrantes del equipo

* Idekel Plaza Osorio
* Cristian Ramirez
* Samuel Patiño
* Matteo Paganessi

---

## 1. Qué hace este paquete

Parte del paquete de la Entrega 1 (robot diferencial en Gazebo con LiDAR, cámara, nodo de
cinemática y teleoperación) y le agrega lo que pide el reporte *"Robot diferencial: AEB,
seguimiento de pared y competencia"*:

| Requisito | Dónde está |
|---|---|
| 1. Adaptación al MicroROS de Yahboom (4 ruedas skid-steer, LiDAR MS200, límites) | `urdf/robot.urdf.xacro` |
| 2. Odometría en varias vueltas y error de cierre | `nodo_cinematica.py`, `cronometro_vueltas.py`, `tools/graficar_odometria.py` |
| 3. AEB por tiempo a colisión (TTC) | `aeb_node.py` (lógica en `nucleo.py`) |
| 4. Seguimiento de pared con PID | `wall_follow.py` (lógica en `nucleo.py`) |
| 5. Pista propia (STL en metros, mesh con colisión) | `models/pista_propia/`, `worlds/pista_propia.sdf`, `tools/generar_pista.py` |
| 6. Modo autónomo: 3 vueltas cronometradas con `/clock` | `cronometro_vueltas.py` |
| 7. Integración: un solo launch con `modo` y `pista` | `launch/gazebo.launch.py`, `mux_cmd.py` |

Cadena de comandos (nadie más publica en `/cmd_vel`):

```text
teleop_twist_keyboard --> /cmd_vel_teleop --+
                                            +--> mux_cmd --> /cmd_vel_mux --> aeb_node --> /cmd_vel --> DiffDrive
wall_follow (modo auto) --> /cmd_vel_auto --+                                   ^    ^      |
                                                                             /scan  /odom   v
                                                                         /aeb/activo, /aeb/ttc
```

`wall_follow` también lee `/aeb/activo`: si el AEB lo está reteniendo, deja de mandar giro y
espera a que se despeje el camino.

## 2. Estructura

```text
robot_diferencial_pkg/
├── config/        competencia.rviz (Entrega 2), visor.rviz (Entrega 1)
├── launch/        gazebo.launch.py (launch único), display.launch.py (solo RViz)
├── models/        pista_competencia/ (del profesor), pista_propia/ (cad/*.FCStd + meshes/*.stl),
│                  obstaculo/ (caja para la prueba del AEB)
├── urdf/          robot.urdf.xacro
├── worlds/        pista_competencia.sdf, pista_propia.sdf, pista.sdf (Entrega 1)
├── robot_diferencial_pkg/
│   ├── nucleo.py              lógica sin ROS: odometría, TTC/AEB, seguidor de pared
│   ├── nodo_cinematica.py     odometría (promedio de las dos ruedas por lado) y versión calibrada
│   ├── aeb_node.py            compuerta de seguridad hacia /cmd_vel
│   ├── wall_follow.py         PID sobre la distancia a la pared
│   ├── mux_cmd.py             selecciona teleop o auto
│   └── cronometro_vueltas.py  tiempos por vuelta y error de cierre (CSV)
├── tools/         generar_pista.py + pista_cad_freecad.py (pista propia), graficar_odometria.py,
│                  sim2d.py y stl_a_segmentos.py (simulador 2D sin ROS)
└── test/          test_nucleo.py (pytest, sin ROS) + linters de ament
```

## 3. Compilar

En una terminal de Ubuntu con ROS 2 Jazzy instalado:

```bash
source /opt/ros/jazzy/setup.bash
```


```bash
cd ~/robotica_movil_entrega2_revisada  # el paquete está en src/
colcon build --symlink-install
source install/setup.bash
```

Dependencias (las del curso): ROS 2 Jazzy, Gazebo Harmonic (`ros-jazzy-ros-gz`), `xacro`,
`teleop_twist_keyboard`, `numpy`.

## 4. Ejecutar (un solo launch desde una terminal limpia)

```bash
ros2 launch robot_diferencial_pkg gazebo.launch.py modo:=teleop pista:=propia
ros2 launch robot_diferencial_pkg gazebo.launch.py modo:=auto   pista:=propia
ros2 launch robot_diferencial_pkg gazebo.launch.py modo:=teleop pista:=competencia
ros2 launch robot_diferencial_pkg gazebo.launch.py modo:=auto   pista:=competencia
```

* `modo:=teleop` abre una ventana con `teleop_twist_keyboard` (publica en `/cmd_vel_teleop`).
* `modo:=auto` lanza el seguidor de pared (publica en `/cmd_vel_auto`).
* En ambos modos el AEB está en la cadena y tiene prioridad.
* El cronómetro imprime cada vuelta y guarda CSV en `~/resultados_entrega2/`.
* Lanzar desde una terminal normal (Terminator, gnome-terminal). Desde la terminal integrada
  de VS Code instalado como snap, Gazebo y la ventana de teleop se cierran con
  `symbol lookup error ... /snap/core20/...` (variables de entorno del snap).

Otros argumentos:

| Argumento | Por defecto | Para qué |
|---|---|---|
| `rviz:=false` | `true` | sin RViz |
| `gui:=false` | `true` | Gazebo sin ventana (corridas largas) |
| `teclado:=false` | `true` | no abre la ventana de teleop (pruebas por topics) |
| `aeb_modo:=ittc` | `corredor` | AEB con todos los rayos: demuestra el frenado falso |
| `d_des:=0.20` | `0.25` | distancia a la pared [m] (0.30 o más no completa la pista) |
| `v_max:=0.6` | `1.0` | velocidad máxima del seguidor [m/s] |
| `lado:=derecha` | `izquierda` | pared que sigue |
| `x:= y:= yaw:=` | `-0.60 0 0` | pose de salida |
| `mundo:=/ruta.sdf` | | otro world |

Prueba del AEB con obstáculo (con la simulación corriendo):

```bash
ros2 run ros_gz_sim create -name caja -x 4.0 -y 0.0 -z 0 -file \
  $(ros2 pkg prefix robot_diferencial_pkg)/share/robot_diferencial_pkg/models/obstaculo/model.sdf
gz service -s /world/pista_competencia/remove --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "caja" type: MODEL'   # quitarla
```

El robot se detiene frente a la caja si esta queda en su camino (centro de la caja entre
y ≈ −0.11 y la pared izquierda) y sigue cuando se quita. Una caja pegada a la pared izquierda
la toma como pared y la rodea.

Ajuste en vivo:

```bash
ros2 param set /wall_follow kp 10.0
ros2 param set /mux_cmd modo auto
ros2 run rqt_plot rqt_plot /wall/error/data /aeb/ttc/data
```

## 5. Resultados medidos en Gazebo (tiempo de simulación)

| Prueba | Resultado |
|---|---|
| Auto, pista propia, 3 vueltas | 92.07 / 92.35 / 92.14 / 92.29 s (4 corridas), 0 frenadas AEB, 0 contactos |
| Auto, pista competencia, 3 vueltas | 116.00 / 115.96 / 115.96 / 115.92 s (4 corridas), 0 frenadas AEB, 0 contactos |
| AEB contra muro a 0.3 / 0.6 / 1.0 m/s | se detiene a 0.19 / 0.22 / 0.29 m del muro, sin tocar |
| Error de cierre tras 3 vueltas (competencia) | 7.2 m, 156° con b = 0.134 m; 0.35 m, 3° con b_ef = 0.153 m |

## 6. Gráficas de odometría (trayectoria y error de cierre)

```bash
cd src/robot_diferencial_pkg
python3 tools/graficar_odometria.py docs/resultados_finales/propia_auto_20261113_134710
```

`nodo_cinematica` publica dos estimaciones: `/odom_calc` con la vía nominal b = 0.134 m (lo
que pide el enunciado) y `/odom_calc_cal` con la vía efectiva b_ef = b / k_giro = 0.153 m
(k_giro = 0.876, medido girando en el sitio). La gráfica muestra las dos contra la pose real.

## 7. Pista propia (CAD)

Ejecutar los siguientes comandos desde `src/robot_diferencial_pkg`:


```bash
python3 tools/generar_pista.py             # trazado: DXF de los muros + plano acotado
freecadcmd tools/pista_cad_freecad.py      # (snap: freecad.cmd) modelo .FCStd y STL en metros
```

El modelo de FreeCAD (`models/pista_propia/cad/pista_propia.FCStd`, en mm) extruye los muros
300 mm y exporta `models/pista_propia/meshes/pista_propia.stl` escalado a metros.

## 8. Pruebas sin Gazebo

Ejecutar desde `src/robot_diferencial_pkg`:


```bash
python3 -m pytest test/ -q          # lógica (test_nucleo.py) + flake8 + pep257
python3 tools/sim2d.py --pista models/pista_competencia/meshes/pista_competencia.stl \
    --vueltas 3 --k_giro 0.88 --png /tmp/sim.png
```

## 9. Referencias

1. M. J. Ortiz Camacho, *Reporte Robot Diferencial: AEB, seguimiento de pared y competencia*, Robótica 2026-2, Universidad EIA (enunciado y ecuaciones de TTC y seguimiento de pared).
2. M. J. Ortiz Camacho, *LiDAR y Frenado Automático de Emergencia (AEB)* y *Navegación reactiva con LiDAR: Wall Following y Follow the Gap*, presentaciones de clase, Robótica 2026-2 (umbral dinámico, modo corredor, geometría de dos haces).
3. F1TENTH Autonomous Racing, *Lab 2: Automatic Emergency Braking* y *Lab 3: Wall Following*, https://f1tenth.org/learn.html
4. Open Robotics, documentación de Gazebo Sim (Harmonic): sistemas `DiffDrive`, `JointStatePublisher`, `OdometryPublisher` y sensor `gpu_lidar`, https://gazebosim.org/docs/harmonic
5. Yahboom, *MicroROS robot car* y LiDAR *Oradar MS200*: medidas y especificaciones (tabla del enunciado).
