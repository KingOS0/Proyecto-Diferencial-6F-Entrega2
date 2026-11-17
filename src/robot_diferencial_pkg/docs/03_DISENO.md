# 03 — Diseño: arquitectura, ecuaciones, parámetros y supuestos

## 1. Arquitectura

```
                 /scan (10 Hz)                    /scan      /odom (v medida)
                     |                              |           |
teleop_twist_keyboard --> /cmd_vel_teleop --+       v           v
                                            +--> mux_cmd --> /cmd_vel_mux --> aeb_node --> /cmd_vel --> DiffDrive (Gazebo)
wall_follow (modo auto) --> /cmd_vel_auto --+                                   |
                                                                    /aeb/ttc, /aeb/activo

/joint_states --> nodo_cinematica --> /odom_calc, /trayectoria_calc, /trayectoria_real
/odom_real (pose real) --> cronometro_vueltas --> /vueltas + CSV en ~/resultados_entrega2/
```

Topics de Gazebo puenteados (launch): `/clock`, `/cmd_vel`, `/odom` (odometría del DiffDrive),
`/odom_real` (pose real, plugin OdometryPublisher), `/tf` (`mundo → base_footprint`, real),
`/joint_states`, `/scan`, `/image_raw`, `/camera_info`.

Por qué así:
- **AEB como compuerta** (clase lidar_aeb, "Arquitectura"): es un supervisor; no maneja, solo
  bloquea. Si es el único que publica en `/cmd_vel`, tiene prioridad por construcción.
- **mux propio en vez de `twist_mux`**: el enunciado permite ambos; el propio no agrega una
  dependencia que el computador de la competencia puede no tener.
- **Lógica en `nucleo.py` sin ROS**: se prueba con pytest y con el simulador 2D; los nodos solo
  traducen mensajes.
- **Marco `mundo` con la pose real**: `/odom` del DiffDrive también es odometría de ruedas y
  deriva (en skid-steer, mucho). Para medir el error de cierre hace falta la pose real.

## 2. Robot (requisito 1) — `urdf/robot.urdf.xacro`

En el archivo, cada valor está marcado [T] tabla, [≈] tabla estimada, [S] supuesto del equipo.
- 4 ruedas r = 0.0245, vía 0.134, entre ejes 0.096, ancho de rueda 0.019 (= 0.153 − 0.134).
- Chasis 0.1986 × 0.111 × 0.070, despeje 0.015 [S]. Masa total 0.906 kg repartida [S]:
  chasis 0.726, ruedas 4 × 0.030, LiDAR 0.040, cámara 0.020.
- DiffDrive con dos `left_joint` y dos `right_joint`; límites ±1.0 m/s, ±4.0 rad/s, ±2.0 m/s².
- Juntas con `limit effort=0.392 N·m velocity=47.1 rad/s` (4 kg·cm, 450 rpm).
- LiDAR MS200: 450 muestras en 360° (último rayo en π − 2π/450 para no repetir el de atrás),
  0.03–12 m, 10 Hz, ruido gaussiano σ = 0.01, plano de barrido a 0.108 m, centrado (x = 0) [S].
- Fricción de ruedas mu1 = 1.0, mu2 = 0.6 [S] (ver Fase 2: en skid-steer las ruedas tienen que
  patinar de lado para girar).

## 3. Odometría (requisito 2) — `nodo_cinematica.py`, `cronometro_vueltas.py`

`ω_D`, `ω_I` = promedio de las dos ruedas de cada lado. Misma matriz de la Entrega 1:

```
[x_p  ]   [ r/2 cos ψ   r/2 cos ψ ]
[y_p  ] = [ r/2 sin ψ   r/2 sin ψ ] [ω_D, ω_I]ᵀ       Euler: η ← η + η_p·dt
[ψ_p  ]   [ r/b         −r/b      ]
```

con r = 0.0245, b = 0.134, dt del sello de tiempo de `/joint_states`, pose inicial = spawn.
El cronómetro detecta el cruce de x = 0 (avance +x, y ∈ [−0.4, 0.4]) con la pose real e
interpola el instante. Error de cierre = pose estimada − pose real en ese instante.

Qué esperar (para el análisis): en skid-steer el giro real es menor que el que predice el
modelo diferencial ideal (las ruedas patinan de lado), así que la odometría **sobreestima el
giro**. El error de rumbo se acumula vuelta a vuelta y, como la posición integra el rumbo, el
error de posición crece más que linealmente con la distancia. Una mejora defendible: calibrar
una vía efectiva `b_ef > b` con una prueba de giro en el sitio (Fase 4) y mostrar antes/después.

## 4. AEB (requisito 3) — `nucleo.AEB`, `aeb_node.py`

Por rayo: `ṙ_i = −v cos θ_i`; `TTC_i = r_i / max(−ṙ_i, 0)`. Frena si `min TTC_i < τ(v)`.

- **Umbral dinámico** (clase lidar_aeb): `τ(v) = max(τ0, t_lat + |v|/(2 a_max))`, porque la
  distancia de frenado es `d = v·t_lat + v²/(2a)` y `d = v·TTC`.
  - `a_max = 2.0 m/s²`: el límite de aceleración del robot (tabla).
  - `t_lat = 0.20 s`: un barrido del LiDAR puede tener hasta 0.10 s (10 Hz) + ciclo del AEB
    0.05 s (20 Hz) + margen. **Medirlo** en la Fase 2 y ajustar.
  - `τ0 = 0.30 s`: piso para velocidades bajas.
  - Resultado: τ(1.0) = 0.45 s (frena a 0.45 m), τ(0.5) = 0.33 s, τ(0.35) = 0.30 s.
- **Descuento de carrocería**: el LiDAR está en el centro; a cada rango se le resta la
  distancia del LiDAR al borde del robot en esa dirección (0.099 m al frente) y un colchón
  `d_seg = 0.05 m`. Sin eso el robot "frena" cuando ya chocó.
- **Modo corredor** (clase lidar_aeb, "Falsos positivos"): solo cuentan los rayos cuyo impacto
  cae en la franja que barre el robot, `|y_i| < ancho/2 + margen` (0.0765 + 0.03). En el
  estrechamiento las paredes quedan a 0.20 m del centro, fuera de la franja: no frena.
  `aeb_modo:=ittc` cuenta todos los rayos y sirve para **demostrar** el falso positivo.
- **Decisión** (misma lógica de la clase): `emergencia` = TTC con v medida < τ; `inseguro` =
  TTC con v comandada < τ. Bloquea si `inseguro` o (`emergencia` y el comando va en el mismo
  sentido). Retroceder se evalúa con los rayos de atrás, así que se puede salir en reversa.
  Al frenar, el giro solo pasa si el robot ya está casi quieto.
- Sin `/scan` reciente (0.5 s) el robot no se mueve.
- Corre a 20 Hz con el último comando, porque `teleop_twist_keyboard` solo publica al oprimir.

## 5. Seguimiento de pared (requisito 4) — `nucleo.SeguidorPared`, `wall_follow.py`

Pared **izquierda** (interior): la pista se recorre en sentido antihorario (clase gap_wall).

```
b = rayo a +90°;  a = rayo a (90° − θ);  α = atan2(a cos θ − b, a sin θ);  D_t = b cos α
D_{t+1} = D_t + l sin α;   e = D_des − D_{t+1};   ω = −(Kp e + Ki ∫e + Kd ė)
v = v_max − (v_max − v_min)·min(|ω|/w_ref, 1)      y además v baja si hay muro al frente
```

Tercer rayo de anticipación (el enunciado dice "al menos dos rayos"): `c` a γ = 20° del
frente; `D_{t+1} = min(D_{t+1}, c sin γ)`. En recta `c sin γ = D_t` y no cambia nada; cuando la
pared se cierra adelante (curva a la derecha, rampa del estrechamiento) avisa ≈ 0.55 m antes.
En el simulador 2D quitó las frenadas falsas del AEB que quedaban en la pista propia.

| Parámetro | Valor | Justificación |
|---|---|---|
| `d_des` | 0.20 m | Mitad del ancho mínimo (0.40 m): el robot pasa centrado por el estrechamiento sin cambiar de consigna, con 0.12 m libres a cada lado. En el carril de 0.80 m va por la línea interior (la más corta). |
| `theta` | 50° | Dentro del rango típico 30°–70° de la clase. El rayo a queda a 40° del frente: ve la pared ≈ 0.24 m adelante. Más abierto anticipa más pero en las esquinas convexas (horquilla) ve "a través" de la esquina. |
| `l` | 0.35 m | Hace de término derivativo (ver abajo). Más grande suaviza pero recorta las curvas interiores. |
| `kp` | 12 | Ver análisis; barrido en `tools/sim2d.py`: con 4 oscila y frena en falso, con 8–16 da 0 frenadas. |
| `kd` | 1.0 | Amortiguamiento extra; derivada filtrada (τ_d = 0.10 s) por el ruido del LiDAR. |
| `ki` | 0 | La clase dice que el integral no suele hacer falta; el error estacionario en curva lo absorbe el margen. |
| `v_max` / `v_min` | 1.0 / 0.35 m/s | Límite del robot / velocidad en giro fuerte. |
| `w_ref` | 4.0 rad/s | ω a la que v baja hasta v_min (= ω máx. del robot). |
| `gamma` | 20° | Tercer rayo. |

**Análisis para justificar Kp, l y Kd** (sirve para el pitch). Con d = D − D_des pequeño:
`ḋ = v sin α ≈ v α` y `α̇ = −ω` (girar a la izquierda acerca el rumbo a la pared).
Con `ω = Kp (d + l α) + Kd ḋ`:  `d̈ + (Kp l + Kd v) ḋ + Kp v d = 0`, o sea

```
ω_n = sqrt(Kp v)        ζ = (Kp l + Kd v) / (2 sqrt(Kp v))
```

Con Kp = 12, l = 0.35, Kd = 1: a v = 1.0 m/s → ω_n = 3.5 rad/s, ζ = 0.75; a v = 0.35 →
ω_n = 2.0 rad/s, ζ = 1.1. El LiDAR muestrea a 10 Hz: ω_n·T = 0.35 rad (< 0.5), aceptable.
Lectura: **Kp fija la rapidez, l y Kd fijan el amortiguamiento**; l es un "derivativo
geométrico" que no amplifica ruido. Por eso Kp solo (l = 0, Kd = 0) oscila siempre.

**Si el robot queda retenido por el AEB** (`/aeb/activo`): el seguidor manda (v_min, 0), o
sea sigue pidiendo avanzar despacio pero sin girar; cuando se quita el obstáculo, el AEB lo
suelta y el PID arranca limpio. Medido en Gazebo: si se seguía mandando la ω del PID, el AEB
la dejaba pasar (robot quieto) y el robot giraba en el sitio frente a la caja. Se probó
también un "escape" (girar en el sitio tras unos segundos para no quedar detenido 10 s): frente
a una caja terminaba mirando hacia atrás y recorriendo la pista al revés, y en 14 carreras
limpias nunca hizo falta, así que se quitó (ver BITACORA).

## 6. Resultados del simulador 2D (punto de partida, NO son resultados de Gazebo)

`tools/sim2d.py`: cinemática + límites del enunciado + LiDAR de 450 rayos a 10 Hz con ruido +
la misma lógica de `nucleo.py`. `k_giro` = fracción del giro comandado que se logra (aproxima
el deslizamiento del skid-steer).

| Pista | k_giro | Vueltas [s] | Total [s] | Holgura mín. centro–muro | Frenadas AEB |
|---|---|---|---|---|---|
| competencia | 0.55 | 40.2 / 40.1 / 40.1 | 120.5 | 0.169 m | 0 |
| competencia | 0.70 | 39.4 / 39.3 / 39.3 | 117.9 | 0.178 m | 0 |
| competencia | 1.00 | 38.4 / 38.4 / 38.3 | 115.1 | 0.165 m | 0 |
| propia | 0.55 | 33.1 / 33.0 / 33.0 | 99.1 | 0.162 m | 0 |
| propia | 0.70 | 31.9 / 31.8 / 31.8 | 95.4 | 0.159 m | 0 |
| propia | 1.00 | 30.6 / 30.6 / 30.6 | 91.7 | 0.156 m | 0 |

(El robot toca un muro de lado con holgura < 0.077 m.) Otros hallazgos del simulador:
- `aeb_modo:=ittc` a 1.0 m/s: el robot no arranca (la pared lateral a 0.20 m da TTC < τ).
  Es la demostración de por qué hace falta el modo corredor.
- Obstáculo de 0.30 m en la recta: el seguidor baja a 0.35 m/s al verlo y el AEB lo detiene
  con ≈ 0.12 m libres.
- Kp = 4: 6–14 frenadas falsas por vuelta. θ = 70° con l = 0.5: el robot se pega a la esquina
  interior. No usar.

## 7. Supuestos SIN verificar (revisar en la Fase 1, en este orden)

| # | Supuesto | Cómo verificarlo | Si falla |
|---|---|---|---|
| 1 | El DiffDrive de gz-sim 8 acepta varias `<left_joint>`/`<right_joint>` y las etiquetas `max/min_linear_velocity`, `max/min_angular_velocity`, `max/min_linear_acceleration` | Pedir 2.0 m/s y 8 rad/s por `/cmd_vel_teleop`; leer `/odom_real`: no debe pasar de 1.0 y ~4.0. Medir la aceleración al arrancar (≤ 2.0) | Buscar los nombres reales: `find /opt/ros/jazzy -name "DiffDrive.hh"` y leer el comentario de parámetros |
| 2 | `OdometryPublisher` publica en `/odom_real` la pose en coordenadas del MUNDO y el TF `mundo → base_footprint` en `/tf` | Recién lanzado: `ros2 topic echo /odom_real --once` → x ≈ −0.60, y ≈ 0. `ros2 run tf2_ros tf2_echo mundo base_footprint` | Si arranca en (0,0): usar `<xyz_offset>` o sumar el spawn en los nodos. Si el plugin no carga: `find /opt/ros/jazzy -name "OdometryPublisher.hh"`; alternativa: plugin PosePublisher |
| 3 | Los nombres de topic relativos del URDF (`cmd_vel`, `odom`, `odom_real`, `tf`, `joint_states`) quedan en Gazebo como `/cmd_vel`, `/odom`, … | `gz topic -l` con la simulación corriendo | Ajustar los nombres en el bridge del launch (con remapping) |
| 4 | `/joint_states` trae las 4 juntas con velocidad y sello de tiempo de simulación | `ros2 topic echo /joint_states --once` | Revisar el plugin JointStatePublisher; el nodo ya avisa si faltan juntas |
| 5 | El LiDAR entrega 450 rayos, `angle_min ≈ −π`, `range_min = 0.03`, con ruido, a 10 Hz, y NO se ve a sí mismo | `ros2 topic echo /scan --once` (mirar `angle_min`, `angle_increment`, cantidad de `ranges`), `ros2 topic hz /scan` | Ajustar el sensor; el código calcula índices con `angle_min`/`angle_increment`, no los asume |
| 6 | Las mallas STL colisionan (robot contra muro) y el `gpu_lidar` las ve por ambos lados | Con `ros2 param set /aeb_node habilitado false`, empujar el robot contra un muro en teleop a baja velocidad: debe detenerse físicamente. Ver los puntos del LiDAR sobre los muros en RViz | Revisar `model.sdf`; como último recurso, colisión con cajas |
| 7 | El skid-steer gira bien con mu1 = 1.0, mu2 = 0.6 | Girar en el sitio pidiendo 2 rad/s y medir el giro real en `/odom_real` (k_giro = real/pedido). Buscar k_giro ≥ 0.5 y que no vibre | Bajar mu2 (0.3–0.5); probar `<fdir1>`; revisar que el chasis no roce el piso |
| 8 | La ventana de teleop abre con xterm/gnome-terminal/terminator | Lanzar `modo:=teleop` | Ajustar `terminal_para_teleop()` en el launch; siempre queda el plan B de la segunda terminal (el launch lo imprime) |
| 9 | `config/competencia.rviz` (mínimo, escrito a mano) carga bien | Mirar RViz: robot, LiDAR, dos trayectorias, cámara | Ajustar en RViz y guardar con Ctrl+S |
| 10 | `add_post_set_parameters_callback` existe en rclpy Jazzy y `ros2 param set /wall_follow kp 10.0` re-crea el seguidor | Probar el comando (con punto decimal: `10.0`, no `10`) | Cambiar a `add_on_set_parameters_callback` |
| 11 | `ros_gz_sim create` acepta `-Y` para el yaw y el robot cae bien desde z = 0.05 | Ver el spawn | Ajustar argumentos |
| 12 | El PC mantiene tiempo real (RTF ≈ 1) con `gpu_lidar` + cámara | Mirar el RTF abajo a la derecha en Gazebo | Bajar la cámara a 10 fps o 320×240; `rviz:=false`. La lógica usa tiempo de simulación, así que un RTF bajo no cambia el resultado, solo lo hace más lento |

Las referencias del `README.md` del paquete (sección 7) hay que confirmarlas antes del pitch:
las del profesor están en `material_profe/`; el enlace de F1TENTH y los datos de Yahboom se
citaron de memoria y Matteo debe abrirlos y comprobar que dicen lo que se les atribuye.
