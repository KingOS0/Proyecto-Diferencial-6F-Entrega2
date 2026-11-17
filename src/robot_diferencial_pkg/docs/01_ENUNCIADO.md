# 01 — Enunciado (resumen fiel)

Fuente: `material_profe/Reporte_Robot_Diferencial_Competencia-1.pdf` (texto completo en el
`.txt` del mismo nombre). Reporte "Robot diferencial: AEB, seguimiento de pared y
competencia", Manuel José Ortiz Camacho, Robótica 2026-2, Universidad EIA.
Si algo de este resumen choca con el PDF, manda el PDF.

## Condiciones
- Parte del paquete del reporte anterior (robot diferencial en Gazebo con LiDAR, cámara, nodo
  de cinemática y teleoperación). Lo ya evaluado no se recalifica, **pero debe seguir funcionando**.
- No se permite contenido copiado y pegado sin referencia (anula la entrega).
- Cada decisión de diseño se justifica con criterios de ingeniería y referencias.
- El modo autónomo debe ser **reactivo**: sin mapas, waypoints ni trayectorias pregrabadas de
  la pista de competencia. No se evalúa Nav2 ni SLAM.
- Fecha que figura en el enunciado para pitch y competencia: martes 3 de noviembre de 2026
  (Matteo la presenta después; la fecha real la coordina él con el profesor).

## 3.1 Robot y límites (se configuran en el plugin DiffDrive y en el sensor)

| Parámetro | Valor |
|---|---|
| Dimensiones (L × A × H) | 198.6 × 153 × 124.2 mm |
| Masa | 0.906 kg |
| Diámetro de rueda | ≈ 49 mm |
| Vía (centro a centro) | ≈ 134 mm |
| Distancia entre ejes | ≈ 96 mm |
| Motores | 450 rpm, 4 kg·cm |
| Velocidad lineal máx. | 1.0 m/s |
| Velocidad angular máx. | 4.0 rad/s |
| Aceleración lineal máx. | 2.0 m/s² |
| LiDAR: FOV / muestras | 360° / 450 |
| LiDAR: rango / frecuencia | 0.03–12 m / 10 Hz |
| LiDAR: ruido gaussiano | σ = 0.01 m |
| LiDAR: altura de montaje | ≈ 108 mm |

Los ≈ se estimaron del plano del fabricante y deben verificarse midiendo el robot. La velocidad
máxima se fijó por debajo de la de vacío (450 rpm · π · 0.049 m / 60 ≈ 1.15 m/s).

## 3.2 Requisitos
1. **Adaptación al MicroROS (Yahboom):** modificar el URDF/Xacro con las medidas de la tabla;
   pasar de dos ruedas y caster a **cuatro ruedas motrices skid-steer (dos juntas por lado en
   el plugin DiffDrive)**; LiDAR como el MS200; límites de velocidad y aceleración.
2. **Odometría en varias vueltas:** con el nodo de cinemática del reporte anterior, tomando
   ω_D y ω_I como el **promedio de las dos ruedas de cada lado**, integrar la pose 3 vueltas y
   presentar la trayectoria estimada y el **error de cierre en cada vuelta** (pose estimada al
   cruzar la línea de salida/meta frente a la pose real).
3. **AEB:** nodo que para cada rayo i calcule `ṙ_i = −v cos θ_i`, `TTC_i = r_i / max(−ṙ_i, 0)`
   y detenga el robot cuando `min_i TTC_i` sea menor que un **umbral justificado**. Debe
   funcionar en ambos modos, detener el robot frente a un obstáculo en la pista y al dirigirlo
   hacia un muro en teleop, **evitando frenados falsos en las curvas y en el estrechamiento**.
4. **Seguimiento de pared:** PID sobre la distancia a una pared con **al menos dos rayos**.
   Con b perpendicular a la pared y a a un ángulo θ de b:
   `α = arctan((a cos θ − b)/(a sin θ))`, `D_t = b cos α`, `D_{t+1} = D_t + l sin α`,
   error `e = D_des − D_{t+1}` (l = distancia de anticipación). La salida del PID es ω, y **v
   debe ajustarse según la curvatura**. Ganancias, D_des, l y θ deben justificarse.
5. **Pista propia:** circuito cerrado **diseñado en CAD** con 5 cambios de sección (recta, curva
   amplia, curva cerrada, chicana, estrechamiento, horquilla…), exportado como **STL en metros**
   e importado en Gazebo como **mesh con colisión**. Ancho mínimo 0.40 m, muros interior y
   exterior ≥ 0.25 m de alto, línea de salida/meta definida. Reemplaza la pista anterior.
6. **Modo autónomo:** 3 vueltas a la pista propia con seguimiento de pared y sin colisiones.
   Tiempo de cada vuelta y total en tiempo de simulación (`/clock`), presentados en el pitch.
7. **Integración:** extender el launch anterior con argumentos `modo:=teleop|auto` y
   `pista:=propia|competencia`. Teleop y autonomía publican en **topics separados** y un nodo
   (propio o twist_mux) los lleva a `/cmd_vel` **pasando por el AEB**.

## 4. Competencia
- Pista del profesor: circuito cerrado, 6 cambios de sección, **sentido antihorario**, ancho
  nominal 0.80 m, mínimo 0.40 m, muros de 0.25 m, 37.2 m por vuelta (3 vueltas = 111.7 m).
  Secciones: S1 recta principal 9.0 m (salida/meta); S2 curva amplia 180° R=2.5; S3 chicana
  (arcos R=1.6: +40°, −80°, +40°); S4 estrechamiento a 0.40 m (2.5 m); S5 curva cerrada 90°
  R=1.0; S6 curva media 90° R=1.8.
- Robot en **x = −0.60, y = 0, ψ = 0**, detrás de la línea de salida/meta (**x = 0**).
- Categorías: autónoma y teleoperada, cada una con su tabla de tiempos.
- Tiempo en tiempo de simulación, desde que cruza la línea de salida hasta que la cruza por
  tercera vez (3 vueltas).
- Todas las corridas en el mismo computador, dispuesto por el profesor.
- Dos intentos por categoría; cuenta el mejor. **Cada contacto con un muro suma 3 s.**
- El intento se anula si el robot sale de la pista, **queda detenido más de 10 s** o se
  intervienen los comandos en la categoría autónoma.
- El AEB debe estar activo en ambas categorías. Antes de la carrera se verifica con **un
  obstáculo en la recta principal: si el robot no se detiene, no compite en la autónoma**.
- Se verifican los límites del robot antes de cada corrida.
- Bonificación por categoría: 1.er puesto +0.5, 2.º +0.3, 3.º +0.2 (máximo 5.0).

## 5. Evaluación (sobre 100, escalada a 5.0)

| Criterio | Pts | Qué se espera |
|---|---|---|
| Adaptación al MicroROS | 5 | Medidas, skid-steer y límites de la tabla, cargando sin errores |
| Odometría en varias vueltas | 10 | Trayectoria de 3 vueltas, error de cierre por vuelta y análisis de su crecimiento |
| AEB | 20 | TTC correcto, umbral justificado, prioridad en ambos modos, sin frenados falsos, detención efectiva frente a obstáculo y muro |
| Seguimiento de pared | 20 | Estable, sin oscilaciones excesivas ni colisiones, ganancias y parámetros justificados, v ajustada a la curvatura, 3 vueltas a la pista propia |
| Pista propia | 10 | Diseño propio con 5 cambios de sección, STL en escala correcta, colisiones funcionales, línea de meta |
| Integración | 5 | Un solo launch desde terminal limpia, ambos modos y ambas pistas, AEB en la cadena. **En vivo** |
| Competencia | 10 | 3 vueltas a la pista de competencia: autónoma (6) y teleoperada (4), con un intento válido |
| Pitch | 20 | **Todos los integrantes** responden sobre el cálculo del TTC y su umbral, la sintonización del PID y el crecimiento del error de odometría |

## 6. Presentación
Sin entrega escrita, Notion ni Moodle. 8 min de presentación (adaptación al MicroROS, AEB,
controlador, pista propia, tiempos y errores de cierre) + 8 min de preguntas + corridas
(prueba de AEB y las dos categorías). El paquete se lleva listo para correr en el computador
de la competencia.

## Material de clase relacionado (en `material_profe/`)
- `lidar_aeb.txt`: LaserScan, índice↔ángulo, ṙ = −v cos θ, iTTC, umbral dinámico
  τ = max(τ0, t_lat + v/2a), **modo corredor** contra falsos positivos, arquitectura del AEB
  como compuerta, lógica de decisión, preguntas de sustentación.
- `gap_wall.txt`: wall following de dos haces, control PD, v según ω, índices, AEB como capa
  de seguridad, preguntas guía. (Follow the Gap no se pide en este reporte.)
- `codigo_clase/ros2_ws_dif_C2_src/`: workspace de clase (launch, bridge, nodos de control).
- `otros_temas/`: control cinemático, pure pursuit y búsqueda (A*, Dijkstra). **No son de esta
  entrega** (y usar planificación sobre la pista está prohibido); están por contexto.
