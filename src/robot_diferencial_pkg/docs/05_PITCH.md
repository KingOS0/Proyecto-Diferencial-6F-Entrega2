# 05 — Pitch (8 min) y preguntas (8 min)

El pitch vale 20 puntos y **todos los integrantes** responden sobre: (1) el cálculo del TTC y
su umbral, (2) la sintonización del PID, (3) el crecimiento del error de odometría.
Los `____` se llenan con lo medido en Gazebo (Fases 2–5). No presentar números del simulador
2D como si fueran de Gazebo.

## Guion sugerido (8 min)
1. (0:45) **Adaptación al MicroROS.** Tabla del enunciado → URDF: 4 ruedas skid-steer (dos
   juntas por lado en DiffDrive), medidas, masa repartida, MS200 (450 rayos, 0.03–12 m, 10 Hz,
   σ = 0.01), límites 1.0 m/s, 4.0 rad/s, 2.0 m/s². Decir qué valores son supuestos del equipo.
2. (1:00) **Arquitectura.** Diagrama: teleop y auto en topics separados → mux → AEB →
   `/cmd_vel`. El AEB es el único que publica en `/cmd_vel`. Mostrar rqt_graph.
3. (1:45) **AEB.** Fórmula por rayo, umbral dinámico τ(v) = max(τ0, t_lat + v/2a) con sus
   valores, descuento de carrocería, modo corredor. Evidencia: tabla velocidad → distancia
   final; frenada falsa con `ittc` vs ninguna con `corredor` en el estrechamiento.
4. (1:45) **Seguimiento de pared.** Geometría de dos rayos, lookahead, PID, v según ω, tercer
   rayo de anticipación. Por qué D_des = 0.20. Cómo se sintonizó. Gráfica de e(t).
5. (1:00) **Pista propia.** Plano con las secciones y cotas; STL en metros; mesh con colisión;
   línea de meta. Tiempos de 3 vueltas.
6. (1:15) **Odometría.** Trayectoria real vs estimada, error de cierre por vuelta, por qué crece.
7. (0:30) **Competencia.** Tiempos en la pista del profe, autónoma y teleop.

## Pista propia en CAD (cómo se hizo, para contarlo tal cual)
1. **Trazado paramétrico** (`tools/generar_pista.py`): línea central de rectas y arcos con
   sus radios y el perfil de ancho (0.80 m, estrechamiento a 0.40 m con rampas). Sale el DXF
   con los contornos de los muros y el plano acotado (`plano_pista_propia.png`).
2. **Modelo CAD en FreeCAD** (`tools/pista_cad_freecad.py`, en mm): contornos del DXF →
   planta de cada muro → extrusión de 300 mm → sólido. Archivo
   `models/pista_propia/cad/pista_propia.FCStd`: se abre en FreeCAD y se ve el árbol
   (Planta_Muro_interior → Muro_interior → Pista_propia).
3. **Exportación a STL en metros** (escala 0.001 en la exportación, porque el CAD trabaja en
   mm) → `meshes/pista_propia.stl`, que Gazebo carga como mesh con colisión.
Si cambian la altura del muro o un radio: cambiar en el CAD/script, regenerar y listo.
Para mostrarlo en vivo: `freecad models/pista_propia/cad/pista_propia.FCStd`.

## Preguntas que van a caer (con la respuesta corta que hay que saber decir)

### TTC y umbral
1. **¿Por qué ω no aparece en ṙ = −v cos θ?** Girar mueve el punto de lado respecto al rayo
   (tangencial), no cambia su distancia. Solo la componente de v sobre el rayo acerca el obstáculo.
2. **¿Qué pasa con los rayos que no se acercan?** −ṙ ≤ 0 → TTC infinito (por eso el max(−ṙ, 0)).
   Con v > 0 solo cuentan los rayos de adelante; con v < 0, los de atrás.
3. **¿Por qué el umbral no es fijo?** La distancia de frenado es v·t_lat + v²/(2a): crece con
   v². Dividiendo por v: τ_min = t_lat + v/(2a). Un umbral fijo que sirve a 0.3 m/s no alcanza
   a 1.0 m/s. A 1.0 m/s: τ = 0.45 s, empieza a frenar cuando faltan 0.45 m (+ 0.05 m de colchón)
   hasta el parachoques. Medido en Gazebo: se detuvo a 0.29 m del muro (0.22 m a 0.6 m/s, 0.19 m a 0.3).
4. **¿De dónde salen t_lat y a?** a = 2.0 m/s² es el límite del robot (medido: frena a 2.0 m/s²); t_lat = 0.20 s,
   con margen sobre lo medido: ≤ 0.14 s (0.04 s del comando al actuador + hasta 0.10 s de edad
   del barrido, porque el LiDAR va a 10 Hz).
5. **¿Qué pasa si no se descuenta la distancia del LiDAR al parachoques?** El TTC se calcula
   hasta el centro del robot: frena ≈ 0.10 m tarde, o sea ya chocó.
6. **¿Por qué el iTTC frena en el estrechamiento y el corredor no?** El iTTC trata cada punto
   como si el robot fuera a chocar con él. Una pared lateral a 0.20 m vista a 45° da
   TTC ≈ 0.2/(sin45·cos45·v) = 0.4/v: a 1 m/s es 0.4 s, menos que el umbral. El modo corredor
   solo cuenta lo que está dentro de la franja que barre el robot.
7. **¿Qué significa `inf` en el LiDAR?** Que no hay nada dentro del rango, no un obstáculo. Se filtra.
8. **¿Cuándo es peligroso dejar pasar el giro mientras frena?** A velocidad, girar puede meter
   una esquina del robot contra el muro. Por eso el giro solo pasa cuando ya está casi quieto.
9. **¿Cómo tiene prioridad el AEB?** Por arquitectura: es el único nodo que publica en `/cmd_vel`.

### PID de seguimiento de pared
10. **¿Por qué no basta un rayo a 90°?** Da la distancia pero no el ángulo: no distingue si el
    robot va paralelo, acercándose o alejándose. Con dos rayos sale α.
11. **¿Qué hace l (anticipación)?** Proyecta la distancia hacia adelante: D_{t+1} = D_t + l sin α.
    Actúa como derivativo: reacciona al ángulo antes de que cambie la distancia. l grande
    suaviza pero recorta curvas; l pequeño reacciona tarde y oscila.
12. **¿Cómo se sintonizó?** Modelo: d̈ + (Kp·l + Kd·v)·ḋ + Kp·v·d = 0 → ω_n = √(Kp·v),
    ζ = (Kp·l + Kd·v)/(2√(Kp·v)). Kp fija la rapidez; l y Kd el amortiguamiento. Punto de
    partida en un simulador 2D propio y ajuste final en Gazebo con el método de la clase.
    Valores finales: Kp = 12, Kd = 1.0, l = 0.35 m → ω_n = 3.5 rad/s, ζ = 0.75 a v_max = 1.0 m/s.
    En Gazebo no hubo que resintonizar: 0 oscilación sostenida, 0 frenadas, 0 toques.
13. **¿Por qué D_des = 0.20 m?** Es la mitad del ancho mínimo (0.40 m): el robot pasa centrado
    por el estrechamiento sin cambiar de consigna, y en el carril ancho toma la línea interior.
14. **¿Por qué la pared izquierda?** Sentido antihorario → la pared interior queda a la
    izquierda y es la referencia más corta y continua.
15. **¿Qué pasa con α y D_t al entrar a una curva de la pared interior?** La pared se aleja: el
    rayo a crece, α > 0, D_{t+1} > D_des → e < 0 → gira a la izquierda, hacia la pared.
16. **¿Cómo se ajusta v a la curvatura?** v = v_max − (v_max − v_min)·min(|ω|/w_ref, 1), y
    además baja si hay muro cerca al frente.
17. **¿Por qué Ki = 0?** No hay perturbación constante que corregir; el integral agregaría
    retardo y riesgo de windup. (Si se usó Ki, decir cuánto y por qué.)
18. **¿Para qué el tercer rayo?** Mira 20° a la izquierda del frente: si la pared se cierra
    adelante (curva a la derecha, entrada del estrechamiento), c·sin γ < D_t y el robot empieza
    a girar ≈ 0.5 m antes.

### Odometría
19. **¿Cómo se calcula?** ω_D y ω_I = promedio de las dos ruedas de cada lado; u = r(ω_D+ω_I)/2,
    ψ̇ = r(ω_D−ω_I)/b; se integra con Euler.
20. **¿Por qué crece el error?** Es integración sin corrección: cada error se queda. En
    skid-steer las ruedas patinan de lado al girar, el robot gira menos de lo que dice el modelo
    y el rumbo estimado se va corriendo. Como la posición integra el rumbo, un error de rumbo
    pequeño se vuelve un error de posición que crece con la distancia.
21. **¿Cuánto fue?** (pista propia, b = 0.134 m) vuelta 1: 2.37 m, 55°; vuelta 2: 4.19 m, 109°;
    vuelta 3: 5.07 m, 164°. El rumbo se corre ≈ 55° por vuelta = 15 % de 360°, que es lo que
    predice k_giro = 0.876 (1/0.876 − 1 = 14 %). Con la vía calibrada b_ef = 0.153 m:
    0.25 m / 3.5°, 0.41 m / 6.3°, 0.56 m / 9.4° (sigue creciendo: el error no desaparece, se
    reduce la parte sistemática; queda el deslizamiento que varía con la curva).
22. **¿Cómo se mejoraría?** Calibrar la vía efectiva, IMU para el rumbo, o corregir con el LiDAR
    (fuera del alcance: no se evalúa SLAM).
23. **¿Contra qué se compara?** Contra la pose real de Gazebo (plugin OdometryPublisher), no
    contra `/odom` del DiffDrive, que también es odometría de ruedas.

### Generales
24. **¿Por qué es reactivo?** `wall_follow` decide solo con el último `/scan`: no guarda mapa,
    waypoints ni la forma de la pista.
25. **¿Qué pasa si el AEB lo frena en carrera?** El seguidor ve `/aeb/activo` y deja de mandar
    giro: se queda quieto pidiendo avanzar despacio; cuando quitan el obstáculo, sigue. (Se
    probó girar para "escapar" y frente a una caja terminaba recorriendo la pista al revés.)
