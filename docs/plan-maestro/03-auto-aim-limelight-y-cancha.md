# 03 — Auto-aim, Limelight y cancha

> Estado: investigación y diseño inicial; medidas y transforms pendientes de commissioning
> Baseline de referencia: `origin/main@b5a134260456565df9d0295722ebecad900f21b4`
> Última actualización: 2026-07-15
> Alcance: fuentes oficiales, geometría, Limelight 3A, fusión, bearing y modelo de RPM
> Responsable sugerido: responsable de localización/visión con revisión de software y mecánica
> Fuente de verdad: manual/CAD oficial de la temporada y mediciones del robot; ninguna coordenada de ejemplo debe copiarse sin validar el marco.

## 1. Pregunta técnica

¿Cómo lograr que una torreta de arco limitado apunte al goal correcto durante TeleOp sin depender exclusivamente de Limelight?

Respuesta de diseño:

1. tres dead wheels + IMU propagan continuamente la pose del robot;
2. un mapa oficial proporciona la posición del goal/tag por alianza;
3. la geometría calcula el bearing desde robot al punto de tiro;
4. Limelight fija al chasis aporta observaciones con timestamp para corregir la pose y, opcionalmente, un residual angular pequeño al tag correcto;
5. gates rechazan observaciones viejas, ambiguas o incompatibles;
6. la torreta controla el ángulo sólo si está armada, la solución es utilizable y el setpoint cabe en el arco;
7. la distancia geométrica alimenta un modelo empírico de RPM de ángulo fijo;
8. un interlock central decide si el feeder puede actuar.

## 2. Fuentes primarias

| Fuente | Uso en este plan | Qué no demuestra |
|---|---|---|
| [DECODE Competition Manual TU32](https://ftc-resources.firstinspires.org/ftc/archive/2026/game/cm-html/DECODE_Competition_Manual_TU32.htm) | Dimensiones nominales de cancha/goal y IDs/tamaño de AprilTags | La posición real de la cámara, punto balístico de aim o tolerancias de nuestro robot. |
| [Recursos oficiales de cancha 2025–2026](https://ftc-resources.firstinspires.org/ftc/archive/2026/field) | CAD, dibujos y assets para construir mapa/medir anclas | Que el montaje de evento sea perfecto; las medidas siguen siendo nominales. |
| [Game & Season Materials](https://ftc-resources.firstinspires.org/ftc/game) | Punto de entrada a revisiones vigentes del juego | Que una copia archivada siga siendo la revisión más nueva el día del evento. |
| [Ejemplo oficial `SensorLimelight3A`](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/4ed7c4666aec265a6fd9e674ca40462e9dfe4bf8/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorLimelight3A.java) | Lifecycle/API básica, mapping `limelight`, pipeline, validity y resultados | Arquitectura de fusión, seguridad o configuración exacta del robot. |
| [Javadoc FTC SDK 11.0 `Limelight3A`](https://javadoc.io/static/org.firstinspires.ftc/Hardware/11.0.0/com/qualcomm/hardware/limelightvision/Limelight3A.html) | API de `start`, `pause`, `stop`, polling, pipeline, latest result, orientation y fieldmap | Qué API resuelve realmente un checkout distinto; confirmar Gradle. |
| [FTC Docs: external webcams](https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/configuring_external_webcam/configuring-external-webcam.html) | Limelight 3A no es una cámara VisionPortal y consideraciones de coexistencia/alimentación | Que sea necesario mantener la webcam vieja. |
| [FTC Docs: AprilTag reference frame](https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_reference_frame/apriltag-reference-frame.html) | Convenciones de poses/bearing en el stack FTC | Convenciones de Pedro o del field map de Limelight. |
| [Limelight documentation](https://docs.limelightvision.io/) | Configuración, pipelines, MegaTag/field localization y diagnóstico | Reglas FTC o seguridad del mecanismo. |
| [Limelight Field Map Builder](https://tools.limelightvision.io/map-builder) | Crear/verificar field map versionado | Que un mapa cargado esté alineado físicamente con la cancha. |

Antes de una competencia se debe revisar la revisión vigente del manual, no congelar TU32 para siempre. La tabla anterior registra fuentes de investigación, no autoriza usar coordenadas o APIs sin fijar revisión, fecha y versión efectiva.

## 3. Hechos de cancha útiles

Según el manual oficial investigado:

- la cancha es aproximadamente 144 in × 144 in y usa 36 tiles de 24 in;
- el goal es aproximadamente 27 in × 27 in × 54 in;
- su abertura se describe nominalmente como 26.5 in de ancho y 18.3 in de profundidad, con el borde superior a 38.75 in;
- las medidas de campo son nominales y pueden tener tolerancia aproximada de ±1 in;
- el lado negro del AprilTag de goal mide 6.5 in;
- el goal azul usa tag ID 20 y el rojo tag ID 24;
- los tags 21, 22 y 23 pertenecen al Obelisk y no deben tratarse como goals ni como anclas equivalentes sin una política explícita.

La librería oficial incluida en el AAR de FTC SDK 11 inspeccionado durante la investigación contiene poses nominales para los tags de goal cercanas a:

| ID | Nombre observado | X SDK (in) | Y SDK (in) | Z SDK (in) | Tamaño negro (in) |
|---:|---|---:|---:|---:|---:|
| 20 | BlueTarget | -58.3727 | -55.6425 | 29.5 | 6.5 |
| 24 | RedTarget | -58.3727 | +55.6425 | 29.5 | 6.5 |

Estas coordenadas son un punto de partida de verificación, no constantes listas para pegar. Deben comprobarse contra el AAR realmente resuelto, el CAD oficial, la orientación del tag y el marco documentado por el SDK. Además, el punto de tiro efectivo del goal puede requerir un offset respecto al centro del tag.

## 4. Contrato de marcos de coordenadas

Antes de escribir una ecuación de aim, crear una hoja/fixture de verificación con:

| Marco | Origen | +X | +Y | Heading 0 | Sentido positivo | Unidad |
|---|---|---|---|---|---|---|
| Oficial/SDK | Pendiente de confirmar con documentación/AAR | Pendiente | Pendiente | Pendiente | Pendiente | in/rad |
| Pedro | Configuración seleccionada | Pendiente | Pendiente | Pendiente | Pendiente | in/rad |
| Robot/chasis | Centro de referencia acordado | Frente | Izquierda o convención seleccionada | Frente | CCW recomendado | in/rad |
| Cámara | Centro óptico | Según Limelight | Según Limelight | Eje óptico | Según docs | m/in y grados según API |
| Torreta | Eje mecánico | Frente en cero | Lateral según signo | Marca de cero | Medido | ticks/rad |

No llenar esta tabla “de memoria”. El deliverable de MP-04 es una tabla completa con dibujos y pruebas.

### 4.1 Transform necesaria

El estimador necesita, como mínimo:

```text
T_field_robot
T_robot_camera   (extrínseca fija medida)
T_robot_turret   (posición del eje y cero)
T_field_goal     (mapa por alianza)
```

Una pose de cámara reportada por Limelight debe convertirse a pose de robot aplicando la inversa de la extrínseca correspondiente. Si la API ya acepta pose/orientación del robot, documentar exactamente qué transform aplica el firmware para no duplicarla.

### 4.2 Prueba de cuatro anclas

Validar transforms colocando el robot en cuatro poses conocidas, idealmente cerca de esquinas/cuadrantes y con headings distintos:

1. registrar pose física, Pedro y Limelight;
2. convertir todo al marco canónico;
3. comparar signo de X/Y, heading y magnitud;
4. repetir para BLUE y RED;
5. rechazar la integración si una simetría “parece” arreglarse con un signo mágico sin explicación.

También se debe girar el robot en el mismo punto para separar error de extrínseca de error de traslación.

## 5. Montaje y configuración de Limelight

El equipo reporta Limelight instalada y webcam retirada. Esta observación física no confirma el string `limelight`, firmware, pipeline, red, orientación ni extrínseca; todos permanecen `TBD-BLOCKING` hasta completar la guía 08. No se planea coexistencia con webcam/VisionPortal.

### 5.1 Requisitos físicos

- montaje rígido al chasis, no a la torreta;
- vista suficiente de goal tags dentro de posiciones de tiro previstas;
- cableado/alimentación que no se afloje al girar la torreta;
- posición X/Y/Z y yaw/pitch/roll medidas;
- protección contra impactos y reflejos sin obstruir ventilación;
- nombre RC confirmado; `limelight` es sólo la propuesta compatible con el sample oficial.

### 5.2 Lifecycle esperado

```text
map device
  -> configure poll rate/orientation if supported
  -> select known pipeline
  -> start
  -> poll non-blocking in periodic
  -> validate/copy latest result
  -> mark stale on timeout
  -> stop + close at OpMode stop
```

No usar loops bloqueantes esperando un target. Si `start()` o `pipelineSwitch()` falla, el subsystem publica fault y el robot conserva la ruta odométrica/manual permitida.

### 5.3 Assets versionados

Crear, en una fase futura, una carpeta como `TeamCode/src/main/assets/limelight/` o `docs/config/limelight/` según la forma real de exportación, con:

- nombre/versión del pipeline;
- field map exportable;
- captura/lista de parámetros relevantes;
- pose de cámara y convención;
- versión de firmware Limelight y SDK;
- fecha, autor y SHA con que fue validado;
- procedimiento de cargar/verificar en un dispositivo nuevo.

No copiar un pipeline de otro equipo sin entender exposición, lente, tag family y mapa.

## 6. Modelo de observación y gates

Cada frame se reduce a una `VisionObservation`. Una observación sólo se considera para fusión cuando cumple todos los gates aplicables.

### 6.1 Gates obligatorios

| Gate | Pregunta | Rechazo típico |
|---|---|---|
| Validity | ¿El SDK marca el resultado válido y contiene los campos usados? | Resultado nulo/inválido. |
| Freshness | ¿La edad total está debajo del máximo medido? | Frame viejo o repetido. |
| Alliance/ID | ¿Contiene goal 20 para blue o 24 para red cuando se usa bearing de goal? | Tag del Obelisk/otra alianza. |
| Geometry | ¿Pose/distancia/bearing son físicamente posibles? | Z absurda, robot fuera de campo, bearing incompatible. |
| Innovation | ¿Residual visión−odometría está dentro de gate dependiente de calidad? | Salto grande sin relocalización deliberada. |
| Motion/latency | ¿Velocidad y latencia permiten compensación confiable? | Giro rápido con frame atrasado. |
| Ambiguity/quality | ¿Área/tag count/metric seleccionada superan mínimo? | Target diminuto o ambiguo. |
| Extrinsic/config | ¿Pipeline/map/camera pose coinciden con la versión esperada? | Configuración desconocida. |

Todos los rechazos deben incrementar contadores por razón. “No usé visión” sin razón no es diagnosticable.

### 6.2 Edad y compensación

Definir:

```text
observationAgeMs = now - captureTimestamp
```

Si sólo se conoce receive time + latency reportada, documentar la aproximación. Para compensar movimiento, conservar un buffer corto de poses odométricas y comparar/aplicar la observación en su timestamp, después propagar al presente. Si el stack no soporta esto inicialmente, usar gates conservadores de velocidad/edad y medir el error.

## 7. Fusión recomendada por etapas

### Etapa A — Sólo logging

Ejecutar Pedro como pose autoritativa y Limelight en shadow mode. Registrar ambas sin corregir nada. Esto revela offsets, signos, latencia y outliers sin riesgo de mover la torreta por visión.

### Etapa B — Corrección complementaria acotada

Para observación aceptada:

```text
innovation = visionPose - odometryPoseAtCapture
boundedInnovation = clampByAxisAndHeading(innovation)
correctedPose = currentOdometryPose + alpha * boundedInnovation
```

`alpha` puede depender de calidad, cantidad de tags, velocidad y residual. Debe ser pequeño inicialmente. Heading se interpola con wrap angular correcto.

No usar un único alpha sin registrar sus unidades/efectos. Una corrección distribuida en varios loops puede evitar saltos, pero debe expirar si deja de ser válida.

### Etapa C — Trim angular de goal

Cuando el tag correcto es fresco y la geometría coincide:

```text
visionResidual = measuredGoalBearing - predictedGoalBearing
visionTrim = clamp(beta * visionResidual, -maxTrim, +maxTrim)
```

Aplicar low-pass/rate limit y retirar el trim gradualmente al perder visión. No mantenerlo indefinidamente. El signo se valida con el robot inmóvil apuntando a ambos lados del target.

### Etapa D — Filtro más complejo sólo si hace falta

Un EKF o factor estimator requiere modelos de ruido, timestamping y pruebas adicionales. Adoptarlo sólo si A–C no cumplen el gate de pose/aim y los logs identifican por qué. Más matemáticas sin caracterización de sensores no crean robustez.

## 8. Cálculo geométrico de aim

Con pose canónica `(x_r, y_r, theta_r)` y punto de aim `(x_g, y_g)`:

```text
dx = x_g - x_r
dy = y_g - y_r
distance = hypot(dx, dy)
fieldBearing = atan2(dy, dx)
robotRelativeBearing = wrap(fieldBearing - theta_r)
turretTarget = wrap(robotRelativeBearing - turretZeroOffset + aimBias)
```

Luego:

1. agregar trim visual gated;
2. buscar la representación equivalente que esté dentro del arco permitido;
3. aplicar margen a los soft limits;
4. si no existe representación segura, declarar `AT_LIMIT` y pedir giro manual del chasis;
5. calcular distancia desde el punto físico relevante del shooter/torreta, no necesariamente desde el centro del robot.

Si la trayectoria del proyectil requiere apuntar a un punto distinto según distancia, eso se modela como `aimBias(distance)` pequeño y empírico, no cambiando arbitrariamente la pose del tag.

## 9. Goal por alianza

| Alianza | Input init | Goal tag | Selección |
|---|---|---:|---|
| Blue | `gamepad1 X` | 20 | Directa, mostrada y locked en start. |
| Red | `gamepad1 B` | 24 | Directa, mostrada y locked en start. |

Los tags 21–23 pueden servir a otras funciones sólo si se añade un caso documentado. Nunca deben cambiar el goal seleccionado por “primer detection”.

## 10. RPM para shooter de ángulo fijo

### 10.1 Dataset mínimo

Para cada intento guardar:

- fecha, SHA, build/configuración;
- alianza y posición física;
- distancia horizontal al punto de aim;
- RPM target y RPM medida al alimentar;
- voltaje de batería;
- trim del operador;
- temperatura aproximada/orden de la serie;
- error angular de torreta y calidad de pose/visión;
- resultado categórico: corto, anotado, largo, rebote u otro;
- notas mecánicas y video/log ID.

Calibrar primero en posiciones estacionarias y con piezas consistentes. Cada distancia candidata requiere al menos 10 tiros por sesión en dos sesiones separadas; las distancias retenidas también requieren 10 por sesión y no se combinan sesiones para ocultar una falla. Mezclar datos de mecanismos distintos invalida el modelo.

### 10.2 Modelos candidatos

1. **Lineal:** `rpm = a + b*d`. Preferido si cumple; fácil de explicar y estable.
2. **Cuadrático:** `rpm = a + b*d + c*d²`. Grado máximo permitido inicialmente; vigilar extrapolación.
3. **Piecewise-linear:** tabla de distancias/RPM con interpolación. Útil si la respuesta tiene regiones, mantiene valores interpretables.

Separar datos de entrenamiento/calibración y puntos retenidos/intermedios. Elegir el modelo más simple que logre al menos el criterio de tiro definido; no elegir por el menor error dentro de los mismos puntos usados para ajustarlo.

### 10.3 Límites

- `MIN_VALIDATED_RPM` y `MAX_VALIDATED_RPM`: pendientes de aprobación mecánica;
- rango de distancias soportado: sólo donde existen datos;
- trim normal: ±500 RPM;
- paso: 100 RPM;
- clamp físico: siempre activo;
- slew rate: medir para evitar cambios agresivos y waits largos;
- readiness inicial: error ≤100 RPM durante ≥250 ms.
- readiness de movimiento: velocidades lineal y angular bajo umbrales medidos; ambos son `TBD-BLOCKING` hasta T9.

El operador corrige overshoot con trim negativo y undershoot con trim positivo. El sistema registra el trim para mejorar la calibración futura; el trim no se persiste automáticamente al siguiente init.

## 11. Estrategia ante pérdida de sensores

| Odometría | Visión | Modo | Torreta | Feed |
|---|---|---|---|---|
| Buena | Buena | `NORMAL_FUSED` | Bearing geométrico + trim gated | Permitido con readiness. |
| Buena | Perdida/mala | `ODOMETRY_ONLY` | Geométrico, con ventana de deriva validada | Permitido sólo por política validada; indicador visible. |
| Mala | Buena | Relocalización/hold | No seguir un `tx` aislado durante pose inconsistente | Bloqueado hasta corrección aceptada. |
| Mala | Mala | `DEGRADED_FIXED_FORWARD` sólo por entrada deliberada | Cero frontal si armada/plausible | Manual RPM + hold + dwell + timeout. |
| Cualquiera | Fault de torreta | `DISABLED/FAULT` | Power 0 | Bloqueado. |

La visión sola puede ayudar a relocalizar, pero el diseño no permite que un frame aislado tome el control inmediato de la torreta.

## 12. Plan de implementación técnica

1. Crear tipos/contratos y tests de wrap/transforms sin hardware.
2. Encapsular Pedro detrás de `PoseProvider` y `PedroDriveAdapter`; calibrar pose y movimiento como una sola migración.
3. Encapsular Limelight; shadow logging sin consumidor.
4. Completar tabla de marcos y extrínsecas.
5. Implementar gates y replay de logs.
6. Activar corrección complementaria con clamps conservadores.
7. Implementar targeting geométrico en simulación/replay.
8. Probar torreta con target virtual y potencia reducida.
9. Activar trim visual limitado.
10. Recoger dataset de RPM y escoger modelo.
11. Integrar readiness/feeder y fallas.
12. Ejecutar [programa de pruebas](05-programa-pruebas.md).

## 13. Criterios de aceptación de investigación

La fase de investigación termina cuando existe evidencia, no sólo enlaces:

- revisión/versiones de manual, CAD, SDK y Limelight registradas;
- mapa de tags y punto de aim aprobados;
- marcos completos con cuatro anclas y ambas alianzas;
- extrínseca de cámara medida y verificada;
- logs shadow de visión/odometría con razones de rechazo;
- método de fusión elegido por resultados;
- modelo de RPM elegido con datos retenidos;
- fallas de cámara/pose no producen movimiento ni feed no solicitado.

## 14. Preguntas que deben quedar respondidas físicamente

- ¿Cuál es la pose exacta de Limelight respecto al centro canónico?
- ¿Qué API/resultado de Limelight ofrece mejor estabilidad en la versión efectiva del SDK?
- ¿Qué latencia/edad máxima mantiene el bearing dentro del error de torreta a la velocidad real?
- ¿Cuánto deriva Pedro sin visión durante 1, 3 y 5 s de movimiento típico?
- ¿Cuál es el arco seguro real de la torreta con margen de cables?
- ¿Qué punto del goal maximiza consistencia con el shooter fijo?
- ¿Qué rango de distancia y RPM puede soportarse sin extrapolación?
- ¿El criterio 2.5°/±100 RPM, ≥9/10 por distancia calibrada y ≥8/10 por distancia retenida en cada sesión es suficiente para el juego real o necesita estrecharse mediante una decisión con datos?

Hasta responderlas, auto-aim permanece en commissioning.
