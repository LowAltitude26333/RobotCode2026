# 09 — Runbook de Paso 2: calibración física de los 3 pods de odometría + IMU

> Estado: procedimiento documental; no autoriza energizar ni mover hardware por sí solo.
> Baseline de código inspeccionado: `1810a5120dcdd386dabfdc2764912d981f5b141d`
> Última actualización: 2026-07-19
> Alcance: instancia acotada de la guía 08 secc. 5.1/5.2 para Paso 2 de `plan-paralelo-20h.md` secc. 2 (odometría de 3 pods + IMU, sin reabrir mappings ya confirmados).
> No redefine procedimiento, gates ni umbrales: sólo empaqueta lo ya decidido en `08-guia-verificacion-hardware.md`, `05-programa-pruebas.md` (T3/T4.4/T5) y `plan-paralelo-20h.md` secc. 2 para que la sesión física no lo improvise.

## 0. Un hecho que ordena toda la sesión

MP-01 sigue `BLOCKED` (ver `handoff-task.md`). Eso bloquea **sólo** la parte de esta calibración que necesita motores armados. La mayoría del trabajo de pods se hace empujando/girando el robot a mano con los motores apagados, y **no depende de MP-01**.

Verificado directamente (no por inferencia) contra el código de los tuners:

- Pedro (`pedroPathing/Tuning.java`): `ForwardTuner` (L217), `LateralTuner` (L265), `TurnTuner` (L313) y `OffsetsTuner` (L1615) no llaman `follower.setTeleOpDrive(...)` en ningún punto de su cuerpo — a diferencia de `LocalizationTest` (L187) y los tuners de velocidad/frenado, que sí.
- Road Runner (`tuning/TuningOpModes.java`): se descompiló `ftc-0.1.23/jars/classes.jar` y se confirmó a nivel de bytecode que `ForwardPushTest.class`, `LateralPushTest.class` y `DeadWheelDirectionDebugger.class` **no contienen ninguna referencia a `setPower`/`setVelocity`**; `ForwardRampLogger.class` sí. Por eso esos tres tuners quedan habilitados detrás de `TuningOpModes.DISABLED_HAND_PUSH_TUNERS` (ver secc. 5), separado del resto del menú que sigue detrás de `DISABLED_POWERED_TUNERS`.

Esto separa la sesión en tres fases con distinto bloqueo:

- **Fase A (software, sin robot):** ya lista — ver secc. 5.
- **Fase B (física, motores apagados):** disponible ya, en paralelo con el cierre de MP-01.
- **Fase C (física, motores armados):** bloqueada hasta que MP-01 salga de `BLOCKED` (guía 08 secc. 4, etapa "restringido y baja potencia").

## 1. Alcance de esta ventana (no confundir con el gate completo)

`plan-paralelo-20h.md` secc. 2 autoriza explícitamente un alcance reducido para esta ventana:

> "Para esta ventana basta con las repeticiones estrictamente necesarias para firmar signo/offset — no hace falta correr ya el gate completo de ≤2in/2° con todas las combinaciones de ruta si el tiempo aprieta; eso se puede completar en una segunda pasada antes de MP-08."

Esto es **distinto** del gate T5 completo de `05-programa-pruebas.md` (5 repeticiones × forward/strafe/360°CW/CCW/cuadrado/ruta mixta, error por segmento, media, máximo, distribución, ≤2 in/2° sin deriva sistemática), que sigue siendo el gate que exige `plan-maestro-robot.md` para aceptar MP-02 y desbloquear MP-04/MP-05/MP-06.

**Salida de este runbook (Paso 2, alcance reducido):** tabla firmada de signo/ticks-por-rev/offset por pod (Road Runner y Pedro) + signo/orientación de IMU confirmados.
**Deuda explícita que este runbook NO cierra:** el gate estadístico T5 completo. `plan-paralelo-20h.md` secc. 5, punto de sync 4, todavía no decide si el Paso 2 basta como evidencia parcial o si hace falta correr T5 entero antes de MP-08 — no cerrar esa pregunta por omisión.

## 2. Mapeo físico ya confirmado (no remedir, no reabrir)

De `RobotMap.java` y `contrato-hardware.md`, coherente entre Road Runner y Pedro:

| Pod/sensor | Nombre en código | Puerto Control Hub | Encoder RobotMap | Inversión de motor (ya confirmada) | Signo/ticks/offset de encoder |
|---|---|---|---|---|---|
| `par0` | `rightFront` | 0 | `ODOMETRY_PARALLEL_0` | No invertido | **PENDIENTE — esta sesión** |
| `par1` | `leftFront` | 2 | `ODOMETRY_PARALLEL_1` | Invertido | **PENDIENTE — esta sesión** |
| `perp` | `rightBack` | 1 | `ODOMETRY_PERPENDICULAR` | Invertido | **PENDIENTE — esta sesión** |
| IMU | `imu` | — | REV Internal IMU `BHI260AP` | N/A | **PENDIENTE — esta sesión** |

No hay Pinpoint físico — los tres pods se leen por los puertos de encoder del Control/Expansion Hub (`plan-paralelo-20h.md` secc. 1, confirmado 2026-07-18). Si alguien reabre esta pregunta en sesión, la respuesta es "ya cerrada, ver esa sección" — no una re-derivación en vivo.

**No confundir** la columna "inversión de motor" (ya `CONFIRMADO EN DS`, gobierna la dirección de giro cuando el motor recibe potencia) con el signo de lectura del encoder del pod (columna pendiente de esta sesión, gobierna qué dirección de giro incrementa el conteo). Son flags independientes en el código: `par0Reversed`/`par1Reversed`/`perpReversed` en `ThreeDeadWheelLocalizer.Params` y `TBD_*_ENCODER_DIRECTION` en `pedroPathing/Constants.java`.

## 3. Fase B — física, motores apagados (disponible ya)

Guía 08 secc. 4, etapas 1-3 (documental → mecánica desenergizada → sensores con actuadores inhibidos). No requiere MP-01.

1. **Documental:** contrastar `RobotMap.java`, `pedroPathing/Constants.java` y el export de configuración RC contra la tabla de la secc. 2. Cualquier discrepancia detiene la sesión y abre un finding — no se asume ni se corrige en vivo.
2. **Mecánica desenergizada:** inspeccionar holgura de cable en ambos extremos de cada pod (T3 de `05-programa-pruebas.md`), presión/contacto de cada rueda, y **dibujar** la posición X/Y de cada pod respecto al centro de giro con una convención de signo explícita, antes de correr cualquier tuner. Este boceto es lo que permite contrastar el signo que reporte el tuner contra una referencia física independiente.
3. **Sensores con actuadores inhibidos — el trabajo de medición:**
   - Signo de cada encoder: `DeadWheelDirectionDebugger` (RR) — empujar/girar el pod a mano y leer el signo del conteo crudo.
   - Ticks/rev y distancia/tick, **medidos por separado para cada eje**: `ForwardPushTest`/`ForwardTuner` con distancia de cinta medida, ≥5 repeticiones por sentido, para `par0`/`par1`; `LateralPushTest`/`LateralTuner` con distancia de cinta medida, ≥5 repeticiones por sentido, para `perp`.
   - Turn ticks/inches: `TurnTuner` (Pedro) con ángulo medido (transportador o marca), ≥5 repeticiones CW + ≥5 CCW — **este valor no se infiere del forward**, se mide en su propio eje.
   - Offsets de pod (`par0YTicks`/`par1YTicks`/`perpXTicks` en RR; `TBD_LEFT_POD_Y_INCHES`/`TBD_RIGHT_POD_Y_INCHES`/`TBD_STRAFE_POD_X_INCHES` en Pedro): del boceto físico del paso 2, cruzados con `OffsetsTuner` (Pedro). `OffsetsTuner` espera que los offsets de la config activa estén en 0 antes de correr — confirmar esa precondición en pantalla, no asumirla (hoy `TBD_STRAFE_POD_X_INCHES=0.0` ya la cumple, pero verificar igual).
   - IMU: procedimiento ya definido en guía 08 secc. 5.1 — 5 giros manuales CW + 5 CCW, registrar heading al init y el cambio de signo. No rediseñar el procedimiento, sólo ejecutarlo (`plan-paralelo-20h.md` secc. 2 lo pide explícitamente).
4. **Chequeo de cordura antes de cerrar la sesión:**
   - `par0YTicks ≠ par1YTicks` — con offsets idénticos la matemática de three-wheel divide entre `(par0YTicks - par1YTicks)` y produce NaN (el propio código lo advierte en `Constants.java` líneas 67-69).
   - Los tres valores de ticks-por-pulgada (forward/strafe/turn) midieron **distinto** de `.001989436789` copiado tres veces — si salen iguales entre sí después de medir independientemente, es una coincidencia a anotar, nunca un atajo tomado a propósito.

## 4. Fase C — física, motores armados (bloqueada hasta que MP-01 cierre)

T4.4 de `05-programa-pruebas.md`: forward/strafe/giro corto a baja potencia, confirmar que el signo observado coincide con la tabla de la Fase B. Debe ser una confirmación rápida, no una sesión de debugging — la Fase B ya hizo el trabajo pesado.

## 5. Preparación de software ya hecha (Fase A)

- `tuning/TuningOpModes.java`: el flag único `DISABLED` se dividió en `DISABLED_HAND_PUSH_TUNERS` (gobierna `ForwardPushTest`/`LateralPushTest`/`DeadWheelDirectionDebugger`, confirmado sin `setPower`/`setVelocity`) y `DISABLED_POWERED_TUNERS` (gobierna el resto: ramp loggers, feedforward, `LocalizationTest`, splines, y las plantillas OTOS sin sensor correspondiente en este robot). Para habilitar la Fase B, cambiar `DISABLED_HAND_PUSH_TUNERS` a `false`; `DISABLED_POWERED_TUNERS` permanece `true` hasta que MP-01 cierre y su lifecycle de Stop/E-stop quede verificado (DEC-035).
- Por qué no hace falta un wrapper de E-stop adicional para el subconjunto de Fase B: los tres tuners habilitados no comandan ningún motor (confirmado por bytecode), así que no hay actuador que un E-stop deba detener. El requisito de DEC-035 ("stop, interrupción y E-stop verificables") está pensado para los tuners que sí mueven drivetrain — esos siguen bloqueados.
- `pedroPathing/Tuning.java` permanece `@Disabled` por ahora. A diferencia de RR, expone un único menú (`Localization`/`Automatic`/`Manual`/`Tests`/`Swerve`) sin separación por flag — quitar `@Disabled` expone todo el menú a la vez, incluyendo `Swerve` (plantilla sin hardware correspondiente, este robot es mecanum) y las carpetas `Automatic`/`Manual` que sí mueven motores. Antes de quitar `@Disabled`, dejar por escrito en la sesión que **sólo la carpeta `Localization`** aplica, y no navegar a las demás. Quitar la anotación se hace junto con el resto del trabajo de Fase B, no antes.
- No se construyó ningún comparador "shadow" RR-vs-Pedro en vivo: `handoff-plan-MP01-MP02-MasterPlan.md` ya lo prohíbe (nunca instanciar ambos `Follower`/localizador sobre el mismo hardware a la vez). Si se quiere una comparación, son dos corridas **secuenciales** de un solo dueño cada una sobre el mismo recorrido marcado, diferenciadas a mano desde la hoja de evidencia.
- No se construyó `PedroPoseProvider`/`PedroDriveAdapter`: no existen todavía en el repo, y son el cierre de DEC-034/FND-017, posterior a Paso 2 — no un prerrequisito para calibrar los pods.

## 6. Hoja de evidencia (llenar en sesión, no después)

Diez instancias `MEAS-XXX` según la plantilla de guía 08 secc. 6. Campos ya conocidos pre-llenados; el resto se completa en campo.

### MEAS-POD-PAR0-DIR — signo de encoder de `par0` (rightFront)
- SHA / configuración RC: _(llenar el día de la sesión)_
- Fecha / responsables: _(llenar)_
- Estado inicial y restricción física: motores apagados, robot en banco/blocks
- Instrumento / resolución: telemetría de `DeadWheelDirectionDebugger`, lectura directa de encoder
- Unidad y convención: signo (+/-) al empujar el robot hacia adelante
- Valores crudos (repeticiones): _(llenar)_
- Cálculo / incertidumbre: N/A (observación directa de signo)
- Valor candidato: _(llenar — `par0Reversed` true/false)_
- Límite/margen aplicado: N/A
- Stop/E-stop verificado: N/A — tuner sin `setPower`/`setVelocity` (ver secc. 5)
- Evidencia (foto/log/video): _(llenar)_
- Revisor y decisión: _(VERIFIED / REJECTED / TBD-BLOCKING)_
- Finding/test relacionado: FND-004

### MEAS-POD-PAR0-TICKS — ticks/rev, diámetro y distancia/tick de `par0`
- SHA / configuración RC: _(llenar)_
- Fecha / responsables: _(llenar)_
- Estado inicial y restricción física: motores apagados, robot en banco/blocks
- Instrumento / resolución: cinta métrica + `ForwardPushTest`/`ForwardTuner`
- Unidad y convención: pulgadas por tick, forward positivo
- Valores crudos (repeticiones): ≥5 repeticiones forward — _(llenar)_
- Cálculo / incertidumbre: _(llenar)_
- Valor candidato: _(llenar — reemplaza parte de `inPerTick`/`TBD_FORWARD_TICKS_TO_INCHES`)_
- Límite/margen aplicado: N/A
- Stop/E-stop verificado: N/A — tuner sin `setPower`/`setVelocity`
- Evidencia: _(llenar)_
- Revisor y decisión: _(VERIFIED / REJECTED / TBD-BLOCKING)_
- Finding/test relacionado: FND-004

### MEAS-POD-PAR1-DIR — signo de encoder de `par1` (leftFront)
- (mismos campos que MEAS-POD-PAR0-DIR, pod `par1`)

### MEAS-POD-PAR1-TICKS — ticks/rev, diámetro y distancia/tick de `par1`
- (mismos campos que MEAS-POD-PAR0-TICKS, pod `par1`, mismas ≥5 repeticiones forward)

### MEAS-POD-PERP-DIR — signo de encoder de `perp` (rightBack)
- Unidad y convención: signo (+/-) al empujar el robot hacia la izquierda
- (resto de campos igual al patrón DIR)

### MEAS-POD-PERP-TICKS — ticks/rev, diámetro y distancia/tick de `perp`
- Instrumento / resolución: cinta métrica + `LateralPushTest`/`LateralTuner`
- Valores crudos (repeticiones): ≥5 repeticiones strafe — _(llenar)_
- (resto de campos igual al patrón TICKS)

### MEAS-POD-TURN — turn ticks/inches (independiente de forward/strafe)
- Instrumento / resolución: transportador/marca angular + `TurnTuner` (Pedro)
- Valores crudos (repeticiones): ≥5 CW + ≥5 CCW — _(llenar, por separado)_
- Valor candidato: _(llenar — `TBD_TURN_TICKS_TO_INCHES`; no copiar del valor forward/strafe)_

### MEAS-POD-GEOMETRY — offsets X/Y de los tres pods vs. centro de giro
- Instrumento / resolución: regla/cinta + boceto con convención de signo (secc. 3, paso 2) + `OffsetsTuner`
- Valores crudos: medidas en pulgadas de cada pod — _(llenar)_
- Valor candidato: `par0YTicks`/`par1YTicks`/`perpXTicks` (RR) y `TBD_LEFT_POD_Y_INCHES`/`TBD_RIGHT_POD_Y_INCHES`/`TBD_STRAFE_POD_X_INCHES` (Pedro) — _(llenar)_
- Chequeo obligatorio: confirmar `par0YTicks ≠ par1YTicks` antes de cerrar (ver secc. 3, paso 4)

### MEAS-IMU-SIGN — orientación y signo de heading del IMU
- Instrumento / resolución: procedimiento guía 08 secc. 5.1, telemetría de heading
- Unidad y convención: tipo `BHI260AP`, orientación física logo/USB, heading en grados
- Valores crudos (repeticiones): 5 giros manuales CW + 5 CCW — _(llenar)_
- Valor candidato: _(llenar — signo/orientación a usar en `DriveAdapter`)_
- Stop/E-stop verificado: N/A — procedimiento manual sin motores
- Finding/test relacionado: contrato-hardware.md fila IMU

### MEAS-DRIVE-CONFIRM — confirmación a baja potencia (Fase C, sólo tras MP-01)
- Estado inicial y restricción física: robot en banco/blocks, MP-01 fuera de `BLOCKED`
- Instrumento / resolución: T4.4 de `05-programa-pruebas.md`
- Valores crudos: forward/strafe/giro corto, pass/fail por eje — _(llenar)_
- Valor candidato: confirma (no remide) la tabla de signos de la Fase B
- Stop/E-stop verificado: sí — requiere el lifecycle de MP-01 ya validado

## 7. Trampas a evitar

1. **Copiar el mismo valor medido entre forward/strafe/turn** "porque total los pods son iguales" — deben medirse tres veces, de forma independiente. `TBD_FORWARD/STRAFE/TURN_TICKS_TO_INCHES` hoy comparten literalmente `.001989436789`; eso es evidencia de scaffold, no de medición.
2. **Adivinar o invertir un signo "a ver si funciona"** — guía 08 secc. 4 lo prohíbe explícitamente: un signo distinto al esperado detiene la sesión y abre un finding, no se prueba el contrario a ciegas.
3. **Editar un dato crudo para que coincida con la constante "esperada"** — una corrección es una fila nueva en la hoja de evidencia; la fila anterior no se sobreescribe.
4. **Perder tiempo con tuners irrelevantes del menú** (OTOS, Pinpoint, Swerve) — quedan fuera del subconjunto habilitado en Fase A; si aparecen en el menú de Pedro, no navegar a esas carpetas.
5. **Reabrir la pregunta de si hay un Pinpoint físico** — ya cerrada (`plan-paralelo-20h.md` secc. 1).
6. **Confundir inversión de motor de drive con signo de encoder de pod** — son flags independientes (ver secc. 2); uno ya está confirmado, el otro es lo que esta sesión mide.
7. **Tratar "el tuner corrió sin crashear" como evidencia de calibración correcta** — `Constants.createFollower()` sólo advierte por log cuando `LOCALIZER_OFFSETS_CALIBRATED=false`, pero igual construye un `Follower` geométricamente incorrecto. La única evidencia válida es la hoja de este runbook, revisada y firmada.

## 8. Cierre de sesión

1. Confirmar que las 9 filas `MEAS-*` de pods/IMU (todas menos `MEAS-DRIVE-CONFIRM`, que espera Fase C) tienen decisión `VERIFIED`.
2. Escribir los valores medidos en `ThreeDeadWheelLocalizer.Params` (RR) y en los `TBD_*` de `pedroPathing/Constants.java`. Voltear `LOCALIZER_OFFSETS_CALIBRATED=true` sólo si guía 08 secc. 7 (las 6 condiciones: evidencia repetible, revisor mecánico/eléctrico, dueño de software confirma fuente única, Stop/E-stop, vínculo a SHA/config, más las anteriores) está satisfecha.
3. Actualizar las filas de `par0`/`par1`/`perp`/IMU en `contrato-hardware.md`. Si sólo se corrió el alcance reducido de esta sesión (sin T5 completo), el estado correcto es "signo/offset verificado, gate estadístico pendiente" — **no** `VALIDADO` a secas, porque `contrato-hardware.md` exige que coincidan atestación + inspección física + prueba controlada, y la prueba controlada completa (T5) sigue pendiente.
4. Registrar en `handoff-task.md` explícitamente que el gate T5 completo no se corrió todavía y que sigue pendiente antes de MP-08 (secc. 1 de este documento). No dejar que la entrada de la bitácora se lea como "MP-02 cerrado".
5. Cuando llegue la segunda pasada (antes de MP-08): correr el gate T5 completo de `05-programa-pruebas.md` (5 repeticiones × forward/strafe/360°CW/CCW/cuadrado/ruta mixta, error por segmento, media, máximo, distribución, ≤2 in/2° sin deriva sistemática) y recién ahí evaluar aceptar MP-02 formalmente.
