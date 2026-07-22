# Plan paralelo de 20h — pista Software y pista Tuning

> Estado: plan de trabajo activo para la ventana previa a competencia (~20h restantes, equipo grande: 3+ personas de software, varias de mecánica/eléctrica/pruebas)
> Última actualización: 2026-07-22
> Este documento no repite contenido de otros documentos del programa. Cuando un paso ya está descrito en otro archivo, aquí solo se referencia y se dice **cuándo** ejecutarlo y **con quién sincronizarse**, no se vuelve a explicar el procedimiento.

## 0. Cómo usar este documento

**Antes de seguir leyendo, responde: ¿vas a trabajar Software o Tuning en esta sesión?**

- **Software** → ve directo a la [sección 3](#3-pista-software). Antes de empezar cualquier tarea, revisa la [sección 4](#4-tabla-de-bloqueos-cruzados) para confirmar que no estás bloqueado por algo que Tuning todavía no midió.
- **Tuning** → ve directo a la [sección 2](#2-pista-tuning-física), empezando siempre por el Paso 0 si todavía no se hizo en esta máquina. Antes de cada paso, revisa la sección 4 para confirmar que Software ya entregó lo que ese paso necesita.

Si no sabes cuál de las dos te toca, pregúntale a quien esté coordinando la sesión antes de empezar — no arranques por tu cuenta un paso que depende de algo que no has confirmado que ya existe.

El trabajo restante se divide en dos pistas que corren **en paralelo**, no en secuencia:

- **Pista Tuning** — trabajo físico sobre el robot real. Personas de mecánica/eléctrica/pruebas, con apoyo de quien opere el gamepad/Driver Station.
- **Pista Software** — trabajo de código que no requiere tener el robot en la mano. Personas de software.

Cada pista tiene su lista ordenada de pasos. La sección 5 (**tabla de bloqueos cruzados**) es la pieza central: dice exactamente qué debe existir en el código antes de que Tuning pueda escribir un número medido, y qué debe medir Tuning antes de que Software pueda cerrar una constante o un gate. La sección 6 marca los momentos concretos donde ambas pistas deben juntarse antes de seguir.

Fuentes autoritativas que este documento referencia sin duplicar:

- Programa completo y gates MP-00…MP-10: [`../plan-maestro-robot.md`](../plan-maestro-robot.md)
- Procedimiento físico paso a paso: [`08-guia-verificacion-hardware.md`](08-guia-verificacion-hardware.md)
- Gates de prueba T0–T11: [`05-programa-pruebas.md`](05-programa-pruebas.md)
- Estado confirmado/pendiente de cada componente: [`contrato-hardware.md`](contrato-hardware.md)
- Decisiones vigentes: [`decisiones.md`](decisiones.md)
- Findings abiertos: [`hallazgos.md`](hallazgos.md)
- Bitácora corrida de evidencia física y próxima acción: [`handoff-task.md`](handoff-task.md)

Este documento **no reemplaza** `handoff-task.md`: cada resultado físico se sigue registrando ahí, con SHA/APK/fecha/repeticiones como ya se viene haciendo.

## 1. Decisión de hardware que gobierna ambas pistas

**La odometría de los tres pods se lee por los puertos de encoder del Control/Expansion Hub — no hay un goBILDA Pinpoint físico.** Confirmado directamente con el equipo el 2026-07-18.

Esto importa porque `pedroPathing/Constants.java` tiene hoy nombres de encoder placeholder tipo "pinpoint" (`// poner bien los nombres de las pinpoint`), lo cual sugiere que el scaffold de Pedro se armó asumiendo una computadora de odometría dedicada. **Eso es incorrecto para este robot.** Pedro debe leer los mismos tres pods que ya usa Road Runner hoy, por los mismos puertos del Hub:

| Pod | Motor/puerto físico (según `RobotMap`) |
|---|---|
| `par0` | `rightFront` |
| `par1` | `leftFront` |
| `perp` | `rightBack` |

Ninguna de las dos pistas debe reabrir esta pregunta ni asumir un Pinpoint. Si en algún momento aparece evidencia de que sí existe un Pinpoint físico, es un finding nuevo en `hallazgos.md`, no un cambio silencioso aquí.

## 2. Pista Tuning (física)

Sigue el orden seguro ya definido en [`08-guia-verificacion-hardware.md`](08-guia-verificacion-hardware.md) (documental → mecánico desenergizado → sensores con actuadores inhibidos → potencia restringida → carga representativa → campo controlado). Lo que sigue es la secuencia concreta para esta ventana de 20h sobre ese orden.

### Paso 0 — Sacar el entorno de build de OneDrive (una sola vez, antes de cualquier otra cosa)

**Dueño de este paso: la persona de Tuning cuya computadora se usa para compilar e instalar (flashear) el APK en el robot** — no es tarea de todo Tuning ni de Software. Si esa persona no está identificada todavía, identifíquenla antes de arrancar cualquier otro paso de esta pista, porque cualquier build/install posterior pasa por esa máquina.

En la sesión anterior, `packageDebug`/`mergeDebugResources` fallaron repetidamente por locks/placeholders (`ReparsePoint`) de OneDrive — documentado en `handoff-task.md` líneas de build de esa sesión. Cada incidente costó 10-20 minutos de diagnóstico y rebuild. Esto se resuelve una vez, no en cada sesión:

1. En esa máquina (la que compila e instala APKs sobre el robot real), verificar si el checkout del repo vive dentro de una carpeta sincronizada por OneDrive.
2. Si sí: mover el checkout fuera de esa carpeta (ej. a `C:\dev\...`) **o**, si no se puede mover, excluir explícitamente `TeamCode/build/`, `**/build/` y `.gradle/` de "Archivos bajo demanda"/sincronización de OneDrive para esa carpeta.
3. Verificar con un `assembleDebug --no-daemon` limpio (borrando `TeamCode/build` primero) que termina sin intervención manual y sin mensajes de `ReparsePoint`.
4. Registrar en `handoff-task.md` que este paso quedó resuelto, con la máquina/usuario donde se hizo.

**Gate de salida:** un build limpio corre de principio a fin sin pausar OneDrive ni cerrar Android Studio a mano, en la máquina que instala sobre el robot.

Nota: si alguien de Software también compila localmente en su propia laptop solo para verificar que el código compila (sin instalar en el robot), este paso no le aplica a esa laptop — es específico de la máquina que hace el `assembleDebug`/install que termina en el robot físico.

### Paso 1 — Cerrar MP-01 físicamente

- Torreta/FND-027: **completado** bajo APK `9C2F`. DEC-039 acepta 10/10 por sentido: 20/20 terminaron en cero, sin anomalías y dentro del envelope acordado. La asimetría antihoraria promedio de 11.6% queda como limitación conocida; no repetir la matriz ni introducir un multiplicador fijo durante MP-01.
- Shooter/FND-028: **cerrado para MP-01 por aceptación del Test lead** bajo APK `09B2...DAC1`. El pulso sin piezas terminó `STOPPED_TIMEOUT` a `3027.7 ms`, ticks `0 -> 555`, peak/end `1028.6/942.9 RPM`, batería mínima `12.37 V`, power final cero, dirección hacia fuera, parada completa y cero anomalías. El hold continuo fue `125.6 ms`: no cumple todavía T8 (`>=250 ms`), por lo que no se repite este pulso, no se habilita feeder y la estabilidad/carga quedan para MP-06/T8.
- Candidato de prueba integrada `PRUEBA: MECANISMOS SIN SERVO`: **MOTOR-ONLY FINAL / TORRETA VALIDADA**. FND-003 y FND-020 cerrados por confirmación física del lead. Envelope `-983/+1070`, power `0.50` con aproximación `0.05` en 100 ticks; candidato `5FF4...4E61`, `320C...D226` superseded. Feeder con piezas y T8/carga continúan pendientes.
- Kicker: configuración de competencia motor-only formalizada por DEC-037; reintroducir un servo requiere una decisión y ciclo de diseño nuevos.
- Regla de proceso: cuando haya varios fixes de software independientes ya identificados y sin riesgo de interacción entre sí, pedir a Software que los agrupe en un solo candidato antes de reinstalar — no un APK por fix.
- Regla de proceso: los E-stop/Stop de `SystemCheck` y `ShooterTuning` comparten exactamente el mismo lifecycle de `SafeCommandOpMode` ya probado 10/10 en `MainTeleOp`. No repetir la matriz completa ahí — un spot-check de 3-5 repeticiones por modo basta como evidencia de que la misma ruta de código sigue funcionando.

**Gate de salida:** MP-01 pasa de `BLOCKED` a `READY_FOR_GATE` según el criterio ya definido en `handoff-plan-MP01-MP02-MasterPlan.md`.

### Paso 2 — Calibración física de los 3 pods de odometría + IMU

Sobre los puertos del Hub confirmados en la sección 1 (no Pinpoint). Signo, ticks/rev y offset de cada pod respecto al centro de giro. Alimenta directamente `ThreeDeadWheelLocalizer.Params` (Road Runner) y la configuración equivalente que Software deja lista en `pedroPathing/Constants.java` (ver sección 3). Corresponde a T5 de `05-programa-pruebas.md`, aunque para esta ventana basta con las repeticiones estrictamente necesarias para firmar signo/offset — no hace falta correr ya el gate completo de ≤2in/2° con todas las combinaciones de ruta si el tiempo aprieta; eso se puede completar en una segunda pasada antes de MP-08.

La **orientación/signo del IMU** quedó validada 5/5 por sentido bajo `RIGHT/BACKWARD`: antihorario positivo, horario negativo y error máximo `4.4°` con tolerancia `5°`. No repetir salvo cambio de montaje, Control Hub, configuración o código. Continúa la calibración de los tres pods, que deriva su heading por una ruta distinta.

**Gate de salida:** tabla firmada de signo/ticks-por-rev/offset por pod (Road Runner y Pedro) + signo/orientación de IMU confirmados.

### Paso 3 — Torreta: límites finales y falso cero

Tabla ticks/grado firmada, límites finales (partiendo de los ±200 actuales como conservador, solo se amplían con evidencia), comportamiento ante falso cero/reset/brownout (T4/T7 de `05-programa-pruebas.md`, findings FND-003/FND-020/FND-021).

**Gate de salida:** contrato de torreta sin `BLOCKED_PHYSICAL` en `contrato-hardware.md`.

### Paso 4 — Limelight: montaje y anclas físicas

Solo puede arrancar después de que Software entregue `LimelightSubsystem` (sección 3). Montaje rígido, nombre/puerto/red confirmados, anclas físicas por alianza para MP-04 (T6 de `05-programa-pruebas.md`).

### Paso 5 — Caracterización de tiro y auto-aim

Primero cerrar T8.1 (control de RPM, ±100 RPM durante 250 ms) — el shooter tiene su fail-closed cerrado (FND-015) pero la RPM física todavía está `PENDIENTE` en `contrato-hardware.md`, no validada. Recién después, dataset de tiro por distancia (T8.2/T9) y tuning de auto-aim de torreta (T7) una vez que Software tenga el wrapping de ángulo listo con los límites reales del Paso 3.

### Fuera de esta ventana

MP-09 (limpieza) y MP-10 (revalidación post-limpieza) quedan explícitamente pospuestos a después de competir — no aportan capacidad competitiva y consumen tiempo de build/regresión que esta ventana no tiene.

## 3. Pista Software

Ninguna de estas tareas requiere tener el robot físico en la mano. Se puede trabajar completo en paralelo a la Pista Tuning.

> **Estado 2026-07-18: las 6 tareas están ENTREGADAS en `masterplan`** (commits `de15a2f`, `31bbbb0`, `2a3d497`, `9b77ab8`, `3989365` + el commit de este doc). Gate: compile + `testDebugUnitTest` (23/23) + `assembleDebug` PASS. Implicaciones para Tuning:
> - **Sync point 2 desbloqueado:** existe `LimelightSubsystem` + OpMode `Limelight Diagnostic` (grupo Diagnostics, cero actuadores) para el Paso 4.
> - **Sync point 1 (mitad Software) desbloqueado:** `pedroPathing/Constants.java` ya no tiene código muerto; los números del Paso 2 caen en las constantes `TBD_*` y el flag `LOCALIZER_OFFSETS_CALIBRATED`.
> - El dataset del Paso 5 se carga como `ShotSample`s al `RpmModelSelector` (DEC-012 automatizado).
> - Nada del comportamiento validado físicamente cambió; el único APK de referencia sigue siendo el instalado por Tuning. Nota de build: usar `JAVA_HOME` del JBR de Android Studio y `GRADLE_USER_HOME=C:\Users\Public\gradle-home` para correr tests por CLI (ver `inventario-telemetria-tuners.md` secc. 4).

1. **Reparar `pedroPathing/Constants.java`.** Hoy `localizerConstants` fija offsets de pods (`.leftPodY(1).rightPodY(-1).strafePodX(-2.5)`) y los sobreescribe a `0` unas líneas después (código muerto/contradictorio) y usa nombres de encoder tipo "pinpoint" (`//poner bien los nombres de las pinpoint`). Además `rightEncoder_HardwareMapName` está puesto en `"rightRear"`, que no es un nombre de hardware map real de este robot — el nombre confirmado en `RobotMap`/Road Runner es `rightBack`; corregir ese string específico, no solo el patrón general. Corregir para que lea los mismos tres pods que `RobotMap`/Road Runner (par0=`rightFront`, par1=`leftFront`, perp=`rightBack`, sección 1), dejando los valores de offset/signo como constantes claramente `TBD-BLOCKING` hasta que Tuning entregue el Paso 2. Debe compilar y no debe activar Pedro como dueño de movimiento (Road Runner sigue siendo el dueño activo hoy; DEC-034 define a Pedro como dueño objetivo de pose/movimiento pero eso todavía no se cumple en el código).
2. **Crear `PoseProvider`/`DriveAdapter`.** No existe hoy en el árbol (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`). Interfaz neutral para que MP-04/MP-05 no importen APIs concretas del localizador, tal como pide `handoff-plan-MP01-MP02-MasterPlan.md` sección MP-02. Puede escribirse y probarse con datos falsos/mock antes de que existan los offsets reales.
3. **Crear el source set de tests puros.** `TeamCode/src/test` no existe todavía y **`TeamCode/build.gradle` no declara JUnit** (confirmado por grep — cero referencias a `junit`/`testImplementation` en `TeamCode/build.gradle` ni en `build.dependencies.gradle`). Este paso incluye agregar la dependencia de JUnit antes de escribir el primer test, no solo crear la carpeta. Pruebas: wrap de ángulo, transforms de frame, lo que ya pedía FND-004/MP-02.
4. **Crear `LimelightSubsystem`.** No existe hoy — solo un comentario en `MainTeleOp.java:20`. Wrapper único dueño del dispositivo: start, selección de pipeline, poll/timeout, stop/close, conversión a observación inmutable con timestamp. No depende de tener el robot en mano, solo de que el SDK ya resuelva `Limelight3A` (confirmado). **Este archivo bloquea el Paso 4 de Tuning** — priorizarlo si Tuning va a llegar a Limelight en las próximas horas.
5. **Estructura del modelo RPM-por-distancia.** Código de ajuste (lineal/cuadrático/piecewise, MP-06) que solo necesita que Tuning le cargue el dataset de tiros del Paso 5. Puede escribirse y probarse con datos sintéticos antes de tener datos reales.
6. **Telemetría unificada / prep de reducción de menú.** Trabajo de MP-07 (bloques de telemetry, inventario de tuners/ownership) que no mueve actuadores y puede avanzar sin el robot.
7. **(Nueva, 2026-07-22) Alinear e integrar los 10 autónomos Pedro Pathing entregados por el equipo.** DEC-041 revierte la decisión previa de "cero autónomos": el release final sí incluye un conjunto reducido de autos, todos Pedro Pathing. Los templates llegaron en crudo desde la herramienta de paths (misma clase repetida `PedroAutonomous`, starting pose placeholder `(72,8,90°)` igual en los 10, sin extender `SafeAutonomousOpMode`, sin lógica de tiro). Trabajo, ninguno depende del robot: (a) retirar los 8 autónomos Road Runner legado de `opmodes/auto/` (ya no compilan contra el `RobotContainer` Pedro-only); (b) crear `PedroPathCommand` (equivalente a `ActionCommand` pero para `PathChain`/`Follower`) y exponer `followPath`/`isBusy` en `PedroDriveSubsystem`; (c) convertir pose final de Pedro a `PoseStorage.currentPose` (RR `Pose2d`) en `afterSchedulerRun()` para que `MainTeleOp` seguir arrancando desde ahí; (d) renombrar las 10 clases (colisión de nombre real hoy) y corregir su starting pose al primer waypoint real de cada path. Detalle completo en el plan de esa sesión; bloquea sobre todo confirmar con el equipo la lógica de tiro (`autonomousPathUpdate()` viene vacío) y dos discrepancias puntuales (asimetría 45°/55° entre "1 Artifact Red/Blue Full", starting pose distinta de "Leave Goal").

## 4. Tabla de bloqueos cruzados

| Artefacto pendiente | Quién lo produce | Qué necesita para empezar | A quién bloquea | Dónde vive |
|---|---|---|---|---|
| Signo/ticks/offset de los 3 pods | Tuning (Paso 2) | Robot en banco/blocks, encoders del Hub confirmados (sección 1) | `PoseProvider` con datos reales; gate T5; Pedro utilizable | `ThreeDeadWheelLocalizer.Params` (Road Runner) y `pedroPathing/Constants.java` (Pedro) |
| Orientación/signo del IMU | Tuning (Paso 2) | Robot en banco/blocks; procedimiento ya descrito en guía 08 sección 5.1 | `PoseProvider`/`DriveAdapter` (heading field-centric, distinto del heading derivado de pods) | contrato/telemetría IMU; consumido por `DriveSubsystem`/`PoseProvider` |
| Reparación de `pedroPathing/Constants.java` (código muerto + pinpoint→Hub + string `rightRear`→`rightBack`) | Software | Nada — puede empezar ya | Que Tuning tenga dónde escribir el offset del Paso 2 sin pisarlo con código muerto | `TeamCode/src/main/java/.../pedroPathing/Constants.java` |
| Interfaz `PoseProvider`/`DriveAdapter` | Software | Nada — puede empezar ya con mocks | MP-04/MP-05 (auto-aim no puede depender de un localizador concreto) | archivo nuevo, no existe hoy |
| Tabla ticks/grado + límites finales de torreta | Tuning (Paso 3) | MP-01 cerrado (Paso 1), robot con torreta accesible | Soft limits finales y wrapping de ángulo de auto-aim (MP-05) | `TurretSubsystem` / `LowAltitudeConstants` (`TurretConstants`) |
| `LimelightSubsystem` | Software | Nada — puede empezar ya, SDK ya confirmado | Montaje/anclas físicas de Limelight (Tuning Paso 4) — Tuning no debería gastar tiempo de robot en Limelight sin esto listo | archivo nuevo bajo `subsystems/` o `vision/` |
| Anclas físicas por alianza + montaje Limelight | Tuning (Paso 4) | `LimelightSubsystem` listo | MP-04 (fusión de pose) | físico + `contrato-hardware.md` |
| Validación física de RPM del shooter (T8.1, ±100 RPM/250ms) | Tuning (previo al Paso 5) | Shooter con fail-closed cerrado (FND-015 `CLOSED`) — la parte de fault injection ya pasó, pero la RPM física todavía está `PENDIENTE` en `contrato-hardware.md`, no validada | Dataset de tiro por distancia (Paso 5) — no asumir que el shooter ya está listo para caracterizar sin este paso previo | `contrato-hardware.md` (fila shooter) |
| Dataset de tiro por distancia | Tuning (Paso 5) | T8.1 (fila anterior) cerrado, presets manuales disponibles | Modelo RPM-por-distancia (Software) para dejar de ser sintético | `handoff-task.md` (evidencia) → alimenta el modelo en código |
| `TeamCode/src/test` + dependencia JUnit | Software | Nada — puede empezar ya | Ninguna prueba pura de MP-02/MP-04 puede existir sin esto primero | `TeamCode/build.gradle` + carpeta nueva `TeamCode/src/test` |

## 5. Puntos de sincronización

1. **Después del Paso 1 de Tuning (MP-01 cerrado) y de la reparación de `pedroPathing/Constants.java` + `PoseProvider` en Software** → sync antes de arrancar la calibración de pods + IMU (Paso 2 de Tuning), para que el número medido tenga dónde caer sin pisar código muerto.
2. **Después de que Software entregue `LimelightSubsystem`** → sync antes de que Tuning invierta tiempo de robot en montaje/anclas de Limelight (Paso 4).
3. **Después de que Tuning entregue la tabla ticks/grado + límites finales de torreta (Paso 3)** → sync antes de que Software cierre el wrapping de ángulo y soft limits de auto-aim (MP-05).
4. **Antes de MP-08** → sync general para decidir si la calibración de pods necesita completar el gate estadístico completo (5 repeticiones por tipo de ruta) o si lo hecho en el Paso 2 basta como evidencia parcial documentada.

## 6. Qué queda fuera de esta ventana de 20h

- MP-09 (limpieza del candidato) y MP-10 (revalidación post-limpieza) — se retoman después de competir.
- Los tres handoffs formales ya fueron reconciliados al aceptar MP-01/MP-02: `handoff-MP01.md`, `handoff-MP02.md` y `handoff-MasterPlan.md`. `handoff-task.md` se conserva como bitácora cronológica detallada.
