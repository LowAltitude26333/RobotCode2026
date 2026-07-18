# Registro de hallazgos

> Estado: registro vivo; entradas iniciales provienen de auditoría estática, no de pruebas físicas
> Baseline histórico inicial: `main@f91af18`; baseline de implementación: `origin/main@a887fe4f7ca9023eec6034a0db6b8d918c640ecc`
> Última actualización: 2026-07-18
> Alcance: bugs, riesgos, discrepancias, resultados y retests del plan maestro
> Responsable sugerido: test lead; cada entrada debe tener owner técnico
> Fuente de verdad: evidencia vinculada al SHA/configuración/sesión. No cerrar hallazgos sólo porque compile.

## 1. Cómo usar este registro

Crear una entrada cuando:

- el comportamiento observado difiere de lo esperado;
- una suposición física no puede confirmarse;
- un safety path falta o falla;
- una prueba aborta;
- una fuente externa/versión contradice el diseño;
- una métrica no cumple su gate;
- se descubre deuda que puede afectar competencia/mantenimiento.

No crear findings para tareas normales ya planeadas salvo que aparezca una discrepancia concreta. Los items de trabajo viven en el plan; los findings contienen evidencia de un problema/unknown.

## 2. Estados

| Estado | Significado |
|---|---|
| `OPEN` | Confirmado o suficientemente sustentado; necesita acción. |
| `INVESTIGATING` | Se está aislando causa/alcance. |
| `BLOCKED_PHYSICAL` | Falta medición/acceso físico específico. |
| `CONTAINED` | Camino deshabilitado o mitigado, pero causa/código permanece. |
| `FIX_READY` | Fix implementado, falta retest. |
| `CLOSED` | Reproducción y regresión pasan con evidencia. |
| `WONT_FIX` | Riesgo aceptado mediante decisión explícita, con razón. |
| `SUPERSEDED` | Reemplazado por finding más preciso; enlazarlo. |

## 3. Severidad

| Severidad | Criterio orientativo |
|---|---|
| `CRITICAL` | Puede causar movimiento/lanzamiento inseguro, daño, OpMode principal inutilizable o pérdida total de control. Bloquea hardware/release. |
| `HIGH` | Puede causar score loss importante, comportamiento no determinista o bypass de una protección. Bloquea release. |
| `MEDIUM` | Degrada diagnóstico/mantenibilidad o tiene workaround claro; debe priorizarse. |
| `LOW` | Deuda menor/documentación sin impacto operativo inmediato. |

## 4. Índice actual

| ID | Severidad | Estado | Resumen | Owner | Paquete |
|---|---|---|---|---|---|
| FND-001 | CRITICAL | FIX_READY | Cleanup de `VisionPortal` ligado antes de asignarlo | Software vision | MP-01 |
| FND-002 | HIGH | FIX_READY | Chord de armado evaluado sólo una vez en initialize | Software/Ops | MP-01 |
| FND-003 | HIGH | BLOCKED_PHYSICAL | Límites ±200 ticks de torreta no validados | Mechanical + turret | MP-01/05 |
| FND-004 | HIGH | OPEN | Geometría de localización provisional/inconsistente | Localization | MP-02 |
| FND-005 | HIGH | FIX_READY | Feeder tiene caminos directos sin interlock único | Mechanisms | MP-01/06 |
| FND-006 | HIGH | FIX_READY | Secuencia shooter puede esperar readiness sin timeout | Shooter | MP-01/06 |
| FND-007 | MEDIUM | CONTAINED | E-stop no uniforme en OpModes habilitados | Safety | MP-01/07 |
| FND-008 | MEDIUM | OPEN | Ownership de torreta/visión fuera del container | Architecture | MP-07 |
| FND-009 | MEDIUM | OPEN | Declaraciones de versión/dependencias superpuestas | Build | MP-09 |
| FND-010 | MEDIUM | CONTAINED | Superficie de OpModes sin separar commissioning/release | Release | MP-00/07/09 |
| FND-011 | HIGH | SUPERSEDED | `IntakeTeleOp` histórico puede dejar feeder energizado | Safety/mechanisms | FND-013 |
| FND-012 | LOW | OPEN | Conceptos/nombres legacy no reflejan hardware final | Architecture | MP-06/09 |
| FND-013 | CRITICAL | FIX_READY | `IntakeTeleOp` vigente añade shooter directo/latched | Safety/shooter | MP-01/07 |
| FND-014 | CRITICAL | FIX_READY | `TeleopTorreta` entrega shooter nulo a un comando | Safety/shooter | MP-01 |
| FND-015 | CRITICAL | CLOSED | Shooter falla abierto ante encoder/voltaje inválido | Shooter | MP-01/06 |
| FND-016 | HIGH | FIX_READY | `SafeCommandOpMode` no tiene init-loop repetitivo | Architecture/safety | MP-01 |
| FND-017 | HIGH | OPEN | Migración Pedro no poseía movimiento completo | Drive/localization | MP-02 |
| FND-018 | HIGH | OPEN | Feeder carece de pulso máximo y cooldown obligatorios | Mechanisms | MP-06 |
| FND-019 | HIGH | OPEN | Readiness omite velocidad lineal/angular del chasis | Shooter/drive | MP-06/T9 |
| FND-020 | HIGH | BLOCKED_PHYSICAL | Cero manual no detecta falso centro/reset/brownout | Turret/mechanical | MP-01/05 |
| FND-021 | HIGH | BLOCKED_PHYSICAL | Contrato físico incompleto y sin export RC | Hardware/safety | MP-00/01 |
| FND-022 | HIGH | OPEN | Limpieza posterior a aceptación no tenía revalidación | Test/release | MP-09/10 |
| FND-023 | MEDIUM | OPEN | Tags de rollback/final estaban incompletos o tardíos | Release | MP-00/09/10 |
| FND-024 | MEDIUM | OPEN | Gates ambiguos en muestras, tiempo y fallo ambiental | Test | MP-08/10 |
| FND-025 | MEDIUM | OPEN | Estados y poses iniciales no estaban sincronizados | Architecture/ops | MP-04/07 |
| FND-026 | HIGH | CONTAINED | Motor y CRServo del kicker no tienen sincronía mecánica; dual bloqueado, motor-only validado | Mechanical + mechanisms | MP-01/06 |
| FND-027 | MEDIUM | BLOCKED_PHYSICAL | Pulso de commissioning de torreta excede presupuesto de 8 ticks por corte entre ciclos | Software/test + turret | MP-01 |

## 5. Entradas iniciales históricas (`main@f91af18`)

### FND-001 — Cleanup de `VisionPortal` registrado antes de construirlo

- **Severidad / estado:** `CRITICAL / FIX_READY`
- **Fecha / autor:** 2026-07-15 / auditoría asistida
- **Baseline:** `main@f91af18`
- **Configuración/sesión:** revisión estática; no se ejecutó en hardware
- **Componentes:** `MainTeleOp`, `TeleopTorreta`, lifecycle de `VisionPortal`
- **Esperado:** construir el portal, registrar un cleanup válido y cerrarlo al detener el OpMode.
- **Observado:** el código registra `addResourceCleanup(visionPortal::close)` antes de que `visionPortal` reciba el resultado del builder.
- **Riesgo:** evaluar una referencia a método ligada a `null` puede lanzar `NullPointerException` durante init y dejar inoperable el TeleOp.
- **Evidencia:** call path inspeccionado en el baseline; confirmar líneas exactas en el SHA de implementación.
- **Acción propuesta:** prueba de init deshabilitada; construir antes de registrar o usar cleanup que compruebe null y ownership; test de init/stop repetido.
- **Criterio de cierre:** el caso reproduce/falla en baseline cuando sea seguro, pasa ≥10 ciclos init/stop, cierra portal y no deja recurso abierto.
- **Owner:** software vision.
- **Implementación MP-01A:** el portal se construye y asigna antes de registrar el cleanup ligado
  a la instancia creada. Si el builder falla, el lifecycle ejecuta `RobotSafety.stopAll()` sin
  desreferenciar un portal nulo.
- **Retest:** `assembleDebug` PASS en 16.8 s; faltan ciclos con cámara y el caso de dispositivo
  ausente. No cerrar sin evidencia física/logcat.

### FND-002 — Armado de torreta depende de un instante de initialize

- **Severidad / estado:** `HIGH / FIX_READY`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Configuración/sesión:** revisión estática
- **Componentes:** init de `MainTeleOp`/`TeleopTorreta`, `TurretSubsystem`
- **Esperado:** el operador centra y mantiene un chord durante 1 s en init loop con feedback.
- **Observado:** la condición `gamepad2.start && gamepad2.back` se consulta sólo una vez durante `initialize()`.
- **Riesgo:** ventana impráctica/no determinista; torreta puede quedar desarmada o fomentar atajos inseguros.
- **Acción:** máquina de estados `TURRET_CENTER -> ARM_HOLD -> ARMED`, cancelable y state-aware.
- **Criterio de cierre:** hold <1 s no arma; ≥1 s arma sólo en estado correcto; soltar cancela; ninguna potencia antes de arm; 20 repeticiones.
- **Owner:** software turret + operator 2.
- **Implementación MP-01A:** máquina monotónica `WAITING/HOLDING/ARMED` en
  `duringInitLoop()`; 1,000 ms producen una sola transición y muestran progreso.
- **Retest:** `assembleDebug` PASS en 16.8 s; faltan repeticiones en robot elevado.

### FND-003 — Soft limits de torreta no validados físicamente

- **Severidad / estado:** `HIGH / BLOCKED_PHYSICAL`
- **Fecha:** 2026-07-15
- **Baseline:** constantes actuales alrededor de -200/+200 ticks
- **Configuración/sesión:** sin medición física adjunta
- **Esperado:** límites representan arco seguro con margen contra hard stops/cables.
- **Observado:** no existe evidencia física en el repo sobre dirección, ticks/grado, centro o margen.
- **Medición/cálculo añadido 2026-07-18:** motor identificado como goBILDA 5203 223 RPM/26.9:1; ficha oficial=751.8 PPR en salida; transmisión externa reportada/fotografiada de 68 dientes motor a 198 dientes torreta. Resultado teórico=`6.0807 ticks/grado`; 200 ticks≈32.89°. El signo positivo ya se observó como giro horario de la torreta. Falta verificar ángulo real y margen mecánico en ambos sentidos.
- **Retest mecánico parcial:** con robot apagado y desde la marca central, el lado horario alcanzó aproximadamente 33° libremente, sin tensión de cables ni interferencias. Lado antihorario pendiente.
- **Retest mecánico bilateral:** el lado antihorario también alcanzó aproximadamente 33° libremente, sin tensión de cables ni interferencias, y volvió a la marca central. Falta demostrar margen mecánico medido más allá de ±33°; por ello ±200 aún no se cierra.
- **Margen reportado:** máximo manual aproximado ±125° desde centro (≈±760 ticks teóricos), lo que dejaría ≈92.11°/560 ticks de margen fuera del soft limit ±200 por lado. Pendiente precisar qué condición física definió ese “máximo” y ejecutar T4 motorizado sin acercarse a hard-stop.
- **Ticks pasivos medidos:** durante INIT, con salida de torreta en cero, el giro manual desde la marca central reportó +1070 ticks horarios y -988 ticks antihorarios. Frente al límite vigente ±200 hay al menos 870 ticks de margen reportado por el lado positivo y 788 por el negativo. La discrepancia con los ±125° estimados impide aceptar todavía una conversión física ticks/grado; falta lectura de retorno al centro y definición verificable de ambos extremos.
- **Retorno al centro:** al realinear exactamente la cinta después del recorrido, sin reiniciar, el encoder mostró -22 ticks. Se priorizarán ticks sobre grados, pero el offset confirma que hace falta caracterizar repetibilidad/backlash por sentido y volver a establecer cero únicamente con la marca física alineada.
- **Repetibilidad positiva:** una ida manual desde la cinta hasta aproximadamente +200 ticks y regreso exacto a la cinta terminó en -44 ticks, es decir, añadió otros -22 ticks de deriva. Antes de atribuirlo a holgura se debe comprobar si el encoder cambia estando completamente inmóvil; no se autoriza movimiento motorizado durante el diagnóstico.
- **Estabilidad en reposo:** la lectura permaneció en -44 ticks durante 2 minutos con la torreta inmóvil y alineada a la cinta. No se observó deriva eléctrica espontánea; el offset queda asociado provisionalmente a backlash/repetibilidad mecánica o alineación visual y deberá absorberse con margen/tolerancia, sujeto a la prueba desde el lado negativo.
- **Repetibilidad negativa:** desde la cinta en -44, una ida manual hasta aproximadamente -200 ticks y regreso horario a la misma marca terminó en -47, diferencia de sólo 3 ticks con potencia cero. El encoder queda estable y la repetibilidad local por ese lado es compatible con un T4 motorizado conservador; todavía falta ejecutar ese gate y no se amplían los límites actuales.
- **Riesgo:** contacto mecánico, daño de cables o bloqueo prematuro.
- **Acción:** fixture/marks, potencia ≤0.1, medir ambos sentidos, backlash y repetibilidad; revisión mecánica.
- **Criterio de cierre:** tabla firmada con ticks/grados, margen, signos y TEST-T4 aprobado sin tocar hard stop.
- **Owner:** mechanical + turret.

### FND-004 — Localización no tiene constantes aptas para auto-aim

- **Severidad / estado:** `HIGH / FIX_READY`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Componentes:** RR `ThreeDeadWheelLocalizer`, `pedroPathing/Constants.java`
- **Esperado:** tres pods físicos con nombres, offsets, signos, resolución y gains medidos.
- **Observado:** RR contiene offsets 0/1/0 ticks y Pedro contiene masa TODO, setters duplicados de offset, pods asociados a nombres de drive y conversiones provisionales.
- **Riesgo:** pose/distancia/bearing incorrectos; fusión engañosa.
- **Acción:** MP-02 completo; no activar aim con esos valores.
- **Criterio de cierre:** inventario físico y ruta repetida ≤2 in/2°, con logs y signs comprobados.
- **Owner:** localization.

### FND-005 — No existe un único interlock de feeder

- **Severidad / estado:** `HIGH / OPEN`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Componentes:** `SkywalkerProfile`, `KickerSubsystem`, adapters/actions, comandos de tiro
- **Esperado:** todos los requests pasan por `ReadinessInterlock`; soltar/interrumpir/Stop ordena cero.
- **Observado:** existen bindings y adapters que ordenan feeder/kicker directamente, separados de pose/turret/RPM readiness.
- **Riesgo:** lanzamiento no solicitado o fuera de condiciones.
- **Acción:** inventariar cada caller; crear API de request/permiso; eliminar power directo de bindings; tests de matriz T9.
- **Criterio de cierre:** búsqueda demuestra un solo camino de output; todos los casos T9 pasan; cero feed involuntario en dos sesiones.
- **Owner:** mechanisms/software safety.
- **Fix MP-01:** `KickerSubsystem` es el único owner de `kickerMotor` y `kickerServo`; sus adapters delegan en `kick/reverse/stop` y el binding de disparo activo exige `shooter.isReady()`. Falta matriz física; los interlocks completos de pose/velocidad pertenecen a MP-06.

### FND-006 — Espera de readiness sin timeout en secuencia alternativa

- **Severidad / estado:** `HIGH / FIX_READY`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Componentes:** `ShootSequenceCommand`
- **Esperado:** readiness con timeout/failure state e interruption cleanup.
- **Observado:** se conserva `WaitUntilCommand(shooter::isReady)` sin un límite superior explícito, aunque `ShootBurstCommand` ya contiene mejores protecciones.
- **Riesgo:** comando colgado y mecanismos en estado difícil de entender.
- **Acción:** retirar camino legacy o implementar timeout/state central; comprobar top-level `end`.
- **Criterio de cierre:** shooter que nunca llega a RPM termina/bloquea de forma segura dentro del timeout y deja outputs definidos.
- **Owner:** shooter.
- **Fix MP-01:** `ShootSequenceCommand` usa máquina de estados con timeout, aborta ante fault y siempre detiene kicker al terminar/interrumpirse. Falta regresión temporal.

### FND-007 — E-stop no es uniforme

- **Severidad / estado:** `MEDIUM / CONTAINED`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Esperado:** todo OpMode de producción/diagnóstico comparte latch, cancela scheduler y llama `RobotSafety.stopAll()`.
- **Observado:** `SafeCommandOpMode`/`RobotSafety` existen, pero el latch con `gamepad1.back` se observó de forma consistente sólo en `MainTeleOp`; otros modos habilitados no comparten la misma superficie.
- **Riesgo:** expectativas distintas al seleccionar un modo de prueba.
- **Acción:** base única + limpieza; test en Competition TeleOp/System Check.
- **Criterio de cierre:** producción, System Check y cada tuner autorizado pasan T4.1; los tuners pueden seguir visibles en commissioning según DEC-026.
- **Owner:** safety/release.
- **Contención MP-01:** sólo quedan habilitados los modos con `SafeCommandOpMode` que mueven actuadores; tuners y OpModes legacy sin lifecycle uniforme se deshabilitaron. Reabrir este finding al reactivar cualquier tuner en MP-02.

### FND-008 — Composición fragmentada

- **Severidad / estado:** `MEDIUM / OPEN`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Esperado:** `RobotContainer` posee drive, mechanisms, turret, camera y coordinators.
- **Observado:** `MainTeleOp` construye turret/VisionPortal fuera del container.
- **Riesgo:** lifecycle/telemetry/safety duplicados y difícil integración.
- **Acción:** mover ownership en MP-07 después de estabilizar interfaces; no duplicar mappings durante migración.
- **Criterio de cierre:** diagrama/call path demuestra un dueño por device y System Check reutiliza factory segura.
- **Owner:** architecture.

### FND-009 — Fuentes superpuestas de SDK/dependencias

- **Severidad / estado:** `MEDIUM / OPEN`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Esperado:** una versión efectiva fácil de identificar y compatible con `Limelight3A`.
- **Observado:** `TeamCode/build.gradle` declara SDK 10.3.0 mientras `build.dependencies.gradle` incluye SDK 11.0; Dashboard también aparece duplicado.
- **Riesgo:** desarrolladores verifican API contra versión equivocada; cleanup accidental rompe resolución.
- **Acción:** registrar dependency graph efectivo ahora; consolidar la versión ya validada sólo en MP-09, commit separado.
- **Criterio de cierre:** una fuente de versión, build limpio y smoke test, sin upgrade incidental.
- **Owner:** build/release.

### FND-010 — Superficie excesiva de OpModes

- **Severidad / estado:** `MEDIUM / CONTAINED`
- **Fecha:** 2026-07-15
- **Baseline:** 14 TeleOps anotados habilitados + tuning Pedro + registradores RR
- **Esperado:** catálogo explícito de commissioning; release final con un Competition TeleOp y un System Check.
- **Observado:** Driver Station puede presentar múltiples pruebas/variantes; autos están deshabilitados pero permanecen.
- **Riesgo:** seleccionar modo incorrecto y mantener safety policies divergentes.
- **Contención:** el plan no autoriza usar modos legacy como producción.
- **Acción:** conservar tuners necesarios, clasificar/bloquear modos no seguros y reducir el release en MP-09.
- **Criterio de cierre:** commissioning muestra sólo modos deliberados y el release revalidado muestra exactamente dos.
- **Owner:** release.

### FND-011 — Intake test puede dejar feeder energizado

- **Severidad / estado:** `HIGH / SUPERSEDED` por FND-013
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Componentes:** `IntakeTeleOp`
- **Esperado:** movimiento diagnóstico sólo mientras se sostiene el permiso y con timeout.
- **Observado:** comandos instantáneos asociados a A/B pueden dejar power del feeder hasta `DPAD_DOWN` u otra transición.
- **Riesgo:** motor sigue energizado cuando el operador espera un pulso.
- **Acción:** no usar como diagnóstico normal; reemplazar con System Check hold-to-run; revisar stop actual.
- **Criterio de cierre:** modo eliminado del release y System Check pasa stop/timeout/release.
- **Owner:** safety/mechanisms.

## 6. Entradas vigentes añadidas para `origin/main@b5a1342` y revalidadas en `a887fe4`

### FND-013 — `IntakeTeleOp` controla shooter de forma directa y latched

- **Severidad / estado:** `CRITICAL / FIX_READY`
- **Evidencia:** el commit integrado posterior a `f91af18` liga X/Y a `ShooterMotor.right2/left2` con potencia aproximada ±0.9; la salida permanece hasta `DPAD_DOWN` o Stop.
- **Riesgo:** shooter energizado fuera de health/readiness y con expectativa incorrecta de pulso.
- **Contención:** no usar `IntakeTeleOp` para commissioning normal; conservar visible sólo si se clasifica/bloquea expresamente hasta reemplazarlo por System Check.
- **Cierre:** ningún binding activa shooter/feeder directo; release/interrupt/E-stop ordenan cero dentro del gate y veinte repeticiones pasan.
- **Fix MP-01:** se eliminaron `ShooterMotor` y `KickerSubsystem` de `IntakeTeleOp`, el intake pasó a hold-to-run y el OpMode quedó `@Disabled`. Falta retest en Driver Station.

### FND-014 — `TeleopTorreta` programa un comando con shooter nulo

- **Severidad / estado:** `CRITICAL / FIX_READY`
- **Evidencia:** el baseline dejó de inicializar `ShooterMotor`, pero conserva un binding que construye `ShooterCommand2(shooterMotor, ...)`.
- **Riesgo:** `NullPointerException` o pérdida de control al disparar el binding.
- **Contención:** modo no autorizado para movimiento hasta corregir ownership o retirar el binding.
- **Cierre:** análisis de nullability/call path y prueba controlada demuestran owner válido o ausencia total del camino.
- **Implementación MP-01A:** se retiraron field, imports y scheduling huérfanos; no se volvió a
  mapear el shooter en este OpMode.
- **Retest:** `assembleDebug` PASS en 16.8 s; falta init/stop controlado.

### FND-015 — Shooter fail-open ante dato de sensor inválido

- **Severidad / estado:** `CRITICAL / CLOSED`
- **Evidencia:** RPM imposible (>20000) se devuelve como cero y voltaje ≤1 V se sustituye por 12.5 V; con target positivo, bang-bang/output clamp puede pedir power 1.0.
- **Riesgo:** encoder congelado/desconectado o voltaje inválido puede producir máxima salida.
- **Acción:** health explícito para encoder/velocidad/voltaje, timeout de ausencia de pulsos, overspeed y fallo cerrado a target/power cero.
- **Cierre:** fault injection de freeze, jump, NaN/imposible y voltaje inválido nunca eleva power y exige rearmado deliberado.
- **Fix MP-01:** health latched corta target/power ante voltaje inválido/no disponible, lectura no finita, encoder sin cambio antes del timeout, target fuera de 0..6000 u overspeed >6000 RPM. El rearme requiere reiniciar el OpMode.
- **Retest de cierre 2026-07-18:** con el APK SHA-256 `F2E17D94470EF452F97FA62BD49D84E125F619516BC1B223A991605BFC2FA1FB`, `SystemCheck` inyectó voltaje inválido, encoder congelado, RPM no finita y overspeed: `4/4 PASS`. Cada caso dejó target/power en cero, no produjo movimiento, permaneció latched frente a otra solicitud y sólo se limpió reiniciando el OpMode. La salida física del shooter estuvo bloqueada por compilación durante la prueba.
- **Contención previa al retest:** los presets del TeleOp principal no programan RPM; sólo System Check/Shooter Tuning pueden solicitar giro mediante hold-to-run.

### FND-016 — Lifecycle sin init-loop repetitivo

- **Severidad / estado:** `HIGH / FIX_READY`
- **Evidencia:** `SafeCommandOpMode.runOpMode()` ejecuta `initialize()` una vez, llama `waitForStart()` y sólo después entra al loop activo.
- **Riesgo:** hold de torreta, feedback y E-stop pre-start no pueden evaluarse correctamente.
- **Acción:** hook de init-loop cancelable antes de START, sin mover mecanismos y sin bloquear Stop.
- **Cierre:** hold <1 s no arma, ≥1 s arma en estado correcto, Stop durante init termina limpio y veinte repeticiones pasan.
- **Implementación MP-01A:** `SafeCommandOpMode` ejecuta `duringInitLoop()` antes de START,
  atiende BACK durante init/run, cancela scheduler, ordena `RobotSafety.stopAll()` y solicita Stop.
- **Retest:** `assembleDebug` PASS en 16.8 s; falta verificar ≤50 ms con Driver Station real.

### FND-017 — Migración Pedro incompleta

- **Severidad / estado:** `HIGH / OPEN`
- **Evidencia:** la arquitectura proponía `PoseProvider`, mientras `RobotContainer -> DriveSubsystem -> MecanumDrive` conservaba Road Runner como dueño del movimiento.
- **Riesgo:** dual-stack implícito, poses divergentes y rollback parcial.
- **Acción/cierre:** MP-02 migra pose, conducción, paths y stop como paquete; búsqueda/call path demuestra un solo dueño runtime.

### FND-018 — Feeder sin pulso/cooldown acotados

- **Severidad / estado:** `HIGH / OPEN`
- **Evidencia:** request sostenido e interlock no imponen por sí mismos máximo on-time por pieza.
- **Riesgo:** motor energizado indefinidamente, múltiples piezas o jam.
- **Acción:** estados `IDLE`, `WAITING_READINESS`, `PULSING`, `COOLDOWN`, `FAULT`; duración/power/cooldown son `TBD-BLOCKING`.
- **Cierre:** veinte secuencias no producen pulso extra y cada pérdida de permiso corta dentro de 50 ms.

### FND-019 — Readiness no bloquea robot en movimiento

- **Severidad / estado:** `HIGH / OPEN`
- **Evidencia:** criterios de tiros estacionarios no se reflejan en campos de readiness para velocidad lineal/angular.
- **Riesgo:** alimentar durante traslación/giro no caracterizados.
- **Acción/cierre:** medir umbrales, publicarlos en telemetry y probar ambos lados del límite sin feed indebido.

### FND-020 — Cero de torreta puede ser falso o perderse

- **Severidad / estado:** `HIGH / BLOCKED_PHYSICAL`
- **Evidencia:** cero manual depende de colocación humana; reset/brownout puede reiniciar encoder sin demostrar centro.
- **Evidencia física adicional 2026-07-18:** el equipo reporta que un toque físico pequeño produce cambios grandes de ticks; no existe aún conversión validada ticks/grado. La marca de cinta continúa siendo la única referencia física de centro.
- **Riesgo:** límites y setpoint desplazados, contacto mecánico o cables tensionados.
- **Acción:** marca/fixture, `zeroValid`, invalidación en cada init/reset/brownout, zona de frenado y contención exterior.
- **Retest parcial 2026-07-18:** después de un movimiento conocido de +29 ticks y posterior instalación/reinicio, el siguiente INIT reportó -2 ticks, `zero=INVALID_INIT` y cero movimiento. Esto demuestra invalidación fail-closed al reiniciar, pero también confirma que el encoder ya no permite regresar a la marca física; falso centro y brownout eléctrico real siguen pendientes.
- **Cierre:** tabla firmada y pruebas desde ambos sentidos sin hard-stop; un falso/reinicio bloquea movimiento.

### FND-021 — Falta contrato físico completo

- **Severidad / estado:** `HIGH / BLOCKED_PHYSICAL`
- **Evidencia:** no hay export RC versionado; faltan datos verificables de drive, pods, IMU, intake y Limelight además de mecanismos.
- **Riesgo:** mappings, signos, ejes o límites inventados.
- **Acción/cierre:** completar y firmar la guía 08; cada `TBD-BLOCKING` inhibe el componente afectado.

### FND-022 — El SHA posterior a cleanup no estaba plenamente validado

- **Severidad / estado:** `HIGH / OPEN`
- **Evidencia:** MP-08 exigía dos sesiones antes de MP-09, que hacía cambios sustanciales y sólo pedía smoke test.
- **Riesgo:** declarar release un commit diferente al aceptado.
- **Acción/cierre:** MP-10 repite T0–T10 y dos sesiones sobre el SHA limpio antes del tag final.

### FND-023 — Rollback y tag final incompletos

- **Severidad / estado:** `MEDIUM / OPEN`
- **Evidencia:** el tag material se creaba al cleanup y no existía tag final posterior a revalidación.
- **Acción/cierre:** verificar `baseline/pre-mp01-*`, `archive/pre-cleanup-*` y `release/competition-*` desde otro checkout en sus gates respectivos.

### FND-024 — Gates no reproducibles

- **Severidad / estado:** `MEDIUM / OPEN`
- **Evidencia:** “siguiente ciclo”, 9/10, 8/10 y “fallo ambiental documentado” permitían interpretaciones distintas.
- **Acción/cierre:** primer ciclo y ≤50 ms; diez tiros por distancia/sesión sin pooling; fallo de build bloquea el gate; odometría registra cinco repeticiones y distribución.

### FND-025 — Estados y poses iniciales ambiguos

- **Severidad / estado:** `MEDIUM / OPEN`
- **Evidencia:** enums de arquitectura/master divergían y no existía catálogo validado de poses iniciales.
- **Acción/cierre:** enums únicos, transiciones documentadas y catálogo con alianza/X/Y/heading/unidades/evidencia; todo preset no medido queda bloqueado.

## 7. Entrada histórica restante

### FND-012 — Terminología y hardware legado

- **Severidad / estado:** `LOW / OPEN`
- **Fecha:** 2026-07-15
- **Baseline:** `KickerSubsystem`, `ShooterHoodSubsystem`, `Shooter2`, string `kickerM otor`
- **Esperado:** conceptos reflejan feeder, shooter de un motor y hood fijo, sin romper mappings reales.
- **Observado:** nombres mezclan generaciones del robot.
- **Riesgo:** mantenimiento confuso o “corrección” accidental del hardware string.
- **Acción:** confirmar config; renombrar clases/conceptos por commits controlados; retirar legado en MP-09; preservar string hasta decisión explícita.
- **Criterio de cierre:** inventario final coincide con hardware/config y build; estudiantes pueden explicar naming exception si permanece.
- **Owner:** architecture/release.

### FND-026 — Kicker dual no cumple sincronía mecánica

- **Severidad / estado:** `HIGH / CONTAINED`
- **Fecha / autor:** 2026-07-17 / prueba física del equipo
- **Componente/owner:** `KickerSubsystem`; mechanical + mechanisms
- **Requisito:** `kickerMotor` y `kickerServo` deben iniciar y detener físicamente juntos; release/E-stop conservan salida cero inmediata.
- **Contención 2026-07-18:** DEC-037 mantiene `KICKER_SERVO_ENABLED=false`, servo físicamente desconectado y operación motor-only. El candidato motor-only pasó INIT, avance, reversa, release, Stop y E-stop sin piezas. La configuración dual permanece bloqueada hasta corrección mecánica, APK nuevo y regresión completa; reinstalar el servo no la autoriza por sí solo.
- **Precondiciones:** goBILDA `2000-0025-0002` programado en modo continuo, conectado a una llanta; motor en BRAKE; CRServo a ±0.5.
- **Esperado:** movimiento mecánico sincronizado en avance, reversa y stop.
- **Observado:** el servo tarda aproximadamente 0.5 s en iniciar y 0.5–1.0 s en detenerse; el motor responde antes. El servo finalmente se detiene, no zumba y el comportamiento aparece en ambos sentidos.
- **Impacto/riesgo:** pulso o alimentación adicional después de release/E-stop; posibilidad de doble alimentación o atasco.
- **Evidencia:** sesión física MP-01 registrada en `handoff-task.md`.
- **Hipótesis:** dinámica/inercia del CRServo y la llanta; el software ordena ambas salidas en el mismo método, pero eso no garantiza sincronía física.
- **Contención:** presets de shooter siguen inhibidos; no ejecutar matriz de 20 ciclos ni alimentar piezas.
- **Acción:** decisión mecánica. Preferencia: un solo actuador con transmisión común; alternativas requieren revisión de puertos, freno, corriente y parada.
- **Criterio de cierre:** 20/20 ciclos bajo configuración final con inicio/parada sincronizados dentro de una tolerancia acordada, sin retrasar la orden cero de release/E-stop.

### FND-027 — Presupuesto de ticks del pulso de torreta no es un límite duro

- **Severidad / estado:** `MEDIUM / BLOCKED_PHYSICAL`
- **Fecha / autor:** 2026-07-18 / commissioning físico del equipo
- **SHA / rama / dirty status:** APK SHA-256 `7DA3A5518979BF8C44787482B7B052F9724205B39A13D3836275B3E1CCB6FDD3`; `masterplan@24f9911`, worktree dirty
- **Componente/owner:** pulso histórico `TurretSubsystem.startCommissioningPulse`; reemplazo actual `requestCommissioningJog`; software/test + turret
- **Esperado:** un pulso cercano al centro se corta al primero de 8 ticks o 150 ms y termina en power cero.
- **Observado:** primer pulso terminó en +14 ticks sin desplazamiento visible; una repetición en el mismo sentido terminó en +29 ticks y produjo giro horario visible de la torreta. Ambos reportaron `STOPPED_TICK_BUDGET`, pulse inactive y power=0, sin ruido, vibración ni error.
- **Frecuencia/reproducibilidad:** 2/2 pulsos superaron 8 ticks en la muestra final.
- **Impacto/riesgo:** el polling del scheduler detecta el umbral después de cruzarlo; describir 8 ticks como recorrido máximo es incorrecto y no debe usarse para acercarse a un límite físico no validado.
- **Evidencia:** sesión y telemetría registradas en `handoff-task.md`.
- **Hipótesis:** el motor avanza varios ticks entre ciclos de control y la inercia/holgura agrega desplazamiento antes de que BRAKE estabilice.
- **Acción/contención:** la versión por polling y el candidato temporal de recuperación quedaron retirados. El nuevo candidato T4 exige marca física + confirmación manual de cero, usa hold-to-run con potencia máxima fija 0.05 dentro de -200/+200, timeout 2 s, watchdog 100 ms y corte por release/límite/pérdida de cero/Stop/E-stop; un corte exige soltar antes de reactivar. APK SHA-256 `D5AC6F1E11B42F5EC8E973AEB3132B359733A09CB6791809EA442D59E7557536`, build PASS; pendiente instalación y retest físico escalonado.
- **Retest inicial del candidato T4:** instalado; INIT sin armar permaneció inmóvil en -47 ticks con cero `INVALID_INIT`. Después de START, `DPAD_RIGHT` fue rechazado como `REJECTED_ZERO_INVALID`, power=0, move active=false, ticks sin cambio y cero ruido/vibración. Gate fail-closed inicial=PASS; faltan pruebas armadas, límites y paradas.
- **Primer movimiento T4 armado:** un toque de `DPAD_RIGHT` produjo giro horario y terminó en +12 ticks, `STOPPED_RELEASE`, power=0 y move active=false; release instrumentado en 2.265 ms, gate 50 ms PASS, sin ruido ni vibración. Falta estabilidad, repetición, límite y sentido negativo antes de cerrar.
- **Primer movimiento negativo T4:** desde +12 ticks, un toque de `DPAD_LEFT` giró antihorario y terminó en -50, delta=-62 ticks. Release sí terminó en power=0/move inactive, 2.726 ms y gate 50 ms PASS. La asimetría 12 vs 62 ticks por toques no temporizados impide atribuir todavía el resultado al mecanismo o al control; se pausa para medir estabilidad y después instrumentar duración real de mando antes de repetir.
- **Estabilidad tras lado negativo:** -50 ticks y power=0 permanecieron sin cambio durante 20 s, sin movimiento, ruido, vibración ni error. El stop queda estable; el siguiente diagnóstico debe registrar duración del hold y delta de ticks, no modificar potencia ni límites.
- **Criterio de cierre:** nueva implementación documenta correctamente su garantía, termina en cero y 20/20 pulsos por sentido permanecen dentro del límite físico acordado sin ruido, error ni acercamiento a hard-stops.

## 8. Plantilla para nuevas entradas

```markdown
### FND-XXX — Título corto

- **Severidad / estado:** `HIGH / OPEN`
- **Fecha / autor:**
- **SHA / rama / dirty status:**
- **Session/config/build:**
- **Componente/owner:**
- **Requisito o decisión relacionada:**
- **Precondiciones:**
- **Esperado:**
- **Observado:**
- **Frecuencia/reproducibilidad:**
- **Impacto/riesgo:**
- **Evidencia:** log, CSV, video, screenshot, file/line o test ID
- **Hipótesis (separada de hechos):**
- **Acción/contención:**
- **Criterio de cierre:**
- **Retest y resultado:**
- **Decisión asociada:**
```

## 9. Reglas de mantenimiento

- IDs nunca se reutilizan.
- No editar el resultado observado para que coincida con la explicación posterior.
- Separar hecho, inferencia e hipótesis.
- Si el baseline cambia, registrar nuevo SHA; no sobrescribir evidencia vieja.
- `CONTAINED` no equivale a `CLOSED`.
- Todo `WONT_FIX` enlaza una decisión con riesgo aceptado y responsable.
- Findings históricos permanecen en [critical-findings.md](../critical-findings.md); este archivo registra el programa actual.
- En release, ningún `CRITICAL` o `HIGH` puede quedar abierto/contained sin una excepción aprobada explícitamente; la política preferida es cero excepciones.
