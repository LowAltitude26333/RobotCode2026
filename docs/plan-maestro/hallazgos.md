# Registro de hallazgos

> Estado: registro vivo; entradas iniciales provienen de auditoría estática, no de pruebas físicas
> Baseline histórico inicial: `main@f91af18`; baseline de implementación: `origin/main@a887fe4f7ca9023eec6034a0db6b8d918c640ecc`
> Última actualización: 2026-07-23
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
| FND-001 | CRITICAL | CLOSED | Webcams retiradas del runtime activo; 20/20 init/stop sin lookup ni excepción | Software vision | MP-01 |
| FND-002 | HIGH | CLOSED | Chord repetitivo validado 20/20 corto y 20/20 completo | Software/Ops | MP-01 |
| FND-003 | HIGH | CLOSED | Límites `-983/+1070` y potencia `0.50→0.05` validados por el lead | Mechanical + turret | MP-01/05 |
| FND-004 | HIGH | CLOSED | Geometría, signos, escala y masa Pedro validados con T5 40/40 | Localization | MP-02 |
| FND-005 | HIGH | CONTAINED | Producción exige `shooter.isReady()`; interlock completo de pose/torreta/chasis pasa a MP-06 | Mechanisms | MP-06 |
| FND-006 | HIGH | CLOSED | Secuencias activas tienen timeout, intentos acotados, failure y cleanup | Shooter | MP-01/06 |
| FND-007 | MEDIUM | CONTAINED | E-stop no uniforme en OpModes habilitados | Safety | MP-01/07 |
| FND-008 | MEDIUM | OPEN | Ownership de torreta/visión fuera del container | Architecture | MP-07 |
| FND-009 | MEDIUM | OPEN | Declaraciones de versión/dependencias superpuestas | Build | MP-09 |
| FND-010 | MEDIUM | CONTAINED | Superficie de OpModes sin separar commissioning/release | Release | MP-00/07/09 |
| FND-011 | HIGH | SUPERSEDED | `IntakeTeleOp` histórico puede dejar feeder energizado | Safety/mechanisms | FND-013 |
| FND-012 | LOW | OPEN | Conceptos/nombres legacy no reflejan hardware final | Architecture | MP-06/09 |
| FND-013 | CRITICAL | CLOSED | `IntakeTeleOp` está deshabilitado y ya no posee shooter/kicker | Safety/shooter | MP-01/07 |
| FND-014 | CRITICAL | CLOSED | `TeleopTorreta` está deshabilitado y no contiene camino de shooter nulo | Safety/shooter | MP-01 |
| FND-015 | CRITICAL | CLOSED | Shooter falla abierto ante encoder/voltaje inválido | Shooter | MP-01/06 |
| FND-016 | HIGH | CLOSED | Init-loop, Stop y E-stop pre-START validados en hardware | Architecture/safety | MP-01 |
| FND-017 | HIGH | CLOSED | Pedro posee pose, movimiento, paths y stop en producción | Drive/localization | MP-02 |
| FND-018 | HIGH | CONTAINED | Feeder carece de pulso máximo y cooldown obligatorios | Mechanisms | MP-06 |
| FND-019 | HIGH | CONTAINED | Readiness omite velocidad lineal/angular del chasis | Shooter/drive | MP-06/T9 |
| FND-020 | HIGH | CLOSED | Gates de falso centro/reset aprobados por el lead | Turret/mechanical | MP-01/05 |
| FND-021 | HIGH | CONTAINED | Contrato MP-01 confirmado; calibración de pods y Limelight quedan en MP-02/03 | Hardware/safety | MP-02/03 |
| FND-022 | HIGH | OPEN | Limpieza posterior a aceptación no tenía revalidación | Test/release | MP-09/10 |
| FND-023 | MEDIUM | OPEN | Tags de rollback/final estaban incompletos o tardíos | Release | MP-00/09/10 |
| FND-024 | MEDIUM | OPEN | Gates ambiguos en muestras, tiempo y fallo ambiental | Test | MP-08/10 |
| FND-025 | MEDIUM | OPEN | Estados y poses iniciales no estaban sincronizados | Architecture/ops | MP-04/07 |
| FND-026 | HIGH | CLOSED | Configuración final motor-only; CRServo retirado por decisión del lead | Mechanical + mechanisms | MP-01/06 |
| FND-027 | MEDIUM | CLOSED | Autocorte temporal de torreta validado 10/10 por sentido; asimetría direccional aceptada | Software/test + turret | MP-01 |
| FND-028 | CRITICAL | CLOSED | Dirección, encoder, watchdog y autocorte del shooter validados; T8/carga quedan separados | Shooter + electrical | MP-01/06 |
| FND-029 | CRITICAL | INVESTIGATING | Encoder del shooter no cambia al giro manual en la configuración actual y discrepa de las cuentas bajo potencia | Shooter + electrical | MP-06/T8 |

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

- **Severidad / estado:** `HIGH / CLOSED`
- **Fecha:** 2026-07-15
- **Baseline:** límites anteriores `-200/+200 ticks`
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
- **Confirmación del lead 2026-07-20:** extremos observados `-993/+935 ticks`; se ordenan límites `-983/+925`, 10 ticks hacia el centro por lado. Falta retest motorizado de corte/power cero y revisión de cables/contacto antes de cerrar.
- **Riesgo residual explícito:** 10 ticks superan por sólo 1 tick la deriva postcorte máxima de 9 ticks observada en FND-027. No usar estos extremos en aim automático hasta pasar el retest a potencia reducida.
- **Actualización del lead 2026-07-20:** límite positivo final `+1070` con tolerancia ya aplicada; negativo permanece `-983`. Se autoriza potencia de recorrido `0.30`; el software reduce a `0.05` en una zona de 100 ticks antes de cada límite. El cambio de envelope exige retest FND-003 nuevo.
- **Aumento posterior 2026-07-20:** el lead sube la potencia de recorrido a `0.50`; se conserva aproximación `0.05` en los últimos 100 ticks. El APK `320C...D226` queda superseded y FND-003 requiere retest del nuevo candidato.
- **Cierre 2026-07-20:** el Test lead confirma aprobado el retest físico prescrito del envelope final `-983/+1070` con recorrido `0.50` y aproximación `0.05`; ambos sentidos cortaron en cero, permitieron regresar hacia el interior y no presentaron contacto ni tensión observada. No se atribuyen valores crudos no reportados.
- **Riesgo:** contacto mecánico, daño de cables o bloqueo prematuro.
- **Acción:** fixture/marks, potencia ≤0.1, medir ambos sentidos, backlash y repetibilidad; revisión mecánica.
- **Criterio de cierre:** tabla firmada con ticks/grados, margen, signos y TEST-T4 aprobado sin tocar hard stop.
- **Owner:** mechanical + turret.

### FND-004 — Localización no tiene constantes aptas para auto-aim

- **Severidad / estado:** `HIGH / CLOSED`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Componentes:** RR `ThreeDeadWheelLocalizer`, `pedroPathing/Constants.java`
- **Esperado:** tres pods físicos con nombres, offsets, signos, resolución y gains medidos.
- **Observado:** RR contiene offsets 0/1/0 ticks y Pedro contiene masa TODO, setters duplicados de offset, pods asociados a nombres de drive y conversiones provisionales.
- **Riesgo:** pose/distancia/bearing incorrectos; fusión engañosa.
- **Acción:** MP-02 completo; no activar aim con esos valores.
- **Criterio de cierre:** inventario físico y ruta repetida ≤2 in/2°, con logs y signs comprobados.
- **Cierre 2026-07-21:** los nombres/puertos, signos, escalas y offsets de los tres pods quedaron comprobados; la masa Pedro se corrigió de `5 kg` provisional a `8.5 kg` medida. T5 pasó `40/40`, sin deriva progresiva, con máximo global `0.787 in` radial y `1.650°` de heading, dentro del gate `≤2 in/≤2°`.
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
- **Grafo efectivo registrado 2026-07-22 (solo lectura, sin cambiar ninguna declaración):**
  `./gradlew.bat :TeamCode:dependencies --configuration debugCompileClasspath` (mismo
  `JAVA_HOME`/`GRADLE_USER_HOME` que para correr tests) muestra que **`11.0.0` gana
  siempre**: cada línea `10.3.0` de `TeamCode/build.gradle` aparece en el árbol resuelto
  como `org.firstinspires.ftc:<módulo>:10.3.0 -> 11.0.0`, y además existen constraints
  `{strictly 11.0.0} -> 11.0.0 (c)` para `Inspection`, `RobotCore`, `Blocks`,
  `RobotServer`, `Vision`, `Hardware`, `FtcCommon`, `OnBotJava` que fuerzan esa versión
  incluso si algo pidiera menos. En otras palabras: los ocho `implementation
  '...:10.3.0'` en `TeamCode/build.gradle:31-38` son declaraciones muertas hoy — no
  representan la versión que realmente compila ni corre. `Dashboard` no se investigó en
  este pase (no forma parte de este grafo `org.firstinspires.ftc:*`); sigue como
  duplicado sin confirmar. No se tocó ninguna versión declarada (DEC-022).

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

- **Severidad / estado:** `HIGH / CLOSED`
- **Evidencia:** la arquitectura proponía `PoseProvider`, mientras `RobotContainer -> DriveSubsystem -> MecanumDrive` conservaba Road Runner como dueño del movimiento.
- **Riesgo:** dual-stack implícito, poses divergentes y rollback parcial.
- **Acción/cierre:** MP-02 migra pose, conducción, paths y stop como paquete; búsqueda/call path demuestra un solo dueño runtime.
- **Implementación MP-02:** el runtime de producción quedó en `MainTeleOp -> RobotContainer -> PedroDriveSubsystem -> PedroDriveAdapter -> Follower`; una sola instancia de `Follower` posee motores, pose, paths, actualización y stop. Los tuners dinámicos de Road Runner no se registran.
- **Cierre físico 2026-07-21:** después de T5 `40/40`, el lead confirmó en el APK final con masa `8.5 kg`: reset de heading con START, release-stop, BACK E-stop y forward+turn, todos `PASS`. Se cierra el riesgo de ownership/lifecycle dual.

### FND-018 — Feeder sin pulso/cooldown acotados

- **Severidad / estado:** `HIGH / CONTAINED`
- **Evidencia:** request sostenido e interlock no imponen por sí mismos máximo on-time por pieza.
- **Riesgo:** motor energizado indefinidamente, múltiples piezas o jam.
- **Acción:** estados `IDLE`, `WAITING_READINESS`, `PULSING`, `COOLDOWN`, `FAULT`; duración/power/cooldown son `TBD-BLOCKING`.
- **Cierre:** veinte secuencias no producen pulso extra y cada pérdida de permiso corta dentro de 50 ms.
- **Contención de software 2026-07-22 (sin robot físico):** `safety/FeederPulseStateMachine.java`
  implementa `IDLE -> PULSING -> COOLDOWN` (mismo patrón testable, `nowNanos` explícito,
  que `TurretArmingStateMachine`); `KickerSubsystem.kick()` consulta la máquina y
  `KickerSubsystem.periodic()` la actualiza cada ciclo, ordenando cero fuera de
  `PULSING`. Así el corte a `KICKER_MAX_PULSE_MS` no depende de que el caller vuelva a
  invocar `kick()`; el cooldown impide reenergizar hasta completar
  `KICKER_COOLDOWN_MS`. `WAITING_READINESS` se dejó fuera a propósito: el readiness
  (`shooter.isReady()`) sigue siendo responsabilidad del caller (fix ya aceptado de
  FND-005), para no mezclar ownership de shooter y feeder (ver FND-008, no reabierto
  aquí). `KICKER_MAX_PULSE_MS=700`/`KICKER_COOLDOWN_MS=300` en `LowAltitudeConstants`
  quedan explícitamente `TBD-BLOCKING`, valores conservadores sin medir. Cubierto por
  `FeederPulseStateMachineTest` (corte automático por request y por update periódico,
  bloqueo en cooldown, cooldown cero y reset en `release()`);
  `:TeamCode:testDebugUnitTest` y `assembleDebug` PASS. No
  cierra el finding — falta el retest físico prescrito (20 secuencias, corte ≤50 ms) y
  los valores TBD-BLOCKING una vez Tuning los mida.

### FND-019 — Readiness no bloquea robot en movimiento

- **Severidad / estado:** `HIGH / CONTAINED`
- **Evidencia:** criterios de tiros estacionarios no se reflejan en campos de readiness para velocidad lineal/angular.
- **Riesgo:** alimentar durante traslación/giro no caracterizados.
- **Acción/cierre:** medir umbrales, publicarlos en telemetry y probar ambos lados del límite sin feed indebido.
- **Contención de software 2026-07-22 (sin robot físico):** el dato ya existía —
  `PedroDriveAdapter` calcula `vx`/`vy`/`omega` robot-céntricos hacia cada `PoseSnapshot`
  desde MP-02 — pero nada lo leía antes de disparar. Nueva clase pura
  `safety/ChassisMotionGate.isWithinShotEnvelope(PoseSnapshot)` implementa DEC-031
  (feeder solo con robot estacionario), fail-closed ante snapshot nulo/no usable.
  Cableada en `SkywalkerProfile.java` (gatilla junto a `shooter.isReady()` antes de
  `kickerSubsystem.kick()`) y en el constructor de ángulo fijo de `ShootBurstCommand`.
  El equipo confirmó que los ocho autos `Artifacts` intentan disparar detenidos al final
  del path; el scaffolding ya pasa el snapshot de Pedro antes de cada pieza. La salida
  física del shooter de producción permanece inhibida hasta MP-06, por lo que esto no
  habilita tiros. `MAX_SHOT_LINEAR_SPEED_INCHES_PER_SEC=1.0`/
  `MAX_SHOT_ANGULAR_SPEED_RADIANS_PER_SEC≈0.087 rad` (`LowAltitudeConstants`) quedan
  `TBD-BLOCKING`, conservadores. Cubierto por `ChassisMotionGateTest`;
  `:TeamCode:testDebugUnitTest` y `assembleDebug` PASS. No cierra el finding —
  falta publicar el umbral real medido por Tuning y el retest físico de ambos lados del
  límite.

### FND-020 — Cero de torreta puede ser falso o perderse

- **Severidad / estado:** `HIGH / CLOSED`
- **Evidencia:** cero manual depende de colocación humana; reset/brownout puede reiniciar encoder sin demostrar centro.
- **Evidencia física adicional 2026-07-18:** el equipo reporta que un toque físico pequeño produce cambios grandes de ticks; no existe aún conversión validada ticks/grado. La marca de cinta continúa siendo la única referencia física de centro.
- **Riesgo:** límites y setpoint desplazados, contacto mecánico o cables tensionados.
- **Acción:** marca/fixture, `zeroValid`, invalidación en cada init/reset/brownout, zona de frenado y contención exterior.
- **Retest parcial 2026-07-18:** después de un movimiento conocido de +29 ticks y posterior instalación/reinicio, el siguiente INIT reportó -2 ticks, `zero=INVALID_INIT` y cero movimiento. Esto demuestra invalidación fail-closed al reiniciar, pero también confirma que el encoder ya no permite regresar a la marca física; falso centro y brownout eléctrico real siguen pendientes.
- **Cierre:** tabla firmada y pruebas desde ambos sentidos sin hard-stop; un falso/reinicio bloquea movimiento.
- **Resultado 2026-07-20:** el lead confirma que todos los pasos prescritos pasaron: inicio sin armar, pérdida simulada y reinicio/power-cycle fuera de centro rechazaron movimiento con cero inválido y power cero. Se cierra FND-020; tras cambiar potencia/límites sólo se exige spot-check de no movimiento sin armar.

### FND-021 — Falta contrato físico completo

- **Severidad / estado:** `HIGH / BLOCKED_PHYSICAL`
- **Evidencia:** no hay export RC versionado; faltan datos verificables de drive, pods, IMU, intake y Limelight además de mecanismos.
- **Actualización del lead 2026-07-20:** mappings de odometría confirmados: `par0=rightFront`, `par1=leftFront`, `perp=rightBack`. Montaje físico del Control Hub confirmado `logo=RIGHT`, `USB=BACK`; se detectó que `MecanumDrive` declaraba incorrectamente `USB=UP` y se corrigió a `UsbFacingDirection.BACKWARD`. Falta retest físico del signo/heading con el nuevo candidato; esta corrección de software no valida por sí sola el IMU.
- **Retest IMU 2026-07-20:** evidencia `TUNEO IMU.xlsx` SHA-256 `817FFB58302D753B4A56B8FC4D2BFF2B77CEC01BD554B8793985B8A85CBB0DF6`. Resultado 10/10 PASS: antihorario positivo 5/5 (delta medio `+90.52°`, error máximo `1.2°`) y horario negativo 5/5 (magnitud media `92.42°`, error máximo `4.4°`), con tolerancia `5°`. Orientación y signo del IMU quedan validados.
- **Reconciliación 2026-07-21:** los mappings, signos, escalas y geometría base de los tres pods ya fueron medidos y el tuneo RR fue aceptado de forma pragmática. FND-021 deja de bloquear MP-01. La repetibilidad final y migración de ownership son gate de MP-02; nombre, extrínseca y pipeline de Limelight son gate de MP-03. El finding queda `CONTAINED` hasta cerrar esas fases.
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

- **Severidad / estado:** `HIGH / CLOSED`
- **Fecha / autor:** 2026-07-17 / prueba física del equipo
- **Componente/owner:** `KickerSubsystem`; mechanical + mechanisms
- **Requisito:** `kickerMotor` y `kickerServo` deben iniciar y detener físicamente juntos; release/E-stop conservan salida cero inmediata.
- **Contención 2026-07-18:** DEC-037 mantiene `KICKER_SERVO_ENABLED=false`, servo físicamente desconectado y operación motor-only. El candidato motor-only pasó INIT, avance, reversa, release, Stop y E-stop sin piezas. La configuración dual permanece bloqueada hasta corrección mecánica, APK nuevo y regresión completa; reinstalar el servo no la autoriza por sí solo.
- **Cierre 2026-07-20:** el lead confirmó configuración final sin servo. La bandera se vuelve constante de compilación `false`; el problema de sincronía dual sale del diseño final. Reinstalar un servo reabriría el finding como cambio nuevo.
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

- **Severidad / estado:** `MEDIUM / CLOSED`
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
- **Criterio histórico de cierre:** 20/20 pulsos por sentido. Queda superseded por DEC-039; no se reescribe como si nunca hubiera existido.
- **Candidato final probado:** `SystemCheckOpMode` conserva hold-to-run, potencia máxima fija `0.05`, watchdog `100 ms`, soft limits `-200/+200`, cero manual confirmado y Stop/E-stop; agrega autocorte de comparación a `850 ms` y bloqueo hasta soltar D-pad. APK SHA-256 `9C2F58F3327064A199BC9426A646C05550BDC3F462ACF90E3534851BAD9B38E0` (`9C2F`).
- **Gate aceptado 2026-07-20:** el Test lead aprobó explícitamente reducir el criterio a 10/10 por sentido. Los 20 pulsos terminaron `STOPPED_TIMEOUT`, power final `0`, `Move active=false`, parada completa, estabilidad durante la espera y cero anomalías. Duración `855.0–881.6 ms`; |delta| positivo `54–60 ticks`; |delta| negativo `59–72 ticks`; todos dentro de los límites acordados de `800–900 ms`, `|delta|<=75` y desplazamiento postcorte `<=80 ticks`.
- **Caracterización direccional:** sobre nueve parejas con precarga estandarizada, promedio horario/positivo `67.8 ticks/s` y antihorario/negativo `75.7 ticks/s`; negativo `11.6%` más rápido en promedio, con variación por pareja de `-1.1%` a `+25.5%`. Se acepta como limitación mecánica/control conocida; no justifica un multiplicador fijo ni declara resuelto el backlash.
- **Trazabilidad:** la matriz `PRUEBAS COMPLETAS.xlsx` fue revisada completa; faltan hora en pruebas 1, 2, 8 y 18, y batería en 1, 2, 6, 8, 18 y 19. Se conservan como `NO REGISTRADO`; no se inventan valores y no invalidan el gate de parada.
- **Cierre:** 10/10 por sentido `PASS` bajo el APK `9C2F`; cualquier cambio de APK, potencia, timeout, watchdog, soft limits, transmisión o configuración física reabre la validación.

### FND-028 — Shooter con dirección incorrecta y encoder sin respuesta

- **Severidad / estado:** `CRITICAL / CLOSED`
- **Fecha / autor:** 2026-07-20 / commissioning físico del equipo
- **SHA / rama / dirty status:** candidato esperado SHA-256 `0ADF7218167026BF63D1E7F58F55761F9CB63D4B72C4D224C26492A82D79A1F4`; `masterplan@fed3424`, worktree dirty; hash instalado no verificado independientemente
- **Componente/owner:** `ShooterSubsystem`, `RobotMap.SHOOTER_MOTOR_IS_INVERTED`, cableado/puerto de encoder; shooter + electrical
- **Precondiciones:** shooter firme/despejado, sin piezas, batería `11.61 V`, Safety operator con Stop; gate sin chord aprobado.
- **Esperado:** pulso único power `0.10`/`500 ms`, sentido físico de lanzamiento, ticks/RPM no cero y corte estable.
- **Observado:** `STOPPED_TIMEOUT` a `527.3 ms`; el mecanismo giró hacia dentro (sentido incorrecto), pero ticks quedaron `0 -> 0`, delta `0` y peak indicated RPM `0.0`. Batería del reporte `11.58 -> 11.58 V`; power final `0`, pulse inactive/consumed, parada completa, health=`HEALTHY`, fault=`none`, sin ruido/vibración/roce/calor/olor y RPM indicada `0` tras 20 s.
- **Impacto/riesgo:** el control cerrado no tiene feedback verificable y la dirección física es incorrecta; cualquier nuevo giro o alimentación queda bloqueado.
- **Causa de software confirmada para el health engañoso:** el pulso se corta a `500 ms`, mientras `monitorEncoderResponse()` usa `SHOOTER_READY_TIMEOUT_MS=2000`; por diseño, el pulso puede terminar con encoder inmóvil antes de que el fault se active.
- **Hipótesis física principal, aún no confirmada:** encoder desconectado, mal asentado o conectado a un puerto distinto; `RUN_WITHOUT_ENCODER` todavía permite leer posición, por lo que motor moviéndose con posición y velocidad exactamente cero no se explica por ese modo. La inversión vigente `true` contradice el sentido físico requerido observado, pero no se cambia hasta cerrar la inspección.
- **Inspección desenergizada:** motor reportado como goBILDA 5203 Series Yellow Jacket brushed DC `6000 RPM`; cable de encoder presente, firme, sin daño visible y conectado al puerto de encoder 3. El sentido correcto confirmado es hacia fuera. Esto descarta una desconexión obvia, pero no demuestra continuidad eléctrica ni funcionamiento del encoder/puerto.
- **Candidato diagnóstico pasivo:** `ShooterSubsystem.PHYSICAL_OUTPUT_ALLOWED=false` fuerza cero en el último punto antes de `motorLeader.set(...)` para todas las instancias; `SystemCheck` publica `Shooter/Encoder live` y rechaza cualquier chord como `REJECTED_OUTPUT_DISABLED`. APK SHA-256 `B4CBC43A7ABA4BD02FB3ADB6B9A49A33FEA8A0AD1054BE681DD088BA25050988`, 81,311,287 bytes, 32/32 tests y `assembleDebug` PASS; pendiente instalación/prueba manual.
- **Resultado pasivo:** con output allowed=`false` y power=`0`, giro manual hacia fuera produjo ticks `0 -> 28`; regreso hacia dentro volvió `28 -> 0`. `Actual RPM` alcanzó aproximadamente `85` durante el giro y volvió casi a cero; health=`HEALTHY`, fault=`none` y no hubo movimiento espontáneo. Encoder/cable/puerto/lectura quedan funcionales pasivamente; no se atribuye el primer `0 -> 0` a un encoder desconectado.
- **Candidato de hipótesis 2:** motor output cambia de invertido a normal para invertir el sentido físico; como FTC SDK 10.3.0 también invierte el signo reportado al cambiar `DcMotor.Direction`, el encoder FTCLib se invierte por separado para conservar RPM de salida positiva. Sólo la instancia de `SystemCheck` habilita output después de fijar cap `0.10`; las demás permanecen fail-closed. Un watchdog nuevo exige al menos un tick en `250 ms` o corta/latcha `ENCODER_FAULT`; autocorte total sigue en `500 ms`. APK SHA-256 `2A193852530748B23C8D8F2B3F27262F0134ADBB84B49D1DE61F00F7727D254D`, 81,352,643 bytes; 32/32 tests, `assembleDebug`, `git diff --check` y cero ReparsePoints PASS; no instalado ni probado físicamente.
- **Resultado hipótesis 2 a power 0.10:** gate sin chord `REJECTED_ARM_CHORD`, output allowed=`true`, applied power=`0`, pulse inactive/not consumed, health=`HEALTHY`, fault=`none` y cero movimiento. El pulso armado terminó `STOPPED_ENCODER_NO_RESPONSE`; el equipo reporta que la rueda no venció el umbral mecánico de arranque a esa potencia. No se recibieron todavía los demás campos del bloque para esta repetición.
- **Candidato de breakaway autorizado por Test lead:** cap sube a `0.50`, pulso total baja a `300 ms` y watchdog de al menos un tick baja a `150 ms`; se conservan un pulso por INIT, chord, dirección corregida, encoder positivo separado, health/fault, release/Stop/E-stop y bloqueo de todos los demás owners. APK SHA-256 `B2946F95B60FD0CF5736EA2CF8B7D24206852E6649487CC84873A745B7C0F7EE`, 81,311,759 bytes; 32/32 tests, `assembleDebug`, `git diff --check` y cero ReparsePoints PASS; no instalado ni probado físicamente.
- **Resultado breakaway `B294`:** `STOPPED_TIMEOUT` a `318.2 ms`, raw ticks `0 -> 4`, delta `4`, encoder responded=`true`, peak indicated RPM `85.7`, batería `11.31 -> 10.39 V`, power final reportado `0`, pulse inactive/consumed, health=`HEALTHY`, fault=`none`, parada completa y sin ruido/vibración/roce/olor/calor. La rotación física fue hacia dentro, por lo que la dirección candidata normal queda refutada. La caída de `0.92 V` con sólo cuatro ticks impide aumentar más potencia antes de corregir dirección/evaluar carga.
- **Candidato de dirección 3:** vuelve motor invertido=`true` y encoder FTCLib normal=`false`, combinación consistente con la prueba pasiva donde hacia fuera produjo `+28` ticks. Conserva power `0.50`, pulso `300 ms`, watchdog `150 ms` y todos los cortes. APK SHA-256 `3C0C8DC6A481254755FE492ABEDD238BA3C11DFD6BBAD7A3E4A5F7CAF7A56144`, 81,311,767 bytes; 32/32 tests, build y diff PASS; no instalado ni probado físicamente.
- **Retest dirección 3:** `STOPPED_TIMEOUT` a `310.5 ms`, ticks `0 -> 11`, delta `11`, encoder responded=`true`, peak indicated RPM `257.1`, batería `13.56 -> 13.23 V`, pulse inactive/consumed, health=`HEALTHY`, fault=`none` y RPM indicada `0` al finalizar. Giro físico hacia fuera, parada completa y cero ruido/vibración/roce/olor/calor. Dirección, encoder-response watchdog y corte quedan aprobados para este candidato; la caracterización RPM/corriente/carga sigue pendiente.
- **Candidato de caracterización RPM 1:** conserva target indicado `1000 RPM`, cap `0.50`, dirección aprobada, watchdog `150 ms`, un pulso por INIT y todos los cortes; extiende sólo el autocorte a `2000 ms`. Agrega RPM indicada justo antes del corte, batería mínima y máximo hold continuo dentro de tolerancia. APK SHA-256 `F847E8BBB747A7C505B5367DED73C4504D31FE805FF93B1C47B96E74A2DD9F21`, 81,312,043 bytes; 32/32 tests, build/diff/ReparsePoints PASS; no instalado ni probado físicamente.
- **Resultado RPM 1:** resultado transcrito por el operador como `SPOTED_TIMEOUT` (confirmación ortográfica exacta pendiente); duración `2027.2 ms`, ticks `0 -> 276`, encoder responded, peak `857.1 RPM`, end `771.4 RPM`, batería `13.31 -> 13.10 V`, mínima `12.97 V`, max ready hold `0.0 ms`, power final `0`, health=`HEALTHY`, fault=`none`, dirección hacia fuera, parada completa y cero anomalías. Cap `0.50` no alcanzó la banda de readiness `910–1090 RPM`.
- **Candidato RPM 2:** target `1000 RPM`, cap `0.75`, autocorte `3000 ms`, watchdog `150 ms`, un pulso por INIT y misma instrumentación/cortes. APK SHA-256 `09B23FD2ED57E30496D5D7FBB710F21FDD34A77255F1D414C8578BBE465DDAC1`, 81,312,039 bytes; 32/32 tests, build y diff PASS; no instalado ni probado físicamente.
- **Resultado RPM 2:** `STOPPED_TIMEOUT` a `3027.7 ms`, ticks `0 -> 555`, encoder responded=`true`, peak/end `1028.6/942.9 RPM`, batería `13.11 -> 12.94 V`, mínima `12.37 V`, max ready hold `125.6 ms`, power final `0`, pulse inactive/consumed, health=`HEALTHY`, fault=`none` y RPM final `0`. El giro fue hacia fuera, se detuvo completamente y no hubo ruido, vibración, roce, olor ni calor. El candidato alcanzó la banda alrededor de `1000 RPM`, pero no sostuvo los `250 ms` continuos exigidos por T8.
- **Aceptación del Test lead:** “Como Test lead, acepto cerrar FND-028 para MP-01 bajo el APK 09B2...DAC1. Dirección, encoder, watchdog y autocorte aprobados; estabilidad de 250 ms y pruebas con carga permanecen pendientes para MP-06/T8. Feeder bloqueado.”
- **Acción/contención:** no repetir los pulsos de dirección/RPM ni alimentar piezas. La estabilidad de control y carga permanecen separadas y pendientes para MP-06/T8; el cierre de este finding no habilita feeder.
- **Criterio de cierre:** encoder cuenta de forma coherente en prueba pasiva/diagnóstica, dirección física correcta, falta de respuesta produce fault antes del autocorte, power final cero y retest físico escalonado aprobado.

### FND-029 — Regresión/intermitencia de la lectura del encoder del shooter

- **Severidad / estado:** `CRITICAL / INVESTIGATING`
- **Fecha / autor:** 2026-07-23 / commissioning físico del equipo
- **Rama / dirty status:** `masterplan@f272792`, worktree deliberadamente dirty antes de este commit.
- **Componente/owner:** Yellow Jacket 5203 1:1 de 6000 RPM, adaptador/cable de encoder, puerto REV y lectura `DcMotorEx`; shooter + electrical.
- **Requisito o decisión relacionada:** MP-06/T8.1 exige feedback de velocidad coherente antes de ajustar `kS`, `kV` o `kP`. Feeder, piezas, `1500+ RPM` y `3600 RPM` permanecen bloqueados.
- **Configuración física confirmada por el equipo:** un motor, reducción interna 1:1 y transmisión motor→rueda del shooter 1:1. El software corrige `SHOOTER_GEAR_RATIO` heredado de `2.0` a `1.0`; `28 ticks/rev` y `6000 RPM` permanecen.
- **Esperado:** con potencia cero y el OpMode activo, girar el eje/rueda manualmente debe cambiar la posición acumulada. Diez vueltas completas deben producir aproximadamente `280 ticks` y el signo outward debe ser positivo.
- **Observado:** en repetidas pruebas manuales el eje del motor gira físicamente, `Shooter/LoopCount` avanza, pero `Encoder live`, posición cruda, posición outward y velocidades permanecen en cero. Bajo pulsos motorizados anteriores sí aparecieron cuentas (`72`, `80`, `92` o `93` ticks netos según intento) y picos instantáneos erráticos; la discrepancia sigue presente.
- **Reproducibilidad:** manual `0 ticks` repetido antes y después de reiniciar el OpMode. Un candidato posterior forzó hubs REV a `AUTO` y limpió bulk cache cada ciclo; el operador volvió a reportar todos los valores del encoder en cero y sólo `LoopCount` cambiante.
- **Hipótesis de caché:** `REJECTED` para este bug. El workaround fue retirado; no se conserva código especulativo en `SystemCheck`.
- **Hipótesis de “encoder corrupto”:** `NO DEMOSTRADA`. La posición motorizada acumuló cuentas monotónicas en una captura y no autoriza atribuir la causa al encoder sin aislar cable, adaptador y puerto.
- **Aclaración de ciclo de vida (no confundir con FND-029):** durante el loop de INIT (antes de PLAY), `SafeCommandOpMode` no corre el `CommandScheduler`, así que `ShooterSubsystem.periodic()` nunca se ejecuta y toda la telemetría de posición/velocidad queda congelada por diseño, sin importar si se gira el eje a mano. Esto es comportamiento esperado, distinto de FND-029. La reproducción válida de este finding es únicamente **post-PLAY**, con `Shooter/LoopCount` avanzando y potencia en `0`.
- **Relación con FND-028:** FND-028 permanece como cierre histórico válido para el APK `09B2...DAC1`; su evidencia no se transfiere al candidato/configuración actual. FND-029 bloquea el gate vigente.
- **Impacto/riesgo:** las RPM instantáneas, readiness y cualquier ajuste de control pueden usar feedback incompleto o intermitente. Continuar energizando o “filtrar” la señal podría ocultar la falla y producir overspeed o control no determinista.
- **Contención:** shooter sin piezas y sin nuevos pulsos; no modificar gains; no habilitar feeder, autos de tiro ni targets mayores. La salida de producción permanece inhibida.
- **Instrumentación nueva:** `DIAG: Shooter Encoder RAW` mapea únicamente `RobotMap.SHOOTER_MOTOR` como `DcMotorEx`, desactiva bulk caching, mantiene potencia hardcoded en `0.0`, no usa FTCLib/scheduler/PID/FF/reset continuo y publica conexión física, modo SDK, ticks/velocidad raw y outward, deltas, min/max y número de muestras distintas.
- **Candidato RAW no instalado:** APK `TeamCode-debug.apk`, `81,370,478` bytes, SHA-256 `FE7A859729BB03681EB630075AF6886EC80D3661FE29EE06DD3EEECD61B4B1AE`; `74/74` tests JVM, `assembleDebug` y `git diff --check` PASS.
- **Plan de aislamiento:** (0) antes de la matriz completa, reasentar el conector del encoder (desconectar y volver a insertar hasta el tope) y repetir las diez vueltas — es la causa más reportada en la comunidad FTC/REV para "encoder no cuenta" (conector JST no asentado o cable dañado; gm0 y Chief Delphi lo documentan) y el paso más barato antes de abrir la matriz; (1) ejecutar diez vueltas en el OpMode RAW; (2) si continúa en cero, con robot apagado intercambiar adaptador/cable por uno conocido, probar encoder conocido en el puerto shooter y encoder shooter en puerto conocido; (3) medir alimentación de 3.3 V y transición de canales A/B usando el pinout oficial; (4) corregir sólo el componente demostrado. Ningún hilo externo reproduce este síntoma exacto; reasentar el conector es la apuesta más barata primero, no una causa confirmada.
- **Criterio de cierre:** `280±1 ticks` por diez vueltas en ambos sentidos, signo outward correcto, lectura repetible después de reiniciar, prueba abierta con corriente/voltaje/RPM coherentes y regresión T8.1 `1000±100 RPM` sostenida `>=250 ms`, sin fault/overspeed ni salida residual.
- **Retest y resultado:** pendiente; el APK RAW no ha sido instalado ni probado físicamente.

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
