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
| FND-029 | CRITICAL | CLOSED | Cable/adaptador reemplazado; RAW bidireccional, signo y retest energizado de 1000 RPM aprobados | Shooter + electrical | MP-06/T8 |

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

- **Severidad / estado:** `CRITICAL / CLOSED — T8.0 RAW PASS; T8.1 1000 RPM PASS`
- **Fecha / autor:** 2026-07-23 / commissioning físico del equipo
- **Rama / dirty status:** `masterplan@3207599`, worktree deliberadamente dirty antes de esta documentación.
- **Componente/owner:** Yellow Jacket 5203 1:1 de 6000 RPM, adaptador/cable de encoder, puerto REV y lectura `DcMotorEx`; shooter + electrical.
- **Requisito o decisión relacionada:** MP-06/T8.1 exige feedback de velocidad coherente antes de ajustar `kS`, `kV` o `kP`. Feeder, piezas, `1500+ RPM` y `3600 RPM` permanecen bloqueados.
- **Configuración física confirmada por el equipo:** un motor, reducción interna 1:1 y transmisión motor→rueda del shooter 1:1. El software corrige `SHOOTER_GEAR_RATIO` heredado de `2.0` a `1.0`; `28 ticks/rev` y `6000 RPM` permanecen.
- **Esperado:** con potencia cero y el OpMode activo, girar el eje/rueda manualmente debe cambiar la posición acumulada. Diez vueltas completas deben producir aproximadamente `280 ticks` y el signo outward debe ser positivo.
- **Observado:** en repetidas pruebas manuales el eje del motor gira físicamente, `Shooter/LoopCount` avanza, pero `Encoder live`, posición cruda, posición outward y velocidades permanecen en cero. Bajo pulsos motorizados anteriores sí aparecieron cuentas (`72`, `80`, `92` o `93` ticks netos según intento) y picos instantáneos erráticos; la discrepancia sigue presente.
- **Reproducibilidad:** manual `0 ticks` repetido antes y después de reiniciar el OpMode. Un candidato posterior forzó hubs REV a `AUTO` y limpió bulk cache cada ciclo; el operador volvió a reportar todos los valores del encoder en cero y sólo `LoopCount` cambiante.
- **Hipótesis de caché:** `REJECTED` para este bug. El workaround fue retirado; no se conserva código especulativo en `SystemCheck`.
- **Hipótesis de “encoder corrupto”:** `REJECTED` para el síntoma de conteo congelado. Después de reasentar el conector, el mismo encoder produjo `280 ticks` exactos y continuos en ambos sentidos. Esto demuestra funcionamiento bidireccional del encoder; no demuestra por separado si el contacto intermitente estaba en el terminal del conector o dentro del cable.
- **Aclaración de ciclo de vida (no confundir con FND-029):** durante el loop de INIT (antes de PLAY), `SafeCommandOpMode` no corre el `CommandScheduler`, así que `ShooterSubsystem.periodic()` nunca se ejecuta y toda la telemetría de posición/velocidad queda congelada por diseño, sin importar si se gira el eje a mano. Esto es comportamiento esperado, distinto de FND-029. La reproducción válida de este finding es únicamente **post-PLAY**, con `Shooter/LoopCount` avanzando y potencia en `0`.
- **Relación con FND-028:** FND-028 permanece como cierre histórico válido para el APK `09B2...DAC1`; su evidencia no se transfiere al candidato/configuración actual. FND-029 bloquea el gate vigente.
- **Impacto/riesgo:** las RPM instantáneas, readiness y cualquier ajuste de control pueden usar feedback incompleto o intermitente. Continuar energizando o “filtrar” la señal podría ocultar la falla y producir overspeed o control no determinista.
- **Contención:** conector reasentado hasta el tope y T8.0 RAW aprobado. No se modificaron gains, filtros ni `ShooterSubsystem`; no se ejecutaron pulsos, feeder, piezas ni autos durante esta sesión. Si la lectura vuelve a congelarse, reemplazar primero cable/adaptador y ejecutar la matriz cruzada antes de cualquier cambio de software.
- **Instrumentación nueva:** `DIAG: Shooter Encoder RAW` mapea únicamente `RobotMap.SHOOTER_MOTOR` como `DcMotorEx`, desactiva bulk caching, mantiene potencia hardcoded en `0.0`, no usa FTCLib/scheduler/PID/FF/reset continuo y publica conexión física, modo SDK, ticks/velocidad raw y outward, deltas, min/max y número de muestras distintas.
- **Candidato RAW instalado y probado:** APK `TeamCode-debug.apk`, `81,370,478` bytes, SHA-256 de la compilación de sesión `EE726B93482F23BD99FA4E998BE2263D261E043952F6198D17FA9AF46A0841A2`; `74/74` tests JVM y `assembleDebug` PASS.
- **Plan de aislamiento ejecutado:** se apagó el robot, se desconectó y volvió a insertar hasta el tope el conector del encoder sin cambiar cable, puerto ni encoder. Después del único cambio físico, el diagnóstico RAW recuperó conteo exacto y bidireccional. La condición para abrir la matriz cruzada (`delta=0`, discontinuidad o lectura unidireccional después del reasentado) no ocurrió; por ello no se intercambiaron componentes ni se midieron 3.3 V/canales A-B.
- **Criterio de cierre:** `280±1 ticks` por diez vueltas en ambos sentidos, signo outward correcto, lectura repetible después de reiniciar, prueba abierta con corriente/voltaje/RPM coherentes y regresión T8.1 `1000±100 RPM` sostenida `>=250 ms`, sin fault/overspeed ni salida residual.
- **Retest y resultado (2026-07-23):** en INIT se observó `Diag/Connection=USB, module 2`, bulk caching `OFF`, commanded power `0.0` y raw inicial `-1626`. La primera pasada hacia fuera dio raw/outward `-282/+282`, `Changed samples=282` y min/max `-2717..-2345`; se descartó como medición de gate porque el operador confirmó que reacomodó la llanta después de PLAY. Con baseline limpio y reinicio del OpMode, diez vueltas en sentido contrario dieron raw/outward `+280/-280`, `Changed samples=280` y min/max `-2730..-2450`. Una segunda pasada limpia hacia fuera dio raw/outward `-280/+280`, `Changed samples=288` y min/max `-2730..-2450`. Los deltas y rangos limpios cumplen `280±1` en ambos sentidos y el signo outward es correcto.
- **Confirmación de reemplazo físico:** antes del retest energizado, el operador confirmó que el cable/adaptador del encoder ya había sido sustituido. El encoder queda exonerado para el síntoma congelado; la categoría culpable fue el camino `CABLE/CONECTOR`.
- **Retest energizado previo a corrección de signo:** un pulso de `3022.9 ms` produjo movimiento continuo hacia fuera, delta reportado `-1454 ticks`, tres estimadores continuos y superpuestos, peak/end `-1542.9 RPM`, ready hold `0.0 ms`, batería `12.44 -> 11.88 V` con mínima `11.63 V`, health `HEALTHY`, fault `none`, power final `0` y safety gate `PASS`. El operador confirmó stop completo y ausencia de ruido, vibración, roce, olor o calentamiento.
- **Causa software separada demostrada:** el target era `+1000 RPM`, pero las tres rutas de feedback resultaron negativas. FTCLib `Motor.setInverted(true)` configura `DcMotor.Direction.REVERSE`, y el SDK ya cambia el signo de posición/velocidad; `ShooterSubsystem` volvía a aplicar `SHOOTER_ENCODER_IS_INVERTED=true`, creando doble inversión. No es evidencia para cambiar PID/gains/filtros.
- **Corrección candidata:** la ruta activa calcula la inversión residual como XOR entre el signo raw y la dirección del motor; el diagnóstico RAW conserva el signo físico directo. No cambian motor direction, ratio, target, PID/FF, filtros, límites, watchdogs, timeout ni stop. APK `TeamCode-debug.apk`, `81,336,915` bytes, SHA-256 `3D579E639619A1D4E15578193C37109B352D29AB23FF4B05E478453C4DAC5B12`; `76/76` tests JVM y `assembleDebug` PASS.
- **Retest de signo corregido:** pulso de `3022.9 ms`, movimiento continuo hacia fuera, delta `+1030 ticks`, tres estimadores positivos y coherentes, peak/end `900/814.3 RPM`, batería `12.32 -> 11.79 V` con mínima `11.58 V`, health `HEALTHY`, fault `none`, power final `0`, safety gate `PASS`, stop completo y sin síntomas mecánicos. La corrección de signo pasa; el gate de control no pasa porque ready hold quedó en `0.0 ms`.
- **Ajuste 1/2 de T8.1:** con `kV=0.003`, `kS=2.5` y `kP=0.00075`, el PIDF se estabilizó cerca de `850 RPM`, coincidiendo con la frontera bang-bang de error `150 RPM`. Se cambió únicamente `SHOOTER_KV` a `0.00365`; el Dashboard confirmó ese valor cargado. Retest: pulso `3012.9 ms`, delta `+1051 ticks`, peak/end `900/900 RPM`, ready hold `0.0 ms`, batería `12.27 -> 12.9 V` reportada con mínima `11.50 V`, health `HEALTHY`, fault `none`, power final `0`, safety gate `PASS`, stop completo y sin síntomas mecánicos. El ajuste elevó la meseta pero no pasó el gate.
- **Ajuste 2/2 de T8.1:** se cambia únicamente `SHOOTER_KV` de `0.00365` a `0.0044`; kS/kP/kI/kD, bang-bang, ratio, target, límites y stops permanecen. La proyección usa la meseta observada con aproximadamente `6.2 V` de comando para estimar cerca de `7.0 V` a `1000 RPM`, aún por debajo del cap `0.75`. APK `TeamCode-debug.apk`, `81,336,923` bytes, SHA-256 `FE31564532DBFE7AD1C9E398CBBC5653309C92FA59FAB3C1F4553674D33CE4EB`; `76/76` tests JVM y `assembleDebug` PASS. El retest siguiente aprobó este candidato; no realizar un tercer ajuste sin reabrir el análisis.
- **Retest final de cierre:** con Dashboard confirmando `SHOOTER_KV=0.0044`, pulso de `3034.6 ms`, delta `+1182 ticks`, peak/end `1071.4/1028.6 RPM`, ready hold `1712.7 ms`, batería `12.29 -> 11.94 V` con mínima `11.52 V`, health `HEALTHY`, fault `none`, power final `0` y safety gate `PASS`. El operador confirmó giro continuo hacia fuera, stop completo y ausencia de ruido, vibración, roce, olor o calentamiento. La gráfica muestra Actual/SDK/window positivos, coherentes y dentro de `1000±100 RPM` por más de `250 ms`.
- **Escalón T8.1 1500 RPM:** pulso de `3023.9 ms`, delta `+1536 ticks`, peak/end `1585.7/1585.7 RPM`, ready hold `1093.5 ms`, batería `12.22 -> 12.15 V` con mínima `11.42 V`, health `HEALTHY`, fault `none`, power final `0` y safety gate `PASS`. El operador confirmó giro continuo hacia fuera, stop completo y ausencia de ruido, vibración, roce, olor o calentamiento. La gráfica muestra las tres rutas coherentes dentro de `1500±100 RPM` durante más de `250 ms`: `PASS`.
- **Escalón T8.1 2000 RPM:** pulso de `3010.0 ms`, delta `+1550 ticks`, peak/end `1671.4/1628.6 RPM`, ready hold `0.0 ms`, batería `12.17 -> 11.67 V` con mínima `11.38 V`, health `HEALTHY`, fault `none`, power final `0` y safety gate `PASS`. El operador confirmó giro continuo hacia fuera, stop completo y ausencia de ruido, vibración, roce, olor o calentamiento. La gráfica muestra feedback coherente pero todavía acelerando al autocorte, sin entrar en `2000±100 RPM`: `BLOCKED`.
- **Análisis del bloqueo 2000:** durante todo el pulso el error permaneció por encima del threshold `150 RPM`, por lo que la rama bang-bang pidió `14 V` y `applyPower()` quedó limitada por el cap de commissioning `0.75`. En este tramo kS/kV/kP no determinan la salida; otro ajuste de gains no corrige el resultado. Alcanzar el target exigiría cambiar el envelope físico de commissioning (más tiempo o mayor cap), lo cual no se autoriza implícitamente.
- **Siguiente diagnóstico autorizado:** caracterización temporal open-loop a potencia fija `0.75` durante máximo `5000 ms`, un pulso por INIT y hold-to-run con `gamepad1 A`. No usa PID/feedforward/bang-bang y conserva el cap `0.75`, watchdog de encoder `150 ms`, overspeed absoluto `6000 RPM` y todos los caminos de stop. Su único objetivo es distinguir tiempo de aceleración de techo físico; no aumenta alcance ni autoriza piezas, feeder o escalones superiores.
- **Candidato open-loop instalado:** APK `TeamCode-debug.apk`, `81,337,103` bytes, SHA-256 `3F3F388B2FD53CD7DBD951D83C126772E319FEF85C8EF0B57F994BDAF4FAC5D6`; `76/76` tests JVM, cero failures/errors/skips y `assembleDebug` PASS. Durante el pulso publica `Shooter/Target=0` y `Shooter/ControlMode=OPEN_LOOP` para evitar interpretar la referencia informativa de `2000 RPM` como control cerrado.
- **Resultado open-loop 0.75:** `STOPPED_TIMEOUT`, pulso `5048.9 ms`, delta `+3117 ticks`, peak/end `1800.0/1800.0 RPM`, batería `12.11 -> 11.81 V` con mínima `11.37 V`, health `HEALTHY`, fault `none`, power final `0` y safety gate `PASS`. El operador confirmó giro continuo hacia fuera, stop completo y ausencia de ruido, vibración, roce, olor o calentamiento. Actual/SDK/window fueron coherentes y la gráfica se aproximó a una meseta de `1750–1800 RPM`.
- **Conclusión del envelope 0.75:** extender el pulso de `3000` a `5000 ms` elevó el peak sólo de `1671.4` a `1800.0 RPM`; `2000 RPM` no es alcanzable con el cap actual y no debe compensarse con otro cambio de gains. Un nuevo intento exige decisión explícita entre certificar un techo de `1800 RPM` o abrir un envelope de potencia superior; `2450/2900 RPM`, piezas y feeder siguen bloqueados.
- **Nuevo envelope autorizado:** el lead autorizó una única caracterización open-loop a potencia fija `0.90` durante máximo `5000 ms`. No cambia PID/FF, relación 1:1, watchdogs, overspeed ni stops; piezas, feeder, kicker y escalones superiores permanecen bloqueados. APK `TeamCode-debug.apk`, `81,337,075` bytes, SHA-256 `1247F3D33B99897C9A9D0B314601E3467F03121F4275AC5E080878D460AB47EB`; `76/76` tests JVM, cero failures/errors/skips y `assembleDebug` PASS. Pendiente instalación y prueba física.
- **Resultado open-loop 0.90:** `STOPPED_TIMEOUT`, pulso `5018.8 ms`, delta `+3963 ticks`, peak/end `2314.3/2314.3 RPM`, batería `12.07 -> 11.45 V` con mínima `11.07 V`, health `HEALTHY`, fault `none`, power final `0` y safety gate `PASS`. El operador confirmó giro continuo hacia fuera, stop completo y ausencia de ruido, vibración, roce, olor o calentamiento. Actual/SDK/window fueron coherentes.
- **Conclusión del envelope 0.90:** `2000 RPM` es físicamente alcanzable sin ampliar nuevamente la potencia. El siguiente gate es una sola corrida closed-loop a `2000 RPM`, cap `0.90`, máximo `5000 ms`, con las ganancias congeladas; no autoriza aún `2450/2900 RPM`, piezas ni feeder. APK candidato `TeamCode-debug.apk`, `81,336,971` bytes, SHA-256 `A4EFBC673E35A6857D0A36C6145560EFAF3FD1713456E6B7F588341A3296E73A`; `76/76` tests JVM, cero failures/errors/skips y `assembleDebug` PASS.
- **Retest T8.1 2000 RPM aprobado:** `STOPPED_TIMEOUT`, pulso `5018.3 ms`, delta `+3871 ticks`, peak/end `2100.0/1971.4 RPM`, ready hold `2357.7 ms`, batería `12.04 -> 11.58 V` con mínima `10.99 V`, health `HEALTHY`, fault `none`, power final `0` y safety gate `PASS`. El operador confirmó giro continuo hacia fuera, stop completo y ausencia de ruido, vibración, roce, olor o calentamiento. Actual/SDK/window permanecieron coherentes y el gate `2000±100 RPM` durante `>=250 ms` queda `PASS`.
- **Una pieza autorizada:** el lead autorizó una sola pieza manual a `2000 RPM`, sin feeder automático. El candidato exige ready continuo `>=250 ms` inmediatamente antes de aceptar un único `RIGHT_BUMPER` por INIT; usa el kicker motor-only existente a `0.85` durante `632 ms`, con corte duro redundante `700 ms`, cooldown `300 ms`, BRAKE y shutdowns. Intake sin bindings y servo kicker deshabilitado. La prueba será sólo hacia red/backstop para medir alimentación y caída/recuperación de RPM; no certifica distancia, precisión, hood, feeder ni `GOAL_AUTO`. APK `TeamCode-debug.apk`, `81,371,839` bytes, SHA-256 `83EFA383DA021B86168FBB376C5080674A1B39A81338B266B200FCE8E2AD2726`; `76/76` tests JVM, cero failures/errors/skips, `assembleDebug` y `git diff --check` PASS. Pendiente instalación y prueba física.
- **Primer intento de una pieza, evidencia incompleta:** el operador reportó que `Shot/Armed` nunca llegó a `true` y que el kicker se movió por muy poco tiempo. Ambos datos no pueden atribuirse todavía al mismo camino: con `Armed=false`, el candidato rechaza el request y no llama `kicker.kick()`. Se requieren los campos persistentes `Shot/Result`, `Shot/Duration`, `Shot/Consumed`, `Shooter/Max ready hold`, `Shooter/Current ready hold` y `Shot/Kicker state` para distinguir rechazo, una ventana transitoria de readiness o instalación del APK anterior. No se cambia software ni se amplía el pulso a 5 s sin esa evidencia.
- **Causa del primer intento aclarada:** el operador confirmó que presionó `RIGHT_BUMPER` antes de ready. El request fue prematuro y, por contrato del candidato, debía rechazarse; por tanto no constituye una medición válida del pulso `632 ms`. Se ordena repetir con el mismo APK, manteniendo `A` y esperando `Shot/Armed=true` antes del único request.
- **Repetición válida con una pieza:** `Shot/Result=COMPLETED`, duración `658.3 ms`, pre/min RPM `1971.4/1500.0`, end RPM `1928.6`, ready hold máximo `1700.8 ms`, batería mínima `10.60 V`, health `HEALTHY`, fault `none`, safety gate PASS y `Shot/Kicker state=IDLE`. La pieza salió completamente y recta hacia la red; no hubo atasco, doble alimentación, ruido, vibración, olor ni calentamiento. La caída de `471.4 RPM` y recuperación hasta `1928.6 RPM` quedan como primera evidencia bajo carga.
- **Retorno de kicker ambiguo/bloqueante:** el operador respondió “no” a la pregunta combinada “¿se detuvo/regresó?”. El camino ejecutado sólo aplica avance y luego `stop()`; no ordena `reverse()`. `IDLE` demuestra estado lógico y orden de cero, no posición mecánica. Antes de otra pieza debe confirmarse por separado si el motor dejó físicamente de girar y si el mecanismo volvió a la posición de carga. No se implementa reversa automática ni se amplía tiempo sin esa observación.
- **Aclaración del stop del kicker:** el operador precisó que el kicker empujó la pieza y se detuvo después del tiempo programado; no permaneció girando. El stop físico de este pulso queda `PASS`. El camino motor-only no indexa ni ordena una posición de regreso.
- **Recuperación del shooter aún no cronometrada:** la muestra demuestra recuperación de `1500.0` a `1928.6 RPM` antes del corte, pero no publica el instante exacto en que reingresa y sostiene readiness. La gráfica sugiere aproximadamente `1–1.5 s`, sólo como estimación visual no certificada. No autorizar tres piezas consecutivas sin exigir `>=250 ms` continuos de readiness entre pulsos y medir recuperación/batería en cada ciclo.
- **Objetivo de ráfaga autorizado:** el lead acepta como objetivo tres piezas en máximo `3500 ms`. El candidato exige mantener `gamepad1 A + RIGHT_BUMPER`, target cargado `1800 RPM`, cap `0.90`, un burst por INIT y readiness continuo `1800±90 RPM` durante `>=250 ms` antes de cada pieza. Cada pulso conserva `632 ms`, corte duro `700 ms` y cooldown `300 ms`; el intake se enciende automáticamente al iniciar la ráfaga y el servo kicker sigue deshabilitado. Aborta ante release, fault, Stop, más de `3500 ms` desde el primer pulso o batería `<10.50 V`, umbral fail-closed colocado por debajo del mínimo ya validado `10.60 V`. Publica tiempo de inicio y pre/min RPM por pieza.
- **Retest bloqueado por precarga del intake:** el operador confirmó el sentido correcto del intake, pero con tres piezas y el intake continuo el shooter no alcanzó readiness con suficiente rapidez y la ráfaga no inició. La diferencia respecto del gate de `2000 RPM` previamente aprobado sin piezas es la carga continua introducida por el intake; se retira su comando predeterminado. El intake queda apagado durante INIT/spin-up, se enciende únicamente después de readiness continuo al comenzar la ráfaga y se apaga al completar, abortar, soltar, fault o Stop/E-stop. No cambia PID/FF, target, cap ni relación 1:1.
- **Retest cargado con intake aislado:** aun con el intake apagado durante spin-up, tres artifacts dentro produjeron peak/end `1800.0/1757.1 RPM`, power saturado `0.90`, batería `12.81 -> 12.15 V`, ready hold `0.0 ms`, `STOPPED_TIMEOUT` y `WAITING_FOR_READY`. La batería saludable y la salida en el cap descartan un ajuste de kP/kV como solución; las piezas imponen carga mecánica antes del disparo.
- **Contención para poder validar la ráfaga:** se fija únicamente el target de este gate cargado en `1800 RPM`, que usa la tolerancia existente `±90 RPM` y hold `>=250 ms`. Cap `0.90`, PID/FF, watchdogs, stops y relación 1:1 permanecen. Esta contención permite probar alimentación de tres piezas a la velocidad cargada demostrada, pero no certifica el alcance del tiro aprobado a `2000 RPM`; recuperar ese alcance exige aislar físicamente las piezas de la rueda durante spin-up o revisar el indexado.
- **Candidato de ráfaga cargada instalado:** APK `TeamCode-debug.apk`, `81,374,220` bytes, SHA-256 `40EDE95DD021AFFF7F4DAF856695110FE2A53915D2E1C25416A937AE73CC87BF`; instalación ADB `Success`, `76/76` tests JVM, cero failures/errors/skips, `assembleDebug` y `git diff --check` PASS. Tres piezas sólo pasan si termina `COMPLETED_3_OF_3` dentro de `3500 ms`, sin anomalía ni salida residual.
- **Gate de velocidad cargada aprobado:** antes de solicitar el burst, el candidato alcanzó peak/end `1842.9/1842.9 RPM`, power `0.90`, batería `12.747 V` con pulso `12.81 -> 11.80 V`, ready hold `4350.2 ms` y `STOPPED_TIMEOUT`. `Burst/Result=NOT_RUN` confirmó que en ese intento no se detectó `gamepad1 RIGHT_BUMPER`; la velocidad cargada `1800±90 RPM` dejó de ser el bloqueo.
- **Primera ráfaga de tres piezas, resultado mixto:** `Burst/Result=STOPPED_BURST_TIMEOUT`, `Consumed=true`, `Completed shots=2`, duración `3528.3 ms`, pero el operador confirmó que físicamente salieron las tres piezas. Shot 1: start `0.0 ms`, pre/min `1842.9/1371.4 RPM`; Shot 2: start `1275.9 ms`, pre/min `1800.0/1114.3 RPM`; Shot 3: start `3170.6 ms`, pre/min publicado `1800.0/1800.0 RPM`.
- **Causa temporal demostrada:** el contador acredita una pieza sólo al terminar su pulso solicitado. Con Shot 3 iniciando en `3170.6 ms` y duración actual `632 ms`, su final teórico es `3802.6 ms`; el timeout `3500 ms` se evalúa antes y aborta cuando sólo dos pulsos terminaron. Reordenar las ramas podría cambiar la etiqueta, pero no cumplir el límite del lead. Para conservar `632 ms`, Shot 3 tendría que iniciar `<=2868.0 ms`, al menos `302.6 ms` antes.
- **Limitación de telemetría:** no existe sensor de salida; `Completed shots` realmente significa pulsos terminados, mientras la observación física demuestra tres piezas expulsadas. El `min=1800 RPM` publicado para Shot 3 es una ventana truncada por timeout y no prueba ausencia de caída. Una revisión debe separar `Started pulses` de `Completed pulses` y nunca equiparar una orden con salida física.
- **Hipótesis siguiente, no implementada:** caracterizar una duración exclusiva del burst de aproximadamente `360 ms`, porque la tercera pieza salió dentro de los `357.7 ms` observables antes del abort. Mantener sin cambios el global `KICKER_EXTEND_TIME_MS=632`, corte duro `700 ms`, cooldown `300 ms`, readiness `250 ms`, PID/FF, target `1800`, cap `0.90` y ratio 1:1. Validar primero una pieza y después tres; si falla salida completa, descartar la hipótesis.
- **Cambios alternativos no autorizados:** ampliar el timeout por encima de `3500 ms`, reducir el hold de readiness, ensanchar tolerancia/bajar RPM o superar cap `0.90` modifican el contrato o el envelope y requieren decisión explícita del lead.
- **Conclusión física:** el congelamiento quedó aislado y corregido en el camino `CABLE/CONECTOR`; el cable/adaptador fue reemplazado y el mismo encoder cuenta coherentemente en RAW y bajo carga. La doble inversión de software quedó corregida sin cambiar la dirección física ni la relación 1:1.
- **Gate/cierre:** `T8.0 RAW PASS`, T8.1 `1000 RPM PASS`, `1500 RPM PASS`, `2000 RPM PASS`, FND-029 `CLOSED`; velocidad cargada `1800 RPM PASS`. Alimentación física de tres piezas demostrada, pero gate temporal `<=3500 ms` sigue `OPEN` por `28.3 ms` en el abort observado y por `302.6 ms` respecto al final teórico del tercer pulso completo. No avanzar a autos de tiro ni declarar la ráfaga cerrada.

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
