# Registro de hallazgos

> Estado: registro vivo; entradas iniciales provienen de auditoría estática, no de pruebas físicas
> Baseline histórico inicial: `main@f91af18`; baseline de implementación: `origin/main@a887fe4f7ca9023eec6034a0db6b8d918c640ecc`
> Última actualización: 2026-07-17
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
| FND-005 | HIGH | OPEN | Feeder tiene caminos directos sin interlock único | Mechanisms | MP-01/06 |
| FND-006 | HIGH | OPEN | Secuencia shooter puede esperar readiness sin timeout | Shooter | MP-01/06 |
| FND-007 | MEDIUM | OPEN | E-stop no uniforme en OpModes habilitados | Safety | MP-01/07 |
| FND-008 | MEDIUM | OPEN | Ownership de torreta/visión fuera del container | Architecture | MP-07 |
| FND-009 | MEDIUM | OPEN | Declaraciones de versión/dependencias superpuestas | Build | MP-09 |
| FND-010 | MEDIUM | CONTAINED | Superficie de OpModes sin separar commissioning/release | Release | MP-00/07/09 |
| FND-011 | HIGH | SUPERSEDED | `IntakeTeleOp` histórico puede dejar feeder energizado | Safety/mechanisms | FND-013 |
| FND-012 | LOW | OPEN | Conceptos/nombres legacy no reflejan hardware final | Architecture | MP-06/09 |
| FND-013 | CRITICAL | OPEN | `IntakeTeleOp` vigente añade shooter directo/latched | Safety/shooter | MP-01/07 |
| FND-014 | CRITICAL | FIX_READY | `TeleopTorreta` entrega shooter nulo a un comando | Safety/shooter | MP-01 |
| FND-015 | CRITICAL | OPEN | Shooter falla abierto ante encoder/voltaje inválido | Shooter | MP-01/06 |
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
- **Riesgo:** contacto mecánico, daño de cables o bloqueo prematuro.
- **Acción:** fixture/marks, potencia ≤0.1, medir ambos sentidos, backlash y repetibilidad; revisión mecánica.
- **Criterio de cierre:** tabla firmada con ticks/grados, margen, signos y TEST-T4 aprobado sin tocar hard stop.
- **Owner:** mechanical + turret.

### FND-004 — Localización no tiene constantes aptas para auto-aim

- **Severidad / estado:** `HIGH / OPEN`
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

### FND-006 — Espera de readiness sin timeout en secuencia alternativa

- **Severidad / estado:** `HIGH / OPEN`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Componentes:** `ShootSequenceCommand`
- **Esperado:** readiness con timeout/failure state e interruption cleanup.
- **Observado:** se conserva `WaitUntilCommand(shooter::isReady)` sin un límite superior explícito, aunque `ShootBurstCommand` ya contiene mejores protecciones.
- **Riesgo:** comando colgado y mecanismos en estado difícil de entender.
- **Acción:** retirar camino legacy o implementar timeout/state central; comprobar top-level `end`.
- **Criterio de cierre:** shooter que nunca llega a RPM termina/bloquea de forma segura dentro del timeout y deja outputs definidos.
- **Owner:** shooter.

### FND-007 — E-stop no es uniforme

- **Severidad / estado:** `MEDIUM / OPEN`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Esperado:** todo OpMode de producción/diagnóstico comparte latch, cancela scheduler y llama `RobotSafety.stopAll()`.
- **Observado:** `SafeCommandOpMode`/`RobotSafety` existen, pero el latch con `gamepad1.back` se observó de forma consistente sólo en `MainTeleOp`; otros modos habilitados no comparten la misma superficie.
- **Riesgo:** expectativas distintas al seleccionar un modo de prueba.
- **Acción:** base única + limpieza; test en Competition TeleOp/System Check.
- **Criterio de cierre:** producción, System Check y cada tuner autorizado pasan T4.1; los tuners pueden seguir visibles en commissioning según DEC-026.
- **Owner:** safety/release.

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

- **Severidad / estado:** `CRITICAL / OPEN`
- **Evidencia:** el commit integrado posterior a `f91af18` liga X/Y a `ShooterMotor.right2/left2` con potencia aproximada ±0.9; la salida permanece hasta `DPAD_DOWN` o Stop.
- **Riesgo:** shooter energizado fuera de health/readiness y con expectativa incorrecta de pulso.
- **Contención:** no usar `IntakeTeleOp` para commissioning normal; conservar visible sólo si se clasifica/bloquea expresamente hasta reemplazarlo por System Check.
- **Cierre:** ningún binding activa shooter/feeder directo; release/interrupt/E-stop ordenan cero dentro del gate y veinte repeticiones pasan.

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

- **Severidad / estado:** `CRITICAL / OPEN`
- **Evidencia:** RPM imposible (>20000) se devuelve como cero y voltaje ≤1 V se sustituye por 12.5 V; con target positivo, bang-bang/output clamp puede pedir power 1.0.
- **Riesgo:** encoder congelado/desconectado o voltaje inválido puede producir máxima salida.
- **Acción:** health explícito para encoder/velocidad/voltaje, timeout de ausencia de pulsos, overspeed y fallo cerrado a target/power cero.
- **Cierre:** fault injection de freeze, jump, NaN/imposible y voltaje inválido nunca eleva power y exige rearmado deliberado.

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
- **Riesgo:** límites y setpoint desplazados, contacto mecánico o cables tensionados.
- **Acción:** marca/fixture, `zeroValid`, invalidación en cada init/reset/brownout, zona de frenado y contención exterior.
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
