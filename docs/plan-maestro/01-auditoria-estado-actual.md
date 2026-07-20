# 01 — Auditoría del estado actual

> Estado: auditoría actualizada con implementación MP-01; export RC y confirmación física pendientes
> Baseline de implementación: `origin/main@a887fe4f7ca9023eec6034a0db6b8d918c640ecc`
> Última actualización: 2026-07-17
> Alcance: repo, composición activa, hardware observado, findings históricos y brechas
> Responsable sugerido: líder de software con revisión mecánica/eléctrica
> Fuente de verdad: reinspeccionar el SHA seleccionado antes de cambiar código; los nombres en código no prueban la configuración del Robot Controller.

## 1. Resumen ejecutivo

El baseline vigente ya no coincide ni con la fotografía descrita al inicio de [critical-findings.md](../critical-findings.md) ni con el baseline documental histórico `main@f91af18`. El reporte histórico sigue siendo valioso como evidencia, pero no debe leerse como inventario actual.

El robot activo todavía mezcla una composición command-based con componentes de torreta/visión construidos fuera de `RobotContainer`. El drivetrain de competencia usa Road Runner y un `ThreeDeadWheelLocalizer`; Pedro existe como scaffold de tuning con constantes provisionales. MP-01A implementa cleanup seguro de `VisionPortal`, init-loop repetitivo y armado sostenido de torreta; falta retest físico antes de cerrar esos findings.

El shooter activo es de un motor y su controlador tiene PID/feedforward, compensación por voltaje y stop. Sin embargo, valores imposibles de RPM se convierten a cero y voltaje ≤1 V se sustituye por 12.5 V; con target positivo, esa combinación puede pedir la salida máxima ante encoder o sensor de voltaje defectuoso. El control de disparo también sigue repartido entre perfiles, comandos y adapters; al menos una secuencia conserva una espera de readiness sin timeout.

`origin/main` incorpora además controles X/Y de shooter en `IntakeTeleOp` que quedan latched hasta otra orden/Stop. MP-01A retira de `TeleopTorreta` el camino que programaba `ShooterCommand2` sin haber inicializado su `ShooterMotor`; `IntakeTeleOp` y el fail-open del shooter permanecen bloqueadores críticos de MP-01.

## 2. Identidad y reproducibilidad del baseline

| Elemento | Observación en el baseline | Implicación |
|---|---|---|
| Ref fuente | `origin/main` | Base integrada antes de abrir `masterplan`. |
| Commit | `a887fe4f7ca9023eec6034a0db6b8d918c640ecc` | Toda evidencia de implementación debe citar este SHA o el commit posterior del paquete. |
| Rama documental | `agent/document-robot-master-plan-20260715` en `c0d93bd` antes de esta edición | Contiene los docs; estaba 2 commits detrás de `origin/main`, cuyo merge incluye el commit documental y cambios de código posteriores. |
| Worktree al iniciar la corrección | Limpio | No había cambios del usuario que separar. |
| Worktree al iniciar MP-01A | `KICKER_OUT_SPEED=0.85` local | Incluido en MP-01A por decisión posterior del equipo; pendiente de validación física. |
| Baseline restaurado | `backup/main-before-split-20260619` en `e42e7cf` | Referencia histórica completa, no base de implementación actual. |
| Lenguaje/arquitectura | Java, FTCLib command-based, Road Runner, FTC Dashboard; scaffold de Pedro | Evitar una reescritura general; migrar por interfaces y gates. |
| Fuente SDK declarada | `TeamCode/build.gradle` aún declara FTC SDK 10.3.0 | En conflicto con el include común descrito abajo. |
| Fuente SDK común | `build.dependencies.gradle` resuelve SDK 11.0 y otras dependencias | La versión efectiva debe verificarse en Gradle; consolidarla es trabajo futuro y revisado. |
| Dashboard | Versiones duplicadas observadas en Gradle | No normalizar incidentalmente durante auto-aim. |

Como evidencia histórica de `main@f91af18`, una compilación con el JBR de Android Studio 21 terminó correctamente: aproximadamente 192.3 s en frío y `TeamCode-debug.apk` de 81,278,696 bytes, con 77 archivos Java y cerca de 7,347 líneas. Esa cifra no se reutiliza como evidencia del paquete actual. El worktree MP-01 sobre `masterplan@24f9911` completó un `assembleDebug` limpio final en 68.9 s y produjo un APK de 81,276,823 bytes, SHA-256 `624EA17B7E79AA594D8CC5720098375C9BD375F10BD8EA82EB02CCF5E1AA15E4`; las pruebas físicas siguen pendientes.

## 3. Superficie de OpModes actual

La contención MP-01 deja cuatro TeleOps anotados y habilitados:

- `Skywalker TeleOp (Manual Principal)`;
- `SYSTEM CHECK and TUNING`;
- `Tuning: Shooter & Systems (Manual)`;
- `Test Color` (sensor-only).

Los demás TeleOps legacy, `TeleopTorreta` y Pedro Tuning están `@Disabled`. Road Runner conserva su `@OpModeRegistrar`, pero `TuningOpModes.DISABLED_HAND_PUSH_TUNERS`/`DISABLED_POWERED_TUNERS=true` impide el registro dinámico durante MP-01 (ver docs/plan-maestro/09-runbook-paso2-odometria.md para el acotamiento de Paso 2). Todos los autónomos anotados están deshabilitados.

Esto importa por dos razones:

1. buscar sólo anotaciones no descubre registros dinámicos;
2. borrar archivos para “acelerar deploy” sin medir puede no atacar el costo real, aunque sí reduce ruido, caminos alternos y riesgo operativo.

Durante MP-01 sólo permanecen visibles los modos con lifecycle seguro que mueven actuadores. Pedro y Road Runner se reactivan en MP-02 únicamente después de integrar Stop/E-stop verificable. Sólo después de MP-08 se crea el snapshot de MP-09; el artefacto final reduce el menú y se revalida en MP-10.

## 4. Camino activo de composición

### 4.1 TeleOp principal

`MainTeleOp` crea `RobotContainer`, que a su vez posee:

- `DriveSubsystem`;
- `IntakeSubsystem`;
- `ShooterSubsystem`;
- shim de compatibilidad de hood sin hardware;
- `KickerSubsystem`;
- bindings de `SkywalkerProfile`.

`MainTeleOp` todavía posee por separado:

- `TurretSubsystem`;
- la máquina de armado manual de torreta.

Las webcams y el seguimiento AprilTag se retiraron del camino activo. Hasta MP-03/05 la torreta
puede armarse para validar el cero, pero no recibe un comando de movimiento.

Esta división permite que el composition root no conozca dos actuadores/sensores esenciales y duplica decisiones de lifecycle. El diseño objetivo mueve torreta y Limelight al mismo dueño que los demás subsistemas.

### 4.2 Drivetrain/localización

El camino activo es:

```text
MainTeleOp
  -> RobotContainer
     -> DriveSubsystem
        -> MecanumDrive (Road Runner)
           -> ThreeDeadWheelLocalizer
              -> rightFront(par0), leftFront(par1), rightBack(perp)
```

El localizador de tres ruedas muertas ya reemplazó al two-dead-wheel histórico que reutilizaba encoders de motores del drivetrain. No obstante, los parámetros observados son todavía provisionales:

- `inPerTick = 0.0019571295433364`;
- `par0YTicks = 0`;
- `par1YTicks = 1`;
- `perpXTicks = 0`;
- reversals en `false`.

Estos valores no prueban geometría física y hacen que la pose no sea apta todavía para apuntado. El hardware observado usa los nombres `par0`, `par1` y `perp`; se deben confirmar puertos, signo y offsets en el Robot Controller.

Pedro Pathing no controla el TeleOp de competencia. `pedroPathing/Constants.java` contiene, entre otros indicios de scaffold:

- masa `5` con TODO;
- gains incompletos;
- pods asignados a nombres de motores de drive;
- setters repetidos de offsets cuyo valor final queda en cero;
- conversiones por tick provisionales.

No debe activarse Pedro cambiando una sola referencia. MP-02 es una migración calibrada y reversible.

### 4.3 Torreta y visión actual

`TurretSubsystem` observa:

- motor `torretaMotor`;
- dirección invertida desde `RobotMap`;
- `RUN_USING_ENCODER`;
- `BRAKE` y potencia inicial cero;
- registro en `RobotSafety`;
- armado explícito;
- soft limits actuales de aproximadamente `-200` a `+200` ticks;
- gate de potencia cuando está desarmada o contra un límite.

`TurretFollowTagCommand`:

- declara requisito de la torreta;
- selecciona el goal tag por alianza: Blue 20, Red 24;
- usa bearing visual con control proporcional y clamp;
- mantiene la última observación por 250 ms y luego detiene;
- ordena stop en `end()`.

Lo anterior es una mejora real respecto al finding histórico, pero sigue siendo vision-only. No conoce pose de cancha ni objetivo geométrico, y la librería `DecodeGoalTags` contiene tamaños/IDs, no poses de campo suficientes para el nuevo diseño.

### 4.4 Shooter, hood y feeder

El `ShooterSubsystem` activo usa sólo `Shooter`; `Shooter2` fue confirmado retirado y ya no se mapea. El controlador incluye:

- encoder feedback;
- PID/feedforward;
- compensación por voltaje;
- filtrado de RPM;
- recuperación bang-bang/coast;
- clamp de salida;
- stop que pone target y potencia en cero.

La readiness actual se basa principalmente en tolerancia instantánea y no incorpora velocidad lineal/angular del chasis. Además, `getActualShooterRPM()` transforma RPM imposibles en cero y la compensación sustituye voltaje ≤1 V por 12.5 V; esto es fail-open porque el bang-bang puede pedir máximo power. El diseño objetivo agregará health explícito, dwell, movimiento del chasis, torreta, pose y feeder, y fallará cerrado a target/power cero.

`ShooterHoodSubsystem` todavía controla dos servos en código, pero el equipo confirmó que el hood ya fue retirado físicamente y el robot final tendrá ángulo mecánico fijo. El retiro físico aún necesita evidencia/configuración; el código se limpia únicamente después del replacement validado.

El feeder físico está representado por `KickerSubsystem`; el equipo confirmó `kickerMotor` y el CRServo `kickerServo`. El subsistema es dueño de ambos y aplica kick/reverse/stop conjuntamente.

### 4.5 Controles actuales relevantes

Los bindings de `SkywalkerProfile` observados incluyen:

- conductor, `RB`: kicker/feeder sostenido de forma directa;
- conductor, `START`: reset de heading;
- conductor, `LB`: precision mode;
- conductor, `DPAD_RIGHT`: alterna alianza durante ejecución;
- conductor, `A/B`: giros del chasis a ángulos fijos por alianza;
- operador 2, `A/X/B/Y`: presets de hood y RPM;
- operador 2, `RB/LB`: intake forward/reverse;
- operador 2, `DPAD_DOWN`: intake y shooter off;
- operador 2, `DPAD_UP`: feeder mientras se sostiene;
- operador 2, `DPAD_LEFT`: feeder reversa mientras se sostiene;
- operador 2, `DPAD_RIGHT`: hood home.

Estos bindings no coinciden con el contrato final. En especial, la alianza no debe ser toggle durante el match, el hood desaparecerá y el feeder no debe energizarse fuera de un interlock central.

En `origin/main`, `IntakeTeleOp` también liga X/Y a `ShooterMotor.right2/left2` con power aproximado ±0.9. La salida permanece hasta `DPAD_DOWN` o Stop; no es un pulso ni pasa por readiness. `TeleopTorreta` dejó de construir `ShooterMotor` pero todavía crea un `ShooterCommand2` que puede recibir `null`.

## 5. Inventario de nombres de hardware observados

| Función | Nombre en código | Estado de confianza |
|---|---|---|
| Drive | `leftFront`, `rightFront`, `leftBack`, `rightBack` | Observado; confirmar direcciones y que `rightRear` en Pedro no sea una inconsistencia. |
| Shooter | `Shooter` | Activo, un motor confirmado por el usuario; faltan constantes físicas. |
| Shooter legado | `Shooter2` | Retirado físicamente y eliminado del mapa activo. |
| Intake | `intakeMotor` | Observado; confirmar dirección/corriente/límites. |
| Hood | `hoodLeft`, `hoodRight` | Retirado físicamente; shim legacy no mapea hardware. |
| Feeder/kicker | `kickerMotor`, `kickerServo` | Confirmados por el equipo; export y prueba pendientes. |
| Turret | `torretaMotor` | Observado; dirección, signo, ticks y arco pendientes. |
| Webcam anterior | `Webcam 1`, `Webcam` | Retiradas y fuera de los OpModes activos. |
| Webcam dormida | `Webcam` | Camino alterno/dormido; hardware reportado retirado. |
| Odometría | `par0`, `par1`, `perp` | Tres pods confirmados conceptualmente; geometría pendiente. |
| IMU | `imu` | Observado; orientación física pendiente. |
| Limelight instalada | `limelight` propuesto | Equipo reporta dispositivo instalado; mapping, firmware, pipeline, red y extrínseca no están confirmados. |

## 6. Reconciliación de hallazgos históricos

| Finding histórico | Estado histórico en `f91af18` | Evidencia/limitación en `b5a1342` | Acción del plan |
|---|---|---|---|
| Giros enviados como 90 radianes | **Contenido/reemplazado** | Autos afectados están deshabilitados o comentados; el legado sigue en source. | No gastar commissioning en repararlos; borrar tras snapshot en MP-09. |
| Encoders de drive usados como two dead wheels | **Reemplazado con nuevo blocker** | El camino activo usa tres pods `par0/par1/perp`; offsets actuales son placeholders. | Calibrar Pedro en MP-02. |
| Comandos interrumpidos dejan motores activos | **Parcialmente corregido** | `ActionCommand` y `ShootBurstCommand` tienen mejor cleanup; adapters y secuencias alternas aún requieren auditoría. | Auditoría/stop contract en MP-01. |
| Requisitos duplicados en grupos paralelos | **Contenido** | Caminos principales descritos eran autos ahora deshabilitados. | Eliminar legado en MP-09; no reintroducir grupos inválidos. |
| Shooter espera readiness sin timeout | **Parcial** | Burst tiene timeout/failure; `ShootSequenceCommand` conserva `WaitUntilCommand(shooter::isReady)` sin límite superior explícito. | Interlock y timeout central en MP-01/06. |
| E-stop no detiene scheduler/actuadores | **Parcialmente corregido** | `SafeCommandOpMode` y `RobotSafety` existen; sólo `MainTeleOp` expone consistentemente el latch observado. | Base común de producción/diagnóstico en MP-01/07. |
| Shooter puede reactivarse tras stop | **Probablemente corregido, pendiente retest** | Tuning usa estado explícito y recarga de controlador; falta prueba física. | Validar apagado/restart en MP-01/08. |
| Ruta blue discontinua | **Contenido** | Auto afectado deshabilitado. | Borrar en MP-09, no portarlo. |
| Alliance state incorrecto en red auto | **Corregido/obsoleto para scope** | Autos no activos; TeleOp aún alterna alianza durante run. | Selector directo durante init en MP-07. |
| Normalización de drive sin magnitud absoluta | **Corregido** | Camino actual usa normalización apropiada. | Test de regresión al migrar Pedro. |
| TeleOp principal pisa default field-centric | **Corregido** | Override histórico ya no es el patrón observado. | Preservar comportamiento explícitamente en MP-02/07. |
| Tracking elige primer tag sin filtro | **Corregido parcialmente** | Comando actual filtra goal 20/24 y tiene hold de pérdida; sigue vision-only. | Reemplazar con solución fusionada en MP-04/05. |

“Contenido” significa que el camino no está habilitado, no que el código sea correcto. “Probablemente corregido” exige retest antes de cerrarse.

## 7. Hallazgos actuales priorizados

| ID | Severidad | Hallazgo estático | Riesgo | Siguiente acción |
|---|---|---|---|---|
| FND-001 | Crítica | `MainTeleOp` y `TeleopTorreta` registran `addResourceCleanup(visionPortal::close)` antes de asignar `visionPortal`. | Posible `NullPointerException` durante init; TeleOp inutilizable. | Reproducir con robot deshabilitado y corregir orden/lambda null-safe en MP-01. |
| FND-002 | Alta | El chord `START+BACK` de centrado se evalúa sólo durante `initialize()`. | El operador puede no tener una ventana práctica para armar; tracking queda deshabilitado o se improvisa. | Máquina de estado de `init_loop` con hold de 1 s y feedback. |
| FND-003 | Alta | Soft limits ±200 ticks no están validados físicamente. | Golpe, cable tensionado o arco insuficiente. | Medición mecánica a potencia reducida antes de tracking. |
| FND-004 | Alta | Localización activa y scaffold Pedro contienen geometría provisional/inconsistente. | Auto-aim y distancia no confiables. | MP-02 antes de cualquier fusión. |
| FND-005 | Alta | Feed puede ser ordenado desde bindings/adapters sin un único readiness interlock. | Lanzamiento no solicitado o fuera de aim/RPM. | Centralizar request e interlock en MP-06. |
| FND-006 | Alta | Una secuencia de shooter conserva una espera sin timeout. | Comando colgado y comportamiento difícil de abortar. | Sustituir por condición con timeout y cleanup. |
| FND-007 | Media | E-stop no está uniformemente expuesto en todos los OpModes habilitados. | Modo de prueba puede carecer del stop esperado. | Herencia/composición segura única y limpieza. |
| FND-008 | Media | Torreta y visión se construyen fuera de `RobotContainer`. | Ownership/lifecycle fragmentado. | Unificar en MP-07. |
| FND-009 | Media | SDK/dependencias tienen fuentes de versión superpuestas. | API efectiva confusa y builds menos reproducibles. | Documentar resolución; consolidar sólo en MP-09. |
| FND-010 | Media | 14+ TeleOps y tuners dinámicos abarrotan el Driver Station. | Selección errónea y caminos de seguridad distintos. | Mantener durante commissioning; reducir al final. |
| FND-011 | Alta | `IntakeTeleOp` puede dejar feeder activo tras comandos instantáneos hasta otro botón. | Motor energizado por estado latched no evidente. | No usar como diagnóstico; reemplazar con System Check hold-to-run. |
| FND-012 | Baja | Nombres/conceptos `kicker`, `feeder`, hood y `Shooter2` no reflejan el robot final. | Mantenimiento confuso; cambios de mapping accidentales. | Renombrado controlado en MP-06/09 tras contrato físico. |
| FND-013 | Crítica | `IntakeTeleOp` vigente añade outputs directos/latched de shooter además de caminos directos de mecanismos. | Shooter puede permanecer energizado fuera de readiness. | Bloquear su uso y reemplazarlo por System Check seguro. |
| FND-014 | Crítica | `TeleopTorreta` puede programar `ShooterCommand2` con `ShooterMotor == null`. | Excepción o pérdida de control al usar el binding. | Contener el modo y corregir ownership en MP-01. |
| FND-015 | Crítica | Encoder/RPM/voltaje inválidos del shooter pueden traducirse en power máximo. | Overspeed ante sensor congelado/desconectado. | Health explícito y fail-closed a cero en MP-01/06. |
| FND-016 | Alta | `SafeCommandOpMode` no ejecuta un init-loop antes de `waitForStart()`. | Armado/E-stop/telemetry pre-start no pueden evaluarse de forma repetitiva. | Hook de init-loop y test de lifecycle. |
| FND-017 | Alta | MP-02 sólo abstraía pose mientras RR seguía poseyendo movimiento. | Dual-stack implícito y rollback ambiguo. | Migrar pose y control de drivetrain como unidad. |
| FND-018 | Alta | Feeder no tiene duración máxima/cooldown obligatorios por pieza. | Request sostenido puede energizar indefinidamente. | Máquina `IDLE/WAITING/PULSING/COOLDOWN/FAULT`. |
| FND-019 | Alta | Readiness no exige chasis estacionario. | Feed durante movimiento lineal/angular no validado. | Medir umbrales de velocidad y bloquear fuera de ellos. |
| FND-020 | Alta | El cero manual no detecta falso centro y se pierde tras reset/brownout. | Setpoint/límites desplazados aunque el encoder diga cero. | Marca/fixture, `zeroValid`, zona de frenado y rearmado. |
| FND-021 | Alta | No existe export RC; contrato omite hechos de drive, pods, IMU, intake y Limelight. | Mappings/direcciones inventados o ownership incompleto. | Completar guía 08 antes de habilitar el componente. |
| FND-022 | Alta | La limpieza MP-09 ocurre después de aceptación pero sólo exigía smoke test. | El SHA liberado puede no ser el SHA validado. | MP-10 repite T0–T10 y dos sesiones. |
| FND-023 | Media | Rollback/final tags se materializaban tarde o no existían. | Recuperación no demostrada. | Tags baseline, pre-cleanup y release en gates separados. |
| FND-024 | Media | Gates omitían muestras, elapsed time y tratamiento de build ambiental. | Resultados no comparables o aprobación falsa. | Criterios exactos en el programa de pruebas. |
| FND-025 | Media | Estados y catálogo de poses iniciales no estaban sincronizados. | Transiciones/arranque ambiguos. | Unificar enums y bloquear poses no verificadas. |

Los registros editables y su evidencia viven en [hallazgos.md](hallazgos.md).

## 8. Hechos físicos faltantes

Nada en la compilación responde estas preguntas. Deben medirse y firmarse usando la [guía de verificación de hardware](08-guia-verificacion-hardware.md):

| Área | Dato requerido | Unidad/formato | Método mínimo |
|---|---|---|---|
| Torreta | Sentido positivo real | CW/CCW visto desde arriba | Potencia ≤0.1, robot elevado/desarmado mecánicamente. |
| Torreta | Ticks del centro a cada límite seguro | ticks y grados | Marcar centro, acercarse sin tocar hard stop, margen documentado. |
| Torreta | Backlash y repetibilidad de cero | grados/ticks | 10 aproximaciones desde ambos sentidos. |
| Torreta | Offset eje→centro del robot | pulgadas, X/Y | Medición mecánica. |
| Cámara | Pose chasis→Limelight | in y grados, X/Y/Z/yaw/pitch/roll | CAD/medición y validación con targets. |
| Odometría | Diámetro efectivo y ticks/rev por pod | in, ticks/rev | Rolling test y especificación de encoder. |
| Odometría | Offsets y signos de tres pods | in, convención | Medición desde centro de giro y spin test. |
| IMU | Logo/USB orientation y sentido heading | orientación SDK | Inspección y giro manual. |
| Shooter | Ticks/rev efectivos y gear ratio | ticks/rev, ratio | Hoja de motor + medición. |
| Shooter | RPM mecánica máxima segura | RPM | Aprobación mecánica, corriente/vibración/temperatura. |
| Shooter | Inversión y spin direction | signo | Prueba restringida a baja potencia. |
| Feeder | Nombre RC exacto | string | Export/inspección de configuración. |
| Feeder | Dirección, potencia y timeout seguros | signo, power, ms | Prueba con pieza y corriente observada. |
| Robot | Masa y centro aproximado | kg, posición | Báscula/medición para Pedro y dinámica. |
| Goal shot | Punto de aim efectivo | coordenada/offset | Calibración de tiros; no equipararlo al centro del tag. |

## 9. Brecha del estado actual al objetivo

| Capacidad | Actual | Objetivo | Paquete |
|---|---|---|---|
| Localización | RR three-wheel con parámetros provisionales | Pedro calibrado detrás de `PoseProvider` | MP-02 |
| Visión | Webcam + AprilTag/VisionPortal, bearing-only | Limelight fija, observaciones validadas | MP-03 |
| Pose fusionada | No existe | Odometría continua + correcciones visuales gated | MP-04 |
| Aim | Tag bearing proporcional | Bearing geométrico + trim visual + estados de calidad | MP-05 |
| Torreta init | Check instantáneo | Hold 1 s en init, feedback y fault handling | MP-01/05 |
| Shooter | Presets hood/RPM | Ángulo fijo, RPM por distancia, manual/trim | MP-06 |
| Feed | Múltiples bindings directos | Request sostenido + readiness central | MP-06 |
| Alianza | Toggle durante run | Selección directa y lock en init | MP-07 |
| Degradación | Implícita/fragmentada | `DEGRADED_FIXED_FORWARD` deliberado | MP-05/06 |
| Diagnóstico | Muchos TeleOps/tuners sin clasificación uniforme | Tuners visibles y controlados en commissioning; System Check seguro | MP-00/07 |
| Release | Muchos modos/caminos | Un TeleOp + un diagnóstico, rollback y revalidación Git | MP-09/10 |

## 10. Condición para cerrar esta auditoría

MP-00 puede cerrarse cuando:

- el SHA y las dependencias declaradas estén registradas; la resolución efectiva queda bloqueada hasta autorizar build;
- exista un export o captura verificable del hardware map;
- todos los nombres y dueños de actuadores estén confirmados;
- cada finding crítico/alto tenga responsable y sesión de prueba programada;
- el equipo haya revisado la discrepancia entre el informe histórico y `origin/main@b5a1342`;
- las medidas físicas faltantes estén asignadas, no inventadas.

Estado al 2026-07-17: **parcial**. El baseline y los cambios de lifecycle están documentados, pero faltan el export RC, la validación física y contener FND-013/FND-015. Los resultados de build y pruebas se registran en [handoff-task.md](handoff-task.md).

Esta auditoría no autoriza movimiento automático. Su siguiente paso es completar MP-01, no Limelight ni tuning de aim.
