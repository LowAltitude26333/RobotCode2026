# 01 — Auditoría del estado actual

> Estado: auditoría estática inicial; requiere confirmación física
> Baseline: `main` en `f91af18`
> Última actualización: 2026-07-15
> Alcance: repo, composición activa, hardware observado, findings históricos y brechas
> Responsable sugerido: líder de software con revisión mecánica/eléctrica
> Fuente de verdad: reinspeccionar el SHA seleccionado antes de cambiar código; los nombres en código no prueban la configuración del Robot Controller.

## 1. Resumen ejecutivo

El `main` actual ya no coincide con la fotografía descrita al inicio de [critical-findings.md](../critical-findings.md): contiene el código completo restaurado y varias remediaciones. El reporte histórico sigue siendo valioso como evidencia, pero no debe leerse como inventario actual.

El robot activo todavía mezcla una composición command-based con componentes de torreta/visión construidos fuera de `RobotContainer`. El drivetrain de competencia usa Road Runner y un `ThreeDeadWheelLocalizer`; Pedro existe como scaffold de tuning con constantes provisionales. La torreta ya tiene mejor protección que el baseline histórico —armado, soft limits, stop y filtro de goal tag— pero el flujo actual de inicialización presenta dos bloqueadores probables: cleanup de `VisionPortal` registrado antes de construirlo y un chord de centrado evaluado sólo una vez durante `initialize()`.

El shooter activo es de un motor y su controlador tiene PID/feedforward, compensación por voltaje y stop. El control de disparo, sin embargo, aún está repartido entre perfiles, comandos y adapters; al menos una secuencia conserva una espera de readiness sin timeout. La futura arquitectura debe centralizar el interlock de feeder.

## 2. Identidad y reproducibilidad del baseline

| Elemento | Observación en el baseline | Implicación |
|---|---|---|
| Rama | `main` | Es la base seleccionada; no asumir que una feature branch es más nueva por nombre. |
| Commit | `f91af18` | Toda evidencia de este documento debe compararse contra este SHA. |
| Worktree al auditar | Limpio | No había cambios del usuario que separar en esta auditoría. |
| Baseline restaurado | `backup/main-before-split-20260619` en `e42e7cf` | Referencia histórica completa, no base de implementación actual. |
| Lenguaje/arquitectura | Java, FTCLib command-based, Road Runner, FTC Dashboard; scaffold de Pedro | Evitar una reescritura general; migrar por interfaces y gates. |
| Fuente SDK declarada | `TeamCode/build.gradle` aún declara FTC SDK 10.3.0 | En conflicto con el include común descrito abajo. |
| Fuente SDK común | `build.dependencies.gradle` resuelve SDK 11.0 y otras dependencias | La versión efectiva debe verificarse en Gradle; consolidarla es trabajo futuro y revisado. |
| Dashboard | Versiones duplicadas observadas en Gradle | No normalizar incidentalmente durante auto-aim. |

Una compilación de baseline realizada durante la investigación con el JBR de Android Studio 21 terminó correctamente. Como medición puntual —no benchmark universal— el build frío tomó aproximadamente 192.3 s y produjo `TeamCode-debug.apk` de 81,278,696 bytes. Se contaron 77 archivos Java y aproximadamente 7,347 líneas. MP-00 debe repetir la medición con cachés/entorno registrados; MP-09 comparará build frío, caliente, APK e instalación real.

## 3. Superficie de OpModes actual

La inspección encontró 14 TeleOps anotados y habilitados en total, incluido el tuning de Pedro. Sus nombres de Driver Station son:

- `Skywalker TeleOp (Manual Principal)`;
- `Skywalker TeleOp 2`;
- `Skywalker Turret: SENTINEL (Solo Conectados)`;
- `TeleOp Intake Control`;
- `Skywalker TeleOp w/ April`;
- `TeleOpBETA 1 ROJO`;
- `boton`;
- `Motores`;
- `ShooterTeleOpAdaptado`;
- `SYSTEM CHECK and TUNING`;
- `Tuning: Shooter & Systems (Manual)`;
- `Test Color`;
- `FTCLib Test`;
- `Tuning` de Pedro Pathing.

`TeleOpFieldCentric` está anotado pero también `@Disabled`, por lo que no forma parte de esos 14. Road Runner registra además rutinas dinámicas mediante `@OpModeRegistrar` cuando su bandera `DISABLED` está en `false`. Los ocho archivos con anotación `@Autonomous` compilable están deshabilitados, y la anotación de `AutonomoOfficialRed.java` está dentro de un comentario de bloque; no existe un autónomo de competencia habilitado en este baseline.

Esto importa por dos razones:

1. buscar sólo anotaciones no descubre registros dinámicos;
2. borrar archivos para “acelerar deploy” sin medir puede no atacar el costo real, aunque sí reduce ruido, caminos alternos y riesgo operativo.

El objetivo final es uno más uno: un TeleOp de competencia y un System Check seguro. Los modos restantes no se deben borrar hasta completar MP-08 y crear el snapshot de MP-09.

## 4. Camino activo de composición

### 4.1 TeleOp principal

`MainTeleOp` crea `RobotContainer`, que a su vez posee:

- `DriveSubsystem`;
- `IntakeSubsystem`;
- `ShooterSubsystem`;
- `ShooterHoodSubsystem`;
- `KickerSubsystem`;
- bindings de `SkywalkerProfile`.

`MainTeleOp` todavía posee por separado:

- `TurretSubsystem`;
- procesamiento AprilTag;
- `VisionPortal` con `Webcam 1`;
- el comando de seguimiento de torreta.

Esta división permite que el composition root no conozca dos actuadores/sensores esenciales y duplica decisiones de lifecycle. El diseño objetivo mueve torreta y Limelight al mismo dueño que los demás subsistemas.

### 4.2 Drivetrain/localización

El camino activo es:

```text
MainTeleOp
  -> RobotContainer
     -> DriveSubsystem
        -> MecanumDrive (Road Runner)
           -> ThreeDeadWheelLocalizer
              -> par0, par1, perp + IMU/drive según la clase
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

El `ShooterSubsystem` activo usa sólo `Shooter`; `Shooter2` permanece como declaración heredada, no como segundo motor activo. El controlador incluye:

- encoder feedback;
- PID/feedforward;
- compensación por voltaje;
- filtrado de RPM;
- recuperación bang-bang/coast;
- clamp de salida;
- stop que pone target y potencia en cero.

La readiness actual se basa principalmente en tolerancia instantánea. El diseño objetivo agregará dwell y combinará shooter, torreta, pose y feeder.

`ShooterHoodSubsystem` controla dos servos, pero el equipo definió que el robot final tendrá ángulo mecánico fijo y retirará esos servos. No se debe borrar el código hasta confirmar la modificación física y completar el replacement.

El feeder físico está representado por `KickerSubsystem`; el hardware string observado es exactamente `kickerM otor`, con espacio interno. Renombrar la clase/concepto a feeder es razonable, pero cambiar el string sin revisar la configuración puede romper el robot.

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

## 5. Inventario de nombres de hardware observados

| Función | Nombre en código | Estado de confianza |
|---|---|---|
| Drive | `leftFront`, `rightFront`, `leftBack`, `rightBack` | Observado; confirmar direcciones y que `rightRear` en Pedro no sea una inconsistencia. |
| Shooter | `Shooter` | Activo, un motor confirmado por el usuario; faltan constantes físicas. |
| Shooter legado | `Shooter2` | Declarado pero no mapeado por el subsistema activo; candidato a borrar al final. |
| Intake | `intakeMotor` | Observado; confirmar dirección/corriente/límites. |
| Hood | `hoodLeft`, `hoodRight` | Código activo hoy; hardware destinado a removerse. |
| Feeder/kicker | `kickerM otor` | Preservar exactamente hasta confirmar configuración. |
| Turret | `torretaMotor` | Observado; dirección, signo, ticks y arco pendientes. |
| Webcam actual | `Webcam 1` | Usada por tracking actual; objetivo final es Limelight. |
| Webcam dormida | `Webcam` | Camino alterno/dormido; candidato a limpiar. |
| Odometría | `par0`, `par1`, `perp` | Tres pods confirmados conceptualmente; geometría pendiente. |
| IMU | `imu` | Observado; orientación física pendiente. |
| Limelight propuesta | `limelight` | Aún no confirmada en configuración; no inventar el mapping. |

## 6. Reconciliación de hallazgos históricos

| Finding histórico | Estado en `f91af18` | Evidencia/limitación actual | Acción del plan |
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

Los registros editables y su evidencia viven en [hallazgos.md](hallazgos.md).

## 8. Hechos físicos faltantes

Nada en la compilación responde estas preguntas. Deben medirse y firmarse:

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
| Diagnóstico | Muchos TeleOps/tuners | Un System Check seguro | MP-07/09 |
| Release | Muchos modos/caminos | Un TeleOp + un diagnóstico, rollback Git | MP-09 |

## 10. Condición para cerrar esta auditoría

MP-00 puede cerrarse cuando:

- el SHA y la versión de dependencias efectiva estén registrados;
- exista un export o captura verificable del hardware map;
- todos los nombres y dueños de actuadores estén confirmados;
- cada finding crítico/alto tenga responsable y sesión de prueba programada;
- el equipo haya revisado la discrepancia entre el informe histórico y `main`;
- las medidas físicas faltantes estén asignadas, no inventadas.

Esta auditoría no autoriza movimiento automático. Su siguiente paso es MP-01, no Limelight ni tuning de aim.
