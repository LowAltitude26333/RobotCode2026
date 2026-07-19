# Inventario de telemetría y tuners — prep MP-07

> Estado: entregable de la Pista Software (plan-paralelo-20h.md secc. 3.6)
> Última actualización: 2026-07-18
> Propósito: dejar la migración de telemetría de MP-07 como trabajo mecánico ya diseñado, y dar a MP-09 la lista de tuners con dueño para decidir qué sobrevive a la limpieza.

## 1. Helper de bloques

`TeamCode/.../telemetry/TelemetryBlocks.java` define los bloques estándar de MP-07: `header`, `mode`, `pose(PoseSnapshot)`, `vision(LimelightObservation, Health, tagId)`, `turret`, `shooter`, `faults`. Primer consumidor real: `LimelightDiagnosticOpMode` (bloques mode + vision + faults). `MainTeleOp` NO se migra en esta ventana — está validado físicamente y MP-07 requiere MP-02…MP-06 aceptados.

## 2. Inventario de `@Config` / tuners Dashboard

| Clase `@Config` | Dueño | ¿Sobrevive a MP-09? |
|---|---|---|
| `LowAltitudeConstants` (raíz + `TurretConstants` + `VisionConstants` + `ShooterModelConstants`) | Software (contrato de constantes del robot) | SÍ — es el contrato central; MP-09 solo poda constantes muertas (p.ej. HOOD_* si mecánica confirma retiro definitivo) |
| `MecanumDrive.PARAMS` (Road Runner) | Tuning (valores medidos kS/kV/trackWidth) | Mientras RR sea owner; se retira con DEC-034 ya ejecutada |
| `ThreeDeadWheelLocalizer.Params` | Tuning Paso 2 (offsets/signos TBD) | Mientras RR sea owner |
| `TwoDeadWheelLocalizer` / `PinpointLocalizer` / `OTOSLocalizer` / `TankDrive` | Nadie (plantillas quickstart RR sin hardware correspondiente) | NO — candidatos a borrar en MP-09 |
| `pedroPathing/Tuning.java` (OpModes de tuning Pedro) | Tuning (cuando se calibre Pedro, Paso 2 / DEC-034) | SÍ hasta aceptar el gate de Pedro; después se revisa |
| `SystemCheckOpMode` | Software+Tuning (commissioning MP-01) | SÍ — es el System Check que MP-07 jerarquiza |
| `ShooterTuningOpMode` | Tuning Paso 5 (caracterización T8) | SÍ hasta cerrar MP-06 |
| `TestShootBurstAuto` | Nadie identificado | Revisar en MP-09; probable retiro |
| `tuning/TuningOpModes` (registro de tuners RR) | Tuning | Mientras RR sea owner |

## 3. Mapeo de telemetría actual → bloque MP-07 destino

| Fuente actual | Contenido | Bloque destino |
|---|---|---|
| `MainTeleOp.run()` L72-82 | Alianza, modo chasis/precisión | `mode` |
| `MainTeleOp.run()` L84-90 | Heading, diagnóstico joysticks | `pose` (heading) + descarte del diagnóstico de sticks tras commissioning |
| `DriveSubsystem.periodic()` | Pose X/Y/H | `pose` (vía `PoseSnapshot` del `PoseProvider`) |
| `MainTeleOp.run()` L92-95 ("SISTEMA SENTINELA") | Estado torreta sin visión | `turret` |
| `MainTeleOp.run()` L97-100 | Kicker servo opcional (solicitado/disponible/activo) | `faults` (cuando difieran) + bloque feeder |
| `ShooterSubsystem.periodic()` | Target/actual RPM, health, fault latched | `shooter` + `faults` |
| `TurretSubsystem` (commissioning) | Ticks, arming, zero state, último resultado | `turret` |
| `LimelightSubsystem` (nuevo) | Observación quality/tx/ty/staleness/latencia/botpose | `vision` (ya usa `TelemetryBlocks`) |
| `SafeCommandOpMode.addSafetyTimingTelemetry()` | Timing de stop/E-stop | se conserva tal cual (contrato de seguridad, no se migra) |

## 4. Notas de entorno de build (independiente de MP-07 pero descubierto en esta ventana)

- `JAVA_HOME` del sistema apunta a JDK 23, que Gradle 8.9 no soporta. Para línea de comandos usar el JBR de Android Studio: `JAVA_HOME=C:\Program Files\Android\Android Studio\jbr`.
- Los test workers de Gradle (`:TeamCode:testDebugUnitTest`) fallan con `GradleWorkerMain no encontrado` cuando `GRADLE_USER_HOME` vive bajo el perfil de usuario con acentos (`C:\Users\Joaquín Rosales`). Workaround verificado: `GRADLE_USER_HOME=C:\Users\Public\gradle-home` (caches ya copiados ahí). Android Studio no se ve afectado porque no usa test workers de CLI con ese home.
