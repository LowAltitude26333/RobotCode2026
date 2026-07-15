# 02 — Arquitectura objetivo

> Estado: diseño aprobado, pendiente de implementación y validación física
> Baseline de referencia: `origin/main@b5a134260456565df9d0295722ebecad900f21b4`
> Última actualización: 2026-07-15
> Alcance: ownership, interfaces, estados, controles y fallas seguras
> Responsable sugerido: líder de software con aprobación de responsables de mecanismos
> Fuente de verdad: las interfaces aquí descritas son el contrato objetivo; los límites numéricos físicos siguen sujetos a medición.

## 1. Principios

1. **Un dispositivo, un propietario.** Sólo un subsystem/componente mapea cada motor, servo o sensor.
2. **Una pose, una convención.** Todos los consumidores reciben la misma pose y calidad; no reinterpretan ejes por su cuenta.
3. **Sensores proponen, seguridad decide.** Ninguna lectura visual mueve directamente un motor sin validación, límites y estado.
4. **Fallar cerrado.** Datos inválidos detienen torreta/feeder o degradan explícitamente; no reutilizan indefinidamente el último valor.
5. **Request no es permiso.** Un botón expresa intención; el interlock determina si el feeder puede actuar.
6. **Unidades en nombres/contratos.** Pulgadas, radianes, grados, RPM, ticks y milisegundos no se mezclan implícitamente.
7. **Determinismo de lifecycle.** Init, arm, run, interruption, stop y close tienen salidas definidas.
8. **Complejidad ganada por evidencia.** Empezar con fusión complementaria y modelo de RPM simple; elevar complejidad sólo si los logs lo exigen.

## 2. Composición objetivo

`CompetitionTeleOp` será el único composition root de producción y construirá un solo `RobotContainer`. El container poseerá o construirá:

```text
RobotContainer
├── DriveSubsystem
│   ├── PedroDriveAdapter
│   └── PedroPoseProvider
├── IntakeSubsystem
├── FeederSubsystem
├── ShooterSubsystem
├── TurretSubsystem
├── LimelightSubsystem
├── FusedPoseEstimator
├── TargetingCoordinator
├── ShotPlanner
├── ReadinessInterlock
├── OperatorInterface
└── RobotSafety / FaultManager
```

El `SystemCheckOpMode` reutilizará los mismos subsystems o una factory común, pero no la lógica completa de competencia. Su controlador de diagnóstico tendrá permisos hold-to-run, potencia limitada y timeout. No mapeará hardware por nombres alternos.

Hood y webcam se consideran físicamente retirados según el reporte del equipo; no forman parte del target. Limelight está instalada pero `TBD-BLOCKING` hasta confirmar configuración. Road Runner, VisionPortal de targeting y código alterno permanecen sólo hasta el gate de limpieza.

## 3. Flujo de datos

```text
Dead wheels + IMU
       |
       v
 PedroPoseProvider -----> odometry pose + covariance/quality + timestamp
       |                                      |
       |                                      v
       |          LimelightSubsystem --> VisionObservation
       |                                      |
       +------------------> FusedPoseEstimator <---- gates/residuals
                                  |
                                  v
                      FieldPose + PoseQuality
                                  |
             Alliance + goal map--+
                                  v
                      TargetingCoordinator
                       |                 |
                       v                 v
              turret setpoint       distance to shot point
                       |                 |
                       v                 v
              TurretSubsystem       ShotPlanner/RpmModel
                       |                 |
                       +--------+--------+
                                v
                      ReadinessInterlock <--- held feed request/mode/faults
                                |
                                v
                        FeederSubsystem
```

Limelight no escribe directamente en la torreta. Puede producir dos aportaciones separadas:

- una observación de pose para corregir lentamente el estimador;
- un residual angular directo al tag correcto para un trim final de aim, limitado y fresco.

Separar ambos evita convertir un `tx` ruidoso en la única verdad del robot.

## 4. Contratos de tipos

El proyecto actual usa Java compatible con Android/FTC. No se necesitan `record`, sealed classes ni características nuevas de Java. Las formas siguientes son especificaciones, no código listo para copiar.

### 4.1 Enumeraciones centrales

```java
enum Alliance { BLUE, RED, UNSELECTED }

enum PoseQuality {
    UNINITIALIZED,
    ODOMETRY_ONLY,
    FUSED_GOOD,
    DEGRADED,
    INVALID
}

enum TargetingMode {
    DISABLED,
    NORMAL_FUSED,
    ODOMETRY_ONLY,
    DEGRADED_FIXED_FORWARD
}

enum ShooterMode {
    AUTO_DISTANCE,
    MANUAL_RPM
}

enum TurretState {
    DISARMED,
    ARM_HOLD,
    ARMED_IDLE,
    TRACKING,
    READY,
    AT_LIMIT,
    ZERO_INVALID,
    FAULT
}

enum ShooterState {
    OFF,
    SPINNING,
    STABLE,
    SENSOR_FAULT,
    OVERSPEED_FAULT
}

enum FeederState {
    IDLE,
    WAITING_READINESS,
    PULSING,
    COOLDOWN,
    FAULT
}
```

`UNSELECTED` y `UNINITIALIZED` son estados reales, no aliases de RED o pose cero. Iniciar sin seleccionar no debe ocultarse con defaults.

### 4.2 Pose provider

```java
interface PoseProvider {
    FieldPose getPose();
    PoseQuality getQuality();
    long getTimestampNanos();
    void setPose(FieldPose pose, PoseResetReason reason);
    void update();
    void stop();
}
```

`FieldPose` debe ser inmutable y declarar unidades, por ejemplo `xInches`, `yInches`, `headingRadians`. No exponer el tipo de pose de Pedro fuera del adapter. `setPose` sólo se usa en init/relocalización deliberada, no para copiar cada frame de cámara.

### 4.3 Control del drivetrain

`DriveSubsystem` es el único dueño de los cuatro motores y expone conducción field/robot-centric, seguimiento de movimiento, actualización de pose y `stop()`. `PedroDriveAdapter` implementa esas operaciones y entrega el `PoseProvider` asociado. Ningún consumidor de producción conserva un `MecanumDrive` Road Runner paralelo; el rollback restaura el paquete completo desde Git.

### 4.4 Observación de visión

```java
final class VisionObservation {
    final boolean valid;
    final long captureTimestampNanos;
    final long receiveTimestampNanos;
    final double latencyMs;
    final int primaryTagId;
    final Double bearingRadians;
    final FieldPose robotPose;
    final int tagCount;
    final double ambiguityOrQuality;
    final String rejectionHint;
}
```

La clase debe copiar los valores necesarios de la respuesta del SDK y no conservar una referencia mutable al resultado de Limelight. Si un campo no está disponible, usar ausencia explícita, no cero.

### 4.5 Solución de targeting

```java
final class TargetingSolution {
    final TargetingMode mode;
    final boolean usable;
    final double fieldDistanceInches;
    final double desiredTurretAngleRadians;
    final double odometryBearingRadians;
    final double appliedVisionTrimRadians;
    final PoseQuality poseQuality;
    final String blockReason;
}
```

La solución es un cálculo puro: no escribe motores. Si el ángulo deseado rebasa el arco validado, `usable=false` y la razón indica que el conductor debe orientar el chasis.

### 4.6 Modelo de tiro

```java
interface RpmModel {
    double rpmForDistanceInches(double distanceInches);
    boolean isDistanceSupported(double distanceInches);
    String modelVersion();
}

final class ShotSolution {
    final ShooterMode mode;
    final double baseRpm;
    final double operatorTrimRpm;
    final double commandedRpm;
    final boolean clamped;
    final String modelVersion;
}
```

El clamp físico se aplica después de base + trim. El operador no puede deshabilitar ni el límite normal de ±500 RPM ni el clamp físico absoluto desde gamepad.

### 4.7 Readiness

```java
final class ReadinessState {
    final boolean feedAllowed;
    final boolean requestHeld;
    final boolean shooterSensorHealthy;
    final boolean shooterStable;
    final boolean turretZeroValid;
    final boolean turretReady;
    final boolean poseOrDegradedModeReady;
    final boolean feederHealthy;
    final boolean feederPulseAvailable;
    final boolean driveStationary;
    final long stableForMs;
    final String primaryBlockReason;
}
```

`driveStationary` exige que velocidad lineal y angular estén por debajo de umbrales medidos. El feeder consume sólo `feedAllowed`, no repite la lógica, y su state machine impone duración máxima/cooldown. Telemetry publica cada condición para que el operador sepa qué falta.

## 5. Ownership y lifecycle por componente

| Componente | Posee | Inicialización | Salida segura | Prohibido |
|---|---|---|---|---|
| `DriveSubsystem` | 4 motores, `PedroDriveAdapter`, pose/IMU según diseño | Configura sin movimiento, pose sólo tras confirmación | Power cero y stop de follower | Road Runner controlando movimiento/pose en paralelo. |
| `PedroPoseProvider` | Estado del localizador, no actuadores extra | Calidad `UNINITIALIZED` hasta pose y calibración | `INVALID`/stop de updates | Exponer ejes Pedro directamente a targeting. |
| `LimelightSubsystem` | `Limelight3A` | Pipeline/poll rate/start después de config | stop/close; observación inválida | Llamar motor o setear pose sin gate. |
| `TurretSubsystem` | `torretaMotor` | Brake, power 0, desarmada | power 0, desarmar en fault/stop | Reset automático suponiendo centro; ignorar limits. |
| `ShooterSubsystem` | `Shooter` | target 0, controller conocido, health desconocido hasta sensor válido | target 0 y power 0 | Convertir encoder/voltaje inválido en demanda máxima o reactivar tras stop. |
| `FeederSubsystem` | `kickerM otor` inicialmente | power 0, estado `IDLE` | power 0, cancelar pulso/cooldown | Binding directo o on-time sin máximo. |
| `IntakeSubsystem` | `intakeMotor` | power 0 | power 0 | Estado latched sin indicador/stop. |
| `RobotSafety` | Registro de callbacks, no hardware map duplicado | Activo antes de comandos | `stopAll`, scheduler cancel/disable | Depender de telemetry o cámara para parar. |

## 6. Init y armado

El TeleOp no debe saltar directamente de construcción a tracking. La secuencia objetivo es:

1. **BOOT:** mapear sólo hardware verificado, ordenar ceros, iniciar fault manager y dejar cada `TBD-BLOCKING` inhibido.
2. **ALLIANCE_SELECT:** `gamepad1 X = BLUE`, `gamepad1 B = RED`; mostrar selección grande.
3. **POSE_SELECT:** proponer preset de start por alianza; el conductor lo confirma o selecciona otro preset permitido.
4. **VISION_CHECK:** iniciar Limelight, mostrar pipeline, edad de frame y tags; un fallo no mueve nada.
5. **TURRET_CENTER:** colocar físicamente la torreta en la marca central.
6. **ARM_HOLD:** operador 2 mantiene `START+BACK` durante 1 s. Soltar reinicia el progreso. Al completar, se pone encoder a cero y `zeroValid=true`; cada init/reset/brownout lo invalida.
7. **READY_TO_START:** mostrar alianza, pose, turret armed, visión y faults. `start()` bloquea alianza/preset para el match.
8. **RUN:** scheduler normal; cambios peligrosos requieren chords state-aware y feedback.
9. **STOP:** cancelar comandos, potencia cero en todo, invalidar armado, detener Limelight y deshabilitar scheduler.

`SafeCommandOpMode` conservará una inicialización única y añadirá un hook de init repetido antes de START. Ese hook procesa selección, E-stop, telemetry y `ARM_HOLD` sin permitir movimiento automático; Stop debe salir sin esperar a START.

Si la torreta no se arma, el TeleOp todavía puede conducir/intake según la política aprobada, pero no puede auto-apuntar ni alimentar. El UI no debe confundir “TeleOp corriendo” con “sistema de tiro listo”.

## 7. Máquina de estados de targeting

### 7.1 Estado normal

`NORMAL_FUSED` requiere:

- alianza seleccionada;
- pose `FUSED_GOOD` o calidad explícitamente aceptada;
- torreta armada y encoder plausible;
- goal dentro del arco;
- ausencia de fault crítico.

La odometría calcula el bearing en cada ciclo. Una observación visual fresca puede corregir pose y/o aplicar trim acotado.

### 7.2 Odometría solamente

Si la cámara se pierde pero la pose conserva calidad suficiente:

- el modo pasa a `ODOMETRY_ONLY`;
- la torreta sigue el bearing geométrico;
- telemetry/LED/rumble indican pérdida de visión;
- los criterios de feed pueden ser más estrictos o bloquear a partir de un tiempo/distancia de deriva configurado.

El tiempo permitido debe salir de pruebas de deriva, no ser infinito.

### 7.3 Target fuera del arco

- detener torreta antes del soft limit;
- `feedAllowed=false`;
- mostrar flecha/dirección sugerida al conductor;
- generar rumble distintivo con rate limit;
- no comandar giro del drivetrain.

Cuando el conductor reorienta el chasis y el setpoint vuelve al rango con histéresis, tracking puede continuar.

### 7.4 `DEGRADED_FIXED_FORWARD`

Este es un procedimiento de emergencia deliberado cuando pose y visión no son confiables, no un fallback automático silencioso.

Entrada propuesta durante run: operador 2 sostiene `START+BACK` 1 s cuando la torreta ya está armada; el mismo chord en init sólo arma. El estado de la partida elimina la ambigüedad. La entrada requiere confirmación visible/rumble y puede cancelarse con el mismo hold o E-stop.

Restricciones:

- torreta va únicamente al cero calibrado/frontal dentro de límites;
- si no está armada o su encoder es implausible, no se mueve;
- conductor apunta el chasis manualmente;
- shooter usa `MANUAL_RPM` dentro del rango físico;
- feeder necesita request sostenido, turret-at-zero, RPM dwell, timeout y ausencia de faults;
- no se afirma pose ni se aceptan correcciones visuales;
- telemetry muestra `DEGRADED` de forma dominante.

## 8. Control de torreta

La salida deseada se calcula así conceptualmente:

```text
bearing_field = atan2(goalY - robotY, goalX - robotX)
bearing_robot = wrap(bearing_field - robotHeading)
turret_target = wrap(bearing_robot - turretZeroOffset + calibratedAimOffset)
turret_target += gatedVisionBearingTrim
```

Antes de controlar:

- convertir al equivalente dentro del arco, sin seleccionar una vuelta prohibida;
- aplicar soft limits con margen e histéresis;
- reducir velocidad/potencia dentro de una zona de frenado antes del límite;
- si la medida ya aparece fuera del rango, permitir sólo movimiento de regreso;
- validar encoder y velocidad;
- limitar setpoint rate, potencia y término integral;
- exigir dwell dentro de tolerancia para `READY`.

La convención exacta de signos se fija con pruebas, no copiando el signo de la implementación visual actual.

## 9. Shooter, trim y feeder

### 9.1 Modos

`AUTO_DISTANCE`:

- obtiene distancia desde `TargetingSolution`;
- rechaza distancias fuera del rango calibrado o aplica sólo una extrapolación expresamente aprobada;
- calcula RPM base con el `RpmModel` versionado;
- suma trim del operador;
- aplica slew y clamp absoluto.

`MANUAL_RPM`:

- conserva un setpoint base ajustable dentro del rango validado;
- se usa en tuning controlado y modo degradado;
- no implica permiso automático de feed.

### 9.2 Controles aprobados del operador 2

| Control | Acción | Protección |
|---|---|---|
| `A` | Armar/iniciar setpoint manual | Sólo con health sano y constantes verificadas. |
| `B` | Detener shooter | Target y power cero; no toggle. |
| `DPAD_UP` | +100 RPM por flanco | Debounce; trim normal máximo +500. |
| `DPAD_DOWN` | -100 RPM por flanco | Debounce; trim normal mínimo -500. |
| `X` | Restablecer trim a 0 | Confirmación breve en telemetry/rumble. |
| `Y` | Alternar `AUTO_DISTANCE` / `MANUAL_RPM` después de MP-06 | Sólo por flanco; sin efecto antes del gate. |
| `START+BACK` en init | Armar cero de torreta tras 1 s | Sólo en estado de init correcto. |
| `START+BACK` en run | Entrar/salir de degradado tras 1 s | Sólo si estado permite; feedback distintivo. |

No existe override de trim o clamp desde gamepad. Si más adelante se necesita ampliar el rango, requiere una decisión nueva, revisión mecánica y dataset; no se resuelve con una constante Dashboard.

### 9.3 Readiness central

En modo normal:

```text
feedAllowed = requestHeld
           && shooterSensorHealthy
           && shooterWithinToleranceForDwell
           && turretZeroValid
           && turretWithinToleranceForDwell
           && targetingSolutionUsable
           && poseQualityAllowed
           && feederHealthy
           && feederPulseAvailable
           && driveStationary
           && !criticalFault
```

Mientras `gamepad1 RB` se sostiene, readiness sólo permite comenzar un pulso acotado; al terminar entra en cooldown y vuelve a evaluar todas las condiciones antes de otra pieza. La pérdida de una condición crítica corta feeder en el primer scheduler cycle y ≤50 ms. No se permite completar un pulso tras release, E-stop, limit/fault o sensor inválido.

## 10. Alianza, pose inicial y goal

- `X = BLUE`, `B = RED` durante init.
- Ninguna alianza por default se considera confirmada.
- Goal tag: Blue 20; Red 24.
- La pose preset se selecciona por start location, no se deriva sólo de la alianza.
- Limelight puede validar/corregir la pose inicial si su observación pasa gates; no cambia la alianza.
- La selección queda locked al comenzar. Un cambio durante run sólo sería posible mediante procedimiento técnico no disponible al operador normal.

## 11. Fault model

| Fault | Detección | Acción segura | Recuperación |
|---|---|---|---|
| Limelight desconectada/stale | Sin frame válido por timeout | Invalidar visión; continuar odometría si confiable | Automática tras frames frescos y gate. |
| Pose inválida | Timestamp viejo, salto, localizador fault | Stop turret/feed; permitir drive manual | Reinit deliberado o degradado. |
| Turret encoder implausible | Salto/velocidad/límite incompatible | Power 0, disarm, feed block | Inspección y rearm explícito. |
| Turret target fuera de arco | Setpoint fuera de safe range | Stop/hold seguro, feed block, guía driver | Driver gira chasis; histéresis. |
| Shooter overspeed | RPM > límite/tolerancia | Target/power 0 o estrategia validada; feed block | Fault reset tras stop e inspección. |
| Shooter encoder/voltaje inválido | Pulsos ausentes/congelados, valor imposible o voltaje fuera de rango | `SENSOR_FAULT`, target/power 0, feed block | Inspección y rearm deliberado tras datos válidos. |
| Shooter no llega a RPM | Dwell timeout | Feed block, seguir/stop según política | Operador ajusta o aborta; log. |
| Feeder jam | Corriente/tiempo si disponible | Stop; reversa sólo por acción sostenida validada | Procedimiento manual. |
| E-stop | `gamepad1 BACK`; fallback validado `START+Y` 0.5 s | Latch, cancel scheduler y `stopAll` inmediato | Reiniciar OpMode e inspeccionar. |
| Stop de OpMode | Lifecycle | Todos cero, cámara stop/close | Nuevo init. |

## 12. Telemetría mínima

La pantalla principal debe responder en menos de dos segundos:

- ¿qué alianza y preset están activos?;
- ¿qué modo de targeting/shooter está activo?;
- ¿pose y visión son confiables?;
- ¿la torreta está armada y dentro de rango?;
- ¿cuál es RPM base, trim, target y medida?;
- ¿se permite feed? Si no, ¿cuál es la razón principal?;
- ¿existe degradado o E-stop latched?;
- ¿qué fault requiere acción?

Los datos detallados se registran en logs/Dashboard; el Driver Station muestra decisiones, no una pared de números.

## 13. Reglas de implementación incremental

- Introducir primero interfaces/adapters con el comportamiento actual; después cambiar proveedores.
- No mezclar migración Pedro, renombrado de feeder y eliminación de hood en un solo diff.
- Toda clase que ordene un actuador declara requirement o pasa por el dueño único.
- Toda prueba de subsystem incluye stop/interrupted/fault.
- No crear un segundo `RobotContainer` para el nuevo stack.
- No copiar arquitecturas externas completas; adoptar patrones pequeños después de licencia, versión API y safety review.
- No habilitar tracking en el mismo PR que introduce constantes físicas sin validar.

## 14. Criterio de aceptación arquitectónico

La arquitectura queda aceptada cuando una revisión puede demostrar, desde cada input hasta cada motor:

1. el dueño único del hardware;
2. las unidades y convenciones;
3. el estado que habilita la salida;
4. los clamps y límites;
5. el comportamiento con dato nulo/viejo/imposible;
6. la ruta de interrupción/E-stop/stop;
7. la evidencia de prueba.

Si cualquiera requiere “porque normalmente la cámara funciona” o “porque el operador no presionará eso”, el contrato todavía no es robusto.
