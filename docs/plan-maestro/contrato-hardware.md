# Contrato de hardware — MP-00

Estado: **pendiente de export y validación física**

Baseline de código: `a887fe4`

Rama de implementación: `masterplan`

Última actualización: 2026-07-17

Este documento separa lo observado en código de lo confirmado en el Robot Controller y en el robot. Un nombre en código no demuestra que el dispositivo exista, esté conectado al puerto correcto ni tenga el sentido mecánico esperado. La captura de datos y los criterios de prueba están en [08-guia-verificacion-hardware.md](08-guia-verificacion-hardware.md).

## Evidencia requerida

- Export real del Robot Controller en `docs/plan-maestro/hardware/robot-controller-config.xml`.
- Fotos legibles de hubs, puertos y cableado.
- Responsable, fecha, SHA probado y versión de Driver Hub/Robot Controller.
- Medición de sentidos, encoders, zero-power, límites y corriente donde aplique.

No se creará un XML de ejemplo: el archivo debe provenir del Robot Controller.

## Inventario observado y contrato por completar

| Componente | Nombre observado en código | Tipo/propietario en código | Puerto/dirección/encoder/zero-power/límites | Estado RC | Estado físico |
|---|---|---|---|---|---|
| Drive frontal izquierdo | `leftFront` | `MecanumDrive` | Pendiente de export y prueba | PENDIENTE | PENDIENTE |
| Drive frontal derecho | `rightFront` | `MecanumDrive` | Pendiente de export y prueba | PENDIENTE | PENDIENTE |
| Drive trasero izquierdo | `leftBack` | `MecanumDrive` | Pendiente de export y prueba | PENDIENTE | PENDIENTE |
| Drive trasero derecho | `rightBack` | `MecanumDrive` | Pendiente de export y prueba | PENDIENTE | PENDIENTE |
| IMU | `imu` | `MecanumDrive` | Orientación y signo pendientes | PENDIENTE | PENDIENTE |
| Pod paralelo 0 | `par0` | `ThreeDeadWheelLocalizer` | Puerto, signo, ticks/rev y offset pendientes | PENDIENTE | PENDIENTE |
| Pod paralelo 1 | `par1` | `ThreeDeadWheelLocalizer` | Puerto, signo, ticks/rev y offset pendientes | PENDIENTE | PENDIENTE |
| Pod perpendicular | `perp` | `ThreeDeadWheelLocalizer` | Puerto, signo, ticks/rev y offset pendientes | PENDIENTE | PENDIENTE |
| Shooter activo | `Shooter` | `ShooterSubsystem` / caminos de prueba | Dirección, encoder, ratio, RPM y límite pendientes | PENDIENTE | PENDIENTE; **no energizar por FND-015** |
| Shooter legado | `Shooter2` | Declarado, no activo en el subsystem principal | Confirmar inexistencia o propósito | PENDIENTE | PENDIENTE |
| Intake | `intakeMotor` | `IntakeSubsystem` y modos de prueba | Dirección, zero-power y límites pendientes | PENDIENTE | PENDIENTE |
| Feeder/kicker | `kickerM otor` | `KickerSubsystem` y caminos directos | Nombre exacto, dirección, zero-power, potencia y timeout pendientes | PENDIENTE | PENDIENTE |
| Torreta | `torretaMotor` | `TurretSubsystem` | Encoder; BRAKE en código; ±200 ticks **no validados** | PENDIENTE | PENDIENTE |
| Hood izquierdo | `hoodLeft` | `ShooterHoodSubsystem` | Código vigente; rango físico sin validar | PENDIENTE | REPORTADO RETIRADO; confirmar |
| Hood derecho | `hoodRight` | `ShooterHoodSubsystem` | Código vigente; rango físico sin validar | PENDIENTE | REPORTADO RETIRADO; confirmar |
| Webcam de torreta | `Webcam 1` | `MainTeleOp` / `TeleopTorreta` | Puerto USB, orientación y pose pendientes | PENDIENTE | REPORTADA RETIRADA; confirmar |
| Webcam dormante | `Webcam` | `RobotContainer.initAprilTag()` no invocado | No asumir que existe | PENDIENTE | PENDIENTE |
| Limelight propuesta | `limelight` por validar | Aún sin integración activa | Mapping, orientación, pose y pipeline pendientes | PENDIENTE | REPORTADA INSTALADA; confirmar |

## Reglas de aceptación

Un renglón sólo cambia a `VALIDADO` cuando coinciden export RC, inspección física y prueba controlada. Las constantes, direcciones o nombres no se corregirán por inferencia. Los límites ±200 de torreta permanecen provisionales y el shooter no se habilita hasta cerrar su health fail-closed.

Por decisión del equipo, el cambio de `KICKER_OUT_SPEED` de `0.70` a `0.85` se incluye en MP-00/01A. Sigue sin estar validado físicamente y debe verificarse con hold-to-run, mecanismo contenido y acceso inmediato a Stop antes de autorizar su uso.
