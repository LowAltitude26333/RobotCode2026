# Contrato de hardware — MP-00

Estado: **mappings y puertos confirmados directamente por el operador en Driver Station; pruebas físicas pendientes**

Baseline de código: `a887fe4`

Rama de implementación: `masterplan`

Última actualización: 2026-07-17

Este documento separa lo observado en código de lo confirmado en el Robot Controller y en el robot. Un nombre en código no demuestra que el dispositivo exista, esté conectado al puerto correcto ni tenga el sentido mecánico esperado. La captura de datos y los criterios de prueba están en [08-guia-verificacion-hardware.md](08-guia-verificacion-hardware.md).

## Evidencia de configuración disponible

El 2026-07-17 el operador verificó directamente `Configure Robot` con el robot
deshabilitado. La versión instalada no expone descarga del XML desde `Manage` y
`config.html` responde HTTP 404, por lo que MP-01 conserva esta atestación trazable
como evidencia sustituta; no se fabricará un XML.

- Robot Controller: `11.0`.
- Driver Station: `11.1`.
- Control Hub OS: `1.1.6`.
- Driver Hub OS: `1.2.0`.
- Expansion Hub firmware: `1.8.2`.
- IMU integrada: REV Internal IMU `BHI260AP`.
- Control Hub motores: puerto 0 `rightFront`, 1 `rightBack`, 2 `leftFront`, 3 `leftBack`.
- Expansion Hub motores: puerto 0 `intakeMotor`, 1 `kickerMotor`, 2 `torretaMotor`, 3 `Shooter`.
- Servo puerto 0: `kickerServo`, configurado como `CRServo`.
- Odometría confirmada: `par0` lee `rightFront`, `par1` lee `leftFront` y `perp` lee `rightBack`.
- Torreta: goBILDA 5203 Yellow Jacket `5203-2402-0027`, 223 RPM, 26.9:1 y 751.8 PPR oficiales en el eje de salida; engrane motor 68T a corona torreta 198T. Conversión teórica `2189.06 ticks/rev` de torreta=`6.0807 ticks/grado`; los grados quedan sólo como estimación. Medición pasiva desde la marca central: +1070 ticks horarios y -988 ticks antihorarios; retorno a la misma marca=-22 ticks sin reinicio. Los ticks son la referencia primaria, pero aún no se consideran límites calibrados hasta caracterizar repetibilidad/backlash y causa física de cada extremo.

## Evidencia adicional requerida

- Capturas o fotos de hubs, puertos y cableado cuando el equipo pueda conservarlas.
- Responsable, fecha, SHA probado y versión de Driver Hub/Robot Controller.
- Medición de sentidos, encoders, zero-power, límites y corriente donde aplique.

No se creará un XML de ejemplo. Si una versión posterior permite exportarlo, el
archivo deberá provenir del Robot Controller.

## Inventario confirmado y contrato por completar

| Componente | Nombre observado en código | Tipo/propietario en código | Puerto/dirección/encoder/zero-power/límites | Estado RC | Estado físico |
|---|---|---|---|---|---|
| Drive frontal izquierdo | `leftFront` | `MecanumDrive` | Control Hub 2; invertido; encoder usado como `par1` | CONFIRMADO EN DS | VALIDADO ELEVADO |
| Drive frontal derecho | `rightFront` | `MecanumDrive` | Control Hub 0; no invertido; encoder usado como `par0` | CONFIRMADO EN DS | VALIDADO ELEVADO |
| Drive trasero izquierdo | `leftBack` | `MecanumDrive` | Control Hub 3; invertido | CONFIRMADO EN DS | VALIDADO ELEVADO |
| Drive trasero derecho | `rightBack` | `MecanumDrive` | Control Hub 1; invertido; encoder usado como `perp` | CONFIRMADO EN DS | VALIDADO ELEVADO |
| IMU | `imu` | `MecanumDrive` | REV Internal IMU BHI260AP; orientación y signo pendientes | CONFIRMADO EN DS | PENDIENTE |
| Pod paralelo 0 | encoder de `rightFront` | `ThreeDeadWheelLocalizer` | Conexión confirmada; signo, ticks/rev y offset pendientes | CONFIRMADO POR EQUIPO; falta export | PENDIENTE CALIBRACIÓN |
| Pod paralelo 1 | encoder de `leftFront` | `ThreeDeadWheelLocalizer` | Conexión confirmada; signo, ticks/rev y offset pendientes | CONFIRMADO POR EQUIPO; falta export | PENDIENTE CALIBRACIÓN |
| Pod perpendicular | encoder de `rightBack` | `ThreeDeadWheelLocalizer` | Conexión confirmada; signo, ticks/rev y offset pendientes | CONFIRMADO POR EQUIPO; falta export | PENDIENTE CALIBRACIÓN |
| Shooter activo | `Shooter` | `ShooterSubsystem` | Expansion Hub 3; un motor, invertido, máximo seguro declarado 6000 RPM; fault exige reiniciar OpMode | CONFIRMADO EN DS | FAULT INJECTION VALIDADA; RPM FÍSICA PENDIENTE |
| Intake | `intakeMotor` | `IntakeSubsystem` | Expansion Hub 0; no invertido tras prueba funcional; límites/corriente pendientes | CONFIRMADO EN DS | VALIDADO SIN PIEZAS; CARGA/CORRIENTE PENDIENTES |
| Kicker motor | `kickerMotor` | `KickerSubsystem` | Expansion Hub 1; no invertido; +0.85/-0.7; BRAKE | CONFIRMADO EN DS | BLOQUEADO: SINCRONÍA MECÁNICA |
| Kicker CRServo | `kickerServo` | `KickerSubsystem` | Servo 0; goBILDA 2000-0025-0002 en modo continuo; opcional con `KICKER_SERVO_ENABLED`; +0.5/-0.5/0 | CONFIRMADO EN DS COMO CRSERVO | BLOQUEADO: 0.5–1.0 s DE DEMORA |
| Torreta | `torretaMotor` | `TurretSubsystem` | Expansion Hub 2; goBILDA 5203 223 RPM/26.9:1, 751.8 PPR; engranes 68T→198T; teórico 6.0807 ticks/grado; invertida; encoder/BRAKE; signo positivo=horario visto desde el frente; ±200 ticks≈±32.89° **no validados** | CONFIRMADO EN DS | MOTOR/RELACIÓN/SIGNO CONFIRMADOS; ÁNGULO/LÍMITES/CERO/BROWNOUT PENDIENTES |
| Shooter legado | `Shooter2` | Sin propietario activo | Retirado; no se mapea | RETIRADO CONFIRMADO | RETIRADO CONFIRMADO |
| Hood | `hoodLeft`, `hoodRight` | Shim legacy sin hardware | Retirado; no se mapea ni inicializa | RETIRADO CONFIRMADO | RETIRADO CONFIRMADO |
| Webcams | `Webcam 1`, `Webcam` | Sin propietario activo | Retiradas; no se mapean ni inicializan | RETIRADAS CONFIRMADAS | RETIRADAS CONFIRMADAS |
| Limelight propuesta | `limelight` por validar | Aún sin integración activa | Mapping, orientación, pose y pipeline pendientes | PENDIENTE | REPORTADA INSTALADA; confirmar |

## Reglas de aceptación

Un renglón sólo cambia a `VALIDADO` cuando coinciden la atestación de configuración,
la inspección física y la prueba controlada. Las constantes, direcciones o nombres no
se corregirán por inferencia. Los límites ±200 de torreta permanecen provisionales.
El health fail-closed del shooter está implementado pero no se autoriza energizarlo
hasta completar fault injection y validación física.

Por decisión del equipo, el cambio de `KICKER_OUT_SPEED` de `0.70` a `0.85` se incluye en MP-00/01A. Sigue sin estar validado físicamente y debe verificarse con hold-to-run, mecanismo contenido y acceso inmediato a Stop antes de autorizar su uso.

El equipo confirmó el 2026-07-17 que `kickerServo` es un CRServo y debe permanecer en 0 durante init/stop, usar +0.5 durante `kick()` y -0.5 durante `reverse()`. Motor y servo deben recibir cada transición en el mismo ciclo del subsistema.

Durante la iteración mecánica, `LowAltitudeConstants.KICKER_SERVO_ENABLED=false`
permite retirar el dispositivo sin bloquear INIT. `KickerSubsystem` usa `tryGet()`,
ordena cero si el servo está disponible y cae explícitamente a modo sólo-motor si
está ausente. La bandera se captura al INIT: cambiarla requiere detener y reiniciar
el OpMode. Esta opcionalidad evita el crash, pero no cierra FND-026 ni autoriza el
kicker dual mientras no cumpla sincronía mecánica.
