# Contrato de hardware — MP-00

Estado: **mappings y puertos confirmados directamente por el operador en Driver Station; pruebas físicas pendientes**

Baseline de código: `a887fe4`

Rama de implementación: `masterplan`

Última actualización: 2026-07-20

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
- Servo puerto 0: mapping histórico `kickerServo`; configuración final confirmada sin servo.
- Odometría confirmada: `par0` lee `rightFront`, `par1` lee `leftFront` y `perp` lee `rightBack`.
- Montaje físico del Control Hub confirmado por el lead: logo hacia `RIGHT` y USB hacia `BACK`; en el SDK el valor correspondiente es `UsbFacingDirection.BACKWARD`. El código anterior declaraba `RIGHT/UP`, por lo que el signo del IMU debe retestearse con el candidato corregido antes de validar esta fila.
- Torreta: goBILDA 5203 Yellow Jacket `5203-2402-0027`, 223 RPM, 26.9:1 y 751.8 PPR oficiales en el eje de salida; engrane motor 68T a corona torreta 198T. Límites `-983/+1070`. Potencia `0.50` en recorrido y `0.05` en los últimos 100 ticks antes de límites. FND-003 y FND-020 cerrados por confirmación física del lead.

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
| IMU | `imu` | `MecanumDrive` | REV Internal IMU BHI260AP; montaje físico `RIGHT/BACK`; código `RIGHT/BACKWARD`; antihorario positivo y horario negativo | CONFIRMADO EN DS | VALIDADO 5/5 POR SENTIDO; ERROR MÁX. 4.4° |
| Pod paralelo 0 | encoder de `rightFront` | `ThreeDeadWheelLocalizer` | `par0Reversed=true`; derecha `6.75 in`; candidato RR `par0YTicks=+3518.6112`; escala `0.00191837052073273 in/tick` | CONFIRMADO POR EQUIPO; export no disponible | SIGNO/ESCALA/GEOMETRÍA MEDIDOS; RETEST ANGULAR PENDIENTE |
| Pod paralelo 1 | encoder de `leftFront` | `ThreeDeadWheelLocalizer` | `par1Reversed=true`; izquierda `6.75 in`; candidato RR `par1YTicks=-3518.6112`; escala `0.00191837052073273 in/tick` | CONFIRMADO POR EQUIPO; export no disponible | SIGNO/ESCALA/GEOMETRÍA MEDIDOS; RETEST ANGULAR PENDIENTE |
| Pod perpendicular | encoder de `rightBack` | `ThreeDeadWheelLocalizer` | `perpReversed=true`; detrás `7.00 in`; candidato RR `perpXTicks=-3648.93013333333` | CONFIRMADO POR EQUIPO; export no disponible | SIGNO/GEOMETRÍA MEDIDOS; RETEST ANGULAR PENDIENTE |
| Shooter activo | `Shooter` | `ShooterSubsystem` | Expansion Hub motor/encoder 3; goBILDA 5203 Series Yellow Jacket brushed DC 6000 RPM; cable firme/sin daño; encoder pasivo `0→28→0`; motor invertido + encoder FTCLib normal; APK `09B2...DAC1` produjo giro hacia fuera, `0→555` ticks y peak/end `1028.6/942.9 RPM` indicadas con target `1000`; fault exige reiniciar OpMode | CONFIRMADO EN DS/FÍSICO | DIRECCIÓN/ENCODER/CORTE VALIDADOS; ESTABILIDAD T8/CARGA PENDIENTES |
| Intake | `intakeMotor` | `IntakeSubsystem` | Expansion Hub 0; no invertido tras prueba funcional; límites/corriente pendientes | CONFIRMADO EN DS | VALIDADO SIN PIEZAS; CARGA/CORRIENTE PENDIENTES |
| Kicker motor | `kickerMotor` | `KickerSubsystem` | Expansion Hub 1; no invertido; +0.85/-0.7; BRAKE; único actuador final | CONFIRMADO EN DS | VALIDADO MOTOR-ONLY SIN PIEZAS; CARGA PENDIENTE |
| Kicker CRServo | `kickerServo` | Sin actuador final; salida compilada en cero | Mapping histórico Servo 0; `KICKER_SERVO_ENABLED=false` constante | RETIRADO POR DECISIÓN DEL LEAD | RETIRADO; NO REINSTALAR |
| Torreta | `torretaMotor` | `TurretSubsystem` | Expansion Hub 2; invertida; encoder/BRAKE; límites `-983/+1070`; power `0.50`, aproximación `0.05` en 100 ticks | CONFIRMADO EN DS | VALIDADO; FND-003/FND-020 CERRADOS |
| Shooter legado | `Shooter2` | Sin propietario activo | Retirado; no se mapea | RETIRADO CONFIRMADO | RETIRADO CONFIRMADO |
| Hood | `hoodLeft`, `hoodRight` | Shim legacy sin hardware | Retirado; no se mapea ni inicializa | RETIRADO CONFIRMADO | RETIRADO CONFIRMADO |
| Webcams | `Webcam 1`, `Webcam` | Sin propietario activo | Retiradas; no se mapean ni inicializan | RETIRADAS CONFIRMADAS | RETIRADAS CONFIRMADAS |
| Limelight 3A | `limelight` confirmado | Wrapper/diagnóstico sin actuadores; sin consumidor | LimelightOS/UI `2026.0` y pipeline `0` AprilTags confirmados; field map, extrínseca y gate físico pendientes; USB-A azul del Control Hub confirmado; ver [commissioning MP-03](mp03-limelight-commissioning.md) | PARCIAL | INSTALADA Y MAPPING CONFIRMADO 2026-07-21 |

## Reglas de aceptación

Un renglón sólo cambia a `VALIDADO` cuando coinciden la atestación de configuración,
la inspección física y la prueba controlada. Las constantes, direcciones o nombres no
se corregirán por inferencia. Los límites anteriores `-200/+200` fueron reemplazados
por `-983/+1070`; el positivo fue entregado por el lead con tolerancia ya aplicada y el corte motorizado final fue confirmado físicamente por el lead.
El health fail-closed del shooter está implementado pero no se autoriza energizarlo
hasta completar fault injection y validación física.
El candidato de primer pulso del 2026-07-20 está limitado a power `0.10` por `500 ms`,
un pulso por INIT y chord sostenido `gamepad1 START+A`; aún no está instalado ni
validado físicamente. No cambia el estado `RPM FÍSICA PENDIENTE` ni certifica las
constantes de encoder/relación/RPM.
El primer pulso físico terminó estable en cero, pero giró hacia dentro y reportó
encoder/RPM exactamente cero mientras hubo movimiento. No repetir ni alimentar
piezas; verificar físicamente el encoder del motor 3 con el robot desenergizado.
La inspección encontró el cable firme y sin daño visible en encoder 3. El siguiente
APK `B4CB...0988` bloquea globalmente la salida y expone ticks en vivo para una
prueba manual; no autoriza ningún pulso motorizado.
La prueba pasiva confirmó `0→28→0` ticks y ≈`85 RPM` al girar manualmente. El
candidato siguiente `2A19...254D` separa dirección física y signo de encoder,
mantiene power `0.10`/`500 ms` y corta/latcha si no ve un tick en `250 ms`.
Ese nivel no venció el umbral mecánico. El Test lead autorizó el candidato
`B294...F7EE`: power `0.50`, pulso `300 ms` y watchdog de un tick en `150 ms`,
sin piezas y con todos los demás owners del shooter bloqueados.

Por decisión del equipo, el cambio de `KICKER_OUT_SPEED` de `0.70` a `0.85` se incluye en MP-00/01A. Sigue sin estar validado físicamente y debe verificarse con hold-to-run, mecanismo contenido y acceso inmediato a Stop antes de autorizar su uso.

El equipo confirmó el 2026-07-17 que `kickerServo` es un CRServo y debe permanecer en 0 durante init/stop, usar +0.5 durante `kick()` y -0.5 durante `reverse()`. Motor y servo deben recibir cada transición en el mismo ciclo del subsistema.

Durante la iteración mecánica, `LowAltitudeConstants.KICKER_SERVO_ENABLED=false`
permite retirar el dispositivo sin bloquear INIT. `KickerSubsystem` usa `tryGet()`,
ordena cero si el servo está disponible y cae explícitamente a modo sólo-motor si
está ausente. La bandera se captura al INIT: cambiarla requiere detener y reiniciar
el OpMode. Esta opcionalidad evita el crash, pero no cierra FND-026 ni autoriza el
kicker dual mientras no cumpla sincronía mecánica.
