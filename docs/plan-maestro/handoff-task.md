# Handoff actual — commissioning del shooter T8.1

> **SIGUIENTE PASO ÚNICO:** instalar el APK RAW `FE7A...B1AE`, abrir
> `DIAG: Shooter Encoder RAW`, pulsar INIT/PLAY y girar el eje exactamente diez
> vueltas hacia fuera con potencia hardcoded en cero. Registrar `Diag/Connection`,
> `Raw delta`, `Outward delta`, `Changed samples` y `Raw min..max`. No energizar
> el shooter, no cambiar gains y no sustituir cable/puerto hasta obtener este
> resultado de frontera.

## Bloqueo vigente — FND-029

- FND-029 está `CRITICAL / INVESTIGATING` y bloquea T8.1, `1500+ RPM`, piezas,
  feeder y autos de tiro.
- Hecho físico actual: motor y rueda del shooter son 1:1. El valor de software
  heredado `SHOOTER_GEAR_RATIO=2.0` se corrigió a `1.0`; no se modificó ninguna
  relación mecánica.
- Bug reproducible: con potencia cero, el eje gira manualmente y `LoopCount`
  avanza, pero todas las lecturas de posición/velocidad permanecen en cero. Bajo
  potencia se han observado pocas cuentas y picos instantáneos incompatibles con
  la posición acumulada.
- La hipótesis de bulk cache quedó refutada por retest con `AUTO` y limpieza
  explícita por ciclo; ese workaround se retiró de `SystemCheck`.
- La afirmación previa de encoder corrupto tampoco está demostrada: una captura
  motorizada mostró posición raw monotónica. La causa sigue abierta entre
  configuración/puerto REV, adaptador/cable, alimentación/canales A-B y encoder.
- FND-028 conserva su cierre histórico para `09B2...DAC1`, pero su evidencia no
  se transfiere al candidato/configuración actual.

## Candidato RAW actual — no instalado

- OpMode: `DIAG: Shooter Encoder RAW`.
- Sin FTCLib, scheduler, PID, feedforward ni reset continuo.
- `DcMotorEx` directo a `RobotMap.SHOOTER_MOTOR`; bulk caching `OFF`.
- Potencia sin binding y fijada a `0.0` en init, cada loop y stop.
- Telemetría: nombre/configuración, `getConnectionInfo()`, modo SDK, posición y
  velocidad raw/outward, delta desde START, min/max y muestras modificadas.
- APK: `C:\dev\RobotCode2026\TeamCode\build\outputs\apk\debug\TeamCode-debug.apk`.
- Tamaño: `81,370,478` bytes.
- SHA-256:
  `FE7A859729BB03681EB630075AF6886EC80D3661FE29EE06DD3EEECD61B4B1AE`.
- Verificación final: `74/74` tests JVM, cero
  failures/errors/skips, `assembleDebug` y `git diff --check` PASS.

### Decisión después de la única prueba RAW

- `Outward delta≈+280` en diez vueltas: el path SDK/puerto/encoder cuenta; pasar
  a caracterización abierta instrumentada con corriente y voltaje.
- Delta `0`: apagar robot y ejecutar matriz cruzada de encoder/cable/puerto; no
  compilar otro controlador.
- Delta distinto de cero pero muy lejos de `280`, discontinuo o unidireccional:
  tratar como señal cuadratura incompleta/intermitente y ejecutar la misma matriz.
- Sólo después de `280±1` en ambos sentidos se permite calcular `kS/kV`; `kP`
  se ajusta después. `kI/kD` permanecen cero salvo evidencia.

## Cambios de software acumulados en este bloque

- `AGENTS.md`: fija `C:\dev\RobotCode2026` como único workspace autoritativo.
- `RobotMap`: separa el signo outward del encoder del sentido físico del motor.
- `ShooterFeedbackSign` y tests: normalización raw/outward explícita.
- `ShooterVelocityWindow` y tests: diagnóstico independiente por delta de
  posición en ventana de 100 ms.
- `ShooterSubsystem`: publica posición, velocidad SDK, velocidad FTCLib y RPM por
  ventana sin eliminar clamps, watchdogs, timeout ni stop.
- `SystemCheckOpMode`: crea las claves diagnósticas desde INIT y muestra los tres
  caminos de medición durante T8.1.
- `LowAltitudeConstants`: corrige exclusivamente la relación externa confirmada
  `2.0→1.0`; conserva `28 ticks/rev`, `6000 RPM` y gains existentes.
- `ShooterHardwareConfigTest`: protege 28 ticks/rev, 6000 RPM y relación 1:1.
- `ShooterEncoderRawDiagnosticOpMode`: diagnóstico de frontera con potencia cero.
- Intento de caché REV: implementado para probar la hipótesis, refutado físicamente
  y retirado; no forma parte del diff final.

## Workspace, alcance y estado

- Workspace autoritativo obligatorio: `C:\dev\RobotCode2026`.
- La copia `C:\Users\brito\OneDrive\Documentos\FTC\RobotCode2026` no es
  autoritativa: no editar, compilar, desplegar ni tomar APK/hashes desde ella.
- Rama/base inspeccionada: `masterplan@f272792`; el working tree contiene cambios
  deliberados sin commit y evidencia del equipo que se debe preservar.
- MP-03 está cerrado por excepción documentada. No reabrirlo.
- Alcance activo exclusivo: commissioning sin carga del shooter, T8.1.
- Fuera de alcance: MP-04, fusión, auto-aim, torreta, feeder, intake y piezas.
- `3600 RPM` permanece bloqueado.
- Una sola prueba física por interacción; el humano decide gate o aborto.

## Candidato diagnóstico probado físicamente

- APK: `C:\dev\RobotCode2026\TeamCode\build\outputs\apk\debug\TeamCode-debug.apk`.
- Tamaño: `81,335,740` bytes.
- SHA-256:
  `CCD709023E403963533297AC8F5AF221F57B6F4079B96209ADB937681D68428F`.
- Verificación software: `73/73` pruebas JVM, cero failures/errors,
  `:TeamCode:testDebugUnitTest`, `assembleDebug` y `git diff --check` PASS.
- Cambio exclusivamente diagnóstico: conserva control, gains, relación, dirección,
  límite `0.75`, watchdogs y stop; añade comparación de velocidad SDK cruda,
  FTCLib corregida y RPM calculada por delta de ticks en ventana de `100 ms`.
- Gráfica requerida: `Shooter/Target`, `Shooter/Actual`,
  `Shooter/DiagSdkRPM` y `Shooter/DiagWindowRPM`.

### Resultado físico `CCD7...428F`

- `STOPPED_TIMEOUT`, `3007.8 ms`, ticks outward `0 -> 72` y raw `0 -> -72`.
- Peak/end indicado `1028.6/685.7 RPM`; ready hold `0.0 ms`.
- Batería `13.26 -> 12.75 V`, mínima `12.47 V`.
- Power final `0`, health=`HEALTHY`, fault=`none`.
- Safety stop: mismo ciclo, `0.129 ms`, gate 50 ms PASS y cero fallos.
- La gráfica simultánea demuestra que `Actual`/FTCLib y SDK RPM siguen la misma
  señal instantánea errática; la estimación independiente por delta de posición
  en `100 ms` también cambia de signo y sólo acumula `72` ticks netos.
- La salida física sólo permite potencia `0..0.75` y el operador confirmó giro
  continuo hacia fuera. Por tanto, los cambios de signo no corresponden a una
  reversa ordenada ni a la corrección de overflow de FTCLib.

Conclusión de diagnóstico: el fallo está aguas arriba de los tres cálculos de
software, en la señal cuadratura que llega al Hub. La hipótesis principal es un
canal intermitente/ausente por cable, conector, encoder interno o entrada del Hub;
la prueba no distingue todavía cuál de esos componentes. No filtrar, suavizar ni
ajustar PID para ocultarlo.

## Evidencia del candidato corregido `2E28...52C2`

- APK histórico; la ruta de build local ahora contiene el candidato diagnóstico
  `CCD7...428F`.
- Tamaño: `81,333,611` bytes.
- SHA-256:
  `2E28D228462EAE23C22D37008D932935BDBC2D02AE502A35BF3BE8B7C4EE52C2`.
- Estado físico: instalado y probado; no pasó T8.1.
- Verificación software: `70/70` pruebas JVM, cero failures/errors,
  `:TeamCode:testDebugUnitTest`, `assembleDebug` y `git diff --check` PASS.
- Advertencias de compilación sobre Java 8/JDK 21 son preexistentes; no se cambió
  Gradle, SDK ni dependencias.

Dos intentos parciales se conservaron separados porque FTC Dashboard no permitió
dos gráficas simultáneas:

- Intento 1: gráfica `Target/Actual`, sin telemetría final coincidente; sólo
  diagnóstico visual, no evidencia de gate.
- Intento 2: `STOPPED_TIMEOUT`, `3005.9 ms`, ticks `0 -> 93`, encoder responded,
  peak/end indicado `685.7/685.7 RPM`, batería `13.73 -> 13.46 V`, mínima
  `12.84 V`, ready hold `0.0 ms`, output limit/potencia durante pulso `0.75`,
  potencia final `0`, health=`HEALTHY`, fault=`none`, giro rápido hacia fuera,
  paro completo y sin ruido, vibración, roce, olor ni calentamiento. Safety stop:
  mismo ciclo, `0.044 ms`, gate 50 ms PASS y cero fallos.

Conclusión: T8.1 a `1000 RPM` no pasa. Batería, potencia ordenada y stop quedaron
descartados como causa inmediata; la lectura instantánea de velocidad no concuerda
con los ticks acumulados. No ajustar `kS`, `kV` ni `kP` hasta comparar los tres
paths de medición con el APK diagnóstico.

La prueba pasiva manual tampoco se usa como evidencia: con loop activo y potencia
cero, el eje del motor giró físicamente pero `Encoder live` no cambió; bajo potencia
sí registró cuentas. No repetirla ni inferir de ella una falla definitiva.

## Causa y corrección aplicada

La primera corrida demostró que el shooter giraba físicamente en el sentido correcto,
pero el encoder publicaba RPM/ticks negativos. FTCLib `2.1.1` aplica
`Encoder.setDirection()` a posición, pero `getCorrectedVelocity()` conserva el signo
crudo; el comentario anterior suponía lo contrario.

Corrección mínima:

- `RobotMap.SHOOTER_MOTOR_IS_INVERTED` permanece `true`: no cambió el sentido físico.
- `SHOOTER_ENCODER_IS_INVERTED=true`.
- `ShooterFeedbackSign` normaliza velocidad y ticks a la convención
  “salida física del shooter = positiva”.
- `ShooterSubsystem` usa esa convención tanto en control/readiness como en reportes.
- `SystemCheckOpMode` publica `Shooter/Target`, `Shooter/Actual`, `Shooter/Power` y
  `Shooter/BatteryV` desde INIT, siempre con salida física en cero.
- Se añadió `ShooterFeedbackSignTest`; primero reprodujo el fallo y después pasó.
- No se redujo ningún clamp, watchdog, timeout, Stop/E-stop ni ruta de cero.

## Evidencia física conservada — primera corrida 1000 RPM, candidato anterior

Esta evidencia explica la corrección, pero **no se transfiere al APK nuevo**:

- Target: `1000 RPM`.
- `Last result`: `STOPPED_TIMEOUT`.
- Duración: `3027.5 ms`.
- Encoder confirmado: `0 -> -170 ticks`.
- Peak reportado con valor absoluto: `1028.6 RPM`.
- End RPM indicado: `-85.7 RPM`.
- Batería antes de la sesión: `12.36 V`.
- Voltaje mínimo durante el pulso: `11.62 V`.
- Max ready hold: `0.0 ms`.
- Potencia final: `0`.
- Health: `HEALTHY`.
- Fault: `none`.
- Paró completamente: sí.
- Dirección física del shooter: correcta.
- Anomalías mecánicas: ninguna.
- Dashboard: target positivo y actual mayormente negativo; por eso el operador no
  mantuvo/aceptó el gate.
- Captura persistente:
  [`t8.1-1000rpm-pre-fix-dashboard.png`](../../outputs/plan-final-antespremier/evidence/t8.1-1000rpm-pre-fix-dashboard.png),
  SHA-256
  `C5EC5D65AA0477CCFC5BDF6BE3BA6C0E4F77F9C142AF5118D8F5C79E61A3F16F`.

## Próxima prueba física única — T8.1 a 1000 RPM

1. Instalar únicamente el APK `2E28...52C2` desde `C:\dev`.
2. Área despejada, sin piezas, intake/feeder en cero, torreta sin armar y
   Stop/E-stop disponible. Palabra de aborto: `STOP`.
3. Conectar sólo Gamepad 1; mantener Gamepad 2 desconectado.
4. Seleccionar `SYSTEM CHECK and TUNING` y pulsar INIT.
5. En Dashboard seleccionar `Shooter/Target`, `Shooter/Actual`, `Shooter/Power` y
   `Shooter/BatteryV`; confirmar durante INIT `Actual=0` y `Power=0`.
6. Pulsar Start y sostener `START+A` en Gamepad 1 para el único pulso automático de
   aproximadamente `3000 ms`.
7. Pulsar `B` o Stop inmediatamente si Actual vuelve a ser negativo, supera
   `1100 RPM`, persiste potencia al soltar o aparece ruido, vibración, olor, fault,
   excepción o cualquier movimiento no solicitado.
8. Esperar el paro completo y reportar: `Last result`, duración, ticks inicial/final,
   peak RPM, end RPM, voltaje mínimo, max ready hold, potencia final, Health, Fault,
   paro completo, anomalías y captura de gráfica.
9. Gate de este setpoint: `900..1100 RPM` durante al menos `250 ms`, sin overspeed,
   fault, reinicio ni salida residual. No asumir PASS.

## Paquete de evidencia

- Template:
  [`02-sesion-larga_v1.xlsx`](../../outputs/plan-final-antespremier/02-sesion-larga_v1.xlsx),
  SHA-256
  `5D7AFE7553061799D5F6FE7A4186AE441BD053213AB756DC2EBACE124FCB32FA`.
- `02-sesion-larga_v1_FILLED.xlsx` es sólo un borrador migrado, no una entrega ni
  evidencia final.
- Por instrucción del operador, durante las pruebas se conservan los datos en el
  handoff/chat y el XLSX se llena únicamente al terminar toda la serie.
- La secuencia futura, sólo después de cada gate aceptado, es
  `1000 -> 1500 -> 2000 -> 2450 -> 2900 RPM`.

---

# Historial de handoff — MP-01/MP-02 y apertura de MP-03

Rama: `masterplan`

Base histórica: `a887fe4`

Implementación de código al cierre: `masterplan@38d1372`

Última actualización: 2026-07-22

Estado: **MP-01 `ACCEPTED`; MP-02 `ACCEPTED`; MP-03 queda como siguiente fase activa**

## Preparación de cierre rápido MP-01 — 2026-07-21

- La reinspección del runtime activo confirma que `MainTeleOp` usa `SkywalkerProfile`: el kicker de disparo sólo recibe `kick()` mientras `ShooterSubsystem.isReady()` sea verdadero y se ordena cero al soltar. Los caminos directos restantes pertenecen a commissioning deliberado o a modos deshabilitados; el interlock completo con pose, torreta y velocidad de chasis sigue siendo trabajo de MP-06.
- `ShootSequenceCommand` y `ShootBurstCommand` ya no pueden esperar indefinidamente: usan `SHOOTER_READY_TIMEOUT_MS`, número acotado de intentos, estado de fallo y `kicker.stop()` en fallo/interrupción. FND-006 se cierra por flujo estático y regresión de build.
- FND-001 se cierra porque las webcams fueron retiradas del runtime activo y el lead ya aportó 20/20 ciclos init/stop sin lookup ni excepción. La Limelight es hardware de MP-03 y no se usa para cerrar el gate de actuadores de MP-01.
- FND-002 y FND-016 se cierran con la evidencia ya registrada de 20/20 holds incompletos, 20/20 completos, init-loop y E-stop antes/después de START.
- FND-013 y FND-014 se cierran: ambos OpModes están `@Disabled`; `IntakeTeleOp` no posee shooter/kicker y `TeleopTorreta` no construye ni programa un shooter nulo.
- FND-021 deja de bloquear MP-01: mappings, puertos, drive, IMU, mecanismos, torreta y shooter necesarios para este gate están confirmados. Las métricas de repetibilidad de pods pertenecen a MP-02 y el mapping/extrínseca/pipeline de Limelight a MP-03.
- El gate acumulado de Road Runner no sustituye MP-02: MP-02 sigue requiriendo migración de ownership a Pedro y cinco repeticiones por sentido de forward, strafe, giro y ruta mixta con ≤2 in/≤2° y sin crecimiento sistemático.

**Spot-check final sobre un único APK:** sin piezas, ejecutar `MainTeleOp`; confirmar INIT sin movimiento, START sin armar torreta, kicker rechazado con shooter apagado, release en cero y `gamepad1 BACK` deteniendo el OpMode.

**Candidato final aceptado:** `TeamCode-debug.apk`, 81,322,181 bytes, SHA-256 `7A9060615BFD6FB0C31E5E5C2D9B68E2E41EE228B99CAD385ABBF00B349CF5AC`. Verificación: 36/36 pruebas JVM, `assembleDebug` y `git diff --check` PASS.

**Aceptación física 2026-07-21:** el lead reportó `5/5 PASS` en el mismo spot-check: INIT quieto, torreta inmóvil sin armar, kicker bloqueado con shooter apagado, release en cero y `gamepad1 BACK` E-stop. Con esta evidencia MP-01 cambia de `READY_FOR_GATE` a `ACCEPTED`. No se trasladan estos resultados a futuros APK que cambien rutas de seguridad o ownership de hardware.

## Inicio MP-02 — 2026-07-21

- Se añadió `MP02 Pedro Localization Push Test`, un OpMode de sólo lectura/localización. Construye un único `Follower` Pedro, nunca llama `startTeleopDrive()` ni `followPath()`, y ejecuta `breakFollowing()` antes y después de cada `update()`.
- `gamepad1 A` reinicia la pose entre mediciones manuales; `gamepad1 BACK` ordena cero, ejecuta el apagado global y detiene el OpMode. `stop()` también ordena cero de forma idempotente.
- Road Runner conserva el ownership de producción. Este test aislado no instancia RR y no autoriza todavía movimiento motorizado Pedro.
- Fuentes verificadas: documentación oficial actual de Pedro y fuentes locales exactas `core/ftc 2.1.2`; se confirmó `setPose`, `getPose`, `update` y `breakFollowing`.
- APK de primer gate MP-02: 81,322,875 bytes, SHA-256 `C51CC7CCF0DC12280013E9E7776186280A9B076026A0BD7D4D192F671CEB93BF`; 36/36 pruebas JVM, `assembleDebug` y `git diff --check` PASS.
- Próximo gate físico, sin usar joysticks: elevar primero para verificar cero, luego colocar en piso y mover a mano 60 cm forward, 60 cm left y un giro CCW de 90°, reseteando con A antes de cada segmento. Registrar X/Y/heading inicial y final. Si cualquier rueda intenta moverse, pulsar BACK y detener la prueba.
- Resultado físico inicial: quieto elevado 10 s `PASS`; forward 60 cm `X=23.352 in, Y=-0.540 in, heading=-1.08°`; left 60 cm `X=-0.086 in, Y=23.747 in, heading=0.85°`; CCW 90° `X=-12.726 in, Y=2.161 in, heading=92.29°`; BACK E-stop `PASS`.
- Interpretación: las escalas recta y lateral están dentro de aproximadamente 1.2%, y el signo del heading es correcto. El giro revela traslación ficticia severa, sobre todo en X, por lo que no se aceptan aún offsets ni escala angular. El OpMode ahora reporta deltas raw y corregidos de `par0`, `par1` y `perp`; se requiere un giro CCW 90° y uno CW 90° para resolver la geometría efectiva antes de habilitar movimiento Pedro.
- APK instrumentado para esos dos giros: 81,323,339 bytes, SHA-256 `4032926688067CCFB117EE37A669478A2EFC1811D81DC8D13D2077597E7E7B5E`; 36/36 pruebas JVM, `assembleDebug` y `git diff --check` PASS.
- Primer par de giros instrumentados, descartado para cálculo de offsets por traslación del centro: CCW `X=0.091 in, Y=6.432 in, heading=90.78°`, deltas Pedro `par0=7668, par1=-3082, perp=-3544`; CW `X=-11.325 in, Y=-1.751 in, heading=-90.19°`, deltas `par0=-9560, par1=1122, perp=691`; BACK `PASS`. El centro se desplazó ≈`6.43 in` y ≈`11.46 in`, respectivamente, por lo que la asimetría de paralelos incluye movimiento lineal real y no permite resolver la geometría de giro. No se cambian constantes con esta corrida.
- Para no depender de un giro humano perfecto, se reutiliza la regresión automática válida de `AngularRampLogger` sobre los mismos pods: RR `par0YTicks=+3174.0795443819798`, `par1YTicks=-3619.1632183254724`, `perpXTicks=-2330.341135601611`. Traducida a Pedro queda `leftPodY=+6.942896027755779 in`, `rightPodY=-6.089060628403165 in`, `strafePodX=-4.636066386085883 in` y `turnTicksToInches=forwardTicksToInches=0.00191837052073273`. `LOCALIZER_OFFSETS_CALIBRATED` permanece `false` hasta la validación Pedro; no se autoriza movimiento sólo por importar estas constantes.
- APK de validación con la traducción RR→Pedro: 81,323,371 bytes, SHA-256 `8421EFEEF7387A956B666FD0F3B16B3418F5B69F4C20999294AB247E8AEBC2D5`; 36/36 pruebas JVM, `assembleDebug` y `git diff --check` PASS.
- Validación pasiva de la traducción automática: CCW 90° `X=-0.396 in, Y=0.933 in, heading=90.72°`; CW 90° `X=-4.225 in, Y=-0.355 in, heading=-89.39°`. Ambos headings quedan dentro de `1°`; CCW cierra por debajo de `1 in`. La traslación CW se atribuye a desplazamiento manual no controlado y no reemplaza la regresión automática. Se cambia `LOCALIZER_OFFSETS_CALIBRATED=true` para avanzar al gate de drive, sin declarar MP-02 cerrado.
- Antes de energizar Pedro se corrigieron dos direcciones que no coincidían con el `RobotMap` físicamente validado: `leftFront FORWARD→REVERSE` y `rightBack FORWARD→REVERSE`; `leftBack=REVERSE` y `rightFront=FORWARD` permanecen. Los nombres ahora provienen de `RobotMap`.
- Se añadió `MP02 Pedro Drive Direction Test`: potencia fija `0.10`, hold-to-run, sólo acepta un botón, release/múltiples entradas mandan cero y BACK ejecuta E-stop. Controles: A forward, B backward, X strafe left, Y strafe right, D-pad left CCW y D-pad right CW. Sólo construye el `Follower` Pedro; no mapea mecanismos ni Road Runner.
- APK del gate de direcciones no instalado: 81,324,579 bytes, SHA-256 `762F1AE05F4E325A33FE0057602D97EE0661C352D0B239D276E92125563413D9`; 36/36 pruebas JVM, `assembleDebug` y `git diff --check` PASS.
- Por solicitud del lead, `MP02 Pedro Drive Direction Test` queda en potencia fija `0.50` (el working tree ya contenía `0.30`, posterior al APK anterior). Se conservan hold-to-run, rechazo de entradas múltiples, release cero, Stop y BACK E-stop; el primer gate continúa obligatorio con el robot elevado y pulsos breves.
- APK `0.50` no instalado: 81,324,571 bytes, SHA-256 `E78768BD3FE20885B9211E907E12396B9F7473F582639FA054C4B320BCF5218E`; 36/36 pruebas JVM, `assembleDebug` y `git diff --check` PASS. Sustituye el candidato `762F...13D9`.
- Prueba física del candidato `E787...218E`: FAIL; ningún botón movió las ruedas. Causa raíz confirmada contra Pedro Pathing `2.1.2`: `start()` activaba `startTeleopDrive()` y enseguida `zeroDrive()` llamaba `breakFollowing()`, que pone `manualDrive=false`; después `update()` no entraba a la ruta manual ni enviaba potencia. Se separaron `commandZero()` (mantiene teleop activo y manda vector cero) y `shutdownDrive()` (cero + `breakFollowing()` sólo para INIT/Stop/BACK). Se añadió telemetría `Pedro manual activo` para validar la frontera de estado. APK corregido pendiente de build/instalación y retest elevado.
- APK corregido del gate Pedro `0.50`: 81,324,635 bytes, SHA-256 `A9388D3CE10FB4C625781DB7363BFE92B17DA917C369108038542FB5537E661E`; comprobación específica del ciclo manual PASS, 36/36 pruebas JVM, `assembleDebug` y `git diff --check` PASS. Sustituye `E787...218E`; prueba física elevada pendiente.
- Gate físico de dirección/seguridad Pedro sobre `A938...661E`: **PASS 9/9** por confirmación del lead. START quieto, A forward, B backward, X left, Y right, D-pad left CCW, D-pad right CW, release stop, rechazo de dos botones y BACK E-stop pasaron; sin observaciones adicionales. Quedan validados el modo manual Pedro aislado, las cuatro direcciones básicas y las rutas de cero/E-stop a potencia `0.50`. Esto no cierra MP-02: siguen pendientes el cambio atómico de ownership hacia el adapter/`DriveSubsystem` Pedro y el gate estadístico T5 de repetibilidad.
- El lead reporta que `0.10` no vence la carga del robot en el suelo. El gate de dirección sube a potencia fija `0.30`, conservando hold-to-run, cero al soltar, rechazo de múltiples entradas y BACK E-stop. El APK `762F...13D9` queda superseded y no debe usarse para la prueba en piso.

- Se agregó `PedroDriveAdapter`, frontera única que implementa `DriveAdapter` y `PoseProvider` sobre una sola instancia de `Follower`. Conserva el contrato neutral (+forward, +left en pose, +right en el comando de strafe y heading CCW), rota la velocidad de campo de Pedro al frame del robot, rechaza entradas no finitas, limita comandos y centraliza `breakFollowing()`/cero. Pruebas JVM cubren signos, saturación, fail-closed y transformación de velocidad.
- Se agregó `MP02 Pedro T5 Validation`, aislado de `RobotContainer`, mecanismos y Road Runner. Pedro es el único owner del drivetrain. Potencia máxima local `0.50`; selección en INIT de forward/back/left/right a 24 o 48 in, 360° CW/CCW en cuatro cuartos, cuadrado y ruta mixta. START queda quieto, A ejecuta una sola vez, B cancela, timeout de 20 s y BACK E-stop. La telemetría reporta pose/objetivo/error estimado y recuerda que el error de aceptación debe medirse con marcas independientes.
- Este candidato **no cambia el ownership de producción**: `MainTeleOp -> RobotContainer -> DriveSubsystem -> MecanumDrive` sigue intacto como rollback operativo. FND-017 permanece `OPEN` hasta aprobar T5 y realizar el cambio atómico; no se permite ejecutar RR y Pedro simultáneamente sobre el hardware.
- APK del gate T5 pendiente de instalación: 81,330,943 bytes, SHA-256 `6DA561490258484351D4880A08BD19AD54EF8CE99566784572DFF1D0DFC14F21`; 39/39 pruebas JVM, `assembleDebug`, revisión de owner único y `git diff --check` PASS.
- Gate físico T5 del candidato `6DA5...4F21`: **PASS 40/40** por evidencia del lead. Se completaron cinco intentos por grupo: forward (24/48 in), backward (24/48 in), left (24/48 in), right (24/48 in), 360° CCW, 360° CW, cuadrado y ruta mixta. Todos quedaron dentro de `2 in / 2°` y en todos se reportó `sin deriva progresiva`.
- Estadística T5, valores `(media X, media Y, media heading; máximo radial, máximo |heading|)`: forward `(-0.398, +0.043, -0.260°; 0.574 in, 0.620°)`; backward `(+0.570, -0.062, -0.806°; 0.787 in, 1.170°)`; left `(+0.003, -0.185, +0.228°; 0.227 in, 0.660°)`; right `(+0.044, +0.217, -0.266°; 0.342 in, 0.510°)`; CCW `(+0.052, +0.101, -0.124°; 0.201 in, 0.390°)`; CW `(+0.102, +0.081, -0.304°; 0.207 in, 0.680°)`; cuadrado `(+0.016, +0.453, +0.110°; 0.642 in, 0.340°)`; mixta `(+0.207, +0.251, -1.324°; 0.372 in, 1.650°)`. Máximo global: `0.787 in` radial y `1.650°` heading.
- Interpretación: la repetibilidad/calibración Pedro supera T5 con margen. La ruta mixta presenta un sesgo de heading repetible negativo, pero su peor caso `1.65°` permanece dentro del gate y no crece por intento. No se retocan constantes con estos datos. El T5 aislado queda cerrado; MP-02 aún requiere el cambio atómico del owner de producción a Pedro y el retest de lifecycle/direcciones antes de cerrar FND-017 y MP-02.
- Cambio atómico candidato posterior a T5: `MainTeleOp -> RobotContainer -> PedroDriveSubsystem -> PedroDriveAdapter -> Follower`. `RobotContainer` ya no construye `DriveSubsystem/MecanumDrive`; un solo `Follower` posee motores, pose, paths y stop, y `periodic()` ejecuta un único `update()` por ciclo. El field-centric, reset con START, release-stop, interrupción y BACK E-stop conservan su contrato neutral.
- Potencia inicial de producción: `0.50`, idéntica a la validada en el T5 de 40 corridas. No se incorporó compensación ni ajuste de constantes a partir de los errores físicos. `PRUEBA: FIELD CENTRIC CHASIS` también usa Pedro; todos los tuners dinámicos de Road Runner quedan sin registrar.
- El lead indicó que los autónomos anteriores ya no se necesitan. La inspección confirma que todos estaban `@Disabled` o completamente comentados, por lo que no aparecen en Driver Station; sus fuentes se conservan únicamente como historial/rollback y los nuevos autónomos se implementarán cuando el lead entregue las rutas.
- Verificación estática del candidato de producción: 39/39 pruebas JVM (`0` failures, `0` errors), `assembleDebug` y `git diff --check` PASS. APK no instalado: 81,332,570 bytes, SHA-256 `98F5C871550BA2938633E779D81F1521D11FE105BD564E12CF813B6B7D8BFC75`. Falta instalar este APK y ejecutar el spot-check corto de producción antes de cerrar FND-017/MP-02: INIT quieto, direcciones simples, combinación forward+rotate, field-centric tras ±90°, release y BACK.

Este archivo se actualiza con cada modificación material del plan maestro. Debe indicar qué cambió, cuál es la próxima acción de Codex y qué evidencia debe aportar el equipo desde el robot real.

**Reparto de trabajo para la ventana previa a competencia:** ver [`plan-paralelo-20h.md`](plan-paralelo-20h.md), que divide el trabajo restante en pista Software y pista Tuning con su tabla de bloqueos cruzados. Este archivo (`handoff-task.md`) sigue siendo la bitácora de evidencia física — cada resultado de la pista Tuning se registra aquí igual que hasta ahora.

## Modificaciones realizadas

- Se rebaselinó la documentación al merge endurecido `a887fe4` sin borrar la evidencia histórica de `b5a1342`/`f91af18`.
- `SafeCommandOpMode` incorpora `duringInitLoop()` y atiende `gamepad1 BACK` durante init y ejecución mediante cancelación del scheduler, `RobotSafety.stopAll()` y solicitud de Stop.
- El armado de torreta usa estados `WAITING`, `HOLDING` y `ARMED`; exige `START+BACK` de gamepad 2 durante 1,000 ms y resetea el encoder una sola vez.
- Init muestra estado, progreso y la advertencia de que el centro es una confirmación física manual.
- `MainTeleOp` y `TeleopTorreta` asignan el `VisionPortal` antes de registrar el cleanup de esa instancia.
- `TeleopTorreta` ya no programa un comando de shooter con referencia nula.
- `RobotMap` usa los nombres confirmados: `kickerMotor`, `kickerServo`, `Shooter`, `intakeMotor`, `torretaMotor` y los cuatro motores de drive. Left front, left back y right back están invertidos; sólo right front queda normal.
- La odometría lee `par0` desde `rightFront`, `par1` desde `leftFront` y `perp` desde `rightBack`, según la conexión confirmada por el equipo.
- `KickerSubsystem` conserva compatibilidad defensiva con el mapping histórico, pero la configuración final usa sólo `kickerMotor`; el CRServo está retirado y compilado en cero.
- Hood, `Shooter2` y ambas webcams se retiraron del hardware activo. El shim de hood no mapea dispositivos y los OpModes activos no construyen `VisionPortal`.
- El shooter tiene máximo seguro declarado de 6000 RPM y health fail-closed latched para voltaje, encoder, overspeed y targets inválidos. Un fault exige reiniciar el OpMode.
- Los presets de shooter del TeleOp principal quedan inhibidos hasta pasar fault injection; sólo System Check/Shooter Tuning ofrecen prueba hold-to-run.
- `IntakeTeleOp` perdió sus bindings directos de shooter/kicker y quedó `@Disabled`.
- Los tuners/OpModes legacy sin lifecycle uniforme quedaron deshabilitados hasta una fase que les agregue E-stop y stop verificables.
- `ShootSequenceCommand` ya no espera readiness indefinidamente y los adapters de intake/kicker delegan en los mismos métodos seguros del subsistema.
- Por decisión del equipo, `KICKER_OUT_SPEED` cambia de `0.70` a `0.85`; queda pendiente su validación física controlada.
- El operador confirmó en Driver Station los puertos y tipos reales. La versión instalada no ofrece export XML; se registró la atestación, versiones y la ausencia del export sin fabricar un archivo.
- No se cambiaron mappings, límites ±200, Pedro, Road Runner, dependencias ni controles de aim.

## Estado de hallazgos relacionados

| Finding | Estado tras este cambio | Gate |
|---|---|---|
| FND-001 | `CLOSED` | Webcams fuera del runtime y 20/20 ciclos sin lookup/excepción. |
| FND-002 | `CLOSED` | 20/20 holds cortos y 20/20 completos aprobados. |
| FND-014 | `CLOSED` | Modo deshabilitado y camino nulo retirado. |
| FND-016 | `CLOSED` | Init-loop y E-stop pre/post START aprobados. |
| FND-003 | `CLOSED` | Lead confirma retest físico de límites `-983/+1070` y potencia escalonada `0.50→0.05`. |
| FND-020 | `CLOSED` | Lead confirma todos los gates de cero inválido/reset aprobados. |
| FND-021 | `CONTAINED` | MP-01 cubierto; pods completos pasan a MP-02 y Limelight a MP-03. |
| FND-005 | `CONTAINED` | Interlock de producción cubierto; matriz completa pasa a MP-06. |
| FND-006 | `CLOSED` | Timeout/failure/cleanup acotados en ambas secuencias. |
| FND-013 | `CLOSED` | Modo oculto y sin binding directo de shooter/kicker. |
| FND-015 | `CLOSED` | Fault injection 4/4 y regresión fail-closed aprobadas. |
| FND-007 | `CONTAINED` | Legacy/tuners inseguros deshabilitados; reabrir al habilitarlos. |
| FND-026 | `CLOSED` | Configuración final motor-only confirmada; CRServo retirado. |
| FND-027 | `CLOSED` | Matriz 10/10 por sentido aceptada bajo APK `9C2F`; reabrir si cambia candidato o configuración. |
| FND-028 | `CLOSED` | Test lead aceptó dirección, encoder, watchdog y autocorte bajo APK `09B2...DAC1`; T8 de estabilidad/carga permanece separado y feeder bloqueado. |

MP-01 no se cierra mientras los findings `FIX_READY` no pasen su regresión y FND-021 permanezca bloqueado por evidencia física. FND-003/FND-020/FND-027 ya no forman parte del bloqueo.

## Evidencia de software

- `git diff --check`: PASS.
- `:TeamCode:compileDebugJavaWithJavac`: PASS después del cambio de MP-01.
- `assembleDebug` limpio pre-commit: PASS en 53 s después de limpiar únicamente `TeamCode/build`, afectado por un placeholder de OneDrive.
- APK limpio no instalado: 81,285,031 bytes, SHA-256 `B8E72645AA8FA36678F9EDEC26216259679208F4BDCDCFE6DB3ABA2857E5E1B6`.
- APK T4 instalado y físicamente probado: SHA-256 `D5AC6F1E11B42F5EC8E973AEB3132B359733A09CB6791809EA442D59E7557536`; no trasladar resultados entre hashes.
- Código consolidado en `masterplan@38d1372`; la documentación de cierre se commitea por separado.
- Dependencias, límites ±200 de torreta y stack de localización no se actualizaron; sí cambiaron mappings confirmados, hardware activo y contención de OpModes.
- `KICKER_OUT_SPEED = 0.85`: validado en la configuración motor-only mediante pruebas forward/reverse, release, Stop y E-stop registradas abajo.

## Evidencia física en curso — 2026-07-17

- Artefacto instalado: APK SHA-256 `624EA17B7E79AA594D8CC5720098375C9BD375F10BD8EA82EB02CCF5E1AA15E4`.
- Mapping: confirmado directamente por el operador en Driver Station; puertos, tipos y versiones registrados en `contrato-hardware.md`.
- `MainTeleOp`, 20/20 ciclos INIT sin chord: `PASS`; sin error de mapping, movimiento, ruido ni vibración.
- Arranque sin armar torreta, 10/10 ciclos: `PASS`; ningún actuador se movió, `Torreta Armada=false` y encoder estable exactamente en 15 ticks.
- Cámara/webcams desconectadas, 20/20 ciclos: `PASS`; sin lookup, excepción ni recurso requerido.
- Precondición mecánica de torreta confirmada por el operador: centrada, marca visual colocada y alineada, holgura de cables hacia ambos lados y recorrido despejado.
- Armado de torreta: 20/20 holds incompletos regresaron de `HOLDING` a `WAITING`; 20/20 holds completos llegaron a `ARMED`. No hubo movimiento, ruido ni error y las marcas permanecieron alineadas. Primera lectura después de START: -3 ticks; las 19 repeticiones siguientes marcaron 0 ticks.
- E-stop primario `gamepad1 BACK`: 10/10 durante INIT y 10/10 después de START detuvieron el OpMode; ningún actuador saltó o se movió, no apareció error y la detención fue aparentemente inmediata. La medición instrumentada de comando cero ≤50 ms bajo carga sigue pendiente.
- Primera prueba elevada de drive: al pedir avance, las cuatro ruedas giraron coherentemente hacia atrás y se detuvieron al soltar, sin ruido ni error. La inspección encontró una doble inversión longitudinal: FTCLib `GamepadEx.getLeftY()` ya entrega forward positivo y `FieldCentricDriveCommand` lo negaba otra vez. Se corrigió sólo ese signo; no se cambiaron las direcciones confirmadas de los motores. Retest pendiente con APK reconstruido.
- APK de retest después de corregir el signo longitudinal: `assembleDebug` limpio `PASS`; 81,276,815 bytes; SHA-256 `1E293D68F315B392E3F47872201FBDD5467484FD1BB41B2413B044F37139EC7D`.
- Instalación del APK de retest en el Control Hub `7eb6a352c3d5557b`: ADB streamed install `Success`. Se usó instalación directa porque Android Studio/OneDrive bloqueó el snapshot de outputs generados; no fue un error de compilación ni de fuente.
- Retest elevado con APK corregido: Driver Station reconectó, INIT permaneció sin movimiento y E-stop funcionó. Al pedir avance reducido, `leftFront`, `rightFront`, `leftBack` y `rightBack` giraron hacia adelante y las cuatro se detuvieron al soltar: `PASS`.
- Drive elevado, reversa reducida: las cuatro ruedas giraron hacia atrás, se detuvieron al soltar y no hubo ruido, vibración ni error: `PASS`. Android Studio volvió a ejecutar Run sin el bloqueo de snapshot mientras OneDrive permaneció pausado.
- Primera prueba de strafe derecho: produjo el patrón opuesto (`leftFront/rightBack` atrás y `rightFront/leftBack` adelante), confirmando strafe izquierdo. La telemetría mostraba además derecha como `-1.0` porque negaba el eje visualmente. Se corrigió el contrato a strafe-right positivo en `FieldCentricDriveCommand` y la telemetría; direcciones de motores sin cambios. Retest pendiente.
- Android Studio compiló e instaló correctamente el APK con la corrección lateral; Driver Station reconectó. OneDrive permaneció pausado durante el build. Retest físico de strafe pendiente.
- Retest elevado de strafe derecho: telemetría X positiva; `leftFront/rightBack` adelante y `rightFront/leftBack` atrás; todas se detuvieron al soltar, E-stop funcionó y no hubo ruido, vibración ni error: `PASS`.
- Strafe izquierdo elevado: telemetría X negativa y patrón exactamente opuesto al strafe derecho; todas las ruedas se detuvieron al soltar, sin ruido, vibración ni error: `PASS`.
- Giro elevado: stick derecho produjo giro físico correcto a derecha e izquierda; todas las ruedas se detuvieron al soltar, sin ruido, vibración ni error: `PASS`. Con esto, avance, reversa, ambos strafes y ambos giros del drive quedan validados a potencia reducida.
- Kicker dual, smoke test sin piezas: cero movimiento en INIT; `gamepad1 X` movió `kickerMotor` y `kickerServo` juntos hacia adelante y ambos pararon al soltar; `DPAD_UP` movió ambos juntos en reversa y ambos pararon al soltar. Sin ruido, atasco ni error: `PASS`.
- Kicker dual, respuesta física: el operador observó aproximadamente 0.5 s de demora del goBILDA Torque servo al arrancar y 0.5–1.0 s al detenerse, tanto en avance como en reversa; finalmente se detiene y no zumba. Las órdenes de software se emiten juntas, pero el gate físico queda pausado hasta confirmar SKU, modo continuo interno y si trabaja bajo carga. No ejecutar las 20 repeticiones todavía.
- Identificación del kicker CRServo: goBILDA `2000-0025-0002`, programado físicamente con el programador en modo continuo (`C`) y conectado a una llanta. El mapping como `CRServo` queda confirmado. La demora física se conserva como dato de dinámica bajo carga; aún falta que el equipo decida si el requisito es simultaneidad de comando o sincronía mecánica observable.
- El equipo exige sincronía mecánica observable, no sólo simultaneidad de comando. La configuración motor BRAKE + CRServo con llanta no cumple; FND-026 bloquea el kicker y los presets. No se retrasará el apagado del motor ni se aplicará frenado inverso para ocultar la demora, porque release/E-stop deben ordenar cero inmediatamente.
- Por solicitud de mecánica, el CRServo pasa a hardware opcional: `LowAltitudeConstants.KICKER_SERVO_ENABLED=false` por defecto. `KickerSubsystem` usa `hardwareMap.tryGet`, nunca desreferencia un servo ausente, lo deja en cero si existe y expone telemetría solicitado/disponible/activo. Activarlo requiere cambiar la bandera con el OpMode detenido y reiniciar INIT. El modo motor-only evita crashes, pero FND-026 permanece bloqueado.
- Candidato motor-only generado e instalado desde Android Studio el 2026-07-18: APK incremental de 81,277,349 bytes, SHA-256 `B21E9BEE3C0AD400C7EAF66F1AF648723E46334C23A925EDBDF598C283D08A81`, sobre `masterplan@24f9911` con worktree dirty. El `kickerServo` quedó físicamente desconectado pero conservado en la configuración del Driver Station, por lo que la telemetría posterior a START reportó solicitado=`false`, disponible=`true`, activo=`false`.
- Regresión motor-only de INIT con ese APK: 20/20 ciclos en `MainTeleOp` sin movimiento, ruido, vibración ni errores. La telemetría detallada del kicker se publica después de START; durante INIT se verificó ausencia de movimiento y fallo de mapping.
- Regresión motor-only de START sin armar con ese APK: 10/10 ciclos en `MainTeleOp` mantuvieron `Torreta Armada=false`, ticks estables, cero movimiento y cero errores. No se accionaron sticks, botones de mecanismos ni el chord de armado.
- Regresión funcional de E-stop durante INIT con ese APK: `gamepad1 BACK` detuvo `MainTeleOp` en 10/10 ciclos, sin movimiento, ruido ni errores. Esta observación valida accesibilidad/función, pero no sustituye la medición instrumentada de comando cero en el primer ciclo y ≤50 ms.
- Regresión funcional de E-stop después de START con ese APK: `gamepad1 BACK` detuvo `MainTeleOp` en 10/10 ciclos con la torreta desarmada y sin accionar mecanismos; no hubo movimiento, ruido ni errores. La medición instrumentada del gate temporal sigue pendiente.
- Regresión funcional de `SystemCheck`: `gamepad1 BACK` durante INIT detuvo el OpMode en 10/10 ciclos, sin movimiento, ruido ni errores. No se presionaron los controles de shooter, intake o kicker; falta la variante posterior a START y el gate temporal instrumentado.
- Regresión funcional de `SystemCheck` después de START: `gamepad1 BACK` detuvo el OpMode en 10/10 ciclos, sin movimiento, ruido ni errores. No se accionaron controles de mecanismos; queda pendiente únicamente la medición instrumentada del gate temporal para este modo.
- Regresión funcional de `ShooterTuning`: `gamepad1 BACK` durante INIT detuvo el OpMode en 10/10 ciclos, sin movimiento, ruido ni errores. No se energizó el shooter; falta la variante posterior a START y el gate temporal instrumentado.
- Regresión funcional de `ShooterTuning` después de START: `gamepad1 BACK` detuvo el OpMode en 10/10 ciclos, sin movimiento, ruido ni errores. Con esto, `MainTeleOp`, `SystemCheck` y `ShooterTuning` tienen prueba funcional repetida de E-stop durante INIT y después de START sobre el APK motor-only; la medición instrumentada de primer ciclo/≤50 ms continúa pendiente.
- Primer pulso controlado del kicker motor-only en `SystemCheck`, sin piezas: `gamepad1 RB` durante aproximadamente 0.5 s movió únicamente `kickerMotor` hacia delante y el motor se detuvo al soltar. Dirección correcta, sin ruido, vibración ni errores. Falta completar la serie repetida y las variantes Stop/E-stop.
- Serie de kicker motor-only hacia delante en `SystemCheck`, sin piezas: 20/20 pulsos de aproximadamente 0.5 s tuvieron dirección correcta y se detuvieron en cada release, sin pulsos adicionales, ruido, vibración, calentamiento ni errores. Faltan reversa y variantes Stop/E-stop.
- Primer pulso de reversa del kicker motor-only en `MainTeleOp`, sin piezas: `gamepad2 DPAD_LEFT` durante aproximadamente 0.5 s movió únicamente el kicker en la dirección correcta y se detuvo al soltar, sin ruido, vibración ni errores. Falta completar la serie repetida y las variantes Stop/E-stop.
- Serie de reversa del kicker motor-only en `MainTeleOp`, sin piezas: 20/20 pulsos de aproximadamente 0.5 s tuvieron dirección correcta y se detuvieron en cada release, sin pulsos adicionales, ruido, vibración, calentamiento ni errores. El release normal queda validado en ambos sentidos; faltan Stop/E-stop activos y medición temporal instrumentada.
- Primera prueba de Stop de OpMode con kicker motor-only activo en `SystemCheck`: al mantener `gamepad1 RB` y presionar Stop en Driver Station, el motor se detuvo, no volvió a arrancar aunque la solicitud siguiera sostenida y no hubo ruido, vibración ni errores. Falta completar la serie repetida y la variante E-stop.
- Serie de Stop de OpMode con kicker motor-only activo en `SystemCheck`: 10/10 pruebas detuvieron el motor y evitaron toda reactivación mientras la solicitud permanecía sostenida, sin ruido, vibración, calentamiento ni errores. Falta la variante E-stop y la medición temporal instrumentada.
- Primera prueba de E-stop con kicker motor-only activo en `SystemCheck`: al mantener `gamepad1 RB` y presionar `gamepad1 BACK`, se detuvieron el OpMode y el motor, sin reactivación, ruido, vibración ni errores. Falta completar la serie repetida y medir el gate temporal de forma instrumentada.
- Serie de E-stop con kicker motor-only activo en `SystemCheck`: 10/10 pruebas detuvieron el OpMode y el motor, sin reactivaciones, ruido, vibración, calentamiento ni errores. El kicker motor-only queda funcionalmente validado en INIT, avance, reversa, release, Stop y E-stop sin piezas; siguen pendientes la medición instrumentada de primer ciclo/≤50 ms, la validación bajo carga autorizada y la decisión formal que contenga FND-026.
- Primera prueba controlada del intake en `SystemCheck`, sin piezas: `gamepad1 LB` movió únicamente el intake y se detuvo al soltar, sin ruido, vibración ni errores, pero el sentido físico fue opuesto a introducir una pieza. La inspección confirmó un solo owner y un solo punto de inversión; se cambió únicamente `RobotMap.INTAKE_MOTOR_IS_INVERTED` de `true` a `false`. Potencias y bindings no cambiaron. El APK `B21E...A81` queda invalidado para continuar las pruebas de intake; falta build, instalación y retest con artefacto nuevo.
- Candidato de retest del intake generado e instalado desde Android Studio el 2026-07-18: APK incremental de 81,315,661 bytes, SHA-256 `A133A8ECB64E995418990B2C3DE5140B82AA9AC57B5918D40AA12C9385DF1863`, sobre `masterplan@24f9911` con worktree dirty. Incluye únicamente la corrección de inversión del intake respecto del candidato anterior; falta retest físico.
- Primer retest físico del intake con el APK `A133...1863`, sin piezas: `gamepad1 LB` movió únicamente el intake en la dirección correcta para introducir una pieza y el motor se detuvo al soltar, sin ruido, vibración ni errores. La hipótesis de inversión quedó confirmada; falta completar repeticiones, reversa y variantes Stop/E-stop.
- Serie del intake hacia dentro con el APK `A133...1863`, sin piezas: 20/20 pulsos de aproximadamente 0.5 s tuvieron dirección correcta y se detuvieron en cada release, sin pulsos adicionales, ruido, vibración, calentamiento ni errores. Falta reversa y variantes Stop/E-stop.
- Primer pulso de reversa del intake con el APK `A133...1863`, sin piezas: `gamepad1 Y` movió únicamente el intake en sentido contrario y el motor se detuvo al soltar, sin ruido, vibración ni errores. Falta completar la serie repetida y las variantes Stop/E-stop.
- Serie de reversa del intake con el APK `A133...1863`, sin piezas: 20/20 pulsos de aproximadamente 0.5 s tuvieron dirección correcta y se detuvieron en cada release, sin pulsos adicionales, ruido, vibración, calentamiento ni errores. El release normal queda validado en ambos sentidos; faltan Stop/E-stop activos y medición temporal instrumentada.
- Primera prueba de Stop de OpMode con intake activo en `SystemCheck`: al mantener `gamepad1 LB` y presionar Stop en Driver Station, el motor se detuvo y no volvió a arrancar aunque la solicitud siguiera sostenida, sin ruido, vibración ni errores. Falta completar la serie repetida y la variante E-stop.
- Serie de Stop de OpMode con intake activo en `SystemCheck`: 10/10 pruebas detuvieron el motor y evitaron toda reactivación mientras la solicitud permanecía sostenida, sin ruido, vibración, calentamiento ni errores. Falta la variante E-stop y la medición temporal instrumentada.
- Primera prueba de E-stop con intake activo en `SystemCheck`: al mantener `gamepad1 LB` y presionar `gamepad1 BACK`, se detuvieron el OpMode y el intake, sin reactivación, ruido, vibración ni errores. Falta completar la serie repetida y medir el gate temporal de forma instrumentada.
- Serie de E-stop con intake activo en `SystemCheck`: 10/10 pruebas detuvieron el OpMode y el intake, sin reactivaciones, ruido, vibración, calentamiento ni errores. Con el APK `A133...1863`, el intake queda funcionalmente validado sin piezas en dirección de entrada, reversa, release, Stop y E-stop. Siguen pendientes la medición instrumentada de primer ciclo/≤50 ms y la prueba bajo carga autorizada.
- Decisión del equipo 2026-07-18: se conserva la configuración dual como opción futura si mecánica reinstala el servo, pero MP-01 continúa motor-only. DEC-037 exige corrección/aceptación mecánica, bandera activada con OpMode detenido, APK nuevo y regresión completa antes de autorizar dual. FND-026 queda `CONTAINED`, no cerrado.
- Instrumentación del gate temporal implementada después de las pruebas anteriores: `RobotSafety` conserva el último evento, tiempo desde solicitud observada hasta completar las órdenes cero, cantidad de hooks, fallos y resultado ≤50 ms. `SafeCommandOpMode` mide E-stop y Stop incluyendo cancelación más todos los hooks; preserva el reporte de E-stop durante el cleanup final. `SystemCheck` mide release de shooter, kicker e intake dentro del mismo ciclo del scheduler. `FieldCentricDriveCommand` mide transición a cero e interrupción y ahora ordena stop explícito en `end()`.
- Verificación software del candidato instrumentado: `:TeamCode:compileDebugJavaWithJavac` PASS en 22.7 s; `assembleDebug` PASS en 20.9 s; `git diff --check` PASS. APK `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`, 81,280,319 bytes, SHA-256 `BE23F0D4FC41BCFEB6EEBDBB157612F398ED158850B9AD9132BE716D285D90EA`. Aún no instalado ni probado físicamente.
- Android Studio instaló el candidato instrumentado desde su APK incremental de 81,280,324 bytes, SHA-256 `D30A0FED102C841C556F5BA54748ABFCFE8F2CCE9B79F92BD09F305641FE909A`.
- Primera medición instrumentada de release del intake en `SystemCheck`: `SYSTEM_CHECK_INTAKE_RELEASE`, 4.786 ms, mismo ciclo=`true`, acciones stop=1, fallos=0, gate 50 ms=`PASS`; el intake se detuvo físicamente al soltar. Falta completar la serie y registrar el máximo.
- Serie instrumentada de release del intake en `SystemCheck`: 20/20 `PASS`, mismo ciclo siempre `true`, fallos stop=0 y detención física en cada release. El máximo de las 19 repeticiones adicionales fue 3.125 ms; incluyendo la primera medición, el máximo total conservador es 4.786 ms.
- Primera medición instrumentada de release del kicker motor-only en `SystemCheck`: `SYSTEM_CHECK_KICKER_RELEASE`, 10.613 ms, mismo ciclo=`true`, acciones stop=1, fallos=0, gate 50 ms=`PASS`; el kicker se detuvo físicamente al soltar. El objeto CRServo sigue disponible en la configuración y recibe cero dentro del mismo `stop()`, aunque el dispositivo físico está desconectado. Falta completar la serie y registrar el máximo.
- Serie instrumentada de release del kicker motor-only en `SystemCheck`: 20/20 `PASS`, mismo ciclo siempre `true`, fallos stop=0 y detención física en cada release. El máximo de las 19 repeticiones adicionales fue 10.762 ms y supera la primera lectura, por lo que también es el máximo total.
- Primera medición instrumentada de E-stop con kicker motor-only activo en `SystemCheck`: el reporte persistente leído en el siguiente INIT mostró `E_STOP`, 16.005 ms, mismo ciclo=`true`, acciones stop=3, fallos=0 y gate 50 ms=`PASS`; el kicker se detuvo físicamente. Falta completar la serie y registrar el máximo.
- Serie instrumentada de E-stop con kicker motor-only activo en `SystemCheck`: 10/10 `PASS`, mismo ciclo siempre `true`, acciones stop siempre=3, fallos=0 y detención física en cada caso. El máximo de las nueve repeticiones adicionales fue 15.772 ms; incluyendo la primera medición, el máximo total conservador es 16.005 ms.
- Primera tentativa de medir Stop normal con intake activo reportó `E_STOP` (13.737 ms) y se descartó para ese gate por no corresponder al disparador esperado; el intake sí se detuvo. Al repetir usando explícitamente el botón Stop de la pantalla y sin tocar `BACK`, el reporte válido mostró `OPMODE_STOP`, 4.569 ms, mismo ciclo=`true`, acciones stop=3, fallos=0, gate 50 ms=`PASS` e intake detenido. Falta completar nueve repeticiones válidas y registrar el máximo.
- Serie instrumentada de Stop normal con intake activo en `SystemCheck`: 10/10 `PASS`, evento siempre `OPMODE_STOP`, mismo ciclo siempre `true`, acciones stop siempre=3, fallos=0 y detención física en cada caso. El máximo de las nueve repeticiones adicionales fue 5.324 ms y supera la primera lectura, por lo que también es el máximo total.
- Primera medición instrumentada de release del drivetrain elevado en `MainTeleOp`: `DRIVE_RELEASE`, 6.217 ms, mismo ciclo=`true`, acciones stop=1, fallos=0 y gate 50 ms=`PASS`; las cuatro ruedas avanzaron correctamente, se detuvieron al soltar y no hubo ruido ni vibración. Falta completar la serie y registrar el máximo.
- Serie instrumentada de release del drivetrain elevado en `MainTeleOp`: 20/20 `PASS`, mismo ciclo siempre `true`, fallos stop=0 y las cuatro ruedas detenidas en cada release, sin ruido ni vibración. El máximo de las 19 repeticiones adicionales fue 11.200 ms y supera la primera lectura, por lo que también es el máximo total.
- Primera medición instrumentada de E-stop con drivetrain activo y robot elevado en `MainTeleOp`: `E_STOP`, 31.315 ms, mismo ciclo=`true`, acciones stop=5, fallos=0 y gate 50 ms=`PASS`; se detuvieron el OpMode y las cuatro ruedas. Falta completar la serie y registrar el máximo.
- Serie instrumentada de E-stop con drivetrain activo y robot elevado en `MainTeleOp`: 10/10 `PASS`, mismo ciclo siempre `true`, acciones stop siempre=5, fallos=0 y las cuatro ruedas detenidas en cada caso. El máximo de las nueve repeticiones adicionales fue 5.009 ms; incluyendo la primera medición, el máximo total conservador es 31.315 ms.
- Primera medición instrumentada de Stop normal con drivetrain activo y robot elevado en `MainTeleOp`: `OPMODE_STOP`, 6.028 ms, mismo ciclo=`true`, acciones stop=5, fallos=0 y gate 50 ms=`PASS`; las cuatro ruedas se detuvieron. Falta completar la serie y registrar el máximo.
- Serie instrumentada de Stop normal con drivetrain activo y robot elevado en `MainTeleOp`: 10/10 `PASS`, evento siempre `OPMODE_STOP`, mismo ciclo siempre `true`, acciones stop siempre=5, fallos=0 y las cuatro ruedas detenidas en cada caso. El máximo de las nueve repeticiones adicionales fue 7.150 ms y supera la primera lectura, por lo que también es el máximo total.
- Candidato de fault injection del shooter implementado para MP-01: `SystemCheck` conserva la salida física del shooter bloqueada en compilación (`SHOOTER_OUTPUT_ALLOWED=false`) y usa exclusivamente `gamepad2` para solicitar cuatro fallos simulados: A=voltaje inválido, B=encoder congelado, X=RPM no finita y Y=overspeed. Cada solicitud ordena potencia/target cero antes de activar la inyección; el fault permanece latched y sólo se limpia reiniciando el OpMode. Telemetría nueva: modo inyectado, estado/fault existente y última potencia aplicada. Aún no instalado ni validado físicamente.
- Verificación del candidato de fault injection fuera de OneDrive, usando una copia temporal con los dos archivos fuente relevantes confirmados byte por byte contra el workspace: `git diff --check` PASS; `:TeamCode:packageDebug --stacktrace` PASS; `assembleDebug --no-daemon` PASS en 13 s. APK temporal de 81,282,473 bytes, SHA-256 `B69F467373608F436357F0FFEC51359F435FA4FC35EA882575A6D6F725BD1134`. El primer intento completo en el workspace quedó afectado por reparse points de OneDrive en outputs generados y el primer empaquetado temporal falló transitoriamente; la repetición diagnóstica y la verificación final completaron correctamente sin cambiar Gradle ni dependencias.
- Android Studio compiló e instaló el candidato de fault injection después de pausar OneDrive y regenerar únicamente los directorios `build/`. Artefacto incremental instalado: 81,281,753 bytes, SHA-256 `F2E17D94470EF452F97FA62BD49D84E125F619516BC1B223A991605BFC2FA1FB`, modificado el 2026-07-18 13:45:04. Las cuatro inyecciones aún están pendientes de prueba.
- Primera inyección de fallo con el APK `F2E1...1FB` en `SystemCheck`: al pulsar una vez `gamepad2 A`, el operador confirmó todos los resultados esperados: salida física permitida=`false`, modo=`INVALID_VOLTAGE`, health=`VOLTAGE_FAULT`, target=0, power=0 y cero movimiento; sin ruido ni errores adicionales. Falta comprobar persistencia del latch dentro del mismo OpMode y que sólo un reinicio lo limpia.
- Persistencia de la primera inyección confirmada: sin reiniciar `SystemCheck`, pulsar `gamepad2 B` no reemplazó el fallo; permanecieron modo=`INVALID_VOLTAGE`, health=`VOLTAGE_FAULT`, target=0 y power=0, sin movimiento. El latch fail-closed dentro del mismo OpMode queda `PASS`; falta confirmar limpieza por reinicio.
- Reiniciar `SystemCheck` limpió correctamente la primera inyección y devolvió modo=`NONE`. Segunda inyección: `gamepad2 B` produjo modo=`ENCODER_FROZEN`, health=`ENCODER_FAULT`, razón=`injected frozen encoder`, target=0 y power=0, sin movimiento ni ruido. La detección fail-closed queda `PASS`; falta comprobar persistencia de este segundo latch.
- Persistencia de la segunda inyección confirmada: sin reiniciar `SystemCheck`, pulsar `gamepad2 X` no reemplazó el fallo; permanecieron modo=`ENCODER_FROZEN`, health=`ENCODER_FAULT`, target=0 y power=0, sin movimiento. Segundo latch `PASS`.
- Reiniciar `SystemCheck` limpió correctamente la segunda inyección y devolvió modo=`NONE`. Tercera inyección: `gamepad2 X` produjo modo=`RPM_NON_FINITE`, health=`ENCODER_FAULT`, razón=`non-finite encoder velocity`, target=0 y power=0, sin movimiento, ruido ni errores adicionales. La detección fail-closed queda `PASS`; falta comprobar persistencia de este tercer latch.
- Persistencia de la tercera inyección confirmada: sin reiniciar `SystemCheck`, pulsar `gamepad2 Y` no reemplazó el fallo; permanecieron modo=`RPM_NON_FINITE`, health=`ENCODER_FAULT`, target=0 y power=0, sin movimiento. Tercer latch `PASS`.
- Reiniciar `SystemCheck` limpió correctamente la tercera inyección y devolvió modo=`NONE`. Cuarta inyección: `gamepad2 Y` produjo modo=`OVERSPEED`, health=`OVERSPEED`, razón indicando que las RPM medidas exceden el máximo, target=0 y power=0, sin movimiento, ruido ni errores adicionales. La detección fail-closed queda `PASS`; falta comprobar persistencia y limpieza final de este cuarto latch.
- Cierre del bloque de fault injection con el APK `F2E1...1FB`: sin reiniciar, pulsar `gamepad2 A` no reemplazó la cuarta inyección; permanecieron modo/health=`OVERSPEED`, target=0 y power=0, sin movimiento. Tras Stop y un nuevo INIT+START, el modo regresó a `NONE`, power permaneció en 0 y no hubo movimiento ni ruido. Resultado global: voltaje inválido, encoder congelado, RPM no finita y overspeed detectados `4/4 PASS`; cada caso ordenó target/power cero, permaneció latched frente a otra solicitud y sólo se limpió reiniciando el OpMode. La salida física del shooter permaneció bloqueada durante todo el bloque.
- Candidato de commissioning acotado de torreta implementado después del cierre de fault injection. `SystemCheck` instancia el único owner `TurretSubsystem`, exige centrado manual y `gamepad2 START+BACK` durante 1 s en INIT, y permite un pulso por flanco con `gamepad2 DPAD_LEFT/RIGHT`. Cada pulso queda limitado dentro del subsystem a potencia absoluta 0.10 y se corta al primero de 8 ticks o 150 ms; Stop/E-stop conserva el hook de desarmado/cero. `gamepad2 DPAD_DOWN` simula pérdida de cero, ordena cero, cambia a `INVALID_SIMULATED_RESET` y debe rechazar pulsos posteriores. Esta simulación valida la lógica fail-closed, pero no prueba por sí sola un brownout eléctrico real ni valida los límites provisionales ±200.
- Verificación software del candidato de torreta: `git diff --check` PASS; `:TeamCode:compileDebugJavaWithJavac --no-daemon` PASS en 19 s; `assembleDebug --no-daemon` PASS en 19 s. APK de salida: 81,284,183 bytes, SHA-256 `E952B822B7FA4A7333CE7F594E5F0CAA4B1EB564CCE2C0809BAE7601E4B59581`, modificado el 2026-07-18 14:03:39. Aún no instalado ni probado físicamente.
- Android Studio instaló el candidato incremental de commissioning de torreta: 81,284,196 bytes, SHA-256 `7DA3A5518979BF8C44787482B7B052F9724205B39A13D3836275B3E1CCB6FDD3`, modificado el 2026-07-18 14:06:11. Falta ejecutar la validación física escalonada, comenzando por rechazo de pulso con cero inválido.
- Primera prueba de commissioning de torreta con el APK `7DA3...FDD3`: sin sostener el chord de armado, INIT/START mostraron armed=`false`, zero=`INVALID_INIT` y power=0. Un flanco de `gamepad2 DPAD_RIGHT` fue rechazado con `REJECTED_ZERO_INVALID`; la torreta no se movió, los ticks permanecieron estables y no hubo ruido, vibración ni error. Gate de inmovilidad con cero inválido: `PASS` inicial.
- Armado inicial de commissioning con el APK `7DA3...FDD3`: después de detener el OpMode y realinear físicamente la marca central con cables libres, sostener `gamepad2 START+BACK` durante 1 s en INIT produjo arming=`ARMED`, zero=`VALID_MANUAL_CONFIRMATION` y ticks en cero o muy cerca; no hubo movimiento, ruido, vibración ni error. Falta el primer pulso acotado para identificar dirección física y signo del encoder.
- Primer pulso acotado de torreta con `gamepad2 DPAD_RIGHT`: el operador no observó movimiento físico, pero la telemetría terminó en +14 ticks y `STOPPED_TICK_BUDGET`; pulse active=`false`, sin ruido, vibración ni error. El límite de 8 ticks se detectó y ordenó el corte en el siguiente ciclo, aunque la muestra final fue +14 ticks. La dirección física y la transmisión mecánica quedan sin validar; no repetir pulsos hasta confirmar power final, estabilidad de ticks, marca central y acoplamiento/holgura mecánica.
- Inspección posterior al primer pulso: marca central exactamente alineada, poca holgura visible en la transmisión y `Turret/Power=0` después del corte. Hipótesis de diagnóstico: el recorrido de encoder se consumió tomando holgura antes de producir desplazamiento visible. Se autoriza una única repetición controlada en el mismo sentido para comprobarla; no se cambian potencia, tiempo ni presupuesto de ticks.
- Segunda prueba con un único `gamepad2 DPAD_RIGHT`: la torreta que porta el shooter giró físicamente en sentido horario vista respecto al frente del robot y el engrane del motor giró antihorario, consistente con engranes acoplados en sentidos opuestos. Telemetría final: +29 ticks, `STOPPED_TICK_BUDGET`, pulse inactive y power=0; sin ruido, vibración ni error. Queda confirmado que signo positivo de encoder corresponde a giro horario de la torreta. La muestra final excede el presupuesto declarado de 8 ticks: el corte por polling actúa en el siguiente ciclo y no constituye un límite duro de desplazamiento. Se detiene esta versión de commissioning; no ejecutar el pulso contrario hasta corregir/revalidar el test.
- Candidato de recuperación posterior a FND-027: se eliminó todo binding de movimiento hacia afuera en `SystemCheck`. Durante INIT, `gamepad2 LB+RB` sostenido 1 s puede aceptar la referencia de encoder existente únicamente si `|ticks|<=40`, sin resetearla. Después de START, sólo `gamepad2 DPAD_LEFT` está mapeado y solicita target absoluto 0 mediante `RUN_TO_POSITION`, potencia positiva máxima 0.08, tolerancia 2 ticks y timeout 750 ms. El subsystem detiene si se aleja más del cero, sale de ±40, pierde el cero, llega al target, vence el timeout o recibe Stop/E-stop; luego restaura `RUN_USING_ENCODER`. `DPAD_RIGHT` no tiene binding.
- Verificación fresca del candidato de recuperación: fuente/API comprobada contra `RobotCore-10.3.0-sources.jar`; `:TeamCode:compileDebugJavaWithJavac` PASS; `assembleDebug --no-daemon` PASS final en 15 s; `git diff --check` PASS; búsqueda de binding hacia afuera=`NONE`. APK de salida: 81,285,533 bytes, SHA-256 `3E62729E166C2DBB7CC5B2360391E82FF6590FFEF48D7B2BF2BF22395FEDCE97`, modificado el 2026-07-18 14:28:20. Falta instalar y validar que la lectura cruda siga cerca de +29 antes de autorizar la recuperación.
- Android Studio instaló el APK incremental de recuperación: 81,284,816 bytes, SHA-256 `AEBB4962CB9BBD9C1DED4C1EA6B1C96D4039772E191ED9DEFC693A4BE64834DE`, modificado el 2026-07-18 14:29:50. Antes de armar, se exige leer en INIT la posición cruda conservada y abortar si no permanece cerca de +29.
- Precondición de recuperación fallida de forma segura: después de instalar el APK `AEBB...34DE`, `SystemCheck` en INIT mostró ticks=-2, zero=`INVALID_INIT`, cero movimiento y cero ruido. La referencia previa +29 no sobrevivió la instalación/reinicio, por lo que target 0 ya no representa la marca física. No se sostuvo `LB+RB`, no se inició el OpMode y no se pulsó D-pad. La recuperación electrónica queda abortada; se requiere recenter mecánico mediante procedimiento aprobado y sin forzar la transmisión.
- Confirmación mecánica posterior: el equipo usa una marca de cinta como referencia física de centro y reporta que la torreta ya está nuevamente alineada con esa marca; el mecanismo fue descrito como “muy sensible”. Por tanto no se usará el candidato de recuperación basado en la referencia perdida. El siguiente armado deberá partir de la marca física, resetear encoder durante INIT y conservar cero movimiento hasta una prueba de posición controlada revisada.
- Aclaración de “muy sensible”: con un toque físico pequeño los ticks suben o bajan mucho. Esto puede explicar que +29 ticks correspondieran a un desplazamiento visual pequeño, pero confirma que no existe todavía una conversión validada ticks/grado. Ningún nuevo target, presupuesto de ticks ni límite ±200 se autoriza hasta identificar motor/encoder y relación mecánica o medir ticks/grado con un procedimiento controlado.
- Identificación física de torreta aportada por el equipo y foto: motor goBILDA 5203 Yellow Jacket, 223 RPM, reducción interna 26.9:1; engrane blanco de 68 dientes en el eje del motor impulsa directamente la corona negra de 198 dientes de la torreta. La ficha oficial vigente del SKU `5203-2402-0027` declara 751.8 PPR en el eje de salida. Cálculo teórico: reducción externa `198/68=2.9118`, `2189.06 ticks/rev` de torreta y `6.0807 ticks/grado`. Así, 14 ticks≈2.30°, 29 ticks≈4.77° y los límites actuales ±200 equivaldrían a ±32.89° (arco total≈65.78°). Esta conversión es calculada con dientes reportados y aún requiere comprobación angular física; no valida por sí sola ±200.
- Verificación mecánica manual con robot apagado, lado horario: desde la marca central, la torreta llegó libremente a aproximadamente 33° horarios; cables sin tensión y sin contacto, resistencia anormal ni interferencia reportada. Esto respalda físicamente el margen equivalente a +200 ticks en ese lado; falta repetir desde centro hacia 33° antihorarios antes de aceptar el arco bilateral.
- Verificación mecánica manual con robot apagado, lado antihorario: desde la marca central, la torreta llegó libremente a aproximadamente 33° antihorarios, con cables en buen estado, sin interferencia y con retorno correcto a la marca central. El arco ±33° queda libre bilateralmente; falta medir margen adicional fuera de esos 33° antes de declarar seguros los soft limits ±200.
- Medición manual adicional reportada por el operador: recorrido máximo de aproximadamente 125° horarios y 125° antihorarios desde el centro. Con la conversión teórica 6.0807 ticks/grado, eso equivale a aproximadamente ±760 ticks; el soft limit actual ±200=±32.89° conservaría aproximadamente 92.11°/560 ticks de margen por lado. Falta confirmar si “máximo” se definió por hard-stop, cables, contacto o margen voluntario; no se amplían los límites actuales.
- Medición pasiva posterior en `SYSTEM CHECK` durante INIT, con `Turret/Power=0` y giro manual: alcance reportado de +1070 ticks en sentido horario y -988 ticks en sentido antihorario desde el cero de la marca central. El arco observado suma 2058 ticks; el soft limit vigente ±200 conserva 870 ticks de margen respecto del extremo horario reportado y 788 ticks respecto del extremo antihorario. Estos valores contradicen la equivalencia aproximada previa de ±125°≈±760 ticks, por lo que no se usa todavía esa medición angular para recalibrar ni ampliar límites. Falta registrar el tick al regresar exactamente a la marca central y precisar qué condición física definió cada extremo.
- Retorno posterior exactamente a la marca física de cinta, sin reiniciar el OpMode: `Turret/Ticks=-22`. Para commissioning y límites se acuerda confiar principalmente en ticks medidos; los grados quedan sólo como estimación. El desfase de 22 ticks demuestra que el cero de encoder no sustituye la marca física y requiere medir repetibilidad/backlash desde ambos sentidos antes de un test motorizado.
- Primera prueba de repetibilidad desde el lado positivo: partiendo de la marca física en -22, giro manual hasta aproximadamente +200 ticks y retorno a la misma cinta desde sentido horario terminó en -44 ticks, con salida de motor en cero. La deriva acumulada adicional de -22 ticks no se acepta aún como backlash normal; se pausa el movimiento para comprobar primero estabilidad del encoder sin tocar el mecanismo.
- Prueba de estabilidad posterior: la torreta permaneció exactamente en la cinta y la lectura se mantuvo en -44 ticks durante 2 minutos sin tocar el mecanismo. Esto descarta deriva observable del encoder en reposo durante la prueba y localiza el desfase en el movimiento/retorno mecánico o la repetibilidad visual de la marca, no en un conteo que cambie por sí solo.
- Prueba de repetibilidad desde el lado negativo: con `Turret/Power=0`, partiendo de la cinta en -44 ticks, giro manual hasta aproximadamente -200 y retorno horario a la misma marca terminó en -47 ticks. Diferencia de retorno=3 ticks y sin deriva en reposo reportada. La repetibilidad local desde ese sentido es aceptable para diseñar el siguiente T4 motorizado con margen conservador; no se amplían los límites ±200.
- Candidato T4 motorizado implementado después de la medición pasiva: se retiró la ruta temporal de recuperación y `SystemCheck` volvió al armado estándar desde la marca de cinta, sosteniendo `gamepad2 START+BACK` 1 s durante INIT para resetear encoder a cero. Después de START, `gamepad2 DPAD_LEFT/RIGHT` son hold-to-run a potencia fija máxima 0.05. El subsystem exige cero `VALID_MANUAL_CONFIRMATION`, conserva los soft limits -200/+200, corta por release, límite, cambio de dirección, timeout de 2 s, watchdog de entrada de 100 ms, pérdida de cero, Stop o E-stop, y bloquea reactivación por límite/timeout hasta soltar el D-pad. `DPAD_DOWN` conserva la simulación fail-closed de pérdida de cero. Shooter físico permanece bloqueado por compilación.
- Verificación software fresca del candidato T4: fuente de `DcMotor` contrastada con `RobotCore-10.3.0-sources.jar`; rutas antiguas de recuperación=`NONE`; `:TeamCode:compileDebugJavaWithJavac --no-daemon` PASS; `assembleDebug --no-daemon` PASS final en 18 s; `git diff --check` PASS. APK `TeamCode-debug.apk`: 81,285,752 bytes, SHA-256 `D5AC6F1E11B42F5EC8E973AEB3132B359733A09CB6791809EA442D59E7557536`, modificado el 2026-07-18 15:21:54. Aún no instalado ni probado físicamente.
- Candidato T4 instalado por Android Studio. Primer gate físico sin armar durante INIT: `Turret/Arming=WAITING`, `Turret/Zero=INVALID_INIT`, ticks=-47 conservados y ningún movimiento, ruido ni vibración. No se sostuvo el chord ni se pulsó D-pad antes de esta lectura.
- Gate físico sin armar después de START: un toque de `gamepad2 DPAD_RIGHT` terminó en `Turret/Last result=REJECTED_ZERO_INVALID`, `Power=0`, `Move active=false` y ticks=-47 sin cambio; no hubo movimiento, ruido ni vibración. La ruta T4 queda fail-closed frente a cero inválido.
- Precondición armada T4 después de reiniciar el OpMode: marcas de cinta exactamente alineadas; `gamepad2 START+BACK` durante INIT llegó a `Turret/Arming=ARMED`, `Turret/Zero=VALID_MANUAL_CONFIRMATION` y `Turret/Ticks=0`. El reset no produjo movimiento, ruido ni vibración. Aún no se había pulsado START ni solicitado movimiento.
- Primer pulso armado T4, un toque de `gamepad2 DPAD_RIGHT`: giro físico horario, ticks finales +12, `Turret/Last result=STOPPED_RELEASE`, `Power=0`, `Move active=false`, sin ruido ni vibración. Release instrumentado `SYSTEM_CHECK_TURRET_RIGHT_RELEASE`, tiempo cero 2.265 ms y gate 50 ms=`PASS`. No repetir hasta confirmar estabilidad posterior.
- Estabilidad posterior al primer pulso: después de 20 s sin controles, ticks permanecieron en +12, power=0, torreta completamente detenida y sin error, ruido ni vibración. No hubo reactivación espontánea.
- Primer toque T4 en sentido negativo, partiendo de +12: `gamepad2 DPAD_LEFT` produjo giro antihorario y terminó en -50 ticks, desplazamiento neto -62 ticks; `Turret/Last result=STOPPED_RELEASE`, power=0 y move active=false. Release instrumentado normalizado como `SYSTEM_CHECK_TURRET_LEFT_RELEASE`, tiempo cero 2.726 ms y gate 50 ms=`PASS`. El desplazamiento por “toque” fue mucho mayor que los +12 del lado derecho, por lo que no se repite ni se corrige hasta comprobar estabilidad y distinguir duración humana de respuesta mecánica/control.
- Estabilidad posterior al toque negativo: tras 20 s sin controles todo permaneció igual en -50 ticks, power=0, torreta completamente detenida y sin ruido, vibración ni error. No hay deriva ni reactivación; para investigar la asimetría hace falta instrumentar duración real del hold y delta de ticks, porque el tiempo cero de release no mide cuánto tiempo estuvo presionado el D-pad.
- Cierre de sesión 2026-07-18: después de la estabilidad en -50 ticks, el operador pulsó `STOP` y confirmó explícitamente `DETENIDO`. Última evidencia observable previa al Stop: power=0, move inactive, -50 estable durante 20 s y sin ruido/vibración/error. `-50` es sólo la lectura relativa final de esa inicialización y no debe tratarse como centro físico ni reutilizarse tras reinicio; el centro sigue siendo la marca de cinta. La próxima sesión comienza con robot deshabilitado, inspección visual y recentrado manual a la cinta antes de confirmar un cero nuevo.
- Verificación pre-commit de cierre: el primer rebuild encontró un placeholder `ReparsePoint` de OneDrive dentro del directorio ignorado `TeamCode/build` y falló en `mergeDebugResources`; se validó la ruta, se eliminó únicamente ese output generado y `assembleDebug --no-daemon` pasó desde limpio en 53 s. APK limpio resultante: 81,285,031 bytes, SHA-256 `B8E72645AA8FA36678F9EDEC26216259679208F4BDCDCFE6DB3ABA2857E5E1B6`, modificado el 2026-07-18 15:49:50. Este APK limpio no fue instalado ni probado físicamente; toda evidencia T4 de esta sesión sigue ligada al APK instalado `D5AC...7536`.
- Segunda repetición en el mismo sentido después de nuevo centrado/armado: ningún movimiento físico visible, ticks finales +29, resultado `STOPPED_TICK_BUDGET`, power=0 y sin ruido, vibración ni error. Esto no apoya que el primer resultado fuera sólo toma de holgura. Nueva hipótesis: los ticks corresponden al eje del motor antes de una reducción mecánica suficiente para volver imperceptible el desplazamiento de salida. No ejecutar un tercer pulso ni ampliar el presupuesto sin identificar modelo/ticks por revolución del motor y relación/transmisión hasta la torreta.

## Entrega de la Pista Software — 2026-07-18

Las 6 tareas de `plan-paralelo-20h.md` sección 3 quedaron implementadas y commiteadas en `masterplan` (C1-C6). Solo software: ningún APK fue instalado y el comportamiento validado físicamente no cambió.

- `LimelightSubsystem` + `LimelightObservation` + OpMode `Limelight Diagnostic` (cero actuadores) — desbloquea el Paso 4 de Tuning. Dispositivo opcional vía `tryGet` (DEC-028): corre degradado sin la Limelight configurada.
- `pedroPathing/Constants.java` reparado: sin código muerto, encoders leyendo los pods reales vía `RobotMap` (el string `rightRear` y el strafe cruzado a `rightFront` eran bugs), offsets como constantes `TBD_*` + flag `LOCALIZER_OFFSETS_CALIBRATED=false` para el Paso 2.
- `TeamCode/src/test` creado con JUnit 4.13.2 (única dependencia nueva); `util/Angles` extraído de `DriveSubsystem` con semántica idéntica; 23/23 tests PASS.
- Paquete `localization/`: `PoseProvider`/`PoseSnapshot`/`DriveAdapter` neutrales (MP-02) con adapter Road Runner; RR sigue siendo el owner (DEC-034 pendiente). Cambio aditivo en `DriveSubsystem` (cachea velocidad ya calculada, `resetEpoch`).
- Paquete `shooter/`: modelos RPM-por-distancia lineal/cuadrático/piecewise (piecewise adaptado de HyperionBots FTC 18011) + selector DEC-012 con clamp de seguridad dentro del modelo. Espera el dataset del Paso 5.
- `telemetry/TelemetryBlocks` + `inventario-telemetria-tuners.md` (prep MP-07 y notas de entorno de build: JBR + `GRADLE_USER_HOME` ASCII para tests CLI).

- Instrumentación FND-027 implementada (2026-07-19): `TurretSubsystem` mide cada jog de commissioning — duración real del hold (desde el primer ciclo del D-pad hasta el corte), delta de ticks (inicio→fin) y resultado — sin cambiar potencia 0.05, timeout 2 s, watchdog 100 ms ni soft limits ±200. `SystemCheck` muestra el bloque `TORRETA: ULTIMO JOG (FND-027)` con duración, delta, ticks inicio→fin, potencia, resultado y ticks/segundo. Un Stop/E-stop que corte un jog activo también cierra su medición. Los rechazos (`REJECTED_*`) no generan medición.
- Verificación software del candidato instrumentado FND-027: `:TeamCode:compileDebugJavaWithJavac` PASS, `:TeamCode:testDebugUnitTest` 23/23 PASS, `assembleDebug` PASS, `git diff --check` PASS. Primer APK: 81,349,920 bytes, SHA-256 `45BB93AFE5D59ACEB7E1542D85114636A90D640A3886B257E1E9A56F5F540AEE` (2026-07-19 01:12:59) — **superseded, no instalar**: la revisión de código posterior produjo fixes (`5cda4ce`).
- Revisión de código de la pista completa (2026-07-19): sin errores de signo/matemática en encoders, ángulos, pods ni modelos RPM; tres fixes menores aplicados en `5cda4ce` — warning global de Pedro sin calibrar (fail-loudly restaurado), referencia histórica de `strafePodX=-2.5`, y documentación de que la velocidad de `PoseSnapshot` está en frame del robot. **APK candidato vigente** `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`: 81,365,379 bytes, SHA-256 `C2341F90CAAAFBE111007582E8D43DDD35D85F5F9712D34C6893BDA3E61AA6FC`, modificado 2026-07-19 09:50:11, sobre `masterplan@5cda4ce`. **No instalado ni probado físicamente.** Incluye las entregas C1-C6 + instrumentación FND-027 + fixes de review, agrupadas en un solo APK según la regla del plan.

### Remediación de revisión adversarial posterior — 2026-07-19

Una segunda revisión externa encontró defectos fail-open que no sobrevivieron la primera pasada; por tanto, la afirmación histórica anterior queda superseded para RPM/pose/Limelight y el APK `C234...A6FC` no debe instalarse como candidato vigente.

- DEC-012 ahora separa ajuste matemático de certificación física. `ShotTrial` conserva candidato/sesión/distancia/rol, target, RPM al feed, hold de readiness y resultado. Cada celda requiere ≥10 tiros en cada una de dos sesiones; calibración ≥9/10 y holdout ≥8/10. Además, cada intento debe usar el target del candidato y cumplir ±100 RPM por ≥250 ms. Si ningún modelo pasa, la salida es cero RPM con `NO_MODEL_MET_CRITERIA`.
- Los tres modelos RPM devuelven cero para distancia negativa, NaN o infinita; los ajustes lineal/cuadrático rechazan overflow o coeficientes no finitos.
- `PoseSnapshot` normaliza heading a `(-π,π]` y rechaza/sanitiza pose o velocidad NaN/∞. `RoadRunnerPoseProvider` conserva el frame de velocidad del robot y publica el valor raw fallido únicamente en diagnóstico; un `setPose` no finito no muta Road Runner.
- Limelight separa `LimelightRawSample` diagnóstico de `LimelightObservation` accionable. Sólo un tag esperado, fresco y completamente finito puede ser `VALID`; cualquier rechazo publica ceros finitos. El bloque raw aparece sólo en `Limelight Diagnostic`, que continúa con cero actuadores.
- No se tocaron mappings, signos/direcciones de pods, ownership de Road Runner/Pedro, límites ni rutas Stop/E-stop de torreta, `MainTeleOp`, `RobotContainer`, `FieldCentricDriveCommand`, ni el bloqueo físico del shooter en SystemCheck. Falta validación física; esta remediación no certifica hardware.
- Verificación software de la remediación: `:TeamCode:compileDebugJavaWithJavac` PASS; `:TeamCode:testDebugUnitTest` 32/32 PASS; `assembleDebug` PASS; `git diff --check` PASS. APK generado desde el working tree: 81,381,794 bytes, SHA-256 `86867EAD8C9476E3D5C706981A7A16CC606943CC1D9C5614EBA8CF827EEC5756`, modificado 2026-07-19 12:26:19. **No instalado ni probado físicamente; no promover hasta revisión humana y gates de hardware.**

## Gate completado — Tuning: instrumentación y matriz de jog (FND-027)

**Completado y aceptado el 2026-07-20.** El procedimiento siguiente se conserva como registro histórico del gate ejecutado; no es una autorización para repetirlo.

La instrumentación ya está implementada; lo que sigue es físico. Procedimiento para la persona de Tuning, paso a paso:

1. **Precondiciones** (robot deshabilitado): OpMode detenido, área despejada, torreta alineada exactamente a la marca de cinta con cables libres. No conservar ni reutilizar el cero relativo `-50` de la sesión anterior: el centro es la marca física.
2. **Instalar el candidato**: APK SHA-256 `C234...A6FC` sobre `masterplan@5cda4ce` (o el incremental que genere Android Studio con OneDrive pausado — registrar el hash real instalado; no trasladar resultados entre hashes). No usar el APK `45BB...0AEE` anterior: quedó superseded por los fixes de la revisión.
3. **Gate sin armar** (igual que siempre): INIT sin chord → `Turret/Zero=INVALID_INIT`, cero movimiento. Tras START, un toque de `gamepad2 DPAD_RIGHT` debe terminar en `REJECTED_ZERO_INVALID`, ticks sin cambio, y el bloque `ULTIMO JOG` debe seguir en `SIN MEDICIÓN` (los rechazos no miden). Si aparece una medición aquí, detener y reportar: es un bug.
4. **Armar desde la marca**: `gamepad2 START+BACK` sostenido 1 s durante INIT → `ARMED`, `Turret/Ticks=0`, sin movimiento.
5. **Un solo hold temporizado sentido positivo**: tras START, sostener `gamepad2 DPAD_RIGHT` intentando una duración uniforme (~0.5 s) y soltar. Leer y registrar el bloque `ULTIMO JOG`: `Duración hold` (ms), `Delta ticks`, `Ticks X -> Y`, `Ticks por segundo`, `Resultado` (se espera `STOPPED_RELEASE`). Esperar 20 s sin controles y registrar también el `Turret/Ticks` en vivo — la diferencia contra el `endTicks` del reporte es deriva post-corte por inercia, dato aparte.
6. **Recentrar y repetir en sentido negativo**: Stop, realinear la torreta a la cinta, reiniciar INIT, armar de nuevo (ticks=0) y hacer un solo hold con `gamepad2 DPAD_LEFT` de duración similar. Registrar lo mismo. Recentrar antes del hold negativo es deliberado: la sesión anterior midió el negativo partiendo de +12, lo que contamina la comparación.
7. **Comparación que cierra o escala el finding**: con duraciones de hold similares, si los `Ticks por segundo` de ambos sentidos son similares, la asimetría +12/-62 se explica por duración humana del D-pad y T4 puede continuar (release/Stop/E-stop y límites por etapas). Si con duraciones similares los ticks/segundo difieren claramente, es mecánica/control: detenerse, no ejecutar matrices ni buscar límites, y reportar aquí con el formato `SHA | fecha | operador | configuración | repeticiones | resultado | tiempo máximo | evidencia | observaciones`.
8. En cualquier ruido, vibración, deriva o movimiento no explicado: conservar el punto de detención, Stop, y registrar antes de continuar.

### Evidencia de cierre FND-027 — 2026-07-20

- El checkout de build/instalación se movió fuera de OneDrive a `C:\dev\RobotCode2026`; build limpio, 32/32 tests y `assembleDebug` pasaron antes de instalar el candidato.
- Candidato probado: autocorte `850 ms`, potencia `±0.050`, hold-to-run, soft limits `±200`, watchdog `100 ms`, cero manual confirmado y Stop/E-stop sin debilitar. APK SHA-256 `9C2F58F3327064A199BC9426A646C05550BDC3F462ACF90E3534851BAD9B38E0`.
- Gate sin armar: `REJECTED_ZERO_INVALID`, ticks sin cambio, power `0`, move inactive y cero movimiento/ruido/vibración.
- Matriz aceptada por el Test lead mediante DEC-039: 10/10 positivos y 10/10 negativos, 20/20 `STOPPED_TIMEOUT`, power final `0`, `Move active=false`, parada completa, estabilidad durante la espera y cero anomalías.
- Rangos: duración `855.0–881.6 ms`; |delta| positivo `54–60`; |delta| negativo `59–72`; deriva postcorte `-6..+9 ticks`, sin cambio posterior durante la espera. Batería registrada `11.74–11.93 V`.
- Nueve parejas con precarga estandarizada: positivo `67.8 ticks/s`, negativo `75.7 ticks/s`; negativo `11.6%` más rápido en promedio, pero la razón por pareja varió de `-1.1%` a `+25.5%`. Se documenta como limitación conocida, no como constante de compensación.
- Mecánica reportó un punto muerto aproximado de tres dientes dentro del tren motor/reductora, con ambos engranes externos moviéndose juntos y sin piezas flojas ni salto de dientes. La estimación teórica es ≈`33 ticks`/`5.45°`; no queda corregida por este gate.
- Fuente de captura: `PRUEBAS COMPLETAS.xlsx`, conservada por el Test lead. Hora no registrada en pruebas 1, 2, 8 y 18; batería no registrada en 1, 2, 6, 8, 18 y 19. No se inventan esos campos.

## Próxima acción — terminar los gates restantes de MP-01

No ejecutar más pulsos FND-027. Reconciliar FND-003/FND-020/FND-021 y las regresiones `FIX_READY` contra el gate MP-01; después emitir `READY_FOR_GATE` o conservar `BLOCKED` con el pendiente exacto. La asimetría de velocidad se tratará en el control cerrado/auto-aim posterior, no mediante un multiplicador fijo en este candidato.

### Candidato de primer pulso físico del shooter — 2026-07-20

- Sólo `SystemCheck`: `gamepad1 START+A` sostenido solicita un único pulso por INIT; `A` sin `START` termina en `REJECTED_ARM_CHORD` y cero. Soltar, `B`, Stop o E-stop ordenan cero.
- Valores fijos no editables por Dashboard para este gate: objetivo indicado `1000 RPM`, techo de salida `0.10` aplicado en el último punto antes de `motorLeader.set(...)`, autocorte `500 ms`. El bang-bang/PID puede pedir más internamente, pero esta instancia no puede aplicar más de `0.10`.
- Telemetría: health/fault, power aplicado, resultado, duración, ticks de encoder inicio/fin, delta, RPM indicada máxima y batería inicio/fin. No alimentar piezas ni accionar intake/kicker.
- Verificación software fresca: `:TeamCode:testDebugUnitTest` 32/32, `assembleDebug` PASS, `git diff --check` PASS y cero ReparsePoints en `C:\dev\RobotCode2026`.
- APK no instalado ni probado físicamente: 81,311,179 bytes; SHA-256 `0ADF7218167026BF63D1E7F58F55761F9CB63D4B72C4D224C26492A82D79A1F4`; generado 2026-07-20 13:26:53. No atribuirle evidencia del APK `9C2F`.
- Antes de energizar faltan inspección de guardas/montaje/cables, Safety operator con Stop y confirmación del sentido físico esperado. El modelo exacto, relación efectiva, corriente, temperatura y RPM independiente siguen pendientes; la RPM mostrada es indicada por las constantes actuales, no una certificación física.
- Gate físico sin chord reportado después de instalar el candidato: `Shooter/Health=HEALTHY`, fault=`none`, output limit=`0.10`; `gamepad1 A` sin `START` terminó en `REJECTED_ARM_CHORD`, power `0`, pulse active=`false`, pulse consumed=`false`, ticks sin cambio y cero movimiento/ruido/vibración. Safety operator presente con Stop. El hash instalado no se verificó independientemente en esta lectura; no se autoriza todavía `START+A` hasta cerrar la inspección mecánica previa.
- Primer pulso autorizado: batería previa `11.61 V`; `STOPPED_TIMEOUT` a `527.3 ms`, ticks `0 -> 0`, delta `0`, peak indicated RPM `0.0`, batería `11.58 -> 11.58 V`. Hubo giro físico hacia dentro (incorrecto), luego parada completa; power final `0`, pulse active=`false`, consumed=`true`, health=`HEALTHY`, fault=`none`, sin anomalías y RPM indicada `0` tras 20 s. **Stop/no repetir:** FND-028 queda abierto; revisar encoder físicamente y corregir el gate de detección antes de otro movimiento.
- Inspección con robot desenergizado: motor goBILDA 5203 Series Yellow Jacket brushed DC 6000 RPM; encoder conectado firmemente al puerto 3, sin daño visible; sentido correcto confirmado hacia fuera. Candidato pasivo generado: salida global del shooter compilada en cero, ticks de encoder en vivo y rechazo `REJECTED_OUTPUT_DISABLED`; 32/32 tests, `assembleDebug` y `git diff --check` PASS, cero ReparsePoints. APK no instalado: 81,311,287 bytes, SHA-256 `B4CBC43A7ABA4BD02FB3ADB6B9A49A33FEA8A0AD1054BE681DD088BA25050988`, generado 2026-07-20 13:52:29.
- Prueba pasiva con ese candidato: output allowed=`false`, applied power=`0`; giro manual hacia fuera `0 -> 28` ticks y retorno hacia dentro `28 -> 0`; `Actual RPM` ≈`85` durante el giro y casi cero después, health=`HEALTHY`, fault=`none`, sin movimiento espontáneo. El encoder está funcional; queda por explicar por qué el primer pulso no produjo un tick.
- Candidato de hipótesis 2 no instalado: motor `SHOOTER_MOTOR_IS_INVERTED=false` para cambiar el giro hacia fuera; encoder FTCLib `SHOOTER_ENCODER_IS_INVERTED=true` para conservar RPM positiva según las semánticas verificadas de SDK 10.3.0/FTCLib 2.1.1. Sólo `SystemCheck` habilita output bajo cap `0.10`; watchdog de respuesta `250 ms`, autocorte `500 ms`, un pulso por INIT. Build/test PASS 32/32; APK 81,352,643 bytes, SHA-256 `2A193852530748B23C8D8F2B3F27262F0134ADBB84B49D1DE61F00F7727D254D`, generado 2026-07-20 14:05:49.
- Gate sin chord del candidato anterior: `REJECTED_ARM_CHORD`, physical output allowed=`true`, applied power=`0`, pulse inactive/not consumed, health=`HEALTHY`, fault=`none` y sin movimiento. El pulso armado posterior terminó `STOPPED_ENCODER_NO_RESPONSE`; el equipo reportó cero movimiento porque power `0.10` no vence el umbral mecánico. Por autorización explícita del Test lead se preparó un único pulso de breakaway a cap `0.50`, autocorte `300 ms` y watchdog de un tick en `150 ms`, conservando todos los cortes. APK no instalado: 81,311,759 bytes, SHA-256 `B2946F95B60FD0CF5736EA2CF8B7D24206852E6649487CC84873A745B7C0F7EE`; 32/32 tests, build/diff/ReparsePoints PASS.
- Pulso `B294`: `STOPPED_TIMEOUT` `318.2 ms`, ticks `0 -> 4`, encoder responded, peak indicated `85.7 RPM`, batería `11.31 -> 10.39 V`, power final `0`, parada completa y cero anomalías, pero sentido físico hacia dentro. La dirección normal queda refutada. Candidato siguiente vuelve motor invertido y encoder FTCLib normal, manteniendo `0.50`/`300 ms`/watchdog `150 ms`; APK no instalado 81,311,767 bytes, SHA-256 `3C0C8DC6A481254755FE492ABEDD238BA3C11DFD6BBAD7A3E4A5F7CAF7A56144`, build/test 32/32 PASS.
- Retest `3C0C`: `STOPPED_TIMEOUT` a `310.5 ms`, ticks `0 -> 11`, delta `11`, encoder responded=`true`, peak indicated `257.1 RPM`, batería `13.56 -> 13.23 V`, pulse inactive/consumed, health=`HEALTHY`, fault=`none`, RPM final `0`. Dirección física hacia fuera, parada completa y cero anomalías. Dirección/watchdog/corte aprobados; no repetir este pulso. Siguiente frente: caracterización RPM escalonada sin piezas.
- Candidato RPM 1 no instalado: target indicado `1000 RPM`, cap `0.50`, autocorte `2000 ms`, watchdog `150 ms`, sin piezas/feeder y un pulso por INIT. Reporta peak/end RPM, batería inicio/fin/mínima y máximo ready hold. APK 81,312,043 bytes, SHA-256 `F847E8BBB747A7C505B5367DED73C4504D31FE805FF93B1C47B96E74A2DD9F21`; 32/32 tests, build/diff/ReparsePoints PASS.
- Resultado RPM 1: texto transcrito `SPOTED_TIMEOUT` (exacto pendiente de confirmar), `2027.2 ms`, ticks `0 -> 276`, peak/end `857.1/771.4 RPM`, batería `13.31 -> 13.10 V`, mínima `12.97 V`, ready hold `0 ms`, power final `0`, dirección afuera, parada y cero anomalías. No alcanzó readiness. Candidato RPM 2 no instalado: target `1000`, cap `0.75`, `3000 ms`, watchdog `150 ms`; APK 81,312,039 bytes, SHA-256 `09B23FD2ED57E30496D5D7FBB710F21FDD34A77255F1D414C8578BBE465DDAC1`, 32/32 tests/build/diff PASS.
- Resultado RPM 2 bajo APK `09B2...DAC1`: `STOPPED_TIMEOUT`, `3027.7 ms`, ticks `0 -> 555`, peak/end `1028.6/942.9 RPM`, batería `13.11 -> 12.94 V`, mínima `12.37 V`, ready hold `125.6 ms`, power final `0`, inactive/consumed, health=`HEALTHY`, fault=`none`, dirección hacia fuera, parada completa y cero anomalías. Pasa dirección/encoder/watchdog/autocorte de FND-028; no pasa todavía T8 (`>=250 ms` continuos) y el feeder sigue bloqueado.
- Cierre formal FND-028: el Test lead aceptó el gate funcional para MP-01 bajo APK `09B2...DAC1`. El cierre no traslada evidencia a T8, no certifica carga y no habilita feeder; esas tareas permanecen en MP-06/T8.

### Candidato de TeleOp integrado sin kicker servo — 2026-07-20

- Nuevo OpMode visible `PRUEBA: MECANISMOS SIN SERVO`; no instancia drive, cámara, hood ni CRServo como actuador. Reutiliza ownership de `ShooterSubsystem`, `IntakeSubsystem`, `KickerSubsystem` y `TurretSubsystem` una sola vez cada uno.
- `KickerSubsystem` acepta ahora un interlock por owner. Este TeleOp construye `new KickerSubsystem(hardwareMap, false)`, por lo que `kick()`/`reverse()` sólo pueden mover `kickerMotor`; un CRServo disponible recibe cero en stop pero nunca potencia distinta de cero desde este owner, incluso si la bandera Dashboard estaba activa al INIT.
- Controles: gamepad1 `START+A` shooter hold-to-run, D-pad arriba/abajo selecciona `1000/2450/2900/3600 RPM`, `B` stop manual latched y `BACK` E-stop; gamepad2 `START+RB` kicker forward sólo tras `250 ms` continuos de readiness, `START+LB` reversa sólo con shooter apagado, triggers intake y D-pad izquierda/derecha torreta.
- Límites/cortes: shooter cap `0.75`, watchdog de encoder `150 ms`, autocorte `5000 ms`; torreta exige cero manual en INIT, power `0.05`, soft limits `±200`, watchdog y autocorte `850 ms`; todos los controles son hold-to-run y cada subsistema conserva su hook de Stop.
- Verificación fresca: `:TeamCode:testDebugUnitTest` 32/32 (`0` failures, `0` errors), `assembleDebug` PASS, `git diff --check` PASS y cero ReparsePoints. APK no instalado: 81,315,843 bytes, SHA-256 `D51A618B18ADE4652646F74B83357742D0C960BE031EAC7ACCCFE1751D9A0639`, generado 2026-07-20 16:53:47.
- Este APK cambia respecto a `09B2...DAC1`: no hereda automáticamente su aceptación física. Primera etapa obligatoria tras instalar: INIT/telemetría con todos los outputs en cero y `Kicker/Servo allowed by TeleOp=false`, seguida de regresión sin piezas a `1000 RPM`. Targets mayores, kicker integrado y piezas requieren autorización física separada.

### Corrección de handoff — TeleOp integrado rechazado — 2026-07-20

- El Test lead rechazó como incorrecto el mapa de controles comunicado para `PRUEBA: MECANISMOS SIN SERVO`. No usar ese mapa, no iniciar la prueba física y no inferir que el diseño actual permite decidir todavía si se reinstala el servo.
- El APK `D51A...0639` y sus parámetros `0.75`/`5000 ms` quedaron superseded por cambios locales posteriores; su hash no identifica el APK producido después de esos cambios.
- Único requisito nuevo aceptado: techo máximo permitido del shooter `0.90`. Esta aceptación fija un límite de configuración; no constituye autorización de movimiento, piezas, feeder ni prueba integrada.
- El código local posterior fue compilado con `:TeamCode:testDebugUnitTest assembleDebug --no-daemon` y terminó `BUILD SUCCESSFUL`; no se alcanzó a registrar un SHA-256 nuevo. Por ello el APK posterior queda `NO IDENTIFICADO / NO AUTORIZADO PARA PRUEBA FÍSICA`.
- Estado de reanudación: robot detenido; feeder y CRServo continúan bloqueados; conservar como evidencia física válida únicamente FND-027 bajo `9C2F` y FND-028 bajo `09B2...DAC1`. Antes de otro upload o movimiento, Software debe corregir el contrato exacto de botones con el Test lead, generar hash nuevo y entregar una tabla revisada.

### Reanudación autorizada — controles simples del TeleOp integrado — 2026-07-20

- El lead autorizó reemplazar el mapa rechazado. Contrato nuevo: gamepad 1 `A` enciende intake, `B` lo apaga, `X` enciende únicamente `kickerMotor`, `Y` lo apaga; D-pad izquierda/arriba/abajo/derecha solicita respectivamente `-150/-50/+50/+150 ticks` de torreta. Gamepad 2 `A` enciende shooter a potencia constante `0.90` y `B` lo apaga.
- El shooter ya no usa chord, hold-to-run ni autocorte temporal en este OpMode. Una caída normal de RPM no lo apaga; permanecen activos los cortes por fault latched (voltaje/encoder/lectura/overspeed), Stop y E-stop.
- La torreta conserva confirmación manual del cero durante INIT (`gamepad2 START+BACK` por 1 s), potencia máxima `0.05`, límites `-200/+200`, tolerancia de llegada, timeout de 5 s y corte por cero inválido/Stop/E-stop. Los cuatro presets son candidatos de prueba, no posiciones mecánicas calibradas.
- El interlock del owner conserva el CRServo en cero: `new KickerSubsystem(hardwareMap, false)`. Este candidato autoriza únicamente el kicker motor-only; no autoriza reinstalar el CRServo ni alimentar piezas.
- Verificación fresca en `C:\dev\RobotCode2026`: 32/32 tests JVM, `assembleDebug` y `git diff --check` PASS. APK no instalado: 81,315,943 bytes, SHA-256 `BC7DCF58EE32C973E428EE2304CF68E74FD1CC20047B62C83A42D60F7CB0B1A6`, generado 2026-07-20 17:20:43.
- Este gate correspondía al candidato previo `BC7D...B1A6`; quedó superseded por la confirmación motor-only y los nuevos límites registrados abajo.

### Configuración final sin servo y nuevos límites de torreta — 2026-07-20

- El lead confirmó que el robot se queda definitivamente sin `kickerServo`. `KICKER_SERVO_ENABLED` pasa a constante de compilación `false`; ninguna edición de Dashboard ni otro OpMode puede reactivarlo. `KickerSubsystem` conserva el lookup opcional únicamente para ordenar cero si el nombre todavía existe en la configuración RC.
- FND-026 queda cerrado para la configuración final motor-only. Reinstalar un servo sería un cambio de diseño nuevo, no un rollback autorizado.
- Límites anteriores de software: `-200/+200 ticks`.
- Nueva medición/decisión del lead: extremos confirmados `-993/+935`; límites activos `-983/+925`, dejando 10 ticks completos hacia el centro en ambos lados. `TurretSubsystem.LIMIT_LEFT/RIGHT` quedan en `-983/+925`.
- El margen de 10 ticks supera por 1 tick la deriva postcorte máxima de 9 ticks observada en FND-027. Sigue siendo un margen estrecho: no autorizar aim/movimiento automático hasta demostrar a baja potencia que el corte real no contacta estructura ni tensa cables.
- FND-003 pasa de `BLOCKED_PHYSICAL` a `FIX_READY`: ya existe contrato numérico, pero falta verificar con robot elevado que ambos sentidos corten en el límite, terminen con power cero y no tensen cables ni contacten estructura. Los presets del TeleOp (`-150/-50/+50/+150`) no cambiaron.
- Los APK `BC7D...B1A6` y `42CC...9C4D` quedan superseded; ninguno contiene el margen bilateral final de 10 ticks. Candidato nuevo no instalado: 81,355,551 bytes, SHA-256 `A8CC04737C11551801AE4C890EE2CEC9FF172E77CC9C112263E1E678C110083F`, generado 2026-07-20 17:59:45; 32/32 tests JVM, `assembleDebug` y `git diff --check` PASS.

### Gate ejecutado — candidato anterior FND-003/FND-020

El procedimiento siguiente se conserva como evidencia del candidato `A8CC...083F`; FND-020 ya cerró y los números de FND-003 quedaron superseded por la actualización posterior.

- Usar exclusivamente el APK `A8CC...083F`, robot elevado, sin piezas, torreta centrada inicialmente, cables libres y Safety operator sobre Stop.
- FND-003: en `SystemCheck`, confirmar cero durante INIT y acercarse por pulsos `gamepad2 DPAD_LEFT/RIGHT` a potencia `0.05`, soltando entre pulsos. Validar por separado `-983` y `+925`: `STOPPED_SOFT_LIMIT`, power final cero, sin contacto/tensión y sin reactivación hacia afuera; el mando hacia adentro sí debe alejarse. Repetir 3 veces por lado desde un INIT/cero nuevo y registrar primer corte y lectura estable a 2 s.
- FND-020A: sin confirmar cero, START y ambos D-pads deben producir `REJECTED_ZERO_INVALID`, power cero y ningún movimiento.
- FND-020B: tras armar, `gamepad2 DPAD_DOWN` simula pérdida de cero; debe mostrar `INVALID_SIMULATED_RESET`, cortar en el mismo ciclo y rechazar movimiento posterior hasta reiniciar/recentrar/rearmar.
- FND-020C: mover modestamente a un lado, Stop y hacer power-cycle controlado del Control Hub con la torreta físicamente fuera de la marca. En el nuevo INIT, aunque el encoder muestre cero, `zero=INVALID_INIT` y todo movimiento debe quedar bloqueado. Repetir desde ambos lados. No provocar brownout eléctrico bajo carga deliberadamente.
- Cierre: FND-003 requiere 3/3 por lado sin rebasar extremos `-993/+935`; FND-020 requiere 3/3 por gate/lado con cero inválido y power cero. Registrar SHA, operador, batería, ticks, resultado, power, tiempo de corte y estado físico.

### Resultado de gates y nuevo envelope de torreta — 2026-07-20

- El lead confirma que todos los gates prescritos, incluido FND-020, pasaron correctamente. FND-020 queda `CLOSED` por atestación del operador: inicio sin armar, pérdida simulada y reinicio/power-cycle fuera de centro conservaron cero inválido, power cero y bloqueo de movimiento.
- El límite negativo permanece `-983`. El lead reemplaza el límite positivo anterior por `+1070`, indicando que ese número ya incluye la tolerancia de 10 ticks.
- Potencia de torreta actualizada por el lead a `0.50`. Para conservar margen de corte, el subsystem usa `0.50` durante recorrido y reduce automáticamente a `0.05` dentro de los últimos 100 ticks antes de cada soft limit. Presets también reducen a `0.05` en los últimos 100 ticks antes del target.
- El APK `320C...D226` queda superseded por este aumento de potencia. FND-003 requiere retest nuevo; FND-020 permanece cerrado y sólo necesita spot-check sin armar.
- Candidato `0.50→0.05` no instalado: 81,315,491 bytes, SHA-256 `5FF4D30D0BD4758089CD4A99EB865D8ECDF8B9567FAEC72D0CBDBF786BFC4E61`, generado 2026-07-20 18:26:19; 32/32 tests JVM, `assembleDebug` y `git diff --check` PASS.
- El APK `A8CC...083F` queda superseded por el nuevo límite/potencia. Candidato nuevo no instalado: 81,315,527 bytes, SHA-256 `320CC11B18A950BB17365D52ABEBF8589304B666008FC427C4903F5AB5BBD226`, generado 2026-07-20 18:17:22; 32/32 tests JVM, `assembleDebug` y `git diff --check` PASS. FND-003 permanece `FIX_READY`: repetir 3/3 por lado con este APK y registrar primer corte, lectura estable, power cero y ausencia de contacto/tensión. FND-020 sólo requiere un spot-check sin armar después de instalarlo.

### Cierre FND-003 — 2026-07-20

- El lead confirma que terminó correctamente el retest físico prescrito con el envelope final `-983/+1070`, potencia de recorrido `0.50` y aproximación `0.05` en los últimos 100 ticks.
- La confirmación cubre ambos límites, corte a power cero, retorno permitido hacia el interior y ausencia de contacto o tensión observada. FND-003 queda `CLOSED` por atestación del Test lead; no se inventan ticks, batería ni tiempos que no fueron reportados.
- Siguiente bloqueo físico: FND-021. El export XML no está disponible en la versión instalada y su atestación DS ya está aceptada como sustituto; quedan por firmar orientación/signo del IMU y los datos físicos pendientes del contrato.

### FND-021 — corrección de orientación IMU 2026-07-20

- El lead confirma los mappings `par0=rightFront`, `par1=leftFront` y `perp=rightBack`.
- Montaje físico confirmado: logo del Control Hub hacia `RIGHT`, USB hacia `BACK`. El dueño activo `MecanumDrive` declaraba `RIGHT/UP`; se corrigió únicamente `UsbFacingDirection.UP` a `UsbFacingDirection.BACKWARD`.
- La orientación queda `FIX_READY`, no físicamente validada: instalar el APK nuevo y confirmar heading con giros manuales antes de cerrar esa parte de FND-021. Pedro/TankDrive, direcciones de motores y offsets de pods no cambian.
- Candidato IMU no instalado: 81,315,519 bytes, SHA-256 `E586B4B0EE047382F2BD7E2B9D26E8F53EC72650971EE1A7A9751BFCE228DBB6`, generado 2026-07-20 18:52:40; `:TeamCode:testDebugUnitTest` 32/32, `assembleDebug` y `git diff --check` PASS.

### Resultado físico IMU — 2026-07-20

- El lead entrega `TUNEO IMU.xlsx`, 11,157 bytes, SHA-256 `817FFB58302D753B4A56B8FC4D2BFF2B77CEC01BD554B8793985B8A85CBB0DF6`; la hoja contiene 10/10 mediciones y reporta `APROBADO`, sin errores de fórmula.
- Antihorario: deltas `+89.3/+90.6/+91.0/+91.2/+90.5°`, media `+90.52°`, signo correcto 5/5 y error máximo `1.2°`.
- Horario: deltas `-91.7/-92.0/-94.4/-92.9/-91.1°`, magnitud media `92.42°`, signo correcto 5/5 y error máximo `4.4°`.
- Con tolerancia de `5°`, orientación física `logo=RIGHT`, `USB=BACK` y código `RIGHT/BACKWARD`, la orientación/signo del IMU queda validada. FND-021 continúa abierto por la calibración física de `par0/par1/perp` y los demás datos pendientes del contrato; no repetir esta matriz salvo cambio de montaje, Control Hub, configuración o código de orientación.

### Habilitación de tuners manuales de odometría — 2026-07-20

- El lead autoriza explícitamente habilitar los tuners manuales. `TuningOpModes.DISABLED_HAND_PUSH_TUNERS=false` registra únicamente `ForwardPushTest`, `LateralPushTest` y `DeadWheelDirectionDebugger`.
- `DISABLED_POWERED_TUNERS=true` permanece sin cambios: ramp loggers, feedforward, `LocalizationTest`, splines y demás tuners que pueden energizar el drivetrain continúan ocultos.
- Alcance autorizado: Fase B con el robot empujado/girado por una persona y motores sin comando. Primer gate: signos crudos de `par0=rightFront`, `par1=leftFront` y `perp=rightBack` mediante `DeadWheelDirectionDebugger`.
- APK no instalado: 81,315,819 bytes, SHA-256 `99D0CE4A96423AEE7BC73D31265071044FB00EFF85A682947308ED2DCDA1C4D3`, generado 2026-07-20 19:24:56; `:TeamCode:testDebugUnitTest` 32/32, `assembleDebug` y `git diff --check` PASS. El APK IMU `E586...DBB6` queda superseded para la siguiente sesión.

### Resultado de signos de pods — 2026-07-20

- Con el candidato `99D0...C4D3` y los tres flags inicialmente `false`, el lead reporta tres empujes forward: `par0=-12401/-12344/-12164`, `par1=-12677/-12263/-12243`, `perp=-14/+765/+286`; y tres empujes left: `par0=+638/+301/+551`, `par1=+559/+351/+163`, `perp=-12056/-11891/-12101`.
- Promedios del eje dominante: forward `par0=-12303`, forward `par1=-12394.3`, left `perp=-12016`. Los tres signos dominantes son opuestos a la convención Road Runner (forward positivo y left positivo); los valores cruzados menores no alteran esa conclusión.
- Candidato siguiente: `ThreeDeadWheelLocalizer.Params.par0Reversed/par1Reversed/perpReversed=true`. El espejo inactivo de Pedro queda `left/right/strafe=Encoder.REVERSE`. Escalas y offsets permanecen provisionales y Pedro no se activa.
- Gate siguiente de ese candidato: reinstalar y repetir un empuje forward y uno left en `DeadWheelDirectionDebugger`; los tres ejes dominantes deben aparecer positivos. Después ejecutar `ForwardPushTest` con distancia medida. `LateralPushTest` puede aparecer en el menú compartido, pero no aplica a dead wheels en Road Runner 0.1.23.
- APK de corrección de signos no instalado: 81,315,819 bytes, SHA-256 `C58E75CF52979C5B01B0285FA20A2EA996A0C08215FD678D73B2B8A7E6B85E7D`, generado 2026-07-20 19:59:57; 32/32 tests JVM, `assembleDebug` y `git diff --check` PASS. `99D0...C4D3` queda superseded.

### Retest de signos corregidos — 2026-07-20

- El lead reporta el spot-check con los tres flags en `true`: forward `par0=+12236`, `par1=+12165`, `perp=-838`; left `par0=-505`, `par1=-305`, `perp=+11964`.
- Los ejes dominantes cumplen la convención: forward produce `par0/par1` positivos y left produce `perp` positivo. Las lecturas cruzadas son menores que el eje dominante (máximo ≈6.9 %) y no contradicen el signo.
- Se cierra el subgate de dirección de `par0`, `par1` y `perp`. FND-021 continúa abierto por escala `inPerTick`, geometría/offsets y demás datos físicos pendientes; este resultado no cierra MP-02 ni el gate estadístico T5.
- Siguiente prueba: `ForwardPushTest`, empuje recto lento sobre una distancia física medida y cálculo `inPerTick = distancia real en pulgadas / ticks traveled`. No ejecutar `LateralPushTest`: Road Runner 0.1.23 lo limita a mecanum con encoders de drive y lanza error cuando detecta dead wheels.

### Escala forward de pods — 2026-07-20

- El lead confirma que las cuatro corridas forward anteriores y una quinta corrida corresponden al mismo recorrido medido de `60.0 cm` (`23.6220472441 in`). Para las tres corridas tomadas antes de invertir los flags se conserva la magnitud y se aplica el signo corregido; la inversión no cambia la escala.
- Promedios `par0/par1` por corrida: `12539`, `12303.5`, `12203.5`, `12200.5`, `12321.5 ticks`. Promedio global: `12313.6 ticks`; rango total: `338.5 ticks` (≈`2.75 %` del promedio).
- Cálculo: `23.6220472441 / 12313.6 = 0.00191837052073273 in/tick`. Sustituye el provisional `0.0019571295433364` en `MecanumDrive.Params.inPerTick` y `ThreeDeadWheelLocalizer.Params.inPerTick`.
- El espejo inactivo de Pedro actualiza sólo `TBD_FORWARD_TICKS_TO_INCHES` al mismo valor. `TBD_STRAFE_TICKS_TO_INCHES`, `TBD_TURN_TICKS_TO_INCHES` y los offsets no se infieren de esta prueba y permanecen pendientes.
- FND-021 sigue abierto por geometría/offsets y demás datos físicos pendientes; el subgate de escala forward queda medido, sujeto al retest de distancia estimada con el nuevo APK.
- APK de retest no instalado: 81,315,863 bytes, SHA-256 `9732D5027C4F3EBE7DE7100ECD3542CC99B37F90870F098872F4B2BD2976F445`, generado 2026-07-20 20:18:47; 32/32 tests JVM, `assembleDebug` y `git diff --check` PASS. El candidato de signos `C58E...5E7D` queda superseded.

### Prueba aislada field-centric — 2026-07-20

- El lead solicita validar el manejo field-centric antes de continuar con offsets. Se agrega `PRUEBA: FIELD CENTRIC CHASIS`, dueño únicamente de `DriveSubsystem`; no construye ni mapea intake, kicker, shooter o torreta.
- Controles gamepad 1: stick izquierdo Y forward/back, stick izquierdo X strafe, stick derecho X rotate. Los tres ejes se leen en el mismo ciclo, por lo que translación y giro pueden combinarse simultáneamente.
- `START` interrumpe el default command durante un ciclo, ordena cero y ejecuta `resetHeading()`: el frente físico actual pasa a ser heading `0°`. `BACK` conserva el E-stop global de `SafeCommandOpMode`.
- No se modificaron direcciones, signos, potencias ni la matemática del owner activo. Forward ya fue validado; strafe, rotate y las combinaciones quedan pendientes de prueba física controlada.
- APK no instalado: 81,340,063 bytes, SHA-256 `47706D0ECA1CCDB6ADCDC99C447FE70BD61D3C1C6B3EC66EC415B1674A574940`, generado 2026-07-20 20:28:02; 32/32 tests JVM, `assembleDebug` y `git diff --check` PASS. El candidato de escala `9732...F445` queda superseded.

### Hallazgo y corrección de field-centric a ±90° — 2026-07-20

- Prueba física del lead bajo `4770...4940`: reset heading `0.0°`, forward/back, strafe y rotate individuales correctos a heading cero. Después de girar `90°` a derecha o izquierda, forward/strafe field-centric ordenaron el lado contrario. Las combinaciones, release y BACK no se califican con ese candidato.
- Causa raíz: `SkywalkerProfile`/FTCLib expresan strafe con derecha positiva y el IMU expresa antihorario positivo/horario negativo, pero `FieldCentricDriveCommand` rotaba esa representación por `-heading`, fórmula correspondiente a la convención lateral opuesta. El fallo no estaba en direcciones de motores, mappings, IMU ni reset.
- Prueba de regresión añadida para `0°`, `-90°` horario y `+90°` antihorario. Antes del fix: 4 casos, 3 fallos; sólo `0°` pasaba, reproduciendo el robot. Cambio único: la transformación right-positive usa `+heading`; no cambian joystick, motor directions, giro, potencia, localizador, reset ni Stop/E-stop.
- El test focalizado pasa 4/4 después del cambio. Retest físico obligatorio: ambos `±90°`, forward, strafe, forward+rotate, strafe+rotate, release y BACK.
- APK corregido no instalado: 81,317,247 bytes, SHA-256 `12B7E8C3A6BC924D1DBCAED74A17ECFF590D2EDCA1F49537A8C9846A854FC296`, generado 2026-07-20 20:43:05; suite completa 36/36, `assembleDebug` y `git diff --check` PASS. `4770...4940` queda superseded y no debe repetirse.
- El lead confirma el retest físico del APK corregido con “ya funciona” y ordena volver a Road Runner. Se acepta la corrección de la transformación field-centric a heading distinto de cero. No se inventan valores de heading, tiempos ni resultados individuales de release/BACK que no fueron reportados.
- Próximo frente Road Runner: medir físicamente los puntos de contacto de `par0`, `par1` y `perp` respecto al centro de giro; después convertir pulgadas a ticks con `inPerTick=0.00191837052073273`. Los tuners motorizados permanecen bloqueados.

### Candidatos físicos de geometría de pods — 2026-07-20

- El lead mide `par0=171.45 mm` a la derecha, `par1=171.45 mm` a la izquierda y `perp=177.8 mm` detrás del centro. Conversiones exactas reportadas: `6.75 in`, `6.75 in` y `7.00 in`.
- Road Runner: por la ecuación instalada y los encoders forward-positive, el eje efectivo de estos parámetros es derecha positiva: `par0YTicks=+3518.6112`, `par1YTicks=-3518.6112`; X es frente positivo, por lo que `perpXTicks=-3648.93013333333`. La separación paralela candidata es `7037.2224 ticks` (`13.5 in`).
- Pedro inactivo conserva su convención propia: left `+6.75 in`, right `-6.75 in`, strafe X `-7.0 in`. `LOCALIZER_OFFSETS_CALIBRATED=false` permanece hasta validación; no se activa Pedro.
- Son semillas geométricas físicas, no el resultado final de regresión. Siguiente gate: comparar heading de pose de tres pods contra IMU en giros controlados CW/CCW; después `AngularRampLogger` refina posiciones cuando se autoricen los tuners motorizados.
- Chequeo algebraico con giro CCW ideal: heading RR `+1 rad`, traslación lateral falsa ≈`-4.55e-13 ticks` (cero numérico) y denominador `7037.2224 ticks` no nulo.
- APK no instalado: 81,317,259 bytes, SHA-256 `C5556DDED71A095AADBA8687C41F244B671CFBDB1C8FE2A3E421601D8B7AA0B4`, generado 2026-07-20 21:07:59; 36/36 tests JVM, `assembleDebug` y `git diff --check` PASS. El APK field-centric `12B7...C296` queda superseded.

### Autorización de `ForwardRampLogger` — 2026-07-20

- El lead autorizó continuar con el siguiente tuner motorizado oficial de Road Runner. Se expone únicamente `ForwardRampLogger`; `LateralRampLogger`, `AngularRampLogger`, feedforward/feedback, spline, `LocalizationTest`, depurador de motores y OTOS permanecen ocultos.
- La implementación oficial de Road Runner FTC 0.1.23 incrementa la potencia forward a razón de `0.1/s` hasta un máximo constante de `0.9`. No termina por tiempo o distancia: el operador debe pulsar Stop antes del borde u obstáculo.
- Se verificó en el bytecode de la dependencia instalada que, al salir de `opModeIsActive()`, el OpMode manda `setPower(0)` a todos los motores antes de guardar el archivo `FORWARD_RAMP`. Esto no sustituye la prueba física ni la responsabilidad de conservar acceso inmediato a Stop.
- Criterio de esta etapa: recorrido recto libre, sin piezas ni personas en la trayectoria; ejecutar una sola corrida y revisar el registro en `/tuning/forward-ramp.html` antes de habilitar cualquier otro tuner motorizado.
- Candidato no instalado: 81,317,511 bytes, SHA-256 `97CEBA4FA2BC387D5588611ACA1721BFCA7F191ADB3EE90C68F3BCD7953980AE`, generado 2026-07-20 21:57:31; 36/36 tests JVM, `assembleDebug` y `git diff --check` PASS. El APK de geometría `C555...0B4` queda superseded.

### Cierre de `ForwardRampLogger` y autorización de `LateralRampLogger` — 2026-07-21

- El lead entregó tres corridas sin asistencia manual, con batería reportada en `13.25 V`. Archivos: `forward-ramp-1784659085251.json` (`DDFD5A...E581C`), `forward-ramp-1784659035964.json` (`26F174...475B`) y `forward-ramp-2.json` (`660D88...026`). Las corridas con ayuda manual anteriores quedan excluidas por decisión expresa del lead.
- Las tres corridas duran `6.87–7.02 s`, alcanzan potencia `0.69–0.70` y registran batería bajo carga de aproximadamente `13.24` a `12.46–12.53 V`. Después de excluir únicamente el tramo inmóvil de arranque, las dos corridas más lineales coinciden y la mediana robusta de las tres produce `kS=2.48 V` y `kV=0.000189 V/(tick/s)`; esos valores sustituyen los anteriores `2.2724850864058084` y `0.00018662940312018166`.
- Limitación aceptada por el lead por falta de tiempo: `par1` recorrió consistentemente alrededor de `6–7 %` más que `par0`, equivalente con la geometría candidata a aproximadamente `-40.7°/-45.5°/-49.1°` de giro horario por corrida. El robot se desvía a la derecha; se atribuye tentativamente a carga desigual y una rueda irregular, pero no se declara causa raíz demostrada ni se añade compensación artificial al feedforward.
- El lead autoriza pasar a la siguiente etapa. Se expone `LateralRampLogger` además de conservar `ForwardRampLogger`; `AngularRampLogger`, feedforward/feedback, spline, `LocalizationTest`, depurador individual de motores y OTOS permanecen ocultos.
- `LateralRampLogger` se desplaza automáticamente hacia la izquierda, sube `0.1` de potencia por segundo hasta `0.9` y, según el bytecode de Road Runner FTC 0.1.23, ordena potencia cero a los cuatro motores al recibir Stop. Debe ejecutarse sin contacto humano, con recorrido lateral libre y Stop antes del borde.
- Candidato no instalado: 81,317,627 bytes, SHA-256 `B57768DB597A2D6BF82A8FF36973AD3048245FF552FE7DAAA39091DDB4AE1B74`, generado 2026-07-21 12:54:55; 36/36 tests JVM, `assembleDebug` y `git diff --check` PASS. El APK de ForwardRamp `97CE...80AE` queda superseded.

### Cierre de `LateralRampLogger` y autorización de `AngularRampLogger` — 2026-07-21

- El operador introdujo correctamente en la página lateral los valores activos: `inPerTick=0.00191837052...`, `kV=0.000189` y `kS=2.48`. El archivo `lateral-ramp-1784663471110.json` tiene SHA-256 `B5282B4C524674F7067B0E47734F5FC29BFE5F3FBBB12F0EE5B585C13E702884`.
- La corrida dura aproximadamente `9.30 s`, alcanza potencia `0.90`, registra batería bajo carga `12.70→11.06 V` y velocidad perpendicular máxima `25000 ticks/s`. La meseta inicial observada es una limitación común del modelo mecanum descrita por Road Runner, no un error de los campos introducidos.
- Se reprodujo el algoritmo exacto de `assets/web` incluido en Road Runner FTC 0.1.23: regresión por el origen entre velocidad esperada de feedforward y velocidad perpendicular real. Resultado `lateralInPerTick=0.0010034879603150176 in/tick`, que sustituye `0.0011569903073521382`.
- Conforme a la orden previa del lead de aplicar valores y avanzar, se expone únicamente el siguiente tuner adicional, `AngularRampLogger`. Éste gira automáticamente en sentido antihorario, eleva potencia `0.1/s` hasta `0.9` y ordena cero a los cuatro motores al recibir Stop. Feedback, spline, `LocalizationTest`, depurador individual y OTOS permanecen ocultos.
- Candidato no instalado: 81,317,743 bytes, SHA-256 `B264C18C79BC593D332460B9C1F0D57A5BF53CF613448BB131F8C687464EC8B9`, generado 2026-07-21 13:56:10; 36/36 tests JVM, `assembleDebug` y `git diff --check` PASS. El APK lateral `B577...1B74` queda superseded.

### Cierre de `AngularRampLogger` — 2026-07-21

- Archivo recibido: `angular-ramp-1784664449328.json`, SHA-256 `F811B97CEA69FE9748FCCAA0D51DBC986AE08FA38DC7AE8523DE727702D0DAF3`. La corrida dura aproximadamente `10.78 s`, alcanza potencia `0.90`, registra batería `12.72→11.80 V` y velocidad angular máxima `7.106 rad/s`; el eje dominante es correctamente `+Z`.
- Regresiones por el origen reproducidas con el algoritmo de Road Runner FTC 0.1.23: `par0YTicks=+3174.0795443819798` (`R²=0.9980`), `par1YTicks=-3619.1632183254724` (`R²=0.9987`) y `perpXTicks=-2330.341135601611` (`R²=0.9971`). Sustituyen las semillas físicas `+3518.6112/-3518.6112/-3648.9301`; la diferencia, especialmente en `perp`, queda registrada como calibración empírica y no como cambio de la medición física reportada.
- `trackWidthTicks=6250.5090234054815` (`11.99 in` con el `inPerTick` activo) sustituye `7290.158084947854`. Se comprobó contra la ecuación de `MecanumKinematics` 1.0.1, donde en giro puro `wheelSpeed=omega*trackWidth`; no se aplica un factor adicional de dos.
- El siguiente paso oficial es `ManualFeedforwardTuner`, pero la clase upstream FTC 0.1.23 retorna al recibir Stop sin ordenar explícitamente cero mediante `DriveView`. Permanece oculta hasta añadir/revisar una variante con paro explícito; no se abre el resto del menú. `AngularRampLogger` sí conserva su cero explícito verificado.
- Candidato no instalado: 81,317,747 bytes, SHA-256 `D2ECE3A869FE4E2EAA24D9E265BD0CAC0226D479156A6E550ED9270849CA4CAF`, generado 2026-07-21 14:14:21; 36/36 tests JVM, `assembleDebug` y `git diff --check` PASS. El APK que sólo habilitaba AngularRamp `B264...C8B9` queda superseded.

### Habilitacion de `SafeManualFeedforwardTuner` — 2026-07-21

- Se implementa una variante local equivalente al `ManualFeedforwardTuner` de Road Runner FTC 0.1.23. Conserva el perfil automatico de `64 in`, la telemetria `vref/v0...`, `Y` para pasar a manejo manual y `B` para reanudar el perfil.
- Diferencia de seguridad: establece potencia cero antes de Start, al cambiar entre modos, ante voltaje invalido y dentro de un bloque `finally`, por lo que Stop o una excepcion ordenan cero a los cuatro motores.
- `MecanumDrive.Params.kA` baja de `0.0001` a `0.0000001` como punto inicial recomendado para esta etapa. `kS=2.48` y `kV=0.000189` no cambian.
- Se ocultan los tres ramp loggers ya cerrados. Queda habilitado solo `SafeManualFeedforwardTuner` como siguiente etapa motorizada; feedback, spline, `LocalizationTest`, depurador individual y OTOS siguen bloqueados.
- Prueba pendiente: instalar el APK nuevo, disponer de al menos `64 in` de recorrido mas zona de frenado en ambos sentidos, conservar acceso inmediato a Stop y comparar `vref` contra las velocidades medidas en FTC Dashboard. No aplicar un `kA` nuevo sin revisar la grafica.
- APK no instalado: 81,320,939 bytes, SHA-256 `87E49A1E7AA972936563D2C9BC0B2C15EB097EAF9B055A37BB62E6564A948912`, generado 2026-07-21 14:24:05; 36/36 tests JVM y `assembleDebug` PASS. El APK angular `D2EC...C4CAF` queda superseded.

### Cierre de feedforward y habilitacion de `ManualFeedbackTuner` — 2026-07-21

- Pruebas comparativas en FTC Dashboard: `kA=1e-7`, `1e-6` y `1e-5` conservaron retraso visible de `v0/v1` respecto a `vref`. Con `kA=0.0001`, ambas velocidades siguieron de cerca la referencia durante aceleracion y frenado; `v0` y `v1` permanecieron proximas entre si.
- Se fija `MecanumDrive.Params.kA=0.0001`. Permanecen `kS=2.48` y `kV=0.000189`.
- Se oculta `SafeManualFeedforwardTuner` y se habilita solamente `ManualFeedbackTuner` como siguiente etapa. La clase local ahora establece cero antes de Start y en un bloque `finally`, cubriendo Stop o excepcion aunque `Actions.runBlocking()` termine por interrupcion.
- Prueba fisica pendiente: recorrido automatico `0 -> 64 in -> 0`, revisar `xError`, `yError` y `headingError (deg)` en Dashboard. No cambiar ganancias antes de la primera corrida.
- APK no instalado: 81,321,091 bytes, SHA-256 `EC9D8C318BF508C71F07E16218B2E991463745CD435251D2D6D8C194501EE1A5`, generado 2026-07-21 15:44:05; 36/36 tests JVM, `assembleDebug` y `git diff --check` PASS. El APK de feedforward `87E4...8912` queda superseded.

### Diagnostico de rueda `leftFront` durante feedback — 2026-07-21

- Con bateria cargada, `yError` bajo de aproximadamente `2–5 in` a cerca de `±1 in`; el desvio lateral grande anterior estaba contaminado por caida de bateria. Con `headingGain=12` aplicado solo en Dashboard, el operador reporta que `leftFront` se detiene bruscamente al frenar hacia adelante y la correccion heading hacia atras se vuelve mas fuerte.
- No se fija `headingGain=12` ni se cambia otra ganancia. Se agregan a la telemetria de `FollowTrajectoryAction` los comandos efectivos `lfPower/lbPower/rfPower/rbPower` leidos despues de `setPower`, sin modificar control, limites ni potencia.
- Gate: repetir una sola ida y vuelta con `headingGain=12` y graficar las cuatro potencias mas `headingError (deg)`. Si `lfPower` tambien cae a cero antes que las demas, el origen esta en la mezcla del controlador; si conserva una orden comparable mientras la rueda se frena, revisar rueda, transmision, motor, cableado y hub antes de seguir ajustando ganancias.
- APK diagnostico no instalado: 81,321,191 bytes, SHA-256 `E57FDB4185C2166573D5A17869FE1BEDE8EC4E18F99C29EB778FE252126A34C4`, generado 2026-07-21 16:17:27; 36/36 tests JVM, `assembleDebug` y `git diff --check` PASS. El APK `EC9D...E1A5` queda superseded.

### Cierre pragmático de feedback y habilitación de `SplineTest` — 2026-07-21

- Por decisión del lead y falta de tiempo se fija la mejor combinación física probada: `maxWheelVel=60 in/s`, `minProfileAccel=-30 in/s²`, `maxProfileAccel=55 in/s²`, `axialGain=8`, `lateralGain=8`, `headingGain=12` y ganancias de velocidad en cero. El frenado brusco de `leftFront` quedó resuelto con el perfil de desaceleración más suave.
- En tres ciclos de `ManualFeedbackTuner`, el chasis terminó recto y el heading final quedó cerca de cero, pero el robot acumuló aproximadamente `14 cm` de strafe a la izquierda. Dashboard reportó un error lateral mucho menor; por ello no se declara resuelta la causa ni se aumenta más `lateralGain`. Se conserva como limitación conocida de localización/contacto del pod perpendicular o deriva mecánica pendiente.
- `ManualFeedbackTuner` queda oculto y se habilita únicamente `SplineTest` como siguiente gate de Road Runner. La variante local ahora ordena potencia cero a los cuatro motores en `finally`, incluso ante Stop o excepción.
- Próxima prueba: área libre mínima de `60 x 30 in` más margen de frenado; colocar el robot en `(0,0,0)`, ejecutar una sola vez y observar la trayectoria `splineTo(30,30,90°)` seguida de `splineTo(0,60,180°)`. Detener ante salida clara de trayectoria, pod sin contacto o rumbo divergente.
- Verificación software del candidato: 36/36 pruebas JVM, `assembleDebug` y `git diff --check` PASS. APK no instalado: 81,321,523 bytes, SHA-256 `9D31945960AFCE70701CE7ABE1BCB2F575BF504AD1DAA55448970C23A9DC009F`.

### Aceptación física de `SplineTest` y cierre del tuneo base de Road Runner — 2026-07-21

- El lead reporta que terminó “súper cerca” del punto final, con orientación correcta. El movimiento fue suave salvo una transición algo brusca a mitad de ruta, calificada como aceptable y “muy bien”.
- La captura aportada corresponde a la trayectoria todavía en ejecución (`heading≈144.5°` y potencias no nulas), no al estado final; por ello no se inventa error final numérico. La aceptación se basa en la observación física explícita del lead.
- Se acepta el tuneo base de Road Runner para avanzar a construcción y validación de trayectorias reales. Permanece documentada la deriva lateral acumulada de aproximadamente `14 cm` en tres ciclos rectos; este cierre pragmático no declara resuelta su causa.
- `SplineTest` queda nuevamente oculto en el candidato base. No se modifican los valores finales de drive aceptados ni se habilitan otros tuners motorizados.
- APK base final no instalado: 81,321,455 bytes, SHA-256 `B4D447DD43BBD4FD518FF593B22A090E8A67E6580BB57FDBA94323875B16C190`; 36/36 pruebas JVM, `assembleDebug` y `git diff --check` PASS.

### Ajuste de manejo Pedro en produccion — 2026-07-21

- El lead reporta en `MainTeleOp`: traslacion y strafe demasiado lentos, giro manual invertido y giro a stick completo sin traslacion simultanea. Solicita techo de potencia `0.90` y `START` para reset field-centric.
- Causa raiz de la mezcla: Pedro 2.1.2 limita por `maxPowerScaling` y prioriza el vector de heading antes del vector de traslacion. Con giro manual `1.0` y techo anterior `0.50`, el giro consumia el margen completo y Pedro reducia la traslacion casi a cero.
- Se fija el techo global Pedro en `0.90` para forward, backward y strafe. El giro manual se invierte y limita a `0.70`: stick derecho a derecha ordena giro horario, y aun a fondo conserva `0.20` del margen global para combinar traslacion. No cambian direcciones de motores, odometria, limites de mecanismos ni Stop/E-stop.
- El binding existente de `gamepad1 START` se conserva con requisito exclusivo del drivetrain: interrumpe un ciclo, ordena cero y convierte el heading fisico actual en `0`. Se agrega `Heading resets (START)` a telemetria para confirmar que el evento fue recibido.
- Verificacion software: 40/40 pruebas JVM, `assembleDebug` y `git diff --check` PASS. APK no instalado: 81,332,051 bytes, SHA-256 `99EA6E6BD9F6F605C05084778D22686F52F065F48A13B355B275A0D49A67E810`.
- Pendiente fisico: probar primero elevado/restringido; luego forward/back, strafe, giro en ambos sentidos, forward+giro, strafe+giro, reset con START, release y BACK. La potencia `0.90` no se declara validada fisicamente hasta ese retest.
- Retest parcial del lead: potencia, strafe, signo de giro y mezcla traslacion+giro PASS; `START` no cambio el heading. Causa raiz en software: el adaptador usaba `Follower.setStartingPose()` durante TeleOp, aunque Pedro 2.1.2 documenta que no debe llamarse despues de mover el robot. Se separan los caminos: construccion usa `setStartingPose()` y el reset activo usa `setPose()`, conservando X/Y y fijando heading `0`. Queda pendiente repetir solamente reset, release y BACK con el APK corregido.
- APK de reset corregido no instalado: 81,332,195 bytes, SHA-256 `F5B62197A557F77D768D9972246E46FF636C96FF16E9B8DBF899FC8650A0FE7A`; 40/40 pruebas JVM y `assembleDebug` PASS. Sustituye el candidato `99EA...E810`.
- El lead confirma masa total del robot de `8.5 kg`. Pedro cambia su placeholder anterior de `5 kg` a la constante explicita `ROBOT_MASS_KG=8.5`; esta masa alimenta la correccion centripeta y no modifica el techo manual de potencia `0.90`.
- APK con masa corregida no instalado: 81,356,132 bytes, SHA-256 `9287337811E98428E46DA81FF0A6526DE3FDFA33990ADC26BE41FE80F32A7362`; 40/40 pruebas JVM y `assembleDebug` PASS. Sustituye `F5B6...FE7A`.

### Cierre físico MP-02 — 2026-07-21

- El lead instaló y probó el candidato final con masa `8.5 kg`. Spot-check de producción: `START reset=PASS`, `release stop=PASS`, `BACK E-stop=PASS` y `forward+turn=PASS`.
- Esta evidencia completa el gate pendiente posterior al cambio atómico de ownership. Junto con T5 `40/40`, direcciones `9/9`, owner único Pedro y verificación software `40/40`, MP-02 cambia a `ACCEPTED`.
- Se cierran FND-004 y FND-017. No se repite la matriz T5 mientras no cambien geometría de pods, signos, escalas, masa, ownership, direcciones o rutas de stop.
- Siguiente fase: MP-03, inventario/configuración y calibración de visión Limelight; no se habilita auto-aim ni alimentación automática por cerrar únicamente MP-02.

## Lo que se espera del usuario en pruebas reales

### Apertura MP-03 — 2026-07-21

- Se verificó directamente contra las fuentes locales del artefacto FTC
  `Hardware:10.3.0` que `Limelight3A` ya forma parte del SDK; no se agregó
  dependencia externa.
- El wrapper previo se endureció sin conectarlo a pose ni actuadores: estado de
  conexión/pipeline, timestamp real del resultado en Control Hub, familia, área
  y conteo de fiduciales; rechazo por desconexión, pipeline incorrecto, metadata
  incompleta o geometría físicamente imposible. `getBotpose()` sólo se publica
  cuando `getBotposeTagCount() > 0`.
- `Limelight Diagnostic` sigue siendo el único punto de commissioning y no
  construye mecanismos ni drivetrain. Se creó
  [mp03-limelight-commissioning.md](mp03-limelight-commissioning.md) para mapping,
  versiones, exportables, extrínseca y gate repetido init/stop/desconexión.
- Estado: `IN_PROGRESS / BLOCKED_PHYSICAL_INPUT`. MP-04/MP-05 y todo movimiento
  por visión siguen inhibidos.
- Verificación software del candidato: `:TeamCode:testDebugUnitTest` 41/41,
  cero failures/errors/skips; `:TeamCode:assembleDebug` y `git diff --check`
  PASS. APK no instalado ni probado físicamente: 81,419,928 bytes, SHA-256
  `59BBA3F1372AC46B88910711B21A0A0A3C0E4A39CE606A08AF75A33321318982`,
  generado 2026-07-21 22:31:20. No promover ni conectar consumidores hasta
  completar la hoja y el gate físico MP-03.
- Primera evidencia física MP-03 aportada por el lead: dispositivo detectado
  inicialmente como `Ethernet Device`, renombrado exactamente a `limelight`,
  configuración guardada/activada y cable confirmado en el puerto USB-A azul
  del Control Hub. La pantalla sólo mostró nombre y una dirección IP, que no se
  registra. Montaje rígido confirmado; IDs planeados azul/rojo `20`/`24`.
- Segunda evidencia MP-03 mediante la UI web: `Limelight 3A 2026.0`, pipeline
  `0`/`Pipeline_Name` de tipo `AprilTags`, resolución `640x480 90fps` y detección
  visible del tag rojo `24`. La muestra reportó `tx=-20.66°`, `ty=-8.36°`,
  `ta=1.605%`, `tl=12.5 ms`; se registra sólo como diagnóstico puntual, no como
  calibración ni validación de extrínseca. Hardware Manager no fue necesario.
- Tercera evidencia MP-03: familia `AprilTag Classic 36h11 (587 tags)`, engine
  `U-Michigan`, marker size configurado `101.6 mm`, downscale `2`, quality
  threshold `2` e ID filter vacío. No se modificó ningún valor. El marker size
  queda pendiente de medir sobre el cuadrado negro físico; la detección del ID
  por sí sola no valida la escala de pose 3D.
- Cuarta evidencia MP-03: el lead midió el cuadrado negro del tag de práctica
  `24` como `160 x 160 mm`. La producción oficial DECODE exige `6.5 in =
  165.1 mm` para ese cuadrado; el manual TU32 confirma target completo de
  `8.125 in` e IDs goal azul/rojo `20`/`24`. El `Marker Size=101.6 mm` del
  pipeline queda identificado como incorrecto para el GOAL. No se cambia aún:
  primero descargar/versionar el pipeline original; después usar `165.1 mm` y
  evitar el tag de práctica subescalado para validar distancia/pose 3D.
- Respaldo original recibido y versionado como
  `docs/vision/limelight/pipeline-0-original-2026-07-21.vpr`: 1,992 bytes,
  SHA-256 `F5C48FF047E17D6CC02551122E51957218DA37D7E85C7CF24B50E0CDD0100994`.
  La inspección confirma `fiducial_size=101.6`, familia 36h11, pipeline fiducial
  y `fiducial_skip3d=1`; no contiene IP ni secretos. Ya existe rollback antes de
  corregir marker size.
- Quinta evidencia MP-03: después del respaldo, el lead corrigió `Marker Size`
  a `165.1 mm`. La UI continuó detectando el tag `24`; muestra posterior
  `tx=-20.75°`, `ty=-8.44°`, `ta=1.600%`, `tl=16.4 ms`. Esto confirma detección,
  no exactitud de distancia/pose 3D, porque el tag de práctica mide `160 mm` y el
  respaldo conserva `fiducial_skip3d=1`.
- Pipeline corregido recibido y versionado como
  `docs/vision/limelight/pipeline-0-decode-165.1mm-2026-07-21.vpr`: 1,992 bytes,
  SHA-256 `0B954CFA8641B460C13E8DF65A7A4691E7FD409821A09A008BF3ADA141D20693`.
  La comparación campo por campo con el original encontró una sola diferencia:
  `fiducial_size=101.6` → `165.1`. `fiducial_skip3d=1` permanece; no se declara
  pose 3D habilitada ni validada.

Antes de habilitar: robot elevado o mecanismos desacoplados, área despejada, cámara cubierta/ausente según la configuración real y acceso inmediato a Stop. Registrar responsable, fecha, SHA exacto, versiones RC/DS y video o cronometraje cuando aplique.

1. Mapping completado por atestación directa en Driver Station; adjuntar capturas o export si la interfaz los permite posteriormente.
2. Ejecutar 20 inicializaciones sin chord: cero armados y cero movimiento.
3. Ejecutar 20 holds menores de 1 s: todos regresan a `WAITING`, sin reset.
4. Ejecutar 20 holds de al menos 1 s: transición a `ARMED` y un solo reset por inicialización.
5. Pulsar Start sin armar en 10 repeticiones: torreta en cero durante toda la ejecución.
6. Probar `gamepad1 BACK` 10 veces durante init y 10 después de Start: actuadores en cero en el siguiente ciclo y objetivo ≤50 ms, con evidencia temporal.
7. Ejecutar 20 ciclos init/stop sin webcams: no debe existir lookup, excepción ni recurso abierto.
8. Verificar el `kickerMotor` final sin servo: ON/OFF, Stop y E-stop ordenan cero en el siguiente ciclo y ≤50 ms; confirmar que `Kicker/Servo active=false`.
9. Inyectar voltaje inválido, encoder congelado/lectura imposible y overspeed de forma controlada: target/power quedan en cero y sólo reiniciar el OpMode elimina el fault.
10. No seleccionar los OpModes `@Disabled` ni energizar shooter/torreta antes de sus gates físicos.

## Geometría de torreta y autónomos Pedro Pathing — 2026-07-22

- El equipo entregó geometría corregida para `geometria-robot-mp04.md`: los cinco
  centros ópticos de la Limelight y el punto `T` de la torreta subieron
  `+10.79 mm` en Z respecto a la entrega del 2026-07-21 (corrección de plano de
  referencia, no de signos); el signo de `T0` quedó alineado con `T`; se
  completó la sección 3 (shooter en cero) y se verificó por cálculo que
  `S-T`, `ρ`, `h` y el punto auxiliar `D` son consistentes entre sí.
- **Sección 4 (límites de giro) cerrada**, sin sesión física dedicada: el
  equipo decidió reusar el límite de software ya aceptado (`-983/+1070` ticks,
  `TurretSubsystem`) en vez de medir de nuevo. La magnitud se validó de memoria
  contra el cálculo de PPR/engranes (`+1070` ticks ≈ 176°, casi mirando hacia
  atrás, coincide con el recuerdo del equipo de "gira 180° aprox"), y el
  sentido se confirmó con la posición física del motor/cable (montados a la
  derecha del robot): girar hacia `+1070` es yaw **negativo** bajo la
  convención de esta hoja, no positivo. Rango final: yaw `[-175.97°, +161.66°]`.
  Queda abierto, sin bloquear el uso del rango: el código no distingue tope
  mecánico de tope por cableado dentro de ese mismo par de ticks.
- El equipo confirmó que la configuración de Limelight físicamente montada
  ahora mismo es `30°` (de las cinco discretas documentadas), cerrando ese
  pendiente para elegir la fila correcta de extrínseca.
- **DEC-041** revierte la decisión "cero autónomos en el artefacto final de
  competencia" (`plan-maestro-robot.md` §1, MP-09): el equipo ya escribió 10
  autónomos con Pedro Pathing y confirmó que sí van a competencia, a cambio de
  retirar los autónomos Road Runner legado (ya no compilaban contra el
  `RobotContainer` Pedro-only). Ver `decisiones.md` DEC-041 para el detalle
  completo; `plan-maestro-robot.md`, `06-limpieza-y-release.md` y
  `plan-paralelo-20h.md` §3 ya quedaron actualizados con la nueva postura.
- Trabajo de código de esta sesión (sin robot físico, sólo software):
  - Borrados los 8 autónomos Road Runner (`AutonomoBetaPosition`,
    `AutonomoOfficialBlue`, `AutonomoOfficialRed`/`Red2`, `FullOfficialBlue2`,
    `FullOfficialRed2`, `FullOficialBlue`, `FullOficialRed`); `TestShootBurstAuto`
    se conserva (no mueve drivetrain).
  - Infraestructura Pedro-autónomo nueva: `commands/PedroPathCommand.java`
    (equivalente a `ActionCommand` pero para `PathChain`/`Follower`);
    `pathBuilder()`/`followPath()`/`isBusy()` expuestos como passthrough en
    `PedroDriveAdapter`/`PedroDriveSubsystem`; `PoseStorage.recordAutonomousResult(...)`
    para convertir la pose final de Pedro a `Pose2d` y que `MainTeleOp` siga
    arrancando desde ahí; `LowAltitudeConstants.AutoConstants` con las poses
    iniciales reales por alianza (derivadas del primer waypoint real de cada
    path, no del placeholder `(72, 8, 90°)` que traían los 10 templates).
  - Los 10 autónomos del equipo (`2/1 Artifacts Red/Blue Goal/Full`, `Leave
    Full`, `Leave Goal`) reescritos en `opmodes/auto/` con nombre de clase
    único, `SafeAutonomousOpMode` (E-stop/`RobotSafety` de fábrica) y starting
    pose corregida; mismos waypoints exactos del export del equipo.
  - Sin resolver a propósito, no inventado: la lógica de disparo
    (`autonomousPathUpdate()` venía vacío en los 10 templates — cada auto
    nuevo tiene un `TODO(equipo)` marcando dónde integrar `ShootBurstCommand`);
    la asimetría de heading 45°/55° entre "1 Artifact Red Full" y "1 Artifact
    Blue Full" (transcrita tal cual, con comentario); la starting pose y
    alianza de `LeaveGoal` (su path arranca en `(61, 75.682)`, muy distinto a
    los otros 9 — el archivo compila con placeholders marcados
    `_UNCONFIRMED`/`_PLACEHOLDER`, no con valores dados por buenos).
- Verificación software: `:TeamCode:assembleDebug` y `:TeamCode:testDebugUnitTest`
  PASS con los 10 autos nuevos + la infraestructura agregada. No se instaló ni
  probó nada físicamente — todo este trabajo es de la Pista Software, sin robot.

## Pista Software sin robot — FND-018/FND-019 y preguntas pendientes al equipo — 2026-07-22

Sesión de software puro (sin robot físico), a partir de revisar qué de lo que hoy se
trata como "bloqueado por hardware" en `hallazgos.md`/`plan-maestro-robot.md` en
realidad no lo está.

- **FND-019 (readiness omite velocidad del chasis) contenido en software.** El dato ya
  existía: `PedroDriveAdapter` calcula `vx`/`vy`/`omega` robot-céntricos hacia cada
  `PoseSnapshot` desde MP-02, pero nada los leía antes de disparar. Nueva clase pura
  `safety/ChassisMotionGate.isWithinShotEnvelope(PoseSnapshot)` implementa DEC-031
  (feeder solo con robot estacionario), fail-closed ante snapshot nulo/no usable.
  Cableada en el binding RB de `SkywalkerProfile.java` (junto a `shooter.isReady()`) y
  como constructor opcional nuevo de `ShootBurstCommand` para cuando el equipo confirme
  dónde disparan los autos. Umbral `MAX_SHOT_LINEAR_SPEED_INCHES_PER_SEC=1.0`/
  `MAX_SHOT_ANGULAR_SPEED_RADIANS_PER_SEC≈0.087 rad` en `LowAltitudeConstants`,
  explícitamente `TBD-BLOCKING`, conservador. Test nuevo `ChassisMotionGateTest`.
- **FND-018 (feeder sin pulso máximo/cooldown) contenido en software.** Nueva clase
  `safety/FeederPulseStateMachine.java` (mismo patrón testable con `nowNanos` explícito
  que `TurretArmingStateMachine`): `IDLE -> PULSING -> COOLDOWN`. `KickerSubsystem.kick()`
  ahora corta sola al llegar a `KICKER_MAX_PULSE_MS`, aunque el binding `whileHeld` de
  `SkywalkerProfile` la siga pidiendo, y bloquea reenergizar hasta completar
  `KICKER_COOLDOWN_MS`. El readiness (`shooter.isReady()`) se queda deliberadamente
  fuera de esta máquina — sigue siendo responsabilidad del caller, para no mezclar
  ownership de shooter y feeder (FND-008, no reabierto). `KICKER_MAX_PULSE_MS=700`/
  `KICKER_COOLDOWN_MS=300`, `TBD-BLOCKING`, conservadores. Test nuevo
  `FeederPulseStateMachineTest`.
- **FND-009: grafo de dependencias efectivo documentado**, sin cambiar ninguna
  versión declarada (DEC-022). `:TeamCode:dependencies --configuration
  debugCompileClasspath` confirma que `11.0.0` gana siempre sobre las líneas
  `10.3.0` explícitas de `TeamCode/build.gradle` (hay constraints
  `{strictly 11.0.0}`). Detalle en `hallazgos.md` FND-009.
- Verificación software: `:TeamCode:testDebugUnitTest` y `:TeamCode:assembleDebug`
  PASS con las dos clases nuevas + sus tests. Nada de esto mueve actuadores ni
  reemplaza los gates físicos T8/T9/MP-08 pendientes — sólo evita que Tuning tenga
  que construir la mecánica de estos dos findings además de medir sus números.

### Preguntas pendientes al equipo — autónomos Pedro Pathing (bloqueo por decisión, no por robot)

Los 8 `TODO(equipo)` de disparo y las 2 discrepancias geométricas de la sesión anterior
(2026-07-22, "Geometría de torreta y autónomos Pedro Pathing") no requieren una sesión
física — requieren que el equipo responda estas preguntas puntuales:

1. **Punto de disparo por auto.** Para cada auto, los candidatos naturales son los
   puntos donde el `PathChain` termina un tramo mirando hacia el gol (heading final de
   cada `BezierCurve`/`BezierLine`, no un punto intermedio arbitrario):
   - `1 Artifact Red/Blue Goal`: un solo tramo termina en `(61.5, 79.0)` / `(80.0, 79.0)`
     a heading `135°` / `45°` — ¿disparan ahí, al final del chain, o antes?
   - `1 Artifact Red/Blue Full`: termina en `(55.94, 7.483)` / `(85.56, 7.483)` a
     heading `135°` / `55°` (ver punto 2, este heading está bajo sospecha) — mismo tipo
     de pregunta.
   - `2 Artifacts Red/Blue Goal` y `2 Artifacts Red/Blue Full`: el chain tiene DOS
     retornos hacia el gol (un artefacto cada vez): el primero a mitad del chain
     (p.ej. Red Goal en `(61.5, 79.0)` heading `135°`) y el segundo al final
     (Red Goal en `(58.176, 82.839)` heading `135°`). ¿Disparan uno después de cada
     retorno (2 `ShootBurstCommand` de 1 pieza) o los 2 juntos al final?
2. **Asimetría 45°/55°.** Los 4 autos "Goal" mirroran perfecto (`135°` Red ↔ `45°`
   Blue). Los "Full" no: Red termina en `135°` pero Blue termina en `55°`, no en el
   `45°` que mirroraría igual que los "Goal". Esto apunta a que el `55°` de Blue Full
   sea el valor con error de transcripción del export, no que `135°`/`45°` estén mal —
   pero no se corrige sin que el equipo lo confirme.
3. **`LeaveGoal`: pose y alianza de arranque real.** Su path arranca en
   `(61.0, 75.682)`, muy lejos de la baldosa `(56/85.5, 8)` que usan los otros 9.
   ¿Desde qué baldosa/pose arranca en la práctica, con qué heading, y de qué alianza?
   Hoy el archivo compila con placeholders `_UNCONFIRMED`/`_PLACEHOLDER` explícitos y
   sin fijar `PoseStorage.isRedAlliance`.

## Más Pista Software sin robot — dataset RPM, telemetría, conversión de torreta — 2026-07-22

Tercera tanda de software puro de la misma sesión. Se ofrecieron 3 candidatos con
distinto nivel de choque con el orden del plan maestro y el equipo pidió los 3:

- **Cargador de dataset RPM (sin tensión con ningún prerrequisito).** Nuevo
  `shooter/ShotDatasetCsv.java`: convierte filas de texto (separadas por coma,
  `#` para comentarios/encabezado) en `List<ShotSample>` (`distanceInches,rpm`)
  y `List<ShotTrial>` (`sessionId,distanceGroupId,modelKind,role,distanceInches,
  targetRpm,measuredRpmAtFeed,rpmReadyHoldMs,outcome`) para `RpmModelSelector`.
  Antes había que escribir cada punto a mano como código Java. Fila mal formada
  lanza `IllegalArgumentException` con el número de línea, no se descarta en
  silencio. Test `ShotDatasetCsvTest`.
- **`TelemetryBlocks` — bloques `readiness`/`feeder` agregados.** Cerraban la
  lista de bloques que pide MP-07 ("modo/alianza, pose/calidad, visión, torreta,
  shooter, **readiness**, **feeder** y faults") — eran los dos únicos que
  faltaban. `feeder()` usa `FeederPulseStateMachine.State` (de la tanda
  anterior). **Alcance deliberadamente acotado:** no se migró ningún OpMode de
  producción (`MainTeleOp`, `SystemCheckOpMode`, etc.) a usar estos bloques —
  esos archivos ya pasaron gates físicos 10/10 o 20/20 y tocar su telemetría es
  blast radius innecesario para el valor que aporta; la migración real de
  OpModes queda para cuando se abra MP-07 formalmente.
- **`geometry/TurretYawConversion.java` — conversión pura ticks↔yaw.**
  Implementa exactamente la fórmula ya verificada por el equipo en
  `geometria-robot-mp04.md` sección 4.1 (`TICKS_PER_DEGREE ≈ 6.0807`, signo
  invertido por posición del motor/cable). Verificado con los dos ejemplos
  trabajados de esa hoja (`+1070 → -175.97°`, `-983 → +161.66°`). Cableado sólo
  como lectura adicional en `TelemetryBlocks.turret()` (`Turret/Yaw`) — no
  mueve nada ni decide nada, es sólo conversión de unidades para diagnóstico.
  **Explícitamente NO se tocó** la tabla del contrato formal de marcos
  (`03-auto-aim-limelight-y-cancha.md` sección 4): esa tabla exige la prueba
  física de cuatro anclas (sección 4.2) y su propio texto prohíbe llenarla "de
  memoria" — ninguna fila se completó, sigue `Pendiente` como estaba. MP-04
  formalmente sigue esperando que MP-03 se acepte primero
  (`plan-maestro-robot.md`, prerrequisito de MP-04).
- Verificación software: `:TeamCode:testDebugUnitTest` y `:TeamCode:assembleDebug`
  PASS con las tres piezas + sus tests. Nada de esto mueve actuadores.

## Reanudación de sesión Claude — scaffolding de disparo autónomo — 2026-07-22

- Se recuperó la sesión local `f1a3d0a2-e817-4b94-b647-53e1cd404af2`, interrumpida
  por límite de uso mientras modificaba `OneArtifactRedFull`. Las decisiones del
  equipo registradas allí son: los ocho autos `Artifacts` disparan detenidos al final
  del `PathChain`; los autos de dos piezas disparan ambas juntas al final; el heading
  `55°` de `OneArtifactBlueFull` era un typo y se corrige a `45°`; `LeaveGoal` sigue
  bloqueado hasta confirmar pose, heading y alianza.
- Los ocho autos quedaron estructurados como `PedroPathCommand -> ShootBurstCommand`:
  Goal usa `SHORT_SHOT_RPM`, Full usa `LONG_SHOT_RPM`, y el conteo es 1 o 2 según el
  nombre. Todos entregan `drive::getPoseSnapshot` al gate DEC-031/FND-019. Los autos
  permanecen registrados y visibles por decisión del equipo.
- El shooter final tiene ángulo mecánico fijo. `ShootBurstCommand` ofrece una ruta sin
  `ShooterHoodSubsystem`; ningún auto `Artifacts` instancia el shim legacy retirado.
  Las firmas legacy se conservan para código deshabilitado/compatibilidad.
- **Limitación deliberada:** esto es scaffolding, no autorización de tiro. Cada
  `ShooterSubsystem` de producción conserva `physicalOutputAllowed=false`; sólo
  `SystemCheckOpMode` y `MechanismTestTeleOp` pueden habilitar una instancia limitada
  de commissioning. En los autos actuales el shooter permanece en power cero, el
  feeder no recibe permiso y `ShootBurstCommand` falla cerrado al agotar hasta tres
  ventanas de readiness de 2 s. No están listos para competencia ni para alimentar
  piezas hasta completar MP-06/T8/T9 y MP-08.
- Se cerraron dos huecos de apagado del scaffolding: `ShootBurstCommand.end()` detiene
  shooter y kicker también al terminar normalmente; `FeederPulseStateMachine.update()`
  corta `PULSING` al máximo aun si el caller no repite `kick()`, y
  `KickerSubsystem.periodic()` ordena cero fuera de `PULSING`.
- Verificación final: 68/68 pruebas JVM, `:TeamCode:compileDebugJavaWithJavac`,
  `:TeamCode:testDebugUnitTest`, `assembleDebug` y `git diff --check` PASS. La revisión
  estática encontró 8/8 autos con `SequentialCommandGroup` + `ShootBurstCommand`, cero
  referencias a hood, cero TODO de disparo y cero headings `55°`; la habilitación de
  salida sigue limitada a los dos owners de commissioning. APK no instalado:
  81,379,421 bytes, SHA-256
  `79660A67559A29D70847DB33FF11CE9F32CA302A59D54CF49A73DA3F8A7F9FB2`. No hubo
  prueba física y estos resultados no autorizan alimentar piezas.

Reportar cada prueba como: `SHA | fecha | operador | configuración | repeticiones | resultado | tiempo máximo | evidencia | observaciones`. Un build exitoso no reemplaza estas pruebas.
