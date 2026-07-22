# Handoff MP-02 — odometría y drivetrain Pedro

Fecha: 2026-07-21

Estado: **ACCEPTED**

Rama: `masterplan`

Base de esta consolidación: `fed3424`
Commits: `74e950d` (`feat(tuning): record calibrated Road Runner baseline`) y `7b1535e` (`feat(drive): migrate production drivetrain to Pedro`)

## Arquitectura aceptada

El owner de producción quedó migrado atómicamente a:

`MainTeleOp -> RobotContainer -> PedroDriveSubsystem -> PedroDriveAdapter -> Follower`

Una sola instancia de `Follower` posee motores, pose, paths, actualización y stop. Road Runner queda en fuente como baseline calibrado/rollback, pero sus tuners dinámicos no se registran en Driver Station y no se instancia junto con Pedro sobre el mismo hardware.

Constantes confirmadas: tres pods `par0=rightFront`, `par1=leftFront`, `perp=rightBack`; masa Pedro `8.5 kg`; escala `0.00191837052073273 in/tick`; offsets Pedro `leftPodY=+6.942896027755779 in`, `rightPodY=-6.089060628403165 in`, `strafePodX=-4.636066386085883 in`.

## Evidencia física

- Direcciones/seguridad aisladas: `9/9 PASS`.
- T5: `40/40 PASS`; cinco intentos por grupo para forward, backward, left, right, CCW 360°, CW 360°, cuadrado y ruta mixta.
- Máximo global T5: `0.787 in` radial y `1.650°` de heading; sin deriva progresiva.
- Spot-check final de producción con masa `8.5 kg`: START reset, release stop, BACK E-stop y forward+turn, todos `PASS`.
- FND-004 y FND-017 están `CLOSED`.

APK instalado y físicamente probado al cierre: 81,356,132 bytes, SHA-256 `9287337811E98428E46DA81FF0A6526DE3FDFA33990ADC26BE41FE80F32A7362`.

Build reproducido antes de commits: 40/40 pruebas JVM, cero failures/errors, `assembleDebug` y `git diff --check` PASS. APK reproducido localmente: 81,332,203 bytes, SHA-256 `27BAAE88C1E5E375589F276E157379CFAFE006820FA74890ABE167839B0FB35B`. Este segundo hash demuestra build, no hereda automáticamente la aceptación física del APK instalado.

## Contrato operativo

- Potencia manual máxima Pedro: `0.90`.
- Giro del stick derecho: derecha=horario; escala `0.70` para conservar margen de traslación simultánea.
- START convierte el frente actual en heading cero usando `Follower.setPose()` durante runtime.
- Release y BACK ordenan cero; BACK solicita Stop global.
- No repetir T5 mientras no cambien geometría/signos/escala de pods, masa, direcciones, ownership o rutas de stop.

## Rollback

`74e950d` conserva el baseline Road Runner calibrado inmediatamente anterior al cambio de owner de producción. Revertir la migración como unidad; no crear un runtime dual RR/Pedro.

## Siguiente dependencia

MP-03 puede iniciar. Cerrar MP-02 no habilita visión, fusión de pose, auto-aim ni alimentación automática.

## Referencias

- [Bitácora cronológica](handoff-task.md)
- [Runbook de odometría](09-runbook-paso2-odometria.md)
- [Hallazgos](hallazgos.md)
- [Plan maestro](../plan-maestro-robot.md)
