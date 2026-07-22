# Handoff MasterPlan — continuación en MP-03

Fecha: 2026-07-21

Rama: `masterplan`

Base previa a la consolidación: `fed3424`
Estado de trabajo: MP-01 y MP-02 aceptados; MP-03 es la siguiente fase activa.

## Estado ejecutivo

| Paquete | Estado | Evidencia/condición | Próxima acción |
|---|---|---|---|
| MP-01 | `ACCEPTED` | [Handoff MP-01](handoff-MP01.md) | Reabrir sólo si cambia hardware, límites, ownership o safety path |
| MP-02 | `ACCEPTED` | [Handoff MP-02](handoff-MP02.md) | Conservar Pedro como owner único |
| MP-03 | `NOT_STARTED` / activo | Limelight instalada, mapping/configuración aún no verificados en repo | Inventario sin movimiento y wrapper fail-closed |
| MP-04 | `NOT_STARTED` | Requiere MP-02 + MP-03 | Contrato de frames y fusión |
| MP-05 | `NOT_STARTED` | Requiere pose/fusión confiable | Auto-aim de torreta dentro de límites |
| MP-06 | `NOT_STARTED` | Shooter base seguro, pero faltan modelo RPM e interlocks completos | T8/T9 y feeder acotado |
| MP-07…MP-10 | `NOT_STARTED` | Dependencias anteriores pendientes | Integración, validación, limpieza y release |

## Contexto técnico vigente

- Checkout correcto: `C:\dev\RobotCode2026`.
- Producción usa Pedro mediante `MainTeleOp -> RobotContainer -> PedroDriveSubsystem -> PedroDriveAdapter -> Follower`.
- Road Runner se conserva sólo como rollback/baseline; no habilitar sus tuners ni instanciarlo junto con Pedro.
- Los autónomos históricos siguen ocultos/deshabilitados; las rutas nuevas se crearán cuando el lead entregue su definición.
- No hay webcam activa. La siguiente cámara es Limelight 3A fija al chasis.
- Visión, auto-aim y corrección de pose continúan inhibidos.

## Primera secuencia segura de MP-03

1. Sin mover actuadores, abrir `Configure Robot` y registrar nombre exacto, tipo y conexión de Limelight.
2. Confirmar alimentación/red, versión de firmware/app y acceso a su interfaz.
3. Registrar pipeline, field map y familia/IDs de tags planeados.
4. Medir extrínseca desde el origen acordado del robot: X forward, Y left, Z up y yaw/pitch/roll, con unidades explícitas.
5. Verificar contra el SDK declarado que `Limelight3A` ya está disponible; no agregar dependencia externa por conveniencia.
6. Implementar un único `LimelightSubsystem`/wrapper con start, pipeline, poll, timestamp, latencia, calidad, timeout y stop/close fail-closed.
7. Primer gate: diagnóstico sólo de lectura. Desconectar/tapar cámara no puede mover drivetrain, torreta, feeder ni shooter.

Punto de detención: no conectar ninguna observación visual a pose o torreta hasta que MP-03 pase init/stop repetido, desconexión segura y telemetry de freshness/latencia.

## Datos que debe aportar el equipo al comenzar

```text
Nombre exacto en Configure Robot:
Tipo mostrado:
Conexión/puerto:
Firmware/app:
Pipeline actual:
Field map/tag family:
X/Y/Z de cámara respecto al origen del robot:
Yaw/pitch/roll:
Unidades:
```

## Riesgos abiertos relevantes

- FND-021 permanece contenido hasta confirmar mapping, extrínseca y pipeline de Limelight en MP-03.
- FND-018 y FND-019 pertenecen a MP-06; cerrar odometría no habilita feeder automático.
- FND-008 (ownership de torreta/visión) se resuelve en integración posterior; MP-03 debe encapsular la cámara sin mapear torreta.

## Verificación y rollback

Antes de esta consolidación: `:TeamCode:testDebugUnitTest :TeamCode:assembleDebug` PASS, 40/40 pruebas, cero failures/errors y `git diff --check` PASS. Consultar [handoff MP-02](handoff-MP02.md) para distinguir APK físicamente probado del APK reproducido.

Rollback de producción Pedro: `74e950d`, inmediatamente anterior al commit de migración `7b1535e`. Rollback general previo a la sesión consolidada: `fed3424`. Usar rama/worktree limpio; no resetear destructivamente trabajo local.

## Fuentes autoritativas para el siguiente chat

1. Este índice.
2. [Handoff MP-02](handoff-MP02.md).
3. [Bitácora cronológica](handoff-task.md).
4. [Plan maestro](../plan-maestro-robot.md), sección MP-03.
5. [Contrato de hardware](contrato-hardware.md).
6. [Protocolo de sesión](10-protocolo-agente-sesion.md).
