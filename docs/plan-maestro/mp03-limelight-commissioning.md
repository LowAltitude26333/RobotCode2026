# MP-03 — commissioning de Limelight 3A

Fecha de apertura: 2026-07-21

Estado: `IN_PROGRESS / BLOCKED_PHYSICAL_INPUT`

Esta hoja es la fuente de evidencia para MP-03. El único OpMode autorizado en
esta fase es `Limelight Diagnostic`: construye solamente el wrapper de cámara y
no mapea drivetrain, torreta, shooter, kicker ni intake. Ninguna observación se
consume todavía para pose o movimiento.

El equipo está entregando la geometría física (sistema de coordenadas, extrínseca
de cámara, eje de torreta, shooter y límites de giro) de forma incremental. Los
datos crudos se acumulan en [geometria-robot-mp04.md](geometria-robot-mp04.md)
conforme llegan — esa hoja alimenta el contrato de marcos formal de MP-04
(`03-auto-aim-limelight-y-cancha.md` secc. 4), que sigue `NOT_STARTED`.

## Contrato software verificado

- SDK efectivo declarado: FTC SDK `10.3.0`; su artefacto `Hardware` incluye
  `Limelight3A`, `LLResult`, `LLResultTypes` y `LLStatus`. No se agregó una
  dependencia externa.
- Owner único: `LimelightSubsystem`, mediante `hardwareMap.tryGet` y el nombre
  provisional `RobotMap.LIMELIGHT = "limelight"`.
- Poll rate solicitado: `50 Hz`, validado dentro del rango `1..250 Hz` del SDK.
- Pipeline solicitado y cotejado con la interfaz: índice `0`, nombre
  `Pipeline_Name`, tipo `AprilTags`.
- Freshness máxima provisional: `200 ms`.
- El canal accionable exige dispositivo presente y conectado, resultado válido
  y fresco, pipeline confirmado, tag esperado, timestamp y metadata finitos,
  familia no vacía, área `(0,100] %` y ángulos físicamente posibles.
- `stop`/`close` son idempotentes. Una excepción del SDK, desconexión o fallo de pipeline
  publica una observación rechazada con valores accionables en cero.
- `getBotpose()` no se considera disponible sólo por devolver un objeto: además
  se exige `getBotposeTagCount() > 0`.

## Inventario físico pendiente

Completar sin inventar valores. No registrar direcciones IP, contraseñas ni
otros datos privados de red en el repositorio.

| Dato | Valor/evidencia | Estado |
|---|---|---|
| Nombre exacto en Configure Robot | `limelight`; renombrado, guardado y configuración activada por el lead | CONFIRMADO 2026-07-21 |
| Tipo mostrado | Entrada detectada inicialmente como `Ethernet Device`; la pantalla no mostró otro campo de tipo | CONFIRMADO 2026-07-21 |
| Conexión/puerto físico | USB-C de Limelight al puerto USB-A azul del Control Hub; la pantalla reportó `USB 2.0` | CONFIRMADO 2026-07-21 |
| Alimentación y enlace Ethernet-over-USB | Detectada por Configure Robot; falta confirmar comunicación desde `Limelight Diagnostic` | PARCIAL |
| Versión firmware | LimelightOS `2026.0`, visible como `Limelight 3A 2026.0` en la UI | CONFIRMADO 2026-07-21 |
| Versión app/interfaz Limelight | UI web integrada `2026.0`; Hardware Manager no usado en esta verificación | CONFIRMADO 2026-07-21 |
| Pipeline AprilTag: índice/nombre/tipo | Índice `0`, nombre visible `Pipeline_Name`, tipo `AprilTags`; resolución `640x480 90fps` | CONFIRMADO 2026-07-21 |
| Marker size del pipeline | Corregido a `165.1 mm` para el cuadrado negro oficial DECODE; tag de práctica medido `160 x 160 mm` no sirve para validar escala 3D | CORREGIDO / VALIDACIÓN 3D PENDIENTE |
| Export del pipeline y SHA-256 | Original `pipeline-0-original-2026-07-21.vpr`: `F5C48FF047E17D6CC02551122E51957218DA37D7E85C7CF24B50E0CDD0100994`; corregido `pipeline-0-decode-165.1mm-2026-07-21.vpr`: `0B954CFA8641B460C13E8DF65A7A4691E7FD409821A09A008BF3ADA141D20693` | AMBOS VERSIONADOS |
| Field map y SHA-256 | `TBD` | PENDIENTE |
| Familia de tags | `AprilTag Classic 36h11 (587 tags)`; engine `U-Michigan` | CONFIRMADO 2026-07-21 |
| IDs objetivo azul/rojo | Manual DECODE TU32: azul `20`, rojo `24`; rojo detectado por la UI, azul pendiente de detección | PARCIAL |
| Origen del robot usado para medir | Centro de giro proyectado al plano del chasis, `(0,0,0) mm`; +X hacia intake, +Y izquierda, +Z arriba, yaw+ antihorario visto desde arriba. El plano del chasis es el piso de la cancha (confirmado verbalmente), así que este Z ya es comparable con el Z del marco de cancha/SDK sin offset — ver [geometria-robot-mp04.md](geometria-robot-mp04.md) secc. 0 | CONFIRMADO 2026-07-21 |
| Cámara X/Y/Z/pitch | Cinco configuraciones discretas de inclinación (30°/40°/50°/60°/67.2631°), cada una con su propio centro óptico — tabla completa en [geometria-robot-mp04.md](geometria-robot-mp04.md) secc. 1 | PARCIAL — falta confirmar cuál configuración está físicamente montada ahora mismo |
| Cámara yaw/roll | `Yaw=0.00°`, `Roll=0.00°` en las cinco configuraciones | CONFIRMADO 2026-07-21 |
| Unidades de extrínseca | mm y grados, tal como las entregó el equipo; conversión a la unidad interna del código (pulgadas) todavía no decidida | CONFIRMADO EL DATO CRUDO / CONVERSIÓN PENDIENTE |
| Montaje rígido y sin oclusión | Montaje rígido confirmado; oclusiones aún no verificadas | PARCIAL |

Muestra diagnóstica de la UI web aportada por el lead el 2026-07-21: tag `24`,
`tx=-20.66°`, `ty=-8.36°`, `ta=1.605%` y `tl=12.5 ms`. Es evidencia de
detección y observabilidad únicamente; no calibra exposición, latencia,
extrínseca, distancia ni precisión. La captura no se versiona porque contiene
parte del escritorio del usuario.

Configuración AprilTag original observada antes de corregir: `Marker Size=101.6 mm`,
`Detector Downscale=2`, `Quality Threshold=2`, `ID Filters` vacío, crop completo
`X/Y=-1..1`, keystone horizontal/vertical `0`, orden `Largest`, área
`0.0010..100.0000%` y agrupación `Single Target`.

El lead midió el cuadrado negro del tag de práctica `24` como `160 x 160 mm`.
La [producción oficial DECODE](https://ftc-resources.firstinspires.org/ftc/archive/2026/field/apriltag-us)
especifica `6.5 in = 16.50 cm` para el cuadrado negro y el
[Competition Manual TU32](https://ftc-resources.firstinspires.org/ftc/archive/2026/game/cm-html/DECODE_Competition_Manual_TU32.htm)
confirma targets completos de `8.125 in` e IDs azul/rojo `20`/`24`. Por tanto,
`101.6 mm` no es válido para el GOAL DECODE. Después de respaldar el pipeline
original, el lead corrigió `Marker Size` a `165.1 mm`. La UI continuó detectando
el tag `24`; la muestra posterior reportó `tx=-20.75°`, `ty=-8.44°`, `ta=1.600%`
y `tl=16.4 ms`. Esto confirma continuidad de detección, no precisión de distancia
ni pose 3D. El tag de práctica debe reimprimirse a tamaño real o etiquetarse como
aproximadamente `3%` menor que el oficial para no usarlo como referencia de
escala 3D.

El pipeline original descargado mide `1,992 bytes`, no contiene secretos ni
configuración IP y quedó versionado antes de cualquier ajuste. También confirma
`fiducial_skip3d=1`: la pose 3D está deshabilitada en este respaldo. Es válido
para diagnóstico 2D, pero no puede satisfacer MP-04 mientras no se configure y
valide field map, extrínseca y procesamiento 3D en una fase posterior.

El pipeline corregido también mide `1,992 bytes` y quedó versionado con SHA-256
`0B954CFA8641B460C13E8DF65A7A4691E7FD409821A09A008BF3ADA141D20693`.
La comparación de todos los campos encontró una sola diferencia:
`fiducial_size=101.6` pasó a `165.1`. Conserva `fiducial_skip3d=1`; por tanto,
todavía no habilita ni valida pose 3D.

Los exportables sanitizados se guardan bajo `docs/vision/limelight/` con nombre,
fecha, versión de firmware/app y SHA-256 anotados en esta hoja.

## Secuencia de prueba sin movimiento

1. Mantener robot inhibido y mecanismos sin piezas; acceso inmediato a Stop.
2. Confirmar en Configure Robot nombre, tipo y conexión antes de editar
   `RobotMap.LIMELIGHT`.
3. Abrir la interfaz Limelight, registrar versiones y exportar pipeline/field map.
4. Medir extrínseca desde el origen acordado con ejes `X forward`, `Y left`,
   `Z up`; registrar yaw/pitch/roll y unidades.
5. Instalar un APK construido desde el SHA candidato y abrir solamente
   `Limelight Diagnostic`.
6. Verificar nombre/estado de conexión, pipeline solicitado/observado, timestamp
   de Control Hub, staleness, latencia, familia, ID y área.
7. Ejecutar cinco ciclos `INIT -> START -> STOP`. No debe haber excepción,
   recurso huérfano ni movimiento.
8. Con el OpMode diagnóstico activo, tapar la cámara y desconectarla. La calidad
   debe cambiar a rechazo (`NO_TARGET`, `STALE` o `DEVICE_DISCONNECTED`) y todos
   los actuadores deben permanecer inmóviles.
9. Repetir conexión y confirmar recuperación sólo de telemetría. No habilitar
   pose, torreta ni feeder.

## Gate MP-03

MP-03 sólo puede cambiar a `ACCEPTED` cuando:

- el inventario físico y los hashes de exportables estén completos;
- cinco ciclos init/stop pasen con el mismo APK;
- pérdida/desconexión pase sin movimiento ni excepción;
- timestamp, staleness, latencia, pipeline, familia, ID y área sean observables;
- revisión humana confirme que ningún consumidor de pose o actuador está unido
  al wrapper.

Hasta entonces FND-021 permanece `CONTAINED` y MP-04/MP-05 continúan bloqueados.
