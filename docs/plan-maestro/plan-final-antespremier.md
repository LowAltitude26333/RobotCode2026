# Plan final antes del Premier

> Fecha: 2026-07-22
> Rama: `masterplan`
> Horizonte: dos días, una sesión larga de 4–6 h y 3–4 sesiones cortas de 45–90 min
> Estado: commissioning; no equivale a `release/competition`

## 1. Objetivo y autoridad

Este plan organiza el trabajo restante sin declarar fases aceptadas antes de pasar
sus gates. Complementa al [plan maestro](../plan-maestro-robot.md), a
[decisiones.md](decisiones.md) y al
[programa de pruebas](05-programa-pruebas.md). Ante una contradicción, esos
contratos y la evidencia del checkout actual tienen prioridad.

El candidato buscado incluye:

- drivetrain e intake estables;
- shooter con uno o dos setpoints **manuales por posición** validados;
- Limelight corrigiendo la pose propagada por Pedro;
- torreta apuntando geométricamente desde la pose fusionada;
- feeder protegido por pose, torreta, shooter, chasis, request y faults;
- `1 Artifact Red Goal` y `1 Artifact Blue Goal` como autos prioritarios;
- Leave correcto por alianza como fallback.

El modelo RPM-por-distancia es **extra/post-Premier**. No bloquea el shooter manual
ni los autos. Todas las sesiones de tiro sí llenarán el dataset compatible con
`ShotDatasetCsv` para conservar evidencia y permitir el ajuste posterior.

## 2. Estado de partida

- MP-01 y MP-02 están `ACCEPTED`; Pedro es el owner activo de pose/drivetrain.
- MP-03 está `IN_PROGRESS / BLOCKED_PHYSICAL_INPUT`.
- Limelight usa mapping `limelight`, pipeline `0` y firmware/UI 2026.0.
- El pipeline versionado conserva `fiducial_skip3d=1`; todavía no existe pose 3D
  autorizada para fusión.
- Falta exportar/versionar el field map y demostrar el tag azul 20 en condiciones
  equivalentes al rojo 24.
- La extrínseca activa corresponde al montaje de 30°.
- La torreta tiene cero manual seguro, límites `-983/+1070` ticks y conversión
  física ticks/grado registrada.
- El shooter ya tiene PID/feedforward/compensación/watchdogs/faults. No se construirá
  otro controlador desde cero.
- La prueba a 1000 RPM confirmó dirección, encoder, watchdog y stop, pero no T8.1.
- La salida de competencia del shooter sigue inhibida.
- Los presets 2450/2900/3600 RPM son candidatos, no valores certificados.
- El worktree contiene cambios sin commit que deben preservarse.

## 3. Prioridad y recortes

### P0 — nunca recortar

- Stop/E-stop, cero, límites, clamps, watchdogs y paths de potencia cero.
- Identidad de commit, APK, configuración, pipeline, field map y extrínseca.
- Gate MP-03 sin movimiento.
- T8.1 para cualquier RPM que llegue al candidato.
- Matriz completa del feeder.

### P1 — objetivo de Premier

- Shooter manual `GOAL_AUTO`.
- Fusión Pedro–Limelight.
- Bearing geométrico y auto-aim.
- Dos autos Goal espejo y Leave por alianza.

### P2 — sólo si P0/P1 pasan

- Segundo preset `NEAR_WALL`.
- Otra batería/iluminación.
- Trim visual angular pequeño y gated.
- Más autos.

### Extra — no bloquea

- Ajustar/comparar RPM lineal, cuadrática y piecewise por distancia.
- Certificar el modelo en dos sesiones independientes.
- Activar `AUTO_DISTANCE`.

No se intentará antes del Premier un EKF, 3600 RPM sin caracterización, limpieza
MP-09 ni release MP-10.

## 4. Arquitectura obligatoria

```text
PedroDriveAdapter
  -> PoseSnapshot odométrico + historial corto

Limelight 3D + field map + extrínseca
  -> observación de pose del robot con timestamp/calidad

PoseFusionEstimator
  -> gates de validez/frescura/config/geometría/movimiento/innovation
  -> corrección acotada de robotPose

robotPose fusionada
  -> eje real de torreta en campo
  -> bearing/distancia al goal de la alianza
  -> target angular/ticks dentro del arco
  -> readiness -> feeder interlocked
```

Queda prohibido usar `tx` como controlador directo de torreta. Podrá guardarse y,
después de validar la solución geométrica, aportar un trim pequeño con clamp, rate
limit y expiración.

### 4.1 Contrato de pose

- `PoseSnapshot` continúa como tipo neutral en pulgadas/radianes.
- Pedro propaga pose en cada ciclo.
- Limelight sólo corrige observaciones 3D que pasen todos los gates.
- Un `botpose` crudo nunca reemplaza de golpe la pose Pedro.
- La observación se compara con la pose Pedro al tiempo estimado de captura.
- MP-04/MP-05 no importan tipos Road Runner.

### 4.2 Interfaces previstas

- Observación 3D con pose, polling/capture timestamp, staleness, latencia, pipeline,
  tag count/IDs, metadata y razón de rechazo.
- Buffer de pose de 500 ms; rechazar si no existe bracket confiable o el gap supera
  60 ms.
- Estimador puro que produce innovation, bounded innovation, corrección, calidad y
  razón.
- `setPose` conserva semántica de reset y aumenta `resetEpoch`.
- La corrección visual usa una vía distinta y contador propio; no simula un reset.
- `PedroDriveSubsystem.getPoseSnapshot()` publica pose fusionada/calidad.

### 4.3 Calidad y candidatos iniciales

- `UNINITIALIZED`: Pedro aún no actualiza.
- `ODOMETRY_ONLY`: Pedro válido, sin visión aceptada reciente.
- `FUSED_GOOD`: cinco observaciones consecutivas aceptadas y corrección estable.
- `DEGRADED`: odometría viva fuera de la ventana autorizada.
- `INVALID`: pose/sensores inconsistentes o no finitos.

Candidatos de shadow, no constantes certificadas:

- edad máxima 200 ms;
- innovation máxima 6 in / 10°;
- corrección máxima por observación 0.25 in por eje / 0.5°;
- `alphaXY=0.10`, `alphaHeading=0.10`;
- corrección activa inicialmente sólo con robot estacionario.

Sólo `FUSED_GOOD` permite feed en el primer candidato. No habrá relocalización por
saltos grandes antes del Premier.

### 4.4 Bearing de torreta

El solver transforma el eje físico de torreta al campo y calcula:

```text
dx = goalX - turretAxisFieldX
dy = goalY - turretAxisFieldY
fieldBearing = atan2(dy, dx)
robotRelativeBearing = wrap(fieldBearing - robotHeading)
turretAngle = wrap(robotRelativeBearing - turretZeroOffset + aimBias)
```

Después convierte con `TurretYawConversion`, busca soluciones dentro del arco y deja
`AT_LIMIT` con potencia cero cuando ninguna es segura. El drivetrain nunca gira
automáticamente para rescatar un target.

## 5. Trabajo sin robot

Esta pista continúa activa entre sesiones y es avance real de software.

### 5.1 Implementable

- contratos de observación 3D/calidad;
- transforms field map–Pedro–robot–cámara–torreta–goal;
- historial e interpolación temporal;
- estimator shadow y contadores de rechazo;
- corrección acotada separada de `resetEpoch`;
- solver geométrico bearing/distancia/ticks;
- readiness de pose/torreta/shooter/chasis;
- telemetría y exports CSV;
- bindings manuales de presets/trim con output físico bloqueado;
- adaptación de autos prioritarios al mismo readiness;
- builders y QA de paquetes Excel.

### 5.2 Simulable

- cuatro anclas sintéticas y transform inversa;
- signos X/Y, wrap de heading y cruce de ±π;
- interpolación/rechazo por gap;
- outliers, stale, tag/pipeline/map incorrectos y pérdida/recuperación;
- replay de logs Pedro/Limelight al regresar los XLSX;
- convergencia acotada, rate limits y calidad;
- bearing desde eje de torreta, goal por alianza y target fuera de arco;
- grados/ticks y soft limits;
- dwell/readiness y matriz del feeder;
- geometría/simetría de autos;
- parser/export del dataset y modelos RPM como extra.

No certifica sin robot: extrínseca real, precisión Limelight, PID, RPM bajo carga,
signo físico, recuperación tras artifact, trayectoria, FOV, tracción, flexión,
cableado ni seguridad mecánica.

## 6. Paquetes Excel y versionado

Flujo híbrido y just-in-time:

1. `01-cierre-mp03_v1.xlsx`
2. `02-sesion-larga_v1.xlsx`
3. `03-cierre-mp04-mp05_v1.xlsx`, sólo si hace falta
4. `04-premier-cancha_v1.xlsx`
5. `consolidado-analisis_v1.xlsx`, después de recibir datos

Cada archivo lleno vuelve con `_FILLED`, se calcula su SHA-256 y se conserva
inmutable. El análisis ocurre en otro archivo. Si cambia el formato se crea una nueva
versión; nunca se sobrescribe evidencia.

### 6.1 Estructura común

- `INSTRUCCIONES`, `SESION`, `CAMBIOS`, `DASHBOARD`, `CONFIG`, `CATALOGOS`;
- hojas pequeñas de captura, `EXPORT_*` y `QA`.

Los XLSX serán offline, sin macros/conexiones externas. Entradas amarillas, fórmulas
grises, PASS verde, FAIL rojo y BLOCKED ámbar. Cada tabla cruda contiene una
observación por fila, unidades, filtros y paneles congelados.

Estados: `NOT_RUN`, `IN_PROGRESS`, `PASS`, `FAIL`, `BLOCKED`, `ABORTED`,
`INVALID_DATA`, `NOT_APPLICABLE`.

Cada paquete contiene `SchemaVersion`, `TemplateVersion`, `PacketId`,
`PacketVersion`, `SessionId`, `ConfigRevision`, commit y APK SHA.

Si cambia APK, PID, dirección, límites, watchdog, field map, pipeline, extrínseca o
hardware, se registra en `CAMBIOS`, aumenta `ConfigRevision` y no se combinan los
intentos para declarar un gate. Un cambio de seguridad detiene el bloque.

## 7. Paquete 01 — cierre MP-03

Primer entregable descargable: `01-cierre-mp03_v1.xlsx`.

### `MP03_01_Config`

Mapping, firmware/UI, conexión, pipeline, resolución/FPS, familia, marker size,
`fiducial_skip3d`, field map/hashes, extrínseca, rigidez, oclusiones y tags.

### `MP03_02_Lifecycle`

Cinco ciclos precreados con INIT/START/STOP, conexión, pipeline, excepción, recurso
abierto, movimiento y resultado.

### `MP03_03_Tags`

IDs 20/24, tx/ty/ta, staleness, latencias, tag count, pose 3D, calidad, razón y
evidencia.

### `MP03_04_Faults`

Cámara tapada, tag incorrecto, resultado inválido, pipeline incorrecto,
desconexión/reconexión, stale y pose 3D ausente. Todo fail-closed y sin movimiento.

### `MP03_05_StaticPose`

Pose física medida frente a pose Limelight. Es diagnóstico: no corrige ni mueve.

### `SHOT_DATASET`

Queda armado desde el primer paquete, pero no autoriza disparar durante MP-03.
Columnas exactas compatibles con `ShotDatasetCsv`:

```text
sessionId,distanceGroupId,modelKind,role,distanceInches,
targetRpm,measuredRpmAtFeed,rpmReadyHoldMs,outcome
```

Columnas auxiliares: AttemptId, ConfigRevision, posición, batería, pose fusionada,
error de torreta, artifact/batch, evidencia y notas. `EXPORT_SHOT_DATASET` conserva
exactamente las nueve columnas que parsea el código.

### Gate MP-03

- exportables/hashes completos;
- 3D habilitado/observable;
- IDs 20/24 demostrados;
- cinco ciclos sin excepción/recurso/movimiento;
- desconexión/reconexión fail-closed;
- rechazos sanitizados;
- ningún consumidor de pose/actuador conectado.

## 8. Sesión larga y paquete 02

Orden: seguridad, shooter sin carga, tiros `GOAL_AUTO`, `NEAR_WALL` sólo si pasa,
cuatro anclas/shadow, corrección, torreta e integración.

### Shooter

- 1000→1500→2000→2450→2900 RPM;
- primero kS/kV, después kP; kI/kD quedan cero salvo evidencia;
- cada setpoint: ±100 RPM ≥250 ms, sin overspeed y con stop;
- 3600 RPM bloqueado.

`GOAL_AUTO` usa 2900 sólo como candidato: diez tiros, ≥9/10 y gate RPM. `NEAR_WALL`
2450 es secundario. Máximo dos ajustes documentados. El dataset siempre se llena,
aunque el modelo automático quede diferido.

### MP-04

- cuatro poses conocidas por alianza cuando sea posible;
- rotación sin traslación;
- física/Pedro/Limelight/transformada/residual/latencia;
- shadow antes de corrección;
- corrección estacionaria acotada;
- no cambia `resetEpoch` ni excede clamps;
- pérdida de visión conserva odometría y cambia calidad.

### MP-05

- INIT: X=BLUE/B=RED y lock al START;
- bearing desde pose fusionada/eje real;
- error estable ≤2.5°;
- sin cruce de límites ni movimiento desarmada;
- `AT_LIMIT`, reset, pérdida de pose, interruption y E-stop terminan en cero.

### Feeder

Veinte secuencias request/shooter/turret/pose/chassis/fault: ningún pulso extra,
duración ≤700 ms, cooldown ≥300 ms y corte ≤50 ms.

## 9. Sesiones cortas restantes

### Sesión corta 2

Cerrar MP-04/MP-05 si la larga terminó en shadow. No reabrir tuning del shooter salvo
rollback registrado.

### Sesión corta 3 — Premier

- una pasada seca y dos completas consecutivas de cada `1 Artifact Goal`;
- validar Leave rojo/azul;
- validar pose, parada, aim, RPM y feed;
- deshabilitar individualmente cualquier auto fallido;
- `Leave Goal` y los otros seis autos de tiro quedan fuera.

### Sesión corta 4 opcional

Smoke test, segunda batería/iluminación, fallbacks, jam, Stop/E-stop y simulación de
match. No se agregan funciones.

## 10. Triple QA de cada Excel

### QA-1 — fuente de verdad

- releer código/docs del commit;
- cotejar mappings, IDs, unidades, límites, timeouts y gates;
- confirmar que el paquete corresponde al APK;
- registrar fuentes/discrepancias.

### QA-2 — fórmulas y estructura

- fixtures PASS/FAIL/BLOCKED/ABORTED/vacío/límite/fuera;
- libro vacío nunca muestra PASS;
- revisar tablas, validaciones, fórmulas y exports;
- escanear `#REF!`, `#DIV/0!`, `#VALUE!`, `#NAME?`, `#N/A` y ciclos;
- cálculo independiente de error RPM, residual XY, wrap angular, hit rate y gates;
- ConfigRevision no se mezcla.

### QA-3 — round-trip y visual

- exportar, reimportar e inspeccionar;
- renderizar todas las hojas;
- revisar clipping, colores, entradas, fórmulas, filtros, paneles y dashboard;
- simular flujo vacío y con fixtures;
- calcular SHA-256 final.

Si falla una capa se corrige y se repiten las tres. Ningún XLSX no verificado se
entrega.

## 11. QA de archivos llenos

- conservar original/hash;
- comprobar schema/version;
- detectar columnas borradas/reordenadas o fórmulas alteradas;
- validar tipos/unidades y separar ConfigRevision;
- marcar ambigüedad `INVALID_DATA`, nunca adivinar;
- analizar/generar el siguiente paquete en archivos nuevos.

## 12. Fallbacks

| Falla | Acción |
|---|---|
| MP-03 | Limelight diagnóstica; MP-04 no abre. |
| Pose 3D/shadow | Pedro `ODOMETRY_ONLY`; nunca tx directo. |
| Corrección salta/oscila | Volver a shadow; bloquear aim/feed. |
| MP-05 | Torreta detenida/cero seguro. |
| T8.1 | Salida de competencia inhibida. |
| GOAL_AUTO | Autos con tiro disabled; usar Leave. |
| Segundo preset | Publicar sólo GOAL_AUTO. |
| Feeder | Salida cero; no hay override. |
| Auto individual | Deshabilitar sólo ese OpMode. |
| Fault crítico | Shooter, feeder y torreta a cero. |

## 13. Orden inmediato

1. **Llenar** el template canónico
   [`01-cierre-mp03_v1.xlsx`](../../outputs/plan-final-antespremier/01-cierre-mp03_v1.xlsx)
   usando el APK diagnóstico documentado en `handoff-task.md`.
2. Guardar una copia como `01-cierre-mp03_v1_FILLED.xlsx`, sin borrar columnas ni
   editar fórmulas.
3. Regresar el `_FILLED.xlsx` y conservar el original sin cambios.
4. Analizar y aprobar el archivo lleno antes de habilitar consumidores.
5. Sólo después implementar MP-04 shadow y preparar el paquete largo.

El plan, el template y su triple QA ya están generados/versionados. Por tanto, ningún
trabajo de MP-04, fusión o aim es el siguiente paso mientras el Excel siga vacío.

## 14. Candidato para Premier

- MP-03, fusión y torreta pasan gates aplicables;
- `GOAL_AUTO` pasa T8.1 y dataset provisional;
- feeder pasa 20 secuencias;
- existe auto/Leave válido por alianza;
- operadores completan smoke/fallback/E-stop;
- cero findings críticos/altos;
- mismo commit/APK/configuración de la evidencia;
- todos los XLSX pasan QA triple.

Hasta MP-08–MP-10, el nombre correcto es **candidato operativo para el Premier**.
