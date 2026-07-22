# Plan maestro de robustecimiento del robot

> Estado: MP-01 y MP-02 `ACCEPTED`; MP-03 es la siguiente fase activa sobre `masterplan`
> Baseline de implementación: `origin/main@a887fe4f7ca9023eec6034a0db6b8d918c640ecc`; preserva evidencia histórica de `b5a1342`
> Última actualización: 2026-07-22
> Alcance: arquitectura, torreta, Limelight 3A, odometría, shooter, controles, pruebas y limpieza
> Responsables sugeridos: líder de software, responsable mecánico/eléctrico, operador 1, operador 2 y responsable de pruebas
> Fuente de verdad: antes de ejecutar una fase se debe volver a inspeccionar la rama y el hardware; este documento describe el baseline indicado, no garantiza el estado físico del robot.

Reinspección MP-01: sobre `a887fe4`/`24f9911`, el equipo confirmó el mapa lógico, tres encoders
de odometría conectados a puertos del drivetrain, un kicker compuesto por motor + CRServo y
shooter de un motor con máximo declarado de 6000 RPM. El software contiene los caminos directos,
retira hood/webcams/Shooter2 del hardware activo e implementa shooter fail-closed. Los findings
permanecen `FIX_READY` o `BLOCKED_PHYSICAL` hasta completar sus gates.

## 1. Resultado buscado

Construir una base de software de competencia pequeña, entendible y tolerante a fallas para un robot con:

- drivetrain mecanum;
- tres ruedas muertas de odometría;
- intake;
- feeder, actualmente llamado `KickerSubsystem`;
- shooter de un motor y ángulo mecánico fijo;
- torreta con recorrido limitado y cero manual;
- Limelight 3A fija al chasis;
- un solo TeleOp de competencia para ambas alianzas;
- un solo OpMode de diagnóstico seguro;
- un conjunto reducido y versionado de autónomos Pedro Pathing en el artefacto final de competencia, cada uno sujeto al mismo contrato de seguridad que el TeleOp (`SafeAutonomousOpMode`, E-stop, `RobotSafety`, límites vigentes) — ver DEC-041. Los autónomos Road Runner legado no se conservan: Pedro es dueño único de pose/movimiento (DEC-034) y esos autos ya no compilan contra la composición actual.

El auto-aim no dependerá exclusivamente de una cámara. La odometría mantendrá continuamente la pose y el ángulo geométrico al goal; Limelight aportará observaciones frescas y validadas para corregir pose y, cuando sea apropiado, ajustar el bearing final. Si ambas fuentes dejan de ser confiables, el robot entrará en un modo degradado deliberado, limitado y visible, nunca en una degradación silenciosa.

Este plan es un programa de trabajo, no una especificación ya validada. Toda constante física, nombre de hardware, orientación, relación mecánica y límite marcado como pendiente se clasifica `TBD-BLOCKING`: el actuador o automatismo afectado permanece inhibido hasta medir, revisar y registrar el dato en la [guía de verificación de hardware](plan-maestro/08-guia-verificacion-hardware.md).

## 2. Decisiones no negociables del diseño aprobado

1. Pedro Pathing será el stack primario de localización y movimiento. Road Runner se conservará como rollback en un snapshot Git previo a la limpieza, no como segunda arquitectura activa.
2. La localización física utilizará tres dead wheels. Sus offsets, signos y resolución se medirán; no se reutilizarán valores provisionales por conveniencia.
3. Limelight 3A estará fija al chasis. Su nombre propuesto es `limelight`, sujeto a confirmar la configuración del Robot Controller.
4. La torreta se centra manualmente, tiene arco limitado por cables/mecánica y no puede asumir rotación continua. No se moverá hasta quedar armada explícitamente.
5. El seguimiento podrá operar continuamente después del armado, pero respetará límites físicos y calidad de estimación. Si el target queda fuera del arco, se detendrá y pedirá al conductor orientar el chasis; el software no girará el drivetrain automáticamente.
6. El shooter tiene un solo motor y ángulo fijo. La selección automática de RPM dependerá de distancia mediante un modelo empírico validado.
7. El operador 2 podrá iniciar/detener RPM manual, aplicar trim en pasos de 100, restaurarlo y, después de MP-06, cambiar entre RPM automática y manual. No habrá override de límites desde gamepad; siempre existirá un clamp absoluto de RPM validado físicamente.
8. `gamepad1 RB` expresa una solicitud sostenida, no potencia directa. El feeder sólo producirá pulsos acotados, separados por cooldown y revalidados antes de cada pieza. Soltar el control, perder readiness, interrumpir el comando, detener el OpMode o activar E-stop debe ordenar potencia cero.
9. La alianza se seleccionará directamente durante init: `X = BLUE`, `B = RED`. Se mostrará claramente y se bloqueará al iniciar; no se alternará accidentalmente durante el match.
10. Durante MP-01 quedan visibles `MainTeleOp`, System Check y Shooter Tuning. Los tuners de Pedro y los registradores dinámicos de Road Runner permanecen en código pero deshabilitados hasta que MP-02 les proporcione lifecycle/E-stop verificable. El release de competencia tendrá exactamente un TeleOp y un System Check.
11. Ningún cambio de fase se considera terminado sólo porque compile. Cada gate exige evidencia reproducible y validación física proporcional al riesgo.
12. La referencia primaria de E-stop será `gamepad1 BACK`, inmediata y latched. Debe probarse con el Driver Station y gamepad exactos; si falla esa validación, el fallback será `gamepad1 START+Y` sostenido 0.5 s.

El registro completo y modificable de estas decisiones está en [decisiones.md](plan-maestro/decisiones.md).

## 3. Cómo ejecutar el plan en partes

Cada paquete de trabajo, o `MP`, es una unidad entregable. Para comenzar uno:

1. crear una rama pequeña desde el baseline aprobado;
2. copiar sus criterios de aceptación a la descripción del PR o bitácora;
3. confirmar los prerrequisitos y hechos de hardware;
4. implementar sólo el alcance del paquete;
5. ejecutar su gate de seguridad y sus pruebas;
6. registrar resultados en [hallazgos.md](plan-maestro/hallazgos.md);
7. actualizar la decisión correspondiente si cambió un supuesto;
8. integrar únicamente con revisión de estudiantes/mentores capaces de explicar flujo de datos, unidades, límites y apagado.

No deben abrirse simultáneamente paquetes que compitan por la misma propiedad de hardware. En particular, la migración de localización, la integración de Limelight y el control de torreta necesitan interfaces acordadas primero para evitar que cada rama invente su propio sistema de coordenadas.

## 4. Mapa de dependencias

```text
MP-00 Auditoría vigente
   |
   +--> MP-01 Contrato físico y bloqueadores críticos
           |
           +--> MP-02 Pedro + odometría calibrada
           |       |
           |       +--> MP-04 Estimador de pose y coordenadas de cancha
           |               |
           |               +--> MP-05 Auto-aim de torreta
           |               +--> MP-06 Modelo de RPM, controles e interlocks
           |
           +--> MP-03 Limelight, configuración y wrapper
                   |
                   +--> MP-04

MP-05 + MP-06 --> MP-07 Integración y System Check
MP-07          --> MP-08 Validación completa
MP-08 aceptado --> MP-09 Limpieza del candidato
MP-09          --> MP-10 Revalidación y release de competencia
```

La limpieza sólo comienza después de demostrar el reemplazo, pero no hereda automáticamente la aceptación anterior: MP-10 repite el programa completo sobre el SHA limpio antes de llamarlo release.

## 5. Programa de trabajo

### MP-00 — Congelar y auditar el baseline

**Objetivo.** Convertir la fotografía actual del repo en una lista trazable de riesgos, dueños de hardware y comportamientos activos.

**Entradas.** `origin/main@a887fe4f7ca9023eec6034a0db6b8d918c640ecc`, con `b5a1342` preservado como evidencia histórica, `AGENTS.md`, `docs/critical-findings.md` y, cuando el equipo la entregue, la exportación de configuración del Robot Controller. La configuración no existe hoy en el repo.

**Trabajo.**

- Repetir inventario de OpModes registrados, incluidos registradores dinámicos.
- Recorrer la composición real desde `MainTeleOp` hasta subsistemas, comandos, localizador, constantes y hardware map.
- Clasificar cada finding histórico como corregido, contenido, parcial, reemplazado o vigente.
- Confirmar que no haya dos componentes activos mapeando el mismo actuador.
- Registrar SHA y dependencias declaradas mediante inspección estática. Build, APK e instalación se miden sólo cuando una fase de código sea autorizada; no forman parte de esta corrección documental.
- Crear una tabla de hechos físicos todavía desconocidos.
- Registrar como hechos actuales que hood y webcam fueron retiradas, mientras Limelight está instalada pero todavía no tiene mapping/configuración verificable en el repo.
- Antes del primer cambio de código, crear/publicar/verificar `baseline/pre-mp01-YYYYMMDD`; esta edición documental no materializa el tag.

**Salidas.** [Auditoría del estado actual](plan-maestro/01-auditoria-estado-actual.md) revisada y configuración física adjunta a la bitácora del equipo.

**Gate MP-00.** Ningún finding crítico queda sin dueño, siguiente acción o condición de contención. El equipo puede señalar cuál OpMode controla cada motor en el baseline y cuáles caminos visibles son tuners deliberados de commissioning.

**Rollback.** No aplica: es sólo inspección y documentación.

### MP-01 — Contrato de hardware y bloqueadores críticos

**Objetivo.** Hacer que el robot pueda inicializarse, armarse y detenerse de forma determinista antes de agregar auto-aim.

**Prerrequisitos.** MP-00 aceptado; robot elevado o mecanismo desconectado según la prueba.

**Trabajo.**

- Resolver el registro de cleanup de `VisionPortal` antes de su construcción; la auditoría detectó una referencia ligada a `null` con riesgo de fallar en init.
- Corregir `TeleopTorreta` para que nunca programe `ShooterCommand2` con un `ShooterMotor` nulo y contener los outputs directos/latched del `IntakeTeleOp` vigente.
- Extender `SafeCommandOpMode` con un hook repetitivo de init previo a START. Allí se evaluará `gamepad2 START+BACK` durante 1 s, con progreso, cancelación y confirmación antes de poner a cero el encoder y armar.
- Confirmar nombre, dirección, signo de encoder, ticks por arco permitido, centro físico, margen contra hard stops y ruta de cables de la torreta.
- Confirmar un solo motor de shooter, ticks/rev efectivos, relación externa, inversión, RPM máxima segura y comportamiento de zero power.
- Hacer que encoder congelado, RPM imposible, voltaje inválido/no disponible u overspeed de shooter produzcan `SENSOR_FAULT`, target cero y power cero; nunca convertir el dato inválido en RPM cero que solicite máxima potencia.
- Usar los nombres confirmados `kickerMotor` y `kickerServo`; ambos outputs pertenecen a `KickerSubsystem` y cambian a cero/kick/reverse en el mismo ciclo.
- Completar el contrato físico de drive, tres pods, IMU, intake, shooter, feeder, torreta y Limelight; registrar hood/webcam como retiradas, sin inferir mappings ausentes.
- Hacer que E-stop y `RobotSafety.stopAll()` estén disponibles desde producción, diagnóstico y cada tuner autorizado. `BACK` es el control primario; si la prueba del Driver Station exacto falla, documentar y activar el fallback `START+Y` por 0.5 s.
- Auditar todos los adapters `Action` y comandos secuenciales para que interrupción y stop ordenen cero a drive, intake, feeder, shooter y torreta.

**Salidas.** Contrato de hardware versionado, init seguro y pruebas de apagado. No se cambia todavía el stack de localización.

**Gate MP-01.** Cero movimiento de torreta antes de armado; el cero se invalida en cada init/reset/brownout; todo actuador recibe comando cero en el primer ciclo del scheduler y en ≤50 ms tras soltar/interrumpir/parar; E-stop es accesible y probado a potencia reducida. El spin-down físico se mide aparte.

**Rollback.** Revertir el PR del paquete. No eliminar límites ni restaurar un camino que deje motores energizados.

### MP-02 — Pedro Pathing y odometría de tres pods

**Objetivo.** Obtener una única pose continua, calibrada y con calidad observable usando Pedro Pathing.

**Prerrequisitos.** MP-01; medidas físicas de pods y direcciones; superficie y trayectoria de prueba repetible.

**Trabajo.**

- Crear una rama de migración sin borrar Road Runner.
- Medir y documentar diámetro efectivo, ticks por revolución, offsets de cada pod respecto al centro de giro, signo de cada encoder y orientación de IMU.
- Corregir los valores provisionales y asignaciones ambiguas de `pedroPathing/Constants.java`.
- Elegir y documentar un marco de coordenadas del robot y otro de cancha; unidades internas explícitas.
- Integrar `PoseProvider` detrás de una interfaz independiente de Pedro para que auto-aim no importe APIs del localizador.
- Migrar también la propiedad del movimiento: conducción manual, seguimiento de paths, actualización de pose y `stop()` pasan por un único adapter/`DriveSubsystem` respaldado por Pedro. Road Runner no puede seguir siendo el dueño silencioso del drivetrain.
- Calibrar forward/lateral/turn, compensación de track width y estabilidad de heading.
- Marcar calidad `UNINITIALIZED`, `ODOMETRY_ONLY`, `FUSED_GOOD`, `DEGRADED` o `INVALID` con razones y timestamps.

**Salidas.** `PoseProvider` primario, constants medidas, telemetría de pose/calidad y logs de repetibilidad.

**Gate MP-02.** Cinco repeticiones por sentido de forward, strafe, giro y ruta mixta registran error por segmento, media y máximo; la ruta acordada termina con error ≤2 in y ≤2° sin error sistemático creciente. Ningún encoder de drive sustituye accidentalmente un dead wheel.

**Rollback.** Tag previo conserva Road Runner; revertir la rama completa si Pedro no supera el gate. No mantener ambos stacks activos en producción.

### MP-03 — Integración encapsulada de Limelight 3A

**Objetivo.** Incorporar Limelight usando la API incluida en el FTC SDK ya resuelto, con ciclo de vida, configuración y datos observables.

**Prerrequisitos.** MP-01; Limelight montada rígidamente; nombre del dispositivo, alimentación y red confirmados.

**Trabajo.**

- Verificar que la versión de SDK efectiva exponga `Limelight3A`; evitar una dependencia externa si el SDK ya la incluye.
- Crear un `LimelightSubsystem`/wrapper como único propietario del dispositivo.
- Definir `start`, selección de pipeline, poll rate, lectura, timeout, `stop`/`close` y comportamiento de error.
- Versionar en el repo las instrucciones y exportables necesarios: pipeline, field map, orientación, pose extrínseca de cámara y versión de firmware/app. No guardar secretos ni datos de red privados.
- Convertir resultados válidos a una observación inmutable con timestamp, tag ID, bearing, pose opcional, latencia y calidad.
- Rechazar resultados nulos, inválidos, viejos, de ID incorrecto, fuera de límites o físicamente imposibles.
- Tratar la webcam como hardware físicamente retirado. El viejo `VisionPortal` sólo se conserva como código histórico hasta que Limelight pase su gate; no se diseña coexistencia de cámaras.

**Salidas.** Fuente de observaciones desacoplada del control de torreta y herramienta de diagnóstico sin movimiento.

**Gate MP-03.** Init y stop pueden repetirse sin excepción ni recurso abierto; pérdida/desconexión de cámara no mueve actuadores; timestamps/latencia se ven en telemetry/log.

**Rollback.** Deshabilitar el consumidor y conservar odometría/manual. El wrapper debe fallar cerrado.

### MP-04 — Cancha, coordenadas y fusión de pose

**Objetivo.** Relacionar pose de Pedro, pose reportada por Limelight, goal de alianza y punto de mira en un contrato matemático único.

**Prerrequisitos.** MP-02 y MP-03 aceptados; CAD/manual oficial revisados.

**Trabajo.**

- Documentar el origen, ejes, sentido positivo, unidad y heading de cada marco: cancha oficial/SDK, Pedro, robot, cámara y torreta.
- Crear un catálogo de poses iniciales con nombre humano, alianza, X/Y/heading, unidades, referencia de colocación y evidencia. Ninguna pose `TBD-BLOCKING` puede seleccionarse como default silencioso.
- Validar transforms con al menos cuatro puntos/anclas conocidos y ambas alianzas; no asumir que dos librerías comparten origen o convención.
- Medir la transform rígida chasis→cámara y chasis→eje de torreta.
- Definir el punto físico de tiro del goal separado de la pose del AprilTag: el centro del tag no es automáticamente el centro balístico del goal.
- Propagar odometría en cada loop y aplicar sólo correcciones visuales que pasen gates de frescura, ID, residual, velocidad y geometría.
- Comenzar con una corrección complementaria acotada. Adoptar EKF únicamente si los logs demuestran que el método simple no cumple.
- Mantener una calidad y una razón de rechazo legibles; jamás reemplazar la pose con un salto visual sin gate.

**Salidas.** Estimador fusionado, contrato de coordenadas, pruebas de transform y replay de logs.

**Gate MP-04.** Ninguna corrección aceptada produce saltos fuera de umbral; al tapar la cámara el pose continúa con odometría y cambia explícitamente de calidad; al recuperar visión converge de forma acotada.

**Rollback.** Desactivar correcciones visuales y operar con pose odométrica marcada como degradada; conservar observaciones para análisis.

### MP-05 — Auto-aim robusto de torreta

**Objetivo.** Apuntar al goal correcto con odometría continua y corrección visual condicionada, dentro del arco seguro.

**Prerrequisitos.** MP-01, MP-04 y límites físicos validados.

**Trabajo.**

- Calcular bearing geométrico al goal de la alianza desde la pose fusionada.
- Transformarlo del chasis a la torreta considerando cero, offset, wrapping y arco permitido.
- Aplicar, si existe, un trim visual de bearing sólo para el tag de goal correcto: Blue ID 20, Red ID 24.
- Implementar los estados únicos `DISARMED`, `ARM_HOLD`, `ARMED_IDLE`, `TRACKING`, `READY`, `AT_LIMIT`, `ZERO_INVALID` y `FAULT`, con salida de motor definida en todos. Pose inválida y target perdido pertenecen al estado de targeting, no al mecanismo.
- Usar PID/FF limitado, tolerancia angular con dwell, límite de velocidad/potencia y anti-windup.
- Añadir zona de frenado antes de cada soft limit, contención exterior que sólo permita movimiento de regreso y pérdida explícita de `zeroValid` ante reset/brownout.
- Al pedir un ángulo fuera del arco: potencia cero, bloquear feed y avisar al conductor con telemetry/rumble para orientar el chasis.
- Detectar encoder implausible, comando en dirección bloqueada y discrepancia persistente; fallar cerrado.

**Salidas.** Controlador de torreta independiente de la cámara, telemetría de setpoint/medida/error/calidad y pruebas por estado.

**Gate MP-05.** Error estable ≤2.5° en puntos validados; nunca cruza soft limits; no se mueve desarmada; al perder visión continúa sólo si la pose sigue confiable; `end`/E-stop dejan potencia cero.

**Rollback.** `DEGRADED_FIXED_FORWARD` con torreta retenida en cero validado y conductor apuntando el chasis; nunca restaurar tracking sin límites.

### MP-06 — Shooter fijo, operador 2 e interlocks de feed

**Objetivo.** Producir un setpoint de RPM explicable y permitir corrección humana sin saltarse límites físicos ni readiness.

**Prerrequisitos.** MP-01 y distancia confiable de MP-04; mecanismo fijo y seguro.

**Trabajo.**

- Registrar que el hood ya fue retirado físicamente y eliminar su dependencia del futuro diseño activo; el código no cambia hasta que la fase correspondiente sea autorizada.
- Caracterizar shooter por distancia, voltaje y resultado, registrando RPM medida y no sólo target.
- Comparar regresión lineal, cuadrática de grado máximo 2 y tabla piecewise-linear. Elegir el modelo más simple que cumpla datos de validación retenidos.
- Definir clamp físico absoluto de RPM y slew rate.
- Durante commissioning manual, `gamepad2 A` arma/inicia RPM manual y `B` detiene shooter. `DPAD_UP/DOWN` cambia ±100 RPM por flanco y `X` restaura trim cero. Después de validar MP-06, `Y` cambia entre `AUTO_DISTANCE` y `MANUAL_RPM` con feedback inequívoco.
- El trim inicia en cero y se limita a ±500 RPM por match. No existe control de override; cualquier ampliación futura exige decisión y caracterización nuevas, sin alterar el clamp físico absoluto.
- Readiness exige sensor/RPM sanos dentro de tolerancia durante un dwell, cero/torreta listos, pose o modo degradado permitido, feeder sano, velocidades lineal y angular del chasis dentro de umbrales medidos y solicitud sostenida.
- `gamepad1 RB` solicita disparos repetidos mientras se sostiene. Cada pulso tiene duración máxima y cooldown `TBD-BLOCKING`, reevalúa readiness antes de comenzar y se corta inmediatamente al soltar o perder una condición crítica.
- En `DEGRADED_FIXED_FORWARD`, permitir sólo torreta en cero validado, RPM manual, solicitud sostenida, dwell y timeout. Si torreta no está armada o el encoder es implausible, no moverla ni alimentar.

**Salidas.** Modelo calibrado, controles del operador, interlock central y reporte de por qué está o no listo.

**Gate MP-06.** RPM dentro de ±100 durante al menos 250 ms antes de feed; trim nunca rebasa clamp absoluto; cero alimentación involuntaria; cada pulso/cooldown está acotado; al soltar RB o perder una condición crítica se corta el feeder en el primer ciclo y ≤50 ms.

**Rollback.** RPM manual dentro de límites y feed interlocked. No ofrecer un botón que energice feeder sin estado de seguridad.

### MP-07 — Integración y System Check único

**Objetivo.** Unir todos los subsistemas bajo una sola composición y ofrecer diagnóstico seguro, manteniendo separados los modos deliberados de commissioning de los modos de producción.

**Prerrequisitos.** MP-02 a MP-06 aceptados de forma individual.

**Trabajo.**

- Mover propiedad de torreta y Limelight a la composición principal; ningún OpMode los mapea por separado.
- Crear el único TeleOp con init por etapas: hardware, alianza, pose inicial, cámara, centro/armado de torreta y confirmación de ready.
- Implementar el único System Check jerárquico: sensores primero; movimiento sólo con permiso sostenido, potencia reducida, timeout, límites y abort global.
- Mantener visibles durante commissioning `MainTeleOp`, System Check, Shooter Tuning y las suites Pedro/Road Runner. Inventariar cada tuner, su actuator ownership, potencia, stop y E-stop; un tuner visible pero sin contrato seguro queda bloqueado para uso.
- Unificar telemetry en bloques: modo/alianza, pose/calidad, visión, torreta, shooter, readiness, feeder y faults.
- Añadir logging con timestamp, SHA/config y eventos de transición; no saturar el loop.
- Ensayar reinicio, desconexión de cámara, brownout simulado de software, sensor inválido, pérdida de pose y E-stop.

**Salidas.** Candidato integrado aún en rama de commissioning.

**Gate MP-07.** Un solo propietario por dispositivo; init fallido no mueve actuadores; todo fault tiene estado seguro y mensaje; System Check no puede mover sin autorización sostenida.

**Rollback.** Integrar por feature flags sólo durante commissioning. El release no conservará banderas que oculten caminos no probados.

### MP-08 — Programa de pruebas y aceptación

**Objetivo.** Demostrar funcionamiento repetible, fallas seguras y operación entendible antes de limpiar.

**Prerrequisitos.** MP-07 y checklist de seguridad física.

**Trabajo.** Ejecutar [el programa de pruebas](plan-maestro/05-programa-pruebas.md): análisis estático, unit tests, replay, robot deshabilitado, blocks, baja potencia, calibración de tiro, integración, fallas y scrimmage.

**Criterios iniciales de aceptación.**

- odometría: ≤2 in y ≤2° en ruta repetida acordada;
- torreta: error estable ≤2.5° sin cruzar límites;
- shooter: ±100 RPM durante 250 ms antes de feed;
- tiro estacionario: al menos 10 tiros por distancia y por sesión, con ≥9/10 en cada distancia de calibración;
- distancia intermedia/retenida: al menos 10 tiros por distancia y por sesión, con ≥8/10 sin combinar sesiones para ocultar una falla;
- cero movimiento de torreta antes de arm;
- cero feed no solicitado;
- stop/interrupción: comando cero en el primer ciclo del scheduler y ≤50 ms;
- operador puede identificar alianza, modo, trim y razón de bloqueo sin abrir Dashboard.

Los números son gates iniciales del proyecto, no especificaciones oficiales del juego. Si se cambian, registrar datos, razón y decisión.

**Gate MP-08.** Dos sesiones separadas cumplen criterios, sin finding crítico/alto abierto y con operadores capaces de ejecutar normal, degradado y E-stop.

**Rollback.** Volver al último candidato aceptado; no reducir el criterio para aprobar un build inestable sin una decisión documentada.

### MP-09 — Limpieza del candidato

**Objetivo.** Reducir superficie de error y ruido operativo conservando un rollback completo y verificable.

**Prerrequisitos.** MP-08 aceptado; autorización explícita para borrar código activo del branch principal.

**Trabajo.**

- Crear tag anotado `archive/pre-cleanup-YYYYMMDD`, registrar SHA, build y artefactos; publicar/verificar el tag remoto.
- Mantener una rama de commissioning para tuners complejos y el snapshot de Road Runner.
- Eliminar de `main`, no mover a un paquete Java `/archive`, los TeleOps, tuners, localizadores, VisionPortal viejo, hood y declaraciones obsoletas que ya no forman parte de producción. Los autónomos Road Runner legado se eliminan de la misma forma (ya lo hizo DEC-041 al abrir esta ventana de trabajo, antes de llegar a MP-09); no queda nada de ese stack para volver a borrar aquí salvo residuos que aparezcan en el inventario.
- Dejar exactamente un TeleOp de competencia, un System Check seguro, los autónomos Pedro Pathing aceptados por su gate MP-08 (DEC-041) y cero tuners/registrars extra.
- Quitar MeepMeep/Road Runner sólo después de comprobar que Pedro cubre lo necesario; no modificar `FtcRobotController` como limpieza incidental.
- Consolidar la versión de SDK ya resuelta por el proyecto en una sola fuente sólo como cambio revisado; no actualizar SDK, Gradle, AGP o Java por conveniencia.
- Medir builds frío/caliente, APK, instalación real y menú del Driver Station antes/después. Reportar datos aunque no mejoren.
- Finalizar [manual de operación](plan-maestro/07-manual-operacion.md) y runbook de rollback.

**Salidas.** Candidato de competencia pequeño, tag pre-cleanup recuperable, manual actualizado y matriz de cambios. Todavía no es release.

**Gate MP-09.** Build limpio, diff revisado, sólo dos OpModes de producción visibles, instalación y smoke test exitosos y tag recuperable en otro checkout. Este gate habilita MP-10, no el release.

**Rollback.** Crear rama desde el tag; no reinsertar archivos sueltos copiándolos sin historial.

### MP-10 — Revalidación posterior a limpieza y release

**Objetivo.** Demostrar que la limpieza no cambió comportamiento ni debilitó safety sobre el SHA exacto que se desplegará.

**Prerrequisitos.** MP-09 aceptado; candidato instalado desde un build limpio; configuración RC y hardware identificados.

**Trabajo.**

- Repetir T0–T10, las dos sesiones integradas, drills de E-stop y matriz de faults sobre el commit limpio.
- Confirmar menú del Driver Station, ownership, ausencia de caminos legacy y reconstrucción desde otro checkout.
- Crear y publicar `release/competition-YYYYMMDD` sólo después de aprobar toda la evidencia.

**Gate MP-10.** Dos sesiones separadas pasan en el SHA final, cero findings críticos/altos abiertos, build/deploy reproducible, manual firmado y tag final recuperable.

**Rollback.** Volver a `archive/pre-cleanup-YYYYMMDD` o al último candidato aceptado; no reducir criterios para declarar release.

## 6. Gates transversales de seguridad

Ningún MP que mueva hardware puede aprobarse sin verificar:

- nombre exacto del hardware y propietario único;
- dirección/signo y unidades;
- límites físicos y de software;
- clamp de potencia/RPM y timeout;
- requisito FTCLib del comando;
- salida en `end(interrupted)`, stop de OpMode y E-stop;
- comportamiento ante dato viejo, nulo, imposible o desconectado;
- prueba inicial sin ruedas en el piso o con mecanismo restringido;
- responsable con acceso inmediato a Stop/E-stop;
- evidencia guardada con SHA y configuración.

Compilar, simular o ver telemetry razonable no demuestra seguridad mecánica.

## 7. Riesgos principales del programa

| Riesgo | Consecuencia | Prevención / contención | Gate dueño |
|---|---|---|---|
| Convenciones de coordenadas incompatibles | Apuntar al lado equivocado o saltar pose | Contrato explícito y cuatro anclas por alianza | MP-04 |
| Cero/límites de torreta incorrectos | Golpe mecánico o daño de cables | Cero manual, armado sostenido, límites medidos, baja potencia | MP-01/05 |
| Limelight vieja o falsa | Corrección errónea de pose/bearing | Timestamp, ID, residual, latencia y gates | MP-03/04 |
| Odometría provisional | Distancia/RPM/aim equivocados | Calibración independiente antes de fusión | MP-02 |
| Sensor de shooter inválido tratado como RPM baja | Potencia máxima ante encoder/voltaje defectuoso | Health explícito y fallo cerrado a power cero | MP-01/06 |
| Feed directo desde bindings/adapters | Lanzamiento no solicitado | Interlock central y request sostenido | MP-01/06 |
| Limpiar antes de validar o no revalidar después | Pérdida de diagnóstico o regresión invisible | MP-09 bloqueado por MP-08 y revalidación MP-10 | MP-09/10 |
| Modos visibles sin clasificación | Operación confusa o tuner inseguro | Catálogo commissioning y release reducido | MP-00/07/09 |
| Build grande atribuido a fuentes equivocadas | Trabajo de limpieza sin beneficio | Benchmark de build, APK e instalación | MP-00/09 |

## 8. Evidencia mínima por entrega

Cada PR o sesión de commissioning debe adjuntar:

- SHA y rama;
- Robot Controller configuration y versión de firmware relevantes;
- archivos/constantes cambiados;
- diagrama o explicación del flujo de datos;
- unidades, límites, timeouts y stop paths;
- comandos de build/test y resultados;
- log/CSV/video cuando haya comportamiento físico;
- findings nuevos o cerrados con evidencia de retest;
- limitaciones pendientes y rollback.

## 9. Documentos del programa

- [01 — Auditoría del estado actual](plan-maestro/01-auditoria-estado-actual.md)
- [02 — Arquitectura objetivo](plan-maestro/02-arquitectura-objetivo.md)
- [03 — Auto-aim, Limelight y cancha](plan-maestro/03-auto-aim-limelight-y-cancha.md)
- [04 — Repositorios de referencia](plan-maestro/04-repositorios-referencia.md)
- [05 — Programa de pruebas](plan-maestro/05-programa-pruebas.md)
- [06 — Limpieza y release](plan-maestro/06-limpieza-y-release.md)
- [07 — Manual de operación](plan-maestro/07-manual-operacion.md)
- [08 — Guía de verificación de hardware](plan-maestro/08-guia-verificacion-hardware.md)
- [09 — Runbook de Paso 2: calibración de odometría](plan-maestro/09-runbook-paso2-odometria.md)
- [Hallazgos](plan-maestro/hallazgos.md)
- [Decisiones](plan-maestro/decisiones.md)
- [Contrato operativo de hardware](plan-maestro/contrato-hardware.md)
- [Handoff de la tarea](plan-maestro/handoff-task.md)
- [Plan de handoffs MP-01, MP-02 y MasterPlan](plan-maestro/handoff-plan-MP01-MP02-MasterPlan.md)
- [Plan paralelo de 20h — pista Software y pista Tuning](plan-maestro/plan-paralelo-20h.md)
- [Plan final antes del Premier](plan-maestro/plan-final-antespremier.md)
- [Hallazgos críticos históricos](critical-findings.md), preservado sin editar

## 10. Definición de terminado del programa

El programa termina cuando el release de competencia:

1. inicializa de forma repetible y no mueve la torreta antes de confirmación;
2. conoce alianza y pose inicial de manera directa y visible;
3. mantiene pose con tres dead wheels y usa Limelight sólo como observación validada;
4. apunta dentro de límites o pide ayuda al conductor sin ordenar giro autónomo del chasis;
5. selecciona RPM por distancia y permite trim dentro de límites físicos sin override de gamepad;
6. alimenta sólo con request sostenido, readiness verificable y pulsos acotados;
7. entra a estados seguros ante pérdida de cámara, pose, encoder, comando o OpMode;
8. cumple el programa de pruebas en dos sesiones separadas sobre el SHA posterior a limpieza;
9. expone únicamente el TeleOp y System Check aprobados;
10. puede reconstruirse o revertirse desde Git y puede ser explicado por el equipo.

Hasta entonces, el estado correcto es **commissioning**, no “listo para competencia”.
