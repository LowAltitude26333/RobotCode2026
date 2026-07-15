# Plan maestro de robustecimiento del robot

> Estado: aprobado para ejecución por fases; todavía no autoriza despliegue al robot
> Baseline documental: `main` en `f91af18`
> Última actualización: 2026-07-15
> Alcance: arquitectura, torreta, Limelight 3A, odometría, shooter, controles, pruebas y limpieza
> Responsables sugeridos: líder de software, responsable mecánico/eléctrico, operador 1, operador 2 y responsable de pruebas
> Fuente de verdad: antes de ejecutar una fase se debe volver a inspeccionar la rama y el hardware; este documento describe el baseline indicado, no garantiza el estado físico del robot.

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
- cero autónomos en el artefacto final de competencia.

El auto-aim no dependerá exclusivamente de una cámara. La odometría mantendrá continuamente la pose y el ángulo geométrico al goal; Limelight aportará observaciones frescas y validadas para corregir pose y, cuando sea apropiado, ajustar el bearing final. Si ambas fuentes dejan de ser confiables, el robot entrará en un modo degradado deliberado, limitado y visible, nunca en una degradación silenciosa.

Este plan es un programa de trabajo, no una especificación ya validada. Toda constante física, nombre de hardware, orientación, relación mecánica y límite marcado como pendiente debe medirse en el robot antes de habilitar movimiento.

## 2. Decisiones no negociables del diseño aprobado

1. Pedro Pathing será el stack primario de localización y movimiento. Road Runner se conservará como rollback en un snapshot Git previo a la limpieza, no como segunda arquitectura activa.
2. La localización física utilizará tres dead wheels. Sus offsets, signos y resolución se medirán; no se reutilizarán valores provisionales por conveniencia.
3. Limelight 3A estará fija al chasis. Su nombre propuesto es `limelight`, sujeto a confirmar la configuración del Robot Controller.
4. La torreta se centra manualmente, tiene arco limitado por cables/mecánica y no puede asumir rotación continua. No se moverá hasta quedar armada explícitamente.
5. El seguimiento podrá operar continuamente después del armado, pero respetará límites físicos y calidad de estimación. Si el target queda fuera del arco, se detendrá y pedirá al conductor orientar el chasis; el software no girará el drivetrain automáticamente.
6. El shooter tiene un solo motor y ángulo fijo. La selección automática de RPM dependerá de distancia mediante un modelo empírico validado.
7. El operador 2 podrá aplicar trim de RPM en pasos de 100, restaurarlo, cambiar entre RPM automática y manual, y activar un override limitado. Siempre existirá un clamp absoluto de RPM validado físicamente.
8. El feeder sólo actuará mientras exista una solicitud sostenida y se cumplan interlocks de disparo. Soltar el control, interrumpir el comando, detener el OpMode o activar E-stop debe ordenar potencia cero.
9. La alianza se seleccionará directamente durante init: `X = BLUE`, `B = RED`. Se mostrará claramente y se bloqueará al iniciar; no se alternará accidentalmente durante el match.
10. El artefacto final tendrá exactamente un TeleOp de competencia y un System Check seguro. Tuning, prototipos, autónomos y alternativas vivirán en historial/tag/rama de commissioning, no en el APK normal.
11. Ningún cambio de fase se considera terminado sólo porque compile. Cada gate exige evidencia reproducible y validación física proporcional al riesgo.

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
MP-08 aceptado --> MP-09 Limpieza y release de competencia
```

La limpieza es deliberadamente la última fase: borrar alternativas antes de demostrar el reemplazo elimina el rollback que más se necesita durante commissioning.

## 5. Programa de trabajo

### MP-00 — Congelar y auditar el baseline

**Objetivo.** Convertir la fotografía actual del repo en una lista trazable de riesgos, dueños de hardware y comportamientos activos.

**Entradas.** `main`, `AGENTS.md`, `docs/critical-findings.md`, Robot Controller configuration export y robot disponible para inspección sin habilitar.

**Trabajo.**

- Repetir inventario de OpModes registrados, incluidos registradores dinámicos.
- Recorrer la composición real desde `MainTeleOp` hasta subsistemas, comandos, localizador, constantes y hardware map.
- Clasificar cada finding histórico como corregido, contenido, parcial, reemplazado o vigente.
- Confirmar que no haya dos componentes activos mapeando el mismo actuador.
- Registrar SHA, dependencias resueltas, tiempo de build frío/caliente y tamaño de APK como baseline, sin prometer que reducir archivos Java reducirá por sí solo el deploy.
- Crear una tabla de hechos físicos todavía desconocidos.

**Salidas.** [Auditoría del estado actual](plan-maestro/01-auditoria-estado-actual.md) revisada y configuración física adjunta a la bitácora del equipo.

**Gate MP-00.** Ningún finding crítico queda sin dueño, siguiente acción o condición de contención. El equipo puede señalar cuál OpMode controla cada motor en el baseline.

**Rollback.** No aplica: es sólo inspección y documentación.

### MP-01 — Contrato de hardware y bloqueadores críticos

**Objetivo.** Hacer que el robot pueda inicializarse, armarse y detenerse de forma determinista antes de agregar auto-aim.

**Prerrequisitos.** MP-00 aceptado; robot elevado o mecanismo desconectado según la prueba.

**Trabajo.**

- Resolver el registro de cleanup de `VisionPortal` antes de su construcción; la auditoría detectó una referencia ligada a `null` con riesgo de fallar en init.
- Sustituir el chequeo instantáneo del chord de armado por una máquina de estado en `init_loop`: `gamepad2 START+BACK` sostenido durante 1 s, con progreso, confirmación y posibilidad de cancelar antes de armar.
- Confirmar nombre, dirección, signo de encoder, ticks por arco permitido, centro físico, margen contra hard stops y ruta de cables de la torreta.
- Confirmar un solo motor de shooter, ticks/rev efectivos, relación externa, inversión, RPM máxima segura y comportamiento de zero power.
- Confirmar el nombre exacto `kickerM otor` antes de renombrar únicamente el concepto de software a feeder.
- Hacer que E-stop y `RobotSafety.stopAll()` estén disponibles desde todo OpMode de producción/diagnóstico, no sólo desde un TeleOp.
- Auditar todos los adapters `Action` y comandos secuenciales para que interrupción y stop ordenen cero a drive, intake, feeder, shooter y torreta.

**Salidas.** Contrato de hardware versionado, init seguro y pruebas de apagado. No se cambia todavía el stack de localización.

**Gate MP-01.** Cero movimiento de torreta antes de armado; todo actuador se detiene como máximo en el siguiente ciclo del scheduler tras soltar/interrumpir/parar; E-stop es accesible y probado a potencia reducida.

**Rollback.** Revertir el PR del paquete. No eliminar límites ni restaurar un camino que deje motores energizados.

### MP-02 — Pedro Pathing y odometría de tres pods

**Objetivo.** Obtener una única pose continua, calibrada y con calidad observable usando Pedro Pathing.

**Prerrequisitos.** MP-01; medidas físicas de pods y direcciones; superficie y trayectoria de prueba repetible.

**Trabajo.**

- Crear una rama de migración sin borrar Road Runner.
- Medir y documentar diámetro efectivo, ticks por revolución, offsets de cada pod respecto al centro de giro, signo de cada encoder y orientación de IMU.
- Corregir los valores provisionales y asignaciones ambiguas de `pedroPathing/Constants.java`.
- Elegir y documentar un marco de coordenadas del robot y otro de cancha; unidades internas explícitas.
- Integrar el pose provider detrás de una interfaz independiente de Pedro para que auto-aim no importe APIs del localizador.
- Calibrar forward/lateral/turn, compensación de track width y estabilidad de heading.
- Marcar calidad `UNINITIALIZED`, `GOOD`, `DEGRADED` o `INVALID` con razones y timestamps.

**Salidas.** `PoseProvider` primario, constants medidas, telemetría de pose/calidad y logs de repetibilidad.

**Gate MP-02.** En una ruta repetida de commissioning, error final no mayor a 2 in y 2°; el equipo acepta o ajusta estos umbrales con evidencia antes de auto-aim. Ningún encoder de drive sustituye accidentalmente un dead wheel.

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
- Probar coexistencia sólo si todavía se necesita una webcam durante migración; el objetivo final elimina el viejo `VisionPortal` de tracking.

**Salidas.** Fuente de observaciones desacoplada del control de torreta y herramienta de diagnóstico sin movimiento.

**Gate MP-03.** Init y stop pueden repetirse sin excepción ni recurso abierto; pérdida/desconexión de cámara no mueve actuadores; timestamps/latencia se ven en telemetry/log.

**Rollback.** Deshabilitar el consumidor y conservar odometría/manual. El wrapper debe fallar cerrado.

### MP-04 — Cancha, coordenadas y fusión de pose

**Objetivo.** Relacionar pose de Pedro, pose reportada por Limelight, goal de alianza y punto de mira en un contrato matemático único.

**Prerrequisitos.** MP-02 y MP-03 aceptados; CAD/manual oficial revisados.

**Trabajo.**

- Documentar el origen, ejes, sentido positivo, unidad y heading de cada marco: cancha oficial/SDK, Pedro, robot, cámara y torreta.
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
- Implementar estados `DISARMED`, `TRACKING`, `AT_LIMIT`, `POSE_INVALID`, `TARGET_LOST` y `FAULT`, con salida de motor definida en todos.
- Usar PID/FF limitado, tolerancia angular con dwell, límite de velocidad/potencia y anti-windup.
- Al pedir un ángulo fuera del arco: potencia cero, bloquear feed y avisar al conductor con telemetry/rumble para orientar el chasis.
- Detectar encoder implausible, comando en dirección bloqueada y discrepancia persistente; fallar cerrado.

**Salidas.** Controlador de torreta independiente de la cámara, telemetría de setpoint/medida/error/calidad y pruebas por estado.

**Gate MP-05.** Error estable ≤2.5° en puntos validados; nunca cruza soft limits; no se mueve desarmada; al perder visión continúa sólo si la pose sigue confiable; `end`/E-stop dejan potencia cero.

**Rollback.** Modo manual limitado de commissioning o `DEGRADED_FIXED_FORWARD`; nunca restaurar tracking sin límites.

### MP-06 — Shooter fijo, operador 2 e interlocks de feed

**Objetivo.** Producir un setpoint de RPM explicable y permitir corrección humana sin saltarse límites físicos ni readiness.

**Prerrequisitos.** MP-01 y distancia confiable de MP-04; mecanismo fijo y seguro.

**Trabajo.**

- Quitar del diseño activo la dependencia de hood variable después de confirmar que los servos fueron removidos físicamente.
- Caracterizar shooter por distancia, voltaje y resultado, registrando RPM medida y no sólo target.
- Comparar regresión lineal, cuadrática de grado máximo 2 y tabla piecewise-linear. Elegir el modelo más simple que cumpla datos de validación retenidos.
- Definir clamp físico absoluto de RPM y slew rate.
- Implementar `AUTO_DISTANCE` y `MANUAL_RPM`; el cambio se solicita con `gamepad2 Y` y se confirma visualmente.
- `gamepad2 DPAD_UP/DOWN`: trim ±100 RPM por pulsación; `X`: trim cero. El trim inicia en cero y normalmente se limita a ±500 RPM por match.
- Mantener `RPM_TRIM_LIMIT_ENABLED=true`. Si un responsable lo desactiva para un override deliberado, el target aún queda dentro del rango físico validado y la pantalla muestra `OVERRIDE`.
- Readiness exige RPM dentro de tolerancia durante un dwell, torreta lista, pose/modo permitido, feeder sin fault y solicitud sostenida.
- Mantener el request del conductor en `gamepad1 RB` salvo decisión explícita posterior. Soltarlo detiene feeder inmediatamente.
- En `DEGRADED_FIXED_FORWARD`, permitir sólo torreta en cero validado, RPM manual, solicitud sostenida, dwell y timeout. Si torreta no está armada o el encoder es implausible, no moverla ni alimentar.

**Salidas.** Modelo calibrado, controles del operador, interlock central y reporte de por qué está o no listo.

**Gate MP-06.** RPM dentro de ±100 durante al menos 250 ms antes de feed; trim/override nunca rebasa clamp absoluto; cero alimentación involuntaria; al soltar RB o perder una condición crítica se corta el feeder.

**Rollback.** RPM manual dentro de límites y feed interlocked. No ofrecer un botón que energice feeder sin estado de seguridad.

### MP-07 — Integración y System Check único

**Objetivo.** Unir todos los subsistemas bajo una sola composición y ofrecer diagnóstico seguro sin poblar el Driver Station de modos de prueba.

**Prerrequisitos.** MP-02 a MP-06 aceptados de forma individual.

**Trabajo.**

- Mover propiedad de torreta y Limelight a la composición principal; ningún OpMode los mapea por separado.
- Crear el único TeleOp con init por etapas: hardware, alianza, pose inicial, cámara, centro/armado de torreta y confirmación de ready.
- Implementar el único System Check jerárquico: sensores primero; movimiento sólo con permiso sostenido, potencia reducida, timeout, límites y abort global.
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
- tiro estacionario: ≥9/10 en cada distancia de calibración soportada;
- distancia intermedia/retenida: ≥8/10;
- cero movimiento de torreta antes de arm;
- cero feed no solicitado;
- stop/interrupción: potencia cero como máximo en el siguiente ciclo del scheduler;
- operador puede identificar alianza, modo, trim y razón de bloqueo sin abrir Dashboard.

Los números son gates iniciales del proyecto, no especificaciones oficiales del juego. Si se cambian, registrar datos, razón y decisión.

**Gate MP-08.** Dos sesiones separadas cumplen criterios, sin finding crítico/alto abierto y con operadores capaces de ejecutar normal, degradado y E-stop.

**Rollback.** Volver al último candidato aceptado; no reducir el criterio para aprobar un build inestable sin una decisión documentada.

### MP-09 — Limpieza, release y transferencia

**Objetivo.** Reducir superficie de error y ruido operativo conservando un rollback completo y verificable.

**Prerrequisitos.** MP-08 aceptado; autorización explícita para borrar código activo del branch principal.

**Trabajo.**

- Crear tag anotado `archive/pre-cleanup-YYYYMMDD`, registrar SHA, build y artefactos; publicar/verificar el tag remoto.
- Mantener una rama de commissioning para tuners complejos y el snapshot de Road Runner.
- Eliminar de `main`, no mover a un paquete Java `/archive`, los TeleOps, autos, tuners, localizadores, VisionPortal viejo, hood y declaraciones obsoletas que ya no forman parte de producción.
- Dejar exactamente un TeleOp de competencia, un System Check seguro y cero autónomos/tuners registrados.
- Quitar MeepMeep/Road Runner sólo después de comprobar que Pedro cubre lo necesario; no modificar `FtcRobotController` como limpieza incidental.
- Consolidar la versión de SDK ya resuelta por el proyecto en una sola fuente sólo como cambio revisado; no actualizar SDK, Gradle, AGP o Java por conveniencia.
- Medir builds frío/caliente, APK, instalación real y menú del Driver Station antes/después. Reportar datos aunque no mejoren.
- Finalizar [manual de operación](plan-maestro/07-manual-operacion.md) y runbook de rollback.

**Salidas.** Release de competencia pequeño, tag recuperable, manual aprobado y matriz de evidencia.

**Gate MP-09.** Build limpio, diff revisado, sólo dos OpModes de producción visibles, instalación y smoke test exitosos, tag recuperable en otro checkout y checklist firmado.

**Rollback.** Crear rama desde el tag; no reinsertar archivos sueltos copiándolos sin historial.

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
| Override del operador sin límites | Overspeed o tiro inseguro | Clamp absoluto inalterable e indicador `OVERRIDE` | MP-06 |
| Feed directo desde bindings/adapters | Lanzamiento no solicitado | Interlock central y request sostenido | MP-01/06 |
| Limpiar antes de validar | Pérdida de diagnóstico y rollback | MP-09 bloqueado por MP-08 y tag verificado | MP-09 |
| Demasiados modos/caminos alternos | Operación confusa y fixes inconsistentes | Una composición, un TeleOp y un diagnóstico | MP-07/09 |
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
- [Hallazgos](plan-maestro/hallazgos.md)
- [Decisiones](plan-maestro/decisiones.md)
- [Hallazgos críticos históricos](critical-findings.md), preservado sin editar

## 10. Definición de terminado del programa

El programa termina cuando el release de competencia:

1. inicializa de forma repetible y no mueve la torreta antes de confirmación;
2. conoce alianza y pose inicial de manera directa y visible;
3. mantiene pose con tres dead wheels y usa Limelight sólo como observación validada;
4. apunta dentro de límites o pide ayuda al conductor sin ordenar giro autónomo del chasis;
5. selecciona RPM por distancia y permite trim/override dentro de límites físicos;
6. alimenta sólo con request sostenido y readiness verificable;
7. entra a estados seguros ante pérdida de cámara, pose, encoder, comando o OpMode;
8. cumple el programa de pruebas en dos sesiones separadas;
9. expone únicamente el TeleOp y System Check aprobados;
10. puede reconstruirse o revertirse desde Git y puede ser explicado por el equipo.

Hasta entonces, el estado correcto es **commissioning**, no “listo para competencia”.
