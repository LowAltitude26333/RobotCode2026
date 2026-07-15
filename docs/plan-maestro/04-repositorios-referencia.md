# 04 — Repositorios de referencia

> Estado: investigación inicial completada; revisión de licencia/API pendiente antes de reutilizar código
> Baseline de referencia: `main` en `f91af18`
> Última actualización: 2026-07-15
> Alcance: equipos públicos con torreta/shooter/feeder, mecanum, Limelight u odometría
> Responsable sugerido: líder de software y estudiante que implementará cada patrón
> Fuente de verdad: el commit y licencia del repositorio externo inspeccionado; una descripción de robot o resultado competitivo no prueba calidad de código.

## 1. Propósito

La investigación externa sirve para descubrir patrones, preguntas y fallas ya resueltas por otros equipos. No sirve para copiar un robot completo. Nuestro hardware, arco de torreta, cámara fija, shooter de ángulo fijo, versión de SDK y política de seguridad son distintos.

Se priorizaron repositorios públicos con parte de este arquetipo:

```text
Intake -> Feeder/Indexer -> Flywheel Shooter -> Turret
                 + Mecanum drive
                 + Odometry
                 + Limelight / AprilTags
```

Ningún repositorio encontrado es una coincidencia perfecta y verificada de todas las decisiones. La utilidad está en comparar componentes y contratos.

## 2. Rúbrica de evaluación

Cada fuente se valora con estas preguntas:

1. **Provenance:** ¿se puede asociar razonablemente con un equipo y temporada?
2. **Licencia:** ¿existe permiso explícito de reutilización y qué obligaciones tiene?
3. **Vigencia:** ¿qué SDK/librerías/temporada usa?
4. **Arquetipo:** ¿contiene turret, flywheel, feeder, intake, mecanum, odometría y visión?
5. **Ownership:** ¿cada hardware device tiene dueño único?
6. **Fusión:** ¿visión corrige odometría con gates o sólo usa `tx`?
7. **Seguridad:** ¿hay limits, timeouts, readiness y stop paths?
8. **Operabilidad:** ¿init, telemetría, manual y diagnósticos son entendibles?
9. **Testabilidad:** ¿hay logs, simulación, replay o procedimientos?
10. **Compatibilidad:** ¿el patrón cabe en FTCLib/Pedro/SDK del repo sin reescritura?

Antes de copiar cualquier fragmento, registrar URL de archivo, commit, licencia, autoría, cambios necesarios y revisión estudiantil.

## 3. Matriz resumida

| Fuente | Coincidencia útil | Fortalezas observadas | Riesgos/diferencias | Uso recomendado |
|---|---|---|---|---|
| [Iron Reign FTC 6832](https://github.com/IronReign/FtcRobotControllerIronReign) | Turret, launcher, visión, odometría/fallback, documentación y diagnostics | Estados de targeting/readiness, manejo de dropout, setup notes, drivers manual, diagnóstico jerárquico | Drivetrain tank en la arquitectura reciente; hardware y políticas de degradado distintas | Referencia primaria de robustness, telemetría y operación; adaptar, no portar drive. |
| [RevAmped Decode V2](https://github.com/junkjunk123/RevAmped-Decode-V2) | Pedro mecanum, intake, feeder, flywheel, turret, Limelight | Arquetipo técnico muy cercano, thread de tracking, matemática de proyectil, state machine | Cuenta personal/provenance y licencia por verificar; hood variable; fusión visión→odo aparece comentada; shooting-on-move agrega complejidad | Estudiar separación de estados y wiring Pedro; no copiar fusión ni balística sin validación. |
| [HyperionBots Worlds Repository](https://github.com/Ashley904/HyperionBotsWorldsRepository) | Mecanum/Pedro, Limelight, turret, shooter, spindexer | Selector de alianza en init, lookup de RPM/hood, pipeline integrado | Cuenta personal; usa auto-heading/direct `tx` en caminos; hood variable; no demuestra fusión robusta | Referencia de operator flow/tablas; rechazar dependencia directa de `tx`. |
| [FTC SDK oficial](https://github.com/FIRST-Tech-Challenge/FtcRobotController) | API oficial de hardware y samples | Fuente primaria para `Limelight3A`, lifecycle y compatibilidad | Samples son demostraciones, no arquitectura de competencia | Fuente obligatoria para API; envolver con safety/quality propias. |
| [Leviathan Robotics 25667](https://leviathanrobotics.org/) | Intake/spindexer/flywheel/turret/Limelight y regresión descritos públicamente | Arquetipo y enfoque de calibración valiosos | No se verificó repo público de código; cámara reportada en torreta y stack Road Runner | Referencia conceptual secundaria, nunca fuente de código. |

Las páginas oficiales de eventos ayudan a confirmar la identidad de equipos, no a certificar el código: [FTC 6832](https://ftc-events.firstinspires.org/2025/team/6832), [FTC 12808](https://ftc-events.firstinspires.org/team/12808) y [FTC 18011](https://ftc-events.firstinspires.org/team/18011).

## 4. Iron Reign — FTC 6832

### 4.1 Por qué importa

El repositorio de la organización Iron Reign contiene una implementación reciente `lebot2` y documentación poco común en repos de equipos. Los documentos observados incluyen conceptos equivalentes a:

- `TargetingOverhaulPRD.md`;
- `CompetitionSetupNotes.md`;
- `DriversManual.md`;
- `Lebot2Diagnostics.java`.

Su valor principal no es una constante ni un algoritmo concreto, sino el tratamiento de targeting como sistema operable: calidad de lock, readiness, transición ante pérdida breve de visión, fallback, inicialización explícita y diagnósticos.

### 4.2 Patrones a adoptar

- Una solución de targeting con estado/calidad, no sólo un número.
- Telemetry que explica por qué no se puede disparar.
- Tolerancia diferenciada a dropout corto frente a pérdida sostenida.
- Procedimientos de setup y drivers manual versionados junto al código.
- Diagnóstico jerárquico que prueba sensores antes que actuadores.
- Registrar y ensayar el modo de degradación como parte del sistema.

### 4.3 Patrones a adaptar

- Fallback de odometría: llevarlo al contrato Pedro + cámara fija y nuestros gates.
- Readiness/lock: sustituir sus sensores, thresholds y drivetrain por los nuestros.
- Diagnóstico: conservar hold-to-run, límites y abort; no copiar hardware mappings.

### 4.4 Patrones a rechazar sin evidencia adicional

- Cualquier supuesto de tank drive.
- Cualquier permiso de feed degradado que sea menos estricto que nuestro contrato.
- Límites/powers/IDs de su robot.
- Clases completas si la licencia no permite reutilización o si no se registra atribución.

## 5. RevAmped Decode V2 — probable FTC 12808

### 5.1 Por qué importa

Es la coincidencia funcional más cercana encontrada: Pedro + mecanum, intake, feeder, flywheel, turret y Limelight. Incluye una forma de `TrackingThread`, matemática de projectile y una máquina de estados para el mecanismo.

### 5.2 Patrones a adoptar conceptualmente

- Desacoplar tracking de los controles del operador.
- Usar estados explícitos para coordinar mecanismos.
- Centralizar las secuencias intake→feeder→shooter.
- Revisar cómo adapta Pedro a su hardware y qué telemetría conserva.

### 5.3 Patrones que requieren cautela

- El código observado para corregir odometría desde visión aparece comentado en partes; no es prueba de una fusión activa y robusta.
- El hood variable contradice nuestro ángulo fijo.
- La balística y shooting while moving agregan dimensiones que no están en el alcance inicial.
- Un thread separado puede introducir carreras con el loop FTC. Antes de adoptar, justificar sincronización, timestamps y lifecycle; preferir periodic no bloqueante si cumple frecuencia.
- La cuenta del repo es personal. Confirmar autoría/equipo y licencia en el commit seleccionado.

### 5.4 Decisión

Usar como catálogo de patrones y comparación de interfaces. No copiar `TrackingThread`, ecuaciones ni constants hasta terminar una revisión de concurrencia, licencia, unidades y hardware.

## 6. HyperionBots Worlds Repository — FTC 18011

### 6.1 Por qué importa

Combina mecanum/Pedro, Limelight, torreta, shooter y spindexer. Se observaron patrones de selector de alianza en init y lookup de RPM/hood, ambos cercanos a necesidades de operación/calibración.

### 6.2 Patrones a adoptar

- Selección directa de alianza antes del start, con feedback claro.
- Tabla/lookup calibrable como candidato de modelo piecewise-linear.
- Coordinación explícita entre shooter e indexer.

### 6.3 Patrones a rechazar o rediseñar

- Auto-heading del chasis: nuestro target indica que el conductor gira manualmente si la torreta queda fuera de arco.
- `tx` directo como única base de aim: nuestra odometría es la propagación principal y la visión se gatea.
- Hood lookup: el hood será mecánicamente fijo.
- Constantes o poses sin reconciliar el marco Pedro/SDK.

## 7. FTC SDK oficial

### 7.1 Uso obligatorio

El sample oficial de `SensorLimelight3A` demuestra el patrón básico para:

- `hardwareMap.get(Limelight3A.class, "limelight")`;
- seleccionar pipeline;
- arrancar el dispositivo;
- consultar el resultado más reciente;
- comprobar validez;
- acceder a botpose, offsets y fiducials disponibles;
- detener el dispositivo.

La Javadoc del SDK resuelto es la autoridad de firmas. El sample no incluye:

- ownership command-based;
- freshness gates;
- fusión con odometría;
- limits de torreta;
- readiness de feeder;
- E-stop/cleanup global.

Por eso se envuelve en `LimelightSubsystem` en lugar de copiar el sample dentro del TeleOp.

## 8. Leviathan Robotics — FTC 25667

El sitio del equipo describe un robot con intake, spindexer, flywheel, turret, Limelight y una relación/regresión de disparo. Sirve para confirmar que el arquetipo y el uso de datos empíricos son viables en FTC.

No se verificó un repositorio público de código asociado durante esta investigación. Además, la cámara parece montada de forma distinta y el stack indicado es Road Runner. Por tanto:

- no atribuirle ninguna implementación concreta;
- no copiar snippets secundarios;
- usarlo sólo como pregunta de diseño: cómo documentan calibración, qué variables registran y cómo separan turret/shooter/indexer.

## 9. Matriz adoptar / adaptar / rechazar

| Tema | Adoptar | Adaptar | Rechazar inicialmente |
|---|---|---|---|
| Targeting | Estado/calidad/readiness | Dropout y fallback a nuestros gates | `tx` → motor directo. |
| Odometría | Provider desacoplado | Adapters Pedro del arquetipo | Dos stacks activos en producción. |
| Visión | Wrapper y lifecycle oficial | Field pose/trim según cámara fija | Copiar pipeline/field map ajeno. |
| Torreta | Soft limits, armed state, error/dwell | PID/FF según mecanismo medido | Powers/ticks de otro robot. |
| Shooter | Modelo empírico versionado | Lookup/regresión con nuestros datos | Hood variable/shoot-on-move en primera versión. |
| Feeder | State machine e interlock | Anti-jam tras medir corriente/tiempo | Botón que energiza sin readiness. |
| Operación | Selector init, manuals, diagnostics | Layout de controles al contrato local | Muchos TeleOps como menú de pruebas. |
| Degradado | Modo explícito y entrenado | Condiciones/timeout según robot | Degradación silenciosa o fire permisivo. |
| Concurrencia | Datos inmutables/timestamps | Thread sólo si se demuestra necesidad | Thread sin ownership/lifecycle claro. |

## 10. Checklist antes de reutilizar código

Para cada fragmento externo:

- [ ] URL exacta y commit fijado.
- [ ] Autor/equipo razonablemente verificados.
- [ ] Licencia encontrada y compatible; atribución preparada.
- [ ] SDK/librerías y Java comparados con este repo.
- [ ] Nombres de hardware eliminados/reemplazados sólo con contrato local.
- [ ] Unidades y coordenadas entendidas.
- [ ] Ownership, requirements, timeouts, limits y stop paths revisados.
- [ ] No existe loop bloqueante ni thread huérfano.
- [ ] El estudiante puede explicar el algoritmo sin depender del código externo.
- [ ] Tests locales escritos antes de habilitar hardware.
- [ ] Crédito/licencia conservados donde corresponda.

Si no hay licencia explícita, tratar el código como referencia de lectura y reimplementar el concepto desde fuentes primarias y entendimiento propio; no copiarlo.

## 11. Preguntas para una segunda ronda de investigación

Sólo investigar más si desbloquea una fase concreta:

- ¿Qué equipos publican replay/logging de Limelight+odometría en FTC SDK 11?
- ¿Existen implementaciones Pedro con tres dead wheels y cámara fija que documenten transforms?
- ¿Qué thresholds de freshness se derivan de una latencia medida comparable, no sólo de constantes ajenas?
- ¿Cómo presentan a operadores la diferencia entre “shooter at speed” y “shot ready”?
- ¿Qué herramientas de System Check pueden incorporarse sin aumentar el APK normal?

La respuesta útil debe terminar en una decisión/test local, no en acumular repositorios.

## 12. Entregable de investigación por patrón

Cuando se adopte un patrón, agregar a [decisiones.md](decisiones.md):

- problema local;
- alternativas consideradas;
- fuente externa y commit;
- partes adoptadas y rechazadas;
- licencia/atribución;
- adaptación a nuestros estados/unidades;
- tests y rollback.

Así la influencia externa queda auditable y el equipo conserva propiedad intelectual y comprensión técnica.
