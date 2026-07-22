# Registro de decisiones

> Estado: registro vivo de decisiones de producto/arquitectura
> Baseline histórico inicial: `main` en `f91af18`; baseline vigente: `origin/main@b5a134260456565df9d0295722ebecad900f21b4`
> Última actualización: 2026-07-22
> Alcance: elecciones aprobadas, alternativas, consecuencias y condiciones de revisión
> Responsable sugerido: líder técnico y responsables nombrados en cada decisión
> Fuente de verdad: la decisión más reciente con estado `ACCEPTED`; cambios importantes requieren nueva entrada, no reescribir silenciosamente la historia.

## 1. Estados

| Estado | Significado |
|---|---|
| `PROPOSED` | Requiere aprobación/evidencia. |
| `ACCEPTED` | Guía la implementación. |
| `VALIDATING` | Aprobada condicionalmente; tests determinarán si permanece. |
| `SUPERSEDED` | Reemplazada por otra decisión enlazada. |
| `REJECTED` | Considerada y descartada. |

## 2. Índice

| ID | Estado | Decisión | Revisión disparada por |
|---|---|---|---|
| DEC-001 | ACCEPTED | Documentación en índice maestro + anexos técnicos | Cambio de estructura del programa. |
| DEC-002 | SUPERSEDED | Pedro primario sólo como proveedor de pose | Reemplazada por DEC-034. |
| DEC-003 | ACCEPTED | Tres dead wheels | Cambio físico del localizador. |
| DEC-004 | ACCEPTED | Limelight 3A fija al chasis | Montaje no viable/oclusiones demostradas. |
| DEC-005 | ACCEPTED | Auto-aim odometría-first con visión gated | Logs demuestran insuficiencia. |
| DEC-006 | ACCEPTED | Goal Blue 20 / Red 24 por selección directa | Revisión oficial de juego. |
| DEC-007 | ACCEPTED | Una TeleOp para ambas alianzas | Requisito estratégico cambia. |
| DEC-008 | ACCEPTED | Torreta cero manual, arco limitado y arm hold | Sensor de home físico aprobado en el futuro. |
| DEC-009 | ACCEPTED | Tracking continuo después de arm | Pruebas muestran inestabilidad/consumo. |
| DEC-010 | ACCEPTED | Fuera de arco guía driver, no auto-turn | Requisito explícito del equipo cambia. |
| DEC-011 | ACCEPTED | Un motor shooter, ángulo fijo, sin hood activo | Hardware cambia. |
| DEC-012 | VALIDATING | RPM por distancia: modelo empírico más simple | Dataset/gates seleccionan modelo. |
| DEC-013 | ACCEPTED | Operador 2 trim ±100, reset X, modo Y | Conflicto final de bindings. |
| DEC-014 | SUPERSEDED | Trim normal ±500 y override limitado | Reemplazada por DEC-030. |
| DEC-015 | ACCEPTED | Feeder es kicker actual; request sostenido + interlock | Diseño mecánico cambia. |
| DEC-016 | ACCEPTED | `DEGRADED_FIXED_FORWARD` deliberado | Safety testing lo invalida. |
| DEC-017 | ACCEPTED | Chord START+BACK state-aware de 1 s | Human-factors testing pide otro control. |
| DEC-018 | ACCEPTED | Un owner/composition root | Arquitectura completa cambia formalmente. |
| DEC-019 | SUPERSEDED | Un TeleOp + un System Check, cero autos final | Reemplazada por DEC-026. |
| DEC-020 | SUPERSEDED | Snapshot Git y borrar legado de main al final | Reemplazada por DEC-032. |
| DEC-021 | ACCEPTED | Sistema y documentación técnica en español | Necesidad del equipo cambia. |
| DEC-022 | ACCEPTED | No actualizar toolchain/SDK incidentalmente | Upgrade plan separado aprobado. |
| DEC-023 | ACCEPTED | Métricas, no promesas, para velocidad de deploy | Nueva evidencia/objetivo. |
| DEC-024 | ACCEPTED | Fuentes externas son patrones, con gate de licencia | Política legal/educativa cambia. |
| DEC-025 | ACCEPTED | `origin/main@b5a1342` es la fuente documental vigente | El source ref seleccionado cambia. |
| DEC-026 | ACCEPTED | Tuners visibles en commissioning; menú mínimo en release | Todos los tuners ya fueron validados y archivados. |
| DEC-027 | VALIDATING | E-stop BACK; fallback START+Y por 0.5 s | Prueba del DS/gamepad exactos. |
| DEC-028 | ACCEPTED | Hood/webcam retiradas; Limelight instalada pero no configurada | Evidencia física/config contradice el reporte. |
| DEC-029 | ACCEPTED | Cero manual con marca/fixture y `zeroValid` efímero | Se instala y valida home sensor. |
| DEC-030 | ACCEPTED | Shooter manual A/B, trim D-pad/X y sin override | Nueva decisión explícita del equipo. |
| DEC-031 | ACCEPTED | Feeder por pulsos acotados y sólo con robot estacionario | Diseño mecánico o estrategia cambia. |
| DEC-032 | ACCEPTED | Tags en tres etapas y revalidación posterior a cleanup | Política de release cambia. |
| DEC-033 | ACCEPTED | Todo dato físico no verificado es `TBD-BLOCKING` | Medición/revisión lo convierte en verificado. |
| DEC-034 | ACCEPTED | Pedro posee pose y movimiento; RR sólo rollback | Pedro no cumple MP-02. |
| DEC-035 | ACCEPTED | Tuners sin lifecycle quedan deshabilitados durante MP-01 | MP-02 integra lifecycle verificable. |
| DEC-036 | ACCEPTED | Contrato de hardware confirmado para MP-01 | Cambia configuración física o RC. |
| DEC-037 | ACCEPTED | Kicker motor-only final; CRServo retirado y compilado en cero | Nueva decisión formal cambia el diseño. |
| DEC-039 | ACCEPTED | FND-027 usa gate 10/10 por sentido para el APK 9C2F | Cambia APK, potencia, límites, transmisión o configuración física. |
| DEC-040 | ACCEPTED | Cierre funcional del shooter para MP-01 bajo APK 09B2...DAC1 | Cambia APK, dirección, signo, watchdog, potencia, transmisión o configuración física. |
| DEC-041 | ACCEPTED | Autónomos Pedro Pathing en el release final; retiro de autos Road Runner | Algún autónomo Pedro no pasa su gate MP-08. |

## 3. Decisiones aceptadas

### DEC-001 — Paquete documental modular

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** el programa abarca auditoría, arquitectura, investigación, pruebas, limpieza y operación; un solo archivo sería difícil de ejecutar por partes.
- **Decisión:** mantener `docs/plan-maestro-robot.md` como índice/programa y anexos numerados más registros de findings/decisions. Preservar `docs/critical-findings.md` sin reescribirlo.
- **Alternativas:** un documento monolítico; tickets sin arquitectura compartida.
- **Razón:** navegación, ownership y cambios independientes, sin perder una secuencia maestra.
- **Consecuencia:** enlaces/metadatos deben mantenerse; el índice resume, los anexos son autoridad temática.

### DEC-002 — Pedro primario y Road Runner como rollback

- **Estado/fecha:** `SUPERSEDED` por DEC-034, 2026-07-15
- **Contexto:** el runtime actual usa RR; el equipo quiere Pedro y ya existe scaffold.
- **Decisión:** migrar/calibrar Pedro como único proveedor de pose de producción. Conservar RR sólo en snapshot/tag/rama de commissioning hasta que Pedro pase gates.
- **Alternativas:** conservar RR; ejecutar ambos; borrar RR inmediatamente.
- **Razón:** decisión técnica del equipo y menor duplicación futura, manteniendo reversibilidad durante commissioning.
- **Consecuencia:** MP-02 debe demostrar ≤2 in/2° antes de avanzar; no dual-stack runtime.

### DEC-003 — Localización de tres ruedas muertas

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** el robot físico tiene tres dead wheels; el baseline ya contiene un localizador RR de tres pods pero con placeholders.
- **Decisión:** Pedro se configura con tres pods físicos, offsets/signos/resolución medidos.
- **Alternativas:** encoders de drive, two-wheel + IMU, Pinpoint/OTOS.
- **Razón:** hardware confirmado por el usuario y menor acoplamiento con slip del drive.
- **Consecuencia:** no copiar mappings provisionales; calibración obligatoria.

### DEC-004 — Limelight fija al chasis

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** se necesita visión para relocalización/corrección además de aim.
- **Decisión:** montar Limelight 3A rígidamente al chasis con extrínseca medida. Nombre propuesto `limelight`, pendiente de configuración.
- **Alternativas:** cámara en torreta; webcam/VisionPortal; visión solamente.
- **Razón:** una transform estable facilita pose global y evita cable/rotación de cámara.
- **Consecuencia:** bearing de cámara se transforma al robot/torreta; el target puede salir de FOV aunque la torreta lo vea mecánicamente.

### DEC-005 — Odometría continua, visión gated

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** Limelight puede perder targets, tener latencia o producir outliers; odometría deriva.
- **Decisión:** odometría propaga pose siempre; Limelight corrige sólo con gates de validez, frescura, ID, geometría, residual y movimiento. Un residual de bearing puede aplicar trim acotado.
- **Alternativas:** `tx` directo; reemplazar pose por cada botpose; EKF desde primera versión.
- **Razón:** continuidad y fallas observables con complejidad incremental.
- **Consecuencia:** logging/shadow mode y quality states son obligatorios; EKF sólo por evidencia.

### DEC-006 — Goal por alianza y selección directa

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** DECODE identifica goal azul con tag 20 y rojo con 24; el baseline usa toggle durante run.
- **Decisión:** durante init, `gamepad1 X=BLUE`, `B=RED`; mostrar y lock al start. Sólo usar tag goal correspondiente para bearing.
- **Alternativas:** toggle; default Red; primer tag detectado.
- **Razón:** menos errores humanos y selección determinista.
- **Consecuencia:** `UNSELECTED` bloquea targeting; revisar manual oficial si cambia la temporada.

### DEC-007 — Un TeleOp para ambas alianzas

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** múltiples modos duplican fixes y confunden DS.
- **Decisión:** un Competition TeleOp parametrizado por alianza/preset en init.
- **Alternativas:** TeleOp Red/Blue separados.
- **Razón:** una sola safety policy y menor superficie.
- **Consecuencia:** selección/init necesita pruebas y visibilidad fuertes.

### DEC-008 — Cero manual y arco limitado de torreta

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** no hay home sensor confirmado y los cables impiden 360°.
- **Decisión:** el operador coloca marca central y mantiene un chord 1 s; entonces encoder=0 y subsystem se arma. Soft limits medidos con margen.
- **Alternativas:** reset automático al construir; búsqueda de hard stop; rotación continua.
- **Razón:** evita asumir posición y protege mecánica/cables.
- **Consecuencia:** cero movimiento antes de arm; repetir init si cero dudoso.

### DEC-009 — Tracking continuo después de armado

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** el usuario desea auto-aim continuo, no sólo al sostener un botón.
- **Decisión:** después de arm, targeting actualiza setpoint continuamente mientras el estado lo permita.
- **Alternativas:** tracking sólo con request; manual turret.
- **Razón:** menor tiempo de adquisición y operación simple.
- **Consecuencia:** limits/quality/faults y power management deben ser sólidos; feed sigue requiriendo request.

### DEC-010 — Sin auto-turn del drivetrain

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** la torreta puede quedarse sin arco.
- **Decisión:** detener torreta/feed y guiar al driver con telemetry/rumble. El driver orienta el chasis.
- **Alternativas:** command automático de giro; forzar turret limit.
- **Razón:** conserva control del driver y evita movimiento inesperado.
- **Consecuencia:** feedback y histéresis deben entrenarse; no portar auto-heading ajeno.

### DEC-011 — Shooter de un motor y ángulo fijo

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** el usuario confirma un motor; el mecanismo se fijará y hood servos se retirarán.
- **Decisión:** `ShooterSubsystem` posee un motor `Shooter`; retirar hood y `Shooter2` sólo después de confirmación física/release gate.
- **Alternativas:** dual motor; hood variable.
- **Razón:** corresponde al robot final y reduce variables/fallas.
- **Consecuencia:** RPM por distancia absorbe la variación; rango de tiro puede ser más limitado.

### DEC-012 — Modelo de RPM más simple que pase datos retenidos

- **Estado/fecha:** `VALIDATING`, 2026-07-15
- **Contexto:** ángulo fijo requiere ajustar velocidad por distancia.
- **Decisión:** comparar lineal, cuadrático máximo grado 2 y piecewise-linear. El ajuste matemático sólo crea candidatos y no certifica ninguno. Elegir el más simple que, en cada una de dos sesiones, cumpla ≥9/10 tiros en cada distancia de calibración y ≥8/10 en cada distancia retenida/intermedia; no combinar sesiones para ocultar una falla. Cada intento debe registrar candidato, sesión, grupo de distancia, rol calibración/holdout, RPM objetivo, RPM medida al feed, hold continuo de readiness y resultado físico. El gate exige que el target sea el del candidato, error medido ≤100 RPM y hold ≥250 ms antes del feed.
- **Alternativas:** presets manuales únicamente; balística compleja; red neuronal.
- **Razón:** explicabilidad y menor extrapolación con dataset FTC pequeño.
- **Consecuencia:** no se conocerá la forma final hasta T8; versionar dataset/modelo. Si falta una celda, una sesión falla o ningún candidato cumple, el selector entrega un modelo seguro de cero RPM y `NO_MODEL_MET_CRITERIA`; nunca habilita piecewise por defecto.

### DEC-013 — Controles de RPM del operador 2

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** el shooter puede overshootear/undershootear por batería/piezas/condición.
- **Decisión:** `DPAD_UP/DOWN` cambia trim ±100 RPM por flanco; `X` reset trim; `Y` alterna `AUTO_DISTANCE`/`MANUAL_RPM`.
- **Alternativas:** sliders Dashboard; presets exclusivamente; pasos mayores.
- **Razón:** corrección rápida, discreta y visible en match.
- **Consecuencia:** debounce, feedback, logs y conflicto de bindings deben probarse.

### DEC-014 — Override limitado, clamp absoluto permanente

- **Estado/fecha:** `SUPERSEDED` por DEC-030, 2026-07-15
- **Contexto:** se desea override sin permitir overspeed peligroso.
- **Decisión:** trim normal se limita a ±500 y se reinicia en init. `RPM_TRIM_LIMIT_ENABLED=true` por default. Un chord sostenido puede desactivarlo, pero `MIN/MAX_VALIDATED_RPM` siempre clampan el target.
- **Alternativas:** sin override; override total; persistir trim.
- **Razón:** resiliencia operacional sin remover protección física.
- **Consecuencia:** chord aún debe asignarse; banner/log `OVERRIDE`; no hay gamepad bypass del clamp físico.

### DEC-015 — Feeder actual y feed interlocked

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** `KickerSubsystem` representa el feeder; bindings actuales pueden actuarlo directamente.
- **Decisión:** renombrar concepto a feeder conservando inicialmente string `kickerM otor`; sólo un interlock central autoriza power. Driver mantiene `gamepad1 RB` para solicitar.
- **Alternativas:** feed directo; pulse automático sin hold; nuevo hardware mapping inventado.
- **Razón:** lenguaje correcto y cero lanzamientos no solicitados.
- **Consecuencia:** todos los adapters/callers directos se eliminan o pasan por el coordinator.

### DEC-016 — Modo degradado fijo hacia delante

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** el robot necesita una forma limitada de seguir anotando si pose+visión fallan.
- **Decisión:** `DEGRADED_FIXED_FORWARD`: torreta al cero validado, conductor apunta chasis, manual RPM, request sostenido, dwell, timeout y clamps. Entrada deliberada, nunca silenciosa.
- **Alternativas:** bloquear todo tiro; usar visión cruda; bypass general.
- **Razón:** disponibilidad con riesgo acotado.
- **Consecuencia:** no mueve turret si desarmada/encoder fault; requiere drills y banner dominante.

### DEC-017 — Chord state-aware de un segundo

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** se quieren pocas combinaciones peligrosas y confirmación deliberada.
- **Decisión:** `gamepad2 START+BACK` sostenido 1 s arma el centro durante init y alterna degraded durante run. El estado elimina la ambigüedad; feedback es distinto.
- **Alternativas:** chequeo instantáneo; botones separados; tap simple.
- **Razón:** entrada deliberada y fácil de recordar.
- **Consecuencia:** el E-stop usa gamepad 1 y no debe colisionar; human-factors test obligatorio.

### DEC-018 — Composition root y ownership únicos

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** turret/vision están fuera del container actual.
- **Decisión:** `RobotContainer`/factory principal posee todos los subsystems y coordinators; cada device se mapea una vez.
- **Alternativas:** ownership por OpMode; service locator; duplicación temporal permanente.
- **Razón:** lifecycle, safety y testabilidad.
- **Consecuencia:** migrar por interfaces y comprobar búsquedas de hardware map.

### DEC-019 — Artefacto final: uno más uno y cero autos

- **Estado/fecha:** `SUPERSEDED` por DEC-026, 2026-07-15
- **Contexto:** el usuario quiere deploy/menu limpios y no requiere autónomos en este objetivo.
- **Decisión:** exactamente un Competition TeleOp y un System Check seguro; cero Autonomous, tests o tuners registrados en build normal.
- **Alternativas:** conservar disabled; múltiples diagnostics; autos viejos.
- **Razón:** menos selección errónea y menor superficie.
- **Consecuencia:** tuners viven en commissioning history/branch; revisar registradores dinámicos.

### DEC-020 — Archivo en Git, eliminación en main

- **Estado/fecha:** `SUPERSEDED` por DEC-032, 2026-07-15
- **Contexto:** mover código a `/archive` puede seguir compilándolo y no limpia realmente.
- **Decisión:** tras MP-08 crear tag anotado verificable y borrar legacy de `main`. Road Runner queda recuperable desde snapshot.
- **Alternativas:** carpetas archive; borrar sin tag; conservar todo disabled.
- **Razón:** rollback completo con main pequeño.
- **Consecuencia:** MP-09 requiere permiso, build/tag remoto y prueba desde otro checkout.

### DEC-021 — Documentación técnica en español

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** el equipo trabajará/operará principalmente en español.
- **Decisión:** plan, manual y findings en español técnico; nombres/API de código se conservan cuando ayuden a precisión.
- **Alternativas:** sólo inglés.
- **Razón:** ownership del equipo y entrenamiento.
- **Consecuencia:** definir/glosar términos como feeder, readiness, dwell, gate y rollback.

### DEC-022 — Sin upgrades incidentales

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** Gradle/SDK tienen deuda, pero auto-aim ya es un cambio amplio.
- **Decisión:** no actualizar FTC SDK, Gradle, AGP o Java como parte incidental. Consolidar sólo la versión ya resuelta/validada en MP-09; upgrades futuros en plan separado.
- **Alternativas:** modernizar todo durante migración.
- **Razón:** aislar riesgos y respetar restricciones del repo.
- **Consecuencia:** verificar API contra resolución efectiva y documentar mismatch.

### DEC-023 — Medir impacto de limpieza/deploy

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** el objetivo incluye deploy más rápido, pero tamaño de source no es el único factor.
- **Decisión:** medir build frío/caliente, incremental, APK, instalación y tiempo hasta OpMode antes/después; reportar aunque no mejore.
- **Alternativas:** asumir beneficio; medir sólo APK.
- **Razón:** orientar esfuerzos a cuello real.
- **Consecuencia:** baseline/repeticiones/entorno controlado en MP-00/09.

### DEC-024 — Repos externos como referencias con gate

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** se investigaron equipos fuertes con patrones similares.
- **Decisión:** usar Iron Reign, RevAmped, Hyperion y otros como catálogo de patrones. Copiar sólo tras fijar commit, licencia, atribución, API, units, safety y tests.
- **Alternativas:** copiar subsystems completos; ignorar fuentes externas.
- **Razón:** aprender sin importar supuestos peligrosos o infringir licencia.
- **Consecuencia:** registrar la influencia concreta en una nueva decisión/PR.

### DEC-025 — Baseline documental vigente

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** la rama documental quedó detrás de cambios integrados en `origin/main`.
- **Decisión:** toda afirmación de estado actual se contrasta con `origin/main@b5a134260456565df9d0295722ebecad900f21b4`. `main@f91af18` permanece sólo como evidencia histórica etiquetada.
- **Consecuencia:** inspeccionar el ref con `git show` no implica mezclar/rebasar la rama documental.

### DEC-026 — Visibilidad distinta en commissioning y release

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** el robot aún no está afinado y ocultar tuners impediría completar commissioning.
- **Decisión:** mantener visibles `MainTeleOp`, System Check, Shooter Tuning, todos los tuners Pedro y los registradores dinámicos Road Runner durante commissioning. Un tuner visible pero sin contrato de safety queda bloqueado para uso. Tras MP-08/09, el release muestra sólo Competition TeleOp y System Check; los tuners permanecen en rama/tag recuperable.
- **Consecuencia:** FND-010 distingue superficie deliberada de tuning y legacy no-tuning.

### DEC-027 — E-stop primario y fallback

- **Estado/fecha:** `VALIDATING`, 2026-07-15
- **Contexto:** FTC SDK moderno permite ligar `back`, pero el comportamiento depende del Driver Station, Advanced Gamepad Features y gamepad real.
- **Decisión:** `gamepad1 BACK` activa inmediatamente un latch global que cancela scheduler y llama `RobotSafety.stopAll()`. Si la prueba del equipo exacto falla, usar `gamepad1 START+Y` sostenido 0.5 s; no habilitar movimiento hasta validar uno.
- **Fuente:** release notes oficiales del FTC Robot Controller, versión 7.1 y posteriores: <https://github.com/FIRST-Tech-Challenge/FTCRobotController>.
- **Consecuencia:** el control debe ser uniforme en producción, System Check y tuners autorizados.

### DEC-028 — Estado físico reportado de cámaras y hood

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** el equipo confirma que hood y webcam ya fueron retiradas y que Limelight está instalada.
- **Decisión:** el diseño futuro no usa hood ni webcam. Limelight permanece `TBD-BLOCKING` hasta confirmar mapping, firmware, pipeline, red, montaje y extrínseca; estar instalada no autoriza consumo de datos.
- **Consecuencia:** el código legacy se conserva hasta la fase de cleanup, pero no se presenta como hardware disponible.

### DEC-029 — Cero manual verificable de torreta

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** no se añadirá sensor de home en este alcance; un reset de encoder no prueba centro físico.
- **Decisión:** usar marca/fixture manual, hold `gamepad2 START+BACK` de 1 s durante init y flag `zeroValid`. Cada init, reset, brownout o encoder implausible invalida el cero y bloquea movimiento/feed hasta repetir el procedimiento. Soft limits incluyen zona de frenado y contención exterior.
- **Consecuencia:** el procedimiento no detecta por sí mismo una marca falsa; requiere inspección humana y cable slack.

### DEC-030 — Controles manuales de shooter sin override

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** MainTeleOp debe conservar disparo manual antes de auto-distance y no existen límites físicos verificados para un override.
- **Decisión:** `gamepad2 A` arma/inicia RPM manual; `B` detiene; `DPAD_UP/DOWN` ajusta ±100 RPM por flanco; `X` restaura trim. El trim normal queda ±500 y el clamp físico absoluto siempre aplica. No hay override desde gamepad. Tras validar MP-06, `Y` puede alternar auto/manual con feedback visible.
- **Consecuencia:** cualquier override futuro requiere una nueva decisión y caracterización.

### DEC-031 — Feeder por pulsos y readiness estacionaria

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** hold directo puede energizar indefinidamente y los criterios anteriores sólo validaban tiros estacionarios sin comprobar movimiento.
- **Decisión:** `gamepad1 RB` solicita disparos repetidos. Cada pulso tiene duración/cooldown físicos `TBD-BLOCKING`, revalida readiness y se corta por release, fault, E-stop o pérdida de cualquier interlock. Readiness incluye velocidades lineal y angular por debajo de umbrales medidos.
- **Consecuencia:** no existe API de power directo desde bindings/adapters de competencia.

### DEC-032 — Rollback y release en tres etapas

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** limpiar después de dos sesiones cambia el SHA aceptado y un smoke test no prueba equivalencia.
- **Decisión:** crear `baseline/pre-mp01-YYYYMMDD` antes del primer paquete de código, `archive/pre-cleanup-YYYYMMDD` antes de borrar y `release/competition-YYYYMMDD` sólo después de MP-10. MP-10 repite T0–T10 y dos sesiones sobre el SHA limpio.
- **Consecuencia:** MP-09 produce candidato, no release.

### DEC-033 — Unknowns físicos bloquean output

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** el repo no contiene export RC ni medidas confiables de varias constantes.
- **Decisión:** mapping, dirección, signo, límite o constante física ausente se marca `TBD-BLOCKING` y mantiene inhibido el actuador/automatismo afectado. La guía 08 define medición, evidencia y aprobación.
- **Consecuencia:** Dashboard/defaults provisionales nunca cuentan como valores seguros.

### DEC-034 — Pedro como dueño único de pose y movimiento

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** abstraer sólo `PoseProvider` dejaría `DriveSubsystem/MecanumDrive` sobre Road Runner.
- **Decisión:** MP-02 migra localización, conducción manual, seguimiento de paths, actualización de pose y stop a un único adapter respaldado por Pedro. Road Runner queda operativo sólo como rollback de commissioning hasta aceptar el gate.
- **Consecuencia:** no se libera un dual-stack runtime; el rollback revierte el paquete completo.

### DEC-035 — Tuners sin lifecycle quedan deshabilitados durante MP-01

- **Estado/fecha:** `ACCEPTED`, 2026-07-17
- **Contexto:** los tuners Pedro/Road Runner pueden mover el drivetrain fuera de `SafeCommandOpMode`; algunos usan acciones bloqueantes y no ofrecen el E-stop latched de MP-01.
- **Decisión:** conservar su código pero deshabilitar su registro durante MP-01. MP-02 sólo los vuelve a exponer después de integrar stop, interrupción y E-stop verificables.
- **Consecuencia:** System Check y Shooter Tuning son los únicos modos de commissioning con actuadores visibles durante MP-01, además del TeleOp principal.
- **Rollback:** reactivar únicamente en una rama de MP-02 después de pasar revisión estática y prueba restringida.

### DEC-036 — Contrato de hardware confirmado para MP-01

- **Estado/fecha:** `ACCEPTED`, 2026-07-17
- **Contexto:** DEC-011/DEC-015 preservaban hardware/nombres legacy mientras faltaba confirmación. El equipo confirmó el mapa lógico y los dispositivos retirados.
- **Decisión:** `kickerMotor` y `kickerServo` reemplazan el string legacy; el CRServo usa +0.5/-0.5/0 y comparte ownership con el motor. `Shooter2`, hood y webcams se retiran del hardware activo. Los encoders par0/par1/perp se leen por `rightFront`/`leftFront`/`rightBack` respectivamente. Left front, left back y right back se invierten; right front no.
- **Consecuencia:** RobotMap se convierte en fuente única de nombres/direcciones; el export RC y las pruebas físicas siguen siendo gates obligatorios.
- **Supersede:** reemplaza las partes de DEC-011/DEC-015 que exigían conservar `kickerM otor` y mantener hardware retirado hasta confirmación.

### DEC-037 — Kicker motor-only final; CRServo retirado

- **Estado/fecha:** `ACCEPTED`, 2026-07-18
- **Contexto:** el motor y el CRServo recibieron comandos simultáneos, pero el CRServo mostró aproximadamente 0.5 s de demora al arrancar y 0.5–1.0 s al detenerse. Mecánica retiró físicamente el servo y el candidato motor-only pasó INIT, avance, reversa, release, Stop y E-stop sin piezas.
- **Decisión actualizada 2026-07-20:** el lead confirma configuración final motor-only. `KICKER_SERVO_ENABLED=false` queda constante de compilación y el servo permanece retirado.
- **Condición para cambiar:** sólo una nueva decisión formal puede reintroducir un servo; requiere diseño, mapping, APK y regresión completos. No existe rollback operativo a dual.
- **Consecuencia:** FND-026 queda `CLOSED` para la configuración final. Kicker motor-only continúa; carga/corriente se validan como pruebas del motor, no del servo.
- **Rollback:** no aplica durante esta configuración; conservar servo retirado y salida compilada en cero.

### DEC-038 — Reutilización de código de repos FTC sin auditoría de licencias, y patrón piecewise adoptado de HyperionBots

- **Estado/fecha:** `ACCEPTED`, 2026-07-18
- **Contexto:** el doc 04 exigía verificación de licencia antes de reutilizar código externo; el equipo decidió que entre equipos FTC se copia/adapta directamente. La Pista Software necesitaba el modelo RPM-por-distancia (MP-06) y el guard fail-closed de Limelight.
- **Decisión:** se autoriza copiar/adaptar código de repos públicos de equipos FTC sin trámite de licencias, dejando en el código un comentario de origen. Patrones adoptados en esta ventana, de `Ashley904/HyperionBotsWorldsRepository` (FTC 18011): (1) interpolación piecewise-linear con clamp en ambos extremos sin extrapolar (`PiecewiseLinearRpmModel`, adaptada de `PrecomputeShooterLookupTable` — sin hood, unidades RPM, duplicados promediados, clamp a `SHOOTER_MAX_SAFE_RPM` dentro del modelo); (2) guard `result == null || !result.isValid()` → rechazo, en `LimelightSubsystem.periodic()`.
- **Partes rechazadas:** hood lookup (ángulo fijo local), auto-heading de chasis y `tx` directo a motor (contradicen DEC-005), `TrackingThread`/código de RevAmped (Pedro "ivy" + Java 17 incompatibles con el toolchain local).
- **Razón:** partir de implementación probada en competencia reduce riesgo en ventana de 20h; la adaptación conserva los contratos de seguridad propios.
- **Consecuencia:** el checklist de licencias del doc 04 secc. 10 queda reducido a "comentario de origen + entrada aquí"; los tests puros (`RpmModelsTest`) son el gate de la adaptación.
- **Rollback:** sustituir la implementación adaptada por una propia equivalente; las interfaces (`RpmModel`) no cambian.

### DEC-039 — Gate 10/10 por sentido para FND-027

- **Estado/fecha:** `ACCEPTED`, 2026-07-20
- **Owner/aprobadores:** Test lead; equipo de Tuning con Safety operator presente.
- **Contexto:** FND-027 exigía históricamente 20/20 pulsos por sentido. La matriz completa implicaba 40 movimientos individuales y competía con los gates restantes de MP-01 y shooter. El candidato temporal de `850 ms` produjo evidencia repetible y sin anomalías.
- **Decisión:** reducir formalmente el criterio de FND-027 a 10/10 pulsos por sentido exclusivamente para el APK SHA-256 `9C2F58F3327064A199BC9426A646C05550BDC3F462ACF90E3534851BAD9B38E0`. El Test lead declaró: “La matriz 10/10 queda aceptada para el APK 9C2F, con asimetría direccional conocida y sin anomalías.”
- **Evidencia/test gate:** 20/20 filas `PASS`; `STOPPED_TIMEOUT`, power final `0`, `Move active=false`, parada completa, cero anomalías y estabilidad durante la espera. Duración `855.0–881.6 ms`; |delta| positivo `54–60`, negativo `59–72`; límites de aceptación `800–900 ms`, `|delta|<=75` y desplazamiento postcorte `<=80 ticks`.
- **Asimetría aceptada:** en nueve parejas con precarga estandarizada, positivo `67.8 ticks/s`, negativo `75.7 ticks/s`, diferencia promedio `11.6%`. La variación por pareja (`-1.1%` a `+25.5%`) impide tratarla como factor fijo; la compensación queda para control cerrado/auto-aim posterior.
- **Safety/operación:** esta decisión no aumenta potencia, no amplía ±200 ticks, no debilita watchdog, cero manual, Stop/E-stop ni hold-to-run. No autoriza más pulsos de torreta para FND-027.
- **Consecuencia:** FND-027 pasa a `CLOSED`. MP-01 no se declara cerrado por esta decisión; conserva sus demás findings y gates físicos.
- **Revisión/rollback:** cualquier cambio de APK, potencia, autocorte, watchdog, soft limits, transmisión, motor, cableado o configuración física invalida esta matriz y requiere un gate nuevo. Si se revoca DEC-039, vuelve a aplicar el criterio histórico 20/20.

### DEC-040 — Cierre funcional del shooter para MP-01

- **Estado/fecha:** `ACCEPTED`, 2026-07-20
- **Owner/aprobadores:** Test lead; equipo de Tuning con Safety operator presente.
- **Contexto:** FND-028 se abrió por giro hacia dentro y ausencia de ticks/RPM durante el primer pulso. La secuencia diagnóstica confirmó encoder pasivo, corrigió la combinación de dirección/signo y agregó watchdog y autocorte fail-closed.
- **Decisión:** cerrar FND-028 para MP-01 exclusivamente bajo APK SHA-256 `09B23FD2ED57E30496D5D7FBB710F21FDD34A77255F1D414C8578BBE465DDAC1`. El Test lead acepta dirección hacia fuera, encoder positivo, watchdog y autocorte.
- **Evidencia:** pulso único sin piezas, target `1000 RPM`, cap `0.75`, `STOPPED_TIMEOUT` a `3027.7 ms`, ticks `0 -> 555`, peak/end `1028.6/942.9 RPM`, batería mínima `12.37 V`, power final `0`, health=`HEALTHY`, fault=`none`, parada completa y cero anomalías.
- **Límite explícito:** ready hold `125.6 ms` no cumple los `250 ms` de T8. El cierre no certifica estabilidad, carga ni tiros y no habilita feeder; permanecen pendientes para MP-06/T8.
- **Revisión/rollback:** cualquier cambio de APK, dirección, signo de encoder, watchdog, límite de potencia, transmisión, motor, cableado o configuración física exige revalidar el gate correspondiente.

### DEC-041 — Autónomos Pedro Pathing en el release final; retiro de autos Road Runner

- **Estado/fecha:** `ACCEPTED`, 2026-07-22
- **Contexto:** `plan-maestro-robot.md` sección 1 (línea 29) y MP-09 fijaban "cero autónomos en el artefacto final de competencia" como decisión no negociable, y `06-limpieza-y-release.md` planeaba borrar todo autónomo en MP-09. Esa postura venía originalmente de DEC-019, que quedó `SUPERSEDED` por DEC-026 — pero DEC-026 sólo resuelve visibilidad de tuners en commissioning/release, no reafirma ni retira explícitamente la parte de "cero autos" de DEC-019, dejando ese punto ambiguo hasta ahora. El equipo ya escribió y entregó 10 rutas autónomas nuevas usando Pedro Pathing (paths exportados desde la herramienta visual de Pedro), y el drivetrain de producción ya migró completamente a Pedro como dueño único de pose/movimiento (DEC-034). Los 8 autónomos Road Runner legado (`AutonomoBetaPosition`, `AutonomoOfficialBlue`, `AutonomoOfficialRed2`, `AutonomoOfficialRed` ya comentado como código muerto, `FullOfficialBlue2`, `FullOfficialRed2`, `FullOficialBlue`, `FullOficialRed`) construyen `DriveSubsystem`/`MecanumDrive` de Road Runner directamente y ya no compilan contra el `RobotContainer` Pedro-only actual; `handoff-task.md` ya los documentaba como "ya no se necesitan, se conservan sólo como historial/rollback".
- **Decisión:** el release final de competencia sí incluye un conjunto reducido de autónomos, todos basados en Pedro Pathing y sujetos al mismo contrato de seguridad que el TeleOp de competencia (`SafeAutonomousOpMode`, E-stop `gamepad1 BACK`, `RobotSafety`, límites ya vigentes de torreta/shooter/feeder). Los 8 autónomos Road Runner legado se eliminan de `masterplan` — no se mueven a un paquete `/archive` dentro de `source`, mismo criterio que ya usa `06-limpieza-y-release.md` §3.1 ("el historial Git es el archivo"). `TestShootBurstAuto` se conserva: no mueve drivetrain, es una prueba estacionaria de shooter/kicker/intake.
- **Alternativas consideradas:** mantener "cero autónomos" y tratar los autos Pedro nuevos como track experimental fuera de competencia; pausar hasta confirmar con todo el equipo/lead. Descartadas: el equipo confirmó directamente que los autónomos sí van a competencia.
- **Consecuencia:** `plan-maestro-robot.md` (línea 29 y la redacción de MP-09) y `06-limpieza-y-release.md` (líneas 16, 109, 208, 274 y la tabla de clasificación §5) se actualizan para distinguir "autónomos Road Runner legado, eliminar" de "autónomos Pedro aceptados, conservar en el release", en vez de "cero autónomos" a secas. `plan-paralelo-20h.md` §3 (Pista Software) suma la integración de estos autos como ítem de trabajo. Ningún autónomo Pedro nuevo se declara listo para competencia sin pasar el mismo gate MP-08 que el resto del programa (dos sesiones integradas, sin finding crítico/alto abierto).
- **Rollback:** si un autónomo Pedro específico no pasa su gate, ese archivo se deshabilita (`@Disabled`) sin afectar a los demás ni forzar volver a "cero autónomos". No revive automáticamente ningún autónomo Road Runner: su código ya no existe en `masterplan`, sólo en el historial de Git (y en cualquier tag/rama de commissioning que se cree antes de borrar).

## 4. Decisiones pendientes de implementación, no de producto

Estas preguntas se resuelven con evidencia dentro del contrato ya aprobado:

| Pregunta | Método de decisión | Paquete |
|---|---|---|
| Valores exactos de turret limits/power/PID | Medición T4/T7 | MP-01/05 |
| Extrínseca de cámara | Medición + cuatro anclas | MP-03/04 |
| Alpha/beta y gates de fusión | Shadow logs/replay | MP-04 |
| Ventana permitida `ODOMETRY_ONLY` | Deriva medida | MP-04/08 |
| Lineal/cuadrático/piecewise | Dataset retenido | MP-06/T8 |
| RPM física min/max y slew | Aprobación mecánica + caracterización | MP-01/06 |
| Duración/power/cooldown de feeder | Caracterización restringida y corriente | MP-01/06/T9 |
| Controles finales intake/jam | Mapa integral + test humano | MP-07 |
| Umbrales lineal/angular de readiness | Logs de drive y tiros estacionarios | MP-06/T9 |

No hace falta una nueva preferencia abstracta para estas cifras; hace falta medir.

## 5. Plantilla de nueva decisión

```markdown
### DEC-XXX — Título

- **Estado/fecha:** `PROPOSED`, YYYY-MM-DD
- **Owner/aprobadores:**
- **Contexto/problema:**
- **Restricciones/hechos:**
- **Decisión:**
- **Alternativas consideradas:**
- **Razón/tradeoff:**
- **Consecuencias positivas/negativas:**
- **Safety/operación:**
- **Evidencia/test gate:**
- **Rollback:**
- **Supersede/revisión:**
- **Fuentes/licencia si aplica:**
```

## 6. Control de cambios

- No cambiar una entrada `ACCEPTED` para aparentar que siempre dijo otra cosa.
- Crear una nueva decisión que marque la anterior `SUPERSEDED`.
- Toda decisión que reduzca un límite/interlock requiere revisión mecánica/safety y pruebas.
- Una constante tunable no reemplaza una decisión: debe tener unidad, rango y owner.
- Al cerrar un paquete, actualizar decisiones que pasaron de `VALIDATING` a `ACCEPTED` o `REJECTED` con evidencia.
