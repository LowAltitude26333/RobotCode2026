# Registro de decisiones

> Estado: registro vivo de decisiones de producto/arquitectura
> Baseline inicial: `main` en `f91af18`
> Última actualización: 2026-07-15
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
| DEC-002 | ACCEPTED | Pedro Pathing primario; RR rollback Git | Pedro no cumple gate MP-02. |
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
| DEC-014 | ACCEPTED | Trim normal ±500, override limitado, clamp absoluto | Límite mecánico cambia con aprobación. |
| DEC-015 | ACCEPTED | Feeder es kicker actual; request sostenido + interlock | Diseño mecánico cambia. |
| DEC-016 | ACCEPTED | `DEGRADED_FIXED_FORWARD` deliberado | Safety testing lo invalida. |
| DEC-017 | ACCEPTED | Chord START+BACK state-aware de 1 s | Human-factors testing pide otro control. |
| DEC-018 | ACCEPTED | Un owner/composition root | Arquitectura completa cambia formalmente. |
| DEC-019 | ACCEPTED | Un TeleOp + un System Check, cero autos final | Alcance de competencia cambia. |
| DEC-020 | ACCEPTED | Snapshot Git y borrar legado de main al final | Política de conservación cambia. |
| DEC-021 | ACCEPTED | Sistema y documentación técnica en español | Necesidad del equipo cambia. |
| DEC-022 | ACCEPTED | No actualizar toolchain/SDK incidentalmente | Upgrade plan separado aprobado. |
| DEC-023 | ACCEPTED | Métricas, no promesas, para velocidad de deploy | Nueva evidencia/objetivo. |
| DEC-024 | ACCEPTED | Fuentes externas son patrones, con gate de licencia | Política legal/educativa cambia. |

## 3. Decisiones aceptadas

### DEC-001 — Paquete documental modular

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** el programa abarca auditoría, arquitectura, investigación, pruebas, limpieza y operación; un solo archivo sería difícil de ejecutar por partes.
- **Decisión:** mantener `docs/plan-maestro-robot.md` como índice/programa y anexos numerados más registros de findings/decisions. Preservar `docs/critical-findings.md` sin reescribirlo.
- **Alternativas:** un documento monolítico; tickets sin arquitectura compartida.
- **Razón:** navegación, ownership y cambios independientes, sin perder una secuencia maestra.
- **Consecuencia:** enlaces/metadatos deben mantenerse; el índice resume, los anexos son autoridad temática.

### DEC-002 — Pedro primario y Road Runner como rollback

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
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
- **Decisión:** comparar lineal, cuadrático máximo grado 2 y piecewise-linear. Elegir el más simple que cumpla ≥9/10 en calibración y ≥8/10 retenido/intermedio.
- **Alternativas:** presets manuales únicamente; balística compleja; red neuronal.
- **Razón:** explicabilidad y menor extrapolación con dataset FTC pequeño.
- **Consecuencia:** no se conocerá la forma final hasta T8; versionar dataset/modelo.

### DEC-013 — Controles de RPM del operador 2

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** el shooter puede overshootear/undershootear por batería/piezas/condición.
- **Decisión:** `DPAD_UP/DOWN` cambia trim ±100 RPM por flanco; `X` reset trim; `Y` alterna `AUTO_DISTANCE`/`MANUAL_RPM`.
- **Alternativas:** sliders Dashboard; presets exclusivamente; pasos mayores.
- **Razón:** corrección rápida, discreta y visible en match.
- **Consecuencia:** debounce, feedback, logs y conflicto de bindings deben probarse.

### DEC-014 — Override limitado, clamp absoluto permanente

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
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
- **Consecuencia:** E-stop/override no deben colisionar; human-factors test obligatorio.

### DEC-018 — Composition root y ownership únicos

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** turret/vision están fuera del container actual.
- **Decisión:** `RobotContainer`/factory principal posee todos los subsystems y coordinators; cada device se mapea una vez.
- **Alternativas:** ownership por OpMode; service locator; duplicación temporal permanente.
- **Razón:** lifecycle, safety y testabilidad.
- **Consecuencia:** migrar por interfaces y comprobar búsquedas de hardware map.

### DEC-019 — Artefacto final: uno más uno y cero autos

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
- **Contexto:** el usuario quiere deploy/menu limpios y no requiere autónomos en este objetivo.
- **Decisión:** exactamente un Competition TeleOp y un System Check seguro; cero Autonomous, tests o tuners registrados en build normal.
- **Alternativas:** conservar disabled; múltiples diagnostics; autos viejos.
- **Razón:** menos selección errónea y menor superficie.
- **Consecuencia:** tuners viven en commissioning history/branch; revisar registradores dinámicos.

### DEC-020 — Archivo en Git, eliminación en main

- **Estado/fecha:** `ACCEPTED`, 2026-07-15
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
| Chord exacto de override | Análisis de conflictos + drills | MP-06/07 |
| Controles finales intake/jam/E-stop | Mapa integral + test humano | MP-07 |
| Umbral final de shots/pose/aim | Dos sesiones y estrategia | MP-08 |

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
