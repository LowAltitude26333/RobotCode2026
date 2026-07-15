# Registro de hallazgos

> Estado: registro vivo; entradas iniciales provienen de auditoría estática, no de pruebas físicas
> Baseline inicial: `main` en `f91af18`
> Última actualización: 2026-07-15
> Alcance: bugs, riesgos, discrepancias, resultados y retests del plan maestro
> Responsable sugerido: test lead; cada entrada debe tener owner técnico
> Fuente de verdad: evidencia vinculada al SHA/configuración/sesión. No cerrar hallazgos sólo porque compile.

## 1. Cómo usar este registro

Crear una entrada cuando:

- el comportamiento observado difiere de lo esperado;
- una suposición física no puede confirmarse;
- un safety path falta o falla;
- una prueba aborta;
- una fuente externa/versión contradice el diseño;
- una métrica no cumple su gate;
- se descubre deuda que puede afectar competencia/mantenimiento.

No crear findings para tareas normales ya planeadas salvo que aparezca una discrepancia concreta. Los items de trabajo viven en el plan; los findings contienen evidencia de un problema/unknown.

## 2. Estados

| Estado | Significado |
|---|---|
| `OPEN` | Confirmado o suficientemente sustentado; necesita acción. |
| `INVESTIGATING` | Se está aislando causa/alcance. |
| `BLOCKED_PHYSICAL` | Falta medición/acceso físico específico. |
| `CONTAINED` | Camino deshabilitado o mitigado, pero causa/código permanece. |
| `FIX_READY` | Fix implementado, falta retest. |
| `CLOSED` | Reproducción y regresión pasan con evidencia. |
| `WONT_FIX` | Riesgo aceptado mediante decisión explícita, con razón. |
| `SUPERSEDED` | Reemplazado por finding más preciso; enlazarlo. |

## 3. Severidad

| Severidad | Criterio orientativo |
|---|---|
| `CRITICAL` | Puede causar movimiento/lanzamiento inseguro, daño, OpMode principal inutilizable o pérdida total de control. Bloquea hardware/release. |
| `HIGH` | Puede causar score loss importante, comportamiento no determinista o bypass de una protección. Bloquea release. |
| `MEDIUM` | Degrada diagnóstico/mantenibilidad o tiene workaround claro; debe priorizarse. |
| `LOW` | Deuda menor/documentación sin impacto operativo inmediato. |

## 4. Índice actual

| ID | Severidad | Estado | Resumen | Owner | Paquete |
|---|---|---|---|---|---|
| FND-001 | CRITICAL | OPEN | Cleanup de `VisionPortal` ligado antes de asignarlo | Software vision | MP-01 |
| FND-002 | HIGH | OPEN | Chord de armado evaluado sólo una vez en initialize | Software/Ops | MP-01 |
| FND-003 | HIGH | BLOCKED_PHYSICAL | Límites ±200 ticks de torreta no validados | Mechanical + turret | MP-01/05 |
| FND-004 | HIGH | OPEN | Geometría de localización provisional/inconsistente | Localization | MP-02 |
| FND-005 | HIGH | OPEN | Feeder tiene caminos directos sin interlock único | Mechanisms | MP-01/06 |
| FND-006 | HIGH | OPEN | Secuencia shooter puede esperar readiness sin timeout | Shooter | MP-01/06 |
| FND-007 | MEDIUM | OPEN | E-stop no uniforme en OpModes habilitados | Safety | MP-01/07 |
| FND-008 | MEDIUM | OPEN | Ownership de torreta/visión fuera del container | Architecture | MP-07 |
| FND-009 | MEDIUM | OPEN | Declaraciones de versión/dependencias superpuestas | Build | MP-09 |
| FND-010 | MEDIUM | CONTAINED | Superficie de 14+ TeleOps/tuners | Release | MP-09 |
| FND-011 | HIGH | OPEN | `IntakeTeleOp` puede dejar feeder energizado | Safety/mechanisms | MP-01/09 |
| FND-012 | LOW | OPEN | Conceptos/nombres legacy no reflejan hardware final | Architecture | MP-06/09 |

## 5. Entradas iniciales

### FND-001 — Cleanup de `VisionPortal` registrado antes de construirlo

- **Severidad / estado:** `CRITICAL / OPEN`
- **Fecha / autor:** 2026-07-15 / auditoría asistida
- **Baseline:** `main@f91af18`
- **Configuración/sesión:** revisión estática; no se ejecutó en hardware
- **Componentes:** `MainTeleOp`, `TeleopTorreta`, lifecycle de `VisionPortal`
- **Esperado:** construir el portal, registrar un cleanup válido y cerrarlo al detener el OpMode.
- **Observado:** el código registra `addResourceCleanup(visionPortal::close)` antes de que `visionPortal` reciba el resultado del builder.
- **Riesgo:** evaluar una referencia a método ligada a `null` puede lanzar `NullPointerException` durante init y dejar inoperable el TeleOp.
- **Evidencia:** call path inspeccionado en el baseline; confirmar líneas exactas en el SHA de implementación.
- **Acción propuesta:** prueba de init deshabilitada; construir antes de registrar o usar cleanup que compruebe null y ownership; test de init/stop repetido.
- **Criterio de cierre:** el caso reproduce/falla en baseline cuando sea seguro, pasa ≥10 ciclos init/stop, cierra portal y no deja recurso abierto.
- **Owner:** software vision.
- **Retest:** pendiente, TEST a crear en T3.

### FND-002 — Armado de torreta depende de un instante de initialize

- **Severidad / estado:** `HIGH / OPEN`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Configuración/sesión:** revisión estática
- **Componentes:** init de `MainTeleOp`/`TeleopTorreta`, `TurretSubsystem`
- **Esperado:** el operador centra y mantiene un chord durante 1 s en init loop con feedback.
- **Observado:** la condición `gamepad2.start && gamepad2.back` se consulta sólo una vez durante `initialize()`.
- **Riesgo:** ventana impráctica/no determinista; torreta puede quedar desarmada o fomentar atajos inseguros.
- **Acción:** máquina de estados `TURRET_CENTER -> ARM_HOLD -> ARMED`, cancelable y state-aware.
- **Criterio de cierre:** hold <1 s no arma; ≥1 s arma sólo en estado correcto; soltar cancela; ninguna potencia antes de arm; 20 repeticiones.
- **Owner:** software turret + operator 2.

### FND-003 — Soft limits de torreta no validados físicamente

- **Severidad / estado:** `HIGH / BLOCKED_PHYSICAL`
- **Fecha:** 2026-07-15
- **Baseline:** constantes actuales alrededor de -200/+200 ticks
- **Configuración/sesión:** sin medición física adjunta
- **Esperado:** límites representan arco seguro con margen contra hard stops/cables.
- **Observado:** no existe evidencia física en el repo sobre dirección, ticks/grado, centro o margen.
- **Riesgo:** contacto mecánico, daño de cables o bloqueo prematuro.
- **Acción:** fixture/marks, potencia ≤0.1, medir ambos sentidos, backlash y repetibilidad; revisión mecánica.
- **Criterio de cierre:** tabla firmada con ticks/grados, margen, signos y TEST-T4 aprobado sin tocar hard stop.
- **Owner:** mechanical + turret.

### FND-004 — Localización no tiene constantes aptas para auto-aim

- **Severidad / estado:** `HIGH / OPEN`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Componentes:** RR `ThreeDeadWheelLocalizer`, `pedroPathing/Constants.java`
- **Esperado:** tres pods físicos con nombres, offsets, signos, resolución y gains medidos.
- **Observado:** RR contiene offsets 0/1/0 ticks y Pedro contiene masa TODO, setters duplicados de offset, pods asociados a nombres de drive y conversiones provisionales.
- **Riesgo:** pose/distancia/bearing incorrectos; fusión engañosa.
- **Acción:** MP-02 completo; no activar aim con esos valores.
- **Criterio de cierre:** inventario físico y ruta repetida ≤2 in/2°, con logs y signs comprobados.
- **Owner:** localization.

### FND-005 — No existe un único interlock de feeder

- **Severidad / estado:** `HIGH / OPEN`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Componentes:** `SkywalkerProfile`, `KickerSubsystem`, adapters/actions, comandos de tiro
- **Esperado:** todos los requests pasan por `ReadinessInterlock`; soltar/interrumpir/Stop ordena cero.
- **Observado:** existen bindings y adapters que ordenan feeder/kicker directamente, separados de pose/turret/RPM readiness.
- **Riesgo:** lanzamiento no solicitado o fuera de condiciones.
- **Acción:** inventariar cada caller; crear API de request/permiso; eliminar power directo de bindings; tests de matriz T9.
- **Criterio de cierre:** búsqueda demuestra un solo camino de output; todos los casos T9 pasan; cero feed involuntario en dos sesiones.
- **Owner:** mechanisms/software safety.

### FND-006 — Espera de readiness sin timeout en secuencia alternativa

- **Severidad / estado:** `HIGH / OPEN`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Componentes:** `ShootSequenceCommand`
- **Esperado:** readiness con timeout/failure state e interruption cleanup.
- **Observado:** se conserva `WaitUntilCommand(shooter::isReady)` sin un límite superior explícito, aunque `ShootBurstCommand` ya contiene mejores protecciones.
- **Riesgo:** comando colgado y mecanismos en estado difícil de entender.
- **Acción:** retirar camino legacy o implementar timeout/state central; comprobar top-level `end`.
- **Criterio de cierre:** shooter que nunca llega a RPM termina/bloquea de forma segura dentro del timeout y deja outputs definidos.
- **Owner:** shooter.

### FND-007 — E-stop no es uniforme

- **Severidad / estado:** `MEDIUM / OPEN`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Esperado:** todo OpMode de producción/diagnóstico comparte latch, cancela scheduler y llama `RobotSafety.stopAll()`.
- **Observado:** `SafeCommandOpMode`/`RobotSafety` existen, pero el latch con `gamepad1.back` se observó de forma consistente sólo en `MainTeleOp`; otros modos habilitados no comparten la misma superficie.
- **Riesgo:** expectativas distintas al seleccionar un modo de prueba.
- **Acción:** base única + limpieza; test en Competition TeleOp/System Check.
- **Criterio de cierre:** ambos OpModes finales pasan T4.1 y no existen modos alternos visibles.
- **Owner:** safety/release.

### FND-008 — Composición fragmentada

- **Severidad / estado:** `MEDIUM / OPEN`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Esperado:** `RobotContainer` posee drive, mechanisms, turret, camera y coordinators.
- **Observado:** `MainTeleOp` construye turret/VisionPortal fuera del container.
- **Riesgo:** lifecycle/telemetry/safety duplicados y difícil integración.
- **Acción:** mover ownership en MP-07 después de estabilizar interfaces; no duplicar mappings durante migración.
- **Criterio de cierre:** diagrama/call path demuestra un dueño por device y System Check reutiliza factory segura.
- **Owner:** architecture.

### FND-009 — Fuentes superpuestas de SDK/dependencias

- **Severidad / estado:** `MEDIUM / OPEN`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Esperado:** una versión efectiva fácil de identificar y compatible con `Limelight3A`.
- **Observado:** `TeamCode/build.gradle` declara SDK 10.3.0 mientras `build.dependencies.gradle` incluye SDK 11.0; Dashboard también aparece duplicado.
- **Riesgo:** desarrolladores verifican API contra versión equivocada; cleanup accidental rompe resolución.
- **Acción:** registrar dependency graph efectivo ahora; consolidar la versión ya validada sólo en MP-09, commit separado.
- **Criterio de cierre:** una fuente de versión, build limpio y smoke test, sin upgrade incidental.
- **Owner:** build/release.

### FND-010 — Superficie excesiva de OpModes

- **Severidad / estado:** `MEDIUM / CONTAINED`
- **Fecha:** 2026-07-15
- **Baseline:** 14 TeleOps anotados habilitados + tuning Pedro + registradores RR
- **Esperado:** un Competition TeleOp y un System Check.
- **Observado:** Driver Station puede presentar múltiples pruebas/variantes; autos están deshabilitados pero permanecen.
- **Riesgo:** seleccionar modo incorrecto y mantener safety policies divergentes.
- **Contención:** el plan no autoriza usar modos legacy como producción.
- **Acción:** snapshot y eliminación en MP-09, sólo tras acceptance.
- **Criterio de cierre:** inspección anotada/dinámica y DS muestran exactamente dos modos.
- **Owner:** release.

### FND-011 — Intake test puede dejar feeder energizado

- **Severidad / estado:** `HIGH / OPEN`
- **Fecha:** 2026-07-15
- **Baseline:** `main@f91af18`
- **Componentes:** `IntakeTeleOp`
- **Esperado:** movimiento diagnóstico sólo mientras se sostiene el permiso y con timeout.
- **Observado:** comandos instantáneos asociados a A/B pueden dejar power del feeder hasta `DPAD_DOWN` u otra transición.
- **Riesgo:** motor sigue energizado cuando el operador espera un pulso.
- **Acción:** no usar como diagnóstico normal; reemplazar con System Check hold-to-run; revisar stop actual.
- **Criterio de cierre:** modo eliminado del release y System Check pasa stop/timeout/release.
- **Owner:** safety/mechanisms.

### FND-012 — Terminología y hardware legado

- **Severidad / estado:** `LOW / OPEN`
- **Fecha:** 2026-07-15
- **Baseline:** `KickerSubsystem`, `ShooterHoodSubsystem`, `Shooter2`, string `kickerM otor`
- **Esperado:** conceptos reflejan feeder, shooter de un motor y hood fijo, sin romper mappings reales.
- **Observado:** nombres mezclan generaciones del robot.
- **Riesgo:** mantenimiento confuso o “corrección” accidental del hardware string.
- **Acción:** confirmar config; renombrar clases/conceptos por commits controlados; retirar legado en MP-09; preservar string hasta decisión explícita.
- **Criterio de cierre:** inventario final coincide con hardware/config y build; estudiantes pueden explicar naming exception si permanece.
- **Owner:** architecture/release.

## 6. Plantilla para nuevas entradas

```markdown
### FND-XXX — Título corto

- **Severidad / estado:** `HIGH / OPEN`
- **Fecha / autor:**
- **SHA / rama / dirty status:**
- **Session/config/build:**
- **Componente/owner:**
- **Requisito o decisión relacionada:**
- **Precondiciones:**
- **Esperado:**
- **Observado:**
- **Frecuencia/reproducibilidad:**
- **Impacto/riesgo:**
- **Evidencia:** log, CSV, video, screenshot, file/line o test ID
- **Hipótesis (separada de hechos):**
- **Acción/contención:**
- **Criterio de cierre:**
- **Retest y resultado:**
- **Decisión asociada:**
```

## 7. Reglas de mantenimiento

- IDs nunca se reutilizan.
- No editar el resultado observado para que coincida con la explicación posterior.
- Separar hecho, inferencia e hipótesis.
- Si el baseline cambia, registrar nuevo SHA; no sobrescribir evidencia vieja.
- `CONTAINED` no equivale a `CLOSED`.
- Todo `WONT_FIX` enlaza una decisión con riesgo aceptado y responsable.
- Findings históricos permanecen en [critical-findings.md](../critical-findings.md); este archivo registra el programa actual.
- En release, ningún `CRITICAL` o `HIGH` puede quedar abierto/contained sin una excepción aprobada explícitamente; la política preferida es cero excepciones.
