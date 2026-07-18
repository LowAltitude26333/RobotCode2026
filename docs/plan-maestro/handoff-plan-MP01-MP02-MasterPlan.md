# Plan auditado de handoff — MP-01, MP-02 y MasterPlan

Fecha de auditoría: 2026-07-18

Repo: `C:\Users\brito\OneDrive\Documentos\FTC\RobotCode2026`

Rama observada: `masterplan` en `24f9911`, tracking `origin/masterplan`
Estado Git al momento de la auditoría: worktree amplio con cambios locales sin commit. El SHA base no identificaba por sí solo el código, APK ni pruebas actuales; la actualización operativa posterior registra la consolidación.

## Actualización operativa al cierre de sesión — 2026-07-18

La auditoría que sigue conserva valor histórico, pero su matriz MP-01 quedó superada por las pruebas posteriores. Para el estado ejecutable vigente usar primero `handoff-task.md`, `hallazgos.md`, `contrato-hardware.md` y el diff/commit actual.

- MP-01 permanece `BLOCKED`, ahora principalmente por T4 de torreta y los contratos físicos todavía abiertos; fault injection del shooter cerró 4/4, intake/kicker motor-only y gates instrumentados de release/Stop/E-stop ya tienen evidencia registrada.
- APK T4 instalado: SHA-256 `D5AC6F1E11B42F5EC8E973AEB3132B359733A09CB6791809EA442D59E7557536`.
- Verificación pre-commit limpia: `assembleDebug` PASS tras regenerar `TeamCode/build`; APK no instalado SHA-256 `B8E72645AA8FA36678F9EDEC26216259679208F4BDCDCFE6DB3ABA2857E5E1B6`. No trasladar resultados físicos entre ambos hashes.
- Torreta: cero inválido rechazó movimiento; cero desde cinta armó en 0; toque derecho llegó a +12 y toque izquierdo de retorno terminó en -50. Ambos releases ordenaron cero en 2.265 ms y 2.726 ms, sin ruido ni reactivación. La diferencia no puede diagnosticarse sin medir duración real del hold.
- Cierre físico confirmado: OpMode detenido por el operador. El -50 final era relativo a esa inicialización y no es un home persistente.
- Próximo cambio autorizado: instrumentar duración del hold y delta de ticks sin modificar potencia 0.05, watchdog, timeout ni soft limits ±200; después repetir un solo hold temporizado por sentido desde la cinta.
- No iniciar MP-02 con movimiento mientras MP-01 siga bloqueado.

Este plan fue revisado en paralelo por tres auditores de sólo lectura: MP-01, MP-02 y gobierno global del MasterPlan. Las correcciones de esas auditorías ya están incorporadas aquí.

## Resultado de la auditoría

- MP-01: `BLOCKED`. Hay implementación avanzada y pruebas físicas aprobadas, pero todavía no está `READY_FOR_GATE`.
- MP-02: `NOT_STARTED` para producción. Sólo se autoriza preparación de software sin movimiento; Road Runner continúa como dueño activo y tampoco está físicamente calibrado.
- MasterPlan: utilizable como estructura, pero sus fuentes deben reconciliarse antes de publicar handoffs autoritativos.
- No se hizo commit, push, cambio de rama ni movimiento de hardware durante esta auditoría.

## Artefactos que debe producir el siguiente trabajo

- `docs/plan-maestro/handoff-MP01.md`
- `docs/plan-maestro/handoff-MP02.md`
- `docs/plan-maestro/handoff-MasterPlan.md`

Los tres deben usar rutas relativas, enlazar las fuentes existentes y resumir únicamente el delta operativo. El handoff global será un índice, no una copia de especificaciones.

## Fuentes autoritativas

- Plan y gates MP-00…MP-10: `docs/plan-maestro-robot.md`
- Evidencia cronológica actual: `docs/plan-maestro/handoff-task.md`
- Hardware confirmado y pendientes: `docs/plan-maestro/contrato-hardware.md`
- Findings: `docs/plan-maestro/hallazgos.md`
- Decisiones: `docs/plan-maestro/decisiones.md`
- Arquitectura objetivo: `docs/plan-maestro/02-arquitectura-objetivo.md`
- Pruebas T0…T11: `docs/plan-maestro/05-programa-pruebas.md`
- Verificación física: `docs/plan-maestro/08-guia-verificacion-hardware.md`
- Release: `docs/plan-maestro/06-limpieza-y-release.md`
- Estado real de implementación: `git status`, `git diff`, build actual y hash del APK. No usar outputs antiguos de `build/` como evidencia del worktree actual.

## Reconciliación documental obligatoria previa

1. Actualizar la cabecera de `docs/plan-maestro-robot.md` a 2026-07-18 y distinguir “atestación DS disponible” de “XML no exportable”.
2. Reconciliar DEC-025: todavía nombra `origin/main@b5a1342`, mientras el trabajo real está en `masterplan@24f9911`. No inventar un nuevo baseline histórico.
3. Agregar DEC-035 y DEC-036 al índice de `decisiones.md`.
4. En MP-02 usar DEC-034 como autoridad; DEC-002 debe aparecer sólo como decisión superseded/histórica.
5. Consolidar `handoff-task.md`: retirar pendientes que contradicen PASS posteriores y conservar la evidencia útil con fecha/configuración.
6. Reconciliar findings:
   - FND-001: evaluar `CLOSED` o `SUPERSEDED` por retiro de visión y 20 ciclos sin webcam.
   - FND-002: evaluar `CLOSED`; su criterio de 20 repeticiones fue cubierto.
   - FND-004: el índice dice `OPEN` y la ficha `FIX_READY`; debe quedar un solo estado.
   - FND-005: el índice dice `FIX_READY` y la ficha `OPEN`; debe quedar un solo estado.
   - FND-021: reformular como contrato físico incompleto; el XML no debe ser el único blocker porque la atestación DS fue aceptada.
7. Registrar que no existe ningún tag `baseline/pre-mp01-*`, `archive/pre-cleanup-*` ni `release/competition-*`. No fabricar retroactivamente el baseline que debió existir antes del cambio.

## Plantilla mínima de cada handoff

1. Propósito y alcance.
2. Identidad exacta: rama, base, dirty status, diff, RC/DS/firmware y APK/hash aplicable.
3. Estado: `NOT_STARTED`, `IN_PROGRESS`, `BLOCKED`, `READY_FOR_GATE` o `ACCEPTED`, respaldado por evidencia.
4. Referencias autoritativas.
5. Evidencia separada por compilación, instalación, observación física y aceptación.
6. Bloqueadores: finding, severidad, owner por rol o `UNASSIGNED`, condición de desbloqueo y siguiente acción segura.
7. Próxima secuencia ejecutable y punto exacto de detención.
8. Guardrails y movimientos no autorizados.
9. Rollback identificable.
10. Gate y definición de terminado.
11. Skills sugeridas para la siguiente sesión.

Cada PASS físico debe registrar ID de prueba, SHA o hash de APK, configuración, fecha, repeticiones y ruta durable de evidencia. No trasladar automáticamente resultados de un APK a otro.

## Handoff MP-01

### Estado honesto

`BLOCKED`.

La evidencia actual sí respalda mappings atestados por DS, 20/20 INIT sin movimiento/webcam, 20/20 intentos cortos y 20/20 armados de torreta, E-stop 10/10 en INIT y 10/10 después de START en `MainTeleOp`, drive elevado en todos los ejes y smoke test dual del kicker.

La optionalidad de `kickerServo` existe en el worktree:

- `LowAltitudeConstants.KICKER_SERVO_ENABLED=false` por defecto;
- lookup con `HardwareMap.tryGet()`;
- bandera capturada durante INIT;
- telemetría solicitado/disponible/activo;
- fallback motor-only: el CRServo queda inhibido y su ausencia se tolera sin crash.

No llamarlo “fail-closed motor-only”: el `kickerMotor` conserva movimiento. El cambio opcional es posterior al APK físicamente probado y aún requiere build, instalación y regresión propios.

### Matriz de artefactos MP-01

| Elemento | Estado |
|---|---|
| Código actual | Worktree dirty sobre `masterplan@24f9911`; incluye servo opcional |
| Último APK empaquetado conocido | 81,276,807 bytes; SHA-256 `0B2237E9336034B687B82BA119C73E1E3370C77657CAC92128CD60F7C43A3382`; 2026-07-17 21:20 |
| Relación del APK con el código actual | Es anterior al cambio opcional; sirve sólo como rollback previo |
| Build del cambio opcional | Java compiló; `:TeamCode:packageDebug` falló por locks de OneDrive |
| APK actual físicamente probado | No existe todavía |

### Bloqueadores y omisiones

- FND-003: signo, ticks/grado y límites físicos de torreta.
- FND-015: fault injection segura del shooter: freeze, lectura imposible, voltaje inválido y overspeed simulado.
- FND-020: falso centro, reinicio y brownout de torreta.
- FND-021: faltan signo/orientación IMU y signos de par0/par1/perp; no bloquear sólo por el XML.
- FND-026: motor y CRServo no cumplen sincronía mecánica. Se requiere decisión formal: configuración motor-only aceptable y dual prohibido, o dual obligatorio y fase bloqueada.
- Falta prueba física explícita de intake: avance, reversa, release, Stop y E-stop.
- Falta E-stop de `SystemCheck` y `ShooterTuning`; sólo `MainTeleOp` tiene evidencia repetida.
- Falta un procedimiento instrumentado para demostrar orden cero en primer ciclo y ≤50 ms. “Aparentemente inmediato” no satisface el gate.
- FND-005/006/013/014/015/016 permanecen `FIX_READY` hasta cubrir su regresión exacta; FND-007 permanece `CONTAINED`.

### Orden seguro de cierre

1. Reinspeccionar rama, diff y OpModes habilitados.
2. Con Android Studio cerrado y OneDrive pausado, hacer build limpio del worktree actual sin modificar tooling.
3. Registrar tamaño/hash del APK e instalarlo.
4. Con el servo físicamente retirado y bandera `false`, probar INIT sin crash, telemetría `false/false/false`, kicker motor-only, release, Stop y E-stop.
5. No probar bandera `true` con piezas mientras FND-026 siga abierto.
6. Probar intake y E-stop de los tres OpModes habilitados.
7. Definir primero la instrumentación de ≤50 ms y después ejecutar el gate.
8. Completar torreta y fault injection del shooter.
9. Decidir formalmente si motor-only permite contener FND-026.
10. Reconciliar documentos y emitir MP-01 como `READY_FOR_GATE` o `BLOCKED`. Sólo usar `ACCEPTED` si el gate completo tiene evidencia.

## Handoff MP-02

### Estado honesto

`NOT_STARTED` para producción; preparación software-only autorizable.

Runtime activo: `MainTeleOp -> RobotContainer -> DriveSubsystem -> MecanumDrive`. Road Runner posee movimiento y pose, e instancia directamente `ThreeDeadWheelLocalizer`. Pedro es sólo scaffold; su tuner está `@Disabled`, y los tuners dinámicos RR están deshabilitados. No existe todavía `PoseProvider`, adapter Pedro ni estado de calidad de pose.

El scaffold Pedro no es físicamente válido:

- versión declarada Pedro 2.1.2 y telemetry 1.0.0;
- usa `rightRear`, nombre inexistente; el contrato usa `rightBack`;
- asigna pods left=`leftFront`, right=`rightRear`, strafe=`rightFront`, pero la evidencia es par0=`rightFront`, par1=`leftFront`, perp=`rightBack`;
- direcciones Pedro: LF y RB contradicen la inversión física; LB y RF coinciden;
- masa, escala y offsets siguen provisionales;
- RR tampoco está calibrado: `inPerTick` y offsets 0/1/0 son provisionales y los signos no están medidos;
- la inversión de encoders debe verificarse independientemente de la inversión de motores; las flags RR de pods están actualmente en `false`.

La IMU se usa para heading field-centric; el localizer de tres pods deriva heading de los pods. El handoff debe distinguir ambos flujos.

### Prerrequisitos separados

**Software-only, sin actuadores:**

- MP-01 puede seguir bloqueado si no se construye hardware ni se instancia Pedro con motores.
- Inventariar todos los constructores directos de `MecanumDrive`, `DriveSubsystem` y Pedro `Follower`.
- Definir contratos neutrales, ownership único, frames, unidades, reset, timestamps, velocidad y calidad.
- Crear pruebas puras de transformaciones, wrap de ángulo, stop/lifecycle y selección de owner.

**Antes de habilitar un tuner restringido:**

- MP-01 aceptado para movimiento o autorización física explícita.
- Safety wrapper, Stop/E-stop en primer ciclo ≤50 ms, potencia reducida y rollback exacto.
- Área, ruta y procedimiento de prueba definidos.

**Resultados de calibración de MP-02, no prerrequisitos circulares:**

- signos de par0/par1/perp;
- ticks/rev efectivos, diámetro efectivo y offsets;
- orientación/signo IMU;
- escalas y constantes medidas.

### Orden de ejecución

1. Crear `PoseProvider`, `PoseSnapshot` y `DriveController/DriveAdapter` neutrales; RR sigue siendo owner activo.
2. Formalizar frame/sign/reset/timestamp/velocity/quality y pruebas puras.
3. Inventariar todos los puntos de construcción RR/Pedro.
4. Ejecutar T2.1/T2.3 y revisión de ownership.
5. Ejecutar T4.1, incluyendo cero en primer ciclo y ≤50 ms; luego T4.4.
6. Calibrar signos y geometría; después ejecutar T5.
7. T5 debe incluir cinco repeticiones por sentido para forward, strafe y 360° CW/CCW, más cuadrados/rectángulos, aceleraciones típicas y conservación de logs/distribuciones. No afirmar cinco rutas mixtas si la fuente no lo exige.
8. Hacer shadow sólo como replay/simulación offline sin `HardwareMap`, o con un único owner explícito y motores inhibidos. Nunca instanciar `Follower` Pedro y RR concurrentemente sobre el mismo hardware.
9. Sólo después del gate, hacer cambio atómico de ownership a Pedro y repetir T4.1/T5.
10. Mantener un commit/tag RR exacto como rollback de commissioning; no una referencia ambigua.

Gate objetivo de T5: ≤2 in y ≤2° sin deriva sistemática, sujeto a la definición autoritativa del programa de pruebas.

## Handoff MasterPlan

Debe ser un índice ejecutivo. Su tabla MP-00…MP-10 incluirá:

- estado;
- `depends_on`;
- gate;
- blockers;
- owner por rol o `UNASSIGNED`;
- última verificación;
- evidencia concreta del gate;
- próxima fase autorizada.

También debe incluir:

- CRITICAL/HIGH abiertos;
- decisiones autoritativas y superseded relevantes;
- riesgos transversales;
- inventario de tags con columnas `esperado/existe/verificado/acción`;
- cadencia de actualización;
- rollback verificable;
- definición global de MP-10: repetir T0–T10 y dos sesiones sobre el SHA limpio, cero CRITICAL/HIGH abiertos, verificar el tag desde otro checkout, build/deploy reproducible, manual firmado y demostrar propiedad del equipo.

No inferir estado ni owner sin evidencia. Usar `NOT_STARTED`, `BLOCKED` o `UNASSIGNED` cuando corresponda.

## QA final de los tres handoffs

1. Resolver contradicciones en las fuentes, no sólo ocultarlas en el handoff.
2. Generar MP-01, luego MP-02 y finalmente MasterPlan.
3. Verificar que cada prueba indique el APK/SHA al que realmente aplica.
4. Comprobar que cada referencia existe y que no se duplicaron specs completas.
5. Hacer lectura en frío: un agente nuevo debe identificar estado, primer paso seguro y blockers en menos de cinco minutos.
6. Ejecutar comprobación de enlaces y `git diff --check`.
7. Para documentación solamente no hace falta build ni hardware.

## Definición de terminado de este trabajo

- Existen los tres handoffs bajo `docs/plan-maestro/`.
- Las fuentes contradictorias fueron reconciliadas o quedaron marcadas explícitamente como bloqueadas, sin inventar hechos.
- MP-01 no aparece aceptado mientras sus gates físicos y FND-026 sigan sin resolución formal.
- MP-02 no autoriza motores con constantes provisionales, tuners sin lifecycle o ownership dual.
- MasterPlan coincide con los otros dos handoffs y registra la ausencia de tags históricos.
- Todos los enlaces y `git diff --check` pasan.
- No se hizo commit, push, cambio de rama ni movimiento de hardware para redactarlos.
