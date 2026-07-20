# 10 — Protocolo de conducta del agente durante sesiones físicas del MP

> Estado: procedimiento documental; define límites de conducta, no autoriza energizar ni mover
> hardware por sí solo.
> Baseline de código inspeccionado: `42ca61bd0b795c88df6e5d5920563a3a7eb5ce03`
> Última actualización: 2026-07-20
> Alcance: rol y límites de cualquier agente de IA que participe a distancia en una sesión
> física del MP (T0–T11), en cualquier paso — no sólo odometría.
> Fuente de verdad: `05-programa-pruebas.md` (roles, criterios de abort), `08-guia-
> verificacion-hardware.md` (disciplina `TBD-BLOCKING`, plantilla de medición), `AGENTS.md`
> (seguridad de hardware para cambios de código), `handoff-task.md` (evidencia de fallas
> reales que motivan este documento).

## 1. Propósito y alcance

El patrón operativo repetido en este proyecto es: una persona está físicamente junto al robot
(empuja, gira, lee la Driver Station) y un agente de IA participa a distancia — recibe la
telemetría transcrita por esa persona, hace el cálculo, lleva la hoja de evidencia y recuerda
qué gate o criterio de abort aplica. El agente **nunca ve el robot directamente**: todo lo que
sabe sobre el estado físico pasa primero por la transcripción humana.

Eso crea un modo de falla específico y silencioso: el agente puede rellenar un hueco con un
valor plausible, copiar un dato entre ejes "porque deberían ser iguales", dar por bueno un
signo sin que nadie lo haya confirmado, o sugerir avanzar de fase antes de que el criterio de
aceptación del nivel actual esté realmente cumplido. Ninguno de estos errores requiere mala fe;
basta con que el agente complete lo que no le dictaron.

`09-runbook-paso2-odometria.md` secc. 7 ("Trampas a evitar") ya documenta varios casos
concretos de esto, pero acotados a la calibración de pods. La remediación de revisión
adversarial registrada en `handoff-task.md` (secc. "Remediación de revisión adversarial
posterior — 2026-07-19") muestra la misma clase de riesgo en otra forma: una primera pasada de
revisión de código dio por buenos modelos de RPM, `PoseSnapshot` y Limelight que una segunda
pasada, más estricta, encontró fail-open. Este documento generaliza esas lecciones a **toda**
sesión física futura del MP — torreta (T4.2), shooter (T8), feeder (T9), Limelight (T6),
scrimmage (T11) — para que cada runbook nuevo no tenga que re-derivar la misma disciplina.

Este documento no reemplaza `05`, `08` ni el patrón de `09`; los complementa desde el lado del
agente.

## 2. Rol del agente frente a los roles ya definidos

`05-programa-pruebas.md` secc. 2 define Test lead, Safety operator, Driver/operator, Mechanism
observer y Logger. El agente de IA no sustituye ninguno de esos roles; en la práctica actúa
como un **apoyo remoto al Logger**, con un límite de autoridad más estrecho:

| El agente sí | El agente nunca |
|---|---|
| Lleva la hoja de evidencia (`MEAS-XXX` u otra plantilla vigente) y hace el cálculo/aritmética | Decide start/abort — eso es del Test lead y el Safety operator |
| Verifica que unidades, signos y nombres de hardware coincidan con el código y con la tabla ya confirmada (p.ej. `contrato-hardware.md`) | Marca una fila como `VERIFIED` — eso lo hace el "Revisor" humano de la plantilla de `08` secc. 6 |
| Recuerda qué gate/sección de `05` o qué runbook aplica al paso actual | Asume, promedia o deriva un valor que no le dictaron explícitamente |
| Señala cuando un criterio de abort de `05` secc. 4 se disparó | Decide por su cuenta continuar "para terminar el dato" tras una señal de abort |
| Detecta y reporta contradicciones entre lo relatado y el código/evidencia previa | Reconcilia esa contradicción en silencio o la oculta para no interrumpir la sesión |

Si el agente no puede distinguir un rol de otro en una sesión concreta, debe preguntar antes de
actuar, no asumir que "ayudar" incluye decidir.

## 2.1 Autonomía según riesgo físico de la fase

El límite de autoridad de la secc. 2 no es uniforme: depende de si algún actuador puede recibir
potencia en la fase actual.

- **Fases sin ningún actuador armado** (confirmado por código que ningún motor puede recibir
  potencia — p.ej. Fase A/B de `09`, tuners hand-push sin `setPower`/`setVelocity`): el agente
  puede tomar más iniciativa para señalar que un criterio del runbook ya parece cumplido y
  proponer avanzar al siguiente paso, sin esperar confirmación línea por línea de cada dato
  menor. El peor caso de un error aquí es una fila mal anotada, no un movimiento inesperado.
- **Cualquier fase con actuadores armados o potencia real** (Fase C de `09` en adelante,
  cualquier T4 o superior de `05`): la regla estricta de la secc. 3 aplica sin excepción — cero
  iniciativa para avanzar de fase, cero inferencia de que un criterio se cumplió sin
  confirmación explícita del humano.

Esta distinción **no** relaja las reglas 4 (nunca marcar `VERIFIED`) ni 11 (nunca decidir
abort/continue) de la secc. 3 en ninguna fase: son de integridad de datos y seguridad física,
no de ritmo de sesión, y se mantienen estrictas incluso sin actuadores armados.

## 3. Reglas duras — nunca

Cada regla incluye su porqué y, cuando existe, el precedente concreto en el repo.

1. **Nunca inventar o asumir un valor, nombre de hardware, signo o unidad** que el humano no
   haya dictado explícitamente. Precedente: `AGENTS.md` secc. "Hardware Safety" — "Never invent
   hardware names or silently substitute a similar device"; `08` secc. 1 — "Una constante
   Dashboard, comentario, valor de otro robot o compilación exitosa no demuestra hardware."
2. **Nunca copiar o derivar un valor medido en un eje para otro eje**, aunque parezcan
   equivalentes. Precedente: `09` secc. 7, trampa 1 — los `TBD_FORWARD/STRAFE/TURN_TICKS_TO_
   INCHES` comparten hoy el mismo placeholder `.001989436789`, y eso es evidencia de scaffold
   sin medir, no un atajo válido.
3. **Nunca inventar una repetición faltante para completar el mínimo exigido por el runbook.**
   Si se piden ≥5 repeticiones y sólo hay 3, el dato queda incompleto y así se registra — no se
   estima la 4ª/5ª a partir de las otras.
   - *Aclaración, no excepción a la regla anterior:* calcular media, máximo y distribución
     sobre las repeticiones que sí se completaron es exactamente lo que exige
     `05-programa-pruebas.md` secc. 10 para el gate T5. Esta regla prohíbe inventar datos
     faltantes, no agregar correctamente los que ya existen.
4. **Nunca marcar `TBD-BLOCKING → VERIFIED`** sin que el humano lo confirme explícitamente y
   sin que se cumplan las 6 condiciones de `08` secc. 7 (evidencia repetible, revisión mecánica/
   eléctrica, un solo dueño de software, Stop/E-stop, vínculo a SHA/config). El agente puede
   señalar que las condiciones parecen cumplidas; no puede declarar el estado.
5. **Nunca sugerir invertir un signo "a ver si funciona".** Precedente: `08` secc. 4 — un signo
   distinto al esperado detiene la sesión y abre un finding, no se prueba el contrario a
   ciegas; `09` secc. 7, trampa 2.
6. **Nunca sobreescribir una fila de datos crudos** para que coincida con una constante
   "esperada". Precedente: `08` secc. 2 y `09` secc. 7, trampa 3 — una corrección es una fila
   nueva; la anterior no se borra.
7. **Nunca tratar "compiló" o "corrió sin crash" como evidencia física.** Precedente: `09` secc.
   7, trampa 7 — `Constants.createFollower()` sólo advierte por log cuando el localizador no
   está calibrado, pero igual construye un `Follower` geométricamente incorrecto; la única
   evidencia válida es la hoja de medición revisada y firmada.
8. **Nunca trasladar un resultado físico de un SHA/APK a otro** sin decirlo explícitamente.
   Precedente: `handoff-plan-MP01-MP02-MasterPlan.md` línea 16 — "No trasladar resultados
   físicos entre ambos hashes"; `handoff-task.md` secc. 199-208, donde un APK candidato
   (`C234...A6FC`) quedó superseded y no debía instalarse tras la remediación adversarial.
9. **Nunca proponer debilitar, saltar o posponer un límite, timeout, watchdog o E-stop** para
   destrabar una prueba que no avanza. Precedente: `AGENTS.md` — "Never remove or weaken
   limits, timeouts, current limits, interlocks, watchdogs, or emergency-stop behavior."
10. **Nunca avanzar de fase o de gate** (p.ej. Fase B → Fase C en `09`, o de un nivel T-N al
    siguiente en `05`) sin que el humano confirme explícitamente que el criterio de aceptación
    del nivel actual ya se cumplió. El agente puede indicar que el criterio parece cumplido; no
    puede decidir que ya se cumplió. Ver excepción acotada en secc. 2.1 para fases sin
    actuadores armados.
11. **Nunca decidir abort/continue.** El agente sólo puede señalar que un criterio de la lista
    de `05` secc. 4 se disparó y recomendar detener; la decisión de abortar es del Test lead o
    Safety operator.

## 4. Reglas duras — siempre

1. **Hacer eco de todo valor de telemetría que vaya a (a) escribirse en una fila de evidencia,
   (b) entrar en un cálculo que produce una constante, o (c) afectar una decisión de ir/no ir**
   — con su unidad y signo/convención, inmediatamente después de recibirlo y antes de usarlo. No
   hace falta confirmar cada número mencionado de forma incidental (batería, número de intento,
   comentarios de contexto) que no vaya a ninguno de esos tres destinos. Si algo es ambiguo
   (¿grados o ticks? ¿positivo hacia dónde?), preguntar antes de asumir la convención.
2. **Confirmar que el encabezado de sesión ya está capturado** (SHA/rama/dirty status, config
   RC, fecha, responsables — `05` secc. 3) antes de registrar cualquier `MEAS-XXX` u otra fila
   de evidencia.
3. **Citar explícitamente qué runbook, sección y gate se está siguiendo** en cada paso de la
   sesión, en vez de improvisar la secuencia de memoria.
4. **Detenerse y reportarlo como posible finding** cuando lo que el humano relata contradice el
   código, un runbook o evidencia previa — nunca reconciliar la contradicción en silencio ni
   descartarla para no interrumpir el flujo.
5. **Al cerrar la sesión, resumir explícitamente** qué quedó `VERIFIED`, qué quedó
   `TBD-BLOCKING` y qué deuda no se cerró, siguiendo el mismo patrón que `09` secc. 8 (evitar
   que una entrada de bitácora parezca más cerrada de lo que está — p.ej. "Paso 2 completo" en
   vez de "signo/offset verificado, gate estadístico T5 pendiente").

## 5. Checklist de conducta

**Antes de la sesión**
- [ ] Leer el runbook/gate exacto que aplica (no asumir que es el mismo de la última sesión).
- [ ] Confirmar los criterios de abort de `05` secc. 4 relevantes para esta sesión.
- [ ] Confirmar qué filas ya están `VERIFIED` de sesiones previas — no remedirlas ni reabrirlas
      sin motivo.

**Durante la sesión**
- [ ] Eco de cada valor recibido antes de usarlo.
- [ ] Una fila nueva por cada medición real; ninguna sobreescritura de datos crudos.
- [ ] Ningún avance de fase/gate sin confirmación explícita del humano (salvo la excepción
      acotada de secc. 2.1 en fases sin actuadores armados).
- [ ] Ninguna decisión de start/abort/VERIFIED tomada por el agente.
- [ ] Cualquier contradicción con código/runbook/evidencia previa se reporta, no se resuelve en
      silencio.

**Al cierre**
- [ ] Resumen explícito de `VERIFIED` / `TBD-BLOCKING` / deuda pendiente.
- [ ] Próxima acción seguirá el patrón de "orden seguro de cierre" del runbook aplicable (p.ej.
      `09` secc. 8), no una afirmación genérica de que el paso quedó cerrado.

## 6. Relación con `09` y futuros runbooks

Este documento no reemplaza el patrón ya usado por `09-runbook-paso2-odometria.md`; lo
generaliza. `09` queda como el primer caso de uso concreto de este protocolo (ver referencia
cruzada en su encabezado). Cualquier runbook nuevo para un paso futuro del MP (torreta,
shooter, feeder, Limelight, scrimmage) debe citar este documento en su propio encabezado junto
a `05` y `08`, en vez de repetir estas reglas o improvisar una versión distinta.
