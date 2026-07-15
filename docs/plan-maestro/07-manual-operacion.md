# 07 — Manual de operación del robot

> Estado: borrador de diseño; **no usar para operar hardware hasta implementar y validar los controles**
> Baseline de referencia: arquitectura objetivo posterior a MP-07, no el `main` actual
> Última actualización: 2026-07-15
> Alcance: preparación, operadores, disparo normal/degradado, fallas y shutdown
> Responsables sugeridos: operadores 1 y 2, drive coach y safety lead
> Fuente de verdad: release notes y mapa de controles del APK desplegado; si difieren, detener y resolver antes de habilitar.

## 1. Advertencia

Este manual describe el comportamiento que se va a construir. El repo `main@f91af18` todavía no implementa todos estos controles ni estados. No entrenar muscle memory ni habilitar el robot suponiendo que este documento ya corresponde al APK.

La versión de competencia deberá mostrar en telemetry su SHA/versión y enlazar con la revisión exacta del manual.

## 2. Roles

- **Operador 1 / driver:** conduce mecanum, orienta el chasis cuando la torreta llega a límite y mantiene la solicitud de disparo/feed.
- **Operador 2:** opera intake/shooter según mapa final, centra/arma torreta, elige modo de RPM, aplica trim y decide entrar al modo degradado.
- **Drive coach:** confirma alianza/preset, comunica target/estrategia y vigila estados/faults sin pedir bypasses inseguros.
- **Safety lead/pit:** valida mecánica/configuración y conserva acceso a Stop/E-stop durante pruebas.

## 3. Conceptos que todo el equipo debe entender

### 3.1 Armed no significa ready

- `TURRET ARMED`: el cero se confirmó y el motor puede obedecer dentro de límites.
- `SHOT READY`: además, pose/modo, aim, RPM y feeder cumplen todos los interlocks.

### 3.2 Request no significa feed

Mantener `gamepad1 RB` pide un disparo. El feeder sólo actúa cuando `SHOT READY`. Soltar RB debe cortar el feeder.

### 3.3 Modos de targeting

- `NORMAL_FUSED`: odometría + Limelight validada.
- `ODOMETRY_ONLY`: cámara no utilizable, pose aún confiable; indicador de degradación.
- `DEGRADED_FIXED_FORWARD`: pose/visión no confiables; entrada deliberada, torreta frontal y RPM manual.
- `DISABLED/FAULT`: no se permite targeting/feed.

### 3.4 Modos de shooter

- `AUTO_DISTANCE`: modelo calcula RPM por distancia y luego aplica trim.
- `MANUAL_RPM`: operador selecciona/ajusta setpoint dentro del rango físico.
- `OVERRIDE`: sólo el límite normal de trim fue desactivado deliberadamente; el clamp físico sigue activo.

## 4. Checklist pre-match en pits

Con robot apagado/deshabilitado:

- [ ] Release/SHA y manual corresponden.
- [ ] Robot Controller configuration correcta.
- [ ] Batería cargada y asegurada; conectores firmes.
- [ ] Control/Expansion Hub, cables de odometría e IMU firmes.
- [ ] Limelight firme, lente limpia, cable/alimentación seguros.
- [ ] Pipeline/fieldmap/firmware esperados.
- [ ] Tres pods bajan, giran libres y no tienen residuos.
- [ ] Ruedas/drivetrain sin obstrucción.
- [ ] Torreta se coloca manualmente en marca central sin forzar.
- [ ] Cableado de torreta tiene slack hacia ambos límites.
- [ ] Shooter sin daño, objeto suelto ni vibración visible.
- [ ] Feeder/intake libres de piezas atoradas.
- [ ] Zona de salida del proyectil despejada.
- [ ] Gamepads correctos y controles neutrales.
- [ ] Stop/E-stop y responsable identificados.

No mover la torreta a mano con el mecanismo energizado salvo procedimiento mecánico aprobado.

## 5. Init paso a paso

### Paso 1 — Seleccionar el OpMode

Elegir el único TeleOp de competencia. Si aparecen múltiples versiones/tuners o un nombre inesperado, no iniciar: probablemente se desplegó el APK equivocado.

### Paso 2 — Revisar boot

Al presionar INIT:

- todos los actuadores deben permanecer en cero;
- telemetry muestra versión/SHA;
- no debe aparecer una excepción;
- `ALLIANCE` inicia `UNSELECTED`;
- turret inicia `DISARMED`;
- feeder debe indicar `BLOCKED`.

Si cualquier motor se mueve, presionar Stop y registrar el finding.

### Paso 3 — Seleccionar alianza

Con gamepad 1:

- `X` selecciona **BLUE**;
- `B` selecciona **RED**.

Confirmar color/texto grande. Presionar de nuevo cambia directamente, no alterna de manera oculta. La selección queda bloqueada al comenzar el match.

### Paso 4 — Seleccionar pose inicial

Elegir el preset de posición inicial aprobado para esa estrategia y confirmarlo. La pantalla debe mostrar X/Y/heading y nombre humano. No confirmar una pose que no coincide con la colocación real.

Limelight puede mostrar una validación/corrección si ve tags confiables. Una discrepancia grande bloquea ready; no aceptar a ciegas.

### Paso 5 — Verificar visión

Telemetry debe mostrar:

- Limelight conectada/start;
- pipeline/fieldmap esperado;
- frame age razonable;
- goal tag compatible cuando esté visible;
- ninguna corrección grande pendiente.

La falta de visión puede permitir odometría según política, pero debe ser visible.

### Paso 6 — Centrar y armar torreta

1. confirmar físicamente la marca central y cable slack;
2. operador 2 mantiene `START+BACK` durante 1 segundo;
3. la barra/contador de hold progresa; soltar cancela;
4. al completar, el encoder se pone en cero y aparece `TURRET ARMED` con rumble/confirmación;
5. si aparece fault o la torreta no está realmente centrada, Stop; no “compensar” conduciendo.

El chord se evalúa durante init loop. No debe requerir adivinar el instante de `initialize()`.

### Paso 7 — Confirmación final antes de START

- [ ] Alliance correcta y locked-on-start.
- [ ] Pose preset correcta.
- [ ] Pose quality aceptable.
- [ ] Turret armed y cero físico correcto.
- [ ] Shooter target 0 hasta que corresponda.
- [ ] Feeder power 0.
- [ ] Sin fault crítico.
- [ ] Ambos operadores conocen modo inicial.

## 6. Mapa de controles objetivo relevante

Sólo se fijan aquí los controles ya decididos; intake/drive restantes deben completarse en MP-07 y validarse contra conflictos.

### Gamepad 1 — Driver

| Control | Acción | Nota |
|---|---|---|
| Sticks | Conducir mecanum field-centric según perfil final | Reset de heading se documentará tras cerrar bindings. |
| `RB` sostenido | Solicitud de disparo/feed | No salta readiness; soltar corta feeder. |
| `X` durante init | Seleccionar BLUE | Sin efecto de cambio de alianza durante run. |
| `B` durante init | Seleccionar RED | Sin efecto de cambio de alianza durante run. |
| Feedback de límite | Orientar chasis en la dirección indicada | El robot no gira automáticamente. |

### Gamepad 2 — Operador

| Control | Acción | Nota |
|---|---|---|
| `DPAD_UP` | Trim +100 RPM | Por pulsación/flanco. |
| `DPAD_DOWN` | Trim -100 RPM | Por pulsación/flanco. |
| `X` | Trim a 0 | Confirmación visible. |
| `Y` | Alternar `AUTO_DISTANCE`/`MANUAL_RPM` | Modo siempre visible. |
| `START+BACK` sostenido 1 s en init | Confirmar centro y armar torreta | Sólo si el init state lo permite. |
| `START+BACK` sostenido 1 s durante run | Entrar/salir de `DEGRADED_FIXED_FORWARD` | Feedback distinto al arm. |
| Chord de override por definir | Desactivar/activar límite normal de trim | Nunca desactiva clamp físico. |

El chord de override y controles finales de intake/reversa/jam deben documentarse antes de marcar el manual como aprobado. No reutilizar un botón con una acción contextual peligrosa sin feedback.

## 7. Disparo normal

1. Driver coloca el robot en una distancia soportada.
2. El sistema calcula pose, goal de alianza, bearing y distancia.
3. Torreta sigue el goal dentro de su arco.
4. `AUTO_DISTANCE` selecciona RPM base; se aplica trim.
5. Telemetry muestra `base + trim = target`, RPM medida y error.
6. Driver sostiene `RB`.
7. Si no está listo, feeder permanece cero y aparece la razón principal.
8. Al cumplir pose/modo, torreta y RPM durante dwell, se permite feed.
9. Soltar `RB` corta feeder aunque el sistema siga listo.

No pulsar repetidamente RB intentando “forzar” el interlock. Leer la razón: RPM, turret limit, pose, target, fault o request.

## 8. Corregir overshoot/undershoot con trim

En `AUTO_DISTANCE`:

- tiro largo/overshoot: operador 2 pulsa `DPAD_DOWN` para -100 RPM;
- tiro corto/undershoot: pulsa `DPAD_UP` para +100 RPM;
- pantalla muestra base, trim y final;
- `X` restaura trim a 0;
- trim normal se limita a ±500 RPM y se reinicia en cada init.

Aplicar una corrección por vez y observar varios tiros si el ritmo lo permite. No compensar un error evidente de pose/aim con RPM.

El equipo debe revisar después los logs: un trim repetitivo por distancia probablemente indica que el modelo necesita recalibración.

## 9. Manual RPM y override

### 9.1 Entrar a `MANUAL_RPM`

Operador 2 pulsa `Y`. Confirmar que la pantalla cambia a `MANUAL_RPM` y muestra el setpoint. Usarlo sólo cuando la estrategia o el modo degradado lo requieran.

Manual RPM no desactiva:

- RPM mínima/máxima física;
- slew/overspeed protection;
- turret readiness;
- feeder interlock;
- E-stop.

### 9.2 Override de trim normal

La implementación final tendrá un chord sostenido que cambia `RPM_TRIM_LIMIT_ENABLED`. Cuando está desactivado:

- aparece `OVERRIDE` dominante;
- el operador puede recorrer el rango validado, no más;
- la acción queda en log;
- se cancela al reiniciar init;
- el clamp físico absoluto sigue vigente.

Si el target deseado exige superar el clamp físico, abortar y corregir calibración/mecánica; no existe un override desde gamepad para ese límite.

## 10. Target fuera del arco

Síntomas:

- estado `AT_LIMIT`;
- feeder blocked;
- flecha/indicación de girar chasis;
- rumble rate-limited.

Respuesta:

1. driver suelta request si lo sostenía;
2. gira el chasis manualmente en la dirección indicada;
3. evita golpes/cambios bruscos si la torreta está cerca del límite;
4. espera que el setpoint reentre con histéresis y estado vuelva a tracking/ready;
5. vuelve a solicitar el tiro.

No debe existir auto-turn del drivetrain ni intento de la torreta de cruzar el límite.

## 11. Pérdida de Limelight

Si aparece `ODOMETRY_ONLY`:

- seguir conduciendo sólo si pose quality se mantiene aceptable;
- esperar criterios más conservadores de feed según validación;
- no asumir que tener `tx=0` significa estar apuntado;
- evitar oclusiones prolongadas cuando se necesite revalidación;
- si la deriva/pose se invalida, targeting/feed se bloqueará.

Al recuperar frames válidos, la pose debe converger de forma acotada. Si la torreta salta violentamente, E-stop/Stop y finding.

## 12. `DEGRADED_FIXED_FORWARD`

Usar sólo cuando pose y visión no son confiables y la estrategia acepta apuntado manual del chasis.

### Entrada

1. asegurar que la torreta fue armada correctamente y no tiene encoder fault;
2. operador 2 sostiene `START+BACK` 1 s durante run;
3. confirmar rumble y banner `DEGRADED_FIXED_FORWARD`;
4. sistema lleva/retiene torreta en cero frontal dentro de límites;
5. cambiar/confirmar `MANUAL_RPM`.

Si la torreta no está armada o encoder es implausible, el modo no puede moverla ni permitir feed.

### Disparo degradado

1. driver apunta el chasis manualmente;
2. operador elige RPM dentro del rango validado;
3. sistema exige turret-at-zero y RPM estable;
4. driver mantiene RB;
5. feeder actúa sólo durante request y timeout;
6. soltar RB corta feed.

### Salida

Sostener nuevamente el chord aprobado o usar Stop/E-stop. No volver a normal hasta recuperar/revalidar pose y visión.

## 13. Jam de intake/feeder

Procedimiento final pendiente de validar controles y corrientes:

1. soltar request de disparo;
2. confirmar shooter/feeder bloqueados o a cero según procedimiento;
3. usar reversa sólo mientras se sostenga el control dedicado;
4. detener si hay ruido, corriente, calentamiento o pieza atrapada;
5. si no libera en el timeout corto, Stop y retirar con robot desenergizado según reglas del equipo.

Nunca dejar reversa latched ni meter manos con mecanismos energizados.

## 14. E-stop y Stop

La combinación exacta de E-stop debe confirmarse en el mapa final. Su efecto requerido es:

- cancelar/deshabilitar scheduler;
- `RobotSafety.stopAll()`;
- drive, intake, feeder, shooter y turret en cero;
- bloquear reactivación por periodic;
- exigir reiniciar OpMode e inspeccionar.

Usar E-stop/Stop ante movimiento inesperado, limit contact, overspeed, feed no solicitado, pérdida de control o peligro humano. No intentar navegar menús o telemetry primero.

## 15. Mensajes y respuesta

| Mensaje | Significado | Acción del operador |
|---|---|---|
| `ALLIANCE UNSELECTED` | No hay goal válido | X Blue o B Red durante init. |
| `POSE UNINITIALIZED/INVALID` | No hay geometría confiable | Confirmar preset/relocalizar; no disparar. |
| `TURRET DISARMED` | Cero no confirmado | Centrar y hold en init; no forzar. |
| `TURRET AT_LIMIT` | Goal fuera del arco | Driver gira chasis. |
| `VISION STALE` | Limelight no aporta frame fresco | Operar según odometría/política; revisar cámara. |
| `RPM NOT STABLE` | Shooter no completó dwell | Mantener request/esperar; revisar target/batería. |
| `OVERRIDE` | Límite normal de trim desactivado | Confirmar setpoint; mantener dentro de estrategia aprobada. |
| `FEED BLOCKED: ...` | Interlock no satisfecho | Corregir la razón; no intentar bypass. |
| `DEGRADED FIXED FORWARD` | Apuntado manual del chasis | Torreta cero, manual RPM, procedimiento degradado. |
| `FAULT / E-STOP LATCHED` | Outputs detenidos | Stop, desenergizar/inspeccionar y reiniciar. |

## 16. Shutdown post-match

1. soltar todos los requests;
2. presionar Stop;
3. confirmar que todos los mecanismos están quietos;
4. desenergizar según procedimiento del equipo;
5. inspeccionar cables de torreta, pods, shooter y feeder;
6. guardar logs/session ID;
7. anotar trims usados, fallas, jams, golpes y discrepancias;
8. no borrar faults sin crear finding si afectaron seguridad/score.

## 17. Checklist de entrenamiento

Cada operador debe demostrar en robot restringido o simulación apropiada:

- [ ] Identificar release y OpMode correcto.
- [ ] Seleccionar Blue/Red y pose.
- [ ] Centrar/armar sin movimiento previo.
- [ ] Explicar fused vs odometry-only.
- [ ] Solicitar un tiro y leer block reason.
- [ ] Aplicar +100/-100 y reset.
- [ ] Cambiar auto/manual.
- [ ] Explicar override y clamp físico.
- [ ] Responder a `AT_LIMIT` girando chasis.
- [ ] Entrar/salir de degraded.
- [ ] Resolver un jam simulado.
- [ ] Ejecutar E-stop/Stop.
- [ ] Hacer shutdown y reportar finding.

## 18. Condiciones para aprobar este manual

Quitar la advertencia de borrador sólo cuando:

- controles finales no tengan conflictos;
- E-stop y override chords estén definidos;
- textos/rumble coincidan con implementación;
- normal/degradado/jam hayan sido probados;
- operadores completen drills sin ayuda del programador;
- versión del manual se vincule al release;
- safety lead apruebe el procedimiento físico.
