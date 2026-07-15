# 05 — Programa de pruebas

> Estado: plan inicial; no se ha ejecutado contra la arquitectura objetivo
> Baseline de referencia: `origin/main@b5a134260456565df9d0295722ebecad900f21b4`
> Última actualización: 2026-07-15
> Alcance: pruebas estáticas, simuladas, físicas, fallas, aceptación y registro
> Responsable sugerido: test lead independiente del implementador de cada fase
> Fuente de verdad: resultados adjuntos a SHA/configuración; una prueba de escritorio no reemplaza validación física.

## 1. Objetivo

Demostrar que el robot funciona cuando los sensores son correctos y que se detiene o degrada de manera entendible cuando no lo son. El programa ordena las pruebas desde menor riesgo hasta mayor integración:

```text
Review/build -> unit/static -> replay/sim -> disabled inspection
 -> restrained/on-blocks -> low-power mechanism -> field calibration
 -> integrated shots -> fault injection -> driver drills -> scrimmage
```

No se salta directamente a disparos para “ver si funciona”. Cada nivel requiere aceptar el anterior y registrar cualquier desviación.

## 2. Roles durante pruebas físicas

| Rol | Responsabilidad | No debe combinarse cuando el riesgo sea alto |
|---|---|---|
| Test lead | Lee el caso, decide start/abort y registra resultado | No opera distraído mientras documenta. |
| Safety operator | Tiene acceso inmediato a Stop/E-stop y observa zona | No sostiene el robot ni recupera piezas. |
| Driver/operator | Ejecuta sólo inputs escritos | No improvisa tuning. |
| Mechanism observer | Vigila cables, hard stops, vibración, temperatura/corriente | No está en línea de tiro. |
| Logger | Guarda SHA, config, telemetry/log/video y counts | Puede ser combinado sólo en pruebas sin movimiento. |

Antes de habilitar: anunciar, despejar área, usar protección apropiada, elevar/restringir cuando corresponda y acordar una palabra de abort.

## 3. Identidad de una ejecución

Cada sesión crea un encabezado:

```text
Session ID:
Date/time/timezone:
Git SHA / branch / dirty status:
APK/build ID:
Robot Controller configuration export/hash:
Control/Expansion Hub firmware:
Limelight firmware + pipeline/fieldmap version:
Battery start/end voltage:
Mechanical configuration (wheel/pod/turret/shooter/feeder):
Field layout/manual revision:
Participants and roles:
Environmental notes:
```

Si cambia una pieza, relación, cámara, pod, firmware o constante crítica, la evidencia anterior no se hereda automáticamente.

## 4. Criterios de severidad y abort

Abort inmediato ante:

- movimiento antes de arm/start;
- dirección inesperada hacia hard stop;
- cable tensionado, contacto mecánico, olor, humo o vibración anormal;
- feeder/shooter que no se detiene al soltar/Stop;
- overspeed o corriente fuera de límite aprobado;
- pose/heading que salta y mueve la torreta;
- proyectil fuera de la zona segura;
- E-stop que no ordena cero en el primer scheduler cycle o tarda más de 50 ms;
- pérdida de comunicación o telemetry necesaria para la prueba.

Un abort es un resultado útil, no una invitación a repetir sin diagnosticar. Crear finding, llevar actuadores a cero, desenergizar si hace falta y revisar.

## 5. Gate T0 — Revisión estática y build

### T0.1 Descubrimiento de repo

- `git status --short` y rama/SHA.
- Releer `AGENTS.md` aplicable.
- Enumerar OpModes anotados y dinámicos.
- Trazar composition root y hardware owner.
- Revisar diff completo por scope accidental.

**Aceptación:** ningún archivo generado editado; ningún hardware name inventado; no hay owner duplicado.

### T0.2 Safety review por actuador

Para drive, intake, feeder, shooter y turret verificar:

- init power cero;
- clamp/límite/timeout;
- requisito FTCLib;
- `end(interrupted)`;
- stop de OpMode;
- E-stop;
- alternos `Action`/adapter;
- periodic no reactiva después de stop.

**Aceptación:** cada camino tiene stop explícito o se corrige antes de hardware.

### T0.3 Build y checks

Ejecutar desde PowerShell con el Java aprobado por el repo:

```powershell
$env:JAVA_HOME='C:\Program Files\Android\Android Studio\jbr'
.\gradlew.bat assembleDebug
git diff --check
```

Registrar cold/warm, warnings relevantes, APK y hash. No cambiar Gradle/SDK para “arreglar” un ambiente sin autorización.

**Aceptación:** build exitoso y cero errores introducidos por el paquete. Un fallo ambiental se documenta, pero deja T0 bloqueado; no cuenta como aceptación.

## 6. Gate T1 — Tests puros y estáticos

### T1.1 Matemática angular

Casos mínimos:

- wrap cerca de `-π`, `0`, `+π`;
- diferencia angular cruzando ±π;
- goals en cuatro cuadrantes;
- headings del robot en 0°, 90°, 180°, -90°;
- setpoint con y sin turret offset;
- ninguna solución equivalente dentro del arco.

**Aceptación:** error numérico dentro de tolerancia; nunca elige una vuelta prohibida.

### T1.2 Transforms

- compose/invert de field→robot→camera;
- unidad in↔m sólo en adapters explícitos;
- poses blue/red y simetría esperada;
- extrínseca cero y no cero;
- replay de cuatro anclas.

**Aceptación:** round-trip recupera pose dentro de tolerancia numérica y fixtures físicos dentro de gate.

### T1.3 Vision gates

Inyectar observaciones:

- null/invalid;
- stale;
- ID incorrecto;
- pose fuera del campo;
- residual alto;
- latencia alta con giro;
- observación correcta;
- recuperación tras varios frames correctos.

**Aceptación:** cada rechazo tiene razón determinista; ningún reject cambia pose/actuador.

### T1.4 RPM model

- límites exactos de distancia;
- distancia fuera de soporte;
- base + trim positivo/negativo;
- trim normal ±500;
- intento de exceder trim/clamp sin camino de override;
- NaN/infinito/dato inválido;
- slew y reset en init.

**Aceptación:** nunca comanda fuera de rango físico; modelo/version visibles.

### T1.5 Readiness/state machines

- request no sostenido;
- shooter entrando/saliendo de tolerancia;
- health de encoder/voltaje y overspeed;
- dwell incompleto/completo;
- turret at limit/fault;
- pose degradada/invalid;
- drive estacionario/movimiento lineal/movimiento angular;
- feeder `WAITING_READINESS/PULSING/COOLDOWN`;
- cámara perdida con odometría buena;
- entrada/salida de `DEGRADED_FIXED_FORWARD`;
- interruption/E-stop.

**Aceptación:** feeder sólo se permite en estados documentados y se corta inmediatamente en condiciones críticas.

## 7. Gate T2 — Replay, simulación y shadow mode

### T2.1 Replay de localización/visión

Guardar logs sincronizados de:

- timestamp loop;
- pose Pedro y calidad;
- pose/tx/ty/tags/latencia Limelight;
- observación aceptada/rechazada + razón;
- innovación antes/después de clamp;
- pose fusionada;
- bearing, trim y turret setpoint calculados.

Reproducirlos offline con el mismo algoritmo. Barrer alpha/beta/gates sin mover hardware.

**Aceptación:** mismo input produce mismo output; no hay salto fuera de límites; los parámetros elegidos se justifican con métricas.

### T2.2 Shadow targeting

Con el robot conducido pero la torreta deshabilitada, calcular y registrar setpoint. Compararlo visualmente con el goal y con una referencia manual.

**Aceptación:** signo, alianza, wrapping y respuesta a pérdida de visión son correctos antes de conectar control.

### T2.3 Simulación Pedro

Usar simulación sólo para lógica de trayectoria/pose que realmente comparte código con producción. No atribuir seguridad física a la simulación.

**Aceptación:** no hay discontinuidades de coordenadas o selección de goal; las limitaciones de la simulación quedan documentadas.

## 8. Gate T3 — Robot deshabilitado

Sin habilitar motores:

- verificar configuración del RC y nombres exactos;
- verificar que un solo subsystem mapea cada device;
- inspeccionar orientación IMU/pods;
- medir cámara, eje de turret y marcas de centro;
- confirmar cable slack a ambos extremos;
- abrir TeleOp repetidamente y probar init/stop sin NPE;
- seleccionar Blue/Red y presets;
- observar Limelight pipeline, frames, ID y timestamps;
- probar hold de armado sin permitir potencia si el estado no corresponde;
- desconectar cámara y verificar fault seguro.
- confirmar físicamente hood/webcam retiradas y Limelight instalada, sin asumir mapping/configuración.

**Aceptación:** init y stop repetibles; cero movimiento; todos los hechos físicos críticos registrados.

## 9. Gate T4 — Restrained/on-blocks a potencia reducida

### T4.1 Stop contract

Para cada actuador, a potencia reducida:

1. command normal;
2. soltar request;
3. interrumpir command;
4. cambiar a fault simulado;
5. presionar E-stop;
6. Stop de OpMode.

**Aceptación:** comando cero en el primer scheduler cycle y ≤50 ms de tiempo de pared; no se reactiva en periodic. El spin-down físico se registra aparte y no sustituye esta medida.

### T4.2 Torreta manual y límites

- confirmar signo encoder/motor;
- recorrer en pasos pequeños desde centro;
- medir ticks/grados hasta límites con margen;
- aproximar cada límite desde ambos sentidos;
- comprobar gate contra movimiento exterior y permitir movimiento de salida;
- probar pérdida/reinicio de encoder según escenarios realistas;
- medir backlash/repetibilidad.

**Aceptación:** cero movimiento desarmada; límites funcionan con signos correctos; margen físico documentado; no tocar hard stop.

### T4.3 Shooter/feeder/intake

- confirmar sentido a baja potencia;
- medir RPM vs comando y voltaje sin pieza;
- verificar overspeed/timeout/stop y fault injection de encoder congelado, salto/RPM imposible y voltaje inválido;
- feeder sólo hold-to-run en System Check; en integración usa pulsos acotados/cooldown;
- probar reversa anti-jam sólo con procedimiento;
- medir temperatura/corriente si disponible.

**Aceptación:** clamps y stop; un sensor inválido deja target/power cero; ninguna alimentación por un botón no interlocked en TeleOp de competencia.

### T4.4 Drive/odometría

- girar ruedas/pods manualmente y comprobar signos;
- drive forward/strafe/turn corto a baja potencia;
- confirmar heading y ejes;
- levantar un pod o simular dato inválido si es seguro.

**Aceptación:** pose avanza en dirección esperada y calidad refleja faults.

## 10. Gate T5 — Calibración de odometría

Usar marcas/medición independiente. Mínimo:

1. forward recto varias distancias y ambos sentidos, mínimo cinco repeticiones por sentido;
2. strafe ambos sentidos, mínimo cinco repeticiones por sentido;
3. giro 360° CW/CCW, mínimo cinco repeticiones por sentido;
4. cuadrado/rectángulo que vuelve al inicio;
5. ruta mixta acordada;
6. repetir con aceleraciones típicas.

Registrar error por segmento e inicial/final X/Y/heading, media, máximo y distribución, no sólo mejor intento.

**Gate inicial:** error final ≤2 in y ≤2° en la ruta repetida acordada. También revisar que no haya error sistemático creciente. Si el equipo cambia el gate, registrar decisión y datos.

## 11. Gate T6 — Limelight y fusión

### T6.1 Cámara estática

- poses/distancias conocidas;
- Blue 20 y Red 24;
- headings distintos;
- iluminación representativa;
- target parcial y múltiples tags;
- latencia/poll rate.

**Aceptación:** IDs correctos, extrínseca coherente, error/variación caracterizados.

### T6.2 Shadow y gates

- odometría buena + visión buena;
- visión con outlier artificial/real;
- tag incorrecto;
- cámara tapada 0.25, 1, 3, 5 s;
- recuperación;
- robot girando rápido;
- robot fuera de posición razonable.

**Aceptación:** razones de aceptación/rechazo coinciden; no hay corrección catastrófica.

### T6.3 Corrección activa sin torreta

Activar fusión, aún sin control de torreta. Medir:

- máximo salto por frame;
- tiempo de convergencia;
- error con/sin visión;
- deriva durante dropout;
- recuperación sin oscilación.

**Aceptación:** cumple gate de pose y clamps definidos; pérdida de cámara no invalida instantáneamente odometría buena.

## 12. Gate T7 — Control de torreta

Primero usar setpoints virtuales/con robot restringido, después goal real:

- ángulos dentro del rango;
- cerca de ambos soft limits;
- target que cruza cero;
- target fuera del arco;
- chassis heading cambia lentamente;
- pérdida de visión con odometría buena;
- pose inválida;
- encoder implausible;
- interruption y E-stop.

Métricas:

- error angular RMS/máximo;
- overshoot;
- settling time;
- dwell hasta ready;
- potencia/velocidad;
- intentos contra limit;
- fault/recovery.

**Gate inicial:** error estable ≤2.5°, sin cruce de límites ni movimiento desarmada. El target fuera de arco debe detener y guiar al conductor sin girar el drivetrain.

## 13. Gate T8 — Caracterización del shooter fijo

### T8.1 Control de velocidad

Por varios setpoints y voltajes:

- step response;
- tiempo a tolerancia;
- overshoot;
- error steady-state;
- caída al alimentar una pieza;
- recuperación;
- stop y restart;
- temperatura/vibración.

**Gate:** ±100 RPM durante ≥250 ms antes de feed en el rango aprobado, sin overspeed.

### T8.2 Dataset de tiro

Seleccionar distancias que cubran el rango de juego. En cada una:

- hacer warmup definido;
- usar pose/aim estacionario;
- registrar al menos 10 tiros por distancia en cada una de dos sesiones separadas;
- separar datos para ajustar y distancias intermedias retenidas;
- no ajustar después de cada miss sin registrar la intervención.

Comparar lineal, cuadrático máximo grado 2 y piecewise-linear.

**Gate inicial:** en cada sesión, ≥9/10 en cada distancia soportada de calibración y ≥8/10 en cada punto retenido/intermedio, con mecanismo y piezas representativas. No combinar sesiones para ocultar una celda fallida. Elegir el modelo más simple que lo cumpla.

### T8.3 Controles manuales y trim del operador

- DPAD up/down exactamente 100 por flanco;
- mantener botón no repite sin política definida;
- X vuelve a cero;
- init siempre reinicia trim;
- ±500 normal;
- clamp absoluto permanece;
- A inicia/arma RPM manual sólo con health sano;
- B detiene target y power;
- Y cambia modo sin doble toggle sólo después del gate MP-06;
- no existe override desde gamepad.

**Aceptación:** el operador corrige overshoot/undershoot sin perder conciencia del target final; logs guardan base/trim/clamp.

## 14. Gate T9 — Feeder e integración de disparo

Matriz de permisos:

| Request held | RPM/health ready | Turret/zero ready | Pose/mode ready | Drive stationary | Fault | Esperado |
|---|---|---|---|---|---|---|
| No | Cualquiera | Cualquiera | Cualquiera | Cualquiera | No | Feeder 0. |
| Sí | No | Sí | Sí | Sí | No | Feeder 0; razón RPM/sensor. |
| Sí | Sí | No | Sí | Sí | No | Feeder 0; razón turret/zero. |
| Sí | Sí | Sí | No | Sí | No | Feeder 0; razón pose/mode. |
| Sí | Sí | Sí | Sí | No | No | Feeder 0; razón drive moving. |
| Sí | Sí | Sí | Sí | Sí | Sí | Feeder 0; fault. |
| Sí | Sí | Sí | Sí | Sí | No | Pulso acotado, cooldown y revalidación. |

Casos extra:

- soltar request durante feed;
- RPM cae tras primer contacto;
- torreta sale de tolerancia;
- target entra en límite;
- camera dropout;
- E-stop/Stop;
- jam y reversa manual.
- sostener RB durante al menos 20 secuencias y contar pulsos/cooldowns;
- cruzar en ambos sentidos los umbrales lineal/angular de chasis.

**Aceptación:** cero feed no solicitado; ningún pulso extra en 20 secuencias; release/fault/E-stop corta en el primer ciclo y ≤50 ms. Un pulso iniciado no se completa después de perder permiso.

## 15. Gate T10 — Modos degradados y fallas

### 15.1 `ODOMETRY_ONLY`

- tapar cámara durante trayectorias típicas;
- medir deriva a 1/3/5 s;
- verificar indicador y política de feed;
- recuperar visión con corrección acotada.

### T10.2 `DEGRADED_FIXED_FORWARD`

- entrada sólo con hold state-aware;
- indicador/rumble inequívoco;
- torreta a cero si armada/plausible;
- bloquear si no armada/encoder fault;
- manual RPM dentro de clamp;
- request sostenido, dwell y timeout;
- conductor apunta chasis;
- salida/E-stop.

**Aceptación:** ningún uso de pose/visión inválida; no se convierte en bypass general de interlocks.

### T10.3 Fault injection

Cuando sea seguro y sin dañar hardware:

- resultado Limelight null/stale;
- pipeline equivocado;
- pose jump;
- encoder freeze/jump simulado;
- encoder/RPM/voltaje inválido de shooter;
- reset/brownout simulado que invalida `zeroValid` de torreta;
- scheduler interruption;
- battery low condition controlada;
- init/stop repetido;
- exception de subsystem si se puede inyectar en test.

**Aceptación:** fault visible, output seguro y recuperación definida.

## 16. Gate T11 — Operación y scrimmage

### Drills del equipo

- setup desde robot apagado;
- selección de alianza y preset;
- centro/arm de torreta;
- shot normal;
- trim -100/+100 y reset;
- manual RPM;
- target fuera de arco y giro manual;
- pérdida de cámara;
- modo degradado;
- jam;
- E-stop y shutdown.

El test lead introduce fallas sin avisar cuál, dentro de un guion seguro. El operador debe identificar el estado por feedback.

### Scrimmage

Ejecutar al menos dos sesiones separadas sobre el mismo SHA candidato con:

- batería/piezas representativas;
- posiciones de campo relevantes;
- tráfico/oclusiones razonables;
- logs completos;
- inspección mecánica posterior.

**Gate pre-cleanup:** dos sesiones cumplen criterios, no hay critical/high abierto y el equipo completa el manual sin ayuda del programador. Después de MP-09, MP-10 repite T0–T10 y estas dos sesiones sobre el SHA limpio antes del release.

## 17. Plantilla de caso de prueba

```markdown
### TEST-XXX — Nombre

- Requisito/decisión:
- Riesgo cubierto:
- SHA/config/session:
- Nivel: T0..T11
- Prerrequisitos:
- Área/equipo de seguridad:
- Estado inicial:
- Inputs/pasos exactos:
- Resultado esperado y tolerancia:
- Evidencia a guardar:
- Resultado observado:
- PASS / FAIL / ABORT / BLOCKED:
- Finding asociado:
- Responsable y fecha de retest:
```

## 18. Datos y métricas

CSV/log recomendado por loop/evento:

```text
timestamp_ns,session_id,sha,alliance,
pose_x_in,pose_y_in,heading_rad,pose_quality,
vision_valid,vision_age_ms,tag_ids,vision_x_in,vision_y_in,vision_heading_rad,
vision_reject_reason,innovation_x_in,innovation_y_in,innovation_heading_rad,
targeting_mode,target_distance_in,target_bearing_rad,vision_trim_rad,
turret_state,turret_target_rad,turret_measured_rad,turret_power,
shooter_mode,base_rpm,trim_rpm,target_rpm,measured_rpm,
feed_request,feed_allowed,readiness_reason,feeder_power,
fault_code,battery_voltage
```

Registrar transiciones/eventos aunque se reduzca la frecuencia de samples. No guardar datos personales innecesarios.

## 19. Cierre de findings

Un finding se cierra sólo cuando:

1. existe cambio o explicación de no-cambio;
2. se identifica el test que reproduce el problema;
3. el test falla en baseline cuando sea seguro demostrarlo;
4. pasa con el fix;
5. pasa una regresión relevante;
6. la evidencia está vinculada;
7. el responsable revisa stop/failure paths.

“No volvió a pasar” sin una prueba controlada no es cierre.

## 20. Checklist de aprobación final

- [ ] Build limpio y reproducible.
- [ ] Hardware configuration versionada/identificada.
- [ ] Tres dead wheels calibradas, ≤2 in/2° en gate.
- [ ] Limelight lifecycle y gates aprobados.
- [ ] Transform de cuatro anclas en ambas alianzas.
- [ ] Torreta cero/límites/error ≤2.5°.
- [ ] Shooter ±100 RPM por 250 ms.
- [ ] Modelo ≥9/10 calibración y ≥8/10 retenido en cada distancia y sesión, 10 tiros por celda.
- [ ] Trim/manual dentro de clamp; cero override de gamepad.
- [ ] Cero feed involuntario.
- [ ] Pulsos/cooldown acotados y robot estacionario antes de cada feed.
- [ ] Stop en primer ciclo y ≤50 ms.
- [ ] Normal, odometry-only y degradado ensayados.
- [ ] Dos scrimmages aceptados antes y después de cleanup sobre sus SHAs exactos.
- [ ] Sin critical/high abiertos.
- [ ] Operadores aprueban el manual.
- [ ] Snapshot/tag y rollback verificados antes de limpieza.
