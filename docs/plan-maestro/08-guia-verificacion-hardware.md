# 08 — Guía de verificación de hardware y constantes

> Estado: procedimiento documental; **no autoriza energizar ni mover hardware**
> Baseline de código inspeccionado: `origin/main@b5a134260456565df9d0295722ebecad900f21b4`
> Última actualización: 2026-07-15
> Alcance: configuración RC, mappings, direcciones, signos, unidades, límites y evidencia física
> Responsable sugerido: safety lead con responsables mecánico, eléctrico y de software
> Regla central: todo valor no verificado es `TBD-BLOCKING` y mantiene inhibido el actuador o automatismo afectado.

## 1. Propósito y límites

Esta guía convierte unknowns físicos en datos revisables sin inventar nombres, signos o constantes. Una constante Dashboard, comentario, valor de otro robot o compilación exitosa no demuestra hardware.

La fase documental actual sólo prepara las hojas. La inspección deshabilitada, pruebas con robot elevado, movimiento a baja potencia y field testing requieren autorización posterior, área despejada, responsable con Stop/E-stop y gates previos aprobados.

## 2. Paquete de evidencia obligatorio

Cada sesión crea una carpeta/registro con:

- fecha, hora y zona horaria;
- responsables que midieron y revisaron;
- Git SHA, rama y dirty status;
- export/capturas de Robot Controller configuration y su hash o identificador;
- versiones de RC/DS, Control/Expansion Hub y firmware relevante;
- batería inicial/final;
- instrumento usado, resolución y unidad;
- mínimo de repeticiones indicado por esta guía;
- tabla de valores crudos, cálculo y valor aceptado;
- foto/video/log ID cuando aplique;
- finding asociado si algo difiere de lo esperado.

No editar los datos crudos para que coincidan con una constante deseada. Una corrección crea una nueva fila/revisión.

## 3. Inventario y estado inicial

| Área | Nombre observado/propuesto | Estado físico reportado | Datos que bloquean uso |
|---|---|---|---|
| Drive | `leftFront`, `rightFront`, `leftBack`, `rightBack` | Presente; no verificado por esta documentación | Puertos, direcciones, inversión, zero-power, corriente y convención de ejes. |
| Odometría | `par0`, `par1`, `perp` | Tres pods reportados | Puerto, signo, ticks/rev, diámetro efectivo, offsets y contacto con suelo. |
| IMU | `imu` | Observada en código | Tipo, orientación logo/USB, heading positivo y calibración. |
| Shooter | `Shooter` | Un motor confirmado por el equipo | Modelo, encoder, ticks/rev efectivos, ratio, sentido, RPM/corriente/temperatura máximas. |
| Shooter legado | `Shooter2` | No debe asumirse presente | Confirmación en export antes de retirar declaración legacy. |
| Intake | `intakeMotor` | Presente; pendiente | Dirección, power, corriente, jam y stop. |
| Feeder motor | `kickerMotor` | Confirmado por equipo; falta export RC | Dirección, power, duración de pulso, cooldown, corriente, jam y reversa. |
| Feeder CRServo | `kickerServo` | Confirmado por equipo; falta export RC | +0.5 kick, -0.5 reverse, 0 stop; simultaneidad y corriente. |
| Torreta | `torretaMotor` | Presente con cero manual | Dirección, signo, ticks/grado, marca/fixture, arco, frenado, backlash y cable slack. |
| Hood | `hoodLeft`, `hoodRight` en código | Equipo reporta hardware retirado | Evidencia física y ausencia/presencia en configuración RC. |
| Webcam | `Webcam 1`/`Webcam` en código | Equipo reporta hardware retirado | Evidencia física y configuración; no diseñar coexistencia. |
| Limelight | `limelight` sólo propuesto | Equipo reporta dispositivo instalado | Nombre RC, firmware, pipeline, red, orientación, montaje y extrínseca. |

## 4. Orden seguro de verificación

1. **Documental:** comparar código contra export RC sin habilitar el robot; marcar cada discrepancia.
2. **Mecánica desenergizada:** inspeccionar fijación, cableado, holguras, hard stops, pods y marcas.
3. **Sensores con actuadores inhibidos:** leer encoders/IMU/Limelight y mover manualmente sólo elementos cuyo procedimiento mecánico lo permita.
4. **Restringido y baja potencia:** ejecutar únicamente después de MP-01, con robot elevado o mecanismo desacoplado, límites conservadores y E-stop probado.
5. **Carga representativa:** medir corriente, temperatura, repetibilidad y pieza sólo después del gate restringido.
6. **Campo controlado:** validar dinámica y criterios de aceptación; nunca usar el primer movimiento como calibración de competencia.

Si el signo, nombre, límite, corriente o comportamiento difiere, ordenar cero, detener la sesión y crear finding. No probar el signo contrario “a ver si funciona” sin revisar el procedimiento.

## 5. Hojas por sistema

### 5.1 Drive e IMU

Registrar por motor: hub/puerto, string exacto, dirección lógica, giro físico con power positivo, encoder si se usa, `ZeroPowerBehavior`, corriente sin carga y owner de software. Confirmar forward, strafe y turn a potencia reducida; los cuatro deben coincidir con la convención documentada.

Para IMU registrar tipo, orientación física logo/USB, heading al init y cambio de signo en al menos cinco giros manuales CW y cinco CCW. No corregir un signo de IMU invirtiendo fórmulas al azar.

**Desbloquea:** drive manual únicamente cuando mappings/direcciones/Stop son correctos; movimiento automático espera además odometría calibrada.

### 5.2 Tres pods de odometría

Por `par0`, `par1` y `perp` registrar:

- hub/puerto y encoder real;
- ticks por revolución nominales y efectivos;
- diámetro de rueda medido y distancia por tick;
- posición X/Y respecto al centro de giro, con dibujo y convención;
- signo al empujar forward/left y al girar CW/CCW;
- presión/contacto, juego y repetibilidad.

Usar rolling tests de varias distancias y al menos cinco repeticiones por sentido. Después ejecutar forward, strafe, 360° CW/CCW y ruta mixta según T5; reportar error por segmento, media y máximo.

**Desbloquea:** `PoseProvider`/Pedro sólo al cerrar todos los datos y cumplir ≤2 in/≤2° sin deriva sistemática.

### 5.3 Torreta

1. Con robot desenergizado, definir y fotografiar marca/fixture central.
2. Verificar cable slack y margen físico hacia ambos lados sin forzar hard stops.
3. Después de aprobar MP-01, probar dirección a power ≤0.1 y confirmar signo de encoder.
4. Medir ticks y grados desde centro hasta límite seguro, aproximando desde ambos sentidos al menos diez veces.
5. Registrar backlash, repetibilidad, zona de frenado y margen exterior.
6. Simular/reproducir init, reset y brownout: todos deben producir `zeroValid=false` y bloquear movimiento/feed.

El reset de encoder sólo ocurre después del hold de 1 s con marca confirmada. Si la posición visual no coincide, Stop; no compensar alterando offsets.

### 5.4 Shooter de un motor

Registrar modelo de motor, encoder, ticks/rev, ratio externo, sentido de lanzamiento, `ZeroPowerBehavior`, voltaje, corriente, vibración, temperatura y RPM medida con instrumento/log independiente cuando sea posible.

La caracterización sube setpoint por etapas dentro de límites aprobados, nunca usando el valor actual como prueba de seguridad. Probar encoder congelado/ausente, RPM imposible, voltaje inválido y overspeed: todos deben producir target/power cero y latch de fault.

`MIN/MAX_VALIDATED_RPM`, slew y timeout permanecen `TBD-BLOCKING` hasta revisión mecánica y dataset T8. No sustituir voltaje inválido por uno nominal para permitir output.

### 5.5 Intake y feeder

Para intake registrar dirección de entrada/salida, power útil mínimo, corriente libre/cargada, jam, reversa y Stop. El modo final no puede dejar un estado latched sin indicador y salida explícita.

Para feeder medir con pieza representativa:

- dirección y power mínimo confiable;
- duración mínima/máxima de un pulso;
- cooldown que evita doble alimentación;
- corriente/temperatura y condición de jam;
- reversa anti-jam sólo hold-to-run con timeout.

Power, pulso y cooldown no tienen default seguro: permanecen `TBD-BLOCKING`. T9 exige veinte secuencias sin pulso extra y corte en el primer scheduler cycle/≤50 ms.

### 5.6 Limelight y hardware retirado

Para Limelight registrar string exacto, firmware/app, pipeline, poll rate, field map, red no secreta necesaria para reproducibilidad, orientación, X/Y/Z/yaw/pitch/roll respecto al chasis y rigidez del montaje. Validar con cuatro anclas y ambas alianzas antes de corregir pose o bearing.

Para hood y webcam adjuntar foto/inspección y export RC que confirme su estado. Si siguen configurados aunque estén retirados, registrarlo como deuda; no mapear un sustituto ni borrar strings hasta la fase de cleanup autorizada.

## 6. Plantilla de medición

```text
MEAS-XXX — sistema / variable
SHA / configuración RC:
Fecha / responsables:
Estado inicial y restricción física:
Instrumento / resolución:
Unidad y convención:
Valores crudos (repeticiones):
Cálculo / incertidumbre:
Valor candidato:
Límite/margen aplicado:
Stop/E-stop verificado:
Evidencia (foto/log/video):
Revisor y decisión: VERIFIED | REJECTED | TBD-BLOCKING
Finding/test relacionado:
```

## 7. Regla para retirar `TBD-BLOCKING`

Un dato pasa a `VERIFIED` sólo cuando:

1. nombre, unidad y convención son explícitos;
2. existe evidencia repetible, no una única lectura conveniente;
3. responsable mecánico/eléctrico revisa lo que pueda causar daño;
4. software owner confirma que todos los callers usan el mismo dato y límites;
5. Stop, interruption y E-stop pasan su gate;
6. el resultado queda vinculado a SHA/configuración/test.

Si cambia motor, ratio, rueda/pod, hub/puerto, montaje, firmware, cableado o geometría, se invalida la medición afectada y vuelve a `TBD-BLOCKING`.

## 8. Cierre de la guía

La guía está completa cuando el equipo puede señalar, para cada dispositivo, mapping, owner, dirección/signo, unidades, límites, health, stop path y evidencia. Completar tablas no declara el robot seguro: habilita las pruebas escalonadas del programa y nada más.
