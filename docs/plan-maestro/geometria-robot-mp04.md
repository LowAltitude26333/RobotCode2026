# Geometría física del robot — insumo crudo para MP-04

> Estado: acumulando evidencia física de forma incremental; el equipo la va entregando por partes.
> Última actualización: 2026-07-22
> Esta hoja NO es el "Contrato de marcos de coordenadas" formal que pide `03-auto-aim-limelight-y-cancha.md` sección 4 (ese sigue siendo el deliverable de MP-04, que continúa `NOT_STARTED` según `handoff-MasterPlan.md`). Aquí sólo se registran, sin editar ni interpretar, los datos crudos que el equipo va midiendo, para que MP-04 los use como insumo cuando se abra formalmente.
> No inventar ni interpolar ningún campo marcado `PENDIENTE`/`___`. Cada actualización de esta hoja debe decir qué llegó nuevo y qué sigue faltando.

**Actualización 2026-07-22 — qué llegó nuevo y qué sigue faltando:**

- El equipo reportó una corrección: los cinco centros ópticos de la Limelight (secc. 1) y el punto `T` de la torreta (secc. 2) suben `+10.79 mm` en Z respecto a la entrega anterior (`X`, `Y` y los vectores unitarios no cambiaron). Es un desplazamiento uniforme en las seis filas afectadas, consistente con una corrección del plano de referencia Z más que con una remedición punto por punto — no se pregunta la causa aquí porque no se pidió, pero vale confirmarla con quien midió antes de dar esto por cerrado.
- El signo de `T0` (secc. 2) llegó corregido: ahora coincide con el signo de `T` (`-50.81` en ambos), cerrando la inconsistencia geométrica real que estaba abierta desde 2026-07-21 (pregunta abierta #2 de esa fecha — ver historial).
- Llegó completa la sección 3 (Shooter en cero), antes sólo plantilla. Se verificó por cálculo que `S - T`, `ρ`, `h` y el punto auxiliar `D` son consistentes entre sí (ver nota en secc. 3).
- El equipo confirmó que la configuración de Limelight físicamente montada ahora mismo es `30°` (secc. 1) — cierra ese pendiente.
- **Sección 4 cerrada.** El equipo decidió reusar el límite de software ya aceptado en ticks (`-983/+1070`) en vez de abrir una sesión de medición física aparte, y confirmó de memoria (con la posición del motor/cable como evidencia) que `+1070` barre hacia la derecha del robot — eso fija el signo. Rango final: yaw `[-175.97°, +161.66°]` (ver secc. 4 y 4.1). Lo único que no quedó resuelto es el desglose mecánico vs. cableado por separado (nueva pregunta abierta #2), que no bloquea el uso de este rango.

## 0. Sistema de coordenadas (aportado por el equipo, 2026-07-21)

- **Origen:** centro de giro del robot proyectado al plano del chasis → `(0.00, 0.00, 0.00) mm`.
- **+X:** hacia el intake.
- **+Y:** hacia la izquierda.
- **+Z:** hacia arriba.
- **Yaw positivo:** antihorario, visto desde arriba.

**Confirmado 2026-07-21 (aclaración verbal del equipo):**

- El "plano del chasis" al que se proyecta el origen **es el piso de la cancha** — no es una altura intermedia distinta. Esto significa que el Z de esta hoja ya es directamente comparable con el Z del marco de cancha/SDK (tabla de poses de tags en `03-auto-aim-limelight-y-cancha.md` sección 3) sin offset adicional que buscar.
- El origen se toma como el **centro geométrico del robot** (no un punto arbitrario del chasis). Los valores de la Limelight se explican con esa referencia: `Y=0` porque la cámara está montada sobre la línea central del robot (simétrica en Y); `X` pequeño positivo porque el punto de montaje está un poco adelantado respecto al centro; `Z` cambia entre las cinco configuraciones porque, al rotar el soporte para cambiar el pitch, el punto físico del lente sube/baja.
- Para la torreta, `Y=0` en `T`/`T0` se explica igual: la torreta está centrada sobre el eje Y del robot (sin desfase lateral), no por coincidencia.
- La Limelight sólo tiene grados de libertad mecánicos en pitch — `Yaw=0.00°` y `Roll=0.00°` son cero en las cinco configuraciones porque el dispositivo físicamente no gira en esos dos ejes, no es una casualidad de medición.
- **Confirmado 2026-07-22:** el frente físico del robot es el lado del intake. Esto cierra la pregunta de si "+X hacia el intake" coincide con el frente que ya usa Pedro en producción: `PedroDriveAdapter.toPedroRobotCentric` ya define su convención robot-céntrica como `(+adelante, +izquierda, +CCW)` — mismo signo, mismo sentido. No hay dos definiciones de "frente" compitiendo en el robot; el `+X` de esta hoja y el `forward` que usa Pedro apuntan al mismo lado físico.

**Consecuencia derivada, no medida directamente — verificar físicamente antes de usarla en código:**

- La torreta en cero (`Dirección en cero`, secc. 2) apunta al vector `(1.00, 0.00, 0.00)`, es decir, en la misma dirección que `+X` = intake = frente del robot. Dicho de otro modo: con esta convención, el cero mecánico de la torreta mira hacia el mismo lado que el intake. Vale la pena una confirmación visual rápida con el equipo (¿el cero de la torreta efectivamente mira hacia el frente/intake y no hacia atrás?) antes de usar este dato en el cálculo de `turretZeroOffset` de `03-auto-aim-limelight-y-cancha.md` sección 8, porque si resulta que apunta hacia atrás, el signo de todo el bloque 2 se invertiría.

**Preguntas abiertas que todavía no se cerraron (no asumir, preguntar al equipo):**

1. Unidades: todo lo reportado por el equipo viene en mm y grados. El resto del código mide pods/torreta en pulgadas (`in/tick`, ver `contrato-hardware.md`). Falta decidir si esta geometría se convierte a pulgadas al integrarla o si se mantiene en mm como marco físico separado.
2. **(Nueva, 2026-07-22)** Cuál extremo (`LIMIT_LEFT`/mecánico `-993` o `LIMIT_RIGHT = +1070`) es tope mecánico y cuál es tope por cableado — el código no lo distingue, sólo guarda un rango de ticks ya conciliado. No bloquea el uso de la sección 4 (ver esa sección), pero si se necesita el desglose por separado hace falta una sesión física dedicada.

Cerradas en esta ronda (2026-07-22): el signo de X entre `T` y `T0` (secc. 2); cuál configuración de Limelight está físicamente montada, es la de `30°` (secc. 1); y el sentido de giro de `LIMIT_LEFT`/`LIMIT_RIGHT` respecto al yaw de esta hoja — girar hacia `+1070` barre hacia la derecha (yaw negativo), confirmado de memoria por el equipo con la posición del motor/cable como evidencia (secc. 4.1).

## 1. Limelight — extrínseca de cámara (aportado por el equipo, 2026-07-21; Z corregida 2026-07-22)

La Limelight tiene **cinco configuraciones discretas de inclinación** (no es un solo valor fijo). `Yaw = 0.00°` y `Roll = 0.00°` en las cinco. Convención de pitch: positivo = eje óptico apuntando hacia arriba respecto al plano del chasis.

| Config. | Centro óptico (mm) | Pitch | Vector óptico unitario | Montaje |
|---|---|---:|---|---|
| 30° | `(134.92, 0.00, 293.63)` | `30.00°` | `(0.8660, 0.0000, 0.5000)` | **ACTIVA — confirmada 2026-07-22** |
| 40° | `(130.31, 0.00, 292.60)` | `40.00°` | `(0.7660, 0.0000, 0.6428)` | — |
| 50° | `(125.95, 0.00, 290.78)` | `50.00°` | `(0.6428, 0.0000, 0.7660)` | — |
| 60° | `(121.98, 0.00, 288.24)` | `60.00°` | `(0.5000, 0.0000, 0.8660)` | — |
| 67.2631° | `(119.40, 0.00, 285.97)` | `67.2631°` | `(0.3865, 0.0000, 0.9223)` | — |

**Punto físico usado como centro óptico:** centro geométrico de la abertura circular visible del lente, sobre el plano frontal del lente. Aproximación física del centro óptico interno de la cámara, no una calibración óptica formal.

**Verificado por cálculo (2026-07-22):** en las cinco filas, `(cos(pitch), 0, sin(pitch))` reproduce el vector unitario reportado dentro del redondeo de 4 decimales — la tabla es internamente consistente. La corrección de Z respecto a la entrega anterior es un desplazamiento uniforme de `+10.79 mm` en las cinco filas (mismo valor exacto que en `T`, secc. 2); `X` y los vectores no cambiaron.

**Confirmado 2026-07-22 (equipo):** la configuración físicamente montada ahora mismo es **30°**. Esto cierra el pendiente que bloqueaba elegir la fila correcta de extrínseca para el pipeline (`03-auto-aim-limelight-y-cancha.md` sección 5.1) y para `mp03-limelight-commissioning.md` — ambos documentos deben usar la fila `30°` de esta tabla, no otra, hasta que el equipo reporte un cambio de montaje.

## 2. Torreta — geometría del eje (aportado por el equipo, 2026-07-21; corregido 2026-07-22)

- **Punto de referencia sobre el eje:** `T = (-50.81, 0.00, 263.94) mm`.
- **Proyección del eje sobre el chasis:** `T0 = (-50.81, 0.00, 0.00) mm`.

  El signo de X entre `T` y `T0` llegó corregido el 2026-07-22 (ambos `-50.81` ahora). Cierra la inconsistencia detectada el 2026-07-21: como la dirección del eje es puramente vertical `a=(0,0,1)`, `T0` (proyección de `T` al piso) matemáticamente debe compartir X e Y con `T`, sólo Z distinto — ya se cumple.
- **Dirección del eje:** `a = (0.00, 0.00, 1.00)` (vertical, coincide con +Z).
- **Dirección en cero:** yaw `= 0.00°`, vector horizontal `= (1.00, 0.00, 0.00)` (alineado con +X, es decir con el intake).

Este dato alimenta el transform `T_robot_turret` que pide `03-auto-aim-limelight-y-cancha.md` sección 4.1.

Nota: la Z de `T` también subió `+10.79 mm` respecto a la entrega del 2026-07-21 (`253.15` → `263.94`), el mismo desplazamiento uniforme que en la tabla de Limelight (secc. 1).

## 3. Shooter en cero (aportado por el equipo, 2026-07-22)

Configuración: torreta en su marca mecánica de cero, disparo hacia el intake (`+X`).

- Punto efectivo de salida `S = (41.82, 0.00, 385.99) mm`.
- Vector unitario de salida `d = (0.3865, 0.0000, 0.9223)`.
- Yaw de salida `= 0.00°`; elevación `= 67.2631°`.
- Posición respecto a `T`: `S - T = (92.63, 0.00, 122.05) mm`.
- Distancia radial al eje `ρ = 92.63 mm`; desplazamiento vertical `h = 122.05 mm`.
- Punto auxiliar 100 mm adelante sobre `d`: `D = (80.47, 0.00, 478.22) mm`.

**Definición de "salida efectiva" aportada por el equipo:** centro geométrico de la pieza de juego en el instante en que deja de estar controlada por el último rodillo/superficie guía del mecanismo. La dirección de salida es la tangente de la trayectoria del centro de la pieza en ese punto.

**Verificado por cálculo (2026-07-22), todo consistente:**

- `S - T = (41.82-(-50.81), 0.00-0.00, 385.99-263.94) = (92.63, 0.00, 122.05)` mm — coincide exacto con el valor reportado.
- `ρ = hypot(92.63, 0.00) = 92.63 mm` y `h = 122.05 mm` — ambos coinciden.
- `d` reproduce `(cos(67.2631°), 0, sin(67.2631°)) ≈ (0.3865, 0.0000, 0.9223)` — coincide con el vector reportado, y ese mismo valor numérico también aparece como la fila más inclinada de la tabla de Limelight (secc. 1). **Actualización 2026-07-22:** con el montaje activo de la Limelight confirmado en `30°` (no `67.2631°`), queda claro que la elevación fija del shooter y el pitch de la Limelight son independientes — no comparten soporte ni se mueven juntos. Que `67.2631°` aparezca en ambas listas es, entonces, una coincidencia de valores (o un ángulo de referencia reutilizado en el diseño), no evidencia de acoplamiento mecánico. No cambia el uso del dato.
- `D = S + 100·d = (41.82+38.65, 0.00, 385.99+92.23) = (80.47, 0.00, 478.22)` mm — coincide exacto con el valor reportado.

No queda ningún campo `PENDIENTE` en esta sección.

## 4. Límites de giro

Decisión del equipo (2026-07-22): en vez de abrir una sesión física de medición aparte solo para esta plantilla, se cierra esta sección reusando el límite de software ya aceptado en ticks (`-983/+1070`, ver `contrato-hardware.md` fila Torreta, MP-01 `ACCEPTED`), convertido a grados con la relación de 4.1 y con el signo confirmado de memoria por el equipo (ver 4.1). Esto **no** identifica por separado el tope mecánico y el tope por cableado — ver limitación 1 más abajo.

- Límite mecánico negativo/positivo: **no identificado por separado** — ver nota abajo.
- Límite por cableado negativo/positivo: **no identificado por separado** — ver nota abajo.
- **Límite recomendado de software (yaw, convención secc. 0): `[-175.97°, +161.66°]`**, arco contiguo que pasa por `0°` (frente/intake). Corresponde exactamente a `LIMIT_RIGHT = +1070` ticks → `-175.97°` y `LIMIT_LEFT = -983` ticks → `+161.66°` (ver 4.1).

### 4.1 Referencia cruzada ticks↔grados

`contrato-hardware.md` especifica goBILDA 5203 Yellow Jacket `751.8 PPR` en el eje de salida del motor (ya incluye la reducción interna `26.9:1`) más una reducción externa por engranes `68T` (motor) → `198T` (corona de la torreta). `TurretSubsystem.java` lee `turretMotor.getCurrentPosition()` directamente del encoder del motor, es decir, **antes** de la reducción externa — por eso hace falta aplicar el `198/68` para pasar de ticks del motor a grados de la torreta:

```text
ticks/rev_torreta = 751.8 × (198/68)   ≈ 2189.06 ticks/rev
ticks/°           = 2189.06 / 360      ≈ 6.0807 ticks/°
°/tick            = 1 / 6.0807         ≈ 0.16445 °/tick
```

**Magnitud validada de memoria por el equipo (2026-07-22):** con el shooter en cero mirando al intake, `+1070` ticks (`LIMIT_RIGHT`) lo deja apuntando casi hacia atrás ("180 grados aprox"). Coincide con el `≈175.97°` calculado (a 4° de 180°) — confirma que la relación PPR/engranes de arriba es correcta en magnitud.

**Sentido confirmado de memoria por el equipo (2026-07-22):** el motor de la torreta está montado del lado derecho del robot, y el cable sale de ese mismo lado; al girar hacia `+1070` la torreta barre hacia la **derecha** (`-Y`), no hacia la izquierda — a los "casi 180°" se llega por ese lado, quedando el motor del lado izquierdo y el cable hacia atrás. Bajo la convención de yaw de la secc. 0 (positivo = antihorario visto desde arriba = hacia la izquierda), girar hacia la derecha es **yaw negativo**. Por lo tanto los ticks positivos (`LIMIT_RIGHT`) mapean a yaw negativo y los ticks negativos (`LIMIT_LEFT`) mapean a yaw positivo — signo invertido respecto a los propios ticks:

| Ticks | Yaw equivalente (secc. 0) |
|---:|---:|
| `LIMIT_RIGHT = +1070` (software, tolerancia ya aplicada) | `≈ -175.97°` |
| `LIMIT_LEFT = -983` (software, con margen de 10 ticks) | `≈ +161.66°` |
| `-993` (extremo bruto, según comentario en código: *"el límite negativo conserva 10 ticks desde el extremo -993"*) | `≈ +163.30°` |

Arco total: `≈ 337.6°`, punto medio corrido `≈ 7.15°` hacia el lado negativo (derecha) respecto al cero mecánico — no está centrado. La zona inalcanzable (`≈22.4°`) queda cerca de "apuntando hacia atrás": entre `+161.66°` y `+180°` (`≈18.3°`, lado izquierdo) y entre `-180°` y `-175.97°` (`≈4°`, lado derecho) — el motor/cable, del lado derecho, permite acercarse mucho más a los 180° por ese lado antes de bloquearse.

**Confianza de esta sección:** magnitud y sentido están validados de memoria con una explicación física coherente (posición del motor y del cable), no con una lectura en vivo del encoder. Es razonablemente confiable para cerrar esta plantilla, pero si más adelante se nota un comportamiento de aim inconsistente con este rango, re-verificar contra el robot antes de asumir que el bug está en otro lado.

**Limitación que sigue abierta, no bloqueante para cerrar esta sección:**

1. El comentario en `TurretSubsystem.java` distingue un límite **mecánico** (`-993`, con margen de 10 ticks) del límite por **cableado**, pero el código no registra cuál extremo (izquierdo o derecho) es mecánico y cuál es de cableado. Esta hoja documenta el límite de software ya aceptado como un solo rango recomendado; si en el futuro hace falta el desglose mecánico/cableado por separado (p. ej. para ajustar el margen de seguridad de forma independiente en cada lado), hará falta una sesión física dedicada — el procedimiento de 4.2 sigue disponible para ese caso.

### 4.2 Procedimiento para una futura sesión física (si hiciera falta desglosar mecánico vs. cableado)

Con el patrón de seguridad que ya usa el commissioning de la torreta (`TurretSubsystem`: cero manual confirmado, jog hold-to-run a `LIMIT_APPROACH_POWER = 0.05`, watchdog, BRAKE):

1. Robot inhibido salvo torreta, acceso inmediato a Stop, sin piezas cargadas en el shooter/kicker.
2. Confirmar el cero mecánico de la torreta (marca física) y armar (`confirmCenteredAndResetEncoder`) para que los ticks leídos sean comparables con el yaw de esta hoja.
3. Con potencia baja (jog de comisionamiento, no `setPower` libre), llevar la torreta hacia un lado hasta el primer tope físico real. Registrar por separado: ¿el tope es mecánico (choca una pieza) o de cableado (el cable/mazo se tensa antes de llegar al tope mecánico)? Anotar los ticks del encoder en ese punto y convertir a grados con la convención de yaw de la secc. 0 (no con la tabla de 4.1, que es sólo referencia).
4. Repetir hacia el otro lado.
5. Definir el límite recomendado de software como el más restrictivo de los dos (mecánico vs. cableado) por lado, menos un margen de seguridad — documentar qué margen se usó y por qué.
6. Reconciliar el resultado en grados contra `-983/+1070` ticks ya aceptados (usando 4.1): si el arco medido en grados no corresponde razonablemente al arco derivado de los ticks aceptados, es señal de que alguno de los dos supuestos (PPR, relación de engranes, o el propio ticks-limit) necesita revisión antes de tocar `contrato-hardware.md` o `TurretSubsystem.java`.

## Referencias

- [MP-03 — commissioning de Limelight 3A](mp03-limelight-commissioning.md)
- [03 — Auto-aim, Limelight y cancha](03-auto-aim-limelight-y-cancha.md), sección 4 (contrato de marcos, todavía `Pendiente`) y 4.1 (transforms necesarias)
- [Contrato de hardware](contrato-hardware.md)
- [Handoff MasterPlan](handoff-MasterPlan.md)
