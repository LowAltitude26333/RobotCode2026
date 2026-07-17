# Handoff de tarea — MP-00 + MP-01A

Rama: `masterplan`

Base: `a887fe4`

Última actualización: 2026-07-17

Estado: **implementación de software pendiente de pruebas físicas**

Este archivo se actualiza con cada modificación material del plan maestro. Debe indicar qué cambió, cuál es la próxima acción de Codex y qué evidencia debe aportar el equipo desde el robot real.

## Modificaciones realizadas

- Se rebaselinó la documentación al merge endurecido `a887fe4` sin borrar la evidencia histórica de `b5a1342`/`f91af18`.
- `SafeCommandOpMode` incorpora `duringInitLoop()` y atiende `gamepad1 BACK` durante init y ejecución mediante cancelación del scheduler, `RobotSafety.stopAll()` y solicitud de Stop.
- El armado de torreta usa estados `WAITING`, `HOLDING` y `ARMED`; exige `START+BACK` de gamepad 2 durante 1,000 ms y resetea el encoder una sola vez.
- Init muestra estado, progreso y la advertencia de que el centro es una confirmación física manual.
- `MainTeleOp` y `TeleopTorreta` asignan el `VisionPortal` antes de registrar el cleanup de esa instancia.
- `TeleopTorreta` ya no programa un comando de shooter con referencia nula.
- Por decisión del equipo, `KICKER_OUT_SPEED` cambia de `0.70` a `0.85`; queda pendiente su validación física controlada.
- Se creó [contrato-hardware.md](contrato-hardware.md) para recibir el export real y las validaciones; no se inventaron puertos, sentidos ni límites.
- No se cambiaron mappings, límites ±200, Pedro, Road Runner, dependencias ni controles de aim.

## Estado de hallazgos relacionados

| Finding | Estado tras este cambio | Gate |
|---|---|---|
| FND-001 | `FIX_READY` | Repetir init/stop y ausencia de webcam sin excepción secundaria. |
| FND-002 | `FIX_READY` | Probar cancelaciones <1 s y armado único ≥1 s. |
| FND-014 | `FIX_READY` | Confirmar que `TeleopTorreta` inicia sin el shooter nulo. |
| FND-016 | `FIX_READY` | Validar init-loop y E-stop en el robot. |
| FND-003, FND-020, FND-021 | `BLOCKED_PHYSICAL` | Medidas de torreta y export RC. |
| FND-013 | `OPEN` crítico | `IntakeTeleOp` queda prohibido, pero aún no está contenido por código. |
| FND-015 | `OPEN` crítico | No energizar shooter hasta implementar health fail-closed. |
| FND-007 | `OPEN` parcial | Los OpModes que no heredan del lifecycle seguro aún requieren auditoría. |

MP-01A puede aceptarse después de las pruebas indicadas; MP-01 completo no se cierra mientras FND-013/FND-015 y los gates físicos sigan abiertos.

## Evidencia de software

- `git diff --check`: PASS.
- `assembleDebug`: PASS inicial en 16.8 s; retest final con `KICKER_OUT_SPEED = 0.85`: PASS en 7 s.
- APK del retest final: `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`, 81,301,551 bytes.
- Revisión de alcance: `IntakeTeleOp`, mappings, límites, Pedro y dependencias sin cambios.
- `KICKER_OUT_SPEED = 0.85`: incluido por decisión del equipo; aún no validado físicamente.

## Próxima acción de Codex

1. Recibir y registrar el export RC y los resultados físicos con SHA, repeticiones y tiempos.
2. Corregir cualquier fallo observado sin ampliar mappings ni límites por inferencia.
3. Implementar como siguiente slice de MP-01 el health fail-closed del shooter (FND-015) y contener/deshabilitar `IntakeTeleOp` (FND-013) antes de cualquier prueba de shooter.

## Lo que se espera del usuario en pruebas reales

Antes de habilitar: robot elevado o mecanismos desacoplados, área despejada, cámara cubierta/ausente según la configuración real y acceso inmediato a Stop. Registrar responsable, fecha, SHA exacto, versiones RC/DS y video o cronometraje cuando aplique.

1. Entregar el export real de configuración como `docs/plan-maestro/hardware/robot-controller-config.xml` y completar puertos/tipos del contrato.
2. Ejecutar 20 inicializaciones sin chord: cero armados y cero movimiento.
3. Ejecutar 20 holds menores de 1 s: todos regresan a `WAITING`, sin reset.
4. Ejecutar 20 holds de al menos 1 s: transición a `ARMED` y un solo reset por inicialización.
5. Pulsar Start sin armar en 10 repeticiones: torreta en cero durante toda la ejecución.
6. Probar `gamepad1 BACK` 10 veces durante init y 10 después de Start: actuadores en cero en el siguiente ciclo y objetivo ≤50 ms, con evidencia temporal.
7. Ejecutar 20 ciclos init/stop con cámara disponible sólo si existe una cámara segura y configurada.
8. Repetir sin `Webcam 1`: mensaje legible, ningún movimiento y ningún fallo secundario de cleanup. Si la webcam fue retirada, esta es la variante obligatoria y la prueba “con cámara” queda marcada `BLOCKED_HARDWARE`.
9. No seleccionar `IntakeTeleOp` y no energizar el shooter hasta que FND-013 y FND-015 estén contenidos.

Reportar cada prueba como: `SHA | fecha | operador | configuración | repeticiones | resultado | tiempo máximo | evidencia | observaciones`. Un build exitoso no reemplaza estas pruebas.
