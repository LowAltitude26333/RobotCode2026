# Handoff MP-01 — seguridad y mecanismos

Fecha: 2026-07-21

Estado: **ACCEPTED**

Rama: `masterplan`

Base de esta consolidación: `fed3424`
Commit de implementación principal: `d139aa6` (`feat(safety): finalize mechanism commissioning gates`)

## Alcance cerrado

MP-01 dejó inicialización, ownership y apagado de mecanismos en un estado apto para continuar el plan. La configuración final confirmada usa un shooter, kicker por motor sin CRServo, intake y torreta con cero manual y límites `-983/+1070` ticks. La torreta recorre a `0.50` y reduce a `0.05` dentro de los últimos 100 ticks.

El `SystemCheckOpMode` conserva controles sostenidos, watchdogs, timeouts, fault latch y salidas en cero ante release, interrupción, Stop y BACK E-stop. El OpMode `PRUEBA: MECANISMOS SIN SERVO` se conserva como herramienta de diagnóstico; no sustituye los interlocks del TeleOp de competencia.

## Evidencia de aceptación

- Spot-check final MP-01: `5/5 PASS` — INIT quieto, torreta sin armar inmóvil, kicker bloqueado con shooter apagado, release cero y BACK E-stop.
- FND-003, FND-020, FND-026, FND-027 y FND-028 cerrados con evidencia física registrada en [handoff-task.md](handoff-task.md).
- El feeder completo con pulso/cooldown y el readiness basado en pose/velocidad no pertenecen a este cierre; continúan en MP-06 bajo FND-018/FND-019.

APK físicamente aceptado para el spot-check final MP-01: SHA-256 `7A9060615BFD6FB0C31E5E5C2D9B68E2E41EE228B99CAD385ABBF00B349CF5AC`.

## Guardrails y reapertura

- No reinstalar ni habilitar un CRServo sin tratarlo como cambio nuevo y reabrir el gate mecánico.
- No ampliar límites, potencia o tiempo de torreta sin retest bilateral de corte, cables y retorno.
- No alimentar piezas desde los OpModes de commissioning.
- Cambios a mappings, ownership o rutas de stop requieren repetir el spot-check afectado; no trasladar PASS entre APK distintos automáticamente.

## Referencias

- [Bitácora cronológica](handoff-task.md)
- [Contrato de hardware](contrato-hardware.md)
- [Hallazgos](hallazgos.md)
- [Decisiones](decisiones.md)

## Rollback

El punto previo a esta consolidación es `fed3424`. El rollback debe hacerse mediante Git sobre una rama/worktree limpio; nunca mediante reset destructivo de un worktree con cambios del equipo.
