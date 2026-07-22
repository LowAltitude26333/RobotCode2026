# Exportables Limelight de MP-03

Guardar aquí únicamente exportables sanitizados del pipeline y field map usados
en el robot. No incluir contraseñas, direcciones IP ni configuración privada de
red.

Por cada archivo, registrar en
`docs/plan-maestro/mp03-limelight-commissioning.md`:

- nombre y propósito;
- fecha de exportación;
- versión de firmware/app;
- índice de pipeline o field map correspondiente;
- SHA-256.

## Inventario

| Archivo | Propósito | LimelightOS | Pipeline | SHA-256 |
|---|---|---|---|---|
| `pipeline-0-original-2026-07-21.vpr` | Respaldo anterior a corregir marker size | `2026.0` | `0`, `Pipeline_Name`, `AprilTags` | `F5C48FF047E17D6CC02551122E51957218DA37D7E85C7CF24B50E0CDD0100994` |
| `pipeline-0-decode-165.1mm-2026-07-21.vpr` | Pipeline corregido para el cuadrado negro oficial DECODE | `2026.0` | `0`, `Pipeline_Name`, `AprilTags` | `0B954CFA8641B460C13E8DF65A7A4691E7FD409821A09A008BF3ADA141D20693` |

El respaldo original conserva deliberadamente `fiducial_size=101.6` y
`fiducial_skip3d=1`. No representa la configuración aceptada para DECODE; es el
punto de rollback previo al commissioning.

La comparación campo por campo entre ambos exportables arroja un solo cambio:
`fiducial_size`, de `101.6` a `165.1`. El corregido aún conserva
`fiducial_skip3d=1`; por tanto, no habilita ni valida pose 3D.
