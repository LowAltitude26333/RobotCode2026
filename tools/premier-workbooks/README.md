# Premier workbook builders

`build-mp03-workbook.mjs` genera el paquete descargable de la primera sesión:

- `01-cierre-mp03_v1.xlsx`
- previews PNG de todas las hojas;
- reporte de QA round-trip;
- sidecar SHA-256.

El builder usa exclusivamente `@oai/artifact-tool` mediante el runtime de
dependencias del workspace. No instalar dependencias en este repositorio ni ejecutar
el builder con un Node global.

La función exportada es:

```js
await buildMp03Workbook("<output-directory>")
```

Si el runtime de `@oai/artifact-tool` no está disponible, el equipo autorizó un
fallback reproducible con Excel de escritorio:

```powershell
.\tools\premier-workbooks\build-mp03-workbook.ps1 `
  -OutputDirectory .\outputs\plan-final-antespremier
```

El fallback ejecuta fixtures, reabre el XLSX, escanea errores, exporta cada hoja a
PNG, genera un PDF y escribe un sidecar SHA-256. Después de la revisión humana de
los previews se repite con `-VisualQaPassed` para producir el archivo final.

El XLSX no debe entregarse hasta completar las tres capas descritas en
`docs/plan-maestro/plan-final-antespremier.md`: fuente de verdad, fórmulas/estructura
y round-trip/visual.

Los datos recibidos del equipo son inmutables. Un archivo `_FILLED.xlsx` nunca se
abre para sobrescribirlo; se calcula su hash y se analiza desde una copia o un
consolidado separado.
