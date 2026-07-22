import fs from "node:fs/promises";
import path from "node:path";
import crypto from "node:crypto";
import { FileBlob, SpreadsheetFile, Workbook } from "@oai/artifact-tool";

const COLORS = {
  navy: "#17365D",
  blue: "#2F75B5",
  paleBlue: "#D9EAF7",
  input: "#FFF2CC",
  formula: "#E7E6E6",
  green: "#E2F0D9",
  red: "#F4CCCC",
  amber: "#FCE5CD",
  white: "#FFFFFF",
  text: "#1F2937",
  line: "#CBD5E1",
};

const STATUS_VALUES = [
  "NOT_RUN", "IN_PROGRESS", "PASS", "FAIL", "BLOCKED", "ABORTED",
  "INVALID_DATA", "NOT_APPLICABLE",
];
const YES_NO = ["", "YES", "NO"];
const QUALITY_VALUES = [
  "", "VALID", "NO_TARGET", "WRONG_TAG", "STALE", "PIPELINE_MISMATCH",
  "DEVICE_DISCONNECTED", "INVALID",
];
const MODEL_VALUES = ["", "LINEAR", "QUADRATIC", "PIECEWISE"];
const ROLE_VALUES = ["", "CALIBRATION", "HOLDOUT"];
const OUTCOME_VALUES = ["", "SCORED", "SHORT", "LONG", "REBOUND", "OTHER"];

function title(sheet, text, subtitle, endColumn = "H") {
  sheet.showGridLines = false;
  sheet.getRange(`A1:${endColumn}1`).merge();
  sheet.getRange("A1").values = [[text]];
  sheet.getRange(`A1:${endColumn}1`).format = {
    fill: COLORS.navy,
    font: { bold: true, color: COLORS.white, size: 16 },
    horizontalAlignment: "center",
    verticalAlignment: "center",
  };
  sheet.getRange(`A1:${endColumn}1`).format.rowHeight = 30;
  sheet.getRange(`A2:${endColumn}2`).merge();
  sheet.getRange("A2").values = [[subtitle]];
  sheet.getRange(`A2:${endColumn}2`).format = {
    fill: COLORS.paleBlue,
    font: { italic: true, color: COLORS.text },
    wrapText: true,
    verticalAlignment: "center",
  };
  sheet.getRange(`A2:${endColumn}2`).format.rowHeight = 36;
}

function styleHeader(range) {
  range.format = {
    fill: COLORS.blue,
    font: { bold: true, color: COLORS.white },
    wrapText: true,
    verticalAlignment: "center",
    horizontalAlignment: "center",
    borders: {
      top: { color: COLORS.line, style: "continuous" },
      bottom: { color: COLORS.line, style: "continuous" },
      left: { color: COLORS.line, style: "continuous" },
      right: { color: COLORS.line, style: "continuous" },
    },
  };
  range.format.rowHeight = 32;
}

function styleInput(range) {
  range.format = {
    fill: COLORS.input,
    font: { color: COLORS.text },
    borders: {
      top: { color: COLORS.line, style: "continuous" },
      bottom: { color: COLORS.line, style: "continuous" },
      left: { color: COLORS.line, style: "continuous" },
      right: { color: COLORS.line, style: "continuous" },
    },
  };
}

function styleFormula(range) {
  range.format = {
    fill: COLORS.formula,
    font: { color: COLORS.text },
    borders: {
      top: { color: COLORS.line, style: "continuous" },
      bottom: { color: COLORS.line, style: "continuous" },
      left: { color: COLORS.line, style: "continuous" },
      right: { color: COLORS.line, style: "continuous" },
    },
  };
}

function addStatusFormatting(range) {
  range.dataValidation = { rule: { type: "list", values: STATUS_VALUES } };
  range.conditionalFormats.add("containsText", {
    text: "PASS", format: { fill: COLORS.green, font: { bold: true, color: "#276221" } },
  });
  range.conditionalFormats.add("containsText", {
    text: "FAIL", format: { fill: COLORS.red, font: { bold: true, color: "#9C0006" } },
  });
  range.conditionalFormats.add("containsText", {
    text: "BLOCKED", format: { fill: COLORS.amber, font: { bold: true, color: "#9C5700" } },
  });
  range.conditionalFormats.add("containsText", {
    text: "ABORTED", format: { fill: COLORS.red, font: { bold: true, color: "#9C0006" } },
  });
}

function addTable(sheet, address, name) {
  const table = sheet.tables.add(address, true, name);
  table.style = "TableStyleMedium2";
  table.showFilterButton = true;
  return table;
}

function configureColumns(sheet, widths) {
  for (const [address, width] of Object.entries(widths)) {
    sheet.getRange(address).format.columnWidth = width;
  }
}

function buildInstructions(wb) {
  const s = wb.worksheets.add("INSTRUCCIONES");
  title(s, "PAQUETE 01 — CIERRE MP-03", "Guía offline. No habilita pose, torreta, shooter, feeder, intake ni drivetrain.", "H");
  s.getRange("A4:H4").merge();
  s.getRange("A4").values = [["ORDEN OBLIGATORIO"]];
  styleHeader(s.getRange("A4:H4"));
  const steps = [
    ["1", "Confirmar área despejada, mecanismos sin piezas y Stop accesible."],
    ["2", "Completar SESION y MP03_01_Config antes de INIT."],
    ["3", "Ejecutar cinco ciclos en MP03_02_Lifecycle con el mismo APK."],
    ["4", "Registrar tags 20 y 24 y pose 3D en MP03_03_Tags."],
    ["5", "Ejecutar cada caso de MP03_04_Faults sin mover actuadores."],
    ["6", "Registrar poses estáticas medidas en MP03_05_StaticPose."],
    ["7", "Revisar DASHBOARD. No declarar PASS manualmente."],
    ["8", "Guardar como 01-cierre-mp03_v1_FILLED.xlsx y regresar el archivo sin borrar columnas."],
  ];
  s.getRange(`A5:B${4 + steps.length}`).values = steps;
  styleInput(s.getRange(`A5:B${4 + steps.length}`));
  s.getRange("A14:H14").merge();
  s.getRange("A14").values = [["ABORTAR DE INMEDIATO"]];
  s.getRange("A14:H14").format = { fill: "#C00000", font: { bold: true, color: COLORS.white } };
  const aborts = [
    ["Movimiento de cualquier actuador", "STOP/E-stop, marcar ABORTED y documentar evidencia."],
    ["Humo, olor, calor, ruido o cable suelto", "Cortar energía; no reconectar hasta inspección."],
    ["Excepción/reinicio inesperado", "STOP; registrar mensaje exacto y APK SHA."],
    ["Pipeline, field map o extrínseca distinta", "No continuar con pose 3D; corregir configuración y crear ConfigRevision nueva."],
  ];
  s.getRange("A15:B18").values = aborts;
  styleInput(s.getRange("A15:B18"));
  s.getRange("A20:H20").merge();
  s.getRange("A20").values = [["SHOT_DATASET está preparado para sesiones futuras. MP-03 no autoriza tiros."]];
  s.getRange("A20:H20").format = { fill: COLORS.amber, font: { bold: true, color: "#7F6000" } };
  configureColumns(s, { "A:A": 24, "B:B": 78, "C:H": 12 });
  s.freezePanes.freezeRows(4);
}

function buildSession(wb) {
  const s = wb.worksheets.add("SESION");
  title(s, "IDENTIDAD DE SESIÓN", "Completar antes de abrir el OpMode. Amarillo = entrada.", "F");
  const rows = [
    ["SchemaVersion", "1.0", "Fijo", ""],
    ["TemplateVersion", "1", "Fijo", ""],
    ["PacketId", "01-cierre-mp03", "Fijo", ""],
    ["PacketVersion", "1", "Fijo", ""],
    ["SessionId", "", "Ej. MP03-20260722-A", "REQUIRED"],
    ["Fecha local", "", "YYYY-MM-DD", "REQUIRED"],
    ["Hora inicio", "", "HH:MM", "REQUIRED"],
    ["Ubicación", "", "", "REQUIRED"],
    ["Operador", "", "", "REQUIRED"],
    ["Test lead", "", "", "REQUIRED"],
    ["Safety observer", "", "", "REQUIRED"],
    ["Data recorder", "", "", "REQUIRED"],
    ["Git commit", "", "SHA completo o abreviado inequívoco", "REQUIRED"],
    ["APK SHA-256", "", "64 hex", "REQUIRED"],
    ["ConfigRevision", "R1", "Incrementar tras cambio material", "REQUIRED"],
    ["Robot config", "", "Nombre/revisión RC", "REQUIRED"],
    ["Batería ID", "", "", "REQUIRED"],
    ["Voltaje inicial (V)", "", "Número", "REQUIRED"],
    ["Pipeline SHA-256", "0B954CFA8641B460C13E8DF65A7A4691E7FD409821A09A008BF3ADA141D20693", "Corregido 165.1 mm; actualizar si cambia", "REQUIRED"],
    ["Field map SHA-256", "", "Pendiente", "REQUIRED"],
    ["Condición de luz", "", "", ""],
    ["Notas", "", "", ""],
  ];
  s.getRange("A4:D4").values = [["Campo", "Valor", "Guía", "Obligatorio"]];
  styleHeader(s.getRange("A4:D4"));
  s.getRange(`A5:D${4 + rows.length}`).values = rows;
  styleInput(s.getRange(`B5:B${4 + rows.length}`));
  styleFormula(s.getRange(`A5:A${4 + rows.length}`));
  s.getRange(`C5:D${4 + rows.length}`).format.wrapText = true;
  addTable(s, `A4:D${4 + rows.length}`, "SessionTable");
  configureColumns(s, { "A:A": 26, "B:B": 70, "C:C": 48, "D:D": 18 });
  s.freezePanes.freezeRows(4);
}

function buildChanges(wb) {
  const s = wb.worksheets.add("CAMBIOS");
  title(s, "REGISTRO DE CAMBIOS", "No mezclar intentos de distintas ConfigRevision.", "I");
  const headers = ["ChangeId", "TimestampLocal", "FromRevision", "ToRevision", "Category", "Before", "After", "AppliesFromAttemptId", "Reason"];
  s.getRange("A4:I4").values = [headers];
  styleHeader(s.getRange("A4:I4"));
  s.getRange("A5:I34").values = Array.from({ length: 30 }, (_, i) => [`CHG-${String(i + 1).padStart(2, "0")}`, "", "", "", "", "", "", "", ""]);
  styleInput(s.getRange("B5:I34"));
  styleFormula(s.getRange("A5:A34"));
  s.getRange("E5:E34").dataValidation = { rule: { type: "list", values: ["", "APK", "PIPELINE", "FIELD_MAP", "EXTRINSIC", "HARDWARE", "PROCEDURE", "OTHER"] } };
  addTable(s, "A4:I34", "ChangesTable");
  configureColumns(s, { "A:A": 14, "B:B": 20, "C:D": 15, "E:E": 16, "F:G": 28, "H:H": 23, "I:I": 40 });
  s.freezePanes.freezeRows(4);
}

function buildConfig(wb) {
  const s = wb.worksheets.add("CONFIG");
  title(s, "CONSTANTES DEL PAQUETE", "Editar sólo mediante una nueva TemplateVersion o ConfigRevision documentada.", "F");
  const rows = [
    ["LIMELIGHT_MAX_STALENESS_MS", 200, "ms", "LowAltitudeConstants.VisionConstants", "Gate visual"],
    ["EXPECTED_PIPELINE", 0, "index", "LowAltitudeConstants.VisionConstants", "Pipeline AprilTag"],
    ["BLUE_GOAL_TAG_ID", 20, "id", "LowAltitudeConstants.TurretConstants", "Goal azul"],
    ["RED_GOAL_TAG_ID", 24, "id", "LowAltitudeConstants.TurretConstants", "Goal rojo"],
    ["MARKER_BLACK_SIZE_MM", 165.1, "mm", "mp03-limelight-commissioning.md", "Cuadrado negro oficial"],
    ["EXPECTED_LIFECYCLE_CYCLES", 5, "count", "Gate MP-03", "Mismo APK"],
    ["SHOOTER_READY_ERROR_RPM", 100, "RPM", "T8.1", "Dataset futuro"],
    ["SHOOTER_READY_DWELL_MS", 250, "ms", "T8.1", "Dataset futuro"],
  ];
  s.getRange("A4:E4").values = [["Key", "Value", "Unit", "Source", "Purpose"]];
  styleHeader(s.getRange("A4:E4"));
  s.getRange(`A5:E${4 + rows.length}`).values = rows;
  styleFormula(s.getRange(`A5:E${4 + rows.length}`));
  addTable(s, `A4:E${4 + rows.length}`, "ConfigTable");
  configureColumns(s, { "A:A": 34, "B:B": 15, "C:C": 12, "D:D": 42, "E:E": 35 });
}

function buildCatalogs(wb) {
  const s = wb.worksheets.add("CATALOGOS");
  title(s, "CATÁLOGOS", "Valores válidos para dropdowns y exports.", "J");
  const groups = [
    ["Status", STATUS_VALUES], ["YesNo", ["YES", "NO"]], ["Quality", QUALITY_VALUES.slice(1)],
    ["ModelKind", MODEL_VALUES.slice(1)], ["Role", ROLE_VALUES.slice(1)], ["Outcome", OUTCOME_VALUES.slice(1)],
  ];
  let col = 0;
  for (const [name, values] of groups) {
    const header = s.getCell(3, col);
    header.values = [[name]];
    header.format = { fill: COLORS.blue, font: { bold: true, color: COLORS.white } };
    s.getRangeByIndexes(4, col, values.length, 1).values = values.map(v => [v]);
    s.getRangeByIndexes(4, col, values.length, 1).format = { fill: COLORS.formula };
    col += 2;
  }
  configureColumns(s, { "A:J": 22 });
}

function buildConfigChecklist(wb) {
  const s = wb.worksheets.add("MP03_01_Config");
  title(s, "MP03.1 — CONFIGURACIÓN", "Completar antes de los ciclos lifecycle.", "H");
  const items = [
    ["CFG-01", "Mapping RC", "limelight", "limelight", "", "", "", "Mapping exacto"],
    ["CFG-02", "Firmware/UI", "2026.0", "2026.0", "", "", "", "Registrar captura"],
    ["CFG-03", "Pipeline index", "0", "0", "", "", "", "AprilTags"],
    ["CFG-04", "Resolución/FPS", "640x480 / 90", "640x480 / 90", "", "", "", ""],
    ["CFG-05", "Tag family", "AprilTag Classic 36h11", "AprilTag Classic 36h11", "", "", "", ""],
    ["CFG-06", "Marker black size", "165.1 mm", "165.1 mm", "", "", "", "No validar escala con tag 160 mm"],
    ["CFG-07", "fiducial_skip3d", "0", "", "", "", "", "Debe habilitar pose 3D"],
    ["CFG-08", "Field map export/hash", "Presente + SHA-256", "", "", "", "", "Pendiente"],
    ["CFG-09", "Extrínseca activa", "30 deg, yaw=0, roll=0", "", "", "", "", "Usar geometria-robot-mp04.md"],
    ["CFG-10", "Montaje rígido", "YES", "", "", "", "", ""],
    ["CFG-11", "Oclusión en posiciones de prueba", "NO", "", "", "", "", ""],
    ["CFG-12", "Tag azul 20 disponible", "YES", "", "", "", "", ""],
    ["CFG-13", "Tag rojo 24 disponible", "YES", "", "", "", "", ""],
    ["CFG-14", "Sin consumidores de actuadores", "YES", "", "", "", "", "Revisión humana"],
  ];
  const headers = ["TestId", "Item", "Expected", "Observed", "EvidenceRef", "Status", "AbortReason", "Notes"];
  s.getRange("A4:H4").values = [headers];
  styleHeader(s.getRange("A4:H4"));
  s.getRange(`A5:H${4 + items.length}`).values = items;
  styleInput(s.getRange(`D5:H${4 + items.length}`));
  styleFormula(s.getRange(`A5:C${4 + items.length}`));
  addStatusFormatting(s.getRange(`F5:F${4 + items.length}`));
  addTable(s, `A4:H${4 + items.length}`, "Mp03ConfigTable");
  configureColumns(s, { "A:A": 13, "B:B": 34, "C:D": 31, "E:E": 30, "F:F": 18, "G:H": 36 });
  s.freezePanes.freezeRows(4);
}

function buildLifecycle(wb) {
  const s = wb.worksheets.add("MP03_02_Lifecycle");
  title(s, "MP03.2 — CINCO CICLOS", "Mismo APK. Cualquier movimiento o excepción aborta.", "N");
  const headers = ["TestId", "Cycle", "ConfigRevision", "Init", "Start", "Stop", "Connected", "PipelineObserved", "Exception", "ResourceOpenAfterStop", "AnyMovement", "EvidenceRef", "Status", "Notes"];
  s.getRange("A4:N4").values = [headers];
  styleHeader(s.getRange("A4:N4"));
  const rows = Array.from({ length: 5 }, (_, i) => [`LIFE-${i + 1}`, i + 1, "R1", "", "", "", "", "", "", "", "", "", "NOT_RUN", ""]);
  s.getRange("A5:N9").values = rows;
  styleFormula(s.getRange("A5:C9"));
  styleInput(s.getRange("D5:N9"));
  for (const col of ["D", "E", "F", "G", "I", "J", "K"]) s.getRange(`${col}5:${col}9`).dataValidation = { rule: { type: "list", values: YES_NO } };
  s.getRange("H5:H9").dataValidation = { rule: { type: "whole", operator: "between", formula1: 0, formula2: 20 } };
  s.getRange("M5").formulasR1C1 = [["=IF(COUNTA(RC[-9]:RC[-2])=0,\"NOT_RUN\",IF(OR(RC[-2]=\"YES\",RC[-4]=\"YES\",RC[-3]=\"YES\"),\"FAIL\",IF(AND(RC[-9]=\"YES\",RC[-8]=\"YES\",RC[-7]=\"YES\",RC[-6]=\"YES\",RC[-5]=0,RC[-4]=\"NO\",RC[-3]=\"NO\",RC[-2]=\"NO\"),\"PASS\",\"BLOCKED\")))"]];
  s.getRange("M5:M9").fillDown();
  styleFormula(s.getRange("M5:M9"));
  addStatusFormatting(s.getRange("M5:M9"));
  addTable(s, "A4:N9", "LifecycleTable");
  configureColumns(s, { "A:A": 13, "B:B": 9, "C:C": 15, "D:G": 12, "H:H": 18, "I:K": 22, "L:L": 28, "M:M": 18, "N:N": 35 });
  s.freezePanes.freezeRows(4);
}

function buildTags(wb) {
  const s = wb.worksheets.add("MP03_03_Tags");
  title(s, "MP03.3 — TAGS Y POSE 3D", "Una fila por observación nueva; no promediar a mano.", "V");
  const headers = ["AttemptId", "ConfigRevision", "TagId", "Alliance", "ExpectedTag", "TxDeg", "TyDeg", "AreaPct", "StalenessMs", "TargetLatencyMs", "CaptureLatencyMs", "TotalLatencyMs", "TagCount", "BotPoseAvailable", "BotX", "BotY", "BotZ", "BotYawDeg", "Finite", "Quality", "RejectionReason", "EvidenceRef"];
  s.getRange("A4:V4").values = [headers];
  styleHeader(s.getRange("A4:V4"));
  const rows = Array.from({ length: 40 }, (_, i) => [`TAG-${String(i + 1).padStart(2, "0")}`, "R1", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""]);
  s.getRange("A5:V44").values = rows;
  styleFormula(s.getRange("A5:B44"));
  styleInput(s.getRange("C5:V44"));
  s.getRange("C5:C44").dataValidation = { rule: { type: "whole", operator: "between", formula1: 0, formula2: 586 } };
  s.getRange("D5:D44").dataValidation = { rule: { type: "list", values: ["", "BLUE", "RED"] } };
  for (const col of ["E", "N", "S"]) s.getRange(`${col}5:${col}44`).dataValidation = { rule: { type: "list", values: YES_NO } };
  s.getRange("T5:T44").dataValidation = { rule: { type: "list", values: QUALITY_VALUES } };
  addTable(s, "A4:V44", "TagObservationsTable");
  configureColumns(s, { "A:A": 14, "B:B": 15, "C:E": 13, "F:M": 17, "N:N": 20, "O:R": 16, "S:T": 16, "U:V": 34 });
  s.freezePanes.freezeRows(4);
}

function buildFaults(wb) {
  const s = wb.worksheets.add("MP03_04_Faults");
  title(s, "MP03.4 — FALLOS Y RECUPERACIÓN", "Todos los casos deben mantener actuadores inmóviles.", "L");
  const cases = [
    ["FLT-01", "Camera covered", "NO_TARGET or STALE"],
    ["FLT-02", "Wrong tag", "WRONG_TAG"],
    ["FLT-03", "Invalid result", "INVALID"],
    ["FLT-04", "Wrong pipeline", "PIPELINE_MISMATCH"],
    ["FLT-05", "Disconnect", "DEVICE_DISCONNECTED"],
    ["FLT-06", "Reconnect", "Telemetry only recovery"],
    ["FLT-07", "Stale frame", "STALE"],
    ["FLT-08", "No 3D pose", "Rejected / no actionable pose"],
  ];
  const headers = ["TestId", "Case", "ExpectedQuality", "ObservedQuality", "ActionableValuesZero", "AnyMovement", "Exception", "Recovered", "EvidenceRef", "Status", "AbortReason", "Notes"];
  s.getRange("A4:L4").values = [headers];
  styleHeader(s.getRange("A4:L4"));
  s.getRange("A5:L12").values = cases.map(c => [...c, "", "", "", "", "", "", "NOT_RUN", "", ""]);
  styleFormula(s.getRange("A5:C12"));
  styleInput(s.getRange("D5:L12"));
  for (const col of ["E", "F", "G", "H"]) s.getRange(`${col}5:${col}12`).dataValidation = { rule: { type: "list", values: YES_NO } };
  s.getRange("J5").formulasR1C1 = [["=IF(COUNTA(RC[-6]:RC[-1])=0,\"NOT_RUN\",IF(OR(RC[-5]<>\"YES\",RC[-4]<>\"NO\",RC[-3]<>\"NO\"),\"FAIL\",IF(AND(RC[-6]<>\"\",RC[-2]=\"YES\",RC[-1]<>\"\"),\"PASS\",\"BLOCKED\")))"]];
  s.getRange("J5:J12").fillDown();
  styleFormula(s.getRange("J5:J12"));
  addStatusFormatting(s.getRange("J5:J12"));
  addTable(s, "A4:L12", "FaultsTable");
  configureColumns(s, { "A:A": 13, "B:B": 24, "C:D": 28, "E:H": 22, "I:I": 28, "J:J": 18, "K:L": 35 });
  s.freezePanes.freezeRows(4);
}

function buildStaticPose(wb) {
  const s = wb.worksheets.add("MP03_05_StaticPose");
  title(s, "MP03.5 — POSE ESTÁTICA DIAGNÓSTICA", "No aplica corrección. Registrar unidades y marco exactamente.", "Q");
  const headers = ["AttemptId", "ConfigRevision", "TagId", "PhysicalXIn", "PhysicalYIn", "PhysicalHeadingDeg", "LimelightXIn", "LimelightYIn", "LimelightHeadingDeg", "ResidualXIn", "ResidualYIn", "ResidualDistanceIn", "ResidualHeadingDeg", "StalenessMs", "EvidenceRef", "Status", "Notes"];
  s.getRange("A4:Q4").values = [headers];
  styleHeader(s.getRange("A4:Q4"));
  const rows = Array.from({ length: 20 }, (_, i) => [`POSE-${String(i + 1).padStart(2, "0")}`, "R1", "", "", "", "", "", "", "", "", "", "", "", "", "", "NOT_RUN", ""]);
  s.getRange("A5:Q24").values = rows;
  styleFormula(s.getRange("A5:B24"));
  styleInput(s.getRange("C5:I24"));
  styleFormula(s.getRange("J5:M24"));
  styleInput(s.getRange("N5:Q24"));
  s.getRange("J5").formulasR1C1 = [["=IF(OR(RC[-6]=\"\",RC[-3]=\"\"),\"\",RC[-3]-RC[-6])"]];
  s.getRange("J5:J24").fillDown();
  s.getRange("K5").formulasR1C1 = [["=IF(OR(RC[-6]=\"\",RC[-3]=\"\"),\"\",RC[-3]-RC[-6])"]];
  s.getRange("K5:K24").fillDown();
  s.getRange("L5").formulasR1C1 = [["=IF(OR(RC[-2]=\"\",RC[-1]=\"\"),\"\",SQRT(RC[-2]^2+RC[-1]^2))"]];
  s.getRange("L5:L24").fillDown();
  s.getRange("M5").formulasR1C1 = [["=IF(OR(RC[-7]=\"\",RC[-4]=\"\"),\"\",MOD(RC[-4]-RC[-7]+180,360)-180)"]];
  s.getRange("M5:M24").fillDown();
  addStatusFormatting(s.getRange("P5:P24"));
  addTable(s, "A4:Q24", "StaticPoseTable");
  configureColumns(s, { "A:A": 14, "B:B": 15, "C:C": 11, "D:M": 18, "N:N": 16, "O:O": 28, "P:P": 18, "Q:Q": 35 });
  s.freezePanes.freezeRows(4);
}

function buildShotDataset(wb) {
  const s = wb.worksheets.add("SHOT_DATASET");
  title(s, "DATASET DE TIRO — EXTRA RPM/DISTANCIA", "Preparado para sesiones posteriores. No autoriza tiros durante MP-03.", "V");
  const headers = ["sessionId", "distanceGroupId", "modelKind", "role", "distanceInches", "targetRpm", "measuredRpmAtFeed", "rpmReadyHoldMs", "outcome", "AttemptId", "ConfigRevision", "Position", "BatteryId", "FusedXIn", "FusedYIn", "FusedHeadingDeg", "TurretErrorDeg", "ArtifactBatch", "EvidenceRef", "Notes", "RpmError", "ReadyGate"];
  s.getRange("A4:V4").values = [headers];
  styleHeader(s.getRange("A4:V4"));
  const rows = Array.from({ length: 60 }, (_, i) => ["", "", "", "", "", "", "", "", "", `SHOT-${String(i + 1).padStart(3, "0")}`, "R1", "", "", "", "", "", "", "", "", "", "", ""]);
  s.getRange("A5:V64").values = rows;
  styleInput(s.getRange("A5:T64"));
  styleFormula(s.getRange("J5:K64"));
  styleFormula(s.getRange("U5:V64"));
  s.getRange("C5:C64").dataValidation = { rule: { type: "list", values: MODEL_VALUES } };
  s.getRange("D5:D64").dataValidation = { rule: { type: "list", values: ROLE_VALUES } };
  s.getRange("I5:I64").dataValidation = { rule: { type: "list", values: OUTCOME_VALUES } };
  s.getRange("U5").formulasR1C1 = [["=IF(OR(RC[-15]=\"\",RC[-14]=\"\"),\"\",ABS(RC[-14]-RC[-15]))"]];
  s.getRange("U5:U64").fillDown();
  s.getRange("V5").formulasR1C1 = [["=IF(RC[-21]=\"\",\"\",IF(AND(RC[-1]<=CONFIG!R11C2,RC[-14]>=CONFIG!R12C2),\"PASS\",\"FAIL\"))"]];
  s.getRange("V5:V64").fillDown();
  addStatusFormatting(s.getRange("V5:V64"));
  addTable(s, "A4:V64", "ShotDatasetTable");
  configureColumns(s, { "A:B": 19, "C:D": 16, "E:H": 19, "I:I": 14, "J:K": 17, "L:M": 18, "N:Q": 18, "R:T": 28, "U:V": 16 });
  s.freezePanes.freezeRows(4);
}

function buildShotExport(wb) {
  const s = wb.worksheets.add("EXPORT_SHOT_DATASET");
  title(s, "EXPORT SHOTDATASETCSV", "Exportar esta tabla como CSV; la primera celda inicia con # para que el parser ignore el header.", "I");
  const headers = ["#sessionId", "distanceGroupId", "modelKind", "role", "distanceInches", "targetRpm", "measuredRpmAtFeed", "rpmReadyHoldMs", "outcome"];
  s.getRange("A4:I4").values = [headers];
  styleHeader(s.getRange("A4:I4"));
  for (let row = 5; row <= 64; row++) {
    const sourceRow = row;
    const formulas = [];
    for (let col = 0; col < 9; col++) {
      const sourceCol = String.fromCharCode(65 + col);
      formulas.push(`=IF(SHOT_DATASET!A${sourceRow}=\"\",\"\",SHOT_DATASET!${sourceCol}${sourceRow})`);
    }
    s.getRange(`A${row}:I${row}`).formulas = [formulas];
  }
  styleFormula(s.getRange("A5:I64"));
  addTable(s, "A4:I64", "ShotDatasetExportTable");
  configureColumns(s, { "A:B": 22, "C:D": 17, "E:H": 20, "I:I": 15 });
  s.freezePanes.freezeRows(4);
}

function buildDashboard(wb) {
  const s = wb.worksheets.add("DASHBOARD");
  title(s, "DASHBOARD MP-03", "Resumen automático. No editar las fórmulas.", "J");
  s.getRange("A4:C4").values = [["Gate", "Status", "Criterio"]];
  styleHeader(s.getRange("A4:C4"));
  const gates = [
    ["Configuración", "", "14/14 items PASS"],
    ["Lifecycle", "", "5/5 ciclos PASS"],
    ["Tags", "", "IDs 20 y 24 con pose 3D válida/fresca"],
    ["Faults", "", "8/8 casos PASS"],
    ["MP-03", "", "Todos los gates anteriores PASS"],
  ];
  s.getRange("A5:C9").values = gates;
  styleFormula(s.getRange("A5:C9"));
  s.getRange("B5").formulas = [["=IF(COUNTIF(MP03_01_Config!F5:F18,\"PASS\")=0,\"NOT_RUN\",IF(COUNTIF(MP03_01_Config!F5:F18,\"FAIL\")+COUNTIF(MP03_01_Config!F5:F18,\"ABORTED\")>0,\"FAIL\",IF(COUNTIF(MP03_01_Config!F5:F18,\"PASS\")=14,\"PASS\",\"BLOCKED\")))"]];
  s.getRange("B6").formulas = [["=IF(COUNTIF(MP03_02_Lifecycle!M5:M9,\"PASS\")=0,\"NOT_RUN\",IF(COUNTIF(MP03_02_Lifecycle!M5:M9,\"FAIL\")+COUNTIF(MP03_02_Lifecycle!M5:M9,\"ABORTED\")>0,\"FAIL\",IF(COUNTIF(MP03_02_Lifecycle!M5:M9,\"PASS\")=5,\"PASS\",\"BLOCKED\")))"]];
  s.getRange("B7").formulas = [["=IF(COUNTA(MP03_03_Tags!C5:C44)=0,\"NOT_RUN\",IF(AND(COUNTIFS(MP03_03_Tags!C5:C44,20,MP03_03_Tags!E5:E44,\"YES\",MP03_03_Tags!I5:I44,\"<=\"&CONFIG!B5,MP03_03_Tags!N5:N44,\"YES\",MP03_03_Tags!S5:S44,\"YES\",MP03_03_Tags!T5:T44,\"VALID\")>0,COUNTIFS(MP03_03_Tags!C5:C44,24,MP03_03_Tags!E5:E44,\"YES\",MP03_03_Tags!I5:I44,\"<=\"&CONFIG!B5,MP03_03_Tags!N5:N44,\"YES\",MP03_03_Tags!S5:S44,\"YES\",MP03_03_Tags!T5:T44,\"VALID\")>0),\"PASS\",\"BLOCKED\"))"]];
  s.getRange("B8").formulas = [["=IF(COUNTIF(MP03_04_Faults!J5:J12,\"PASS\")=0,\"NOT_RUN\",IF(COUNTIF(MP03_04_Faults!J5:J12,\"FAIL\")+COUNTIF(MP03_04_Faults!J5:J12,\"ABORTED\")>0,\"FAIL\",IF(COUNTIF(MP03_04_Faults!J5:J12,\"PASS\")=8,\"PASS\",\"BLOCKED\")))"]];
  s.getRange("B9").formulas = [["=IF(COUNTIF(B5:B8,\"PASS\")=0,\"NOT_RUN\",IF(COUNTIF(B5:B8,\"FAIL\")>0,\"FAIL\",IF(COUNTIF(B5:B8,\"PASS\")=4,\"PASS\",\"BLOCKED\")))"]];
  addStatusFormatting(s.getRange("B5:B9"));
  s.getRange("E4:G4").values = [["Dataset extra", "Value", "Meaning"]];
  styleHeader(s.getRange("E4:G4"));
  s.getRange("E5:G8").values = [["Rows captured", "", "Tiros con sessionId"], ["Rows ready", "", "RPM gate PASS"], ["Scored", "", "Outcome SCORED"], ["Model status", "EXTRA / NOT BLOCKING", "No afecta MP-03"]];
  s.getRange("F5").formulas = [["=COUNTIF(SHOT_DATASET!A5:A64,\"<>\")"]];
  s.getRange("F6").formulas = [["=COUNTIF(SHOT_DATASET!V5:V64,\"PASS\")"]];
  s.getRange("F7").formulas = [["=COUNTIF(SHOT_DATASET!I5:I64,\"SCORED\")"]];
  styleFormula(s.getRange("E5:G8"));
  s.getRange("A11:J11").merge();
  s.getRange("A11").values = [["MP-03 PASS sólo autoriza abrir MP-04. Nunca autoriza por sí mismo movimiento o disparo."]];
  s.getRange("A11:J11").format = { fill: COLORS.amber, font: { bold: true, color: "#7F6000" }, wrapText: true };
  configureColumns(s, { "A:A": 24, "B:B": 18, "C:C": 48, "D:D": 5, "E:E": 22, "F:F": 22, "G:G": 38, "H:J": 12 });
}

function buildQa(wb) {
  const s = wb.worksheets.add("QA");
  title(s, "TRIPLE QA DEL TEMPLATE", "La evidencia externa incluye previews, reporte JSON y SHA-256 sidecar.", "H");
  const rows = [
    ["QA1-01", "SOURCE", "AGENTS.md y docs MP-03 leídos", "PASS", "Builder manifest"],
    ["QA1-02", "SOURCE", "IDs/unidades/thresholds cotejados", "PASS", "CONFIG"],
    ["QA1-03", "SOURCE", "Dataset coincide con ShotDatasetCsv", "PASS", "EXPORT_SHOT_DATASET"],
    ["QA2-01", "FORMULA", "Libro vacío no declara MP-03 PASS", "PENDING_RUNTIME", "Automated fixture"],
    ["QA2-02", "FORMULA", "PASS/FAIL/BLOCKED boundaries", "PENDING_RUNTIME", "Automated fixture"],
    ["QA2-03", "FORMULA", "Sin errores de fórmula", "PENDING_RUNTIME", "Workbook inspect"],
    ["QA2-04", "SCHEMA", "Exports y tablas conservan columnas", "PENDING_RUNTIME", "Round-trip inspect"],
    ["QA3-01", "ROUND_TRIP", "Export/reimport exitoso", "PENDING_RUNTIME", "Artifact tool"],
    ["QA3-02", "VISUAL", "Todas las hojas renderizadas", "PENDING_RUNTIME", "Preview folder"],
    ["QA3-03", "VISUAL", "Sin clipping severo ni celdas ocultas", "PENDING_HUMAN", "Visual review"],
    ["QA3-04", "HASH", "SHA-256 final guardado", "PENDING_RUNTIME", ".sha256 sidecar"],
  ];
  s.getRange("A4:E4").values = [["CheckId", "Layer", "Check", "Status", "Evidence"]];
  styleHeader(s.getRange("A4:E4"));
  s.getRange(`A5:E${4 + rows.length}`).values = rows;
  styleFormula(s.getRange(`A5:E${4 + rows.length}`));
  addTable(s, `A4:E${4 + rows.length}`, "QaTable");
  configureColumns(s, { "A:A": 14, "B:B": 16, "C:C": 54, "D:D": 22, "E:E": 34 });
}

function assertSourceSchema(wb) {
  const expected = [
    "INSTRUCCIONES", "SESION", "CAMBIOS", "CONFIG", "CATALOGOS",
    "MP03_01_Config", "MP03_02_Lifecycle", "MP03_03_Tags",
    "MP03_04_Faults", "MP03_05_StaticPose", "SHOT_DATASET",
    "EXPORT_SHOT_DATASET", "DASHBOARD", "QA",
  ];
  const actual = wb.worksheets.items.map(s => s.name);
  for (const name of expected) {
    if (!actual.includes(name)) throw new Error(`QA1 missing sheet: ${name}`);
  }
  const exportHeaders = wb.worksheets.getItem("EXPORT_SHOT_DATASET").getRange("A4:I4").values[0];
  const wanted = ["#sessionId", "distanceGroupId", "modelKind", "role", "distanceInches", "targetRpm", "measuredRpmAtFeed", "rpmReadyHoldMs", "outcome"];
  if (JSON.stringify(exportHeaders) !== JSON.stringify(wanted)) {
    throw new Error(`QA1 ShotDataset schema mismatch: ${JSON.stringify(exportHeaders)}`);
  }
  return { expectedSheets: expected.length, actualSheets: actual.length, shotDatasetColumns: wanted.length };
}

async function sha256(filePath) {
  const bytes = await fs.readFile(filePath);
  return crypto.createHash("sha256").update(bytes).digest("hex").toUpperCase();
}

export async function buildMp03Workbook(outputDir) {
  await fs.mkdir(outputDir, { recursive: true });
  const previewDir = path.join(outputDir, "01-cierre-mp03_v1-previews");
  await fs.mkdir(previewDir, { recursive: true });
  const wb = Workbook.create();
  buildInstructions(wb);
  buildSession(wb);
  buildChanges(wb);
  buildConfig(wb);
  buildCatalogs(wb);
  buildConfigChecklist(wb);
  buildLifecycle(wb);
  buildTags(wb);
  buildFaults(wb);
  buildStaticPose(wb);
  buildShotDataset(wb);
  buildShotExport(wb);
  buildDashboard(wb);
  buildQa(wb);

  const qa1 = assertSourceSchema(wb);
  const outputPath = path.join(outputDir, "01-cierre-mp03_v1.xlsx");
  const exported = await SpreadsheetFile.exportXlsx(wb);
  await exported.save(outputPath);

  const imported = await SpreadsheetFile.importXlsx(await FileBlob.load(outputPath));
  const sheetSummary = await imported.inspect({ kind: "sheet", include: "id,name", maxChars: 12000 });
  const formulaErrors = await imported.inspect({
    kind: "match",
    searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A",
    options: { useRegex: true, maxResults: 300 },
    summary: "formula error scan",
  });
  const dashboard = await imported.inspect({
    kind: "table", range: "DASHBOARD!A1:G12", include: "values,formulas",
    tableMaxRows: 15, tableMaxCols: 8,
  });
  const exportCheck = await imported.inspect({
    kind: "table", range: "EXPORT_SHOT_DATASET!A4:I8", include: "values,formulas",
    tableMaxRows: 8, tableMaxCols: 10,
  });

  const rendered = [];
  for (const sheet of imported.worksheets.items) {
    const preview = await imported.render({ sheetName: sheet.name, autoCrop: "all", scale: 1, format: "png" });
    const previewPath = path.join(previewDir, `${sheet.name}.png`);
    await fs.writeFile(previewPath, new Uint8Array(await preview.arrayBuffer()));
    rendered.push(previewPath);
  }

  const hash = await sha256(outputPath);
  await fs.writeFile(`${outputPath}.sha256`, `${hash}  01-cierre-mp03_v1.xlsx\n`, "utf8");
  const report = {
    createdAt: new Date().toISOString(),
    workbook: outputPath,
    sha256: hash,
    qa1,
    roundTripSheetSummary: sheetSummary.ndjson,
    formulaErrorScan: formulaErrors.ndjson,
    dashboardInspect: dashboard.ndjson,
    exportInspect: exportCheck.ndjson,
    renderedSheets: rendered,
  };
  await fs.writeFile(path.join(outputDir, "01-cierre-mp03_v1-qa.json"), JSON.stringify(report, null, 2), "utf8");
  return report;
}
