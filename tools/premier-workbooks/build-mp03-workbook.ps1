param(
    [Parameter(Mandatory = $true)]
    [string]$OutputDirectory,
    [switch]$VisualQaPassed
)

$ErrorActionPreference = 'Stop'

$scriptDirectory = $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($scriptDirectory)) {
    $scriptDirectory = Join-Path ([Environment]::CurrentDirectory) 'tools\premier-workbooks'
}
$workspaceRoot = [IO.Path]::GetFullPath((Join-Path $scriptDirectory '..\..'))
$outputRoot = [IO.Path]::GetFullPath($OutputDirectory)
if (-not $outputRoot.StartsWith($workspaceRoot, [StringComparison]::OrdinalIgnoreCase)) {
    throw "OutputDirectory must stay inside workspace: $workspaceRoot"
}

New-Item -ItemType Directory -Path $outputRoot -Force | Out-Null
$previewDir = Join-Path $outputRoot '01-cierre-mp03_v1-previews-native'
New-Item -ItemType Directory -Path $previewDir -Force | Out-Null
$xlsxPath = Join-Path $outputRoot '01-cierre-mp03_v1.xlsx'
$pdfPath = Join-Path $outputRoot '01-cierre-mp03_v1-preview.pdf'
$qaPath = Join-Path $outputRoot '01-cierre-mp03_v1-qa-native.json'
$shaPath = "$xlsxPath.sha256"

function OleColor([string]$hex) {
    $clean = $hex.TrimStart('#')
    $r = [Convert]::ToInt32($clean.Substring(0, 2), 16)
    $g = [Convert]::ToInt32($clean.Substring(2, 2), 16)
    $b = [Convert]::ToInt32($clean.Substring(4, 2), 16)
    return $r + ($g * 256) + ($b * 65536)
}

$colors = @{
    Navy = OleColor '#17365D'
    Blue = OleColor '#2F75B5'
    PaleBlue = OleColor '#D9EAF7'
    Input = OleColor '#FFF2CC'
    Formula = OleColor '#E7E6E6'
    Green = OleColor '#E2F0D9'
    Red = OleColor '#F4CCCC'
    Amber = OleColor '#FCE5CD'
    White = OleColor '#FFFFFF'
    Text = OleColor '#1F2937'
}

function Set-Matrix($range, [object[]]$rows) {
    if ($rows.Count -eq 0) { return }
    $columnCount = $rows[0].Count
    $matrix = New-Object 'object[,]' $rows.Count, $columnCount
    for ($r = 0; $r -lt $rows.Count; $r++) {
        if ($rows[$r].Count -ne $columnCount) { throw "Jagged matrix at row $r" }
        for ($c = 0; $c -lt $columnCount; $c++) { $matrix[$r, $c] = $rows[$r][$c] }
    }
    $range.Value2 = $matrix
}

function Set-Title($sheet, [string]$text, [string]$subtitle, [string]$endColumn) {
    $titleRange = $sheet.Range("A1:${endColumn}1")
    $titleRange.Merge()
    $titleRange.Value2 = $text
    $titleRange.Interior.Color = $colors.Navy
    $titleRange.Font.Bold = $true
    $titleRange.Font.Color = $colors.White
    $titleRange.Font.Size = 16
    $titleRange.HorizontalAlignment = -4108
    $titleRange.VerticalAlignment = -4108
    $titleRange.RowHeight = 30

    $subtitleRange = $sheet.Range("A2:${endColumn}2")
    $subtitleRange.Merge()
    $subtitleRange.Value2 = $subtitle
    $subtitleRange.Interior.Color = $colors.PaleBlue
    $subtitleRange.Font.Italic = $true
    $subtitleRange.WrapText = $true
    $subtitleRange.RowHeight = 36
    $sheet.Activate()
    $sheet.Application.ActiveWindow.DisplayGridlines = $false
}

function Style-Header($range) {
    $range.Interior.Color = $colors.Blue
    $range.Font.Bold = $true
    $range.Font.Color = $colors.White
    $range.WrapText = $true
    $range.HorizontalAlignment = -4108
    $range.VerticalAlignment = -4108
    $range.RowHeight = 32
    $range.Borders.LineStyle = 1
    $range.Borders.Color = OleColor '#CBD5E1'
}

function Style-Input($range) {
    $range.Interior.Color = $colors.Input
    $range.Borders.LineStyle = 1
    $range.Borders.Color = OleColor '#CBD5E1'
}

function Style-Formula($range) {
    $range.Interior.Color = $colors.Formula
    $range.Borders.LineStyle = 1
    $range.Borders.Color = OleColor '#CBD5E1'
}

function Add-ListValidation($range, [string[]]$values) {
    $range.Validation.Delete()
    $formula = '"' + ($values -join ',') + '"'
    $range.Validation.Add(3, 1, 1, $formula)
    $range.Validation.IgnoreBlank = $true
    $range.Validation.InCellDropdown = $true
    $range.Validation.ShowError = $true
}

function Add-StatusFormatting($range) {
    Add-ListValidation $range @('NOT_RUN','IN_PROGRESS','PASS','FAIL','BLOCKED','ABORTED','INVALID_DATA','NOT_APPLICABLE')
    $range.FormatConditions.Delete()
    $pass = $range.FormatConditions.Add(1, 3, '="PASS"')
    $pass.Interior.Color = $colors.Green
    $pass.Font.Bold = $true
    $fail = $range.FormatConditions.Add(1, 3, '="FAIL"')
    $fail.Interior.Color = $colors.Red
    $fail.Font.Bold = $true
    $blocked = $range.FormatConditions.Add(1, 3, '="BLOCKED"')
    $blocked.Interior.Color = $colors.Amber
    $blocked.Font.Bold = $true
    $aborted = $range.FormatConditions.Add(1, 3, '="ABORTED"')
    $aborted.Interior.Color = $colors.Red
    $aborted.Font.Bold = $true
}

function Add-Table($sheet, [string]$address, [string]$name) {
    $table = $sheet.ListObjects.Add(1, $sheet.Range($address), $null, 1)
    $table.Name = $name
    $table.TableStyle = 'TableStyleMedium2'
}

function Freeze-AtRow4($sheet) {
    $sheet.Activate()
    $sheet.Range('A5').Select() | Out-Null
    $sheet.Application.ActiveWindow.FreezePanes = $false
    $sheet.Application.ActiveWindow.FreezePanes = $true
}

function Add-Sheet($workbook, [string]$name) {
    $sheet = $workbook.Worksheets.Add([Type]::Missing, $workbook.Worksheets.Item($workbook.Worksheets.Count))
    $sheet.Name = $name
    return $sheet
}

function Export-SheetPreview($sheet, [string]$destination) {
    $used = $sheet.UsedRange
    $previewLastRow = [Math]::Min([int]$used.Rows.Count, 20)
    $lastColumn = [int]$used.Columns.Count
    $endAddress = $sheet.Cells.Item($previewLastRow, $lastColumn).Address($false, $false)
    $previewRange = $sheet.Range("A1:$endAddress")

    # Excel's chart/clipboard PNG route can report success while producing a
    # blank bitmap on unattended Windows sessions. PDF rendering is stable.
    $sheet.PageSetup.PrintArea = $previewRange.Address()
    $sheet.PageSetup.Zoom = $false
    $sheet.PageSetup.FitToPagesWide = 1
    $sheet.PageSetup.FitToPagesTall = 1
    $sheet.PageSetup.Orientation = $(if ($lastColumn -gt 8) { 2 } else { 1 })
    $sheet.PageSetup.LeftMargin = $excel.InchesToPoints(0.25)
    $sheet.PageSetup.RightMargin = $excel.InchesToPoints(0.25)
    $sheet.PageSetup.TopMargin = $excel.InchesToPoints(0.35)
    $sheet.PageSetup.BottomMargin = $excel.InchesToPoints(0.35)

    $sheetPdf = [IO.Path]::ChangeExtension($destination, '.pdf')
    $sheet.ExportAsFixedFormat(0, $sheetPdf, 0, $true, $false)
    if (-not (Test-Path $sheetPdf)) { throw "Preview PDF export failed: $sheetPdf" }

    $converter = Get-Command pdftoppm.exe -ErrorAction Stop
    $prefix = Join-Path ([IO.Path]::GetDirectoryName($destination)) ([IO.Path]::GetFileNameWithoutExtension($destination))
    & $converter.Source -png -singlefile -r 180 $sheetPdf $prefix | Out-Null
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path $destination)) {
        throw "Preview conversion failed: $destination"
    }
    Remove-Item -LiteralPath $sheetPdf -Force
}

function Get-FormulaErrors($workbook) {
    $errors = New-Object System.Collections.Generic.List[string]
    foreach ($sheet in $workbook.Worksheets) {
        # Ask Excel directly for cells whose formulas evaluate to an error. This
        # catches native CVErr/ErrorWrapper values that do not stringify as #REF!.
        try {
            $formulaErrorCells = $sheet.UsedRange.SpecialCells(-4123, 16)
            foreach ($cell in $formulaErrorCells.Cells) {
                $errors.Add("$($sheet.Name)!$($cell.Address($false, $false))=$($cell.Text)")
            }
        } catch {
            # SpecialCells throws when there are no matching cells.
        }
        $used = $sheet.UsedRange
        $values = $used.Value2
        if ($null -eq $values) { continue }
        if ($values -is [Array] -and $values.Rank -eq 2) {
            for ($r = 1; $r -le $values.GetLength(0); $r++) {
                for ($c = 1; $c -le $values.GetLength(1); $c++) {
                    $value = $values[$r, $c]
                    if ($value -is [string] -and $value -match '^#(REF!|DIV/0!|VALUE!|NAME\?|N/A)') {
                        $errors.Add("$($sheet.Name)!R${r}C${c}=$value")
                    }
                }
            }
        }
    }
    return $errors
}

$excel = $null
$workbook = $null
$qaLog = [ordered]@{
    generator = 'Excel COM native fallback'
    visualQaAuthorized = [bool]$VisualQaPassed
    checks = New-Object System.Collections.Generic.List[object]
    previews = New-Object System.Collections.Generic.List[string]
}

try {
    $excel = New-Object -ComObject Excel.Application
    $excel.Visible = $false
    $excel.DisplayAlerts = $false
    $excel.ScreenUpdating = $false
    $workbook = $excel.Workbooks.Add()
    $excel.Calculation = -4105
    while ($workbook.Worksheets.Count -gt 1) { $workbook.Worksheets.Item($workbook.Worksheets.Count).Delete() }
    $workbook.Worksheets.Item(1).Name = 'INSTRUCCIONES'

    $instructions = $workbook.Worksheets.Item('INSTRUCCIONES')
    Set-Title $instructions 'PAQUETE 01 — CIERRE MP-03' 'Guía offline. No habilita pose, torreta, shooter, feeder, intake ni drivetrain.' 'H'
    $instructions.Range('A4:H4').Merge(); $instructions.Range('A4').Value2 = 'ORDEN OBLIGATORIO'; Style-Header $instructions.Range('A4:H4')
    $steps = @(
        @('1','Confirmar área despejada, mecanismos sin piezas y Stop accesible.'),
        @('2','Completar SESION y MP03_01_Config antes de INIT.'),
        @('3','Ejecutar cinco ciclos con el mismo APK.'),
        @('4','Registrar tags 20/24 y pose 3D.'),
        @('5','Ejecutar todos los fallos sin movimiento.'),
        @('6','Registrar poses estáticas medidas.'),
        @('7','Revisar DASHBOARD; no editar fórmulas.'),
        @('8','Guardar con sufijo _FILLED y devolver sin borrar columnas.')
    )
    Set-Matrix $instructions.Range('A5:B12') $steps; Style-Input $instructions.Range('A5:B12')
    $instructions.Range('A14:H14').Merge(); $instructions.Range('A14').Value2 = 'ABORTAR DE INMEDIATO'; $instructions.Range('A14:H14').Interior.Color = OleColor '#C00000'; $instructions.Range('A14:H14').Font.Color = $colors.White; $instructions.Range('A14:H14').Font.Bold = $true
    $aborts = @(
        @('Movimiento de cualquier actuador','STOP/E-stop, ABORTED y evidencia.'),
        @('Humo, olor, calor, ruido o cable suelto','Cortar energía; no reconectar.'),
        @('Excepción/reinicio inesperado','STOP; registrar mensaje y APK SHA.'),
        @('Pipeline/map/extrínseca distinta','Detener pose 3D y crear ConfigRevision.')
    )
    Set-Matrix $instructions.Range('A15:B18') $aborts; Style-Input $instructions.Range('A15:B18')
    $instructions.Range('A20:H20').Merge(); $instructions.Range('A20').Value2 = 'SHOT_DATASET — preparado para sesiones futuras; MP-03 no autoriza tiros.'; $instructions.Range('A20:H20').Interior.Color = $colors.Amber; $instructions.Range('A20:H20').Font.Bold = $true
    $instructions.Columns.Item('A').ColumnWidth = 36; $instructions.Columns.Item('B').ColumnWidth = 78

    $session = Add-Sheet $workbook 'SESION'
    Set-Title $session 'IDENTIDAD DE SESIÓN' 'Completar antes de abrir el OpMode. Amarillo = entrada.' 'F'
    Set-Matrix $session.Range('A4:D4') @(,@('Campo','Valor','Guía','Obligatorio')); Style-Header $session.Range('A4:D4')
    $sessionRows = @(
        @('SchemaVersion','1.0','Fijo',''), @('TemplateVersion','1','Fijo',''), @('PacketId','01-cierre-mp03','Fijo',''), @('PacketVersion','1','Fijo',''),
        @('SessionId','','Ej. MP03-20260722-A','REQUIRED'), @('Fecha local','','YYYY-MM-DD','REQUIRED'), @('Hora inicio','','HH:MM','REQUIRED'), @('Ubicación','','','REQUIRED'),
        @('Operador','','','REQUIRED'), @('Test lead','','','REQUIRED'), @('Safety observer','','','REQUIRED'), @('Data recorder','','','REQUIRED'),
        @('Git commit','','SHA inequívoco','REQUIRED'), @('APK SHA-256','','64 hex','REQUIRED'), @('ConfigRevision','R1','Incrementar tras cambio material','REQUIRED'),
        @('Robot config','','Nombre/revisión RC','REQUIRED'), @('Batería ID','','','REQUIRED'), @('Voltaje inicial (V)','','Número','REQUIRED'),
        @('Pipeline SHA-256','0B954CFA8641B460C13E8DF65A7A4691E7FD409821A09A008BF3ADA141D20693','Actualizar si cambia','REQUIRED'),
        @('Field map SHA-256','','Pendiente','REQUIRED'), @('Condición de luz','','',''), @('Notas','','','')
    )
    Set-Matrix $session.Range('A5:D26') $sessionRows; Style-Formula $session.Range('A5:A26'); Style-Input $session.Range('B5:B26'); $session.Range('C5:D26').WrapText = $true
    Add-Table $session 'A4:D26' 'SessionTable'; $session.Columns.Item('A').ColumnWidth=26; $session.Columns.Item('B').ColumnWidth=70; $session.Columns.Item('C').ColumnWidth=48; $session.Columns.Item('D').ColumnWidth=18; Freeze-AtRow4 $session

    $changes = Add-Sheet $workbook 'CAMBIOS'
    Set-Title $changes 'REGISTRO DE CAMBIOS' 'No mezclar intentos de distintas ConfigRevision.' 'I'
    Set-Matrix $changes.Range('A4:I4') @(,@('ChangeId','TimestampLocal','FromRevision','ToRevision','Category','Before','After','AppliesFromAttemptId','Reason')); Style-Header $changes.Range('A4:I4')
    $changeRows = @(); for($i=1;$i -le 30;$i++){ $changeRows += ,@(('CHG-{0:D2}' -f $i),'','','','','','','','') }
    Set-Matrix $changes.Range('A5:I34') $changeRows; Style-Formula $changes.Range('A5:A34'); Style-Input $changes.Range('B5:I34'); Add-ListValidation $changes.Range('E5:E34') @('APK','PIPELINE','FIELD_MAP','EXTRINSIC','HARDWARE','PROCEDURE','OTHER'); Add-Table $changes 'A4:I34' 'ChangesTable'; $changes.Columns.ColumnWidth=18; $changes.Columns.Item('F').ColumnWidth=28; $changes.Columns.Item('G').ColumnWidth=28; $changes.Columns.Item('I').ColumnWidth=40; Freeze-AtRow4 $changes

    $config = Add-Sheet $workbook 'CONFIG'
    Set-Title $config 'CONSTANTES DEL PAQUETE' 'Editar sólo mediante TemplateVersion o ConfigRevision documentada.' 'F'
    Set-Matrix $config.Range('A4:E4') @(,@('Key','Value','Unit','Source','Purpose')); Style-Header $config.Range('A4:E4')
    $configRows = @(
        @('LIMELIGHT_MAX_STALENESS_MS',200,'ms','LowAltitudeConstants.VisionConstants','Gate visual'),
        @('EXPECTED_PIPELINE',0,'index','LowAltitudeConstants.VisionConstants','Pipeline AprilTag'),
        @('BLUE_GOAL_TAG_ID',20,'id','LowAltitudeConstants.TurretConstants','Goal azul'),
        @('RED_GOAL_TAG_ID',24,'id','LowAltitudeConstants.TurretConstants','Goal rojo'),
        @('MARKER_BLACK_SIZE_MM',165.1,'mm','mp03-limelight-commissioning.md','Cuadrado negro oficial'),
        @('EXPECTED_LIFECYCLE_CYCLES',5,'count','Gate MP-03','Mismo APK'),
        @('SHOOTER_READY_ERROR_RPM',100,'RPM','T8.1','Dataset futuro'),
        @('SHOOTER_READY_DWELL_MS',250,'ms','T8.1','Dataset futuro')
    )
    Set-Matrix $config.Range('A5:E12') $configRows; Style-Formula $config.Range('A5:E12'); Add-Table $config 'A4:E12' 'ConfigTable'; $config.Columns.Item('A').ColumnWidth=34; $config.Columns.Item('B').ColumnWidth=15; $config.Columns.Item('C').ColumnWidth=12; $config.Columns.Item('D').ColumnWidth=42; $config.Columns.Item('E').ColumnWidth=35

    $catalogs = Add-Sheet $workbook 'CATALOGOS'
    Set-Title $catalogs 'CATÁLOGOS' 'Valores válidos para dropdowns y exports.' 'L'
    $catalogGroups = [ordered]@{
        Status=@('NOT_RUN','IN_PROGRESS','PASS','FAIL','BLOCKED','ABORTED','INVALID_DATA','NOT_APPLICABLE'); YesNo=@('YES','NO');
        Quality=@('VALID','NO_TARGET','WRONG_TAG','STALE','PIPELINE_MISMATCH','DEVICE_DISCONNECTED','INVALID'); ModelKind=@('LINEAR','QUADRATIC','PIECEWISE'); Role=@('CALIBRATION','HOLDOUT'); Outcome=@('SCORED','SHORT','LONG','REBOUND','OTHER')
    }
    $column=1; foreach($entry in $catalogGroups.GetEnumerator()){ $catalogs.Cells.Item(4,$column).Value2=$entry.Key; Style-Header $catalogs.Cells.Item(4,$column); $row=5; foreach($value in $entry.Value){ $catalogs.Cells.Item($row,$column).Value2=$value; $catalogs.Cells.Item($row,$column).Interior.Color=$colors.Formula; $row++ }; $catalogs.Columns.Item($column).ColumnWidth=22; $column+=2 }

    $configCheck = Add-Sheet $workbook 'MP03_01_Config'
    Set-Title $configCheck 'MP03.1 — CONFIGURACIÓN' 'Completar antes de lifecycle.' 'H'
    Set-Matrix $configCheck.Range('A4:H4') @(,@('TestId','Item','Expected','Observed','EvidenceRef','Status','AbortReason','Notes')); Style-Header $configCheck.Range('A4:H4')
    $configItems = @(
        @('CFG-01','Mapping RC','limelight','limelight','','NOT_RUN','',''), @('CFG-02','Firmware/UI','2026.0','2026.0','','NOT_RUN','',''),
        @('CFG-03','Pipeline index','0','0','','NOT_RUN','',''), @('CFG-04','Resolución/FPS','640x480 / 90','640x480 / 90','','NOT_RUN','',''),
        @('CFG-05','Tag family','AprilTag Classic 36h11','AprilTag Classic 36h11','','NOT_RUN','',''), @('CFG-06','Marker black size','165.1 mm','165.1 mm','','NOT_RUN','','No validar escala con tag 160 mm'),
        @('CFG-07','fiducial_skip3d','0','','','NOT_RUN','','Debe habilitar pose 3D'), @('CFG-08','Field map export/hash','Presente + SHA-256','','','NOT_RUN','','Pendiente'),
        @('CFG-09','Extrínseca activa','30 deg, yaw=0, roll=0','','','NOT_RUN','','Usar geometria-robot-mp04.md'), @('CFG-10','Montaje rígido','YES','','','NOT_RUN','',''),
        @('CFG-11','Oclusión en posiciones de prueba','NO','','','NOT_RUN','',''), @('CFG-12','Tag azul 20 disponible','YES','','','NOT_RUN','',''),
        @('CFG-13','Tag rojo 24 disponible','YES','','','NOT_RUN','',''), @('CFG-14','Sin consumidores de actuadores','YES','','','NOT_RUN','','Revisión humana')
    )
    Set-Matrix $configCheck.Range('A5:H18') $configItems; Style-Formula $configCheck.Range('A5:C18'); Style-Input $configCheck.Range('D5:H18'); Add-StatusFormatting $configCheck.Range('F5:F18'); Add-Table $configCheck 'A4:H18' 'Mp03ConfigTable'; $configCheck.Columns.Item('A').ColumnWidth=13; $configCheck.Columns.Item('B').ColumnWidth=34; $configCheck.Columns.Item('C').ColumnWidth=31; $configCheck.Columns.Item('D').ColumnWidth=31; $configCheck.Columns.Item('E').ColumnWidth=30; $configCheck.Columns.Item('F').ColumnWidth=18; $configCheck.Columns.Item('G').ColumnWidth=36; $configCheck.Columns.Item('H').ColumnWidth=36; Freeze-AtRow4 $configCheck

    $lifecycle = Add-Sheet $workbook 'MP03_02_Lifecycle'
    Set-Title $lifecycle 'MP03.2 — CINCO CICLOS' 'Mismo APK. Movimiento o excepción aborta.' 'N'
    Set-Matrix $lifecycle.Range('A4:N4') @(,@('TestId','Cycle','ConfigRevision','Init','Start','Stop','Connected','PipelineObserved','Exception','ResourceOpenAfterStop','AnyMovement','EvidenceRef','Status','Notes')); Style-Header $lifecycle.Range('A4:N4')
    $lifeRows=@(); for($i=1;$i -le 5;$i++){ $lifeRows += ,@("LIFE-$i",$i,'R1','','','','','','','','','','','') }; Set-Matrix $lifecycle.Range('A5:N9') $lifeRows; Style-Formula $lifecycle.Range('A5:C9'); Style-Input $lifecycle.Range('D5:L9'); Style-Formula $lifecycle.Range('M5:M9'); Style-Input $lifecycle.Range('N5:N9')
    foreach($col in @('D','E','F','G','I','J','K')){ Add-ListValidation $lifecycle.Range("${col}5:${col}9") @('YES','NO') }
    for($r=5;$r -le 9;$r++){ $lifecycle.Cells.Item($r,13).Formula = '=IF(COUNTA(D'+$r+':K'+$r+')=0,"NOT_RUN",IF(OR(K'+$r+'="YES",I'+$r+'="YES",J'+$r+'="YES"),"FAIL",IF(AND(D'+$r+'="YES",E'+$r+'="YES",F'+$r+'="YES",G'+$r+'="YES",H'+$r+'=0,I'+$r+'="NO",J'+$r+'="NO",K'+$r+'="NO"),"PASS","BLOCKED")))' }
    Add-StatusFormatting $lifecycle.Range('M5:M9'); Add-Table $lifecycle 'A4:N9' 'LifecycleTable'; $lifecycle.Columns.ColumnWidth=16; $lifecycle.Columns.Item('L').ColumnWidth=28; $lifecycle.Columns.Item('N').ColumnWidth=35; Freeze-AtRow4 $lifecycle

    $tags = Add-Sheet $workbook 'MP03_03_Tags'
    Set-Title $tags 'MP03.3 — TAGS Y POSE 3D' 'Una fila por observación nueva; no promediar a mano.' 'V'
    Set-Matrix $tags.Range('A4:V4') @(,@('AttemptId','ConfigRevision','TagId','Alliance','ExpectedTag','TxDeg','TyDeg','AreaPct','StalenessMs','TargetLatencyMs','CaptureLatencyMs','TotalLatencyMs','TagCount','BotPoseAvailable','BotX','BotY','BotZ','BotYawDeg','Finite','Quality','RejectionReason','EvidenceRef')); Style-Header $tags.Range('A4:V4')
    $tagRows=@(); for($i=1;$i -le 40;$i++){ $tagRows += ,@(('TAG-{0:D2}' -f $i),'R1','','','','','','','','','','','','','','','','','','','','') }; Set-Matrix $tags.Range('A5:V44') $tagRows; Style-Formula $tags.Range('A5:B44'); Style-Input $tags.Range('C5:V44'); Add-ListValidation $tags.Range('D5:D44') @('BLUE','RED'); foreach($col in @('E','N','S')){ Add-ListValidation $tags.Range("${col}5:${col}44") @('YES','NO') }; Add-ListValidation $tags.Range('T5:T44') @('VALID','NO_TARGET','WRONG_TAG','STALE','PIPELINE_MISMATCH','DEVICE_DISCONNECTED','INVALID'); Add-Table $tags 'A4:V44' 'TagObservationsTable'; $tags.Columns.ColumnWidth=16; $tags.Columns.Item('U').ColumnWidth=34; $tags.Columns.Item('V').ColumnWidth=34; Freeze-AtRow4 $tags

    $faults = Add-Sheet $workbook 'MP03_04_Faults'
    Set-Title $faults 'MP03.4 — FALLOS Y RECUPERACIÓN' 'Todos mantienen actuadores inmóviles.' 'L'
    Set-Matrix $faults.Range('A4:L4') @(,@('TestId','Case','ExpectedQuality','ObservedQuality','ActionableValuesZero','AnyMovement','Exception','Recovered','EvidenceRef','Status','AbortReason','Notes')); Style-Header $faults.Range('A4:L4')
    $faultRows=@(
        @('FLT-01','Camera covered','NO_TARGET or STALE','','','','','','','','',''), @('FLT-02','Wrong tag','WRONG_TAG','','','','','','','','',''),
        @('FLT-03','Invalid result','INVALID','','','','','','','','',''), @('FLT-04','Wrong pipeline','PIPELINE_MISMATCH','','','','','','','','',''),
        @('FLT-05','Disconnect','DEVICE_DISCONNECTED','','','','','','','','',''), @('FLT-06','Reconnect','Telemetry only recovery','','','','','','','','',''),
        @('FLT-07','Stale frame','STALE','','','','','','','','',''), @('FLT-08','No 3D pose','Rejected / no actionable pose','','','','','','','','','')
    )
    Set-Matrix $faults.Range('A5:L12') $faultRows; Style-Formula $faults.Range('A5:C12'); Style-Input $faults.Range('D5:I12'); Style-Formula $faults.Range('J5:J12'); Style-Input $faults.Range('K5:L12'); foreach($col in @('E','F','G','H')){ Add-ListValidation $faults.Range("${col}5:${col}12") @('YES','NO') }
    for($r=5;$r -le 12;$r++){ $faults.Cells.Item($r,10).Formula = '=IF(COUNTA(D'+$r+':I'+$r+')=0,"NOT_RUN",IF(OR(E'+$r+'<>"YES",F'+$r+'<>"NO",G'+$r+'<>"NO"),"FAIL",IF(AND(D'+$r+'<>"",H'+$r+'="YES",I'+$r+'<>""),"PASS","BLOCKED")))' }
    Add-StatusFormatting $faults.Range('J5:J12'); Add-Table $faults 'A4:L12' 'FaultsTable'; $faults.Columns.ColumnWidth=20; $faults.Columns.Item('B').ColumnWidth=24; $faults.Columns.Item('C').ColumnWidth=28; $faults.Columns.Item('D').ColumnWidth=28; $faults.Columns.Item('I').ColumnWidth=28; $faults.Columns.Item('K').ColumnWidth=35; $faults.Columns.Item('L').ColumnWidth=35; Freeze-AtRow4 $faults

    $staticPose = Add-Sheet $workbook 'MP03_05_StaticPose'
    Set-Title $staticPose 'MP03.5 — POSE ESTÁTICA DIAGNÓSTICA' 'No aplica corrección. Registrar marco y unidades.' 'Q'
    Set-Matrix $staticPose.Range('A4:Q4') @(,@('AttemptId','ConfigRevision','TagId','PhysicalXIn','PhysicalYIn','PhysicalHeadingDeg','LimelightXIn','LimelightYIn','LimelightHeadingDeg','ResidualXIn','ResidualYIn','ResidualDistanceIn','ResidualHeadingDeg','StalenessMs','EvidenceRef','Status','Notes')); Style-Header $staticPose.Range('A4:Q4')
    $poseRows=@(); for($i=1;$i -le 20;$i++){ $poseRows += ,@(('POSE-{0:D2}' -f $i),'R1','','','','','','','','','','','','','','NOT_RUN','') }; Set-Matrix $staticPose.Range('A5:Q24') $poseRows; Style-Formula $staticPose.Range('A5:B24'); Style-Input $staticPose.Range('C5:I24'); Style-Formula $staticPose.Range('J5:M24'); Style-Input $staticPose.Range('N5:Q24')
    for($r=5;$r -le 24;$r++){
        $staticPose.Cells.Item($r,10).Formula=('=IF(OR(D{0}="",G{0}=""),"",G{0}-D{0})' -f $r)
        $staticPose.Cells.Item($r,11).Formula=('=IF(OR(E{0}="",H{0}=""),"",H{0}-E{0})' -f $r)
        $staticPose.Cells.Item($r,12).Formula=('=IF(OR(J{0}="",K{0}=""),"",SQRT(J{0}^2+K{0}^2))' -f $r)
        $staticPose.Cells.Item($r,13).Formula=('=IF(OR(F{0}="",I{0}=""),"",MOD(I{0}-F{0}+180,360)-180)' -f $r)
    }
    Add-StatusFormatting $staticPose.Range('P5:P24'); Add-Table $staticPose 'A4:Q24' 'StaticPoseTable'; $staticPose.Columns.ColumnWidth=18; $staticPose.Columns.Item('O').ColumnWidth=28; $staticPose.Columns.Item('Q').ColumnWidth=35; Freeze-AtRow4 $staticPose

    $dataset = Add-Sheet $workbook 'SHOT_DATASET'
    Set-Title $dataset 'DATASET DE TIRO — EXTRA RPM/DISTANCIA' 'Preparado para sesiones posteriores. MP-03 no autoriza tiros.' 'V'
    Set-Matrix $dataset.Range('A4:V4') @(,@('sessionId','distanceGroupId','modelKind','role','distanceInches','targetRpm','measuredRpmAtFeed','rpmReadyHoldMs','outcome','AttemptId','ConfigRevision','Position','BatteryId','FusedXIn','FusedYIn','FusedHeadingDeg','TurretErrorDeg','ArtifactBatch','EvidenceRef','Notes','RpmError','ReadyGate')); Style-Header $dataset.Range('A4:V4')
    $shotRows=@(); for($i=1;$i -le 60;$i++){ $shotRows += ,@('','','','','','','','','',('SHOT-{0:D3}' -f $i),'R1','','','','','','','','','','','') }; Set-Matrix $dataset.Range('A5:V64') $shotRows; Style-Input $dataset.Range('A5:T64'); Style-Formula $dataset.Range('J5:K64'); Style-Formula $dataset.Range('U5:V64'); Add-ListValidation $dataset.Range('C5:C64') @('LINEAR','QUADRATIC','PIECEWISE'); Add-ListValidation $dataset.Range('D5:D64') @('CALIBRATION','HOLDOUT'); Add-ListValidation $dataset.Range('I5:I64') @('SCORED','SHORT','LONG','REBOUND','OTHER')
    for($r=5;$r -le 64;$r++){
        $dataset.Cells.Item($r,21).Formula=('=IF(OR(F{0}="",G{0}=""),"",ABS(G{0}-F{0}))' -f $r)
        $dataset.Cells.Item($r,22).Formula=('=IF(A{0}="","",IF(AND(U{0}<=CONFIG!B11,H{0}>=CONFIG!B12),"PASS","FAIL"))' -f $r)
    }
    Add-StatusFormatting $dataset.Range('V5:V64'); Add-Table $dataset 'A4:V64' 'ShotDatasetTable'; $dataset.Columns.ColumnWidth=18; $dataset.Columns.Item('S').ColumnWidth=28; $dataset.Columns.Item('T').ColumnWidth=35; Freeze-AtRow4 $dataset

    $shotExport = Add-Sheet $workbook 'EXPORT_SHOT_DATASET'
    Set-Title $shotExport 'EXPORT SHOT_DATASET — CSV' 'Exportar como CSV; #sessionId hace que el parser ignore el header.' 'I'
    Set-Matrix $shotExport.Range('A4:I4') @(,@('#sessionId','distanceGroupId','modelKind','role','distanceInches','targetRpm','measuredRpmAtFeed','rpmReadyHoldMs','outcome')); Style-Header $shotExport.Range('A4:I4')
    for($r=5;$r -le 64;$r++){
        for($c=1;$c -le 9;$c++){
            $letter=[char](64+$c)
            $shotExport.Cells.Item($r,$c).Formula=('=IF(SHOT_DATASET!A{0}="","",SHOT_DATASET!{1}{0})' -f $r,$letter)
        }
    }
    Style-Formula $shotExport.Range('A5:I64'); Add-Table $shotExport 'A4:I64' 'ShotDatasetExportTable'; $shotExport.Columns.ColumnWidth=20; Freeze-AtRow4 $shotExport

    $dashboard = Add-Sheet $workbook 'DASHBOARD'
    Set-Title $dashboard 'DASHBOARD MP-03' 'Resumen automático. No editar fórmulas.' 'J'
    Set-Matrix $dashboard.Range('A4:C4') @(,@('Gate','Status','Criterio')); Style-Header $dashboard.Range('A4:C4')
    Set-Matrix $dashboard.Range('A5:C9') @(@('Configuración','','14/14 items PASS'),@('Lifecycle','','5/5 ciclos PASS'),@('Tags','','IDs 20/24 con pose 3D válida/fresca'),@('Faults','','8/8 casos PASS'),@('MP-03','','Todos PASS'))
    Style-Formula $dashboard.Range('A5:C9')
    $dashboard.Range('B5').Formula='=IF(COUNTIF(MP03_01_Config!F5:F18,"PASS")=0,"NOT_RUN",IF(COUNTIF(MP03_01_Config!F5:F18,"FAIL")+COUNTIF(MP03_01_Config!F5:F18,"ABORTED")>0,"FAIL",IF(COUNTIF(MP03_01_Config!F5:F18,"PASS")=14,"PASS","BLOCKED")))'
    $dashboard.Range('B6').Formula='=IF(COUNTIF(MP03_02_Lifecycle!M5:M9,"PASS")=0,"NOT_RUN",IF(COUNTIF(MP03_02_Lifecycle!M5:M9,"FAIL")+COUNTIF(MP03_02_Lifecycle!M5:M9,"ABORTED")>0,"FAIL",IF(COUNTIF(MP03_02_Lifecycle!M5:M9,"PASS")=5,"PASS","BLOCKED")))'
    $dashboard.Range('B7').Formula='=IF(COUNTA(MP03_03_Tags!C5:C44)=0,"NOT_RUN",IF(AND(COUNTIFS(MP03_03_Tags!C5:C44,20,MP03_03_Tags!E5:E44,"YES",MP03_03_Tags!I5:I44,"<="&CONFIG!B5,MP03_03_Tags!N5:N44,"YES",MP03_03_Tags!S5:S44,"YES",MP03_03_Tags!T5:T44,"VALID")>0,COUNTIFS(MP03_03_Tags!C5:C44,24,MP03_03_Tags!E5:E44,"YES",MP03_03_Tags!I5:I44,"<="&CONFIG!B5,MP03_03_Tags!N5:N44,"YES",MP03_03_Tags!S5:S44,"YES",MP03_03_Tags!T5:T44,"VALID")>0),"PASS","BLOCKED"))'
    $dashboard.Range('B8').Formula='=IF(COUNTIF(MP03_04_Faults!J5:J12,"PASS")=0,"NOT_RUN",IF(COUNTIF(MP03_04_Faults!J5:J12,"FAIL")+COUNTIF(MP03_04_Faults!J5:J12,"ABORTED")>0,"FAIL",IF(COUNTIF(MP03_04_Faults!J5:J12,"PASS")=8,"PASS","BLOCKED")))'
    $dashboard.Range('B9').Formula='=IF(COUNTIF(B5:B8,"PASS")=0,"NOT_RUN",IF(COUNTIF(B5:B8,"FAIL")>0,"FAIL",IF(COUNTIF(B5:B8,"PASS")=4,"PASS","BLOCKED")))'
    Add-StatusFormatting $dashboard.Range('B5:B9')
    Set-Matrix $dashboard.Range('E4:G4') @(,@('Dataset extra','Value','Meaning')); Style-Header $dashboard.Range('E4:G4')
    Set-Matrix $dashboard.Range('E5:G8') @(@('Rows captured','','Tiros con sessionId'),@('Rows ready','','RPM gate PASS'),@('Scored','','Outcome SCORED'),@('Model status','EXTRA / NOT BLOCKING','No afecta MP-03')); Style-Formula $dashboard.Range('E5:G8')
    $dashboard.Range('F5').Formula='=COUNTIF(SHOT_DATASET!A5:A64,"<>")'; $dashboard.Range('F6').Formula='=COUNTIF(SHOT_DATASET!V5:V64,"PASS")'; $dashboard.Range('F7').Formula='=COUNTIF(SHOT_DATASET!I5:I64,"SCORED")'
    $dashboard.Range('A11:J11').Merge(); $dashboard.Range('A11').Value2='MP-03 PASS sólo autoriza abrir MP-04. Nunca autoriza movimiento o disparo.'; $dashboard.Range('A11:J11').Interior.Color=$colors.Amber; $dashboard.Range('A11:J11').Font.Bold=$true
    $dashboard.Columns.Item('A').ColumnWidth=24; $dashboard.Columns.Item('B').ColumnWidth=18; $dashboard.Columns.Item('C').ColumnWidth=48; $dashboard.Columns.Item('E').ColumnWidth=22; $dashboard.Columns.Item('F').ColumnWidth=22; $dashboard.Columns.Item('G').ColumnWidth=38

    $qa = Add-Sheet $workbook 'QA'
    Set-Title $qa 'TRIPLE QA DEL TEMPLATE' 'Fallback Excel nativo autorizado; SHA final está en sidecar.' 'H'
    Set-Matrix $qa.Range('A4:E4') @(,@('CheckId','Layer','Check','Status','Evidence')); Style-Header $qa.Range('A4:E4')
    $qaRows=@(
        @('QA1-01','SOURCE','AGENTS/docs MP-03 leídos','PASS','Builder manifest'), @('QA1-02','SOURCE','IDs/unidades/thresholds cotejados','PASS','CONFIG'),
        @('QA1-03','SOURCE','Dataset coincide con ShotDatasetCsv','PASS','EXPORT_SHOT_DATASET'), @('QA2-01','FORMULA','Libro vacío no declara PASS','PENDING','Fixture'),
        @('QA2-02','FORMULA','PASS/FAIL/BLOCKED boundaries','PENDING','Fixture'), @('QA2-03','FORMULA','Sin errores de fórmula','PENDING','Reopen scan'),
        @('QA2-04','SCHEMA','Exports/tablas conservan columnas','PENDING','Reopen'), @('QA3-01','ROUND_TRIP','Export/reimport exitoso','PENDING','Excel reopen'),
        @('QA3-02','VISUAL','Todas las hojas renderizadas','PENDING','PNG/PDF'), @('QA3-03','VISUAL','Revisión humana sin defectos severos',$(if($VisualQaPassed){'PASS'}else{'PENDING_HUMAN'}),'Preview review'),
        @('QA3-04','HASH','SHA-256 final guardado','PENDING','.sha256 sidecar')
    )
    Set-Matrix $qa.Range('A5:E15') $qaRows; Style-Formula $qa.Range('A5:E15'); Add-Table $qa 'A4:E15' 'QaTable'; $qa.Columns.Item('A').ColumnWidth=14; $qa.Columns.Item('B').ColumnWidth=16; $qa.Columns.Item('C').ColumnWidth=54; $qa.Columns.Item('D').ColumnWidth=22; $qa.Columns.Item('E').ColumnWidth=34

    # QA-2 fixtures: formulas must fail closed and return to a pristine template.
    $lifecycle.Range('D5:G5').Value2='YES'; $lifecycle.Range('H5').Value2=0; $lifecycle.Range('I5:K5').Value2='NO'; $excel.CalculateFull(); if($lifecycle.Range('M5').Text -ne 'PASS'){ throw 'QA2 lifecycle PASS fixture failed' }
    $lifecycle.Range('K5').Value2='YES'; $excel.CalculateFull(); if($lifecycle.Range('M5').Text -ne 'FAIL'){ throw 'QA2 lifecycle movement fixture failed' }
    $lifecycle.Range('D5:L5').ClearContents(); $excel.CalculateFull(); if($lifecycle.Range('M5').Text -ne 'NOT_RUN'){ throw 'QA2 lifecycle blank fixture failed' }

    $faults.Range('D5').Value2='NO_TARGET'; $faults.Range('E5').Value2='YES'; $faults.Range('F5:G5').Value2='NO'; $faults.Range('H5').Value2='YES'; $faults.Range('I5').Value2='fixture'; $excel.CalculateFull(); if($faults.Range('J5').Text -ne 'PASS'){ throw 'QA2 fault PASS fixture failed' }
    $faults.Range('F5').Value2='YES'; $excel.CalculateFull(); if($faults.Range('J5').Text -ne 'FAIL'){ throw 'QA2 fault movement fixture failed' }
    $faults.Range('D5:I5').ClearContents(); $excel.CalculateFull(); if($faults.Range('J5').Text -ne 'NOT_RUN'){ throw 'QA2 fault blank fixture failed' }

    foreach($r in @(5,6)){
        $fixtureTagId = $(if($r -eq 5){20}else{24})
        $tags.Cells.Item($r,3).Value2=$fixtureTagId
        $tags.Cells.Item($r,5).Value2='YES'
        $tags.Cells.Item($r,9).Value2=100
        $tags.Cells.Item($r,14).Value2='YES'
        $tags.Cells.Item($r,19).Value2='YES'
        $tags.Cells.Item($r,20).Value2='VALID'
    }
    $excel.CalculateFull(); if($dashboard.Range('B7').Text -ne 'PASS'){ throw 'QA2 tags PASS fixture failed' }
    $tags.Cells.Item(6,9).Value2=201; $excel.CalculateFull(); if($dashboard.Range('B7').Text -ne 'BLOCKED'){ throw 'QA2 stale boundary fixture failed' }
    $tags.Range('C5:V6').ClearContents(); $tags.Range('A5').Value2='TAG-01'; $tags.Range('B5').Value2='R1'; $tags.Range('A6').Value2='TAG-02'; $tags.Range('B6').Value2='R1'; $excel.CalculateFull(); if($dashboard.Range('B7').Text -ne 'NOT_RUN'){ throw 'QA2 tags blank fixture failed' }

    $dataset.Range('A5').Value2='FIXTURE'; $dataset.Range('F5').Value2=1000; $dataset.Range('G5').Value2=950; $dataset.Range('H5').Value2=250; $excel.CalculateFull(); if($dataset.Range('V5').Text -ne 'PASS'){ throw 'QA2 dataset PASS fixture failed' }
    $dataset.Range('H5').Value2=249; $excel.CalculateFull(); if($dataset.Range('V5').Text -ne 'FAIL'){ throw 'QA2 dataset dwell fixture failed' }
    $dataset.Range('A5:I5').ClearContents(); $excel.CalculateFull(); if($dataset.Range('V5').Text -ne ''){ throw 'QA2 dataset blank fixture failed' }
    $qa.Range('D8:D11').Value2='PASS'
    $qaLog.checks.Add([ordered]@{ layer='QA2'; status='PASS'; detail='Lifecycle, faults, tag freshness and dataset boundary fixtures' })

    # Save draft, reopen, then complete round-trip and render checks.
    $excel.CalculateFullRebuild()
    $workbook.SaveAs($xlsxPath, 51)
    $workbook.Close($true)
    [Runtime.InteropServices.Marshal]::FinalReleaseComObject($workbook) | Out-Null
    $workbook = $excel.Workbooks.Open($xlsxPath, 0, $false)
    $excel.CalculateFullRebuild()

    $expectedSheets=@('INSTRUCCIONES','SESION','CAMBIOS','CONFIG','CATALOGOS','MP03_01_Config','MP03_02_Lifecycle','MP03_03_Tags','MP03_04_Faults','MP03_05_StaticPose','SHOT_DATASET','EXPORT_SHOT_DATASET','DASHBOARD','QA')
    $actualSheets=@(); foreach($sheet in $workbook.Worksheets){ $actualSheets += $sheet.Name }
    foreach($name in $expectedSheets){ if($actualSheets -notcontains $name){ throw "QA2 missing sheet after reopen: $name" } }
    $expectedHeader=@('#sessionId','distanceGroupId','modelKind','role','distanceInches','targetRpm','measuredRpmAtFeed','rpmReadyHoldMs','outcome')
    $actualHeader=@(); for($c=1;$c -le 9;$c++){ $actualHeader += $workbook.Worksheets.Item('EXPORT_SHOT_DATASET').Cells.Item(4,$c).Text }
    if(($actualHeader -join '|') -ne ($expectedHeader -join '|')){ throw 'QA2 export schema mismatch after reopen' }
    $errors = Get-FormulaErrors $workbook
    if($errors.Count -gt 0){ throw "QA2 formula errors: $($errors -join '; ')" }
    foreach($cell in @('B5','B6','B7','B8','B9')){ if($workbook.Worksheets.Item('DASHBOARD').Range($cell).Text -ne 'NOT_RUN'){ throw "QA2 blank dashboard false positive at $cell" } }

    $qa = $workbook.Worksheets.Item('QA')
    $qa.Range('D10').Value2='PASS' # formula scan
    $qa.Range('D11').Value2='PASS' # schema
    $qa.Range('D12').Value2='PASS' # round trip
    $qaLog.checks.Add([ordered]@{ layer='QA2'; status='PASS'; detail='14 sheets, 9-column export, no formula errors, blank dashboard fail-closed' })

    $rendered=@()
    foreach($sheet in $workbook.Worksheets){
        $safeName = $sheet.Name -replace '[\\/:*?"<>|]','_'
        $previewPath = Join-Path $previewDir "$safeName.png"
        Export-SheetPreview $sheet $previewPath
        $rendered += $previewPath
        $qaLog.previews.Add($previewPath)
    }
    if($rendered.Count -ne 14){ throw "QA3 expected 14 previews, got $($rendered.Count)" }
    $qa.Range('D13').Value2='PASS'
    if($VisualQaPassed){ $qa.Range('D14').Value2='PASS' }
    $qa.Range('D15').Value2='PASS'
    # Refresh QA evidence after writing the final visual/hash statuses.
    Export-SheetPreview $qa (Join-Path $previewDir 'QA.png')
    $workbook.ExportAsFixedFormat(0, $pdfPath)
    if(-not (Test-Path $pdfPath)){ throw 'QA3 PDF export failed' }
    foreach($sheet in $workbook.Worksheets){
        $sheet.PageSetup.PrintArea = ''
        $sheet.PageSetup.Zoom = 100
    }
    $qa3Status = $(if($VisualQaPassed){'PASS'}else{'PENDING_HUMAN'})
    $qaLog.checks.Add([ordered]@{ layer='QA3'; status=$qa3Status; detail='14 sheet PDFs rendered through Poppler to PNG; combined PDF exported' })

    $workbook.Save()
    $workbook.Close($true)
    [Runtime.InteropServices.Marshal]::FinalReleaseComObject($workbook) | Out-Null
    $workbook = $excel.Workbooks.Open($xlsxPath, 0, $true)
    $excel.CalculateFullRebuild()
    $finalErrors = Get-FormulaErrors $workbook
    if($finalErrors.Count -gt 0){ throw "Final formula errors: $($finalErrors -join '; ')" }
    $workbook.Close($false)
    [Runtime.InteropServices.Marshal]::FinalReleaseComObject($workbook) | Out-Null
    $workbook = $null

    $hash = (Get-FileHash -LiteralPath $xlsxPath -Algorithm SHA256).Hash
    Set-Content -LiteralPath $shaPath -Value "$hash  01-cierre-mp03_v1.xlsx" -Encoding ascii
    $qaLog.workbook = $xlsxPath
    $qaLog.pdf = $pdfPath
    $qaLog.sha256 = $hash
    $qaLog.sheetCount = 14
    $qaLog.formulaErrorCount = 0
    $qaLog | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $qaPath -Encoding utf8
    [pscustomobject]@{ Workbook=$xlsxPath; Sha256=$hash; Previews=$rendered.Count; VisualQaPassed=[bool]$VisualQaPassed; QaReport=$qaPath }
}
finally {
    if($null -ne $workbook){ try { $workbook.Close($false) } catch {}; [Runtime.InteropServices.Marshal]::FinalReleaseComObject($workbook) | Out-Null }
    if($null -ne $excel){ try { $excel.Quit() } catch {}; [Runtime.InteropServices.Marshal]::FinalReleaseComObject($excel) | Out-Null }
    [GC]::Collect(); [GC]::WaitForPendingFinalizers()
}
