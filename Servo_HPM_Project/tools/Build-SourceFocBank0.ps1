param(
    [string]$EmBuild = 'D:\TOOL\SEGGERS\bin\emBuild.exe'
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$project = Join-Path $projectRoot 'hpm6e00evk_flash_xip_debug\segger_embedded_studio\Servo_HPM_Project.emProject'
$outputDir = Join-Path (Split-Path $project -Parent) 'Output\BANK0\Exe'
$diagnosticDir = Join-Path $projectRoot 'diagnostics\source_foc_check'
if (-not (Test-Path -LiteralPath $EmBuild)) { throw "emBuild not found: $EmBuild" }
New-Item -ItemType Directory -Force -Path $diagnosticDir | Out-Null
$log = Join-Path $diagnosticDir 'bank0_full_build.log'
$started = Get-Date
& $EmBuild -config BANK0 -rebuild $project *> $log
$buildExit = $LASTEXITCODE
if ($buildExit -ne 0) {
    Get-Content -LiteralPath $log -Tail 60
    throw "BANK0 build failed ($buildExit). See $log"
}
$artifacts = foreach ($name in @('Servo_Bank0.elf', 'Servo_Bank0.bin', 'Servo_Bank0.map')) {
    $item = Get-Item -LiteralPath (Join-Path $outputDir $name)
    if ($item.LastWriteTime -lt $started.AddSeconds(-2)) { throw "Stale build artifact: $name" }
    [ordered]@{ name = $name; bytes = $item.Length; sha256 = (Get-FileHash -LiteralPath $item.FullName -Algorithm SHA256).Hash }
}
$map = Get-Content -LiteralPath (Join-Path $outputDir 'Servo_Bank0.map') -Raw
foreach ($symbol in @('SourceFoc_Step', 'ProductionMetrics_Sample', 'g_sensorless_production', 'g_source_foc_ready')) {
    if (-not $map.Contains($symbol)) { throw "Missing firmware symbol: $symbol" }
}
[ordered]@{
    configuration = 'BANK0'; built_at = (Get-Date).ToString('o'); project = $project
    tool = $EmBuild; artifacts = @($artifacts); hardware_validated = $false
} | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $diagnosticDir 'bank0_manifest.json') -Encoding UTF8
Write-Output "BANK0 full build PASS: $outputDir"
Write-Output "Build log: $log"
