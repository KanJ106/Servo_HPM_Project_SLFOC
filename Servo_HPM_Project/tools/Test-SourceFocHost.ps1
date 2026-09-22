param([string[]]$Suite)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskModule = Join-Path $taskRoot 'CtrlLoop/33_Sensorless'
$taskOutput = Join-Path $taskRoot 'diagnostics/source_foc_check'
New-Item -ItemType Directory -Force -Path $taskOutput | Out-Null
$taskVswhere = 'C:/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe'
$taskVs = & $taskVswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (!$taskVs) { throw 'MSVC host compiler unavailable' }
$taskDev = Join-Path $taskVs 'Common7/Tools/VsDevCmd.bat'
$taskGcc = 'D:/WORKING/sdk/toolchains/rv32imac_zicsr_zifencei_multilib_b_ext-win/bin/riscv32-unknown-elf-gcc.exe'
$taskSuites = @{
 'test_trace_schedule' = @('SensorlessTakeoverProbe');
 'test_probe_exit' = @('SensorlessTakeoverProbe','SensorlessStartup','RotorFeedback');
 'test_probe_diagnostics' = @('SensorlessTakeoverProbe');
 'test_takeover_probe' = @('SensorlessTakeoverProbe','SensorlessStartup','RotorFeedback');
 'test_align_capture' = @('SourceFoc','MotorParamProfile','ProductionMetrics','SensorlessShadow','SensorlessStartup','SmoPll','RotorFeedback','HfiEstimator','MotorParamIdentification');
 'test_smo_stability' = @('SourceFoc','MotorParamProfile','SmoPll');
 'test_smo_trace' = @();
 'test_travel90' = @('SensorlessStartup','RotorFeedback');
 'test_startup_timing' = @();
 'test_vofa' = @();
 'test_encoder_monitor' = @();
 'test_canopen_transport' = @('SensorlessCanopen','SourceFoc','MotorParamProfile','ProductionMetrics','SensorlessShadow','SensorlessStartup','SmoPll','RotorFeedback','HfiEstimator','MotorParamIdentification','SensorlessProduction','SensorlessSafety');
 'test_debug_chain' = @('SourceFoc','MotorParamProfile','ProductionMetrics','SensorlessShadow','SensorlessStartup','SmoPll','RotorFeedback','HfiEstimator','MotorParamIdentification','SensorlessProduction','SensorlessSafety');
 'test_startup_debug' = @('SourceFoc','MotorParamProfile','SmoPll','SensorlessStartup','RotorFeedback');
 'test_source_foc' = @('SourceFoc','MotorParamProfile','ProductionMetrics');
 'test_source_recipe_integration' = @('SourceFoc','MotorParamProfile','ProductionMetrics','SensorlessShadow','SensorlessStartup','SmoPll','RotorFeedback','HfiEstimator','MotorParamIdentification')
}
foreach ($taskSuite in $taskSuites.Keys) {
    if ($Suite -and $taskSuite -notin $Suite) { continue }
    $taskFiles = @((Join-Path $taskModule "tests/$taskSuite.c"))
    if ('SensorlessStartup' -in $taskSuites[$taskSuite] -and 'SensorlessTakeoverProbe' -notin $taskSuites[$taskSuite]) {
        $taskFiles += Join-Path $taskModule 'source/SensorlessTakeoverProbe.c'
    }
    $taskFiles += $taskSuites[$taskSuite] | ForEach-Object { Join-Path $taskModule "source/$_.c" }
    $taskIncludes = @((Join-Path $taskRoot 'diagnostics/phase46_compile_stub'),(Join-Path $taskModule 'include'))
    $taskArguments = @('-std=gnu11','-O2','-Wall','-Wextra','-Werror','-march=rv32imac_zba_zbb_zbc_zbs_zicsr_zifencei','-mabi=ilp32')
    $taskArguments += $taskIncludes | ForEach-Object { '-I' + $_ }
    if ($taskSuite -eq 'test_canopen_transport') { $taskArguments += '-DSENSORLESS_CANOPEN_BUILD=1' }
    & $taskGcc @taskArguments @taskFiles '-lm' '-o' (Join-Path $taskOutput "$taskSuite.elf")
    if ($LASTEXITCODE) { throw "Target build failed: $taskSuite" }
    $taskExe = Join-Path $taskOutput "$taskSuite.exe"
    $taskCommand = 'cd /d "' + $taskOutput + '" && call "' + $taskDev + '" -no_logo && cl /nologo /W4 /O2 /std:c11 '
    $taskCommand += ($taskIncludes | ForEach-Object { '/I"' + $_ + '"' }) -join ' '
    $taskCommand += ' ' + (($taskFiles | ForEach-Object { '"' + $_ + '"' }) -join ' ')
    if ($taskSuite -eq 'test_canopen_transport') { $taskCommand += ' /DSENSORLESS_CANOPEN_BUILD=1' }
    $taskCommand += ' /Fe:"' + $taskExe + '" && "' + $taskExe + '"'
    & cmd.exe /d /s /c $taskCommand
    if ($LASTEXITCODE) { throw "Host test failed: $taskSuite" }
}
