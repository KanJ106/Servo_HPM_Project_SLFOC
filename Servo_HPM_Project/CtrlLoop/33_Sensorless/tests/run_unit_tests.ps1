param(
    [switch]$RunSimulator
)

$ErrorActionPreference = 'Stop'

$testDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
$moduleDirectory = Split-Path -Parent $testDirectory
$projectRoot = Split-Path -Parent (Split-Path -Parent $moduleDirectory)
$toolchain = 'D:\WORKING\sdk\toolchains\rv32imac_zicsr_zifencei_multilib_b_ext-win\bin'
$gcc = Join-Path $toolchain 'riscv32-unknown-elf-gcc.exe'
$simulator = Join-Path $toolchain 'riscv32-unknown-elf-run.exe'
$output = Join-Path $testDirectory 'test_sensorless.elf'
$phase46Output = Join-Path $testDirectory 'test_sensorless_phase46.elf'
$productionOutput = Join-Path $testDirectory 'test_sensorless_production.elf'
$hfiOutput = Join-Path $testDirectory 'test_hfi_estimator.elf'
$identOutput = Join-Path $testDirectory 'test_motor_param_identification.elf'
$stubDirectory = Join-Path $projectRoot 'diagnostics\phase46_compile_stub'

& $gcc `
    '-std=gnu11' `
    '-O2' `
    '-march=rv32imac_zba_zbb_zbc_zbs_zicsr_zifencei' `
    '-mabi=ilp32' `
    "-I$(Join-Path $moduleDirectory 'include')" `
    (Join-Path $testDirectory 'test_sensorless.c') `
    (Join-Path $moduleDirectory 'source\RotorFeedback.c') `
    (Join-Path $moduleDirectory 'source\SmoPll.c') `
    '-lm' `
    '-o' $output

if ($LASTEXITCODE -ne 0) {
    throw "Sensorless unit-test build failed with exit code $LASTEXITCODE"
}

Write-Host 'PASS: sensorless unit-test sources compiled for the target RISC-V ISA.'

& $gcc `
    '-std=gnu11' `
    '-O2' `
    '-march=rv32imac_zba_zbb_zbc_zbs_zicsr_zifencei' `
    '-mabi=ilp32' `
    "-I$(Join-Path $moduleDirectory 'include')" `
    (Join-Path $testDirectory 'test_sensorless_phase46.c') `
    (Join-Path $moduleDirectory 'source\MotorParamProfile.c') `
    (Join-Path $moduleDirectory 'source\RotorFeedback.c') `
    (Join-Path $moduleDirectory 'source\SensorlessStartup.c') `
    (Join-Path $moduleDirectory 'source\SensorlessTakeoverProbe.c') `
    '-lm' `
    '-o' $phase46Output

if ($LASTEXITCODE -ne 0) {
    throw "Sensorless phase-4-to-6 unit-test build failed with exit code $LASTEXITCODE"
}

Write-Host 'PASS: phase-4-to-6 unit-test sources compiled for the target RISC-V ISA.'

& $gcc `
    '-std=gnu11' `
    '-O2' `
    '-march=rv32imac_zba_zbb_zbc_zbs_zicsr_zifencei' `
    '-mabi=ilp32' `
    "-I$stubDirectory" `
    "-I$(Join-Path $moduleDirectory 'include')" `
    (Join-Path $testDirectory 'test_sensorless_production.c') `
    (Join-Path $moduleDirectory 'source\RotorFeedback.c') `
    (Join-Path $moduleDirectory 'source\SensorlessStartup.c') `
    (Join-Path $moduleDirectory 'source\SensorlessTakeoverProbe.c') `
    (Join-Path $moduleDirectory 'source\SensorlessProduction.c') `
    (Join-Path $moduleDirectory 'source\ProductionMetrics.c') `
    (Join-Path $moduleDirectory 'source\SensorlessSafety.c') `
    (Join-Path $moduleDirectory 'source\HfiEstimator.c') `
    (Join-Path $moduleDirectory 'source\MotorParamIdentification.c') `
    (Join-Path $moduleDirectory 'source\SmoPll.c') `
    '-lm' `
    '-o' $productionOutput

if ($LASTEXITCODE -ne 0) {
    throw "Sensorless production-interface test build failed with exit code $LASTEXITCODE"
}

Write-Host 'PASS: production-interface tests compiled for the target RISC-V ISA.'

& $gcc `
    '-std=gnu11' `
    '-O2' `
    '-march=rv32imac_zba_zbb_zbc_zbs_zicsr_zifencei' `
    '-mabi=ilp32' `
    "-I$(Join-Path $moduleDirectory 'include')" `
    (Join-Path $testDirectory 'test_hfi_estimator.c') `
    (Join-Path $moduleDirectory 'source\HfiEstimator.c') `
    '-lm' `
    '-o' $hfiOutput

if ($LASTEXITCODE -ne 0) {
    throw "HFI estimator unit-test build failed with exit code $LASTEXITCODE"
}

Write-Host 'PASS: HFI estimator tests compiled for the target RISC-V ISA.'

& $gcc `
    '-std=gnu11' `
    '-O2' `
    '-march=rv32imac_zba_zbb_zbc_zbs_zicsr_zifencei' `
    '-mabi=ilp32' `
    "-I$(Join-Path $moduleDirectory 'include')" `
    (Join-Path $testDirectory 'test_motor_param_identification.c') `
    (Join-Path $moduleDirectory 'source\MotorParamIdentification.c') `
    '-lm' `
    '-o' $identOutput

if ($LASTEXITCODE -ne 0) {
    throw "Motor-parameter identification test build failed with exit code $LASTEXITCODE"
}

Write-Host 'PASS: motor-parameter identification tests compiled for the target RISC-V ISA.'

$vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
if (Test-Path -LiteralPath $vswhere) {
    $vsInstall = & $vswhere -latest -products * `
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
        -property installationPath
    $vsDevCmd = Join-Path $vsInstall 'Common7\Tools\VsDevCmd.bat'

    if (Test-Path -LiteralPath $vsDevCmd) {
        $hostBuildDirectory = Join-Path $projectRoot 'diagnostics\host_test'
        New-Item -ItemType Directory -Force -Path $hostBuildDirectory | Out-Null
        $hostOutput = Join-Path $hostBuildDirectory 'test_sensorless_host.exe'
        $phase46HostOutput = Join-Path $hostBuildDirectory 'test_sensorless_phase46_host.exe'
        $productionHostOutput = Join-Path $hostBuildDirectory 'test_sensorless_production_host.exe'
        $hfiHostOutput = Join-Path $hostBuildDirectory 'test_hfi_estimator_host.exe'
        $identHostOutput = Join-Path $hostBuildDirectory 'test_motor_param_identification_host.exe'
        $hostCommand = "cd /d `"$hostBuildDirectory`"" +
            " && call `"$vsDevCmd`" -no_logo" +
            " && cl /nologo /W4 /O2 /std:c11 /I`"$(Join-Path $moduleDirectory 'include')`"" +
            " `"$(Join-Path $testDirectory 'test_sensorless.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\RotorFeedback.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\SmoPll.c')`"" +
            " /Fe:`"$hostOutput`"" +
            " && `"$hostOutput`""

        & cmd.exe /d /s /c $hostCommand
        if ($LASTEXITCODE -ne 0) {
            throw "Host sensorless unit tests failed with exit code $LASTEXITCODE"
        }

        $phase46HostCommand = "cd /d `"$hostBuildDirectory`"" +
            " && call `"$vsDevCmd`" -no_logo" +
            " && cl /nologo /W4 /O2 /std:c11 /I`"$(Join-Path $moduleDirectory 'include')`"" +
            " `"$(Join-Path $testDirectory 'test_sensorless_phase46.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\MotorParamProfile.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\RotorFeedback.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\SensorlessStartup.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\SensorlessTakeoverProbe.c')`"" +
            " /Fe:`"$phase46HostOutput`"" +
            " && `"$phase46HostOutput`""

        & cmd.exe /d /s /c $phase46HostCommand
        if ($LASTEXITCODE -ne 0) {
            throw "Host phase-4-to-6 unit tests failed with exit code $LASTEXITCODE"
        }

        $productionHostCommand = "cd /d `"$hostBuildDirectory`"" +
            " && call `"$vsDevCmd`" -no_logo" +
            " && cl /nologo /W4 /O2 /std:c11 /I`"$stubDirectory`" /I`"$(Join-Path $moduleDirectory 'include')`"" +
            " `"$(Join-Path $testDirectory 'test_sensorless_production.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\RotorFeedback.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\SensorlessStartup.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\SensorlessTakeoverProbe.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\SensorlessProduction.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\ProductionMetrics.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\SensorlessSafety.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\HfiEstimator.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\MotorParamIdentification.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\SmoPll.c')`"" +
            " /Fe:`"$productionHostOutput`"" +
            " && `"$productionHostOutput`""

        & cmd.exe /d /s /c $productionHostCommand
        if ($LASTEXITCODE -ne 0) {
            throw "Host production-interface tests failed with exit code $LASTEXITCODE"
        }

        $hfiHostCommand = "cd /d `"$hostBuildDirectory`"" +
            " && call `"$vsDevCmd`" -no_logo" +
            " && cl /nologo /W4 /O2 /std:c11 /I`"$(Join-Path $moduleDirectory 'include')`"" +
            " `"$(Join-Path $testDirectory 'test_hfi_estimator.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\HfiEstimator.c')`"" +
            " /Fe:`"$hfiHostOutput`"" +
            " && `"$hfiHostOutput`""

        & cmd.exe /d /s /c $hfiHostCommand
        if ($LASTEXITCODE -ne 0) {
            throw "Host HFI estimator tests failed with exit code $LASTEXITCODE"
        }

        $identHostCommand = "cd /d `"$hostBuildDirectory`"" +
            " && call `"$vsDevCmd`" -no_logo" +
            " && cl /nologo /W4 /O2 /std:c11 /I`"$(Join-Path $moduleDirectory 'include')`"" +
            " `"$(Join-Path $testDirectory 'test_motor_param_identification.c')`"" +
            " `"$(Join-Path $moduleDirectory 'source\MotorParamIdentification.c')`"" +
            " /Fe:`"$identHostOutput`"" +
            " && `"$identHostOutput`""

        & cmd.exe /d /s /c $identHostCommand
        if ($LASTEXITCODE -ne 0) {
            throw "Host motor-parameter identification tests failed with exit code $LASTEXITCODE"
        }
    }
}

if ($RunSimulator) {
    Write-Warning 'The bundled GNU simulator does not implement the compressed RISC-V ISA used by this toolchain.'
    & $simulator $output
    if ($LASTEXITCODE -ne 0) {
        throw "Sensorless unit tests failed with exit code $LASTEXITCODE"
    }
}
