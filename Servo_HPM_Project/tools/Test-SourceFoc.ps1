param([switch]$CompileAll, [switch]$LinkObjects)
$ErrorActionPreference = 'Stop'
if ($LinkObjects -and !$CompileAll) { throw 'Use -CompileAll with -LinkObjects to avoid stale inputs.' }
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskSes = Join-Path $taskRoot 'hpm6e00evk_flash_xip_debug/segger_embedded_studio'
$taskOutput = Join-Path $taskRoot 'diagnostics/source_foc_check'
New-Item -ItemType Directory -Force -Path $taskOutput | Out-Null
[xml]$taskXml = Get-Content -LiteralPath (Join-Path $taskSes 'Servo_HPM_Project.emProject') -Raw
$taskCfg = $taskXml.solution.project.configuration | Where-Object Name -eq Common
$taskArgs = @('-std=gnu11','-O2','-march=rv32imac_zba_zbb_zbc_zbs_zicsr_zifencei','-mabi=ilp32','-D_FOE_LDRPRM_BANK0_','-ffunction-sections','-fdata-sections','-Werror=implicit-function-declaration')
$taskArgs += ($taskCfg.c_user_include_directories.Split(';') | ForEach-Object { '-I' + [IO.Path]::GetFullPath((Join-Path $taskSes $_)) })
$taskArgs += $taskCfg.c_preprocessor_definitions.Split(';') | Where-Object { $_ } | ForEach-Object { '-D' + $_ }
$taskArgs += ($taskXml.solution.configuration | Where-Object Name -eq Common).c_preprocessor_definitions.Split(';') | Where-Object { $_ } | ForEach-Object { '-D' + $_ }
$taskGcc = 'D:/WORKING/sdk/toolchains/rv32imac_zicsr_zifencei_multilib_b_ext-win/bin/riscv32-unknown-elf-gcc.exe'
$taskFiles = @($taskXml.SelectNodes('//folder[@Name="30_SourceFoc"]/file') | Where-Object file_name -like '*.c' | ForEach-Object file_name)
$taskFiles += @('../../CtrlLoop/33_Sensorless/source/SensorlessShadow.c','../../CtrlLoop/33_Sensorless/source/SensorlessProduction.c','../../CtrlLoop/60_User/soure/CtrLoop.c')
if ($CompileAll) {
    $taskFiles = @($taskXml.SelectNodes('//file') | Where-Object {
        $_.file_name -like '*.c' -and -not $_.SelectSingleNode('ancestor::folder[configuration[@Name="BANK0" and @build_exclude_from_build="Yes"]]') -and
        -not $_.SelectSingleNode('configuration[@Name="BANK0" and @build_exclude_from_build="Yes"]') -and
        -not $_.SelectSingleNode('configuration[@Name="Common" and @build_exclude_from_build="Yes"]')
    } | ForEach-Object file_name | Select-Object -Unique)
}
$taskFailures = [Collections.Generic.List[string]]::new()
foreach ($taskFile in $taskFiles) {
    $taskFullPath = [IO.Path]::GetFullPath((Join-Path $taskSes $taskFile))
    $taskName = ($taskFile -replace '\.\./','' -replace '[\\/:]','_')
    $taskLog = Join-Path $taskOutput ($taskName + '.log')
    # SES excludes the GCC RTT syscall shim; do not impersonate the SES libc globally.
    $taskFileArgs = @()
    if ($taskFile -like '*SEGGER_RTT_Syscalls_GCC.c') { $taskFileArgs += '-D__SES_RISCV' }
    & $taskGcc @taskArgs @taskFileArgs -c $taskFullPath -o (Join-Path $taskOutput ($taskName + '.o')) 2> $taskLog
    if ($LASTEXITCODE -ne 0) { $taskFailures.Add($taskFile); Get-Content -LiteralPath $taskLog -Tail 18 }
}
Write-Output ('Compiled {0} files against real project headers; failures: {1}' -f $taskFiles.Count,$taskFailures.Count)
if ($taskFailures.Count) { throw ($taskFailures -join [Environment]::NewLine) }
$taskProbe = Join-Path $taskOutput 'abi_probe.o'
& $taskGcc @taskArgs -c (Join-Path $taskRoot 'CtrlLoop/33_Sensorless/tests/source_foc_abi_probe.c') -o $taskProbe 2> (Join-Path $taskOutput 'abi_probe.log')
if ($LASTEXITCODE) { throw 'ABI probe compile failed' }
$taskNm = Join-Path (Split-Path -Parent $taskGcc) 'riscv32-unknown-elf-nm.exe'
$taskOld = @{}
& $taskNm -S (Join-Path $taskRoot 'CtrlLoop/70_Obj/30_Drive/Drive_GlobalVariableDefs.o') | ForEach-Object {
    if ($_ -match '^\S+\s+(\S+)\s+\S\s+(\S+)$') { $taskOld[$Matches[2]] = $Matches[1] }
}
$taskAbiCount = 0
& $taskNm -S $taskProbe | ForEach-Object {
    if ($_ -match '^\S+\s+(\S+)\s+\S\s+abi_(\S+)$') {
        if ($taskOld[$Matches[2]] -ne $Matches[1]) { throw "ABI size mismatch: $($Matches[2])" }
        $taskAbiCount++
    }
}
if ($taskAbiCount -ne 11) { throw 'Incomplete ABI probe' }
Write-Output 'PASS: 11 shared object sizes match retained binary definitions (not a full field-offset proof).'
if ($LinkObjects) {
    $taskSelected = @($taskXml.SelectNodes('//file') | Where-Object {
        ($_.file_name -like '*.c' -or $_.file_name -like '*.o') -and
        -not $_.SelectSingleNode('ancestor::folder[configuration[(@Name="BANK0" or @Name="Common") and @build_exclude_from_build="Yes"]]') -and
        -not $_.SelectSingleNode('configuration[(@Name="BANK0" or @Name="Common") and @build_exclude_from_build="Yes"]')
    } | ForEach-Object file_name | Select-Object -Unique)
    $taskObjects = foreach ($taskFile in $taskSelected) {
        if ($taskFile -like '*.c') {
            Join-Path $taskOutput (($taskFile -replace '\.\./','' -replace '[\\/:]','_') + '.o')
        } else { [IO.Path]::GetFullPath((Join-Path $taskSes $taskFile)) }
    }
    foreach ($taskObject in $taskObjects) {
        if (!(Test-Path -LiteralPath $taskObject)) { throw "Missing object: $taskObject" }
    }
    $taskResponse = Join-Path $taskOutput 'objects.rsp'
    [IO.File]::WriteAllLines($taskResponse,@($taskObjects | ForEach-Object { '"' + ($_ -replace '\\','/') + '"' }),[Text.UTF8Encoding]::new($false))
    $taskLinker = Join-Path (Split-Path -Parent $taskGcc) 'riscv32-unknown-elf-ld.exe'
    $taskBundle = Join-Path $taskOutput 'application_bundle.o'
    & $taskLinker '-r' ('@' + $taskResponse) '-o' $taskBundle 2> (Join-Path $taskOutput 'relocatable_link.log')
    if ($LASTEXITCODE) { throw 'Relocatable link failed; see relocatable_link.log' }
    & $taskNm '-u' $taskBundle | Set-Content -LiteralPath (Join-Path $taskOutput 'unresolved_symbols.txt')
    Write-Output ('PASS: relocatable link of {0} objects; final startup/runtime/linker script still required.' -f $taskObjects.Count)
}
