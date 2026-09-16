@echo off
setlocal

rem Always clean relative to this batch file, regardless of its launch directory.
pushd "%~dp0" || exit /b 1

call :clean_pattern "*.bak"
call :clean_pattern "*.ddk"
call :clean_pattern "*.edk"
call :clean_pattern "*.lst"
call :clean_pattern "*.lnp"
call :clean_pattern "*.mpf"
call :clean_pattern "*.mpj"
call :clean_pattern "*.obj"
call :clean_pattern "*.omf"
rem Keep *.opt because it may contain J-Link settings.
call :clean_pattern "*.plg"
call :clean_pattern "*.rpt"
call :clean_pattern "*.tmp"
call :clean_pattern "*.__i"
call :clean_pattern "*.crf"
call :clean_pattern "*.o"
call :clean_pattern "*.d"
call :clean_pattern "*.axf"
call :clean_pattern "*.tra"
call :clean_pattern "*.dep"
call :clean_pattern "JLinkLog.txt"

call :clean_pattern "*.iex"
call :clean_pattern "*.htm"
call :clean_pattern "*.sct"
call :clean_pattern "*.map"

call :clean_pattern "*.dbgconf"
call :clean_pattern "*.LINGZHUNING"
call :clean_pattern "*.Administrator"

rem Clean all SES build products, but keep every compile_commands.json.
set "SES_OUTPUT=%~dp0Projects\SES_Prj\Output"
if exist "%SES_OUTPUT%\" (
    for /r "%SES_OUTPUT%" %%F in (*) do (
        if /i not "%%~nxF"=="compile_commands.json" del /f /q "%%F" 2>nul
    )

    rem Remove directories that became empty; paths containing the JSON stay intact.
    for /f "delims=" %%D in ('dir /ad /b /s "%SES_OUTPUT%" 2^>nul ^| sort /r') do rd "%%D" 2>nul
)

popd
endlocal
exit /b 0

rem Delete a matching file type from the project root and every top-level
rem directory except CtrlLoop. Nothing below CtrlLoop may be removed here.
:clean_pattern
del /f /q "%~1" 2>nul
for /d %%D in (*) do (
    if /i not "%%~nxD"=="CtrlLoop" del /f /q "%%D\%~1" /s 2>nul
)
exit /b 0
