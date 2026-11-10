@echo off
rem ------------------------------------------------------------
rem  Start Robot Motion Studio (local server + Edge/Chrome app window).
rem  The first run on a PC sets up .venv automatically.
rem ------------------------------------------------------------
setlocal
cd /d "%~dp0"
set "PYTHONUTF8=1"

rem Started from inside a zip (Explorer extracts it to a Temp1_xxx.zip folder)?
setlocal EnableDelayedExpansion
set "D=%~dp0"
set "Z=0"
if /i not "!D:.zip\=!"=="!D!" set "Z=1"
if /i not "!D:\Temp1_=!"=="!D!" set "Z=1"
endlocal & set "Z=%Z%"
if "%Z%"=="1" (
    set "M=122,105,112,32,12501,12449,12452,12523,12398,20013,12363,12425,30452,25509,36215,21205,12373,12428,12390,12356,12414,12377,12290,10,10,122,105,112,32,12434,21491,12463,12522,12483,12463,32,8594,12300,12377,12409,12390,23637,38283,12301,12391,23637,38283,12375,12390,12363,12425,12289,23637,38283,12375,12383,12501,12457,12523,12480,12398,20013,12398,32,114,117,110,46,98,97,116,32,12434,23455,34892,12375,12390,12367,12384,12373,12356,12290"
    call :msg
    exit /b 1
)
if not exist "main.py" (
    set "M=109,97,105,110,46,112,121,32,12364,35211,12388,12363,12426,12414,12379,12435,12290,10,10,12501,12457,12523,12480,12398,20013,36523,12434,12377,12409,12390,12381,12429,12360,12390,12363,12425,12289,12418,12358,19968,24230,23455,34892,12375,12390,12367,12384,12373,12356,12290"
    call :msg
    exit /b 1
)

if not exist ".venv\Scripts\pythonw.exe" (
    echo First run on this PC: setting up. This takes a few minutes...
    call "%~dp0setup.bat" auto
    if errorlevel 1 exit /b 1
)

rem pythonw = no console window. Errors are shown in a message box and written to robot_motion_studio.log
start "" ".venv\Scripts\pythonw.exe" "%~dp0main.py"
exit /b 0

:msg
rem  Show a Japanese message box. The text is passed as Unicode code points in %M% (keeps this file ASCII-only).
powershell -NoProfile -ExecutionPolicy Bypass -Command "Add-Type -AssemblyName System.Windows.Forms; [void][System.Windows.Forms.MessageBox]::Show((-join [char[]](%M%)), 'Robot Motion Studio')" >nul 2>nul
exit /b 0
