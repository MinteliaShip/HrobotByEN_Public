@echo off
rem ------------------------------------------------------------
rem  Build a stand-alone app (no Python needed on the other PC).
rem  Result: dist\RobotMotionStudio\  -> copy this whole folder.
rem  onedir (not onefile): avoids unpacking to %TEMP%, which can fail with a Japanese user name.
rem ------------------------------------------------------------
setlocal
cd /d "%~dp0"
set "PYTHONUTF8=1"

if not exist ".venv\Scripts\python.exe" (
    call "%~dp0setup.bat" auto
    if errorlevel 1 exit /b 1
)

".venv\Scripts\python.exe" -m pip install "pyinstaller>=6.3,<7"
if errorlevel 1 (
    set "M=24517,35201,12394,37096,21697,12398,12480,12454,12531,12525,12540,12489,12395,22833,25943,12375,12414,12375,12383,12290,10,10,12452,12531,12479,12540,12493,12483,12488,12395,25509,32154,12375,12390,12356,12427,12363,30906,35469,12375,12390,12289,12418,12358,19968,24230,32,114,117,110,46,98,97,116,32,12434,23455,34892,12375,12390,12367,12384,12373,12356,12290"
    call :msg
    exit /b 1
)

".venv\Scripts\python.exe" -m PyInstaller --noconfirm --clean --windowed --name RobotMotionStudio ^
    --add-data "index.html;." ^
    --hidden-import server ^
    --collect-all bleak ^
    --collect-submodules winrt --collect-all bleak_winrt ^
    main.py
if errorlevel 1 (
    echo [ERROR] Build failed. See the messages above.
    set "M=101,120,101,32,12398,20316,25104,12395,22833,25943,12375,12414,12375,12383,12290,34920,31034,12373,12428,12390,12356,12427,12513,12483,12475,12540,12472,12434,30906,35469,12375,12390,12367,12384,12373,12356,12290"
    call :msg
    pause
    exit /b 1
)

rem Take the current settings / pose library along (if they exist)
if exist pose_library.json copy /y pose_library.json dist\RobotMotionStudio\ >nul
if exist settings.json copy /y settings.json dist\RobotMotionStudio\ >nul
if exist README.txt copy /y README.txt dist\RobotMotionStudio\ >nul

echo.
echo Done: dist\RobotMotionStudio\RobotMotionStudio.exe
echo Copy the whole dist\RobotMotionStudio folder to the other PC.
pause
exit /b 0

:msg
rem  Show a Japanese message box. The text is passed as Unicode code points in %M% (keeps this file ASCII-only).
powershell -NoProfile -ExecutionPolicy Bypass -Command "Add-Type -AssemblyName System.Windows.Forms; [void][System.Windows.Forms.MessageBox]::Show((-join [char[]](%M%)), 'Robot Motion Studio')" >nul 2>nul
exit /b 0
