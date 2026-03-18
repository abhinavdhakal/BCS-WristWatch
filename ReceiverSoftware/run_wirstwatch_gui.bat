@echo off
setlocal

cd /d "%~dp0"

echo [WirstWatch_BCS] Starting GUI...

if not exist ".venv\Scripts\python.exe" (
  echo [WirstWatch_BCS] Creating virtual environment...
  py -3.11 -m venv .venv
  if errorlevel 1 goto :fail
)

echo [WirstWatch_BCS] Installing/updating requirements...
".venv\Scripts\python.exe" -m pip install --upgrade pip >nul 2>&1
".venv\Scripts\python.exe" -m pip install -r requirements_gui.txt
if errorlevel 1 goto :fail

echo [WirstWatch_BCS] Launching app...
".venv\Scripts\python.exe" wirstwatch_gui.py
if errorlevel 1 goto :fail

goto :end

:fail
echo.
echo [WirstWatch_BCS] Launch failed. Check errors above.
pause

:end
endlocal
