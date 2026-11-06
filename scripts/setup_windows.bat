@echo off
setlocal
cd /d "%~dp0.."
echo [1/4] Creating Python 3.11 virtual environment...
py -3.11 -m venv .venv
if errorlevel 1 goto fail
echo [2/4] Updating pip...
".venv\Scripts\python.exe" -m pip install --upgrade pip
if errorlevel 1 goto fail
echo [3/4] Installing Windows binary packages...
".venv\Scripts\python.exe" -m pip install --only-binary=:all: -r requirements.txt
if errorlevel 1 goto fail
echo [4/4] Checking installed dependencies...
".venv\Scripts\python.exe" -m pip check
if errorlevel 1 goto fail
echo.
echo SETUP SUCCESS. Next: scripts\test_all.bat
pause
exit /b 0
:fail
echo.
echo SETUP FAILED. Read docs\WINDOWS_SETUP.md and correct the error above.
pause
exit /b 1
