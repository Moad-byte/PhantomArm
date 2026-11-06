@echo off
cd /d "%~dp0.."
".venv\Scripts\python.exe" -m phantomarm.verify_model
if errorlevel 1 goto fail
".venv\Scripts\python.exe" -m phantomarm.app --mode demo --headless --duration 5
if errorlevel 1 goto fail
pause
exit /b 0
:fail
echo HEADLESS CHECK FAILED.
pause
exit /b 1
