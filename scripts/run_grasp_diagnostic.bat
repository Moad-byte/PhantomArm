@echo off
cd /d "%~dp0.."
".venv\Scripts\python.exe" -m phantomarm.grasp_diagnostic
set "RESULT=%ERRORLEVEL%"
echo.
if not "%RESULT%"=="0" echo PHYSICS GATE FAILED - do not claim grasping is working.
if "%RESULT%"=="0" echo PHYSICS GATE PASSED - verify with manual grasping as well.
pause
exit /b %RESULT%
