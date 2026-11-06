@echo off
cd /d "%~dp0.."
".venv\Scripts\python.exe" -m phantomarm.app --mode demo --scene grasp --demo-pattern still
if errorlevel 1 echo GRASP SCENE ERROR: copy the full terminal output for troubleshooting.
pause
