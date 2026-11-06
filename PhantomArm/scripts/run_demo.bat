@echo off
cd /d "%~dp0.."
".venv\Scripts\python.exe" -m phantomarm.app --mode demo --demo-pattern still
if errorlevel 1 echo ERROR: Read docs\TROUBLESHOOTING.md
pause
