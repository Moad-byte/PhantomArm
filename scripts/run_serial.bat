@echo off
cd /d "%~dp0.."
echo Connected serial ports:
".venv\Scripts\python.exe" -m phantomarm.list_ports
set /p PORT=Enter your Pico COM port (e.g. COM5):
".venv\Scripts\python.exe" -m phantomarm.app --mode serial --port %PORT% --record recordings/hardware_session.jsonl
if errorlevel 1 echo ERROR: Close Arduino Serial Monitor and recheck COM port.
pause
