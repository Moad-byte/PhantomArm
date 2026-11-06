@echo off
cd /d "%~dp0.."
".venv\Scripts\python.exe" -m phantomarm.app --mode replay --file recordings/sample_motion.jsonl
pause
