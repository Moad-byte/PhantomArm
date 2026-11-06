@echo off
cd /d "%~dp0.."
".venv\Scripts\python.exe" -m pytest -v tests
if errorlevel 1 goto fail
".venv\Scripts\python.exe" -m phantomarm.verify_model
if errorlevel 1 goto fail
echo UNIT AND STRUCTURAL TESTS PASSED. This does NOT confirm a successful cube grasp.
echo For physical grasp validation, run scripts\run_grasp_diagnostic.bat.
pause
exit /b 0
:fail
echo TEST FAILURE. Do not claim validation passed until fixed.
pause
exit /b 1
