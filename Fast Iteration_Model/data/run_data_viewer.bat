@echo off
cd /d "%~dp0"
set "BUNDLED_PY=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"

if exist "%BUNDLED_PY%" (
    "%BUNDLED_PY%" data_viewer.py
    goto end
)

python data_viewer.py
if not errorlevel 1 goto end

py data_viewer.py

:end
pause
