@echo off
cd /d "%~dp0"
:loop
py relayout_props.py --watch
timeout /t 3 /nobreak >nul
goto loop
