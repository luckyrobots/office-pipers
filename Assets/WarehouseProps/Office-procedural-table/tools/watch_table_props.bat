@echo off
cd /d "%~dp0"
:loop
py relayout_props.py --watch --auto-seed
timeout /t 3 /nobreak >nul
goto loop
