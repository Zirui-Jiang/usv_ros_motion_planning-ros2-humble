@echo off
setlocal
cd /d "%~dp0..\.."
python -B scripts\control\tune_speed.py
if errorlevel 1 (
  echo.
  echo If Python packages are missing, run:
  echo python -m pip install -r scripts\control\speed_tuner\requirements.txt
  echo For another port: python scripts\control\tune_speed.py --port 8768
  pause
)
