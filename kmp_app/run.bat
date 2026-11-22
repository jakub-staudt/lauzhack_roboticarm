@echo off
rem Windows launcher: starts the simulator and the eye tracker if they are not running, then the app.
rem Run from the kmp_app folder (or double-click). Needs JDK 17 (JAVA_HOME) and the Python environments:
rem   py -3.11 -m venv ..\.venv      &&  ..\.venv\Scripts\pip install -r ..\requirements.txt
rem   py -3.11 -m venv ..\.venv-sim  &&  ..\.venv-sim\Scripts\pip install -r ..\simulator\requirements.txt
setlocal
cd /d "%~dp0"
set ROOT=%~dp0..
if "%GAZE_CAMERA%"=="" set GAZE_CAMERA=0

curl -s -m 2 -o NUL http://127.0.0.1:8810/api/state
if errorlevel 1 (
  echo Starting the simulator...
  start "Simulator" /min /d "%ROOT%" "%ROOT%\.venv-sim\Scripts\python.exe" -m simulator.web.server
)
curl -s -m 1 -o NUL http://127.0.0.1:8820/api/gaze
if errorlevel 1 (
  echo Starting the eye tracker on camera %GAZE_CAMERA%...
  start "Eye tracker" /min /d "%ROOT%" "%ROOT%\.venv\Scripts\python.exe" -m gaze_service.server --camera %GAZE_CAMERA%
)
timeout /t 3 /nobreak >NUL
call gradlew.bat --no-daemon :composeApp:run
endlocal
