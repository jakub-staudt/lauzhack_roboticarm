@echo off
rem Laptop side of the Android app on Windows: simulator + eye tracker for the phone over Tailscale.
rem Keep this window open and type the address it prints into the phone app.
setlocal
cd /d "%~dp0.."
for /f "usebackq delims=" %%i in (`"C:\Program Files\Tailscale\tailscale.exe" ip -4`) do if not defined IP set IP=%%i
if "%IP%"=="" ( echo Tailscale is not running on this laptop: start it and sign in, then run this again. & pause & exit /b 1 )
start "Simulator" /min .venv-sim\Scripts\python.exe -m simulator.web.server --host 0.0.0.0 --port 8810
start "Eye tracker (phone)" /min .venv\Scripts\python.exe -m gaze_service.server --phone --host 0.0.0.0 --port 8820
echo.
echo   In the phone app, enter this laptop address:   %IP%
echo   (close the "Simulator" and "Eye tracker" windows to stop)
echo.
pause
endlocal
