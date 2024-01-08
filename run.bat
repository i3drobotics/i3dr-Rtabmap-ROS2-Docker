:: Run Xming before running this script
tasklist | find "Xming.exe" > nul
if errorlevel 1 (
    echo WARNING: Xming is not running, GUI programs may not work. ^
    Please run Xming first to use GUI applications in Docker.
)
call docker run -it --name rtab -e DISPLAY=host.docker.internal:0.0 humble-rtabmap-pyphase
pause