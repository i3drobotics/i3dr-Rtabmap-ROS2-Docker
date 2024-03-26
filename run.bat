@echo off

:: Run Xming before running this script
tasklist | find "Xming.exe" > nul
if errorlevel 1 (
    echo WARNING: Xming is not running, GUI programs may not work.
    echo Please run Xming first to use GUI applications in Docker.
)

:: Prompt for --rm flag
set /p rmflag="Do you want to use the --rm flag? (Y/N): "
if /i "%rmflag%"=="Y" (
    set rm="--rm"
) else (
    set rm=""
)

:: Run Docker container
call docker run -it %rm% --name frtab --gpus all --hostname I3DRWL004 foxy-rtabmap-pyphase