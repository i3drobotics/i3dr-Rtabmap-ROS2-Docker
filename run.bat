:: Run Xming before running this script
call docker run -it --name rtab -e DISPLAY=host.docker.internal:0.0 humble-rtabmap-pyphase
pause