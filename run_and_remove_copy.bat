:: Run Xming before running this script
call docker run --rm --device="class/79bfeecb-947f-11e1-bc58-001b2163ef96" -it --name rtab -e DISPLAY=host.docker.internal:0.0 humble-rtabmap-pyphase
pause