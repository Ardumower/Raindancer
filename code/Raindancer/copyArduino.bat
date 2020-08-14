@echo on
robocopy "D:\HiDrive\Github\Raindancer\code\Raindancer" "F:\Arduino\Version2\Raindancer" *.h
robocopy "D:\HiDrive\Github\Raindancer\code\Raindancer" "F:\Arduino\Version2\Raindancer" *.cpp
robocopy "D:\HiDrive\Github\Raindancer\code\Raindancer" "F:\Arduino\Version2\Raindancer" *.ino
# robocopy "D:\HiDrive\Github\Raindancer\code\Raindancer" "F:\Arduino\Version2\Raindancer" /LEV:0 
pause