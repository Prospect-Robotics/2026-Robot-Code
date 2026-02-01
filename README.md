# Documentation

## How to clear robot preferences.
1. SSH into the robot under lvuser.
   2. lvuser@10.28.13.2" has no password.
3. Navigate to "/home/lvuser"
   4. `cd /home/lvuser`
5. Delete "networktables.json"
   6. `rm networktables.json`
7. Reboot the roborio
   8. `reboot`