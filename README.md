# Documentation

## How to clear robot preferences.
1. SSH into the robot under lvuser
   * "lvuser@10.28.13.2" has no password.
1. Navigate to "/home/lvuser"
   * `cd /home/lvuser`
1. Delete "networktables.json"
   * `rm networktables.json`
1. Reboot the roborio
   * `reboot`
