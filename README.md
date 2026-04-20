# Documentation

## How to clear robot preferences.
1. SSH into the robot under lvuser
   * "lvuser@10.28.13.2" has no password.
1. Execute this command

```
# Navigate to the home directory
cd /home/lvuser
# Delete existing table
rm NetworkTable.json
# Reboot the robot
reboot
```
