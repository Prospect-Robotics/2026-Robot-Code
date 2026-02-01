# Documentation

## How to clear robot preferences.
1. SSH into the robot under lvuser
   * "lvuser@10.28.13.2" has no password.
1. Execute this command: <br>
<code>
\# Navigate to the home directory <br>
cd /home/lvuser <br>
\# Delete existing tables <br>
rm NetworkTable.json <br>
\# Reboot the robot <br>
reboot </code>
