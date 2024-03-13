#!/bin/bash
set +x
echo "Careful! This will delete the ENTIRE deploy folder. Are you SURE you want to continue?"
read -p "Press Enter to continue."
set -x
ssh admin@10.66.57.2 <<'ENDSSH'
set -x
cd ../
cd lvuser
rm -rf deploy
/usr/local/frc/bin/frcKillRobot.sh -r -t 
set +x
ENDSSH
set +x
echo "Cleared."
read -p "Reboot the rio? (y/n) " RESP
if [ "$RESP" = "y" ]; then
  ssh admin@10.66.57.2 reboot
else
  echo "Not rebooting the roboRIO. Please do it later!"
fi