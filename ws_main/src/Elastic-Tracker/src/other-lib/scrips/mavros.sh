#!/bin/bash
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]:-${(%):-%x}}" )" >/dev/null 2>&1 && pwd )"
PASSWD="nv"
sudo chmod 777 /dev/tty*
roslaunch mavros apm.launch & sleep 5;

rosrun mavros mavsys message_interval --id=245  --rate=5;
rosrun mavros mavsys message_interval --id=147  --rate=5;
rosrun mavros mavsys message_interval --id=30   --rate=333;
rosrun mavros mavsys message_interval --id=27   --rate=333;
rosrun mavros mavsys message_interval --id=65   --rate=100;
rosrun mavros mavsys message_interval --id=11030 --rate=333;
rosrun mavros mavsys message_interval --id=36    --rate=333;
wait;
