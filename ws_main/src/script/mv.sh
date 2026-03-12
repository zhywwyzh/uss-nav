sudo chmod 777 /dev/tty*
#roscore & sleep 10;
roslaunch mavros px4.launch & sleep 6;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;   # ATTITUDE_QUATERNION
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;  # HIGHRES_IMU
rosrun mavros mavcmd long 511 83 5000 0 0 0 0 0 & sleep 1;   # ATTITUDE_TARGET
rosrun mavros mavcmd long 511 147 5000 0 0 0 0 0 & sleep 1;  # BATTERY_STATUS

# roslaunch mavros apm.launch & sleep 5;
# rosrun mavros mavsys message_interval --id=245  --rate=5;
# rosrun mavros mavsys message_interval --id=147  --rate=5;
# rosrun mavros mavsys message_interval --id=30   --rate=333;
# rosrun mavros mavsys message_interval --id=27   --rate=333;
# rosrun mavros mavsys message_interval --id=65   --rate=100;
# rosrun mavros mavsys message_interval --id=11030 --rate=333;
# rosrun mavros mavsys message_interval --id=36    --rate=333;

#roslaunch vrpn_client_ros sample.launch server:=10.1.1.198 &sleep 3;
#roslaunch ekf nokov.launch & sleep 3;
#roslaunch odom_visualization odom_visualization.launch & sleep 2;
#roslaunch px4ctrl run_ctrl.launch & sleep 2;
#roslaunch traj_server traj_server.launch & sleep 3;
#roslaunch traj_server traj_server.launch & sleep 3;
wait;
