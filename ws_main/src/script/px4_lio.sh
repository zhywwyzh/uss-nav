echo "nv" | sudo chmod 777 /dev/tty*
#roscore & sleep 10;
roslaunch mavros px4.launch & sleep 10;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;     # ATTITUDE_QUATERNION
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;    # HIGHRES_IMU
rosrun mavros mavcmd long 511 83 5000 0 0 0 0 0 & sleep 1;   # ATTITUDE_TARGET
rosrun mavros mavcmd long 511 36 5000 0 0 0 0 0 & sleep 1;   # SERVO_OUTPUT_RAW
rosrun mavros mavcmd long 511 147 5000 0 0 0 0 0 & sleep 1;  # BATTERY_STATUS

roslaunch fast_lio mapping_mid360.launch & sleep 6; #init pos

roslaunch ekf_quat ekf_quat_lidar.launch & sleep 1;

wait
