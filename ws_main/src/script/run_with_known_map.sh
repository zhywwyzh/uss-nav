ODOM_NAME=/ekf_quat/ekf_odom
echo "nv" | sudo chmod 777 /dev/tty*
#roscore & sleep 10;
roslaunch mavros px4.launch & sleep 4;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;   # ATTITUDE_QUATERNION
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;  # HIGHRES_IMU
rosrun mavros mavcmd long 511 83 5000 0 0 0 0 0 & sleep 1;   # ATTITUDE_TARGET
rosrun mavros mavcmd long 511 36 5000 0 0 0 0 0 & sleep 1;   # SERVO_OUTPUT_RAW
rosrun mavros mavcmd long 511 147 5000 0 0 0 0 0 & sleep 1;  # BATTERY_STATUS

# roslaunch fast_lio mapping_avia.launch & sleep 6; #init pos
# roslaunch fast_lio mapping_mid360.launch & sleep 10; 
roslaunch fast_lio initial_align.launch & sleep 1; 
roslaunch fast_lio odom_mid360_with_map.launch & sleep 1; 
roslaunch fast_lio fy_series_publisher.launch & sleep 1;
# roslaunch faster_lio odom_avia.launch & sleep 10;
# /home/nv/GNF_WS/src/map_trigger.sh & sleep 6;
roslaunch ekf_quat ekf_quat_lidar.launch & sleep 15;

touch yyyyyyyyyyy
roslaunch px4ctrl run_ctrl.launch odom_topic:=$ODOM_NAME & sleep 1;
bash /home/nv/GNF_WS/run_patrol_node.sh  & sleep 1;

touch zzzzzzzzzzzz
bash /home/nv/detection_ws/run_detect.sh & sleep 1;

# bash /home/nv/multi_tracking_real//record_light.sh
# mon launch ego_planner run_in_real_goal.launch odom_topic:=$ODOM_NAME & sleep 1;


# roslaunch nlink_parser linktrack.launch & sleep 1;
# roslaunch odom_correction uwb_correction.launch & sleep 1;

wait
