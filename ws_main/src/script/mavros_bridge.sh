#ODOM_NAME=/ekf_quat/ekf_odom
echo "nv" | sudo chmod 777 /dev/tty*
#roscore & sleep 10;
roslaunch mavros px4.launch gcs_url:="udp://@10.1.1.29" & sleep 6;
#rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;   # ATTITUDE_QUATERNION
#rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;  # HIGHRES_IMU
#rosrun mavros mavcmd long 511 83 5000 0 0 0 0 0 & sleep 1;   # ATTITUDE_TARGET
#rosrun mavros mavcmd long 511 36 5000 0 0 0 0 0 & sleep 1;   # SERVO_OUTPUT_RAW
#rosrun mavros mavcmd long 511 147 5000 0 0 0 0 0 & sleep 1;  # BATTERY_STATUS

#roslaunch fast_lio mapping_avia.launch & sleep 6; #init pos
# roslaunch faster_lio mapping_avia.launch & sleep 10;
# ./map_trigger.sh & sleep 6;
#roslaunch ekf_quat ekf_quat_lidar.launch & sleep 1;
# roslaunch faster_lio fake_ekf.launch & sleep 1;

#roslaunch realsense2_camera rs_camera.launch & sleep 5;

#roslaunch px4ctrl run_ctrl.launch odom_topic:=$ODOM_NAME & sleep 1;

#roslaunch ego_planner tcp_exp_real1.launch odom_topic:=$ODOM_NAME & sleep 1;
# roslaunch ego_planner tcp_exp_real5.launch odom_topic:=$ODOM_NAME & sleep 1;

# roslaunch ego_planner run_in_exp_3.launch odom_topic:=$ODOM_NAME & sleep 1;

# roslaunch nlink_parser linktrack.launch & sleep 1;
# roslaunch odom_correction uwb_correction.launch & sleep 1;

wait
