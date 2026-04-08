# roslaunch fast_lio mapping_avia.launch & sleep 6; #init pos
roslaunch faster_lio odom_avia_debug.launch & sleep 10;
/home/nv/GNF_WS/src/map_trigger.sh & sleep 6;
roslaunch ekf_quat ekf_quat_lidar.launch & sleep 1;
# roslaunch odom_correction uwb_correction.launch & sleep 1;

wait
