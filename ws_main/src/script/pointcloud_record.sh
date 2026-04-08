rosbag record --tcpnodelay \
/livox/imu \
/livox/lidar \
/mavros/imu/data \
# /Odometry \
# /ekf/ekf_odom \
# /mavros/setpoint_raw/attitude \
# /mavros/battery \
# /traj_start_trigger \
# /debugPx4ctrl \
# /mavros/rc/in \
# /mavros/extended_state \
# /mavros/rc/out \
# /mavros/state \
# /position_cmd \
# /px4ctrl/takeoff_land \
# /traj_server/trajserver_debug \
# /vrpn_client_node/flipdrone/pose \

wait;