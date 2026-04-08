rosbag record --tcpnodelay \
/Odometry_map \
/mavros/imu/data_raw \
/mavros/imu/data \
/livox/imu \
/livox/lidar

wait;
