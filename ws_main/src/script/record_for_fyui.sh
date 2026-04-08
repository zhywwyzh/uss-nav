rosbag record --tcpnodelay \
/rolling_map_publisher/communication/grid_PC \
/ekf_quat/ekf_odom
# /livox/lidar \
#/camera/infra1/image_rect_raw \
#/camera/infra2/image_rect_raw \

wait;
