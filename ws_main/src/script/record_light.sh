rosbag record --tcpnodelay \
/traj_start_trigger \
/sdf_map/occupancy_local_${DRONE_ID} \
/quad_${DRONE_ID}_ego_planner_node/failed_list \
/quad_${DRONE_ID}_ego_planner_node/goal_point \
/quad_${DRONE_ID}_ego_planner_node/init_list \
/quad_${DRONE_ID}_ego_planner_node/init_of_init_list \
/quad_${DRONE_ID}_ego_planner_node/optimal_list \
/quad_${DRONE_ID}_ego_planner_node/global_list \
/quad_${DRONE_ID}_ego_planner_node/TopoMapvis \
/quad${DRONE_ID}_traj_server/patrol/fov \
/key_waypts_node/wayPts \
/debugPx4ctrl \
/Odometry \
/ekf_quat/ekf_odom \
/faster_lio/ekf_odom \
/mavros/imu/data_raw \
/mavros/imu/data \
/livox/imu \
/state_to_steamdeck \
/px4ctrl/takeoff_land \
/others_traj_odom \
/others_odom \
/livox/lidar \
/cpu_usage \
/LioDebug \
/usb_cam/image_raw \
/usb_cam1/image_raw \
/usb_cam2/image_raw \
/usb_cam3/image_raw \
/cloud_registered_body \
/bridge/distance_meas \
/bridge/bearing_meas \
/bridge/broadcast_odom_from_planner \
/drift_to_edges
# /detection/usb_cam/annotated_image \
# /detection/usb_cam/bbox_info \
# /detection/usb_cam1/annotated_image \
# /detection/usb_cam1/bbox_info \
# /detection/usb_cam2/annotated_image \
# /detection/usb_cam2/bbox_info \
# /detection/usb_cam3/annotated_image \
# /detection/usb_cam3/bbox_info \
# /target_dist_calculator/detect_out
#/drone_${DRONE_ID}_ego_planner_node/grid_map/occupancy
# /livox/lidar \
#/camera/infra1/image_rect_raw \
#/camera/infra2/image_rect_raw \

wait;
