rosbag record --tcpnodelay \
/traj_start_trigger \
/sdf_map/occupancy_local_${DRONE_ID} \
/sdf_map/occupancy_all_${DRONE_ID} \
/quad_${DRONE_ID}_ego_planner_node/failed_list \
/quad_${DRONE_ID}_ego_planner_node/goal_point \
/quad_${DRONE_ID}_ego_planner_node/init_list \
/quad_${DRONE_ID}_ego_planner_node/init_of_init_list \
/quad_${DRONE_ID}_ego_planner_node/optimal_list \
/quad_${DRONE_ID}_ego_planner_node/global_list \
/key_waypts_node/wayPts \
/debugPx4ctrl \
/Odometry_map \
/ekf_quat/ekf_odom \
/faster_lio/ekf_odom \
/mavros/imu/data_raw \
/mavros/imu/data \
/livox/imu \
/mavros/battery \
/mavros/setpoint_raw/target_attitude \
/mavros/setpoint_raw/attitude \
/state_to_steamdeck \
/setpoints_cmd \
/mavros/rc/in \
/mavros/rc/out \
/mavros/extended_state \
/mavros/state \
/px4ctrl/takeoff_land \
/cpu_usage \
/LioDebug
# /livox/lidar \
#/camera/infra1/image_rect_raw \
#/camera/infra2/image_rect_raw \

wait;
