#! /bin/bash
if [ -z "$DRONE_ID" ]; then
    echo "DRONE_ID is not set"
    DRONE_ID=0
fi

rosbag record --tcpnodelay \
/drone_${DRONE_ID}_ego_planner_node/grid_map/occupancy_inflate \
/drone_${DRONE_ID}_ego_planner_node/frontier \
/drone_${DRONE_ID}_ego_planner_node/frontier_inx \
/drone_${DRONE_ID}_ego_planner_node/frontier_info \
/drone_${DRONE_ID}_ego_planner_node/fc_cenection \
/drone_${DRONE_ID}_ego_planner_node/ftr_blacklist \
/drone_${DRONE_ID}_ego_planner_node/hgrid_info \
/drone_${DRONE_ID}_ego_planner_node/relevent \
/drone_${DRONE_ID}_ego_planner_node/hgrid_text \
/drone_${DRONE_ID}_ego_planner_node/hgrid \
/drone_${DRONE_ID}_ego_planner_node/viewpoints \
/drone_${DRONE_ID}_ego_planner_node/viewpoints_yaw \
/drone_${DRONE_ID}_ego_planner_node/viewpoints_line \
/drone_${DRONE_ID}_ego_planner_node/planning/fsm_vis \
/drone_${DRONE_ID}_ego_planner_node/planning/position_cmd_vis \
/drone_${DRONE_ID}_odom_visualization/robot \
/drone_${DRONE_ID}_ego_planner_node/optimal_list \
/drone_${DRONE_ID}_ego_planner_node/init_list \
/drone_${DRONE_ID}_ego_planner_node/failed_list \
/drone_${DRONE_ID}_ego_planner_node/path_next_goal_pt \
/drone_${DRONE_ID}_ego_planner_node/keyposes \
/drone_${DRONE_ID}_ego_planner_node/keyedges \
/ekf_quat/ekf_odom \
wait;
