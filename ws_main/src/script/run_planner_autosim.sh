#! /bin/bash
# 使用 rosrun plotjuggler plotjuggler 的republish功能，可拖动进度条回放bag。

# 设置循环次数
max_iterations=10

# 初始化循环计数器
iteration_count=0

while [ $iteration_count -lt $max_iterations ]
do

    timestamp=$(date +%Y%m%d_%H%M%S)
    output_file="${iteration_count}_output_${timestamp}.txt"
    bag_filename="${iteration_count}_bag_${timestamp}.bag"

    roslaunch ego_planner rviz.launch & sleep 2;
    roslaunch ego_planner swarm_fy_multidrone_sim.launch &>"${output_file}" & sleep 10;


    if [ -z "$DRONE_ID" ]; then
        echo "DRONE_ID is not set"
        DRONE_ID=0
    fi

    rosbag record --tcpnodelay -O $bag_filename \
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
    /drone_${DRONE_ID}_ego_planner_node/keyedges &
    sleep 2;

    rostopic pub /bridge/Instruct quadrotor_msgs/Instruction "{robot_id: 0, instruction_type: 2}" & 
    
    sleep 500;


    bash src/script/killros_soft.sh & sleep 5;


    # 增加循环计数器
    iteration_count=$((iteration_count + 1))

    # 检查是否达到最大循环次数
    if [ $iteration_count -ge $max_iterations ]; then
        echo "已达到最大循环次数，退出脚本。"
        break
    fi
done

wait

