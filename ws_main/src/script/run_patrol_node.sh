RED='\033[0;31m'
NC='\033[0m' 
ODOM_NAME=/ekf_quat/ekf_odom
GOAL="goal" 
TIMESTAMP=$(date +"%Y%m%d-%H%M%S")
OUTPUT_FILE="${TIMESTAMP}-${GOAL}.txt"

# 目标ROS节点名称
TARGET_NODE="/quad_${DRONE_ID}_ego_planner_node"

# 检查节点是否已经存在
if rosnode list | grep -q "$TARGET_NODE"; then
  echo -e "${RED}检测到节点 $TARGET_NODE 已经存在，后续命令不执行。${NC}"
  exit 0
else
  echo -e "${RED}未检测到节点 $TARGET_NODE，继续执行后续命令。${NC}"
fi


touch xxxxxxxxxx

echo -e "${RED}准备运行roslaunch run_in_real_goal.launch ...${NC}"
# roslaunch ego_planner run_in_real_goal.launch odom_topic:=$ODOM_NAME > "$OUTPUT_FILE" & sleep 1;

roslaunch ego_planner run_in_real_goal.launch odom_topic:=$ODOM_NAME & sleep 1;

RETURN_CODE=$?
if [ $RETURN_CODE -ne 0 ]; then
  echo -e "${RED}roslaunch命令执行失败，返回码为：$RETURN_CODE${NC}"
else
  echo -e "${RED}roslaunch命令执行成功${NC}"
fi
echo -e "${RED}roslaunch命令已结束${NC}"
echo "返回码：$RETURN_CODE"
wait
