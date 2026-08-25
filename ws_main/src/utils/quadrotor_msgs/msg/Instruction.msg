# 总参数
uint8 robot_id
uint8[] to_drone_ids
std_msgs/Header header

# exploration部分参数
uint8 instruction_type
# 上层任务来源，用于区分相同instruction_type由哪个task_id发布
uint8 source_task_id
# 上层任务会话编号，用于隔离多次 counting 等异步任务的返回结果
uint32 task_session_id
# 本任务已排除的物体id列表(复核失败标记), FSM全等匹配时跳过这些物体
int32[] excluded_obj_ids

uint16 target_obj_id
string command
bool has_exploration_region
geometry_msgs/Point[] exploration_region
bool clear_local_map

# vla部分参数
geometry_msgs/Point[] goal
float32[] yaw
bool look_forward
bool goal_to_follower

# Tracking部分参数
bool enable
bool has_target_position
geometry_msgs/Point target_position

# 目标点导航
geometry_msgs/Point[] nav_waypoint
float32[] nav_yaw

sensor_msgs/Image image
string image_camera
int32[] classes
geometry_msgs/Point[] global_poses

# definations for instruction_type
## uss-nav exploration
uint8 TURN_OBJECT_NAV = 1               # 全功能
uint8 TURN_OBJECT_ID_NAV = 2            # 给定物体id导航
uint8 TURN_REGULAR_EXPLORATION = 3      # 纯建图探索
uint8 TURN_DF_DEMO = 4                  # demo演示，给prompt
uint8 TURN_WAYPOINT_NAV = 7             # 给定目标点导航

## navigation
uint8 TURN_GOAL = 5                     # vla的点到点导航

## tracking
uint8 TURN_TRACKING = 6                 # Tracking

## Map IO
string map_folder
uint8 TURN_SAVE_SCENE_GRAPH = 9
uint8 TURN_LOAD_SCENE_GRAPH = 10

## Scene Graph Skill IO
uint8 REQUEST_ALL_AREA_AND_OBJS = 11

## VLA_Swarm
uint8 TURN_VLA_SWARM = 12

## 停止当前运动并回到WAIT_TRIGGER(由agent_run在复核导航到达后下发, 无人机原地悬停)
uint8 STOP_MOTION = 13


# source_task_id definitions
uint8 SOURCE_TASK_EXPLORATION = 2
uint8 SOURCE_TASK_COUNTING = 8
uint8 SOURCE_TASK_VLA_SWARM = 9
