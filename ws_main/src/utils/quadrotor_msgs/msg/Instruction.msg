# 总参数
uint8 robot_id
uint8[] to_drone_ids
std_msgs/Header header

# exploration部分参数
uint8 instruction_type

uint16 target_obj_id
string command

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

#definations for instruction_type
uint8 TURN_OBJECT_NAV = 1               # 全功能
uint8 TURN_OBJECT_ID_NAV = 2            # 给定物体id导航
uint8 TURN_REGULAR_EXPLORATION = 3      # 纯建图探索
uint8 TURN_DF_DEMO = 4                  # demo演示，给prompt
uint8 TURN_GOAL = 5                     # vla的点到点导航
uint8 TURN_TRACKING = 6                 # Tracking
uint8 TURN_WAYPOINT_NAV = 7             # 给定目标点导航


# Map IO
string map_folder
uint8 TURN_SAVE_SCENE_GRAPH = 9
uint8 TURN_LOAD_SCENE_GRAPH = 10

# Scene Graph Skill IO
uint8 REQUEST_ALL_AREA_AND_OBJS = 11