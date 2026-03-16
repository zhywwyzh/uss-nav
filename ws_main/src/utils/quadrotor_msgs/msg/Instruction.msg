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

sensor_msgs/Image image
string image_camera
int32[] classes
geometry_msgs/Point[] global_poses

#definations for instruction_type
uint8 TURN_OBJECT_NAV = 1
uint8 TURN_OBJECT_ID_NAV = 2
uint8 TURN_REGULAR_EXPLORATION = 3
uint8 TURN_DF_DEMO = 4
uint8 TURN_GOAL = 5
uint8 TURN_TRACKING = 6
