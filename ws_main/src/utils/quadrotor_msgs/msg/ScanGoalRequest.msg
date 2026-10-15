# 扫描目标请求 (graph6 -> ego): 包裹既有 EgoGoalSet, 携带会话/请求关联与几何约束。
# 独立消息避免修改通用 EgoGoalSet (保持其 md5 不变)。

std_msgs/Header header          # world 坐标系

uint8 EXECUTE=1
uint8 CANCEL=2
uint8 command

string session_id
uint32 request_id
uint32 waypoint_id

uint8 GOAL_KIND_OBSERVATION=1
uint8 GOAL_KIND_RETURN=2
uint8 GOAL_KIND_HOVER=3
uint8 goal_kind

quadrotor_msgs/EgoGoalSet goal

geometry_msgs/Point[] candidates    # 原目标优先, 最多 9 个候选

bool has_boundary
geometry_msgs/Point boundary_min
geometry_msgs/Point boundary_max
geometry_msgs/Point object_min
geometry_msgs/Point object_max
float32 min_height
float32 min_clearance

float32 reference_azimuth
float32 azimuth_tolerance
float32 reference_height
float32 height_tolerance

float32 reach_tolerance
float32 odom_max_age
