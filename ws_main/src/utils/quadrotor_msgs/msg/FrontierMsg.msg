Header header
# -----------------------
FtrPointArray[] cells
FtrPointArray[] filtered_cells
geometry_msgs/Point[] average
geometry_msgs/Point[] normal
uint16[] id
uint16[] keypose_idx
# view points for each frontier
FtrPointArray[] viewpoints_pos
std_msgs/Float32MultiArray[] viewpoints_yaw
std_msgs/UInt16MultiArray[] viewpoints_visib_num
geometry_msgs/Point[] box_min_
geometry_msgs/Point[] box_max_
# path & costs between frontiers
FtrPathArray[] paths
std_msgs/Float32MultiArray[] costs
FtrPointArray[] path_to_home_3   # (x, y, z, state)
std_msgs/Float32MultiArray[] path_to_home_4
float32[] cost_to_home# (x, y, z, state)
std_msgs/UInt16MultiArray[] topo_blacklist
# frontier blacklist (can't reach)
geometry_msgs/Point[] ftr_blacklist