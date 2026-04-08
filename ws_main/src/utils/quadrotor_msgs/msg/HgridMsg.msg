Header header
bool recv

# every single grid data (GridInfo)
uint16[] id
uint16[] local_id
uint16[] unknown_num  #
uint16[] frontier_num # no use
geometry_msgs/Point[] center
std_msgs/UInt16MultiArray[] frontier_cell_nums
std_msgs/UInt16MultiArray[] contained_frontier_ids
bool[] is_updated
bool[] need_divide
bool[] active
bool[] is_prev_relevant
bool[] is_cur_relevant
bool[] is_covered

# multy grid data (UniformGrid)
std_msgs/UInt16MultiArray relevant_id
std_msgs/UInt16MultiArray relevant_map
geometry_msgs/Point min
geometry_msgs/Point max
geometry_msgs/Point resolution
float32 grid_size
bool multi_layer_hgrid

