Header header
#  --- only support single pose graph for now --- #
geometry_msgs/Point[] key_pose_list_xyz
float32[] key_pose_list_intensity

# p_start no need to tans, because it is bind with keypoint sequences
std_msgs/UInt16MultiArray[] pose_edge_p_end
std_msgs/Float32MultiArray[] pose_edge_weight
