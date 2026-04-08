

#include <ros/ros.h>
#include <plan_manage/ego_replan_fsm.h>
#include <visualization_msgs/Marker.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
  // Set CPU affinity for thread: ego_planner [core 0]
  int core_id = 0;
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);
  if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
    std::cerr << "Failed to set CPU affinity for thread: planner "<< std::endl;
  }

  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");

  ROS_WARN_STREAM(">>>>>>>>>>>> EGO INIT >>>>>>>>>>>>");
  EGOReplanFSM ego_planner_fsm_;
  ego_planner_fsm_.init(nh);
  ROS_WARN_STREAM("<<<<<<<<<< EGO INIT DONE <<<<<<<<<");
  ros::spin();
  return 0;
}
