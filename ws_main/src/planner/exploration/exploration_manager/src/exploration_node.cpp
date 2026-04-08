#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <plan_manage/ego_replan_fsm.h>
#include <exploration_manager/fast_exploration_fsm.h>

#include <memory>

using namespace ego_planner;

int main(int argc, char **argv)
{
  // int core_id = 4;
  // cpu_set_t cpuset;
  // CPU_ZERO(&cpuset);
  // CPU_SET(core_id, &cpuset);
  // if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1)
  // {
  //     std::cerr << "Failed to set CPU affinity for thread: px4ctrl "<< std::endl;
  // }
  // else
  // {
  //     std::cout << "Successfully set CPU affinity to core " << core_id << std::endl;
  // }

  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");

  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  ROS_INFO_STREAM("Init!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

  EGOReplanFSM rebo_replan;
  rebo_replan.init(nh);
  ROS_INFO_STREAM("FSM init done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");


  MapInterface::Ptr map_;
  map_ = std::make_shared<MapInterface>(nh, rebo_replan.getMapPtr());

  ROS_INFO_STREAM("map init done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

  FastExplorationFSM expl_fsm;
  expl_fsm.init(nh, map_);
  ROS_INFO_STREAM("EXP-FSM init done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

  ros::spin();
  // ros::waitForShutdown();
  return 0;
}
