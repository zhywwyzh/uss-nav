#ifndef _MISSION_DATA_H_
#define _MISSION_DATA_H_

#include <Eigen/Eigen>
#include <unordered_map>
#include <vector>

using std::unordered_map;
using std::vector;
using Eigen::Vector3d;

namespace ego_planner {

enum MISSION_FSM_STATE
{
  INIT,
  WAIT_TRIGGER,
  WARM_UP,
  PLAN_EXPLORE,
  LLM_PLAN_EXPLORE,
  APPROACH_EXPLORE, //send goal frequently to followers
  PLAN_TRACK,
  APPROACH_TRACK,
  THINKING,
  YAW_HANDLE,
  FIND_TERMINATE_TARGET,
  FINISH,
  STOP,
  UNKONWN,
  GO_TARGET_OBJECT,
  GO_TARGET_WITH_WAYPOINT,
  DF_DEMO
};

struct MissionData 
{
  /*swarm info*/
  int                     drone_id_;
  int                     swarm_id_;
  vector<int>             swarm_mate_;
  bool                    is_leader_;
  bool                    is_follower_;
  bool                    is_initialized_;
  /*mission info*/
  MISSION_FSM_STATE       mission_state_;
  int                     instruction_;
  unordered_map<MISSION_FSM_STATE, std::string> state_str_;
};


}  // namespace ego_planner

#endif
