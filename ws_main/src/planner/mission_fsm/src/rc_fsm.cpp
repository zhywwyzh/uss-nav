//
// Created by gwq on 11/26/24.
//
#include "rc_fsm.h"

#include <memory>

RCCtrlFSM::RCCtrlFSM(ros::NodeHandle &node, MapInterface::Ptr & map_interface) {
  INFO_MSG_GREEN("[RC-FSM]: initializing...");
  // initialize state machine
  data_           = std::make_shared<RCCtrlFSMData>();
  params_         = std::make_shared<RCCtrlFSMParams>();
  map_interface_  = map_interface;
  scene_graph_    = std::make_shared<SceneGraph>(node, map_interface_);

  // get parameters from parameter server
  readParam(node, "rc_ctrl/odom_topic", params_->odom_sub_topic_, std::string("/odom"));
  readParam(node, "rc_ctrl/rc_topic", params_->rc_sub_topic_, std::string("/rc_raw"));
  readParam(node, "rc_ctrl/px4ctrl_cmd_topic", params_->px4ctrl_cmd_topic_, std::string("/px4ctrl_cmd"));
  readParam(node, "rc_ctrl/loop_rate", params_->fsm_exec_freq_, 100.0);
  readParam(node, "rc_ctrl/max_vel", params_->max_speed_, 1.0);
  readParam(node, "rc_ctrl/max_acc", params_->max_acc_, 1.0);
  readParam(node, "rc_ctrl/max_yaw_rate", params_->max_yaw_rate_, 20.0 * M_PI / 180.0);
  readParam(node, "rc_ctrl/rc_timeout", params_->rc_timeout_, 0.1);
  readParam(node, "rc_ctrl/odom_timeout", params_->odom_timeout_, 0.1);

  readParam(node, "rc_ctrl/rc_reverse_pitch", params_->rc_reverse_pitch_, false);
  readParam(node, "rc_ctrl/rc_reverse_roll", params_->rc_reverse_roll_, false);
  readParam(node, "rc_ctrl/rc_reverse_yaw", params_->rc_reverse_yaw_, false);
  readParam(node, "rc_ctrl/rc_reverse_throttle", params_->rc_reverse_thrust_, false);

  // ros init
  fsm_exec_timer_   = node.createTimer(ros::Duration(1.0 / params_->fsm_exec_freq_), &RCCtrlFSM::fsmExecTimerCallback, this);
  fsm_vis_timer_    = node.createTimer(ros::Duration(1.0 / 25.0), &RCCtrlFSM::fsmVisTimerCallback, this);
  skeleton_mount_point_timer_ = node.createTimer(ros::Duration(1.0 / 5.0), &RCCtrlFSM::skeletonMountPointTimerCallback, this);

  odom_sub_         = node.subscribe<nav_msgs::Odometry>(params_->odom_sub_topic_, 50,
                                                  boost::bind(&Odom_Data_t::feed, &data_->odom_data_, _1),
                                                  ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay(true));
  rc_sub_           = node.subscribe<mavros_msgs::RCIn>(params_->rc_sub_topic_, 10,
                                                  boost::bind(&RC_Data_t::feed, &data_->rc_data_, _1),
                                                  ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay(true));
  px4_state_sub_    = node.subscribe<mavros_msgs::State>("/mavros/state", 10,
                                                  boost::bind(&RC_Data_t::feedMavrosState, &data_->rc_data_, _1),
                                                  ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay(true));
  goal_from_station_sub_ = node.subscribe<quadrotor_msgs::GoalSet>("/goal_with_id_from_station", 1,
                                                  boost::bind(&RCCtrlFSM::goalFromStationCallback, this, _1), ros::VoidConstPtr(),
                                                  ros::TransportHints().tcpNoDelay(true));

  px4ctrl_cmd_pub_          = node.advertise<quadrotor_msgs::PositionCommand>(params_->px4ctrl_cmd_topic_, 50);
  px4ctrl_takeoff_land_pub_ = node.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);
  rc_ctrl_marker_pub_       = node.advertise<visualization_msgs::MarkerArray>("rc_ctrl_marker", 10);
  ego_goal_set_pub_         = node.advertise<quadrotor_msgs::EgoGoalSet>("local_goal", 1);
  ego_goal_yaw_preset_pub_  = node.advertise<quadrotor_msgs::EgoGoalSet>("local_goal_yaw_preset", 1);


  // data init
  data_->fsm_state_           = RCCtrlFSMState::INIT;
  data_->last_fsm_state_      = data_->fsm_state_;
  data_->is_rc_signal_ready_  = false;
  data_->is_odom_ready_       = false;
  data_->last_cmd_pub_time_   = ros::Time::ZERO;
  data_->rc_data_.activate_rc_ctrl_mode_     = false;
  data_->rc_data_.need_takeoff_land_cmd_pub_ = false;
  data_->draw_vel_arrow_last_time_ = data_->draw_text_last_time_ = data_->draw_safety_check_last_time_ = ros::Time::now();

  // params init
  state_name_map_[0] = "INIT";
  state_name_map_[1] = "WAIT_FOR_RC_FLAG";
  state_name_map_[2] = "EXECUTE_RC_CTRL";

  INFO_MSG_BLUE("[RC-FSM]: parameters: \n rc_reverse_pitch: " << params_->rc_reverse_pitch_ << "\n rc_reverse_roll: " << params_->rc_reverse_roll_
                << "\n rc_reverse_yaw: " << params_->rc_reverse_yaw_ << "\n rc_reverse_throttle: " << params_->rc_reverse_thrust_);
  INFO_MSG_GREEN("[RC-FSM]: initialized successfully.");
}

RCCtrlFSM::~RCCtrlFSM() {
  scene_graph_->object_factory_->stopThisModule();
  INFO_MSG_GREEN("[RC-FSM]: deconstructed.");
}

void RCCtrlFSM::goalFromStationCallback(const quadrotor_msgs::GoalSet::ConstPtr& msg){
  scene_graph_->skeleton_gen_->expandSkeleton(data_->odom_data_.p_, data_->odom_data_.yaw_);
  // todo : 解锁对mission fsm的状态保护，为了调试，记得修改回来
  if (data_->fsm_state_ == WAIT_FOR_RC_FLAG || true){
    quadrotor_msgs::EgoGoalSet ego_goal_msg;
    ego_goal_msg.drone_id = 0;
    ego_goal_msg.goal[0]  = msg->goal[0].x;
    ego_goal_msg.goal[1]  = msg->goal[0].y;
    ego_goal_msg.goal[2]  = 1.6;
    ego_goal_msg.yaw              = msg->yaw[0];
    ego_goal_msg.look_forward     = true;
    ego_goal_msg.goal_to_follower = false;
    ego_goal_set_pub_.publish(ego_goal_msg);
    INFO_MSG_GREEN("[RC-FSM] : ============= RECV NEW GOAL FROM STATION ===========");
  }else if (data_->fsm_state_ == EXECUTE_RC_CTRL){
    INFO_MSG_RED("[RC-FSM] : goal feed permission denied, current state is [EXECUTE_RC_CTRL]!");
  }else{
    INFO_MSG_RED("[RC-FSM] : goal feed permission denied, unknown state!");
  }
}

void RCCtrlFSM::skeletonMountPointTimerCallback(const ros::TimerEvent& event){
  static int count = 0;
  if (scene_graph_->skeleton_gen_->ready()){
    scene_graph_->skeleton_gen_->updateMountedTopoPoint(data_->odom_data_.p_);
    if (count >= 2){
      count = 0;
      scene_graph_->initSceneGraph(data_->odom_data_.p_, data_->odom_data_.yaw_);
    }
  }
  count++;

  if (scene_graph_->skeleton_gen_->ready() && scene_graph_->skeleton_gen_->getNodeNum() > 1 && !scene_graph_->object_factory_->ok()) {
    scene_graph_->object_factory_->runThisModule();
  }
}

void RCCtrlFSM::fsmExecTimerCallback(const ros::TimerEvent &event) {
  fsmProcess();
}

void RCCtrlFSM::fsmVisTimerCallback(const ros::TimerEvent &event) {
  ros::Time now = ros::Time::now();
  if (now - data_->draw_vel_arrow_last_time_ > ros::Duration(0.05)) {
    data_->draw_vel_arrow_last_time_ = now;
    drawVelArrow();
  }
  if (now - data_->draw_safety_check_last_time_ > ros::Duration(0.05)){
    data_->draw_safety_check_last_time_ = now;
    drawSaftyCheckPoint(data_->odom_data_.p_, data_->pose_check_);
  }
  if (now - data_->draw_text_last_time_ > ros::Duration(0.05)){
    data_->draw_text_last_time_ = now;
    drawText();
  }
}

void RCCtrlFSM::fsmProcess() {
  data_->now_time_ = ros::Time::now();
  data_->is_rc_signal_ready_ = isRCReady(data_->now_time_);
  data_->is_odom_ready_      = isOdomReady(data_->now_time_);
  if (data_->fsm_state_ != RCCtrlFSMState::INIT)
    checkDataRecvSafety();

  if (data_->rc_data_.need_takeoff_land_cmd_pub_){
    data_->rc_data_.need_takeoff_land_cmd_pub_ = false;
    if (data_->fsm_state_ == EXECUTE_RC_CTRL){
      INFO_MSG_RED("[RC-FSM] : takeoff/land command permission denied, current state is [EXECUTE_RC_CTRL]!");
    }else{
      quadrotor_msgs::TakeoffLand tl_msg;
      tl_msg.takeoff_land_cmd = data_->rc_data_.is_armed_? 2 : 1;
      INFO_MSG_YELLOW("[RC-FSM] : send takeoff/land command: " << (data_->rc_data_.is_armed_? "LAND" : "TAKEOFF"));
      for (int i = 0; i < 20; i++) {
        px4ctrl_takeoff_land_pub_.publish(tl_msg);
        ros::Duration(0.005).sleep();
      }
    }
  }

  switch (data_->fsm_state_) {
    case INIT:
      if (data_->is_rc_signal_ready_ && data_->is_odom_ready_)
        changeFSMState("init done", RCCtrlFSMState::WAIT_FOR_RC_FLAG);
      else if (data_->is_rc_signal_ready_ && !data_->is_odom_ready_)
        ROS_WARN_STREAM_THROTTLE(1.0, "[RC-FSM]: waiting for odom data...");
      else if (!data_->is_rc_signal_ready_ && data_->is_odom_ready_)
        ROS_WARN_STREAM_THROTTLE(1.0, "[RC-FSM]: waiting for rc data...");
      else
        ROS_WARN_STREAM_THROTTLE(1.0, "[RC-FSM]: waiting for rc and odom data...");
      break;

    case WAIT_FOR_RC_FLAG:
      data_->expected_yaw_ = data_->odom_data_.yaw_;
      data_->expected_pos_ = data_->odom_data_.p_;
      if (!data_->rc_data_.check_centered()){
        ROS_WARN_STREAM_THROTTLE(2.0, "[RC-FSM]: please center the rc transmitter");
        break;
      }
      if (!data_->rc_data_.activate_rc_ctrl_mode_)
        break;
      changeFSMState("rc flag received", RCCtrlFSMState::EXECUTE_RC_CTRL);
      rc_cmd_thread_.reset(new std::thread(&RCCtrlFSM::rcCmdProcess, this));
      rc_cmd_thread_->detach();
      break;

    case EXECUTE_RC_CTRL:
      if (!data_->rc_data_.activate_rc_ctrl_mode_){
        changeFSMState("rc ctrl mode de-activate", RCCtrlFSMState::WAIT_FOR_RC_FLAG);
        holdPosition(data_->odom_data_.p_, data_->odom_data_.yaw_, true);
        break;
      }
      break;

    default:
      ROS_ERROR_STREAM_THROTTLE(0.2, "[RC-FSM]: invalid state: " << data_->fsm_state_);
      break;
  }
}

void RCCtrlFSM::rcCmdProcess(){
  INFO_MSG_BLUE("[RC-FSM] : rc cmd process start...");
  auto interval = std::chrono::milliseconds(10);
  while (true)
  {
    auto start = std::chrono::high_resolution_clock::now();
    mutex_.lock();
    if (!data_->rc_data_.activate_rc_ctrl_mode_){
      mutex_.unlock();
      break;
    }
    setHoverWithRC();
    mutex_.unlock();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    if (duration < interval){
      std::this_thread::sleep_for(interval - duration);
    }
  }
  INFO_MSG_BLUE("[RC-FSM] : rc cmd process close...");
}

void RCCtrlFSM::setHoverWithRC() {
  ros::Time now = ros::Time::now();
  double theta_t = (now - data_->last_cmd_pub_time_).toSec();
  theta_t = theta_t > 0.1 ? 0.0 : theta_t;
  data_->expected_speed_.setZero();
  double yaw = data_->odom_data_.yaw_;
//  data_->expected_pos_ = data_->odom_data_.p_;
  double ch_tmp[4] = {data_->rc_data_.ch_[0], data_->rc_data_.ch_[1], data_->rc_data_.ch_[2], data_->rc_data_.ch_[3]};
  ch_tmp[0] *= (params_->rc_reverse_roll_ ? -1.0 : 1.0);
  ch_tmp[1] *= (params_->rc_reverse_pitch_ ? -1.0 : 1.0);
  ch_tmp[2] *= (params_->rc_reverse_thrust_ ? -1.0 : 1.0);
  ch_tmp[3] *= (params_->rc_reverse_yaw_ ? -1.0 : 1.0);

  Eigen::Vector3d expected_speed_tmp;
  double expected_yaw_rate_tmp;

  expected_speed_tmp[0] = min((ch_tmp[1] * cos(yaw) + ch_tmp[0] * sin(yaw)), 1.0) * params_->max_speed_;
  expected_speed_tmp[1] = min((ch_tmp[1] * sin(yaw) - ch_tmp[0] * cos(yaw)), 1.0) * params_->max_speed_;
  expected_speed_tmp[2] = ch_tmp[2] * params_->max_speed_;
  expected_yaw_rate_tmp = ch_tmp[3] * params_->max_yaw_rate_;

  if (!checkMovementSafety(1.0, expected_speed_tmp)){
    holdPosition(data_->expected_pos_, data_->expected_yaw_, false);
    data_->is_in_collision_traj_ = true;
    return ;
  }else{
    data_->is_in_collision_traj_ = false;
    data_->expected_speed_ = expected_speed_tmp;
    data_->expected_yaw_rate_ = expected_yaw_rate_tmp;
  }
  data_->expected_pos_[0] += data_->expected_speed_[0] * theta_t;
  data_->expected_pos_[1] += data_->expected_speed_[1] * theta_t;
  data_->expected_pos_[2] += data_->expected_speed_[2] * theta_t;
  data_->expected_yaw_    += data_->expected_yaw_rate_ * theta_t;
  quadrotor_msgs::PositionCommand cmd_msg;
  data_->last_cmd_pub_time_ = ros::Time::now();
  cmd_msg.header.stamp = data_->last_cmd_pub_time_;
  cmd_msg.header.frame_id = "world";
  cmd_msg.position.x = data_->expected_pos_[0];
  cmd_msg.position.y = data_->expected_pos_[1];
  cmd_msg.position.z = data_->expected_pos_[2];
  cmd_msg.yaw        = data_->expected_yaw_;
  cmd_msg.velocity.x = data_->expected_speed_[0];
  cmd_msg.velocity.y = data_->expected_speed_[1];
  cmd_msg.velocity.z = data_->expected_speed_[2];
  cmd_msg.yaw_dot    = data_->expected_yaw_rate_;
  cmd_msg.acceleration.x = 0.0;
  cmd_msg.acceleration.y = 0.0;
  cmd_msg.acceleration.z = 0.0;
  cmd_msg.jerk.x         = 0.0;
  cmd_msg.jerk.y         = 0.0;
  cmd_msg.jerk.z         = 0.0;
  px4ctrl_cmd_pub_.publish(cmd_msg);
}

bool RCCtrlFSM::isRCReady(const ros::Time &now_time) {
  return (now_time - data_->rc_data_.rcv_stamp_).toSec() < params_->rc_timeout_;
}

bool RCCtrlFSM::isOdomReady(const ros::Time &now_time) {
  return (now_time - data_->odom_data_.rcv_stamp_).toSec() < params_->odom_timeout_;
}

void RCCtrlFSM::changeFSMState(const std::string log_info, RCCtrlFSMState new_state) {
  data_->last_fsm_state_ = data_->fsm_state_;
  data_->fsm_state_ = new_state;
  INFO_MSG_GREEN("[RC-FSM]: " << log_info <<" || from state: " << state_name_map_[data_->last_fsm_state_]
                                       << " || to state: " << state_name_map_[ new_state]);
}

bool RCCtrlFSM::checkDataRecvSafety() {
  if (data_->is_rc_signal_ready_ && data_->is_odom_ready_)
    return true;
  else if (data_->is_rc_signal_ready_ && !data_->is_odom_ready_)
    ROS_ERROR_THROTTLE(1.0, "[RC-FSM]: odom data link lost !!!");
  else if (!data_->is_rc_signal_ready_ && data_->is_odom_ready_)
    ROS_ERROR_THROTTLE(1.0, "[RC-FSM]: rc data link lost !!!");
  else
    ROS_ERROR_THROTTLE(1.0, "[RC-FSM]: rc and odom data link lost !!!");
  changeFSMState("data link lost", RCCtrlFSMState::INIT);
  return false;
}

bool RCCtrlFSM::checkMovementSafety(double safe_distance, const Eigen::Vector3d & expected_vel) {
  Eigen::Vector3d pose_check = data_->odom_data_.p_;
  Eigen::Vector3d vel_norm   = expected_vel.normalized();
  pose_check += vel_norm * safe_distance;
  data_->pose_check_ = pose_check;
  if (!map_interface_->isInLocalMap(pose_check) || !(map_interface_->isVisible(data_->odom_data_.p_, pose_check, 0.2))){
    Eigen::Vector3d pos_refine = data_->odom_data_.p_;
    if (findAvailablePointInRange(pos_refine, 0.5, 0.1)){
      if (map_interface_->isVisible(data_->odom_data_.p_, pose_check, 0.2))
        return true;
    }
    return false;
  }else
    return true;
}

void RCCtrlFSM::holdPosition(const Eigen::Vector3d &pos, const double &yaw, bool use_ego) {
  quadrotor_msgs::PositionCommand cmd_msg;
  cmd_msg.header.stamp = ros::Time::now();
  cmd_msg.header.frame_id = "world";
  cmd_msg.position.x = pos[0];
  cmd_msg.position.y = pos[1];
  cmd_msg.position.z = pos[2];
  cmd_msg.yaw = yaw;
  cmd_msg.velocity.x = 0.0;
  cmd_msg.velocity.y = 0.0;
  cmd_msg.velocity.z = 0.0;
  cmd_msg.yaw_dot    = 0.0;
  cmd_msg.acceleration.x = 0.0;
  cmd_msg.acceleration.y = 0.0;
  cmd_msg.acceleration.z = 0.0;
  cmd_msg.jerk.x         = 0.0;
  cmd_msg.jerk.y         = 0.0;
  cmd_msg.jerk.z         = 0.0;
  px4ctrl_cmd_pub_.publish(cmd_msg);

  // call ego planner to hold position
  if (use_ego){
    quadrotor_msgs::EgoGoalSet msg;
    msg.drone_id = 0;
    msg.goal[0] = pos.x();
    msg.goal[1] = pos.y();
    msg.goal[2] = pos.z();
    msg.yaw     = yaw;
    // todo | yaw rate will emergency changed when swith rc ctrl mode
    // todo | (perhaps need to reset last_yaw in ego planner-traj_server...)
    msg.look_forward     = false;
    msg.goal_to_follower = false;
    ego_goal_yaw_preset_pub_.publish(msg);
  }
}

bool RCCtrlFSM::findAvailablePointInRange(Eigen::Vector3d &pos_refine, const double &range, const double & step_size) {

  std::unordered_map<Eigen::Vector3i, bool, Vector3iHash> visited;
  std::queue<Eigen::Vector3i> q;
  q.emplace(0, 0, 0);
  visited[Eigen::Vector3i (0, 0, 0)] = true;
  int step_range = floor(range / step_size);
  while (!q.empty()){
    Eigen::Vector3i last_step = q.front(); q.pop();
    Eigen::Vector3d last_pos  = pos_refine + step_size * last_step.cast<double>();

    if (map_interface_->getInflateOccupancy(last_pos) == MapInterface::OCCUPANCY::FREE){
      pos_refine = last_pos;
//      INFO_MSG_BLUE("[Collision-Detect] : refine pos to : " << pos_refine.transpose());
      return true;
    }
    // explor neighbors in range
    Eigen::Vector3d cur_pos;
    Eigen::Vector3i cur_step;
    for (int dz = 1; dz >= -1; dz --){
      for (int dy = 1; dy >= -1; dy --){
        for (int dx = 1; dx >= -1 ; dx --){
          cur_step = last_step + Eigen::Vector3i(dx, dy, dz);
          cur_pos = pos_refine + step_size * (cur_step).cast<double>();
          if (!map_interface_->isInLocalMap(cur_pos)) continue;
          if ((cur_pos - pos_refine).norm() > range) continue;
          if (visited.find(cur_step) != visited.end()) continue;
          visited[cur_step] = true;
          q.push(cur_step);
        }
      }
    }
  }
  return false;
}

template<typename T>
void RCCtrlFSM::readParam(ros::NodeHandle &node, std::string param_name, T &param_val, T default_val) {
  if (!node.param(param_name, param_val, default_val))
    INFO_MSG_YELLOW("[RC-FSM]: parameter " << param_name << " not found, using default value: " << default_val);
  else
    INFO_MSG_GREEN("[RC-FSM]: parameter " << param_name << " found: " << param_val);
}

void RCCtrlFSM::drawVelArrow() {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "vel_arrow";
  marker.id = 0;
  marker.lifetime = ros::Duration(0.5);
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  geometry_msgs::Point start_point, end_point;
  Eigen::Vector3d end_point_eigen = data_->odom_data_.p_ + data_->expected_speed_ / params_->max_speed_ * 1.0;
  end_point.x = end_point_eigen[0]; end_point.y = end_point_eigen[1]; end_point.z = end_point_eigen[2];
  start_point.x = data_->odom_data_.p_[0]; start_point.y = data_->odom_data_.p_[1]; start_point.z = data_->odom_data_.p_[2];
  marker.points.push_back(start_point);
  marker.points.push_back(end_point);
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker_array.markers.push_back(marker);
  rc_ctrl_marker_pub_.publish(marker_array);
}

void RCCtrlFSM::drawSaftyCheckPoint(Eigen::Vector3d &check_start, Eigen::Vector3d &check_end) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "safety_check";
  marker.id = 0;
  marker.lifetime = ros::Duration(0.5);
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = check_end[0];
  marker.pose.position.y = check_end[1];
  marker.pose.position.z = check_end[2];
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 0.8;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker_array.markers.push_back(marker);

  visualization_msgs::Marker marker_line;
  marker_line.header.frame_id = "world";
  marker_line.header.stamp = ros::Time::now();
  marker_line.ns = "safety_check";
  marker_line.id = 1;
  marker_line.lifetime = ros::Duration(0.5);
  marker_line.type = visualization_msgs::Marker::LINE_STRIP;
  marker_line.action = visualization_msgs::Marker::ADD;
  marker_line.pose.orientation.w = 1.0;
  marker_line.scale.x = 0.05;
  marker_line.color.a = 0.8;
  marker_line.color.r = 0.0;
  marker_line.color.g = 0.0;
  marker_line.color.b = 1.0;
  geometry_msgs::Point start_point, end_point;
  start_point.x = check_start[0]; start_point.y = check_start[1]; start_point.z = check_start[2];
  end_point.x = check_end[0]; end_point.y = check_end[1]; end_point.z = check_end[2];
  marker_line.points.push_back(start_point);
  marker_line.points.push_back(end_point);
  marker_array.markers.push_back(marker_line);

  rc_ctrl_marker_pub_.publish(marker_array);
}

void RCCtrlFSM::drawText() {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "text";
  marker.id = 2;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = data_->odom_data_.p_[0];
  marker.pose.position.y = data_->odom_data_.p_[1] - 0.3;
  marker.pose.position.z = data_->odom_data_.p_[2] + 0.2;
  marker.scale.z = 0.15;
  marker.color.a = 1.0;
  marker.color.r = static_cast<float>(data_->is_in_collision_traj_ ? 1.0 : 0.0);
  marker.color.g = static_cast<float>(data_->is_in_collision_traj_ ? 0.0 : 1.0);
  marker.color.b = 0.0;
  marker.text = (data_->is_in_collision_traj_ ? "COLLISION" : "SAFE");
  marker_array.markers.push_back(marker);

  visualization_msgs::Marker marker_rc_status;
  marker_rc_status.header.frame_id = "world";
  marker_rc_status.header.stamp = ros::Time::now();
  marker_rc_status.ns = "text";
  marker_rc_status.id = 3;
  marker_rc_status.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_rc_status.action = visualization_msgs::Marker::ADD;
  marker_rc_status.pose.orientation.w = 1.0;
  marker_rc_status.pose.position.x = data_->odom_data_.p_[0];
  marker_rc_status.pose.position.y = data_->odom_data_.p_[1] - 0.3;
  marker_rc_status.pose.position.z = data_->odom_data_.p_[2] - 0.2;
  marker_rc_status.scale.z = 0.15;
  marker_rc_status.color.a = 1.0;
  marker_rc_status.color.r = static_cast<float>(data_->rc_data_.activate_rc_ctrl_mode_ ? 1.0 : 0.0);
  marker_rc_status.color.g = static_cast<float>(data_->rc_data_.activate_rc_ctrl_mode_ ? 0.0 : 1.0);
  marker_rc_status.color.b = 1.0;
  marker_rc_status.text = (data_->rc_data_.activate_rc_ctrl_mode_ ? "RC-CTRL ON" : "RC-CTRL OFF");
  marker_array.markers.push_back(marker_rc_status);
  rc_ctrl_marker_pub_.publish(marker_array);
}

// ==============================================================================//
// ===================================RC CLASS===================================//
// ==============================================================================//
RC_Data_t::RC_Data_t() {
  rcv_stamp_ = ros::Time(0);
  for (double & i : ch_)
    i = 0.0;
}

void RC_Data_t::check_validity() {
}

bool RC_Data_t::check_centered() {
  bool centered = abs(ch_[0]) < 1e-5 && abs(ch_[1]) < 1e-5 && abs(ch_[2]) < 1e-5 && abs(ch_[3]) < 1e-5;
  return centered;
}

void RC_Data_t::check_ch10_trigger_once() {
  if (rcv_stamp_ - ch10_trigger_last_time_ > ros::Duration(1.0))
    ch10_trigger_count_ = 0;
  if (ch10_ * last_ch10_ < 0){
    ch10_value_change_count_++;
    if (ch10_value_change_count_ == 2){
      ch10_trigger_last_time_ = rcv_stamp_;
      ch10_value_change_count_ = 0;
      ch10_trigger_count_ ++;
    }
  }
}

void RC_Data_t::process_ch10_trigger() {
  static bool takeoff_land_flag = false;
  check_ch10_trigger_once();
  if (takeoff_land_flag){
    ROS_WARN_STREAM_THROTTLE(5.0, "[WARNING]: =====================================================================");
    ROS_WARN_STREAM_THROTTLE(5.0, "[WARNING]: RC-CTRL is in "<< (is_armed_ ? "LAND" : "TAKEOFF") <<" mode, please confirm by trigger ch10 once!");
    ROS_WARN_STREAM_THROTTLE(5.0, "[WARNING]: =====================================================================");
    if (ch10_trigger_count_  == 1 && (rcv_stamp_ - ch10_trigger_last_time_ > ros::Duration(0.5))){
      takeoff_land_flag        = false;
      ch10_trigger_count_      = 0;
      ch10_value_change_count_ = 0;
      need_takeoff_land_cmd_pub_ = true;
    }
  }

  if (ch10_trigger_count_  == 3 && (rcv_stamp_ - ch10_trigger_last_time_ > ros::Duration(0.5))){
    ch10_trigger_count_          = 0;
    ch10_value_change_count_     = 0;
    takeoff_land_flag            = !takeoff_land_flag;
    if (!takeoff_land_flag)
      INFO_MSG_YELLOW("[RC-FSM]: Quit "<< (is_armed_ ? "LAND" : "TAKEOFF") <<" mode");
    return ;
  }

  if (!takeoff_land_flag && ch10_trigger_count_  == 2
       && (rcv_stamp_ - ch10_trigger_last_time_ > ros::Duration(0.5))){
    ch10_trigger_count_          = 0;
    ch10_value_change_count_     = 0;
    activate_rc_ctrl_mode_       = !activate_rc_ctrl_mode_;
    INFO_MSG_GREEN("[RC-FSM]: ==============================");
    INFO_MSG_YELLOW("[RC-FSM]: " << (activate_rc_ctrl_mode_ ? "RC-CTRL ON! Be careful!" : "RC-CTRL OFF!"));
    INFO_MSG_GREEN("[RC-FSM]: ==============================");
    return;
  }

}


void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg) {
  msg_       = * pMsg;
  rcv_stamp_ = ros::Time::now();
  //feed ch1-ch4
  int dead_zone_cnt = 0;
  for (int i = 0; i < 4; i++)
  {
    ch_[i] = ((double)msg_.channels[i] - 1500.0) / 500.0;
    if (ch_[i] > DEAD_ZONE)
      ch_[i] = (ch_[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
    else if (ch_[i] < -DEAD_ZONE)
      ch_[i] = (ch_[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
    else{
      ch_[i] = 0.0;
      dead_zone_cnt++;
    }
  }

  if (dead_zone_cnt == 4)
    have_effective_rc_cmd_ = false;
  else
    have_effective_rc_cmd_ = true;

  // feed ch10
  last_ch10_ = ch10_;
  ch10_      = ((double)msg_.channels[9] - 1500.0) / 500.0;
  process_ch10_trigger();
  check_validity();
}

void RC_Data_t::feedMavrosState(mavros_msgs::StateConstPtr pMsg) {
  state_ = *pMsg;
  is_armed_ = state_.armed;
}
// ==============================================================================//
// =================================ODOM CLASS===================================//
// ==============================================================================//
Odom_Data_t::Odom_Data_t() {
  rcv_stamp_ = ros::Time(0);
  q_.setIdentity();
  recv_new_msg_ = false;
}

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg) {
  ros::Time now = ros::Time::now();
  msg_ = *pMsg;
  rcv_stamp_ = now;
  recv_new_msg_ = true;

  p_ = Eigen::Vector3d(msg_.pose.pose.position.x, msg_.pose.pose.position.y, msg_.pose.pose.position.z);
  q_ = Eigen::Quaterniond(msg_.pose.pose.orientation.w, msg_.pose.pose.orientation.x, msg_.pose.pose.orientation.y, msg_.pose.pose.orientation.z);
  v_ = Eigen::Vector3d(msg_.twist.twist.linear.x, msg_.twist.twist.linear.y, msg_.twist.twist.linear.z);
  w_ = Eigen::Vector3d(msg_.twist.twist.angular.x, msg_.twist.twist.angular.y, msg_.twist.twist.angular.z);

  tf::Quaternion q_tf;
  tf::quaternionMsgToTF(msg_.pose.pose.orientation, q_tf);
  yaw_ = tf::getYaw(q_tf);
}
