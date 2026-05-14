#include <plan_manage/traj_server.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;
using namespace std;

namespace ego_planner
{

  void TrajServer::initTrajServer(ros::NodeHandle &node)
  {
    node_ = node;
    pos_cmd_pub_ = node_.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
    cmd_vis_pub_ = node_.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
    
    node_.param("traj_server/time_forward", time_forward_, -1.0);
    node_.param("traj_server/max_yaw_vel", yaw_vel_limit_, 100.0);     // deg/s
    node_.param("traj_server/max_yaw_acc", yaw_acc_limit_, 100.0);     // deg/s^2
    node_.param("traj_server/max_yaw_vel_low", yaw_vel_low_limit_, 50.0);     // deg/s
    node_.param("traj_server/max_yaw_acc_low", yaw_acc_low_limit_, 50.0);     // deg/s^2

    yaw_vel_limit_ = abs(yaw_vel_limit_ * M_PI / 180.0);
    yaw_acc_limit_ = abs(yaw_acc_limit_ * M_PI / 180.0);
    yaw_vel_low_limit_ = abs(yaw_vel_low_limit_ * M_PI / 180.0);
    yaw_acc_low_limit_ = abs(yaw_acc_low_limit_ * M_PI / 180.0);

    last_yaw_     = 0.0;
    last_yawdot_  = 0.0;
    percep_utils_ = std::make_shared<PerceptionUtils>(node_);
    std::thread cmd_thread(TrajServer::cmdThread, this);
    cmd_thread.detach();
  }

  void TrajServer::feedDog()
  {
    heartbeat_time_ = ros::Time::now();
    do_once_ = true;
  }

  void TrajServer::resetYawLookforward(Eigen::Vector3d pos)
  {
    std::cout << "[TrajServer] resetYawLookforward" << std::endl;
    yaw_given_.pos = pos;
    yaw_given_.reach_given_yaw_ = true;
    yaw_given_.look_forward = true;
    yaw_given_.low_speed = false;
  }


  void TrajServer::setYaw(double des_yaw, double cur_yaw, Eigen::Vector3d pos, bool look_forward, bool low_speed)
  {
    (void)cur_yaw;
    // std::cout << "[TrajServer] setYaw : " << des_yaw << ", cur_yaw: " << cur_yaw << ", pos: " << pos.transpose() << ", look_forward: " << look_forward << ", low_speed: " << low_speed << std::endl;
    yaw_given_.yaw = des_yaw;
    yaw_given_.pos = pos;
    yaw_given_.reach_given_yaw_ = false;
    yaw_given_.look_forward = look_forward;
    yaw_given_.low_speed = low_speed;
    // 当前真实 yaw 由 EGOReplanFSM 在新目标边界调用 syncYawFromOdom() 同步，避免高频 setYaw 破坏 yaw 限速递推。
  }

  void TrajServer::syncYawFromOdom(const double yaw, const std::string& source)
  {
    double normalized_yaw = yaw;
    while (normalized_yaw > M_PI) normalized_yaw -= 2 * M_PI;
    while (normalized_yaw < -M_PI) normalized_yaw += 2 * M_PI;

    last_yaw_ = normalized_yaw;
    last_yawdot_ = 0.0;
    time_rec_.has_init = false;
    if (source.empty()) {
      ROS_INFO_STREAM_THROTTLE(0.5, "[TrajServer] sync yaw from odom: " << normalized_yaw);
    } else {
      ROS_INFO_STREAM_THROTTLE(0.5, "[TrajServer] sync yaw from odom: " << normalized_yaw
                               << " source=" << source);
    }
  }

  void TrajServer::setTrajectory(poly_traj::Trajectory &traj, double start_time)
  {
    traj_ = traj;
    start_time_ = start_time;
    traj_duration_ = traj_.getTotalDuration();
    traj_id_++;

    receive_traj_ = true;
    traj_state_   = TrajState::IDLE;
  }

  std::pair<double, double> TrajServer::calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
  {
    // constexpr double YAW_DOT_MAX_PER_SEC = 1.5 * M_PI;
    // constexpr double YAW_DOT_MAX_PER_SEC = 0.8 * M_PI;
    // constexpr double YAW_DOT_DOT_MAX_PER_SEC = 2.0 * M_PI;
    std::pair<double, double> yaw_yawdot(0, 0);

    double yaw_temp;
    // ROS_INFO_STREAM("[traj_server] yaw: " << yaw_given_.yaw << ", auto yaw: " << yaw_given_.auto_yaw_);
    // ROS_INFO_STREAM("dt: " << dt);
    if (yaw_given_.look_forward && yaw_given_.reach_given_yaw_)
    // if (yaw_given_.reach_given_yaw_)
    {
      Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                                ? traj_.getPos(t_cur + time_forward_) - pos
                                : traj_.getPos(traj_duration_) - pos;
      yaw_temp = dir.norm() > 0.1
                     ? atan2(dir(1), dir(0))
                     : last_yaw_;
    }
    else
    {
      yaw_temp = yaw_given_.yaw;
      if (abs(last_yaw_ - yaw_given_.yaw) < 0.01)
      {
        yaw_given_.reach_given_yaw_ = true;
      }
    }

    // ROS_INFO_STREAM("reach_given_yaw_: " << yaw_given_.reach_given_yaw_ << ", yaw_temp: " << yaw_temp);

    double yawdot = 0;
    double d_yaw = yaw_temp - last_yaw_;
    if (d_yaw >= M_PI)
    {
      d_yaw -= 2 * M_PI;
    }
    if (d_yaw <= -M_PI)
    {
      d_yaw += 2 * M_PI;
    }

    double YDM, YDDM;
    if (yaw_given_.low_speed) {
      YDM = d_yaw >= 0 ?  yaw_vel_low_limit_ : -yaw_vel_low_limit_;
      YDDM = d_yaw >= 0 ? yaw_acc_low_limit_ : -yaw_acc_low_limit_;
    }else {
      YDM = d_yaw >= 0 ? yaw_vel_limit_ : -yaw_vel_limit_;
      YDDM = d_yaw >= 0 ? yaw_acc_limit_ : -yaw_acc_limit_;
    }

    double d_yaw_max;
    if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
    {
      // yawdot = last_yawdot_ + dt * YDDM;
      d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
      // ROS_INFO_STREAM("A last_yawdot_: " << last_yawdot_ << ", yd: " << last_yawdot_ + YDDM * dt);
      // printf("A d_yaw_max=%f, last_yawdot_=%f, dt=%f, YDDM=%f\n", d_yaw_max, last_yawdot_, dt, YDDM);
    }
    else
    {
      // yawdot = YDM;
      double t1 = (YDM - last_yawdot_) / YDDM;
      d_yaw_max = (last_yawdot_ * t1 + 0.5 * YDDM * t1 * t1) + YDM * (dt - t1);
      // ROS_INFO_STREAM("B last_yawdot_: " << last_yawdot_ << ", yd: " << last_yawdot_ + YDDM * t1);
      // printf("B d_yaw_max=%f, last_yawdot_=%f, dt=%f, t1=%f, YDM=%f, YDDM=%f\n", d_yaw_max, last_yawdot_, dt, t1, YDM, YDDM);
    }

    if (fabs(d_yaw) > fabs(d_yaw_max))
    {
      d_yaw = d_yaw_max;
    }
    yawdot = d_yaw / dt;
    // printf("yawdot=%f\n", yawdot);
    // ROS_INFO_STREAM("yawdot: " << yawdot);

    double yaw = last_yaw_ + d_yaw;

    // ROS_INFO_STREAM("[traj_server] last_yaw_: " << last_yaw_ << ", dyaw: " << d_yaw);
    // ROS_INFO("------------------------------------");

    if (yaw > M_PI)
      yaw -= 2 * M_PI;
    if (yaw < -M_PI)
      yaw += 2 * M_PI;
    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    last_yaw_ = yaw_yawdot.first;
    last_yawdot_ = yaw_yawdot.second;

    // yaw_yawdot.second = yaw_temp;
    // ROS_ERROR_STREAM("[traj_server] yaw_res: " << yaw_yawdot.first);

    return yaw_yawdot;
  }

  void TrajServer::publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
  {
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = traj_id_;

    cmd.position.x = p(0);
    cmd.position.y = p(1);
    cmd.position.z = p(2);
    cmd.velocity.x = v(0);
    cmd.velocity.y = v(1);
    cmd.velocity.z = v(2);
    cmd.acceleration.x = a(0);
    cmd.acceleration.y = a(1);
    cmd.acceleration.z = a(2);
    cmd.jerk.x = j(0);
    cmd.jerk.y = j(1);
    cmd.jerk.z = j(2);
    cmd.yaw = y;
    cmd.yaw_dot = yd;
    pos_cmd_pub_.publish(cmd);

    last_pos_ = p;
  }

  void TrajServer::resetLastPos(const Eigen::Vector3d pos) {
    last_pos_ = pos;
  }

  void TrajServer::cmdThread(void *obj)
  {
    TrajServer *tsvr = reinterpret_cast<TrajServer *>(obj);

    while (true)
    {
      ros::Time ts = ros::Time::now();
      tsvr->cmdFun();

      std::chrono::milliseconds dura(max(10 - (int)((ros::Time::now() - ts).toSec() * 1000), 1)); // 100Hz
      std::this_thread::sleep_for(dura);
    }
  }

  void TrajServer::cmdFun()
  {

    /* no publishing before receive traj_ and have heartbeat */
    if (heartbeat_time_.toSec() <= 1e-5) return;


    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
    std::pair<double, double> yaw_yawdot(0, 0);
    ros::Time time_now = ros::Time::now();

    if (!receive_traj_ && yaw_given_.reach_given_yaw_) {
      time_rec_.has_init   = false;
      yaw_given_.low_speed = false;
      // ROS_WARN_THROTTLE(5.0, "[traj_server] Waiting for trajectory to be received...");
    }
    if ((time_now - heartbeat_time_).toSec() > 0.5){
      if (do_once_){
        do_once_ = false;
        ROS_ERROR("[traj_server] Lost heartbeat from the planner. t=%f", (time_now - heartbeat_time_).toSec());

        if (last_pos_.init)
          publish_cmd(last_pos_.p, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
        return;
      }
      receive_traj_ = false;
    }

    if (!receive_traj_)
    {
      if (!yaw_given_.reach_given_yaw_)
      {
        pos = yaw_given_.pos;
        if (!time_rec_.has_init)
        {
          time_rec_.time_last = time_now;
          last_yawdot_ = 0.0;
          time_rec_.has_init = true;
        }
        else
        {
          /*** calculate yaw ***/
          yaw_yawdot = calculate_yaw(0.0, pos, (time_now - time_rec_.time_last).toSec());
          /*** calculate yaw ***/
          time_rec_.time_last = time_now;
          last_yaw_ = yaw_yawdot.first;
          last_pos_ = pos;

          // publish
          publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);

          percep_utils_->setPose(pos, yaw_yawdot.first);
          vector<Eigen::Vector3d> l1, l2;
          percep_utils_->getFOV(l1, l2);
          drawFOV(l1, l2, cmd_vis_pub_);
        }
      }
      if (yaw_given_.low_speed) yaw_given_.low_speed = false;
    }
    else
    {
      double t_cur = time_now.toSec() - start_time_;

      // auto exec_yaw = [this, &yaw_yawdot, &t_cur, &time_now, &pos, &vel, &jer, &acc]() {
      //   if (!time_rec_.has_init){
      //     time_rec_.time_last = time_now;
      //     last_yawdot_ = 0.0;
      //     time_rec_.has_init = true;
      //   }else {
      //     setYaw(traj_init_yaw_, last_yaw_, traj_init_pos_, true);
      //     yaw_yawdot = calculate_yaw(0.0, yaw_given_.pos, (time_now - time_rec_.time_last).toSec());
      //     pos = traj_init_pos_;
      //
      //     time_rec_.time_last = time_now;
      //     last_yaw_ = yaw_yawdot.first;
      //     last_pos_ = pos;
      //     if (abs(yaw_yawdot.first - traj_init_yaw_) * 180.0 / M_PI < 5.0f && traj_state_ == TrajState::PRE_YAW) {
      //       start_time_ = time_now.toSec();
      //       traj_state_ = TrajState::EXECUTING_TRAJ;
      //       yaw_given_.reach_given_yaw_ = true;
      //       std::cout << "[TrajServer] >> Traj Init Yaw Reached ... !" << std::endl;
      //     }
      //   }
      // };
      //
      // switch (traj_state_) {
      //   case TrajState::IDLE: {
      //     Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
      //                    ? traj_.getPos(t_cur + time_forward_) - pos
      //                    : traj_.getPos(traj_duration_) - pos;
      //     traj_init_yaw_ = dir.norm() > 0.1
      //                    ? atan2(dir(1), dir(0))
      //                    : last_yaw_;
      //     traj_init_pos_ = traj_.getPos(0.0);
      //     traj_state_    = TrajState::PRE_YAW;
      //     exec_yaw();
      //     break;
      //   }
      //   case TrajState::PRE_YAW: {
      //     exec_yaw();
      //     break;
      //   }
      //   case TrajState::EXECUTING_TRAJ: {
      //     if (t_cur < traj_duration_ && t_cur >= 0.0) {
      //       if (!time_rec_.has_init){
      //         time_rec_.time_last = time_now;
      //         last_yawdot_ = 0.0;
      //         time_rec_.has_init = true;
      //       }else {
      //         pos = traj_.getPos(t_cur);
      //         vel = traj_.getVel(t_cur);
      //         acc = traj_.getAcc(t_cur);
      //         jer = traj_.getJer(t_cur);
      //
      //         /*** calculate yaw ***/
      //         yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_rec_.time_last).toSec());
      //         /*** calculate yaw ***/
      //
      //         time_rec_.time_last = time_now;
      //         last_yaw_ = yaw_yawdot.first;
      //         last_pos_ = pos;
      //       }
      //     }else {
      //       receive_traj_ = false;
      //       yaw_given_.reach_given_yaw_ = true;
      //     }
      //     break;
      //   }
      //   default: {
      //     break;
      //   }
      // }
      // publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
      // percep_utils_->setPose(pos, yaw_yawdot.first);
      // vector<Eigen::Vector3d> l1, l2;
      // percep_utils_->getFOV(l1, l2);
      // drawFOV(l1, l2, cmd_vis_pub_);

      if (t_cur < traj_duration_ && t_cur >= 0.0)
      {
        if (!time_rec_.has_init)
        {
          time_rec_.time_last = time_now;
          last_yawdot_ = 0.0;
          time_rec_.has_init = true;
        }
        else
        {
          pos = traj_.getPos(t_cur);
          vel = traj_.getVel(t_cur);
          acc = traj_.getAcc(t_cur);
          jer = traj_.getJer(t_cur);

          /*** calculate yaw ***/
          yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_rec_.time_last).toSec());
          /*** calculate yaw ***/

          time_rec_.time_last = time_now;
          last_yaw_ = yaw_yawdot.first;
          last_pos_ = pos;

          // slowly_flip_yaw_target_ = yaw_yawdot.first + M_PI;
          // if (slowly_flip_yaw_target_ > M_PI)
          //   slowly_flip_yaw_target_ -= 2 * M_PI;
          // if (slowly_flip_yaw_target_ < -M_PI)
          //   slowly_flip_yaw_target_ += 2 * M_PI;
          // constexpr double CENTER[2] = {0.0, 0.0};
          // slowly_turn_to_center_target_ = atan2(CENTER[1] - pos(1), CENTER[0] - pos(0));

          // publish
          publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
          percep_utils_->setPose(pos, yaw_yawdot.first);
          vector<Eigen::Vector3d> l1, l2;
          percep_utils_->getFOV(l1, l2);
          drawFOV(l1, l2, cmd_vis_pub_);
        }
      }
      else if (t_cur > traj_duration_)
      {
        receive_traj_ = false;
      }
    }
  }


  void TrajServer::drawFOV(const std::vector<Eigen::Vector3d>& list1, const std::vector<Eigen::Vector3d>& list2, ros::Publisher& pub, 
                           double r, double g, double b) 
  {
      visualization_msgs::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = ros::Time::now();
      mk.id = 0;
      mk.ns = "current_pose";
      mk.type = visualization_msgs::Marker::LINE_LIST;
      mk.pose.orientation.x = 0.0;
      mk.pose.orientation.y = 0.0;
      mk.pose.orientation.z = 0.0;
      mk.pose.orientation.w = 1.0;
      mk.color.r = r;
      mk.color.g = g;
      mk.color.b = b;
      mk.color.a = 1.0;
      mk.scale.x = 0.04;
      mk.scale.y = 0.04;
      mk.scale.z = 0.04;

      // Clean old marker
      mk.action = visualization_msgs::Marker::DELETE;
      pub.publish(mk);

      if (list1.size() == 0) return;

      // Pub new marker
      geometry_msgs::Point pt;
      for (int i = 0; i < int(list1.size()); ++i) {
          pt.x = list1[i](0);
          pt.y = list1[i](1);
          pt.z = list1[i](2);
          mk.points.push_back(pt);

          pt.x = list2[i](0);
          pt.y = list2[i](1);
          pt.z = list2[i](2);
          mk.points.push_back(pt);
      }
      mk.action = visualization_msgs::Marker::ADD;
      pub.publish(mk);
  }


} // namespace ego_planner
