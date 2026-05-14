#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

#include <traj_opt/poly_traj_utils.hpp>

ros::Publisher pos_cmd_pub_;
ros::Time heartbeat_time_;
bool receive_traj_ = false;
bool receive_odom_ = false;
bool flight_start_ = false;
bool last_cmd_valid_ = false;
quadrotor_msgs::PolyTraj trajMsg_;
Eigen::Vector3d last_p_;
double odom_yaw_ = 0;
double last_yaw_ = 0;

enum class TrajExecResult {
  WAIT_START,
  EXECUTED,
  FINISHED,
  INVALID,
};

double normalize_yaw(double yaw) {
  while (yaw > M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  return yaw;
}

double shortest_yaw_diff(double from, double to) {
  return normalize_yaw(to - from);
}

double odom_yaw_from_msg(const nav_msgs::Odometry& msg) {
  Eigen::Quaterniond q(msg.pose.pose.orientation.w,
                       msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z);
  const Eigen::Matrix3d rot = q.toRotationMatrix();
  return std::atan2(rot(1, 0), rot(0, 0));
}

void publish_cmd(int traj_id,
                 const Eigen::Vector3d &p,
                 const Eigen::Vector3d &v,
                 const Eigen::Vector3d &a,
                 double y, double yd) {
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub_.publish(cmd);
  last_p_ = p;
  last_yaw_ = y;
  last_cmd_valid_ = true;
}

TrajExecResult exe_traj(const quadrotor_msgs::PolyTraj &trajMsg) {
  double t = (ros::Time::now() - trajMsg.start_time).toSec();
  if (t > 0) {
    if (trajMsg.hover) {
      if (trajMsg.hover_p.size() != 3) {
        ROS_ERROR("[traj_server] hover_p is not 3d!");
        return TrajExecResult::INVALID;
      }
      Eigen::Vector3d p, v0;
      p.x() = trajMsg.hover_p[0];
      p.y() = trajMsg.hover_p[1];
      p.z() = trajMsg.hover_p[2];
      v0.setZero();
      publish_cmd(trajMsg.traj_id, p, v0, v0, last_yaw_, 0);
      return TrajExecResult::EXECUTED;
    }
    if (trajMsg.order != 5) {
      ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
      return TrajExecResult::INVALID;
    }
    if (trajMsg.duration.size() * (trajMsg.order + 1) != trajMsg.coef_x.size()) {
      ROS_ERROR("[traj_server] WRONG trajectory parameters!");
      return TrajExecResult::INVALID;
    }
    int piece_nums = trajMsg.duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i) {
      int i6 = i * 6;
      cMats[i].row(0) << trajMsg.coef_x[i6 + 0], trajMsg.coef_x[i6 + 1], trajMsg.coef_x[i6 + 2],
          trajMsg.coef_x[i6 + 3], trajMsg.coef_x[i6 + 4], trajMsg.coef_x[i6 + 5];
      cMats[i].row(1) << trajMsg.coef_y[i6 + 0], trajMsg.coef_y[i6 + 1], trajMsg.coef_y[i6 + 2],
          trajMsg.coef_y[i6 + 3], trajMsg.coef_y[i6 + 4], trajMsg.coef_y[i6 + 5];
      cMats[i].row(2) << trajMsg.coef_z[i6 + 0], trajMsg.coef_z[i6 + 1], trajMsg.coef_z[i6 + 2],
          trajMsg.coef_z[i6 + 3], trajMsg.coef_z[i6 + 4], trajMsg.coef_z[i6 + 5];

      dura[i] = trajMsg.duration[i];
    }
    Trajectory traj(dura, cMats);
    if (t > traj.getTotalDuration()) {
      ROS_ERROR("[traj_server] trajectory too short left!");
      return TrajExecResult::FINISHED;
    }
    Eigen::Vector3d p, v, a;
    p = traj.getPos(t);
    v = traj.getVel(t);
    a = traj.getAcc(t);
    // yaw 指令从当前 odom 初始化，并限制每个控制周期的变化量，避免远距离 tracking 首帧突然转向。
    double yaw = normalize_yaw(trajMsg.yaw);
    const double d_yaw = shortest_yaw_diff(last_yaw_, yaw);
    const double d_yaw_abs = std::fabs(d_yaw);
    double yaw_step = d_yaw;
    if (d_yaw_abs >= 0.02) {
      yaw_step = d_yaw / d_yaw_abs * 0.02;
    }
    yaw = normalize_yaw(last_yaw_ + yaw_step);
    publish_cmd(trajMsg.traj_id, p, v, a, yaw, yaw_step / 0.01);
    return TrajExecResult::EXECUTED;
  }
  return TrajExecResult::WAIT_START;
}

void heartbeatCallback(const std_msgs::EmptyConstPtr &msg) {
  heartbeat_time_ = ros::Time::now();
}

void reset_active_traj_to_odom_yaw() {
  receive_traj_ = false;
  last_cmd_valid_ = false;
  if (receive_odom_) {
    last_yaw_ = odom_yaw_;
  }
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  odom_yaw_ = odom_yaw_from_msg(*msg);
  receive_odom_ = true;
  if (!last_cmd_valid_) {
    // 第一条 elastic 指令出来前，yaw 参考应当跟随飞机当前姿态，而不是默认从 0 开始。
    last_yaw_ = odom_yaw_;
  }
}

void triggerCallback(const geometry_msgs::PoseStampedConstPtr & /*msg*/) {
  // 新 tracking 触发后，必须等新的 PolyTraj 到达再输出；否则 traj_server 会继续发布上一轮 hover。
  reset_active_traj_to_odom_yaw();
  ROS_INFO("[traj_server] clear cached trajectory on new tracking trigger.");
}

void stopTrackingCallback(const std_msgs::EmptyConstPtr & /*msg*/) {
  // ws_main 主动结束 elastic tracking 后，清掉当前轨迹，避免下一次切换时复用旧任务指令。
  reset_active_traj_to_odom_yaw();
  ROS_INFO("[traj_server] clear cached trajectory on tracking stop.");
}

void polyTrajCallback(const quadrotor_msgs::PolyTrajConstPtr &msgPtr) {
  const bool was_hover = receive_traj_ && trajMsg_.hover;
  trajMsg_ = *msgPtr;
  if (receive_odom_ && (!last_cmd_valid_ || (!trajMsg_.hover && was_hover))) {
    last_yaw_ = odom_yaw_;
  }
  if (!receive_traj_) {
    receive_traj_ = true;
  }
}

void cmdCallback(const ros::TimerEvent &e) {
  if (!receive_traj_) {
    return;
  }
  ros::Time time_now = ros::Time::now();
  if ((time_now - heartbeat_time_).toSec() > 0.5) {
    ROS_ERROR_ONCE("[traj_server] Lost heartbeat from the planner, is he dead?");
    if (last_cmd_valid_) {
      publish_cmd(trajMsg_.traj_id, last_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), last_yaw_, 0);
    }
    return;
  }

  const TrajExecResult exec_result = exe_traj(trajMsg_);
  if (exec_result == TrajExecResult::WAIT_START) {
    // 新轨迹 start_time 通常略晚于当前时间。这里不能回放上一条轨迹，否则任务切换瞬间
    // 会短暂输出上一轮 tracking 的位置/yaw，引发控制器姿态突变。
    return;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("trajectory", 10, polyTrajCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);
  ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber trigger_sub = nh.subscribe("triger", 10, triggerCallback);
  ros::Subscriber stop_tracking_sub = nh.subscribe("stop_tracking", 10, stopTrackingCallback);

  pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}
