#include <ros/ros.h>
#include <std_msgs/String.h>
#include <quadrotor_msgs/PositionCommand.h>

#include <string>

class PlannerCmdMux {
public:
  PlannerCmdMux()
      : nh_("~") {
    nh_.param("default_mode", active_mode_, std::string("ego"));
    nh_.param("ego_mode", ego_mode_, std::string("ego"));
    nh_.param("elastic_mode", elastic_mode_, std::string("elastic"));
    nh_.param("input_timeout", input_timeout_, 0.5);

    cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 20);
    ego_cmd_sub_ = nh_.subscribe("ego_position_cmd", 20, &PlannerCmdMux::egoCmdCallback, this,
                                 ros::TransportHints().tcpNoDelay());
    elastic_cmd_sub_ = nh_.subscribe("elastic_position_cmd", 20, &PlannerCmdMux::elasticCmdCallback, this,
                                     ros::TransportHints().tcpNoDelay());
    mode_sub_ = nh_.subscribe("mode", 10, &PlannerCmdMux::modeCallback, this,
                              ros::TransportHints().tcpNoDelay());

    ROS_INFO_STREAM("[planner_cmd_mux] start with mode=" << active_mode_
                    << ", ego_mode=" << ego_mode_
                    << ", elastic_mode=" << elastic_mode_
                    << ", input_timeout=" << input_timeout_);
  }

private:
  bool isKnownMode(const std::string& mode) const {
    return mode == ego_mode_ || mode == elastic_mode_;
  }

  bool isFresh(const ros::Time& stamp) const {
    // input_timeout<=0 表示不检查缓存指令的新鲜度，直接允许转发最后一帧。
    if (input_timeout_ <= 0.0) return true;
    if (stamp.isZero()) return false;
    return (ros::Time::now() - stamp).toSec() <= input_timeout_;
  }

  bool isElasticCmdUsable() const {
    if (!has_elastic_cmd_ || !isFresh(elastic_cmd_stamp_)) return false;
    // 切换到 elastic 后只能转发切换之后收到的新指令，避免上一轮 tracking 缓存被立即送给控制器。
    if (!elastic_mode_start_time_.isZero() && elastic_cmd_stamp_ < elastic_mode_start_time_) return false;
    if (blocked_elastic_traj_id_ >= 0 && last_elastic_cmd_.trajectory_id == blocked_elastic_traj_id_) return false;
    return true;
  }

  void publishIfActive(const quadrotor_msgs::PositionCommand& cmd,
                       const std::string& source_mode) {
    if (active_mode_ != source_mode) return;
    cmd_pub_.publish(cmd);
  }

  void publishLastForMode(const std::string& source) {
    if (active_mode_ == ego_mode_) {
      if (has_ego_cmd_ && isFresh(ego_cmd_stamp_)) {
        cmd_pub_.publish(last_ego_cmd_);
      } else {
        ROS_WARN_STREAM_THROTTLE(1.0, "[planner_cmd_mux] no fresh ego cmd after switch from " << source);
      }
      return;
    }

    if (active_mode_ == elastic_mode_) {
      if (isElasticCmdUsable()) {
        cmd_pub_.publish(last_elastic_cmd_);
      } else {
        ROS_WARN_STREAM_THROTTLE(1.0, "[planner_cmd_mux] no fresh elastic cmd after switch from " << source);
      }
    }
  }

  void modeCallback(const std_msgs::String::ConstPtr& msg) {
    const std::string next_mode = msg->data;
    if (!isKnownMode(next_mode)) {
      ROS_WARN_STREAM("[planner_cmd_mux] ignore unknown mode: " << next_mode);
      return;
    }
    if (next_mode == active_mode_) return;

    active_mode_ = next_mode;
    if (active_mode_ == elastic_mode_) {
      elastic_mode_start_time_ = ros::Time::now();
      blocked_elastic_traj_id_ = has_elastic_cmd_ ? last_elastic_cmd_.trajectory_id : -1;
      has_elastic_cmd_ = false;
    } else {
      blocked_elastic_traj_id_ = -1;
    }
    ROS_INFO_STREAM("[planner_cmd_mux] switch mode to " << active_mode_);
    publishLastForMode("mode_callback");
  }

  void egoCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    last_ego_cmd_ = *msg;
    ego_cmd_stamp_ = ros::Time::now();
    has_ego_cmd_ = true;
    publishIfActive(last_ego_cmd_, ego_mode_);
  }

  void elasticCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    if (active_mode_ == elastic_mode_ && blocked_elastic_traj_id_ >= 0) {
      if (msg->trajectory_id == blocked_elastic_traj_id_) {
        ROS_WARN_STREAM_THROTTLE(0.5, "[planner_cmd_mux] drop stale elastic trajectory_id="
                                          << msg->trajectory_id << " after mode switch");
        return;
      }
      blocked_elastic_traj_id_ = -1;
    }
    last_elastic_cmd_ = *msg;
    elastic_cmd_stamp_ = ros::Time::now();
    has_elastic_cmd_ = true;
    publishIfActive(last_elastic_cmd_, elastic_mode_);
  }

  ros::NodeHandle nh_;
  ros::Publisher cmd_pub_;
  ros::Subscriber ego_cmd_sub_;
  ros::Subscriber elastic_cmd_sub_;
  ros::Subscriber mode_sub_;

  std::string active_mode_;
  std::string ego_mode_;
  std::string elastic_mode_;
  double input_timeout_{0.5};

  bool has_ego_cmd_{false};
  bool has_elastic_cmd_{false};
  int blocked_elastic_traj_id_{-1};
  ros::Time elastic_mode_start_time_;
  ros::Time ego_cmd_stamp_;
  ros::Time elastic_cmd_stamp_;
  quadrotor_msgs::PositionCommand last_ego_cmd_;
  quadrotor_msgs::PositionCommand last_elastic_cmd_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "planner_cmd_mux");
  PlannerCmdMux mux;
  ros::spin();
  return 0;
}
