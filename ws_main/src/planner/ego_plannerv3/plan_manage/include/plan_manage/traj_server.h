#ifndef _TRAJ_SERVER_H_
#define _TRAJ_SERVER_H_

#include <iostream>
#include <thread>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <perception_utils/perception_utils.h>

namespace ego_planner
{
    class TrajServer
    {
    private:

        enum class TrajState {
            IDLE,          // 空闲（无轨迹）
            PRE_YAW,       // 预转yaw阶段（未执行轨迹）
            EXECUTING_TRAJ // 执行轨迹阶段
          };

        TrajState traj_state_ = TrajState::IDLE; // 默认初始状态为空闲
        double traj_init_yaw_{0.0};
        Eigen::Vector3d traj_init_pos_{Eigen::Vector3d::Zero()};

        ros::NodeHandle node_;
        ros::Publisher pos_cmd_pub_, cmd_vis_pub_;

        shared_ptr<PerceptionUtils> percep_utils_;

        bool receive_traj_{false};
        poly_traj::Trajectory traj_;
        double traj_duration_;
        double start_time_;
        int traj_id_{0};
        ros::Time heartbeat_time_{0};
        bool do_once_ = true;

        // yaw control
        double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
        double time_forward_;

        double yaw_vel_limit_, yaw_acc_limit_, yaw_vel_low_limit_, yaw_acc_low_limit_;

        struct LAST_POS
        {
            Eigen::Vector3d p;
            bool init{false};
            inline void operator=(const Eigen::Vector3d p_in)
            {
                p = p_in;
                init = true;
            }
        } last_pos_;
        struct YAW_GIVEN
        {
            double yaw;
            bool reach_given_yaw_{true};
            bool look_forward{true};
            bool low_speed{false};
            Eigen::Vector3d pos;
        };
        struct TIME_REC
        {
            ros::Time time_last = ros::Time(0);
            bool has_init{false};
        } time_rec_;

    public:
        TrajServer(){};
        ~TrajServer(){};
        
        void initTrajServer(ros::NodeHandle &node);
        void setTrajectory(poly_traj::Trajectory &traj, double start_time);
        void setYaw(double des_yaw, double cur_yaw, Eigen::Vector3d pos, bool look_forward = true, bool low_speed = false);
        void resetYawLookforward(Eigen::Vector3d pos);
        void feedDog();
        void resetLastPos(const Eigen::Vector3d pos);
        void setYawtoGiven(const double yaw);

        YAW_GIVEN yaw_given_;

    private:
        // void heartbeatCallback(std_msgs::EmptyPtr msg);
        std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt);
        void publish_cmd(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, Eigen::Vector3d j, double y, double yd);
        static void cmdThread(void *obj);
        void cmdFun();
        void drawFOV(const std::vector<Eigen::Vector3d>& list1, const std::vector<Eigen::Vector3d>& list2, ros::Publisher& pub, 
                     double r = 1.0, double g = 0.0, double b = 0.0);
    };
} // namespace ego_planner
#endif