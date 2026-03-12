#ifndef _ODOM_BFFER_H_
#define _ODOM_BFFER_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <deque>

using Eigen::Vector3d;
using std::vector;
using std::string;

namespace ego_planner {


class OdomBuffer {
public:
    
    const size_t BUFFER_SIZE = 100;//考虑均匀
    bool inited = false;
    void addOdom(const Eigen::Vector3d& odom,double yaw) {
        if (!inited) {
           std::pair<Eigen::Vector3d, Eigen::Vector3d> last_pair = {odom, odom};//init odom TODO
           last_farthest_point=odom;
           last_farthest_point_yaw=yaw;
            inited = true;
        }
        if (buffer.size() == BUFFER_SIZE) {
            buffer.pop_front();
            buffer_yaw.pop_front();
        }
        buffer.push_back(odom);
        buffer_yaw.push_back(yaw);
    }
    double getsize() {
        return buffer.size(); 
    }
    double distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return (a - b).norm();}
    std::vector<Eigen::Vector3d> getOdoms() {
        vector<Eigen::Vector3d> odoms;
        for (const auto& odom : buffer) {
            odoms.push_back(odom);
        }
        return odoms;
    }
    Eigen::Vector3d getFarthestPoint(const Eigen::Vector3d& current_odom) {
        if (buffer.empty()) {
            throw std::runtime_error("Buffer is empty");
        }
        Eigen::Vector3d farthest_point;
        double max_distance = std::numeric_limits<double>::min();

        for (const auto& odom : buffer) {
            double dist = distance(current_odom, odom);
            if (dist > max_distance) {
                max_distance = dist;
                farthest_point = odom;
            }
        }
        return farthest_point;
    }
    double getFarthestPointYaw(const Eigen::Vector3d& current_odom) {
        if (buffer_yaw.empty()) {
            throw std::runtime_error("Buffer is empty");
        }
        double max_distance = std::numeric_limits<double>::min();
        Eigen::Vector3d farthest_point = Eigen::Vector3d::Zero();
        size_t farthest_index = 0; // Track the index of the farthest point

        for (size_t i = 0; i < buffer.size(); ++i) {
            const auto& odom = buffer[i];
            double dist = distance(current_odom, odom);
            if (dist > max_distance) {
                max_distance = dist;
                farthest_point = odom;
                farthest_index = i; // Update the index of the farthest point
            }
        }
        // Retrieve the corresponding yaw value
        double corresponding_yaw = buffer_yaw[farthest_index];
        return corresponding_yaw;
    }
    inline  double getsize() const {
        return buffer.size(); 
    }
    inline  double distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
        return (a - b).norm();
    }
    std::pair<Eigen::Vector3d, Eigen::Vector3d> getTwoPointinRange(const Eigen::Vector3d& current_odom, const double range) {
        if (buffer.size() < 2) {
            throw std::runtime_error("Not enough points in buffer");
        }

        Eigen::Vector3d point1, point2;
        double max_dist1 = std::numeric_limits<double>::min();
        double max_dist2_current_odom = std::numeric_limits<double>::min();
        double max_dist_between_points = std::numeric_limits<double>::min();
        bool found = false;

        for (size_t i = 0; i < buffer.size(); ++i) {
            double dist1 = distance(current_odom, buffer[i]);
            if (dist1 <= range && dist1 > max_dist1) {
                for (size_t j = i + 1; j < buffer.size(); ++j) {
                    double dist2 = distance(current_odom, buffer[j]);
                    double dist_between_points = distance(buffer[i], buffer[j]);

                    if (dist2 <= range && dist2 > max_dist2_current_odom && dist_between_points > max_dist_between_points) {
                        point1 = buffer[i];
                        point2 = buffer[j];
                        max_dist1 = dist1;
                        max_dist2_current_odom = dist2;
                        max_dist_between_points = dist_between_points;
                        found = true;
                    }
                }
            }
        }

        if (getsize() == BUFFER_SIZE && !found) {
            // std::cout << "xxxxxxx No two points in range found xxxxxxxxxxxx" << std::endl;
              ROS_ERROR_STREAM("xxxxxxx No point in range found xxxxxxxxxxxx");
            // throw std::runtime_error("No two points in range found");
            return last_pair;
        }

        // 保持返回点对的一致性，尽量避免交换位置
        if (!last_pair.first.isZero() && !last_pair.second.isZero()) {
            double current_dist1 = distance(point1, last_pair.first);
            double current_dist2 = distance(point2, last_pair.second);

            double swap_dist1 = distance(point2, last_pair.first);
            double swap_dist2 = distance(point1, last_pair.second);

            if ((swap_dist1 + swap_dist2) < (current_dist1 + current_dist2)) {
                std::swap(point1, point2);
            }
        }

        // 插值平滑处理，避免跳变过大
        double alpha = 0.20;  // 插值系数，值越小平滑效果越强
        point1 = alpha * point1 + (1.0 - alpha) * last_pair.first;
        point2 = alpha * point2 + (1.0 - alpha) * last_pair.second;
        last_pair = std::make_pair(point1, point2);

        return last_pair;
    }

    std::pair<Eigen::Vector3d, double> getFarthestPointinRangewithYaw(const Eigen::Vector3d& current_odom,const double range)
     {
         if (buffer.empty()) {
            throw std::runtime_error("Buffer is empty");
        }
        Eigen::Vector3d farthest_point;
        double max_distance = std::numeric_limits<double>::min();
        size_t farthest_index = 0; // Track the index of the farthest point
        bool found=false;
        for (size_t i = 0; i < buffer.size(); ++i) {
            double dist = distance(current_odom, buffer[i]);
            if (dist > max_distance&&dist <= range) {
                max_distance = dist;
                farthest_point = buffer[i];
                found=true;
                farthest_index = i; // Update the index of the farthest point
            }
        }
        if(getsize()==BUFFER_SIZE && !found)
        {
            ROS_ERROR_STREAM("xxxxxxx No point in range found xxxxxxxxxxxx");
            // std::cout<<"xxxxxxx No point in range found xxxxxxxxxxxx"<<std::endl;
            // throw std::runtime_error("No point in range found");
            return std::make_pair(last_farthest_point,last_farthest_point_yaw);
        }
        double alpha_pos = 1.0;  // 位置不插值，尽量跟踪紧
        double alpha_yaw = 1.0;  // 外面旋转矩阵差值效果更好
        last_farthest_point=alpha_pos*farthest_point+(1.0-alpha_pos)*last_farthest_point;
        last_farthest_point_yaw=alpha_yaw*buffer_yaw[farthest_index]+(1.0-alpha_yaw)*last_farthest_point_yaw;
        // // Normalize the  yaw to the range -pi to pi
        // if(last_farthest_point_yaw<=M_PI&&last_farthest_point_yaw>=0)
        // {
        //     if(buffer_yaw[farthest_index]<=M_PI&&buffer_yaw[farthest_index]>=0)
        //     {
        //     last_farthest_point_yaw=alpha_yaw*buffer_yaw[farthest_index]+(1.0-alpha_yaw)*last_farthest_point_yaw;
        //     }
        //     else
        //     {
        //         last_farthest_point_yaw=alpha_yaw*(buffer_yaw[farthest_index]+2*M_PI)+(1.0-alpha_yaw)*last_farthest_point_yaw;   
        //     }
        // }
        // else//last_farthest_point_yaw<=0 && last_farthest_point_yaw>= -M_PI
        // {
        //     if(buffer_yaw[farthest_index]<=M_PI&&buffer_yaw[farthest_index]>=0)
        //     {
        //     last_farthest_point_yaw=alpha_yaw*(buffer_yaw[farthest_index]-2*M_PI)+(1.0-alpha_yaw)*last_farthest_point_yaw;
        //     }
        //     else
        //     {
        //         last_farthest_point_yaw=alpha_yaw*(buffer_yaw[farthest_index])+(1.0-alpha_yaw)*last_farthest_point_yaw;   
        //     }
        // }
        // last_farthest_point_yaw = atan2(sin(last_farthest_point_yaw), cos(last_farthest_point_yaw));// Normalize the yaw to the range -pi to pi
        // last_farthest_point_yaw = buffer_yaw[farthest_index];
        return std::make_pair(last_farthest_point,last_farthest_point_yaw);

    }
    Eigen::Vector3d getFarthestPointinRange(const Eigen::Vector3d& current_odom,const double range) {
        if (buffer.empty()) {
            throw std::runtime_error("Buffer is empty");
        }
        Eigen::Vector3d farthest_point;
        double max_distance = std::numeric_limits<double>::min();
        bool found=false;
        for (const auto& odom : buffer) {
            double dist = distance(current_odom, odom);
            if (dist > max_distance&&dist <= range) {
                max_distance = dist;
                farthest_point = odom;
                found=true;
            }
        }
        if(getsize()==BUFFER_SIZE &&!found)
        {
             ROS_ERROR_STREAM("xxxxxxx No point in range found xxxxxxxxxxxx");
             return last_farthest_point;
            // std::cout<<"xxxxxxx No point in range found xxxxxxxxxxxx"<<std::endl;
            // throw std::runtime_error("No point in range found");
        }
        double alpha = 1.0;  //位置暂时不插值
        last_farthest_point=alpha*farthest_point+(1.0-alpha)*last_farthest_point;
        return last_farthest_point;
    }
//修改为odom planning的时候初始odom而非简单为0Eigen::Vector3d::Zero()
private:
    std::deque<Eigen::Vector3d> buffer;
    std::deque<double> buffer_yaw;
    Eigen::Vector3d last_farthest_point=Eigen::Vector3d::Zero();
    double last_farthest_point_yaw=0.0;
     std::pair<Eigen::Vector3d, Eigen::Vector3d> last_pair = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};//init odom TODO
};

}
#endif