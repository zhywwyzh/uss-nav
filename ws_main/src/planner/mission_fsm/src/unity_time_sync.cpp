#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Odometry.h>

// 通用消息转发器类模板
template<typename T>
class TimeSyncMessageForwarder {
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    message_filters::Subscriber<T> sub_;
    std::string input_topic_;
    std::string output_topic_;
    int queue_size_;

public:
    TimeSyncMessageForwarder(const std::string& input_topic, const std::string& output_topic, int queue_size = 10)
        : nh_("~"), input_topic_(input_topic), output_topic_(output_topic), queue_size_(queue_size) {
        
        sub_.subscribe(nh_, input_topic_, queue_size_);
        pub_ = nh_.advertise<T>(output_topic_, queue_size_);
        sub_.registerCallback(boost::bind(&TimeSyncMessageForwarder::callback, this, _1));
        
        std::cout << "Forwarding " << input_topic_ << " to " << output_topic_<< std::endl;
        std::cout << "Using receive time for header.stamp\n" << std::endl;
    }

    void callback(const typename T::ConstPtr& msg) {
        ros::Time receive_time = ros::Time::now();
        typename T::Ptr new_msg = boost::make_shared<T>(*msg);
        new_msg->header.stamp = receive_time;
        pub_.publish(new_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "message_time_sync_forwarder");
    ros::NodeHandle nh("~");
    
    // 创建转发器实例（根据需要修改消息类型）
    TimeSyncMessageForwarder<sensor_msgs::PointCloud2> 
                lidar("/livox/lidar", "/livox/lidar_sync",10);
    // TimeSyncMessageForwarder<sensor_msgs::CompressedImage>
    //             color("/camera/color/image/compressed", "/camera/color/image_sync/compressed", 10);
    // TimeSyncMessageForwarder<sensor_msgs::CompressedImage>
    //             depth("/camera/depth/image/compressed", "/camera/depth/image_sync/compressed", 10);
    TimeSyncMessageForwarder<nav_msgs::Odometry>
                odom("/unity_odom", "/unity_odom_sync", 10);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}