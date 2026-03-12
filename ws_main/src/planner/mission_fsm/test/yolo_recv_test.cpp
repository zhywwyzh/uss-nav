//
// Created by gwq on 7/14/25.
//
#include <ros/ros.h>
#include <skeleton_generation/object_factory.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "yolo_recv_test");
    ros::NodeHandle nh("~");
    ObjectFactory obj_factory(nh);

    ros::spin();
    return 0;
}