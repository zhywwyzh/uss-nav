//
// Created by gwq on 7/17/25.
//

#include <ros/ros.h>
#include "../include/scene_graph/object_factory.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "yolo_recv_test");
    ros::NodeHandle nh("~");
    ObjectFactory obj_factory(nh);
    obj_factory.runThisModule();
    ros::spin();
    return 0;
}