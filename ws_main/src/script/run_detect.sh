#!/bin/bash
source /home/nv/qiuzh/oak_ws/devel/setup.bash
roslaunch oakcam_ffc_4p_ros oakcam_ffc_4p.launch fps:=2 & sleep 5; 


source /home/nv/zht_ws/devel/setup.bash
roslaunch py_yolov8 yolov8.launch & sleep 5;


source /home/nv/zht_ws/devel/setup.bash
roslaunch target_dist_calculator target_dist.launch & sleep 1;

wait
