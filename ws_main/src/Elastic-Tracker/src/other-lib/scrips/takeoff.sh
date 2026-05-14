source ../devel/setup.bash

rostopic pub -1  /l_ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
