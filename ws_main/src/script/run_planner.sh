# ODOM_NAME=/relative_loc/revised_odom
ODOM_NAME=/ekf_quat/ekf_odom
source /home/nv/GNF_WS/devel/setup.zsh & sleep 1;
roslaunch ego_planner run_in_exp.launch odom_topic:=$ODOM_NAME & sleep 1;
# bash /home/nv/GNF_WS/src/script/record_light.sh & sleep 1;
wait; 

