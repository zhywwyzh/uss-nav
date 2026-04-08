roslaunch mavros px4.launch & sleep 10;

rostopic echo /mavros/battery/voltage;

wait;
