#!/bin/bash

# 发布一次 quadrotor_msgs/Instruction 到 /instruction
rostopic pub -1 /bridge/Instruct quadrotor_msgs/Instruction "
robot_id: 1
to_drone_ids: [2, 3]
header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'world'

instruction_type: 5

target_obj_id: 0
command: 'go to target'

goal:
  - {x: 1.0, y: 2.0, z: 1.5}
  - {x: 2.0, y: 3.0, z: 1.5}
yaw: [0.0, 1.57]
look_forward: true
goal_to_follower: false

enable: false
has_target_position: false
target_position: {x: 0.0, y: 0.0, z: 0.0}

nav_waypoint:
  - {x: 5.0, y: 6.0, z: 1.2}
  - {x: 7.0, y: 8.0, z: 1.2}
nav_yaw: [0.0, 0.5]

image:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  height: 0
  width: 0
  encoding: ''
  is_bigendian: 0
  step: 0
  data: []

image_camera: ''
classes: [1, 2]
global_poses:
  - {x: 10.0, y: 11.0, z: 1.0}
  - {x: 12.0, y: 13.0, z: 1.0}

map_folder: ''
"