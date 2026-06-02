# uss-nav

`uss-nav` 是一个面向无人机自主探索、目标感知、语义场景理解和多机通信的混合工作区。当前目录下主要包含两部分：

- `ws_main`：ROS1 catkin 工作区，负责规划、建图/仿真接口、任务状态机、场景图、多机桥接和常用脚本。
- `yoloe`：独立的视觉模型子项目，提供 YOLOE 与 MobileCLIP 相关代码，供 `scene_graph` 中的检测节点调用。

## 0. 前请提要
#### 想尽快开始请直接看章节`5`、`6`、`10`

## 1. 工作区结构

```text
uss-nav/
├── ws_main/                     # ROS catkin 工作区
│   ├── src/
│   │   ├── planner/
│   │   │   ├── ego_plannerv3/   # 局部规划、探索主流程、轨迹生成
│   │   │   ├── exploration/     # 探索管理、主动感知、感知工具
│   │   │   ├── mission_fsm/     # 任务状态机、RViz/Unity 辅助、多机启动入口
│   │   │   ├── scene_graph/     # 语义目标融合、场景图、骨架生成、LLM 接口
│   │   │   └── uav_simulator/   # 仿真器、地图生成、深度/点云模拟
│   │   ├── network/
│   │   │   └── NetBridgeForSwarm/  # 多机 ROS Topic/Service/Image 桥接
│   │   ├── unity_utils/         # Unity ROS-TCP 相关工具
│   │   ├── utils/               # 通用消息、RViz 插件、姿态/轨迹工具
│   │   └── script/              # 一键启动、起降、录包、排障脚本
│   ├── build/ devel/ logs/      # catkin 编译产物
│   └── .catkin_tools/           # catkin tools 配置痕迹
└── yoloe/                       # YOLOE 视觉模型代码
```

## 2. 核心模块说明

| 模块 | 位置 | 作用 |
| --- | --- | --- |
| `ego_planner` | `ws_main/src/planner/ego_plannerv3/plan_manage` | 主规划入口，接收里程计、点云/深度、目标信息，输出轨迹与控制指令 |
| `plan_env` / `path_searching` / `traj_opt` | `ws_main/src/planner/ego_plannerv3/*` | 栅格地图、搜索、轨迹优化等基础能力 |
| `exploration_manager` | `ws_main/src/planner/exploration/exploration_manager` | 探索逻辑、前沿点选择、目标/探索模式切换 |
| `scene_graph` | `ws_main/src/planner/scene_graph` | 目标检测结果融合、对象点云、场景图、骨架/区域结构、LLM 交互 |
| `mission_fsm` | `ws_main/src/planner/mission_fsm` | 任务状态、Unity/点云同步、桥接启动入口、RViz 配置 |
| `uav_simulator` | `ws_main/src/planner/uav_simulator` | 仿真无人机、地图和深度/点云渲染 |
| `swarm_ros_bridge` | `ws_main/src/network/NetBridgeForSwarm/swarm_ros_bridge` | 多机 ROS Topic/Service/Image 转发 |
| `quadrotor_msgs` / `traj_utils` | `ws_main/src/utils/*` | 自定义消息、轨迹/命令类型定义 |
| `yoloe` | `yoloe/` | YOLOE、MobileCLIP 和模型推理代码 |

## 3. 主要功能

- 基于局部地图和里程计的自主规划与探索
- RViz 交互式目标下发
- 真实机链路中的里程计、点云、控制指令对接
- 基于 YOLOE 的开放词表目标检测与分割
- 目标点云融合、对象级语义建图、场景图生成
- 可选的 LLM 场景问答/区域分类接口
- 基于 `swarm_ros_bridge` 的多机通信
- Unity 联调与时间同步

## 4. 启动模式总览

| 模式 | 推荐入口 | 是否只依赖本仓库 | 说明 |
| --- | --- | --- | --- |
| 仿真最小闭环 | `roslaunch ego_planner obj_nav.launch` | 是 | 使用仓库内仿真器和规划器，最适合首次验证 |
| 仿真 RViz | `roslaunch mission_fsm rviz.launch` | 是 | 配合仿真启动，使用仓库内现成 RViz 配置和 `2D Nav Goal` 下发目标 |
| 实机未知地图一键启动 | `bash ws_main/src/script/run_with_unknown_map.sh` | 否 | 依赖 `mavros`、`fast_lio`、`ekf_quat`、`px4ctrl` 等外部包 |
| 实机已知地图一键启动 | `bash ws_main/src/script/run_with_known_map.sh` | 否 | 在未知地图链路基础上额外依赖已有地图和检测链路 |
| 追加巡逻节点 | `bash ws_main/src/script/run_patrol_node.sh` | 否 | 启动真实机目标巡逻入口 |
| YOLOE 检测服务 | `yoloe_predict_server_realworld.py` / `yoloe_server.py` | 代码在仓库内，模型文件不在仓库内 | 需要 CUDA、PyTorch、YOLOE 权重、MobileCLIP 权重 |
| 多机桥接 | `roslaunch mission_fsm bridge_drone.launch` | 是 | 运行前需要先改桥接 YAML 和 hostname |

## 5. 环境准备

### 5.1 建议系统环境

从仓库内容判断，这套代码更适合以下环境：

- Ubuntu 20.04
- ROS Noetic
- Python 3
- NVIDIA GPU + CUDA（仅检测/视觉推理链路必须）

说明：

- `NetBridgeForSwarm` 中包含 `cv_bridge_noetic_fit_version`，说明作者明确考虑了 Noetic 适配。
- 部分历史文档仍来自更早版本的 Fast-Planner，和当前工作区不完全一致，优先以当前仓库源码为准。

### 5.2 ROS 与系统依赖

`ws_main/src` 下的 CMakeLists 显示，至少需要准备这些基础依赖：

- `catkin`
- `Eigen3`
- `PCL`
- `OpenCV`
- `Armadillo`
- `OpenMP`
- `Qt5` / `rviz` 开发依赖
- `GLEW`、`OpenGL`、`glfw3`
- `JPEG`
- `zmqpp`
- `igraph >= 0.10`，且需要能被 `find_package(igraph 0.10 CONFIG REQUIRED)` 找到

常见 ROS 运行依赖包括：

- `mavros`
- `cv_bridge`
- `tf`
- `rviz`
- `pcl_ros`
- `message_filters`
- `topic_tools`
- `image_transport`

如果只想先跑仓库内仿真，优先保证：

- ROS Noetic 基础环境
- `rviz`
- `PCL`
- `Eigen3`
- `OpenCV`
- `Armadillo`

### 5.3 真实机链路的仓库外依赖

`ws_main/src/script` 中的实机脚本并不是“纯本仓库自洽”的，它们依赖很多工作区外部包或设备驱动。至少包括：

- `mavros`
- `fast_lio`
- `ekf_quat`
- `px4ctrl`
- `incremental_map_publisher`
- `drone_node`
- `oakcam_ffc_4p_ros`
- `py_yolov8`
- `target_dist_calculator`

另外，多处脚本还存在硬编码路径，例如：

- `/home/nv/GNF_WS/...`
- `/home/nv/zht_ws/...`
- `/home/nv/qiuzh/oak_ws/...`
- `/home/nv/detection_ws/...`

因此，如果你直接在当前机器上运行这些脚本，通常需要先把脚本内路径改成你的实际环境。

### 5.4 YOLOE Python 依赖

如果装不清楚，直接上网搜索YOLOE，看官方安装教程

`yoloe/requirements.txt` 当前包含：

```txt
-e .
-e third_party/lvis-api
-e third_party/ml-mobileclip
-e third_party/CLIP
```

建议单独建立 Python 环境：

```bash
cd /home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav
python3 -m venv .venv-yoloe
source .venv-yoloe/bin/activate
pip install --upgrade pip
pip install -r yoloe/requirements.txt
```

检测脚本还要求：

- `torch` 且可用 CUDA
- `ultralytics`
- `mobileclip`
- YOLOE 模型权重
- `mobileclip_blt.pt`

注意：当前仓库里只有 `yoloe/prompt/prompt.txt`，模型权重文件并未随仓库提供。

## 6. 编译方法

当前 `ws_main` 是标准 catkin workspace `catkin_make`：

```bash
cd /home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

每开一个新终端，都需要重新执行：

```bash
source devel/setup.bash
```

## 7. 快速开始

### 7.1 可运行方案：仿真 + RViz

终端 1：

```bash
cd /home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main
source devel/setup.bash
roslaunch mission_fsm rviz.launch
```

终端 2：

```bash
cd /home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main
source devel/setup.bash
roslaunch ego_planner obj_nav.launch
```

启动后：

- 在 RViz 中确认能看到地图、轨迹和无人机模型
- 使用工具栏中的 `2D Nav Goal`
- 在地图上点击一个目标点
- 规划器会通过 `/move_base_simple/goal` 接收目标并开始规划

`obj_nav.launch` 会进一步 include：

- `run_in_sim.xml`
- `advanced_param_sim.xml`
- `simulator.xml`

这意味着它会同时拉起规划主节点、仿真传感器和相关参数。

说明：

- `ego_planner` 目录下虽然也有 `rviz.launch`，但它当前指向的 `launch/include/default.rviz` 在仓库里缺失
- 因此这里推荐使用 `mission_fsm rviz.launch`

### 7.2 仓库内启动真实机规划入口

如果你的外部定位、点云和控制器都已经准备好，可以只使用本仓库的规划部分。

典型输入至少要满足：

- 里程计：`/ekf_quat/ekf_odom`
- 点云：`/cloud_fov_limited`
- 控制指令接收：外部控制器订阅 `/setpoint_cmd`

规划入口命令：

```bash
cd /home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch ego_planner obj_nav_real.launch odom_topic:=/ekf_quat/ekf_odom
```

这个启动文件会使用 `advanced_param_real.xml`，其中最关键的 remap 是：

- `~grid_map/odom -> /ekf_quat/ekf_odom`
- `~grid_map/cloud -> /cloud_fov_limited`
- `/position_cmd -> /setpoint_cmd`
- `/tracking_target -> /target_dist_calculator/detect_out`

也就是说，真实机链路里你通常还需要：

- 外部定位与建图
- 视场过滤后的点云
- 目标检测输出
- 外部控制器

## 8. 功能使用教程

### 8.1 在 RViz 中下发目标

本仓库规划器默认监听 `/move_base_simple/goal`。

操作方法：

1. 启动 RViz。
2. 选择 `2D Nav Goal` 工具。
3. 在地图中点击并拖拽设置目标位姿。
4. 规划器收到目标后开始生成轨迹。

适用场景：

- 仿真 `obj_nav.launch`
- 真实机 `obj_nav_real.launch`

### 8.3 发送高层任务指令

向 `/bridge/Instruct` 发送高层任务命令。

`1: TURN_OBJECT_NAV`                # 全功能探索
`2: TURN_OBJECT_ID_NAV`             # 给定物体id导航
`3: TURN_REGULAR_EXPLORATION`       # 纯建图探索
`4: TURN_DF_DEMO`                   # demo演示，给prompt
`5: TURN_GOAL`                      # vla的点到点导航
`6: TURN_TRACKING`                  # Tracking
`7: TURN_WAYPOINT_NAV`              # 给定目标点导航


上面这条命令的含义是：给 `robot_id=0` 发送一条“探索”指令。消息类型`quadrotor_msgs/Instruction`

- 详情请见：`ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg`和`instruction_description.md`
- 可使用脚本`bash ws_main/scripts/pub_instrucion.sh`快捷发布

### 8.5 启动 YOLOE 检测服务

仓库中有两个主要检测脚本：

- `yoloe/predict_realtime_cam_sim.py`
  - 仿真，默认话题是：
    - RGB：`/camera/color/image/compressed`
    - Depth：`/camera/depth/image/compressed`
    - Odom：`/unity_depth_odom`
- `yoloe/predict_realtime_cam_real.py`
  - 实机，默认话题是：
    - RGB：`/camera/color/image_raw/compressed`
    - Depth：`/camera/aligned_depth_to_color/image_raw/compressedDepth`
    - Odom：`/ekf_quat/ekf_odom`

推荐真实机场景的启动方式：

```bash
cd /home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav
source ws_main/devel/setup.bash
source .venv-yoloe/bin/activate

python3 yoloe/predict_realtime_cam_sim.py
```

### 8.6 LLM 场景问答接口

`scene_graph/scripts/LLM_interface.py` 与 `LLM_interface_thread.py` 用于处理：

- `/scene_graph/prompt`
- `/scene_graph/llm_ans`

但当前脚本存在几个明显问题：

- API key 和模型地址写死在源码里
- 默认供应商与当前你的生产环境可能不一致
- 没有做敏感信息外置

因此建议在使用前先修改：

- API key
- base URL
- model name

再运行：

```bash
cd /home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main
source devel/setup.bash
python3 src/planner/scene_graph/scripts/LLM_interface_thread.py
```

# 9 完整仿真启动流程

## 终端1：启动rviz
```bash
source ws_main/devel/setup.bash
roslaunch mission_fsm rviz.launch
```

## 终端2：启动主程序
```bash
# 在使用程序之前，请注意调整odom, cloud, rgb, depth等话题的名称，不同仿真器、不同设置下这些话题名会存在差异
source ws_main/devel/setup.bash
roslaunch ego_planner obj_nav.launch
```

## 终端3：启动yoloe
```bash
source ws_main/devel/setup.bash
source .venv/bin/activate
cd yoloe
python predict_realtime_cam_sim.py
```

## 终端4：启动scene-graph
```bash
source ws_main/devel/setup.bash
python ws_main/src/planner/scene_graph/scripts/LLM_interface_thread.py
```

## 终端5：功能发布
```bash
source ws_main/devel/setup.bash
按照
```