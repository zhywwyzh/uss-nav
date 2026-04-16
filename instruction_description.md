# Instruction Description

本文档基于以下两处源码整理：

- [ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg:1)
- [ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1523)

目标是明确说明：

- 每个 `instruction_type` 在当前 FSM 实现里实际会读取哪些字段
- 哪些字段必须发布，哪些字段可选，哪些字段当前完全没有被使用
- 每种指令进入 FSM 后会触发什么行为

## 1. 先说结论

`Instruction.msg` 字段很多，但 `fast_exploration_fsm.cpp` 当前真正使用的字段并不多。

当前 `instructionCallback()` 实际会读取的字段只有：

- `robot_id`
- `instruction_type`
- `target_obj_id`
- `command`
- `goal`
- `yaw`
- `look_forward`
- `enable`
- `has_target_position`
- `target_position`
- `nav_waypoint`
- `nav_yaw`
- `global_poses`
- `map_folder`

当前完全没有在这个 FSM 回调里使用的字段：

- `to_drone_ids`
- `header`
- `goal_to_follower`
- `image`
- `image_camera`
- `classes`

此外，`instruction_type = 8` 在 `Instruction.msg` 里没有定义，也没有对应分支。

## 2. 指令入口和总规则

### 2.1 实际订阅的话题

FSM 订阅的是：

- `/bridge/Instruct`

见：

- [fast_exploration_fsm.cpp:127](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:127)

所以如果你想直接打到这个 FSM，上游应发布到 `/bridge/Instruct`。  
如果你发布到别的话题，例如 `/instruction`，只有在系统其他部分做了转发的前提下，这个 FSM 才能收到。

### 2.2 `robot_id` 必须匹配

第一层过滤就是：

```cpp
if (msg->robot_id != md_->drone_id_) return;
```

见：

- [fast_exploration_fsm.cpp:1525](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1525)

因此：

- `robot_id` 必填
- 且必须等于当前无人机的 `fsm/drone_id`
- 不匹配会被直接丢弃
- 默认填写`0`或`1`即可

### 2.3 指令频率限制

除以下 3 类指令外，其余指令 0.8 秒内连续发送会被丢弃一次：

- `TURN_GOAL`
- `TURN_WAYPOINT_NAV`
- `TURN_TRACKING`

见：

- [fast_exploration_fsm.cpp:1529](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1529)
- [fast_exploration_fsm.cpp:1536](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1536)

### 2.4 INIT/WARM_UP 状态下的行为

除了以下两类指令，其他指令在 `INIT` 或 `WARM_UP` 状态都会直接返回：

- `TURN_LOAD_SCENE_GRAPH`
- `REQUEST_ALL_AREA_AND_OBJS`

见：

- [fast_exploration_fsm.cpp:1548](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1548)
- [fast_exploration_fsm.cpp:1573](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1573)
- [fast_exploration_fsm.cpp:1580](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1580)

## 3. `Instruction.msg` 字段功能速查

来自：

- [Instruction.msg:1-48](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg:1)

| 字段 | 当前 FSM 是否读取 | 用途 |
| --- | --- | --- |
| `robot_id` | 是 | 目标无人机 ID，必须匹配本机 |
| `to_drone_ids` | 否 | 当前回调不使用 |
| `header` | 否 | 当前回调不使用 |
| `instruction_type` | 是 | 决定进入哪个分支 |
| `target_obj_id` | 是 | 用于 `TURN_OBJECT_ID_NAV` |
| `command` | 是 | 用于 `TURN_OBJECT_NAV`、`TURN_DF_DEMO` |
| `goal` | 是 | 用于 `TURN_GOAL` |
| `yaw` | 是 | 用于 `TURN_GOAL`，只读第一个元素 |
| `look_forward` | 是 | 用于 `TURN_GOAL` |
| `goal_to_follower` | 否 | 当前回调不使用 |
| `enable` | 是 | 用于 `TURN_TRACKING` |
| `has_target_position` | 是 | 用于 `TURN_TRACKING` |
| `target_position` | 是 | 用于 `TURN_TRACKING` |
| `nav_waypoint` | 是 | 用于 `TURN_WAYPOINT_NAV`，只读第一个元素 |
| `nav_yaw` | 是 | 用于 `TURN_WAYPOINT_NAV`，只读第一个元素 |
| `image` | 否 | 当前回调不使用 |
| `image_camera` | 否 | 当前回调不使用 |
| `classes` | 否 | 当前回调不使用 |
| `global_poses` | 是 | 用于 `TURN_TRACKING` |
| `map_folder` | 是 | 用于 `TURN_SAVE_SCENE_GRAPH`、`TURN_LOAD_SCENE_GRAPH` |

## 4. 每个 `instruction_type` 的详细说明

### 4.1 `TURN_OBJECT_NAV = 1`

定义位置：

- [Instruction.msg:33](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg:33)

处理分支：

- [fast_exploration_fsm.cpp:1610-1618](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1610)

#### 必须发布的内容

- `robot_id`
- `instruction_type = 1`
- `command`

#### 可选但当前分支不读的内容

- 其他全部字段

#### 作用

这条指令用于“按自然语言目标做语义探索”。FSM 会：

- 关闭 regular exploration 模式
- 关闭 terminate target 模式
- 标记 `new_topo_need_predict_immediately_ = true`
- 关闭 DF demo 模式
- 把 `msg->command` 写入 `fd_->target_cmd_`
- 调用 `scene_graph_->setTargetAndPriorKnowledge(...)`
- 切换到 `MISSION_FSM_STATE::LLM_PLAN_EXPLORE`

#### 对 `command` 的要求

代码里没有做空字符串检查，但从语义上讲 `command` 应该是这条指令的核心输入。  
如果为空，FSM 仍会切状态，但后续 LLM/场景图逻辑大概率没有有效目标。

#### 最小示例

```yaml
robot_id: 0
instruction_type: 1
command: "find the toilet"
```

### 4.2 `TURN_OBJECT_ID_NAV = 2`

定义位置：

- [Instruction.msg:34](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg:34)

处理分支：

- [fast_exploration_fsm.cpp:1585-1590](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1585)

#### 必须发布的内容

- `robot_id`
- `instruction_type = 2`
- `target_obj_id`

#### 作用

这条指令用于“直接导航到某个已知对象 ID”。FSM 会：

- 将 `fd_->object_target_id_ = msg->target_obj_id`
- 将 `fd_->go_object_process_phase = 0`
- 关闭 terminate target 模式
- 切换到 `MISSION_FSM_STATE::GO_TARGET_OBJECT`

#### 注意

- 代码不会在这个入口处校验 `target_obj_id` 是否真实存在
- 如果 ID 不存在，后续 `GO_TARGET_OBJECT` 状态中的路径生成才会失败

#### 最小示例

```yaml
robot_id: 0
instruction_type: 2
target_obj_id: 15
```

### 4.3 `TURN_REGULAR_EXPLORATION = 3`

定义位置：

- [Instruction.msg:35](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg:35)

处理分支：

- [fast_exploration_fsm.cpp:1620-1625](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1620)

#### 必须发布的内容

- `robot_id`
- `instruction_type = 3`

#### 作用

这条指令用于“纯建图探索，不带语义目标”。FSM 会：

- `fd_->regular_explore_ = true`
- `fd_->df_demo_mode_ = false`
- `fd_->find_terminate_target_mode_ = false`
- 切换到 `MISSION_FSM_STATE::PLAN_EXPLORE`

#### 最小示例

```yaml
robot_id: 0
instruction_type: 3
```

### 4.4 `TURN_DF_DEMO = 4`

定义位置：

- [Instruction.msg:36](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg:36)

处理分支：

- [fast_exploration_fsm.cpp:1627-1637](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1627)

#### 必须发布的内容

- `robot_id`
- `instruction_type = 4`
- `command`

#### 作用

这条指令用于 demo 流程。FSM 会：

- 开启 DF demo 模式
- 将 `df_demo_phase_` 置 0
- 将 `explore_count_` 置 0
- 将 `df_demo_target_id_` 置为 `-100`
- 将所有 `areas_need_predict_` 重新标为 `true`
- 将 `msg->command` 写入 `fd_->target_cmd_`
- 调用 `scene_graph_->setTargetAndPriorKnowledge(...)`
- 切换到 `MISSION_FSM_STATE::DF_DEMO`

#### 对 `command` 的要求

和 `TURN_OBJECT_NAV` 一样，逻辑上应提供自然语言目标。

#### 最小示例

```yaml
robot_id: 0
instruction_type: 4
command: "demonstrate how to find a chair"
```

### 4.5 `TURN_GOAL = 5`

定义位置：

- [Instruction.msg:37](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg:37)

处理分支：

- [fast_exploration_fsm.cpp:1639-1641](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1639)
- [fast_exploration_fsm.cpp:153-183](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:153)
- [fast_exploration_fsm.cpp:1363-1376](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1363)

#### 必须发布的内容

- `robot_id`
- `instruction_type = 5`
- `goal` 至少包含 1 个点

#### 可选内容

- `yaw`
- `look_forward`

#### 字段读取细节

- 只读取 `goal.front()`
- 只读取 `yaw.front()`
- 若 `goal` 为空，整条指令被忽略
- 若 `yaw` 为空，默认使用当前机体 `odom_yaw`
- 若 `goal[0].z` 不是有限数，默认高度用 `1.0`

#### 作用

收到后 FSM 会：

1. 关闭 tracking 触发与 tracking 初始化状态
2. 清除当前 tracking target
3. 如果当前处于 tracking 状态，先 `stopMotion()`
4. 切换到 `WAIT_TRIGGER`
5. 直接调用 `pubLocalGoal(...)`

`pubLocalGoal()` 最终发布的是：

- 话题：`local_goal`
- 消息类型：`quadrotor_msgs/EgoGoalSet`

其中：

- `goal = goal.front()`
- `yaw = yaw.front()` 或当前机头朝向
- `look_forward = msg->look_forward`
- `yaw_low_speed = false`

#### 当前不使用的相关字段

- `goal_to_follower` 在这个分支完全没用

#### 最小示例

```yaml
robot_id: 0
instruction_type: 5
goal:
  - {x: 3.0, y: 1.5, z: 1.2}
yaw: [1.57]
look_forward: true
```

### 4.6 `TURN_TRACKING = 6`

定义位置：

- [Instruction.msg:38](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg:38)

处理分支：

- [fast_exploration_fsm.cpp:1643-1678](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1643)
- 跟踪目标更新辅助逻辑见 [fast_exploration_fsm.cpp:185-235](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:185)

#### 这类指令有两种用法
暂未开放

### 4.7 `TURN_WAYPOINT_NAV = 7`

定义位置：

- [Instruction.msg:39](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg:39)

处理分支：

- [fast_exploration_fsm.cpp:1592-1608](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1592)

#### 必须发布的内容

- `robot_id`
- `instruction_type = 7`
- `nav_waypoint` 至少 1 个点

#### 可选内容

- `nav_yaw`

#### 字段读取细节

- 只读取 `nav_waypoint.front()`
- 只读取 `nav_yaw.front()`
- 如果 `nav_waypoint` 为空，FSM 直接切到 `WAIT_TRIGGER`
- 如果 `nav_yaw` 为空，则使用当前 `odom_yaw`
- 如果 `nav_waypoint[0].z` 非有限数，则高度默认 `1.0`

#### 作用

FSM 会：

- 保存首个 waypoint 到 `fd_->waypoint_target_`
- 保存首个 yaw 到 `fd_->waypoint_target_yaw_`
- 将 `go_waypoint_process_phase = 0`
- 关闭 terminate target 模式
- 切换到 `MISSION_FSM_STATE::GO_TARGET_WITH_WAYPOINT`

#### 最小示例

```yaml
robot_id: 0
instruction_type: 7
nav_waypoint:
  - {x: 8.0, y: 3.0, z: 1.5}
nav_yaw: [0.0]
```

### 4.8 `TURN_SAVE_SCENE_GRAPH = 9`

定义位置：

- [Instruction.msg:44](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg:44)

处理分支：

- [fast_exploration_fsm.cpp:1680-1688](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1680)

#### 必须发布的内容

- `robot_id`
- `instruction_type = 9`
- `map_folder`

#### 作用

调用：

- `scene_graph_->saveMap(msg->map_folder)`

只负责保存，不切换 FSM 状态。

#### 注意

- 这条指令在 `INIT/WARM_UP` 状态会被直接忽略，因为它走的是 `switch`，而 `switch` 之前有初始化状态返回

#### 最小示例

```yaml
robot_id: 0
instruction_type: 9
map_folder: "snapshot_20260416_test"
```

### 4.9 `TURN_LOAD_SCENE_GRAPH = 10`

定义位置：

- [Instruction.msg:45](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg:45)

处理分支：

- [fast_exploration_fsm.cpp:1548-1572](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1548)

#### 必须发布的内容

- `robot_id`
- `instruction_type = 10`
- `map_folder`

#### 作用

调用：

- `scene_graph_->loadMap(msg->map_folder)`

如果加载成功，还会做一整套状态清理：

- 清空路径缓存
- 清掉 trigger、regular explore、df demo、terminate target 等模式标记
- 重置 LLM 探索计数和区域决策标志
- `hardResetExploreArea(false)`
- 重启 object factory 线程
- 刷新已加载地图的可视化
- 最后切换到 `WAIT_TRIGGER`

#### 特点

- 这条指令在 `INIT/WARM_UP` 时也能生效，因为它在初始化状态检查之前处理

#### 最小示例

```yaml
robot_id: 0
instruction_type: 10
map_folder: "snapshot_20260329_183344"
```

### 4.10 `REQUEST_ALL_AREA_AND_OBJS = 11`

定义位置：

- [Instruction.msg:48](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/utils/quadrotor_msgs/msg/Instruction.msg:48)

处理分支：

- [fast_exploration_fsm.cpp:1573-1577](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1573)

#### 必须发布的内容

- `robot_id`
- `instruction_type = 11`

#### 作用

FSM 会：

1. 调用 `scene_graph_->DFDemoPromptGen(scene_graph_json_str)`
2. 调用 `scene_graph_->sendSceneGraphJson(scene_graph_json_str)`

也就是说，这条指令不是让飞机运动，而是请求当前场景图中的：

- area 信息
- object 信息

并把结果发送给外部系统，例如 CoPaw。

#### 特点

- 这条指令在 `INIT/WARM_UP` 时也会先执行场景图导出逻辑
- 它不是 `switch` 分支中的 case，而是在前置特殊分支里处理

#### 最小示例

```yaml
robot_id: 0
instruction_type: 11
```

## 5. 哪些字段“看起来有用”但当前没接进 FSM

以下字段在 `Instruction.msg` 中定义了，但当前这个 FSM 回调完全不读：

- `to_drone_ids`
- `header`
- `goal_to_follower`
- `image`
- `image_camera`
- `classes`

这意味着：

- 你可以在消息里带这些字段
- 但 `fast_exploration_fsm.cpp` 当前不会根据它们做任何决策

## 6. 特别注意的实现细节

### 6.1 `TURN_GOAL` 和 `TURN_WAYPOINT_NAV` 都只取第一个点

虽然消息定义是数组：

- `geometry_msgs/Point[] goal`
- `geometry_msgs/Point[] nav_waypoint`

但当前实现只取 `.front()`。

所以：

- 发多个点不会按序执行
- 其余元素会被忽略

### 6.2 `yaw` 和 `nav_yaw` 也只取第一个元素

如果数组为空，代码会回退到当前 `odom_yaw`。

### 6.3 `TURN_GOAL` 会直接发布 `local_goal`

它不是先进一个复杂的 waypoint 状态，而是直接调用 `pubLocalGoal(...)`，发布：

- `quadrotor_msgs/EgoGoalSet`

见：

- [fast_exploration_fsm.cpp:178-182](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:178)
- [fast_exploration_fsm.cpp:1367-1376](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1367)

### 6.4 `TURN_TRACKING` 允许一条消息同时做“开 tracking + 推送当前目标”

只要：

- `enable = true`
- `has_target_position = true`
- `target_position` 合法
- 或者 `global_poses` 非空

就可以让 tracking 立即开始工作。

### 6.5 没有定义的 `instruction_type`

未定义值会走 `default`：

- 切到 `WAIT_TRIGGER`
- 打印错误日志

见：

- [fast_exploration_fsm.cpp:1690-1693](/home/zhywwyzh/workspace/VLA_Diff/ros_ws/uss-nav/ws_main/src/planner/exploration/exploration_manager/src/fast_exploration_fsm.cpp:1690)


然后根据具体 `instruction_type` 只填本文第 4 节列出的必填字段即可。

## 7. 速查

| instruction_type | 必填字段 | 当前作用 |
| --- | --- | --- |
| `1 TURN_OBJECT_NAV` | `robot_id`, `command` | 语义目标探索 |
| `2 TURN_OBJECT_ID_NAV` | `robot_id`, `target_obj_id` | 导航到指定对象 ID |
| `3 TURN_REGULAR_EXPLORATION` | `robot_id` | 纯探索建图 |
| `4 TURN_DF_DEMO` | `robot_id`, `command` | Demo 模式 |
| `5 TURN_GOAL` | `robot_id`, `goal[0]` | 直接发一个本地目标点 |
| `6 TURN_TRACKING` | `robot_id`, `enable` | 开/关 tracking，可附加目标位置 |
| `7 TURN_WAYPOINT_NAV` | `robot_id`, `nav_waypoint[0]` | 导航到指定 waypoint |
| `9 TURN_SAVE_SCENE_GRAPH` | `robot_id`, `map_folder` | 保存场景图快照 |
| `10 TURN_LOAD_SCENE_GRAPH` | `robot_id`, `map_folder` | 加载场景图快照并重置状态 |
| `11 REQUEST_ALL_AREA_AND_OBJS` | `robot_id` | 导出当前 area/object 信息 |

