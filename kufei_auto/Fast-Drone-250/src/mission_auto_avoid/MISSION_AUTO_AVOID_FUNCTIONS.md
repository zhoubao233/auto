# mission_auto_avoid 功能梳理与迁移清单

本文档基于 WSL 路径：

```text
/home/ubuntu20/DroneFullStack/auto/kufei_auto/Fast-Drone-250/src/mission_auto_avoid
```

目标是梳理 `mission_auto_avoid` 已经实现的功能，方便后续把这些能力迁移到 Orin 上的 `mission_manager`。

## 1. 总体定位

`mission_auto_avoid` 不是一个单纯的航点发布器，而是一个围绕飞控 AUTO mission 的自动避障接管系统。

它做的核心事情是：

1. 从本地 YAML 或 FCU mission 获取当前任务航点。
2. 根据当前里程计和点云判断原 mission 航段前方是否有障碍。
3. 发现障碍后，从 AUTO/RTL 等自动模式临时切到 GUIDED。
4. 启用 EGO-Planner，向 `/ego_input_target` 发布避障目标点。
5. 等待 planner 输出新的、有效的 `/position_cmd` 后允许控制节点接管。
6. 障碍清除并重新接近原 mission 航线后，切回进入避障前的飞控模式。
7. 可选地在回 AUTO 前同步 FCU 当前 mission 序号。

注意：这个包的完整功能依赖同一工程里改过的 EGO-Planner 和 `px4/apm_auto`。如果只迁移 `mission_auto_avoid` 目录本身，避障闭环是不完整的。

## 2. 包内文件

```text
mission_auto_avoid/
  CMakeLists.txt
  package.xml
  config/mission_waypoints.yaml
  launch/mission_auto_avoid.launch
  launch/planner_external_target.launch
  scripts/auto_avoid_manager.py
  scripts/mission_waypoint_uploader.py
  use.md
```

主要实现集中在：

```text
scripts/auto_avoid_manager.py
scripts/mission_waypoint_uploader.py
launch/mission_auto_avoid.launch
launch/planner_external_target.launch
```

## 3. 节点一：auto_avoid_manager.py

这是主状态机节点，节点名为：

```text
auto_avoid_manager
```

### 3.1 主要职责

`auto_avoid_manager.py` 实现以下功能：

1. 读取任务航点，支持两种来源：
   - `mission_source=file`：读取 YAML 或 launch 参数中的本地 ENU 航点。
   - `mission_source=fcu`：读取 `/mavros/mission/waypoints` 中的飞控 mission。

2. 把 FCU global mission 转成本地 ENU：
   - 订阅 `/mavros/global_position/gp_origin` 作为经纬度原点。
   - 支持相对高度和绝对高度 frame。
   - 只处理 `NAV_WAYPOINT`、`NAV_LAND`、`NAV_TAKEOFF` 这几类 mission command。

3. 维护当前目标点：
   - file 模式下按本地航点顺序推进。
   - fcu 模式下按 `WaypointList.current_seq` 找当前或后续目标点。
   - return 模式下把 home 位置作为临时回航避障目标。

4. 订阅点云并做障碍判断：
   - 原始/输入点云：`point_cloud_topic`。
   - 膨胀点云：`inflated_cloud_topic`。
   - 判断原 mission 航段上的障碍。
   - 判断当前机体到目标点局部连线上的障碍。

5. 触发避障：
   - 当 route 或 local 任一方向障碍距离低于阈值时，进入避障。
   - 发布 `/mission_auto_avoid/avoidance_active=true`。
   - 调用 `/mavros/set_mode` 请求进入 `GUIDED`。
   - 调用 planner 的 `set_planning_enabled` 服务打开规划器。

6. 控制目标发布：
   - 发布 `geometry_msgs/PoseStamped` 到 `/ego_input_target`。
   - 避障前期发布原目标点。
   - 偏离原航线较多时，发布 route rejoin 目标点，让飞机先回到原航段附近。

7. 验证 planner 输出：
   - 订阅 `/position_cmd`。
   - 检查 `trajectory_id` 是否是进入避障后的新轨迹。
   - 检查 `trajectory_flag` 是否 ready。
   - 检查输出是否新鲜，是否包含速度、加速度或足够的位置误差。
   - 要求 planner 输出持续有效一段时间后才允许退出避障。

8. 控制避障退出：
   - 障碍清除。
   - 清除状态保持超过 `clear_hold_time`。
   - 已经满足最小避障时间 `avoidance_min_duration`。
   - 已经回到原 mission 航线附近，或 route rejoin 已完成。
   - 当前机体附近膨胀点云安全。
   - planner 输出有效且近期有运动意义。
   - 满足退出 debounce 后，切回原模式。

9. 模式恢复：
   - 避障前是 `AUTO`，结束后回 `AUTO`。
   - 避障前是 `RTL/RTN/AUTO.RTL/AUTO.RTN`，结束后恢复对应返航模式。
   - 如果进入避障时已经是 `GUIDED`，默认恢复到 `AUTO`。

10. 安全门控：
    - 只有 `/mode_select_flag` 非 0 时才允许请求 GUIDED。
    - FCU 未连接时不切模式、不退出避障。
    - odom 超时则禁止 planner 并保持当前状态。
    - shutdown 时发布 `avoidance_active=false` 并关闭 planner。

### 3.2 订阅话题

| 话题 | 类型 | 作用 |
| --- | --- | --- |
| `/mavros/state` | `mavros_msgs/State` | 获取 FCU 连接状态和当前模式 |
| `/mode_select_flag` | `std_msgs/UInt8` | 控制是否允许 GUIDED/避障接管 |
| `odom_topic`，默认 `/mavros/local_position/odom` | `nav_msgs/Odometry` | 当前位姿和速度 |
| `point_cloud_topic` | `sensor_msgs/PointCloud2` | 障碍物点云 |
| `inflated_cloud_topic` | `sensor_msgs/PointCloud2` | EGO-Planner 膨胀障碍点云 |
| `/position_cmd` | `quadrotor_msgs/PositionCommand` | planner 输出，用于判断轨迹是否可接管 |
| `/mavros/global_position/gp_origin` | `geographic_msgs/GeoPointStamped` | FCU mission 经纬度原点 |
| `/mavros/home_position/home` | `mavros_msgs/HomePosition` | return/RTL 模式避障目标 |
| `/mavros/mission/waypoints` | `mavros_msgs/WaypointList` | FCU mission 航点列表 |

### 3.3 发布话题

| 话题 | 类型 | 作用 |
| --- | --- | --- |
| `/ego_input_target` | `geometry_msgs/PoseStamped` | 给改造版 EGO-Planner 的外部目标点 |
| `/mission_auto_avoid/avoidance_active` | `std_msgs/Bool` | 通知控制节点是否允许避障接管 |
| `~closest_distance` | `std_msgs/Float32` | route/local 综合最近障碍距离 |
| `~route_closest_distance` | `std_msgs/Float32` | 原 mission 航段最近障碍距离 |
| `~local_closest_distance` | `std_msgs/Float32` | 当前到目标局部线段最近障碍距离 |
| `~route_distance` | `std_msgs/Float32` | 当前点到原 mission 航段的偏离距离 |
| `~obstacle_ahead` | `std_msgs/Bool` | 当前是否判定前方有障碍 |
| `~planner_cmd_valid` | `std_msgs/Bool` | planner 输出是否满足避障接管/退出条件 |
| `~auto_resume_ready` | `std_msgs/Bool` | 是否已经满足回 AUTO 条件 |

### 3.4 调用服务

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | AUTO/GUIDED/RTL 等模式切换 |
| `/drone_0_ego_planner_node/set_planning_enabled` | `std_srvs/SetBool` | 开关改造版 EGO-Planner |
| `/mavros/mission/set_current` | `mavros_msgs/WaypointSetCurrent` | 可选：回 AUTO 前同步 FCU mission 当前序号 |

### 3.5 障碍检测逻辑

代码里分成两类障碍判断：

1. route 障碍：
   - 线段是原 mission 航段。
   - file 模式下是上一个航点到当前航点。
   - fcu 模式下是 FCU mission 的上一个 nav 点到当前 nav 点。
   - 检查线段前方 `route_lookahead_distance` 范围内，点云到航线的横向距离。

2. local 障碍：
   - 线段是当前机体位置到当前目标点。
   - 检查 `local_lookahead_distance` 范围内的障碍。

触发条件：

```text
route_closest_distance <= route_obstacle_distance_threshold
或
local_closest_distance <= local_obstacle_distance_threshold
```

清除条件：

```text
route_closest_distance >= route_clear_distance_threshold
并且
local_closest_distance >= local_clear_distance_threshold
```

### 3.6 避障状态流程

简化流程如下：

```text
正常 AUTO/RTL mission
  |
  | 发现 route/local 障碍
  v
进入 avoidance_active
  |
  | 请求 GUIDED，启用 planner
  v
等待飞控进入 GUIDED 且速度降到阈值以下
  |
  | 发布 /ego_input_target
  v
等待 planner 产生新的有效 /position_cmd
  |
  | 首个 setpoint 距离合理
  v
允许避障接管
  |
  | 障碍未清除：持续发布目标/重试目标
  | 偏离原航线：发布 route_rejoin 目标
  v
路径清除 + 回到原航段 + planner 稳定 + 当前安全
  |
  | debounce 通过
  v
退出 avoidance_active，恢复原模式
```

### 3.7 return 模式处理

`return_mode_names` 默认包含：

```text
RTL, RTN, AUTO.RTL, AUTO.RTN
```

如果进入避障前处于这些模式，manager 不会把普通 mission 航点作为目标，而是使用 `/mavros/home_position/home` 的 home 水平位置作为临时目标，高度保持当前 odom 的 z。

退出 GUIDED 后，完整返航剖面仍交还给飞控原 return 模式继续执行。

### 3.8 重要安全边界

`auto_avoid_manager.py` 自己不直接发 `/mavros/setpoint_raw/local`，也不直接解锁。

但它会：

1. 调用 `/mavros/set_mode` 切换飞控模式。
2. 发布 `/mission_auto_avoid/avoidance_active=true`，允许 `apm_auto` 接管 `/position_cmd`。
3. 启用 EGO-Planner。

因此迁移到 Orin 时，不能把它当作第一阶段只读节点直接全量启用。至少要把模式切换、控制接管、mission set_current 做成可配置安全开关。

## 4. 节点二：mission_waypoint_uploader.py

节点名：

```text
mission_waypoint_uploader
```

### 4.1 主要职责

这个节点负责把本地 ENU 航点文件转换成 FCU 可执行的 global mission，并通过 MAVROS mission 服务上传给飞控。

它实现了：

1. 读取 `mission_waypoints.yaml`。
2. 等待地理原点：
   - `/mavros/global_position/gp_origin`
   - 或 AirSim 的 `/airsim_node/origin_geo_point`，如果 `airsim_ros_pkgs` 可用。
3. ENU 坐标转经纬度：
   - x/east 转 longitude。
   - y/north 转 latitude。
   - z 作为相对高度。
4. 构造 `mavros_msgs/Waypoint`：
   - frame: `FRAME_GLOBAL_REL_ALT`
   - command: `MAV_CMD_NAV_WAYPOINT`
   - param1: 航点停留时间
   - param2: acceptance radius 0.5
   - param4: yaw NaN
5. 清空 FCU mission。
6. 上传新的 mission。
7. pull 回来做验证日志。
8. 可选调用 `/mavros/mission/set_current` 设置起始航点。
9. 上传成功后设置参数 `/mission_auto_avoid/mission_ready=true`。

### 4.2 ArduPilot seq0 兼容

参数 `prepend_home_placeholder` 默认是 `true`。

原因是 ArduPilot 会把 mission seq 0 当作 HOME 或存在 seq 0 语义歧义。代码会在真正任务航点前插入一个 home placeholder，让真实任务从 seq 1 开始。

### 4.3 调用服务

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `/mavros/mission/clear` | `mavros_msgs/WaypointClear` | 清空 FCU mission |
| `/mavros/mission/push` | `mavros_msgs/WaypointPush` | 上传 mission |
| `/mavros/mission/pull` | `mavros_msgs/WaypointPull` | 上传后验证 |
| `/mavros/mission/set_current` | `mavros_msgs/WaypointSetCurrent` | 可选设置当前航点 |

## 5. 配置文件：mission_waypoints.yaml

当前配置：

```yaml
mission:
  frame_id: world
  waypoint_reached_radius: 1.0
  waypoints:
    - name: wp0
      x: 40.0
      y: 0.0
      z: 1.5
    - name: wp1
      x: -13.0
      y: -17.0
      z: 1.5
    - name: wp2
      x: -14.0
      y: 13.0
      z: 1.5
    - name: wp3
      x: 30.0
      y: 0.0
      z: 1.5
```

支持两种航点写法：

```yaml
- {x: 1.0, y: 2.0, z: 1.5}
```

或：

```yaml
- [1.0, 2.0, 1.5]
```

## 6. launch：mission_auto_avoid.launch

这是总入口，负责把各模块串起来。

它包含：

1. 可选 AirSim 点云转换桥：
   - `use_airsim_bridge=true` 时启用。
   - 默认关闭。

2. `planner_external_target.launch`：
   - 启动改造版 EGO-Planner。
   - 使用外部目标模式。
   - 输出 `/position_cmd`。

3. `px4/apm_auto.launch`：
   - 启动 `apm_auto_node`。
   - 该节点负责在 GUIDED 模式下把 `/position_cmd` 转成 `/mavros/setpoint_raw/local`。

4. `mission_waypoint_uploader.py`：
   - launch 条件是 `mission_source=file`。
   - 用于把本地 YAML mission 上传到 FCU。

5. `auto_avoid_manager.py`：
   - 主避障管理状态机。

默认关键参数：

```text
mission_source=fcu
odom_topic=/mavros/local_position/odom
lidar_world_topic=/world_cloud
goal_topic=/ego_input_target
planner_cmd_topic=/position_cmd
guided_mode=GUIDED
auto_mode=AUTO
planning_enabled_on_start=false
avoidance_flag_topic=/mission_auto_avoid/avoidance_active
```

## 7. launch：planner_external_target.launch

这个 launch 启动 EGO-Planner 的外部目标版本。

关键点：

1. 包含 `ego_planner/launch/advanced_param_exp.xml`。
2. 设置：

```text
flight_type=4
```

3. 改造版 EGO-Planner 在 `flight_type=4` 时订阅：

```text
/ego_input_target
```

4. planner 仍然输出：

```text
/drone_0_planning/bspline
/position_cmd
```

5. 同时启动：

```text
ego_planner/traj_server
odom_visualization
rviz，可选
```

## 8. 外部依赖一：改造版 EGO-Planner

`mission_auto_avoid` 依赖工程里已经改过的 EGO-Planner。关键改动在：

```text
src/planner/plan_manage/src/ego_replan_fsm.cpp
src/planner/plan_manage/include/plan_manage/ego_replan_fsm.h
src/planner/plan_manage/launch/advanced_param_exp.xml
```

已实现的外部能力：

1. 新增 planner 开关参数：

```text
fsm/planning_enabled_on_start
```

2. 新增服务：

```text
~set_planning_enabled  std_srvs/SetBool
```

关闭 planner 时会清空当前目标，并把 FSM 退回 `WAIT_TARGET`。

3. 新增外部目标模式：

```text
flight_type=4
```

在该模式下订阅：

```text
/ego_input_target
```

并且使用消息里的 x/y/z，不再把高度写死。

4. 在 planner 被禁用时，忽略：
   - manual waypoint。
   - trigger。
   - external target。
   - replan callback。

迁移时如果 Orin 的 `CoolFly_Fast-Drone-250` 还是干净版 EGO-Planner，需要补这些改动，否则 `auto_avoid_manager` 发 `/ego_input_target` 和调 `set_planning_enabled` 都不会生效。

## 9. 外部依赖二：px4/apm_auto

`mission_auto_avoid.launch` 包含：

```text
$(find px4)/launch/apm_auto.launch
```

对应节点：

```text
apm_auto_node
```

在当前 `auto/kufei_auto` 工程中，`apm_auto.cpp` 已经实现了避障接管相关逻辑：

1. 订阅：

```text
/position_cmd
/mission_auto_avoid/avoidance_active
/mode_select_flag
/mavros/state
/mavros/local_position/pose
```

2. 只有满足这些条件时才向飞控发 setpoint：

```text
send_setpoint_enable == true
mode_select_flag != 0
current_state.mode == GUIDED
avoidance_active == true
```

3. 接管时要求 planner 轨迹是新的：

```text
trajectory_id != avoidance_activation_control_traj_id
```

4. 首个避障 setpoint 距离当前位姿不能超过：

```text
avoidance_takeover_max_setpoint_jump
```

5. 真正控制输出是：

```text
/mavros/setpoint_raw/local
```

迁移到 Orin 时，如果不迁移 `apm_auto` 的这部分能力，`mission_manager` 最多只能做到“发现障碍并让 planner 出轨迹”，不能完成 GUIDED 控制接管。

## 10. 关键参数清单

### 10.1 mission 来源

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `mission_source` | `fcu` | `file` 或 `fcu` |
| `waypoint_file` | `config/mission_waypoints.yaml` | 本地航点文件 |
| `mission_ready_param` | `/mission_auto_avoid/mission_ready` | mission 是否准备完成 |
| `mission_start_wp_seq` | `0` | 上传 mission 后的起始序号 |
| `mission_set_current_after_upload` | `false` | 是否上传后显式 set_current |
| `mission_prepend_home_placeholder` | `true` | 是否插入 ArduPilot seq0 placeholder |

### 10.2 模式与接管

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `guided_mode` | `GUIDED` | 避障控制模式 |
| `auto_mode` | `AUTO` | 普通自动任务模式 |
| `return_mode_names` | `RTL,RTN,AUTO.RTL,AUTO.RTN` | 返航类模式识别 |
| `set_auto_on_start` | `true` | file 模式下可自动请求 AUTO |
| `auto_start_delay` | `2.0` | 自动切 AUTO 延迟 |
| `avoidance_flag_topic` | `/mission_auto_avoid/avoidance_active` | 接管允许标志 |

### 10.3 障碍检测

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `route_obstacle_distance_threshold` | `0.8` | 原航线障碍触发距离 |
| `route_clear_distance_threshold` | `1.2` | 原航线清除距离 |
| `route_lookahead_distance` | `10.0` | 原航线前视距离 |
| `route_min_forward_distance` | `0.5` | 忽略太近的前方点 |
| `local_obstacle_distance_threshold` | `0.8` | 当前到目标连线障碍触发距离 |
| `local_clear_distance_threshold` | `1.2` | 当前到目标连线清除距离 |
| `local_lookahead_distance` | `3.0` | 局部前视距离 |

### 10.4 回归原航线

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `route_rejoin_tolerance` | `0.6` | 小于该距离认为已回到原航线 |
| `route_rejoin_enter_tolerance` | `1.0` | 大于该距离进入 rejoin 状态 |
| `rejoin_forward_distance` | `2.0` | rejoin 目标沿原航线前推距离 |
| `rejoin_path_lookahead_distance` | `5.0` | 检查 rejoin 路径前视 |
| `rejoin_clear_distance_threshold` | 继承 local clear | rejoin 路径清除阈值 |
| `inflated_rejoin_clear_distance_threshold` | `0.25` | 膨胀点云 rejoin 清除阈值 |
| `rejoin_current_clearance_threshold` | `0.25` | 当前点附近安全距离 |
| `exit_current_clearance_threshold` | `0.5` | 退出 GUIDED 前当前点安全距离 |

### 10.5 planner 验证

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `planner_cmd_timeout` | `0.5` | `/position_cmd` 新鲜度 |
| `planner_valid_hold_time` | `1.0` | planner 输出持续有效时间 |
| `planner_motion_recent_window` | `1.0` | 近期有运动意义的窗口 |
| `planner_cmd_min_speed` | `0.15` | 判断轨迹有运动的速度阈值 |
| `planner_cmd_min_acc` | `0.2` | 判断轨迹有运动的加速度阈值 |
| `planner_cmd_min_pos_error` | `0.2` | 判断目标位置有意义的位置误差 |
| `require_new_planner_traj` | `true` | 是否要求新 trajectory_id |
| `takeover_max_cmd_distance` | `1.0` | 首个 planner cmd 距当前位置最大距离 |

## 11. 迁移到 Orin mission_manager 的建议功能分层

建议不要一次性把所有能力打开。可以按下面顺序迁移。

### 阶段 A：只读感知与航点目标发布

先迁移：

1. YAML 航点读取。
2. odom 订阅。
3. `/ego_input_target` 或 `/move_base_simple/goal` 发布。
4. 到点判断和航点推进。
5. route/local 障碍距离计算。
6. diagnostic 话题发布。

暂不迁移：

```text
/mavros/set_mode
/mavros/setpoint_raw/local
/mavros/mission/set_current
```

### 阶段 B：接 EGO-Planner 外部目标

迁移：

1. `/ego_input_target` 外部目标接口。
2. planner `set_planning_enabled` 服务。
3. `/position_cmd` 新鲜度和 trajectory_id 验证。
4. planner 输出有效状态发布。

需要同步改 Orin 上的 EGO-Planner，否则 `flight_type=4` 不可用。

### 阶段 C：FCU mission 读取

迁移：

1. `/mavros/mission/waypoints` 读取。
2. `/mavros/global_position/gp_origin` 读取。
3. global waypoint 到 local ENU 转换。
4. return mode home 目标处理。

这一阶段仍然可以只读，不切模式。

### 阶段 D：AUTO/GUIDED 模式切换

迁移：

1. 避障触发后请求 GUIDED。
2. 退出后恢复 AUTO/RTL。
3. `/mode_select_flag` 安全门控。
4. odom timeout、FCU disconnected 等保护。

建议加一个总开关，例如：

```text
enable_mode_switch:=false
```

默认关闭。

### 阶段 E：控制接管

迁移或重写：

1. `apm_auto` 对 `/mission_auto_avoid/avoidance_active` 的订阅。
2. `/position_cmd` 到 `/mavros/setpoint_raw/local` 的转换。
3. 首个 setpoint 跳变限制。
4. GUIDED 且 mode_select_flag 允许时才发控制。

这是风险最高的一层，应最后做，并且必须地面架上验证。

### 阶段 F：mission 上传和 set_current

迁移：

1. `mission_waypoint_uploader.py` 的 ENU 到 global waypoint 上传。
2. ArduPilot seq0 placeholder。
3. 上传验证。
4. 回 AUTO 前可选同步 `current_seq`。

这部分会直接改飞控 mission，应单独测试。

## 12. 迁移时必须注意的差异

1. Orin 当前第一阶段文档中建议只发布 `/move_base_simple/goal`，而该包使用的是 `/ego_input_target`。
2. Orin 干净版 EGO-Planner 可能没有 `flight_type=4` 和 `set_planning_enabled`。
3. Orin 当前目标可能是 Livox/FAST-LIVO2：

```text
odom:  /aft_mapped_to_init
cloud: /cloud_registered 或 planner 膨胀点云
```

而原包默认是：

```text
odom:  /mavros/local_position/odom
cloud: /world_cloud
```

4. 原包完整链路会切 GUIDED 并通过 `apm_auto` 发 setpoint，不适合第一阶段无保护直接跑。
5. 原包以 ArduPilot/AUTO/GUIDED/RTL 语义为主，如果 Orin 实际飞控固件或模式名不同，需要抽象模式名。

## 13. 最小迁移清单

如果目标是“先把功能骨架加到 Orin mission_manager”，优先实现：

1. `mission_source=file/fcu` 双来源。
2. YAML 航点解析。
3. FCU mission 解析为本地 ENU。
4. route/local 两套障碍距离判断。
5. 避障状态机：

```text
IDLE
OBSTACLE_DETECTED
WAIT_GUIDED_HOLD
WAIT_PLANNER_TRAJ
AVOIDING
ROUTE_REJOIN
WAIT_AUTO_RESUME
FINISHED
```

6. `/ego_input_target` 发布。
7. planner `set_planning_enabled`。
8. `/position_cmd` trajectory_id 与新鲜度验证。
9. `/mission_manager/avoidance_active` 类似标志。
10. 所有飞控写操作加显式安全开关。

## 14. 功能一句话总结

`mission_auto_avoid` 的本质是：让飞控继续执行 AUTO mission，当航线前方出现障碍时，临时切到 GUIDED，把当前 mission 目标交给 EGO-Planner 绕障，等路径清空且飞机回到原 mission 航线附近后，再切回原来的自动模式。
