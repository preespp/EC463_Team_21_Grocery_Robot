# Robot Navigation Stack Execution Plan (2026-03)

## 0. 目标与范围

本执行计划用于把 `PARAMETER_FINETUNE_PLAN.md` 落地为可立即开工的任务清单，优先目标是：

1. 保持 Cartographer 建图输入为当前 `PointCloud2`，在不切换 LaserScan 的前提下完成第一轮高质量地图产出。
2. 建立可重复的质量验收门槛（参数、传感器、地图资产三类）。
3. 为后续 AMCL 运行时定位和 Nav2 profile 对比留好接口，但不阻塞今天开工。

本计划只覆盖 `workspace/src/robot_navigation` 相关内容。

---

## 1. 当前基线与差距（开工前确认）

基于当前仓库（2026-03-10）代码，关键差距如下：

1. Cartographer mapping 当前输入路径是 `PointCloud2`
   - `cartographer_mapping.launch.py` 仅 remap `points2` 和 `imu`。
   - D0 阶段保持这一路径，不引入输入模式切换。
2. 需要在 PointCloud2 路径上验证“质量优先”参数
   - `min_range=0.15`, `max_range=10`, `missing_data_ray_length=10`
   - `translation_delta_cost_weight=0.2`
   - `submaps.grid_options_2d.resolution=0.03`
3. 导图资产一致性还没有自动化验证入口
   - `nav_assistant.py` 目前没有 `verify-mapping-profile / verify-sensors / verify-map-artifacts`
4. 运行时定位仍是 Cartographer localization 主链路
   - 该项不影响今天建图质量开工，但会影响后续 relocalization 阶段。

---

## 2. 开工策略（先质量，后扩展）

### 2.1 固定决策（本轮不再摇摆）

1. 建图继续使用 Cartographer。
2. D0 建图输入固定为 fullframe `PointCloud2`（`/cloud_all_fields_fullframe`）。
3. 原计划中的 `D0-2`（输入模式切换）本轮移除，不改 launch 接口。
4. Cartographer 原生 IMU 继续关闭（`use_imu_data=false`），直到 frame 共点条件满足。
5. `export-map --resolution` 仅作为输出分辨率检查，不作为 SLAM 质量代理指标。

### 2.2 质量通过条件（硬门槛）

建图质量“通过”必须同时满足：

1. 输入路径：Cartographer 订阅 `points2`（`/cloud_all_fields_fullframe`）。
2. 内部 SLAM 分辨率：`submaps.grid_options_2d.resolution == 0.03`。
3. 距离参数：`min_range == 0.15`, `missing_data_ray_length == max_range`。
4. 关键 matcher/filter 参数符合目标值。
5. 地图资产三件套 `.pbstream/.yaml/.pgm` 同 stem、同轮次生成、mtime 差值 <= 120s。

---

## 3. 今日可执行任务（D0，立即开始）

### Task D0-1: 固定 PointCloud2 质量优先配置

目标文件：
- `workspace/src/robot_navigation/config/pico_2d_mapping_quality.lua`

目标参数（固定）：

```lua
num_laser_scans = 0
num_multi_echo_laser_scans = 0
num_subdivisions_per_laser_scan = 1
num_point_clouds = 1

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.03
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e4
```

验收：
- `lua` 语法通过（launch 不报配置错误）。
- 启动后 Cartographer 正常出 `/submap_list` 与轨迹。

---

### Task D0-2（已移除）: Mapping Launch 输入模式切换

说明：
1. 本轮不做 `sensor_input_mode` 改造。
2. `cartographer_mapping.launch.py`、`slam_mapping_stack.launch.py`、`nav_assistant.py` 保持现状。
3. D0 仅验证 PointCloud2 主路径，不引入额外变量。

---

### Task D0-3: 锁定 SICK 建图 profile（PointCloud2 fullframe + 30Hz/0.1deg）

目标文件：
- `workspace/src/robot_navigation/launch/slam_mapping_stack.launch.py`

目标行为：

1. 保持 `custom_pointclouds:=cloud_all_fields_fullframe` 为建图输入基线。
2. 保持当前 `scandataformat`、IMU、`odom` 接线，作为 D0 对比基线。
3. 若驱动同时发布 `/scan_fullframe`，D0 阶段不将其作为质量门槛。

验收：
- `ros2 topic hz /cloud_all_fields_fullframe` 在目标频率 ±10%。
- `ros2 topic echo /cloud_all_fields_fullframe --once` 的 `header.frame_id == lidar_link`。

---

### Task D0-4: 第一轮建图与资产产出

执行命令（示例）：

```bash
cd <repo_root>/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run robot_navigation nav_assistant mapping-stack \
  --cartographer-config-basename pico_2d_mapping_quality.lua

# 另开终端遥控
ros2 run robot_navigation nav_assistant teleop

# 导图
ros2 run robot_navigation nav_assistant save-map --map-name qmap_20260310_pc_a
ros2 run robot_navigation nav_assistant export-map --map-name qmap_20260310_pc_a --resolution 0.03
```

注意：
- 建图过程优先低速平滑运动，避免急停急转。
- 保存前机器人静止数秒，让图优化收敛。

---

### Task D0-5: 手工验收（脚本化前的过渡）

先用命令行执行最小验收：

```bash
ros2 node info /cartographer_node
ros2 topic info /cloud_all_fields_fullframe
ros2 topic hz /cloud_all_fields_fullframe
ros2 topic echo /cloud_all_fields_fullframe --once
ros2 topic hz /sick_scansegment_xd/imu
ros2 topic hz /odom
```

地图资产检查：

```bash
ls -l Maps/qmap_20260310_pc_a.pbstream Maps/qmap_20260310_pc_a.yaml Maps/qmap_20260310_pc_a.pgm
```

通过标准：
- `ros2 node info /cartographer_node` 显示订阅 `points2`。
- `.pbstream/.yaml/.pgm` 都存在且同 stem。
- `.yaml` 的 `image:` 指向存在的 `.pgm`。
- 三个文件修改时间接近（<=120s）。

---

## 4. D1（次日）自动化验证落地

### Task D1-1: nav_assistant 新增只读验证命令

目标命令：
- `verify-mapping-profile`
- `verify-sensors`
- `verify-map-artifacts`

输出目录（固定）：
- `verification/mapping_profile_report.json`
- `verification/sensor_report.json`
- `verification/map_artifacts_report.json`

约束：
- 只读检查，不改 tracked files。

---

### Task D1-2: 质量门槛脚本化

脚本化检查内容：

1. 读取 launch + lua：确认 `points2` 路径和目标参数。
2. ROS runtime 检查：topic 频率、frame_id、node 订阅。
3. 资产检查：三件套、stem、一致性、mtime。

完成标准：
- 三个 JSON 报告均可重复生成。
- 同一次建图可稳定通过。

---

## 5. 风险与回滚

### 风险 R1: 质量配置启动失败

回滚：
- `cartographer_config_basename=pico_2d.lua`

### 风险 R2: 新参数导致 CPU 占用过高

回滚优先级：
1. `submaps.num_range_data: 60 -> 50`
2. `motion_filter.max_time_seconds: 0.5 -> 1.0`
3. `real_time_correlative_scan_matcher.linear_search_window: 0.15 -> 0.1`

### 风险 R3: 近场噪点/车体自障碍

回滚优先级：
1. `min_range: 0.15 -> 0.20`
2. `max_range: 10.0 -> 8.0`
3. `missing_data_ray_length` 与 `max_range` 保持同值

### 风险 R4: 旋转工况下局部几何抖动

症状：
- 旋转时墙边出现抖动、双边轮廓或局部扭曲。

处理顺序：
1. 先降车速并减小角速度，确认运动学与时间同步。
2. 检查 `/odom` 频率与时间戳连续性。
3. 若仍不稳定，回退 `pico_2d.lua` 并保留 rosbag 做参数 A/B 对比。

---

## 6. 里程碑定义（用于周内推进）

### M1（今天）
- 完成 D0-1、D0-3、D0-4、D0-5，产出首张 `qmap_20260310_pc_a` 地图三件套。

### M2（明天）
- `verify-*` 三类命令落地，JSON 报告稳定输出。

### M3（本周）
- 完成 `pico_2d.lua` 与 `pico_2d_mapping_quality.lua` 的同路线 A/B 对比。
- 固化 PointCloud2 默认建图 profile。

### M4（下周）
- 开始 AMCL 运行时定位主线（不再以 Cartographer localization 作为 owner）。

---

## 7. 本计划与上位文档关系

1. 继承 `PARAMETER_FINETUNE_PLAN.md` 的三主线结构。
2. 本文只把“建图质量主线”细化为 D0 可执行任务。
3. 本版明确采用 PointCloud2 主路径，并移除原 `D0-2` 输入模式切换任务。
4. AMCL 与 Nav2 profile 对比继续按上位文档推进，不在 D0 阶段阻塞建图质量改进。
