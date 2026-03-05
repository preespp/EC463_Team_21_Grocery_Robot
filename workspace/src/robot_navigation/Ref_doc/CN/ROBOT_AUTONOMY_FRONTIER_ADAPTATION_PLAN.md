# Robot Autonomy Frontier Adaptation Plan

## 0. 文档目标

本计划用于在当前代码基础上，落地“边建图边定位 + 自动探索 + 可随时人工接管”的完整技术路线，并给出：

1. 先后顺序与优先级（可执行的里程碑）。
2. 当前代码中的 Nav2 参数基线、参数意义、现有效果预估、调整建议。
3. 未完成模块的设计规划（Frontier、仲裁、任务编排、语义叠加）。
4. 每个模块的验证方法与验收标准（DoD）。

---

## 1. 现状基线（按代码核对）

## 1.1 当前主链路（已具备）

1. `robot_localization` EKF：
   `/odom_raw + /sick_scansegment_xd/imu -> /odom`
2. `Cartographer`（在线 SLAM）：
   `use_odometry=true`, `use_imu_data=false`，进行 scan matching + pose graph
3. `Nav2`：
   全局 `NavFnPlanner(use_astar=false)`，局部 `DWBLocalPlanner`
4. 低层安全：
   超声波 `/front_alert,/back_alert,/left_alert,/right_alert` 在串口桥直接限向
5. 地图分辨率默认：
   导出默认 `0.03`，Nav2 local/global costmap 默认 `0.03`

## 1.2 当前已知缺口（未完成）

1. 无 Frontier 节点实现（仓库内未发现 `frontier/explore` 代码）。
2. 无 `cmd_vel` 优先级仲裁（当前为多 topic “最后到达覆盖”）。
3. 无 mission orchestrator 状态机（自动探索 -> 回家 -> 保存 -> 语义叠加）。
4. 无统一一键顶层 launch（目标 `auto_frontier_mission.launch.py` 尚未实现）。
5. 语义货架层（`shelves.yaml/json` 及变换）尚未实现。

---

## 2. 总体架构（目标态）

```text
LiDAR + IMU + Wheel Odom Raw
        |        |        |
        +--------+--------+
                 v
           EKF (/odom)
                 |
                 v
Cartographer (online SLAM, map<->odom, scan matching)
                 |
                 +--> /map (online occupancy grid)
                 |
                 v
            Nav2 (NavFn + DWB)
                 ^
                 |
     Frontier Explorer (auto goals)
                 |
           /cmd_vel_auto

Teleop -> /cmd_vel_manual ----+
                              v
                     cmd_vel_arbiter -> /cmd_vel -> nav2_serial_bridge -> STM32
                              ^
                     /manual_override (抢占)

Mission Orchestrator:
BOOT -> AUTO_EXPLORE -> RETURN_HOME -> SAVE_EXPORT -> LOAD_SEMANTIC -> READY_FOR_TASKS
```

---

## 3. 实施顺序与优先级

## P0（最高优先级）控制权与安全闭环

目标：保证“随时人工接管”是硬约束，不依赖人为时机。

任务：
1. 新增 `cmd_vel_arbiter`：
   `/cmd_vel_manual` 高优先级、`/cmd_vel_auto` 低优先级、输出 `/cmd_vel`
2. 新增 `/manual_override`（`std_msgs/Bool`）：
   - `true`：立即切手动、发 zero-stop、取消自动目标
   - `false`：允许恢复自动
3. 统一将自动控制输出改到 `/cmd_vel_auto`，teleop 改到 `/cmd_vel_manual`
4. 保留低层超声波限向逻辑（现有串口桥逻辑不改）

完成标准：
1. 抢占延迟 <= 150 ms
2. 抢占后停止距离 <= 0.20 m（低速 profile）
3. 人工释放后可恢复自动任务（不中断状态机）

---

## P1 任务状态机骨架（无 Frontier，先可运行）

目标：先把流程框架跑通，降低后续 Frontier 集成风险。

任务：
1. 新增 `mission_orchestrator`（动作编排节点）
2. 状态实现：
   `BOOT -> AUTO_MAP_V1 -> RETURN_HOME -> SAVE_EXPORT -> LOCALIZE_READY`
3. `AUTO_MAP_V1` 先用闭环固定扫图轨迹（map frame 闭环，不用开环速度）
4. 建图结束条件（并行）：
   - 时间阈值
   - 路程阈值
   - `finish_mapping` 手动服务

完成标准：
1. 一键启动后无需手工切流程即可完成“扫图+回家+保存导图”
2. 中途任意时刻可 `manual_override` 抢占并恢复

---

## P2 Frontier Explorer（自动探图核心）

目标：从固定轨迹过渡到真实 frontier 探索。

任务：
1. 新增 `frontier_explorer` 节点：
   - 订阅 `/map`
   - Frontier 提取（未知-自由边界）
   - 聚类（BFS 连通域）
   - 候选点生成（簇重心或最近可达点）
   - 可达性检查（`ComputePathToPose` 或规划服务）
   - 打分选择（信息增益 - 路径代价 - 风险代价）
2. 失败恢复策略：
   - 超时/无进展：取消目标 + 黑名单
   - 连续失败：触发 wait/spin/replan，必要时人工接管
3. 终止条件：
   - 连续 N 轮无有效 frontier
   - 地图新增面积低于阈值
   - 达到最大探索时长

完成标准：
1. 能在未知环境持续自主生成并执行目标
2. 不出现重复陷入同一区域循环
3. 结束条件可稳定触发并进入 `RETURN_HOME`

---

## P3 回家、保存、导图自动化

目标：探索结束后稳定回到起点并沉淀地图产物。

任务：
1. 在 `BOOT` 后静止窗口记录 `home_pose`（`map->base_link`）
2. `RETURN_HOME` 调用 `NavigateToPose(home_pose)`
3. `SAVE_EXPORT`：
   - `/write_state` 保存 `.pbstream`
   - 调 `cartographer_pbstream_to_ros_map` 导出 `.yaml/.pgm`
   - 文件名带时间戳避免覆盖

完成标准：
1. 回家成功率 >= 95%（10 次测试）
2. 回家误差：`xy <= 0.15m`, `yaw <= 10deg`
3. 地图文件完整产出且可直接被 localization stack 加载

---

## P4 语义货架层叠加

目标：不改 occupancy grid 像素，仅叠加业务语义坐标。

任务：
1. 设计 `shelves.yaml/json`（`shelf_id -> pose`）
2. 实现 `semantic_overlay` 节点：
   - 读取参考语义图层
   - 计算 `T_ref_map_to_new_map`
   - 发布变换后货架坐标（Marker + 查询服务）
3. V1：同起点同朝向 + 2 点人工校准
4. V2：自动配准（ICP/特征点）

完成标准：
1. 货架点叠加误差 <= 0.20 m（V1）
2. 可通过 `shelf_id` 查询目标位姿并成功发 Nav2 goal

---

## P5 参数稳态调优与回归

目标：在 `0.03` 分辨率下兼顾稳定性、负载、动态避障表现。

任务：
1. 构建“慢速探索 profile”和“任务运行 profile”
2. 每轮只改 1~2 个参数，记录前后指标
3. 回归四类核心指标：
   - CPU/内存负载
   - 规划与控制频率稳定性
   - 动态避障成功率
   - 人工接管响应性能

完成标准：
1. `controller_frequency` 实测接近目标且无长时掉频
2. 动态障碍场景下显著减少振荡/卡住
3. 关键指标有对比记录并形成调参基线

---

## 4. Nav2 参数基线与调整建议（当前代码对照）

说明：下表“当前考量/效果预估”为基于参数值和现有链路的工程推断，用于调参优先级判断。

## 4.1 全局/局部代价地图

| 参数路径 | 当前值 | 参数意义 | 当前考量/效果预估 | 调整建议（自动探索慢速） |
|---|---:|---|---|---|
| `local_costmap.update_frequency` | `5.0` | 局部地图更新频率 | 中等频率，动态障碍响应可用但非高刷新 | 可尝试 `8~10`（CPU 允许时） |
| `local_costmap.publish_frequency` | `2.0` | 局部地图发布频率 | RViz 可视化偏低，但控制不直接依赖发布频率 | 可维持 |
| `local_costmap.width/height` | `3/3` | 局部窗口范围（m） | 近场避障为主，前瞻距离偏短 | 探索可升到 `4~5` |
| `local_costmap.resolution` | `0.03` | 栅格分辨率 | 精度提升，CPU负载增加 | 维持 `0.03` |
| `local_costmap.robot_radius` | `0.40` | 碰撞包络 | 对底盘外廓较保守，利于安全 | 如盲区风险高可增至 `0.42~0.45` |
| `inflation_radius` | `0.55` | 障碍膨胀半径 | 偏保守，过道可能更难通行 | 探索场景建议维持或小幅上调到 `0.60` |
| `cost_scaling_factor` | `3.0` | 膨胀代价衰减 | 中等偏平衡 | 卡边时下调；过保守时上调 |
| `voxel_layer.z_resolution/z_voxels` | `0.05 / 16` | 3D 体素高度离散 | 对 2D 移动底盘足够 | 保持 |
| `cloud.obstacle_max_range` | `2.5` | 障碍有效距离 | 近中距反应为主 | 场地大可升到 `3.0` |
| `cloud.raytrace_max_range` | `3.0` | 清除射线最大范围 | 支持中距清障 | 保持或跟随激光有效范围 |
| `global_costmap.resolution` | `0.03` | 全局规划分辨率 | 精细路径，负载高于 `0.05` | 资源紧张可回到 `0.05` 做对比 |
| `global_costmap.update_frequency` | `1.0` | 全局更新频率 | 适合静态图+缓慢变化 | 探索中一般可保持 |
| `track_unknown_space` | `true` | 追踪未知区域 | 对 frontier 与未知区域规划必要 | 必须保持 `true` |

## 4.2 规划器与控制器

| 参数路径 | 当前值 | 参数意义 | 当前考量/效果预估 | 调整建议（自动探索慢速） |
|---|---:|---|---|---|
| `planner_server.GridBased.plugin` | `NavfnPlanner` | 全局规划器 | 成熟稳定 | 保持 |
| `GridBased.use_astar` | `false` | Dijkstra/A* 切换 | 当前是 Dijkstra 风格，稳定但可能更保守 | 可维持 `false`，后续对比 `true` |
| `GridBased.tolerance` | `0.5` | 目标容差 | 允许在拥挤环境更易收敛 | 高精定位任务可降到 `0.2~0.3` |
| `controller_frequency` | `20.0` | 控制循环频率 | 标准值，需保证掉频少 | 若 CPU 紧张可降到 `15` |
| `FollowPath.max_vel_x` | `0.26` | 前进最大速度 | 中速偏稳 | 探索先降到 `0.15~0.20` |
| `FollowPath.max_vel_y` | `0.20` | 横移最大速度 | 已启用全向能力 | 探索先降到 `0.08~0.15` |
| `FollowPath.max_vel_theta` | `1.0` | 角速度上限 | 转向较积极 | 探索先降到 `0.6~0.8` |
| `acc_lim_x/y/theta` | `2.5/1.5/3.2` | 加速度上限 | 响应较快 | 探索下调，减少急动与振荡 |
| `decel_lim_x/y/theta` | `-2.5/-1.5/-3.2` | 减速度上限 | 急停能力强 | 与加速度成对慢化 |
| `vx/vy/vtheta_samples` | `20/15/20` | 轨迹采样密度 | 质量较好但算力占用高 | 探索可先降到 `15/10/15` |
| `sim_time` | `1.7` | 前瞻时域 | 中等前瞻 | 拥挤场景可 `1.5~2.0` 调优 |
| `BaseObstacle.scale` | `0.02` | 障碍代价权重 | 数值偏低，可能过于追路径 | 探索建议先提高到 `0.05~0.1` |
| `PathAlign/PathDist` | `32/32` | 贴路径倾向 | 容易“守路径” | 人流环境可适度下调 |
| `GoalDist.scale` | `24` | 逼近目标倾向 | 收敛性好 | 过冲时下调 |

## 4.3 进度检查、恢复、平滑器

| 参数路径 | 当前值 | 参数意义 | 当前考量/效果预估 | 调整建议（自动探索慢速） |
|---|---:|---|---|---|
| `progress_checker.required_movement_radius` | `0.5` | 判定“有进展”的位移阈值 | 阈值较大，窄空间可能误判卡住 | 探索建议 `0.2~0.3` |
| `progress_checker.movement_time_allowance` | `10.0` | 进展时间窗口 | 中等容忍 | 行人密集可增到 `12~15` |
| `goal_checker.xy/yaw_goal_tolerance` | `0.25/0.25` | 到位容差 | 对导航稳定友好 | 可保持 |
| `behavior_plugins` | `spin/backup/drive_on_heading/wait/assisted_teleop` | 恢复动作集合 | 已具备标准恢复链 | 盲区风险下优先 `wait+spin+replan` |
| `velocity_smoother.feedback` | `OPEN_LOOP` | 平滑反馈模式 | 简单稳定，不依赖闭环速度反馈 | 可保持，后续评估 `CLOSED_LOOP` |
| `velocity_smoother.max_velocity` | `[0.26,0.20,1.0]` | 平滑速度上限 | 与 DWB 一致 | 探索 profile 同步下调 |

## 4.4 AMCL 相关（当前非主定位）

| 参数路径 | 当前值 | 说明 |
|---|---:|---|
| `amcl.tf_broadcast` | `false` | 避免与 Cartographer 冲突发布 `map->odom` |
| `amcl.scan_topic` | `/scan_fullframe` | 配置存在，但当前主定位链路为 Cartographer |

---

## 5. EKF + Cartographer 融合链路检查点

## 5.1 当前配置结论

1. EKF 已融合：`/odom_raw + IMU -> /odom`
2. Cartographer 使用 odom 先验：`use_odometry=true`
3. Cartographer 当前不直接用 IMU：`use_imu_data=false`

## 5.2 调整建议

1. 先保持当前分层融合，不立即给 Cartographer 直连 IMU。
2. 优先调好 EKF covariance、时间同步、TF 稳定性。
3. 仅在出现明显航向漂移场景时，再评估 Cartographer `use_imu_data=true` 的收益与副作用。

---

## 6. 未完成模块设计规划（详细）

## 6.1 `cmd_vel_arbiter`（新增）

职责：
1. 输入：`/cmd_vel_manual`, `/cmd_vel_auto`, `/manual_override`
2. 输出：`/cmd_vel`
3. 逻辑：
   - `manual_override=true`：仅放行 manual
   - `manual_override=false`：manual 有效时仍优先，超时回退 auto
4. 安全：
   - 源超时自动发 stop
   - 切换源时发一帧 stop 防突变

## 6.2 `mission_orchestrator`（新增）

职责：
1. 管理任务状态机与跨节点动作调用。
2. 统一处理抢占：
   - 收到 manual override -> 暂停/取消当前自动动作
   - 释放后从可恢复状态继续

核心接口：
1. `start_mission`, `pause_mission`, `resume_mission`, `finish_mapping`
2. action client：`navigate_to_pose`, `follow_waypoints`
3. service client：`/write_state`

## 6.3 `frontier_explorer`（新增）

算法建议：
1. Frontier 提取：`unknown(-1)` 且邻接 `free(0)`
2. 聚类：BFS 连通域
3. 候选点：簇重心 + 最近可达点回退
4. 打分：
   `score = w_gain * info_gain - w_path * path_len - w_risk * risk`
5. 失败处理：
   - 短期黑名单（TTL）
   - 连续失败触发 recovery 链

终止条件（建议默认）：
1. `no_frontier_rounds >= 5`
2. `new_area_ratio < 1%` 持续 `60s`
3. `max_explore_time`（如 20 分钟）

## 6.4 顶层启动 `auto_frontier_mission.launch.py`（新增）

内部拉起：
1. `slam_mapping_stack.launch.py`（`with_collision:=true` 默认）
2. Nav2 bringup（探索可用）
3. `cmd_vel_arbiter`
4. `frontier_explorer`（可开关）
5. `mission_orchestrator`

推荐参数：
1. `map_name:=run_<timestamp>`
2. `with_rviz:=true/false`
3. `explore_profile:=slow`

## 6.5 语义叠加 `semantic_overlay`（新增）

职责：
1. 加载 `shelves.yaml/json`
2. 计算 `T_ref_map_to_new_map`
3. 发布货架 marker + 查询服务

V1：
1. 同起点同朝向 + 2 点人工校准
2. 目标误差控制在 `<=0.20m`

---

## 7. 模块验证计划与验收标准

## 7.1 感知与基础定位链路

检查项：
1. `/cloud_all_fields_fullframe`、`/sick_scansegment_xd/imu` 有效
2. `/odom_raw`、`/odom` 连续
3. `map -> odom -> base_link -> lidar_link` TF 完整

命令：
```bash
ros2 topic hz /cloud_all_fields_fullframe
ros2 topic hz /sick_scansegment_xd/imu
ros2 topic hz /odom_raw
ros2 topic hz /odom
ros2 run tf2_tools view_frames
```

通过标准：
1. 无持续 TF 冲突/超时
2. `/odom` 频率稳定接近 EKF 频率目标

## 7.2 `cmd_vel_arbiter`（抢占验证）

检查项：
1. 自动运动中发布 `manual_override=true`，机器人快速停止自动指令
2. 仅 manual topic 生效
3. 释放后自动继续

通过标准：
1. 抢占延迟 <= 150 ms
2. 停止距离 <= 0.20 m（慢速）
3. 无“抢占后自动又抢回控制”抖动

## 7.3 Frontier 探索

检查项：
1. 能持续生成新目标
2. 失败目标会被黑名单抑制
3. 终止条件能正确触发

通过标准：
1. 连续 10 分钟无死循环
2. 地图覆盖率持续上升直至收敛

## 7.4 回家与建图产出

检查项：
1. `home_pose` 记录有效
2. 终止探索后回家成功
3. 产出 `.pbstream + .yaml + .pgm`

通过标准：
1. 回家成功率 >= 95%（10 次）
2. 误差 `xy <= 0.15m`, `yaw <= 10deg`

## 7.5 动态障碍与恢复行为

检查项：
1. 行人/移动障碍出现时减速、绕行或等待
2. 超声波告警方向速度被正确限制
3. Recovery 不出现高风险倒车

通过标准：
1. 无碰撞事件
2. 卡住后恢复成功率 >= 90%

## 7.6 性能回归（0.03 分辨率）

检查项：
1. CPU 占用与内存稳定
2. 控制循环不明显掉频
3. 局部规划无大面积超时

通过标准：
1. 长时运行（30 分钟）无失稳
2. 轨迹跟踪无明显高频振荡

---

## 8. 推荐里程碑输出（建议提交节奏）

1. `M1`：`cmd_vel_arbiter + manual_override + 验证脚本`
2. `M2`：`mission_orchestrator`（固定轨迹 V1）+ 回家 + 导图自动化
3. `M3`：`frontier_explorer` + 失败恢复 + 终止条件
4. `M4`：`auto_frontier_mission.launch.py` 一键启动
5. `M5`：`semantic_overlay`（V1）
6. `M6`：参数回归报告（探索 profile / 运行 profile）

---

## 9. 风险清单与缓解

1. LiDAR 盲区（后部遮挡）导致动态障碍漏检：
   - 保留超声波低层限向
   - 探索策略优先前向与原地转向
2. 抢占控制抖动：
   - 仲裁器加入源锁定与超时机制
3. 分辨率提高导致负载上升：
   - 先慢速 profile，逐步加速
4. Frontier 循环卡点：
   - 引入黑名单与失败退避
5. 回家失败：
   - 增加中间点回退策略与重规划次数上限

---

## 10. 当前结论

在现有代码基础上，继续走 `EKF + Cartographer + Nav2 + Frontier + Safety + Orchestrator` 是最小改动且最稳的路线。

最关键的第一步不是 Frontier 本身，而是先完成控制权架构（`cmd_vel` 仲裁 + `manual_override`），否则无法保证自动化阶段的工程可控性与安全性。

