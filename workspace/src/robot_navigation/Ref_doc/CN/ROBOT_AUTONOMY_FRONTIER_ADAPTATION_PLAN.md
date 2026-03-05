# Robot Autonomy Frontier Adaptation Plan（中文）

## 0. 目的

本计划用于将当前导航栈演进为完整的“在线建图 + 在线定位 + 自主探索 + 人工随时接管”工作流。

文档包含：
1. 执行顺序与优先级。
2. 基于代码现状的 Nav2 参数基线、设计理由与预期行为。
3. 核心模块设计（frontier、速度仲裁、任务编排、语义货架叠加）。
4. 模块级验证方法与验收标准。

---

## 1. 当前基线（按代码核对）

## 1.1 现有运行链路

1. `robot_localization` EKF：  
   `/odom_raw + /sick_scansegment_xd/imu -> /odom`
2. `Cartographer`（在线 SLAM）：  
   `use_odometry=true`，`use_imu_data=false`，执行 scan matching + pose graph。
3. `Nav2`：  
   全局规划器 `NavFnPlanner (use_astar=false)`，局部控制器 `DWBLocalPlanner`。
4. 底层安全：  
   串口桥中基于超声波告警（`/front_alert`, `/back_alert`, `/left_alert`, `/right_alert`）进行方向限速/限行。
5. 分辨率默认值：  
   地图导出默认 `0.03`，Nav2 本地/全局 costmap 默认 `0.03`。

## 1.2 原始缺口（计划视角）

1. 缺少 frontier 探索节点。
2. 缺少严格的 `manual > auto` `/cmd_vel` 仲裁。
3. 缺少端到端任务状态机（mission orchestrator）。
4. 缺少一键全自动 mission 顶层 launch。
5. 缺少语义货架叠加层（`shelves.yaml/json` + 变换对齐）。

---

## 2. 目标架构

```text
LiDAR + IMU + Raw wheel odom
          |       |
          +-------+-------> EKF (/odom)
                           |
                           v
Cartographer online SLAM (scan matching + loop closure + map<->odom)
                           |
                           +--> /map (online occupancy grid)
                           |
                           v
                        Nav2
                           ^
                           |
                Frontier Explorer (NavigateToPose goals)
                           |
                      /cmd_vel_auto

Teleop --> /cmd_vel_manual ----+
                               v
                     cmd_vel_arbiter --> /cmd_vel --> nav2_serial_bridge --> STM32
                               ^
                     /manual_override (Bool)

Mission Orchestrator:
BOOT -> AUTO_EXPLORE -> RETURN_HOME -> SAVE_EXPORT -> LOAD_SEMANTIC -> READY_FOR_TASKS
```

---

## 3. 实施顺序与优先级

## P0（最高优先级）：控制权与安全闭环

目标：  
保证任何时刻都能立即人工接管。

任务：
1. 增加 `cmd_vel_arbiter`：  
   `/cmd_vel_manual` 高优先级，`/cmd_vel_auto` 低优先级，输出 `/cmd_vel`。
2. 增加 `/manual_override`（`std_msgs/Bool`）：  
   - `true`：立即接管，零速刹停，取消自动导航目标。  
   - `false`：允许自动流程恢复。
3. 所有自治速度统一走 `/cmd_vel_auto`。
4. 保留现有超声波底层限向策略。

完成标准：
1. 人工接管延迟 <= 150 ms。
2. 探索速度下接管后制动距离 <= 0.20 m。
3. 释放接管后自动流程可恢复，且无死锁。

---

## P1：任务状态机骨架（先不引入 frontier）

目标：  
在引入 frontier 复杂性之前，先让全链路稳定可运行。

任务：
1. 增加 `mission_orchestrator`。
2. 状态流：  
   `BOOT -> AUTO_MAP_V1 -> RETURN_HOME -> SAVE_EXPORT -> LOCALIZE_READY`。
3. `AUTO_MAP_V1` 使用地图坐标系下闭环固定轨迹。
4. 建图完成条件：  
   - 达到最大时间阈值  
   - 达到最大行驶距离阈值  
   - 手动触发 `finish_mapping` 服务

完成标准：
1. 一条命令可完成建图 + 回家 + 保存/导出。
2. 任意状态可人工接管，并支持恢复。

---

## P2：集成 Frontier Explorer

目标：  
用真正的 frontier 探索替代固定轨迹建图。

任务：
1. 增加 `frontier_explorer` 节点：  
   - 订阅 `/map`  
   - frontier 提取：未知格邻接自由格  
   - 聚类：BFS 连通域  
   - 候选点生成：质心或最近可达点  
   - 可达性检查：规划服务或 `ComputePathToPose`  
   - 打分：信息增益 - 路径代价 - 风险代价
2. 失败处理：  
   - 超时/无进展：取消目标并拉黑候选  
   - 连续失败：恢复链（等待/旋转/重规划）后再回退
3. 停止条件：  
   - 连续 `N` 轮无 frontier  
   - 地图增长低于阈值  
   - 达到最大探索时间

完成标准：
1. 机器人持续生成并执行自主探索目标。
2. 不在同一区域持续死循环。
3. 停止条件触发后能正确转入 `RETURN_HOME`。

---

## P3：回家与保存导出自动化

目标：  
可靠回到起点并持久化地图产物。

任务：
1. 启动稳定窗口后记录 `home_pose`（`map->base_link`）。
2. `RETURN_HOME` 阶段发送 `NavigateToPose(home_pose)`。
3. `SAVE_EXPORT` 阶段：  
   - 通过 `/write_state` 保存 `.pbstream`  
   - 用 `cartographer_pbstream_to_ros_map` 导出 `.yaml/.pgm`  
   - 时间戳命名，避免覆盖

完成标准：
1. 10 次运行回家成功率 >= 95%。
2. 回家误差：`xy <= 0.15 m`，`yaw <= 10 deg`。
3. 导出的地图可被定位栈加载。

---

## P4：语义货架叠加层

目标：  
在不修改占据栅格像素的前提下叠加语义货架坐标。

任务：
1. 定义 `shelves.yaml/json`（`shelf_id -> pose`）格式。
2. 增加 `semantic_overlay` 节点：  
   - 加载参考语义地图  
   - 估计 `T_ref_map_to_new_map`  
   - 发布对齐后的 marker + 查询服务
3. V1 对齐：  
   同起点假设 + 手工两点标定。
4. V2 对齐：  
   自动配准（ICP 或特征匹配类方案）。

完成标准：
1. V1 叠加误差 <= 0.20 m。
2. 任务层可按 `shelf_id` 查询目标并下发 Nav2 导航。

---

## P5：参数稳定化与回归

目标：  
在 `0.03` 分辨率下稳定导航质量和计算负载。

任务：
1. 维护两套参数档位：  
   - 慢速探索档  
   - 任务执行档
2. 渐进式调参（每次改 1-2 个参数并记录日志）。
3. 跟踪关键指标：  
   - CPU/内存负载  
   - 规划/控制时序稳定性  
   - 动态障碍通过成功率  
   - 手动接管响应

完成标准：
1. 控制环频率稳定，无持续跌频。
2. 动态障碍场景下振荡/卡滞事件下降。
3. 两套档位具备可复现实验报告。

---

## 4. Nav2 参数基线与调参建议

说明：  
“当前理由/预期效果”来自当前参数与栈行为的工程推断。

## 4.1 Costmap 参数

| 参数路径 | 当前值 | 含义 | 当前理由 / 预期效果 | 探索慢速档建议 |
|---|---:|---|---|---|
| `local_costmap.update_frequency` | `5.0` | 本地地图更新频率 | 动态响应与负载平衡 | CPU 允许时测 `8~10` |
| `local_costmap.publish_frequency` | `2.0` | 本地地图发布频率 | 主要影响可视化，不是控制主环 | 保持 |
| `local_costmap.width/height` | `3/3` | 本地窗口大小（m） | 近场聚焦、前视较短 | 可增至 `4~5` |
| `local_costmap.resolution` | `0.03` | 本地栅格分辨率 | 精度高、负载也高 | 保持 `0.03` |
| `local_costmap.robot_radius` | `0.40` | 机器人碰撞包络 | 偏保守、更安全 | 风险高时试 `0.42~0.45` |
| `inflation_radius` | `0.55` | 膨胀半径 | 路径更保守 | 保持或增至 `0.60` |
| `cost_scaling_factor` | `3.0` | 膨胀衰减斜率 | 折中 | 擦边则降、过保守则升 |
| `voxel_layer.z_resolution/z_voxels` | `0.05 / 16` | 体素化配置 | 对当前平台够用 | 保持 |
| `cloud.obstacle_max_range` | `2.5` | 障碍观测距离 | 近中距离优先 | 空旷环境可试 `3.0` |
| `cloud.raytrace_max_range` | `3.0` | 清障射线距离 | 中距离清障能力 | 保持 |
| `global_costmap.resolution` | `0.03` | 全局分辨率 | 全局规划精细但更耗算力 | 算力不足再退 `0.05` |
| `global_costmap.update_frequency` | `1.0` | 全局地图更新频率 | 适合较静态场景 | 保持 |
| `track_unknown_space` | `true` | 是否跟踪未知区 | frontier 必需 | 必须保持 `true` |

## 4.2 规划器与控制器参数

| 参数路径 | 当前值 | 含义 | 当前理由 / 预期效果 | 探索慢速档建议 |
|---|---:|---|---|---|
| `GridBased.plugin` | `NavFnPlanner` | 全局规划器 | 稳定基线 | 保持 |
| `GridBased.use_astar` | `false` | Dijkstra/A* 切换 | 行为稳健可预期 | 先保持，后续可基准对比 |
| `GridBased.tolerance` | `0.5` | 规划终点容差 | 拥挤环境更易收敛 | 可降至 `0.2~0.3` |
| `controller_frequency` | `20.0` | 控制循环频率 | 标准值 | 仅在负载高时降至 `15` |
| `FollowPath.max_vel_x` | `0.26` | 最大前进速度 | 中速 | 探索可降至 `0.15~0.20` |
| `FollowPath.max_vel_y` | `0.20` | 最大横移速度 | 全向底盘开启 | 探索可降至 `0.08~0.15` |
| `FollowPath.max_vel_theta` | `1.0` | 最大角速度 | 转向偏激进 | 探索可降至 `0.6~0.8` |
| `acc_lim_x/y/theta` | `2.5/1.5/3.2` | 加速度限幅 | 响应快但可能生硬 | 降低可更平滑 |
| `decel_lim_x/y/theta` | `-2.5/-1.5/-3.2` | 减速度限幅 | 制动强 | 与加速度联调 |
| `vx/vy/vtheta_samples` | `20/15/20` | 采样密度 | 质量高、计算重 | 可先试 `15/10/15` |
| `sim_time` | `1.7` | 轨迹前瞻时间 | 中等前瞻 | 在 `1.5~2.0` 调整 |
| `BaseObstacle.scale` | `0.02` | 障碍代价权重 | 可能偏低 | 先升到 `0.05~0.10` |
| `PathAlign/PathDist` | `32/32` | 路径贴合权重 | 可能过于贴路径 | 人群动态场景可略降 |
| `GoalDist.scale` | `24` | 目标靠近权重 | 收敛较好 | 过冲时可下调 |

## 4.3 进度检查、恢复与平滑器

| 参数路径 | 当前值 | 含义 | 当前理由 / 预期效果 | 探索慢速档建议 |
|---|---:|---|---|---|
| `progress_checker.required_movement_radius` | `0.5` | 判定“有进展”所需位移 | 狭窄区域可能偏大 | 可降至 `0.2~0.3` |
| `progress_checker.movement_time_allowance` | `10.0` | 进展超时窗口 | 中等 | 拥挤场景可增至 `12~15` |
| `goal_checker.xy/yaw_goal_tolerance` | `0.25/0.25` | 终点位姿容差 | 实用折中 | 保持 |
| `behavior_plugins` | `spin/backup/drive_on_heading/wait/assisted_teleop` | 恢复行为集合 | 基线完整 | 优先使用 `wait + spin + replan` |
| `velocity_smoother.feedback` | `OPEN_LOOP` | 平滑反馈模式 | 简单稳定 | 先保持 |
| `velocity_smoother.max_velocity` | `[0.26,0.20,1.0]` | 平滑器速度上限 | 与 DWB 对齐 | 探索档可同步下调 |

## 4.4 AMCL 状态

| 参数路径 | 当前值 | 说明 |
|---|---:|---|
| `amcl.tf_broadcast` | `false` | 避免与 Cartographer 的 `map->odom` 冲突 |
| `amcl.scan_topic` | `/scan_fullframe` | 参数存在，但当前定位主导仍是 Cartographer |

---

## 5. EKF + Cartographer 融合链

## 5.1 当前配置概述

1. EKF 融合 `/odom_raw` 与 IMU，输出 `/odom`。
2. Cartographer 使用里程计先验（`use_odometry=true`）。
3. Cartographer 不直接使用 IMU（`use_imu_data=false`）。

## 5.2 建议

1. 先保持当前分层融合方案不变。
2. 优先保证时间戳一致性、TF 完整性、协方差合理性。
3. 仅在基线稳定且漂移问题明确时，再评估 `use_imu_data=true`。

---

## 6. 未完成模块设计

## 6.1 `cmd_vel_arbiter`（新增）

职责：
1. 输入：`/cmd_vel_manual`、`/cmd_vel_auto`、`/manual_override`。
2. 输出：`/cmd_vel`。
3. 仲裁：
   - `manual_override=true`：只放行手动。
   - `manual_override=false`：手动活动时仍优先；超时回落到自动。
4. 安全：
   - 源超时时发停止帧
   - 源切换时注入一帧零速，抑制指令突变

## 6.2 `mission_orchestrator`（新增）

职责：
1. 管理任务状态迁移与跨节点动作编排。
2. 处理抢占：
   - 手动接管时暂停/取消自动动作
   - 释放后从合法状态恢复

核心接口：
1. 服务/动作：  
   `start_mission`、`pause_mission`、`resume_mission`、`finish_mapping`
2. 动作客户端：  
   `navigate_to_pose`、`follow_waypoints`
3. 服务客户端：  
   `/write_state`

## 6.3 `frontier_explorer`（新增）

算法规划：
1. frontier 提取：`unknown(-1)` 邻接 `free(0)`。
2. 聚类：BFS 连通域。
3. 目标候选：质心 + 最近可达点回退。
4. 打分：  
   `score = w_gain * info_gain - w_path * path_len - w_risk * risk`
5. 失败策略：  
   - 短期 TTL 拉黑  
   - 连续失败触发恢复链

默认停止条件：
1. `no_frontier_rounds >= 5`
2. `new_area_ratio < 1%` 持续 `60s`
3. `max_explore_time`（例如 20 分钟）

## 6.4 `auto_frontier_mission.launch.py`（新增）

内部组合：
1. `slam_mapping_stack.launch.py`（默认 `with_collision:=true`）
2. Nav2 bringup（探索参数）
3. `cmd_vel_arbiter`
4. `frontier_explorer`（可开关）
5. `mission_orchestrator`

推荐 launch 参数：
1. `map_name:=run_<timestamp>`
2. `with_rviz:=true/false`
3. `explore_profile:=slow`

## 6.5 `semantic_overlay`（新增）

职责：
1. 加载 `shelves.yaml/json`
2. 估计 `T_ref_map_to_new_map`
3. 发布货架 marker + 查询服务

V1：
1. 同起点假设 + 手工两点标定
2. 目标叠加误差 `<= 0.20m`

---

## 7. 验证计划与验收标准

## 7.1 感知与定位链路

检查项：
1. `/cloud_all_fields_fullframe`、`/sick_scansegment_xd/imu` 在线。
2. `/odom_raw`、`/odom` 连续发布。
3. TF 链完整：`map -> odom -> base_link -> lidar_link`。

命令：
```bash
ros2 topic hz /cloud_all_fields_fullframe
ros2 topic hz /sick_scansegment_xd/imu
ros2 topic hz /odom_raw
ros2 topic hz /odom
ros2 run tf2_tools view_frames
```

通过标准：
1. 不出现持续 TF 冲突/超时。
2. `/odom` 频率接近 EKF 目标频率并稳定。

## 7.2 `cmd_vel_arbiter`（人工接管）

检查项：
1. 自动运动中触发 `manual_override=true`。
2. 机器人快速停止自动控制。
3. 手动速度控制可生效。
4. 释放接管后自动流程可恢复。

通过标准：
1. 接管延迟 <= 150 ms。
2. 制动距离 <= 0.20 m（慢速探索档）。
3. 不出现控制权来回抖动。

## 7.3 Frontier 探索

检查项：
1. 持续生成新探索目标。
2. 失败目标进入临时黑名单。
3. 停止条件按设计触发。

通过标准：
1. 连续 10 分钟以上无明显死循环。
2. 地图覆盖率先增长后收敛。

## 7.4 回家与地图产物

检查项：
1. `home_pose` 记录正确。
2. 回家执行成功。
3. 产出 `.pbstream + .yaml + .pgm`。

通过标准：
1. 10 次运行回家成功率 >= 95%。
2. 终点误差 `xy <= 0.15 m`、`yaw <= 10 deg`。

## 7.5 动态障碍与恢复行为

检查项：
1. 行人干扰时机器人能够减速/绕行/等待。
2. 超声波方向限向持续有效。
3. 恢复策略不触发不安全倒车。

通过标准：
1. 测试中零碰撞。
2. 卡住恢复成功率 >= 90%。

## 7.6 `0.03` 分辨率下性能回归

检查项：
1. 长时运行 CPU/内存稳定。
2. 控制/规划频率稳定。
3. 无持续规划超时风暴。

通过标准：
1. 连续 30 分钟运行无失稳。
2. 无持续高频振荡。

---

## 8. 里程碑交付建议

1. `M1`：`cmd_vel_arbiter + manual_override + 接管测试脚本`
2. `M2`：`mission_orchestrator` V1（固定轨迹）+ 回家 + 自动保存/导出
3. `M3`：`frontier_explorer` + 失败回退 + 停止条件
4. `M4`：`auto_frontier_mission.launch.py` 一键启动
5. `M5`：`semantic_overlay` V1
6. `M6`：调参与回归报告（探索档 vs 任务档）

---

## 9. 风险与缓解

1. 物理遮挡导致 LiDAR 盲区：  
   保留超声波底层限向，优先前进与旋转复观测。
2. 控制权切换不稳定：  
   在仲裁器内强制源锁与超时机制。
3. `0.03` 分辨率算力压力：  
   先用慢速档，稳定后再提速。
4. Frontier 死循环：  
   使用 TTL 黑名单 + 失败回退策略。
5. 回家失败：  
   有界重试 + 中间点回退策略。

---

## 10. 结论

在当前代码基线上，风险最低、收益最高的路线仍然是：

`EKF + Cartographer + Nav2 + Frontier + Safety + Mission Orchestrator`

必须先做的不是 frontier 本身，而是控制权架构（`cmd_vel` 仲裁 + 手动接管），因为它是所有自治行为的安全基础。
