# Robot Autonomy Frontier Adaptation Plan（实现对齐更新版）

日期：2026-03-05

## 0. 目的

本文档是原始 adaptation plan 的“按实现对齐”版本。  
原始文件保持不变：`ROBOT_AUTONOMY_FRONTIER_ADAPTATION_PLAN.md`。

本更新在不改变总体适配方向的前提下，补充：
1. 仓库代码对应的 `P0-P5` 当前实现状态。
2. 代码中已经落地的模块行为与接口细节。
3. 仍需硬件验收的剩余项。

---

## 1. 当前基线（按代码核对）

## 1.1 当前运行链路

1. `robot_localization` EKF：  
   `/odom_raw + /sick_scansegment_xd/imu -> /odom`
2. `Cartographer`（在线 SLAM）：  
   `use_odometry=true`、`use_imu_data=false`，执行 scan matching + pose graph。
3. `Nav2`：  
   全局规划器 `NavFnPlanner (use_astar=false)`，局部控制器 `DWBLocalPlanner`。
4. 控制权与安全链路：  
   `cmd_vel_arbiter` 将 manual/auto 融合为 `/cmd_vel`，随后 `nav2_serial_bridge` 在下发 STM32 前应用超声波方向阻断（`/front_alert`, `/back_alert`, `/left_alert`, `/right_alert`）。
5. 分辨率默认值：  
   地图导出默认 `0.03`，Nav2 local/global costmap 默认 `0.03`。

## 1.2 原始缺口与当前状态对照

| 原计划缺口 | 当前状态 |
|---|---|
| 缺少 frontier 探索节点 | 已实现（`frontier_explorer.py`） |
| 缺少手动优先速度仲裁 | 已实现（`cmd_vel_arbiter.py`） |
| 缺少 mission 状态机 | 已实现（`mission_orchestrator.py`） |
| 缺少一键 autonomous mission launch | 已实现（`auto_map_mission_v1.launch.py`、`auto_frontier_mission.launch.py`、`nav_assistant`） |
| 缺少语义货架叠加层 | 已实现（`semantic_overlay.py`，含手动与自动对齐） |

额外现状：
1. `P5` profile 选择已接入 `frontier-mission`（`--explore-profile`）。
2. `P5` 参数对比报告工具已实现（`nav_profile_report`）。
3. 自动运行时回归监控守护进程在当前版本中按决策不纳入。

---

## 2. 目标架构（按当前实现）

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

Mission Orchestrator（frontier 模式）:
BOOT -> AUTO_EXPLORE -> RETURN_HOME -> SAVE_EXPORT -> LOCALIZE_READY

Mission Orchestrator（P1 fixed 模式）:
BOOT -> AUTO_MAP_V1 -> RETURN_HOME -> SAVE_EXPORT -> LOCALIZE_READY

Semantic Overlay（frontier mission 可选）:
shelves.yaml/json + reference_map_yaml + /map -> 对齐 marker + 货架位姿服务
```

---

## 3. 实施优先级与状态（含实现细节）

## P0（最高优先级）：控制权与安全闭环

状态：已实现。

实现细节：
1. `cmd_vel_arbiter` 输入：`/cmd_vel_manual`、`/cmd_vel_auto`（及配置的 auto topics）、`/manual_override`。
2. 仲裁逻辑：  
   - `manual_override=true`：manual fresh 则放行，否则停止。  
   - `manual_override=false`：manual（fresh）> auto（fresh）> stop。
3. 超时行为：  
   `manual_cmd_timeout` 与 `auto_cmd_timeout`（默认均 `0.35s`）。
4. 切源安全：  
   支持切源注入零速帧（默认 `stop_on_source_switch=true`）。
5. 超声波方向限向仍在 `nav2_serial_bridge` 中执行。

仍需机器人侧验收：
1. 接管延迟（`<= 150 ms`）与制动距离（`<= 0.20 m`）需实测。

---

## P1：任务状态机骨架（不含 frontier）

状态：已实现。

实现细节：
1. `mission_orchestrator` 提供：  
   `start_mission`、`pause_mission`、`resume_mission`、`finish_mapping`。
2. P1 状态流：  
   `BOOT -> AUTO_MAP_V1 -> RETURN_HOME -> SAVE_EXPORT -> LOCALIZE_READY`。
3. `BOOT` 在 `boot_capture_delay_sec` 后通过 TF（`map -> base_link`）捕获 `home_pose`。
4. `AUTO_MAP_V1` 完成条件：  
   - `mapping_timeout_sec`  
   - `mapping_max_distance_m`  
   - `finish_mapping` 服务触发
5. 手动接管行为：  
   接管时暂停/取消当前动作，释放后自动恢复。
6. 地图命名行为：  
   产物同名时自动生成唯一名称，避免覆盖。

---

## P2：Frontier Explorer 集成

状态：已实现。

实现细节：
1. `frontier_explorer` 订阅 `/map`，按“未知格邻接自由格”提取 frontier。
2. frontier 聚类使用 BFS 连通域。
3. 可达性与路径代价采用栅格 BFS：  
   - 从机器人自由格构建距离场  
   - 路径长度 = `steps * map_resolution`  
   - 支持 `max_candidate_path_len_m` 过滤（`0` 表示关闭上限）
4. 候选打分：  
   `score = w_gain * info_gain - w_path * path_len - w_risk * risk`。
5. 失败处理：  
   目标超时取消、候选点 TTL 拉黑、连续失败冷却恢复。
6. 停止条件：  
   - 连续无 frontier 轮次超限  
   - 新增面积比在时间窗内过低  
   - 探索时间达到上限
7. 与 mission 集成：  
   由 orchestrator 通过 `/frontier_explorer/start` 与 `/frontier_explorer/stop` 控制启停。

---

## P3：回家与保存导出自动化

状态：已实现。

实现细节：
1. `RETURN_HOME` 发送 `NavigateToPose(home_pose)`，支持有界重试（`home_retry_limit`）。
2. `SAVE_EXPORT` 流程：  
   - 通过 `/write_state` 保存 pbstream  
   - 通过 `cartographer_pbstream_to_ros_map` 导出地图
3. 成功路径进入 `LOCALIZE_READY`。
4. 失败路径进入 `ERROR` 并给出原因日志。

---

## P4：语义货架叠加

状态：已实现（支持 V1 与 V2 风格能力）。

实现细节：
1. `semantic_overlay` 从 YAML/JSON 加载货架（`shelf_id`, `x`, `y`, `yaw`）。
2. 服务接口：  
   - `/semantic_overlay/query_shelf_pose`  
   - `/semantic_overlay/set_alignment`（手动两点对齐）  
   - `/semantic_overlay/reload`  
   - `/semantic_overlay/auto_align`（自动地图模式对齐）
3. marker 发布：  
   在 map 坐标系发布对齐后的货架 marker 与 label。
4. marker 清理：  
   refresh 路径 `DELETEALL + stale marker delete`，避免 RViz 残留。
5. 自动对齐行为：  
   - 订阅当前 `/map` 占据栅格  
   - 从 `reference_map_yaml` 加载参考模式（yaml + pgm）  
   - 在当前对齐附近做局部 yaw/平移搜索  
   - 可选全局粗搜索并做一次细化  
   - `auto_align_on_start=true` 时启动重试
6. 手动触发 `auto_align` 成功后会清除 pending，避免重复自动重跑。

---

## P5：参数稳定化与回归

状态：profile 工作流已实现；运行时回归追踪仍为手工。

实现细节：
1. 两套 Nav2 档位文件已就位：  
   - `config/nav2_params_explore_slow.yaml`  
   - `config/nav2_params_task_run.yaml`
2. `nav_assistant frontier-mission` 支持：  
   - `--explore-profile slow|task`  
   - 可选 `--nav2-params-file` 显式覆盖
3. 参数对比工具：  
   `ros2 run robot_navigation nav_profile_report --base ... --target ... --output ...`
4. 自动回归监控守护进程按当前决策不纳入本版本。

剩余验收工作：
1. CPU/内存/频率/动态障碍指标仍需实机测试记录。

---

## 4. Nav2 参数基线与调参建议

说明：  
“当前理由 / 预期效果”是根据当前参数与系统行为得到的工程推断。

## 4.1 Costmap 参数

| 参数路径 | 当前值 | 含义 | 当前理由 / 预期效果 | 慢速探索建议 |
|---|---:|---|---|---|
| `local_costmap.update_frequency` | `5.0` | 本地地图更新频率 | 动态响应与负载平衡 | CPU 允许时测 `8~10` |
| `local_costmap.publish_frequency` | `2.0` | 本地地图发布频率 | 主要影响可视化 | 保持 |
| `local_costmap.width/height` | `3/3` | 本地窗口大小（m） | 近场聚焦 | 可增至 `4~5` |
| `local_costmap.resolution` | `0.03` | 本地栅格分辨率 | 精度高、负载高 | 保持 `0.03` |
| `local_costmap.robot_radius` | `0.40` | 碰撞包络半径 | 偏保守更安全 | 可试 `0.42~0.45` |
| `inflation_radius` | `0.55` | 膨胀半径 | 路径更保守 | 保持或增至 `0.60` |
| `cost_scaling_factor` | `3.0` | 膨胀衰减 | 折中 | 擦边则降，过保守则升 |
| `voxel_layer.z_resolution/z_voxels` | `0.05 / 16` | 体素层配置 | 当前平台够用 | 保持 |
| `cloud.obstacle_max_range` | `2.5` | 障碍观测范围 | 近中距离优先 | 大场地可试 `3.0` |
| `cloud.raytrace_max_range` | `3.0` | 清障射线范围 | 中距离清障能力 | 保持 |
| `global_costmap.resolution` | `0.03` | 全局栅格分辨率 | 全局规划更细但更耗算力 | 算力受限时退 `0.05` |
| `global_costmap.update_frequency` | `1.0` | 全局更新频率 | 静态环境足够 | 保持 |
| `track_unknown_space` | `true` | 跟踪未知区 | frontier 必需 | 必须保持 `true` |

## 4.2 规划器与控制器参数

| 参数路径 | 当前值 | 含义 | 当前理由 / 预期效果 | 慢速探索建议 |
|---|---:|---|---|---|
| `GridBased.plugin` | `NavFnPlanner` | 全局规划器 | 稳定基线 | 保持 |
| `GridBased.use_astar` | `false` | Dijkstra/A* 切换 | 行为稳健 | 先保持，后续基准对比 |
| `GridBased.tolerance` | `0.5` | 规划终点容差 | 拥挤环境更易收敛 | 可降至 `0.2~0.3` |
| `controller_frequency` | `20.0` | 控制环频率 | 标准值 | 必要时降至 `15` |
| `FollowPath.max_vel_x` | `0.26` | 最大前进速度 | 中速 | 探索可降至 `0.15~0.20` |
| `FollowPath.max_vel_y` | `0.20` | 最大横移速度 | 全向底盘开启 | 探索可降至 `0.08~0.15` |
| `FollowPath.max_vel_theta` | `1.0` | 最大角速度 | 转向偏激进 | 探索可降至 `0.6~0.8` |
| `acc_lim_x/y/theta` | `2.5/1.5/3.2` | 加速度限幅 | 响应快但可能生硬 | 适当下调更平滑 |
| `decel_lim_x/y/theta` | `-2.5/-1.5/-3.2` | 减速度限幅 | 制动偏强 | 与加速度联调 |
| `vx/vy/vtheta_samples` | `20/15/20` | 采样密度 | 质量高但计算重 | 可试 `15/10/15` |
| `sim_time` | `1.7` | 轨迹前瞻 | 中等前瞻 | 在 `1.5~2.0` 调整 |
| `BaseObstacle.scale` | `0.02` | 障碍代价权重 | 可能偏低 | 先升至 `0.05~0.10` |
| `PathAlign/PathDist` | `32/32` | 贴路径权重 | 可能过度贴路径 | 动态场景可略降 |
| `GoalDist.scale` | `24` | 趋近目标权重 | 收敛较好 | 过冲时可下调 |

## 4.3 进度检查、恢复与平滑器

| 参数路径 | 当前值 | 含义 | 当前理由 / 预期效果 | 慢速探索建议 |
|---|---:|---|---|---|
| `progress_checker.required_movement_radius` | `0.5` | 判定进展所需位移 | 狭窄场景可能偏大 | 可降至 `0.2~0.3` |
| `progress_checker.movement_time_allowance` | `10.0` | 进展超时窗口 | 中等 | 拥挤时可增至 `12~15` |
| `goal_checker.xy/yaw_goal_tolerance` | `0.25/0.25` | 终点容差 | 实用折中 | 保持 |
| `behavior_plugins` | `spin/backup/drive_on_heading/wait/assisted_teleop` | 恢复行为集合 | 基线完整 | 优先 `wait + spin + replan` |
| `velocity_smoother.feedback` | `OPEN_LOOP` | 平滑反馈模式 | 简单稳定 | 保持 |
| `velocity_smoother.max_velocity` | `[0.26,0.20,1.0]` | 平滑器速度上限 | 与 DWB 对齐 | 探索档可同步下调 |

## 4.4 AMCL 状态

| 参数路径 | 当前值 | 说明 |
|---|---:|---|
| `amcl.tf_broadcast` | `false` | 避免与 Cartographer 的 `map->odom` 冲突 |
| `amcl.scan_topic` | `/scan_fullframe` | 配置存在，但定位主导仍是 Cartographer |

---

## 5. EKF + Cartographer 融合链

## 5.1 当前配置摘要

1. EKF 融合 `/odom_raw` 与 IMU 输出 `/odom`。
2. Cartographer 使用 odom 先验（`use_odometry=true`）。
3. Cartographer 不直接使用 IMU（`use_imu_data=false`）。

## 5.2 建议

1. 先保持分层融合链不变。
2. 优先保证时间戳一致、TF 完整、协方差合理。
3. 仅在基线稳定并确认漂移问题后，再评估 `use_imu_data=true`。

---

## 6. 已实现模块细节

## 6.1 `cmd_vel_arbiter`

已实现行为：
1. 输入：`/cmd_vel_manual`、配置的 auto topics（默认 `/cmd_vel_auto`, `/cmd_vel_nav`, `/cmd_vel_smoothed`）以及 `/manual_override`。
2. 输出：`/cmd_vel`。
3. 优先级：  
   - `manual_override=true`：manual fresh 则放行，否则 stop。  
   - `manual_override=false`：manual fresh > auto fresh > stop。
4. 安全护栏：  
   - 源超时即 stop  
   - 可选切源 stop 帧（`stop_on_source_switch=true`）  
   - 自动源中会剔除输出 topic，避免自环。

## 6.2 `mission_orchestrator`

已实现职责：
1. 负责 mission 状态迁移与动作编排。
2. 负责抢占恢复：  
   - 手动接管时 pause 并 cancel 当前动作  
   - 释放后按 pause 原因自动恢复。
3. 支持两种建图模式：  
   - `fixed`：`AUTO_MAP_V1`  
   - `frontier`：`AUTO_EXPLORE`

核心接口：
1. 服务：`start_mission`, `pause_mission`, `resume_mission`, `finish_mapping`。
2. 动作客户端：`navigate_to_pose`, `follow_waypoints`（支持 namespace 解析）。
3. 服务客户端：`/write_state`, `/frontier_explorer/start`, `/frontier_explorer/stop`。

## 6.3 `frontier_explorer`

已实现算法：
1. frontier 提取：unknown 邻接 free。
2. 聚类：BFS 连通域。
3. 候选选择：质心邻近的可达自由格。
4. 可达性/路径代价：从机器人位置构建 free-space BFS 距离场。
5. 打分：`score = w_gain * info_gain - w_path * path_len - w_risk * risk`。
6. 失败策略：超时取消、TTL 拉黑、连续失败冷却。
7. 停止条件：无 frontier 轮次、低增量面积窗口、最大探索时长。

公开接口：
1. 服务：`/frontier_explorer/start`, `/frontier_explorer/stop`。
2. 话题：`/frontier_explorer/state`, `/frontier_explorer/done`, `/frontier_explorer/current_goal`。

## 6.4 `auto_frontier_mission.launch.py`

已实现组合：
1. `slam_mapping_stack.launch.py`（包含 `cmd_vel_arbiter`、`nav2_serial_bridge`、Cartographer、EKF、可选超声碰撞 launch）。
2. Nav2 bringup。
3. `frontier_explorer`。
4. frontier 模式的 `mission_orchestrator`。
5. 可选 `semantic_overlay`（`with_semantic_overlay` 开关）。

已实现参数串接重点：
1. frontier 健壮性参数（超时、黑名单、恢复、路径长度上限）。
2. mission 保存导出参数。
3. semantic 自动对齐相关参数。

## 6.5 `semantic_overlay`

已实现职责：
1. 从 YAML/JSON 加载货架。
2. 应用刚体变换并发布 marker。
3. 提供货架查询与对齐服务。
4. 提供基于 `/map` 的自动模式对齐。
5. 配置重载后清理 stale marker。

已实现接口：
1. 服务：  
   `/semantic_overlay/query_shelf_pose`、`/semantic_overlay/set_alignment`、`/semantic_overlay/auto_align`、`/semantic_overlay/reload`。
2. 话题：  
   `/semantic_overlay/markers`。

## 6.6 `nav_assistant` 与 P5 profile 流程

已实现行为：
1. `mission-p1` 与 `frontier-mission` 一键流程。
2. `frontier-mission` profile 选择：  
   `--explore-profile slow|task`，并支持显式 nav2 参数覆盖。
3. frontier mission 已支持 semantic auto-align CLI 参数串接。
4. `P5` 参数对比支持：`nav_profile_report` 输出 markdown。

---

## 7. 验证计划与验收标准

当前代码主验收清单：  
`ROBOT_AUTONOMY_FRONTIER_P5_ACCEPTANCE_CHECKLIST.md`

## 7.1 感知与定位链路

检查项：
1. `/cloud_all_fields_fullframe`、`/sick_scansegment_xd/imu` 存活。
2. `/odom_raw`、`/odom` 连续。
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
1. 无持续 TF 冲突或超时。
2. `/odom` 频率稳定在目标附近。

## 7.2 `cmd_vel_arbiter`（人工接管）

检查项：
1. 自动运动中触发 `manual_override=true`。
2. 自动指令被快速停止。
3. manual 指令可控制底盘。
4. 释放接管后自动恢复。

通过标准：
1. 接管延迟 <= 150 ms。
2. 制动距离 <= 0.20 m（慢速档）。
3. 无 manual/auto 权限抖动。

## 7.3 Frontier 探索

检查项：
1. 持续生成探索目标。
2. 失败目标进入临时拉黑。
3. 停止条件按预期触发。

通过标准：
1. 10 分钟以上无持续死循环。
2. 地图覆盖率增长后收敛。

## 7.4 回家与地图产物

检查项：
1. `home_pose` 记录正确。
2. 回家动作成功。
3. 生成 `.pbstream + .yaml + .pgm`。

通过标准：
1. 10 次运行回家成功率 >= 95%。
2. 终点误差 `xy <= 0.15 m`，`yaw <= 10 deg`。

## 7.5 动态障碍与恢复行为

检查项：
1. 行人干扰时能减速/绕行/等待。
2. 超声波方向阻断保持有效。
3. 恢复链避免盲区不安全倒车。

通过标准：
1. 测试中零碰撞。
2. 卡住恢复成功率 >= 90%。

## 7.6 0.03 分辨率性能回归

检查项：
1. 长时 CPU/内存稳定性。
2. 控制器/规划器频率稳定性。
3. 无持续 planning timeout 风暴。

通过标准：
1. 连续 30 分钟运行无失稳。
2. 无持续高频振荡。

---

## 8. 里程碑交付建议

| 里程碑 | 范围 | 当前状态 |
|---|---|---|
| `M1` | `cmd_vel_arbiter + manual_override + 接管链路` | 已实现；需硬件验收 |
| `M2` | `mission_orchestrator` fixed 模式 + 回家 + 保存导出 | 已实现 |
| `M3` | `frontier_explorer` + 失败处理 + 停止条件 | 已实现 |
| `M4` | `auto_frontier_mission.launch.py` 一键启动 | 已实现 |
| `M5` | `semantic_overlay` V1 + V2风格自动对齐 | 已实现 |
| `M6` | profile 稳定化与回归证据 | 部分实现（工具完成，运行指标采集仍手工） |

---

## 9. 风险与缓解

1. LiDAR 物理遮挡盲区：  
   保留超声波低层限向，优先前进与旋转复观测。
2. 控制权切换不稳定：  
   仲裁层强制 source lock + timeout。
3. `0.03` 分辨率算力负载：  
   先用慢速档，稳定后再提升速度。
4. Frontier 死循环：  
   使用 TTL 黑名单 + 失败回退。
5. 回家失败：  
   使用有界重试与中间点回退。

---

## 10. 结论

当前 adaptation 路线已在代码中打通主链路：

`EKF + Cartographer + Nav2 + cmd_vel 仲裁 + Mission Orchestrator + Frontier + Semantic Overlay`

后续主要工作是“验收与调优证据补齐”，而非核心模块补建：
1. 接管延迟与制动距离硬件实测；
2. 多轮回家/错误率统计；
3. 两套 Nav2 profile 在动态障碍与性能方面的回归证据。
