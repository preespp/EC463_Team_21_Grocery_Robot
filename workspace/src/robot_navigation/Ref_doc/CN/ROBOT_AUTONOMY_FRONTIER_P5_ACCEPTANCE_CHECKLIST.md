# 机器人自治 Frontier P5 验收清单

日期：2026-03-05  
范围：`frontier-mission` 端到端验收，覆盖至 P5 运行链路，并包含语义层自动对齐。  
说明：按当前决策，不包含自动回归监控守护进程。

## 测试输入（执行前填写）

- `MAP_OUTPUT_DIR`：`<绝对输出目录>`
- `MAP_NAME_PREFIX`：`<运行名前缀>`
- `SHELVES_FILE`：`<货架配置yaml绝对路径>`
- `REFERENCE_MAP_YAML`：`<参考地图yaml绝对路径>`

## 验收检查项

| ID | 动作 / 命令 | 预期输出（逐行关键字） | 验收标准 | 结果 |
|---|---|---|---|---|
| AC-01 | `colcon build --packages-select robot_interfaces robot_navigation` | 输出包含 `Finished <<< robot_interfaces`、`Finished <<< robot_navigation`，以及 2 个包完成的汇总行。 | 命令返回码为 `0`，两个包均构建成功。 | `[ ] 通过 / [ ] 失败` |
| AC-02 | `source install/setup.bash` 后执行 `ros2 pkg executables robot_navigation` | 输出包含 `frontier_explorer`、`mission_orchestrator`、`semantic_overlay`、`nav_assistant`。 | 必要可执行入口全部可见。 | `[ ] 通过 / [ ] 失败` |
| AC-03 | `ros2 run robot_navigation nav_assistant frontier-mission --explore-profile slow --dry-run` | 打印的 launch 命令包含 `nav2_params_file:=.../nav2_params_explore_slow.yaml`。 | `slow` 档位正确映射到慢速探索参数文件。 | `[ ] 通过 / [ ] 失败` |
| AC-04 | `ros2 run robot_navigation nav_assistant frontier-mission --explore-profile task --dry-run` | 打印的 launch 命令包含 `nav2_params_file:=.../nav2_params_task_run.yaml`。 | `task` 档位正确映射到任务参数文件。 | `[ ] 通过 / [ ] 失败` |
| AC-05 | 启动任务：`ros2 run robot_navigation nav_assistant frontier-mission --with-semantic-overlay true --shelves-file <SHELVES_FILE> --semantic-reference-map-yaml <REFERENCE_MAP_YAML> --semantic-auto-align-on-start true --map-output-dir <MAP_OUTPUT_DIR> --map-name-prefix <MAP_NAME_PREFIX> --explore-profile slow` | 日志包含 `mission_orchestrator ready (autostart=True, mapping_mode=frontier`、`frontier_explorer ready`、`semantic_overlay ready`。 | 三个关键节点启动正常，且无立即报错。 | `[ ] 通过 / [ ] 失败` |
| AC-06 | 观察 `mission_orchestrator` 日志 | 依次出现 `State -> BOOT`、`State -> AUTO_EXPLORE`。 | 状态机从 `BOOT` 正常进入 `AUTO_EXPLORE`。 | `[ ] 通过 / [ ] 失败` |
| AC-07 | `ros2 topic echo /frontier_explorer/state --once`（必要时重复） | 在探索期间出现 `data: RUNNING`。 | Frontier FSM 进入运行态。 | `[ ] 通过 / [ ] 失败` |
| AC-08 | `ros2 topic echo /frontier_explorer/current_goal`（观察约 20 秒） | 至少收到 1 条 `PoseStamped` 目标（map 坐标系）。 | 探索目标持续产生。 | `[ ] 通过 / [ ] 失败` |
| AC-09 | 观察 `semantic_overlay` 启动日志 | 包含 `Loaded reference map pattern: <N> occupied samples`。 | `<N> > 0`，参考地图占用模式加载成功。 | `[ ] 通过 / [ ] 失败` |
| AC-10 | 在 `/map` 可用后观察 `semantic_overlay` 日志 | 包含 `auto_align success: score=...`。 | 自动对齐成功，且分数不低于阈值（默认 `auto_align_min_score=0.45`）。 | `[ ] 通过 / [ ] 失败` |
| AC-11 | `ros2 service call /semantic_overlay/auto_align std_srvs/srv/Trigger "{}"` | 返回包含 `success: true`，消息含 `auto_align success`。 | 手动触发自动对齐成功。 | `[ ] 通过 / [ ] 失败` |
| AC-12 | `ros2 service call /semantic_overlay/query_shelf_pose robot_interfaces/srv/QueryShelfPose "{shelf_id: 'shelf_A1'}"` | 返回包含 `found: true`，且 `pose.header.frame_id: map`。 | 可按 `shelf_id` 查询语义货架位姿。 | `[ ] 通过 / [ ] 失败` |
| AC-13 | `ros2 service call /semantic_overlay/reload std_srvs/srv/Trigger "{}"` | 返回包含 `success: true`，且消息含 `loaded <count> shelves, reference_points=<count>`。 | 重载流程成功，无服务异常。 | `[ ] 通过 / [ ] 失败` |
| AC-14 | 在 RViz 修改货架列表并 reload 后观察 | 旧 marker/标签被清理，仅保留当前配置。 | 不出现残留语义 marker。 | `[ ] 通过 / [ ] 失败` |
| AC-15 | `ros2 topic pub --once /manual_override std_msgs/msg/Bool "{data: true}"` | mission 与 frontier 日志出现 `State -> PAUSED`（如有在途目标应看到取消迹象）。 | 人工接管可及时暂停自治与运动。 | `[ ] 通过 / [ ] 失败` |
| AC-16 | `ros2 topic pub --once /manual_override std_msgs/msg/Bool "{data: false}"` | 日志恢复为探索状态（`State -> AUTO_EXPLORE`，frontier 为 `State -> RUNNING`）。 | 释放接管后能无死锁恢复。 | `[ ] 通过 / [ ] 失败` |
| AC-17 | 观察探索退出阶段 | 日志出现以下之一：`AUTO_EXPLORE finished by frontier explorer done signal`、`AUTO_EXPLORE reached explore_timeout_sec`、`AUTO_EXPLORE completed by finish_mapping service`，随后出现 `State -> RETURN_HOME`。 | 任务能从探索阶段正确切换到回家阶段。 | `[ ] 通过 / [ ] 失败` |
| AC-18 | 观察回家阶段日志 | 出现 `RETURN_HOME succeeded`。 | 回家动作成功，且未进入 `ERROR`。 | `[ ] 通过 / [ ] 失败` |
| AC-19 | 观察保存导出阶段日志 | 出现 `SAVE_EXPORT pbstream saved: ...` 与 `SAVE_EXPORT map export finished: ...`。 | pbstream 保存与地图导出均成功。 | `[ ] 通过 / [ ] 失败` |
| AC-20 | 检查 `<MAP_OUTPUT_DIR>` 输出文件 | 存在 `<map_name>.pbstream`、`<map_name>.yaml`、`<map_name>.pgm`，且文件非空。 | 产物齐全且可用于定位栈加载。 | `[ ] 通过 / [ ] 失败` |
| AC-21 | 观察最终任务状态 | 日志出现 `Mission reached LOCALIZE_READY` 与 `State -> LOCALIZE_READY`。 | 端到端任务完成到 `LOCALIZE_READY`。 | `[ ] 通过 / [ ] 失败` |
| AC-22 | 全程错误扫描 | mission 日志中不出现 `State -> ERROR`。 | 无错误态才可判定通过。 | `[ ] 通过 / [ ] 失败` |

## 总体通过规则

- 关键项必须通过：`AC-01`、`AC-05`、`AC-06`、`AC-10`、`AC-17`、`AC-18`、`AC-19`、`AC-21`、`AC-22`。
- 最终完全通过要求：所有清单项均通过。
