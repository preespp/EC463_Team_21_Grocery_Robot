Robot Navigation Stack Implementation Plan
摘要
新增一份实施计划文档，建议路径为 workspace/src/robot_navigation/Ref_doc/CN/ROBOT_NAV_STACK_EXECUTION_PLAN_2026_03.md。计划固定分 3 条主线推进，并且每条主线都绑定“Codex 可自动读取并回环验证”的标准。

本次修订后的关键默认决策：

建图阶段继续用 Cartographer
建图质量主路径固定为 PointCloud2（沿用当前 points2 接线）
Cartographer 建图质量判断以“内部 SLAM 参数 + 传感器输入 + 导图资产一致性”为准，不把 export-map --resolution 当成建图质量代理
运行时定位切到 EKF + AMCL + map_server + Nav2
Nav2 目标配置默认是 SmacPlanner2D + MPPI(Omni)
MPPI(DiffDrive) 只作为 Omni 失败时的回退 profile
Cartographer 原生 IMU 默认仍保持关闭，直到 tracking_frame 与 IMU frame 共点后再单独验证
关键实现
1. 建图质量、分辨率与地图导出
实现目标：

提高 Cartographer 的真实建图质量，不把“导出栅格分辨率”误当成 SLAM 质量
优先解决旋转畸变、近场几何丢失、空闲空间证据浪费、前端过度依赖 odom 的问题
让导图流程和地图资产一致性可以被脚本自动验证
实现内容：

新增一套质量优先 Cartographer 配置，固定命名为 pico_2d_mapping_quality.lua
保持 mapping launch 使用现有 points2 输入，不新增 sensor_input_mode
D0 默认使用 /cloud_all_fields_fullframe
原 D0-2 输入模式切换任务移除，后续若需要再单独立项
Cartographer 质量优先参数固定为：
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
建图验收优先关注 PointCloud2 频率与 frame 一致性
scan 相关参数可继续发布但不作为 D0 通过门槛
若后续重启 LaserScan 方案，再单独定义 echo 策略
PicoScan 启动 profile 默认锁定为 30 Hz / 0.1°
导图流程继续复用 nav_assistant save-map 和 nav_assistant export-map
但验收逻辑明确区分：
submaps.grid_options_2d.resolution 负责内部 SLAM 分辨率
export-map --resolution 只负责最终栅格化输出分辨率
地图资产同源规则固定：
.pbstream、.yaml、.pgm 同 stem
同一轮导图生成
运行时禁止混搭不同轮次资产
公开接口变化：

cartographer_mapping.launch.py
保持 `points2` remap，不新增输入模式开关
nav_assistant.py
新增只读命令：
verify-mapping-profile
verify-sensors
verify-map-artifacts
Codex 可验证标准：

输入路径验证
读取 launch 和 Lua，确认 num_laser_scans=0、num_point_clouds=1
ros2 node info /cartographer_node 显示订阅的是 points2
传感器频率与 frame
ros2 topic hz /cloud_all_fields_fullframe 在目标频率 ±10%
ros2 topic echo /cloud_all_fields_fullframe --once 的 header.frame_id == lidar_link
ros2 topic hz /sick_scansegment_xd/imu >= 90 Hz
ros2 topic hz /odom 在 50 Hz ±10%
质量参数验证
读取 pico_2d_mapping_quality.lua，逐项核对上述参数值
若 use_imu_data=true，则自动验证 tracking_frame 必须等于 lidar_link 或零平移共点 frame，否则直接判失败
导图资产验证
.pbstream/.yaml/.pgm 三件套都存在
.yaml 的 image 指向存在的 .pgm
.pgm 可被脚本读取并输出宽高
.pbstream/.yaml/.pgm stem 完全一致
文件修改时间差不超过 120s
地图输出验证
export-map --resolution 只检查导出文件满足请求分辨率，不用于判定建图质量通过
建图质量通过条件固定是：
PointCloud2 路径生效（points2 -> /cloud_all_fields_fullframe）
内部 submaps.grid_options_2d.resolution == 0.03
min_range == 0.15
missing_data_ray_length == max_range
motion filter 和 matcher 参数符合目标配置
结果产物
verification/mapping_profile_report.json
verification/sensor_report.json
verification/map_artifacts_report.json
2. Relocalization 实现
实现目标：

把冷启动和运行时重定位从 Cartographer localization 切到 AMCL
区分“固定起点”和“未知起点”两种启动流程
让定位是否收敛变成可脚本判断的条件，而不是依赖 RViz 人眼判断
实现内容：

新增运行时 launch：nav2_amcl_localization_stack.launch.py
运行时定位栈固定只包含：
LiDAR
static TF
serial bridge
EKF
map_server
AMCL
Nav2
localization 栈中不再启动 cartographer_localization.launch.py
新增运行时参数文件 nav2_params_amcl_mppi.yaml
amcl.scan_topic 固定为 /scan_fullframe（仅运行时定位链路）
启动模式固定公开为：
startup_mode:=fixed_pose
startup_mode:=global_localization
fixed_pose
启动 map_server
启动 AMCL
自动发布 dock initialpose
等待收敛后再激活 planner/controller
global_localization
启动 map_server
启动 AMCL
调用 ReinitializeGlobalLocalization
执行低速原地扫描动作
等待收敛后再激活 planner/controller
运行时地图资产固定只使用 .yaml/.pgm
pbstream 只保留给建图链路，不再进入运行时定位链路
若后续低特征区域仍存在唯一性问题，作为 v2 可选增强：
固定开机位姿
AprilTag / 反光柱
人工地标
公开接口变化：

nav2_amcl_localization_stack.launch.py
新增 startup_mode:=fixed_pose|global_localization
nav_assistant.py
新增只读命令：
verify-localization
benchmark-localization
Codex 可验证标准：

运行所有权
ros2 node list 中必须有 amcl 和 map_server
ros2 node list 中不得有 cartographer_node
ros2 lifecycle get /amcl 和 /map_server 都是 active
参数一致性
ros2 param get /amcl scan_topic == /scan_fullframe
localization launch 中不存在 pbstream_file 必需项
固定起点验证
发送 initialpose 后 30s 内 /amcl_pose 开始稳定输出
协方差下降到阈值以下
与 benchmark/localization_cases.yaml 中期望 pose 比较，误差默认要求 < 0.20 m、< 10 deg
全局重定位验证
ReinitializeGlobalLocalization 调用成功
/amcl_pose 频率 >= 1 Hz
收敛后连续 5s 内 map->odom 位移变化 < 0.05 m、角度变化 < 3 deg
协方差持续下降或稳定在阈值以下
资产一致性
localization 只使用 .yaml/.pgm
benchmark 报告记录当前地图 stem
结果产物
verification/localization_report.json
3. Nav2 新算法实现与实机比较
实现目标：

在统一定位和统一地图下比较新的 planner/controller 组合
默认目标是 SmacPlanner2D + MPPI(Omni)
只有在 Omni 失败时才回退到 MPPI(DiffDrive) 或 RPP
实现内容：

固定 5 套 Nav2 profile：
baseline: NavFn(Dijkstra) + DWB
astar_dwb: NavFn(A*) + DWB
smac_mppi_omni: SmacPlanner2D + MPPI(Omni)
smac_mppi_diff: SmacPlanner2D + MPPI(DiffDrive)
smac_rpp: SmacPlanner2D + RPP
所有 profile 统一使用同一套 EKF + AMCL
Nav2 不直接吃 IMU
IMU 只通过 EKF 影响 /odom
因此 Nav2 比较阶段固定先不改 EKF
只有当 smac_mppi_omni 已通过 localization 验证但出现明显 oscillation 或横移跟踪异常时，才允许第二轮 EKF 微调
第二轮 EKF 微调只允许动：
yaw 相关过程噪声
IMU yaw / yaw rate 相关协方差
BT 默认采用 Nav2 官方恢复树
V1 额外加入：
ReinitializeGlobalLocalization
free-space aware backup 占位接口
参考 SCURM_SentryNavigation 的 free-space backup 思路，但不直接照搬 repo 结构
公开接口变化：

新增 Nav2 参数文件：
nav2_params_baseline.yaml
nav2_params_astar_dwb.yaml
nav2_params_smac_mppi_omni.yaml
nav2_params_smac_mppi_diff.yaml
nav2_params_smac_rpp.yaml
nav_assistant.py
新增只读命令：
verify-nav-profile
benchmark-nav
Codex 可验证标准：

profile 所有权
读取参数文件，确认 planner/controller plugin 名称正确
smac_mppi_omni 中 MPPI 的 motion model 固定为 Omni
astar_dwb 中 NavFn.use_astar == true
BT 与恢复
bt_navigator 加载目标 XML
XML 中包含 ReinitializeGlobalLocalization
XML 中保留 ClearCostmap、Spin、Wait、BackUp
free-space backup 在 V1 先做接口占位并被脚本检测到
实机比较矩阵
固定 4 类场景：
长直通道
90 度拐角
窄口通过
回桩/回家
每个场景固定 start/goal 和地图
每个 profile 运行同一组 case
自动统计指标
goal 成功率
到达时间
路径总长度
重规划次数
恢复触发次数
/cmd_vel 平滑度
停滞时间
若录制了 /odom 和 /tf，再统计 stationary window 下的姿态稳定性
选型规则固定为：
先比成功率
再比恢复次数
再比到达时间
最后比命令平滑度
默认选型规则：
smac_mppi_omni 只要成功率不低于 astar_dwb，且恢复次数更少或持平，就作为默认目标
若 smac_mppi_omni 成功率不达标或横移控制异常，则切到 smac_mppi_diff
若 smac_mppi_diff 仍不稳定，则切到 smac_rpp
结果产物
benchmark/results/<profile>/<case_id>.json
verification/nav_profile_report.json
verification/nav_benchmark_summary.json
验证与验收
统一的 Codex 验证机制
所有新增验证入口都挂到现有 nav_assistant.py 下，统一只做只读检查：

读取配置文件
查询 ROS2 node / topic / param / lifecycle
读取导图资产
回放 rosbag 生成 JSON 报告
不修改 repo tracked files
统一输出固定为 JSON：

verification/mapping_profile_report.json
verification/sensor_report.json
verification/map_artifacts_report.json
verification/localization_report.json
verification/nav_profile_report.json
verification/nav_benchmark_summary.json
阶段验收门槛
建图阶段通过条件：

Cartographer 已固定为 PointCloud2 路径
submaps.grid_options_2d.resolution == 0.03
min_range == 0.15
missing_data_ray_length == max_range
motion_filter 与 translation_delta_cost_weight 达到目标值
导图资产三件套完整且同源
export-map --resolution 仅作为输出检查，不作为建图质量通过条件
relocalization 阶段通过条件：

AMCL 成为唯一运行时定位 owner
fixed pose 和 global localization 两种启动模式都能生成合格 localization_report.json
map->odom 稳定性、/amcl_pose 频率、协方差收敛全部通过
Nav2 阶段通过条件：

所有 profile 都能生成结构一致的 benchmark 结果
smac_mppi_omni 优先参与评估并默认作为目标配置
选型规则可以由脚本自动给出 winner
若 Omni 失败，脚本自动建议切到 smac_mppi_diff 或 smac_rpp
假设与默认值
建图主线仍保留 Cartographer
默认质量主路径是 PointCloud2
默认目标 controller 是 MPPI(Omni)，不是 DiffDrive
DiffDrive 仅作为 Omni 失败后的回退 profile
export-map --resolution 只代表输出栅格分辨率，不代表 SLAM 质量
内部 SLAM 质量判断固定看：
submaps.grid_options_2d.resolution
min_range
missing_data_ray_length
submaps.num_range_data
motion_filter
translation_delta_cost_weight
PointCloud2 输入路径是否生效
Cartographer 原生 IMU 在本计划中默认不启用，直到 frame 设计满足共点要求
所有比较统一基于同一地图、同一 EKF、同一 benchmark case 集
所有验收优先使用脚本和 rosbag 自动判断，不把 RViz 目视结果作为通过标准
