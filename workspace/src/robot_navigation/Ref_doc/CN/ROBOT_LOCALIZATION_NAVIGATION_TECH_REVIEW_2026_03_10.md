# Robot Localization and Navigation Tech Review

> Status update (2026-03-11):
> - Current code now publishes LiDAR in `lidar_link` and IMU in `imu_link`.
> - Cartographer now uses `tracking_frame = "imu_link"`, `published_frame = "base_link"`, `use_imu_data = true`, and `publish_frame_projected_to_2d = true`.
> - The current TF chain is `base_link -> lidar_link -> imu_link`.
> - Current mount preset now uses `base_link -> lidar_link = (0.2413, 0, 0, 0, 0, 0)`.
> - This `0.2413 m` preset comes from the repo's `20 x 20 in` base dimension and the confirmed standard `1.00 in` 80/20 front bar: `0.508 / 2 - 0.0254 / 2 = 0.2413 m`.
> - `lidar_link -> imu_link = (0.0124, 0.0185, -0.0484, 0, 0, 0)` is now set from the SICK operating instructions as the IMU position relative to the optical origin.
> - For the current project preset, `lidar_link` is modeled at the bracket center and treated as the project's optical-origin proxy, following the manual mount assumption for this robot.
> - Mapping stack now enables a `base_link` crop filter before Cartographer by default.
> - Current crop box preset is `x=[-0.2540, 0.1397] m`, `y=[-0.2794, 0.2794] m`, `z=[-1.0, 1.0] m`, i.e. the rear `15.5 x 22 in` self-hit filter box.
> - Localization + Nav2 keeps the older default with crop disabled unless explicitly enabled.
> - This crop filter only cleans Cartographer's point-cloud input path; Nav2 still uses its own robot footprint/box.
> - Any statement below saying "current `tracking_frame = base_link` / `publish_imu_frame_id = lidar_link` / `use_imu_data = false`" should now be read as historical analysis for the pre-2026-03-11 code state.

> 状态更新（2026-03-10）：
> `ROBOT_NAV_STACK_EXECUTION_PLAN_2026_03.md` 的 D0 执行默认已改为 PointCloud2 主路径（`points2 -> /cloud_all_fields_fullframe`），并移除原输入模式切换任务（D0-2）。
> 本文中关于 LaserScan / scan_segment 的内容保留为技术备选与历史分析，不作为当前 D0 默认执行项。

## 0. 文档目的

本文档整理 2026-03-10 对当前 `robot_navigation` 工作区中 localization、mapping、Nav2、PicoScan 传感器接线和参数链路的集中审视结论，目标是把以下问题收敛成一份可追溯记录：

- 当前代码实际在运行什么技术栈
- 为什么旋转时 local costmap 和静态 global map 容易不重合
- Cartographer localization、AMCL、ICP 各自适用边界
- PicoScan100 / `PICS150-01000 Pro-1` 当前是否被充分利用
- Cartographer 建图质量优先时，哪些参数最关键
- segmented / fullframe / LaserScan / PointCloud2 / IMU / odom 的真实通路和约束

---

## 1. 当前代码事实

以下结论来自当前工作区代码核对。

### 1.1 当前运行时定位主链路

- 运行时主定位器是 `Cartographer localization`，不是 `AMCL`
- Nav2 当前全局规划器是 `NavFnPlanner(use_astar=false)`，局部控制器是 `DWBLocalPlanner`
- `robot_localization` EKF 负责：
  `/odom_raw + /sick_scansegment_xd/imu -> /odom`

### 1.2 关键配置事实

`workspace/src/robot_navigation/config/pico_2d.lua`

- `tracking_frame = "base_link"`
- `published_frame = "base_link"`
- `use_odometry = true`
- `num_laser_scans = 0`
- `num_multi_echo_laser_scans = 0`
- `num_subdivisions_per_laser_scan = 1`
- `num_point_clouds = 1`
- `submaps.num_range_data = 35`
- `min_range = 0.3`
- `max_range = 8.0`
- `missing_data_ray_length = 1.0`
- `use_imu_data = false`
- `use_online_correlative_scan_matching = true`
- `real_time_correlative_scan_matcher.linear_search_window = 0.1`
- `real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0`
- `real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.1`

`workspace/src/robot_navigation/config/pico_2d_localization.lua`

- 与 `pico_2d.lua` 的前端定位思路基本一致
- 当前 localization 模式同样是 `use_odometry = true`、`use_imu_data = false`

`workspace/src/robot_navigation/config/ekf_odom_base_imu.yaml`

- `odom0: /odom_raw`
- `imu0: /sick_scansegment_xd/imu`
- `world_frame: odom`
- `publish_tf: false`
- IMU 在当前 EKF 配置里只用了 `yaw` 和 `yaw rate`
- 当前 EKF 没有使用 IMU 的线加速度项做状态融合

### 1.3 驱动与 launch 实际发布内容

`workspace/src/robot_navigation/launch/slam_mapping_stack.launch.py`

- 驱动发布 frame：`publish_frame_id:=lidar_link`
- 驱动发布 IMU frame：`publish_imu_frame_id:=imu_link`
- 启用了 custom pointcloud：`cloud_all_fields_fullframe`
- 启用了 fullframe LaserScan：`publish_laserscan_fullframe_topic:=/scan_fullframe`
- 默认启用 `base_link` 裁剪滤波，输出 `/cloud_all_fields_fullframe_filtered`
- 默认裁剪框：`x=[-0.2540, 0.1397] m`、`y=[-0.2794, 0.2794] m`、`z=[-1.0, 1.0] m`
- 配置了 `base_link -> lidar_link` 静态 TF，默认 `x = 0.2413`

`workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py`

- 运行时 localization 栈与 mapping 栈在 LiDAR / IMU 接线方式上保持一致
- 默认保持 `base_link` 裁剪滤波关闭；只有专项测试时才建议打开
- 同样默认 `base_link -> lidar_link` 平移为 `0.2413 m`

`workspace/src/robot_navigation/launch/cartographer_mapping.launch.py`

- Cartographer 通过 launch 参数 remap：
  - `points2`
  - `imu`
- 在当前完整 mapping stack 里，`points2` 实际接的是 `/cloud_all_fields_fullframe_filtered`
- 当前没有 remap `scan`

`workspace/src/robot_navigation/launch/cartographer_localization.launch.py`

- Localization launch 与 mapping launch 一样，只 remap `points2` 和 `imu`
- 在当前完整 localization stack 里，`points2` 实际接的是 `/cloud_all_fields_fullframe_filtered`
- 当前 Cartographer 实际吃的是 fullframe `PointCloud2`，不是 `LaserScan`

---

## 2. 当前系统的关键判断

### 2.1 现在的问题不是单一参数问题，而是运行时定位架构问题

当前系统更像：

`EKF /odom + Cartographer localization + Nav2`

这套架构对：

- 已知初始位姿附近的连续跟踪
- 特征足够丰富时的 LiDAR 几何贴合

是可以工作的。

但它不天然适合：

- 开机位置变化后的自动冷启动重定位
- 对称货架区、长走廊、低特征环境中的全局位姿搜索

因此：

- local costmap 正常工作
- global map 与 local map 不重合
- 同一位置开机能对上，不同位置开机容易对不上

这些现象和当前技术栈是匹配的。

### 2.2 local costmap 与 global map 旋转时不重合，主因通常是 `map -> odom` 在跳

local costmap 的参考系是 `odom`。  
global costmap / static map 的参考系是 `map`。

因此只要 Cartographer 在旋转时更新或抖动 `map -> odom`，你在 RViz 里就会看到：

- local costmap 仍然围着机器人正常更新
- 但它和静态 global map 看起来不重合

这通常不表示 local costmap 本身坏了，而是定位解在转动时不稳定。

### 2.3 当前看起来“只在 local 附近做 pattern matching”，不是 costmap 把 Cartographer 限住了

Cartographer 根本不读 Nav2 costmap。  
它当前的工作方式是：

- 用 `/odom` 做先验预测
- 在预测位姿附近做局部 scan matching
- 再与附近 submap 建约束

你当前参数把这种“局部微调”倾向放得很重：

- `linear_search_window = 0.1`
- `translation_delta_cost_weight = 10.0`

所以它更像：

- “已知大概在哪，在附近微调”

而不是：

- “拿一帧 scan 在整张图上重新找位姿”

---

## 3. 关于 AMCL、ICP、Cartographer localization

### 3.1 AMCL vs Cartographer localization

对当前项目，运行时定位更推荐：

`EKF + AMCL + map_server + Nav2`

而不是继续长期依赖 `Cartographer localization`。

原因：

- AMCL 更适合冷启动重定位
- AMCL 更适合静态 2D occupancy map
- AMCL 与 Nav2 的工程整合更自然
- 运行时维护成本更低
- 静态地图定位和规划共用同一张 `yaml/pgm` 地图

Cartographer localization 更适合：

- 已知初始位姿附近的持续跟踪
- 建图阶段或 submap-based localization 场景

### 3.2 ICP vs AMCL

ICP 不是完整定位框架，而是局部配准算法。

因此：

- 冷启动重定位：AMCL 通常比纯 ICP 更靠谱
- 已有较好初值时的局部精修：ICP 往往更尖锐、更高精度

对当前项目最稳的路线不是“ICP 替代 AMCL”，而是：

- AMCL 负责全局定位 / 重定位
- 如确有必要，再在 AMCL 收敛后叠一层 ICP 精修

### 3.3 建图与运行时定位应该分栈

推荐目标架构：

- 建图阶段：`Cartographer` 或 `slam_toolbox`
- 运行阶段：`EKF + AMCL + Nav2`

如果继续让 Cartographer 同时承担：

- 建图
- 冷启动定位
- 运行时重定位

工程风险会持续偏高。

---

## 4. 业界常见“边导航边建图”和未知地图探索实现

### 4.1 常见技术栈

对 2D 室内移动机器人，公开 ROS2 生态里最常见的路线是：

- `Nav2 + slam_toolbox` 做在线建图
- 建图完成后切到 `AMCL + Nav2` 做日常定位导航

`Cartographer` 仍可用，但不是当前 ROS2 新项目里最推荐的主线。

### 4.2 实时探索的标准闭环

未知地图自动探索通常不是 Nav2 单独完成，而是：

`SLAM + Frontier Detection + Goal Selection + Nav2 Execution + Recovery`

也就是：

1. SLAM 实时更新 `/map`
2. Frontier 节点在 occupancy grid 上找 frontier
3. 对 frontier 聚类和打分
4. 选中目标点
5. 发给 Nav2 的 `NavigateToPose`
6. 到达后继续下一轮

### 4.3 对当前项目的推荐

如果目标是：

- 自动规划
- 自动探图
- 未知地图实时探索

推荐路线是：

- 建图：`slam_toolbox` 或保留 Cartographer
- 探索：frontier-based exploration
- 导航：`Nav2`
- 运营阶段定位：`AMCL`

---

## 5. PicoScan100 / `PICS150-01000 Pro-1` 的结论

### 5.1 传感器事实

当前确认型号：

- Product family: `picoScan100`
- Type: `PICS150-01000 Pro-1`
- Part number: `1134610`

根据 SICK 官方资料，它是：

- 2D LiDAR
- `276°` 视场
- `15-50 Hz` 扫描频率
- `0.05°-1°` 角分辨率
- `3 echoes`
- 典型量程约 `40 m @ 10% remission`
- 支持 segmented output
- 支持内置 IMU

### 5.2 当前软件没有把它的能力吃满，但瓶颈主要在算法链路

当前没有吃满的关键点不是“LiDAR 本体弱”，而是：

- 当前 Cartographer 吃的是 fullframe `PointCloud2`
- 没有走 `LaserScan` 路径
- 没有利用 subdivision/unwarp 优势
- `max_range = 8.0` 明显保守
- `missing_data_ray_length = 1.0` 明显浪费长距离空闲空间证据
- 前端对 odom 先验约束过强，LiDAR 几何修正空间偏小

因此当前系统更像：

- 把一台高性能 2D LiDAR 当成普通 2D 轮廓输入在用

而不是：

- 把它的高频、长量程、低延迟和 segmented 输出优势真正转化成更高质量的 SLAM

---

## 6. 分段 LiDAR、fullframe、LaserScan、PointCloud2 的真实结论

### 6.1 分段数据能不能直接喂 Cartographer

可以。  
但条件不是“Cartographer 认识 segment”，而是：

- 驱动把 segment 数据发布成 Cartographer 支持的标准消息类型

Cartographer ROS 支持的典型输入包括：

- `sensor_msgs/LaserScan`
- `sensor_msgs/MultiEchoLaserScan`
- `sensor_msgs/PointCloud2`

因此：

- segmented `LaserScan` 可以直接喂
- segmented `PointCloud2` 也可以直接喂

前提是你把话题 remap 给正确的 `scan`、`echoes` 或 `points2`

### 6.2 你当前代码实际没有在用 segmented 路径

当前本地 launch 只显式启用了：

- `/scan_fullframe`
- `/cloud_all_fields_fullframe`

而 Cartographer launch 只 remap 了：

- `points2`
- `imu`

没有 remap `scan`

因此：

- 当前系统没有利用 Cartographer 对 `LaserScan` subdivision 的支持
- 当前 `num_subdivisions_per_laser_scan = 1` 基本没有实际价值，因为 `num_laser_scans = 0`

### 6.3 对当前 LiDAR，更推荐 `LaserScan` 路径而不是 fullframe `PointCloud2`

原因：

- Cartographer 对 `LaserScan` 有原生 subdivision / unwarp 机制
- `PointCloud2` 路径没有这一层针对 2D scan 的 subdivision 处理
- SICK segmented output 的价值本来就是降低延迟和减轻运动畸变

所以如果目标是提高 Cartographer 建图质量，更合理的方向是：

- 让 Cartographer 吃 `LaserScan`
- 优先考虑 segmented `LaserScan`
- 至少也应支持 fullframe `LaserScan + num_subdivisions_per_laser_scan > 1`

---

## 7. 关于 IMU：不是 TF 是假的，而是 Cartographer 对 IMU 有更严格约束

### 7.1 当前 TF 链路是有效的

当前驱动：

- `publish_frame_id := lidar_link`
- `publish_imu_frame_id := lidar_link`

同时你又发布了：

- `base_link -> lidar_link`

这个 TF 在 ROS 层面当然是真实有效的。

### 7.2 但 Cartographer 的 IMU 路径要求 `tracking_frame` 与 IMU frame 共点

Cartographer 官方和源码都要求：

- 如果使用 IMU，`tracking_frame` 应在 IMU 所在位置
- 更严格地说，IMU frame 到 tracking_frame 的平移应近似为 `0`

而你当前配置是：

- `tracking_frame = "base_link"`
- IMU frame = `lidar_link`
- `base_link -> lidar_link` 默认平移 `0.254 m`

所以结论是：

- TF 不是假的
- 但这组 frame 对 Cartographer 原生 IMU 使用不合法

### 7.3 当前 Cartographer 没有直接用 IMU

当前 `use_imu_data = false`，所以 Cartographer 本身没在吃 LiDAR 内置 IMU。  
它当前吃的是：

- EKF 输出的 `/odom`

而 EKF 已经把：

- `/odom_raw`
- `/sick_scansegment_xd/imu`

融合成了 `/odom`

因此当前系统里的 IMU 作用路径是：

`IMU -> EKF -> /odom -> Cartographer`

而不是：

`IMU -> Cartographer`

### 7.4 如果未来要启用 Cartographer 原生 IMU

必须先整理 frame 设计：

- 让 `tracking_frame` 放在 IMU / LiDAR 共点位置
- 或单独定义 colocated 的 tracking frame

否则不建议直接把 `use_imu_data` 改成 `true`

---

## 8. odom 先验、LiDAR 修正能力、7% 恒定误差的真实含义

### 8.1 当前 Cartographer 对 odom 的依赖不是只有“线速度”

Cartographer 前端先做：

- pose extrapolator 预测位姿

再做：

- scan matching

`use_odometry = true` 时，odom 会进入 pose extrapolator，影响：

- 位姿预测
- 线速度估计
- 角速度估计

所以更准确的表述是：

- 当前 Cartographer 会把 `/odom` 当成强先验
- 然后再让 LiDAR 在这个先验附近做几何修正

### 8.2 “信任 odom 先验”是什么意思

就是：

- 搜索窗口只在 odom 预测附近展开
- 偏离 odom 预测值会被代价项惩罚

你当前把这个倾向设得很强：

- `linear_search_window = 0.1`
- `translation_delta_cost_weight = 10.0`

因此现在系统明显更偏：

- “跟随 odom，再让 LiDAR 小幅修正”

而不是：

- “让 LiDAR 主导几何对齐，odom 只是辅助”

### 8.3 7% 恒定误差下，LiDAR 能修到什么程度

如果环境满足：

- 特征丰富
- 静态
- scan 畸变不重
- 每帧增量误差不大

那么 LiDAR 往往能修掉 상당一部分 odom 漂移和尺度误差。

但 LiDAR 不是无限修正器。  
它的修正能力受限于：

- 搜索窗大小
- odom 代价权重
- 环境几何唯一性
- 旋转阶段畸变
- 传感器外参和时间同步

因此在你当前参数下，LiDAR 的修正权限实际上被压小了。

---

## 9. 单 echo 输入是否不合理

不不合理。  
对 2D Cartographer，单 echo 往往比 all echoes 更合理。

原因：

- 2D SLAM 需要的是稳定、单值、连续的边界轮廓
- all echoes 会带来重复边界、穿透回波、反射回波和额外噪声
- Cartographer 2D 不会自动把多 echo 语义化成“更好的表面模型”

因此推荐：

- 常规室内建图：单 echo
- 如果有玻璃 / 透明挡板 / 保护窗：优先考虑 `last echo`
- 不建议把 `all echoes` 作为 2D Cartographer 的默认长期输入

---

## 10. Cartographer 建图质量优先时，当前最关键的参数

### 10.1 优先级最高的不是后端，而是输入形式和前端参数

如果第一目标是提高建图质量，优先级应是：

1. 输入形式
2. 外参与时间同步
3. 前端 scan matching 先验强度
4. submap 和 motion filter
5. 最后才是 pose graph 细调

### 10.2 当前最值得关注的参数

#### `num_laser_scans / num_point_clouds / num_subdivisions_per_laser_scan`

当前值：

- `num_laser_scans = 0`
- `num_point_clouds = 1`
- `num_subdivisions_per_laser_scan = 1`

问题：

- 当前根本没有走 `LaserScan` 路径
- subdivision 机制没有发挥作用

建议方向：

- 改成 `num_laser_scans = 1`
- 改成 `num_point_clouds = 0`
- 把 `num_subdivisions_per_laser_scan` 提到 `4`

#### `min_range`

当前值：`0.3`

问题：

- 对近场角点、门框、货架边界偏保守

建议方向：

- 质量优先时可下调到 `0.15 - 0.2`

#### `max_range`

当前值：`8.0`

问题：

- 对这台 `Pro-1` 型号明显保守

建议方向：

- 建图优先时可上调到 `10 - 12`
- 不建议一开始直接吃满官方最大量程

#### `missing_data_ray_length`

当前值：`1.0`

问题：

- 明显浪费长距离空闲空间证据
- 和 `max_range = 8.0` 完全不匹配

建议方向：

- 设到和 `max_range` 同量级

#### `submaps.num_range_data`

当前值：`35`

问题：

- 偏向快速响应，不偏向局部平滑建图

建议方向：

- 建图优先时提升到 `50 - 70`

#### `real_time_correlative_scan_matcher.linear_search_window`

当前值：`0.1`

问题：

- 搜索范围偏小

建议方向：

- 提到 `0.15 - 0.2`

#### `translation_delta_cost_weight`

当前值：`10.0`

问题：

- 对 odom 先验约束过强
- 明显压缩 LiDAR 几何修正空间

建议方向：

- 回到 `0.1 - 0.3` 量级会更合理

#### `motion_filter`

当前文件里没有显式设置，当前会继承官方默认值。

问题：

- 对高频、高分辨率 2D LiDAR 而言，默认 motion filter 往往更偏节省算力，不偏质量优先

建议方向：

- 显式收紧 motion filter，让更多有效 scan 保留下来

---

## 11. 当前最推荐的目标栈

### 11.1 建图

可接受路线：

- `Cartographer` 继续负责建图
- 或未来迁移到 `slam_toolbox`

### 11.2 运行时定位

推荐路线：

- `robot_localization EKF`
- `AMCL`
- `map_server`
- `Nav2`

### 11.3 Nav2

用户目标明确希望：

- 使用 `MPPI` 代替 `DWB`
- 使用 `A*` 路线规划

推荐实现：

- 局部控制器：`nav2_mppi_controller::MPPIController`
- 全局规划器：优先 `SmacPlanner2D` 或至少把 `NavFnPlanner.use_astar` 改为 `true`

### 11.4 未知地图自动探索

推荐：

- 建图阶段：`SLAM + Frontier Explorer + Nav2`
- 运营阶段：切回 `AMCL + Nav2`

---

## 12. 推荐的阶段性实施顺序

### 阶段 A：先把 Cartographer 建图质量做高

- 把输入从 fullframe `PointCloud2` 改到 `LaserScan`
- 优先评估 segmented `LaserScan`
- 调整 `max_range`、`missing_data_ray_length`、`translation_delta_cost_weight`、`submaps.num_range_data`
- 保持 `use_imu_data = false`，直到 tracking frame 设计整理清楚

### 阶段 B：把运行时定位从 Cartographer localization 切到 AMCL

- 保留 EKF 输出 `/odom`
- 用 `AMCL` 负责静态地图定位
- 加入启动重定位流程

### 阶段 C：把 Nav2 切到目标组合

- 全局规划切到 `A*`
- 局部控制切到 `MPPI`

### 阶段 D：再上 Frontier 自动探索

- frontier 检测
- 目标打分
- 失败恢复
- 与 Nav2 行为树或任务调度层联动

---

## 13. 外部依据

### 13.1 SICK 官方资料

- PicoScan `PICS150-01000 Pro-1` datasheet  
  https://www.sick.com/media/pdf/0/50/850/dataSheet_PICS150-01000-Pro-1_1134610_en.pdf

- PicoScan150 operating instructions  
  https://www.sick.com/media/docs/1/91/691/operating_instructions_picoscan150_2d_lidar_sensors_en_im0106691.pdf

- SICK echo/filter 说明  
  https://support.sick.com/sick-knowledgebase/article/?id=d8f8234a-9f08-f011-bae3-000d3a206a36

- SICK 玻璃 / 透明场景 echo 建议  
  https://support.sick.com/sick-knowledgebase/article/?code=KA-10163

- sick_scan_xd  
  https://github.com/SICKAG/sick_scan_xd

### 13.2 Cartographer 官方资料

- Cartographer ROS configuration  
  https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html

- Cartographer ROS algorithm walkthrough  
  https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html

- Cartographer ROS preparing your bag  
  https://google-cartographer-ros.readthedocs.io/en/latest/your_bag.html

- Cartographer default `trajectory_builder_2d.lua`  
  https://raw.githubusercontent.com/cartographer-project/cartographer/master/configuration_files/trajectory_builder_2d.lua

- Cartographer default `pose_graph.lua`  
  https://raw.githubusercontent.com/cartographer-project/cartographer/master/configuration_files/pose_graph.lua

### 13.3 需要特别记住的源码级结论

- Cartographer 对 `LaserScan` 和 `PointCloud2` 的处理路径不同
- `num_subdivisions_per_laser_scan` 只对 `LaserScan` 路径有意义
- `SensorBridge::ToImuData()` 不读取 `orientation`
- Cartographer 使用 IMU 时要求 IMU frame 与 tracking frame 共点
- 当前代码里 Cartographer 直接吃的是 `PointCloud2`，不是 `LaserScan`

---

## 14. 最终结论

当前系统最核心的问题不是 LiDAR 本体不够强，也不是单独某一个 costmap 参数不对，而是：

- 运行时定位架构不适合冷启动全局重定位
- Cartographer 当前前端配置过度依赖 odom 先验
- PicoScan segmented / LaserScan 优势没有被真正利用
- IMU 目前只通过 EKF 间接进入 Cartographer，尚未按 Cartographer 原生要求使用

如果目标是：

- 更干净的静态地图
- 开机能正确重定位
- 运行时稳定导航
- 未来支持自动探索

那么最合理的整体方向是：

1. 先优化 Cartographer 建图输入和前端参数
2. 再把运行时定位切到 `AMCL`
3. 最后把 Nav2 升级到 `MPPI + A*`

---

## 15. 本次对话的问题上下文与分析逻辑链

### 15.1 用户原始问题的核心诉求

本次讨论的真实问题不是单一一个参数，而是一组相互关联的工程症状：

- Nav2 激活后，机器人更像在用 LiDAR pattern matching 贴静态图，而不是明显依赖 IMU / odom
- 只有 local costmap 看起来正常工作，local / global 不重合
- 开机位置改变后，机器人无法自动把自己放回静态地图中的正确位置
- 转动时更容易掉配准，地图与实时感知不重合
- 希望后续切到 `MPPI` 而不是 `DWB`，全局希望启用 `A*`
- 还希望后期具备自动探索、自动恢复和稳定重定位能力

### 15.2 本次分析的逻辑顺序

这次判断不是直接从直觉下结论，而是按下面的逻辑链收敛的。

#### 步骤 1：先确认当前代码到底在运行什么

结论：

- 当前运行时定位是 `Cartographer localization`，不是 `AMCL`
- 当前 Nav2 仍然是 `NavFn(use_astar=false) + DWB`
- 当前 EKF 在融合 `/odom_raw + /sick_scansegment_xd/imu -> /odom`

本地证据：

- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py:151-152`
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py:164`
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py:183`
- `workspace/src/robot_navigation/config/pico_2d_localization.lua:28`
- `workspace/src/robot_navigation/config/pico_2d_localization.lua:52`
- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml:143`
- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml:283-285`
- `workspace/src/robot_navigation/config/ekf_odom_base_imu.yaml:15`
- `workspace/src/robot_navigation/config/ekf_odom_base_imu.yaml:26`

#### 步骤 2：解释“为什么只有 local costmap 看起来在工作”

结论：

- 这不是 local costmap 自己坏了或 global costmap 自己坏了
- 是因为 `local_costmap` 在 `odom` 坐标系下滚动，而 `global_costmap` / static map 在 `map` 坐标系下
- 一旦 `map -> odom` 不准或在转动时跳动，local 和 global 视觉上就会错开

本地证据：

- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml:184-220`
- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml:227-251`

#### 步骤 3：解释“为什么看起来 Cartographer 只在 local 附近做 matching”

结论：

- 不是 local costmap 限制了 Cartographer
- Cartographer 根本不读 Nav2 costmap
- 当前它是在 odom 预测附近做局部 scan matching，然后和附近 submap 建约束
- `linear_search_window = 0.1` 加上 `translation_delta_cost_weight = 10.0`，会把这种“局部微调”倾向进一步放大

本地证据：

- `workspace/src/robot_navigation/config/pico_2d_localization.lua:54-56`
- `workspace/src/robot_navigation/launch/cartographer_localization.launch.py:79-80`
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py:152`

#### 步骤 4：解释“为什么旋转时更容易掉配准”

结论：

- 当前输入是 `/cloud_all_fields_fullframe`
- 对旋转中的 2D LiDAR，这意味着一帧点云并不是同一时刻采集
- 机器人一转，整帧里前半段和后半段已经不在同一姿态
- 如果又没有用 Cartographer 原生 IMU 稳姿态、外参和时序还有偏差，就很容易在旋转时失锁

本地证据：

- `workspace/src/robot_navigation/launch/cartographer_mapping.launch.py:59`
- `workspace/src/robot_navigation/config/pico_2d.lua:52`
- `workspace/src/robot_navigation/launch/slam_mapping_stack.launch.py:174-179`

#### 步骤 5：解释“为什么换一个开机位置就对不上”

结论：

- 当前启动链路里没有“定位先收敛，再放行导航”的冷启动重定位闭环
- 只是加载 `pbstream` 和 `yaml`，然后固定 `2.0 s` 后直接拉起 Nav2
- 没有 `initialpose`
- 没有 AMCL 全局重定位
- 没有“定位质量达标后再放行”的门控逻辑

本地证据：

- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py:39-40`
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py:151-152`
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py:164`
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py:257`
- `workspace/src/robot_navigation/launch/cartographer_localization.launch.py:31-34`
- `workspace/src/robot_navigation/launch/cartographer_localization.launch.py:75-76`

#### 步骤 6：解释“为什么这是架构问题，不只是参数问题”

结论：

- 地图特征不足会放大问题
- 旋转畸变、时序、外参、odom 质量也会放大问题
- 但根因仍然是当前运行时定位架构不适合冷启动全局重定位

因此最终才会得出：

- 建图阶段可继续用 Cartographer
- 运行时定位更应切到 `EKF + AMCL + map_server + Nav2`

---

## 16. 畸变、数据格式与通路的细化结论

### 16.1 当前工作区里真正走通的 LiDAR 数据通路

当前 mapping / localization 栈里，驱动同时发布了：

- fullframe `LaserScan`：`/scan_fullframe`
- fullframe custom `PointCloud2`：`/cloud_all_fields_fullframe`
- IMU：`/sick_scansegment_xd/imu`

本地证据：

- `workspace/src/robot_navigation/launch/slam_mapping_stack.launch.py:52-54`
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py:71-73`

但是 Cartographer launch 当前只 remap 了：

- `points2`
- `imu`

并没有 remap `scan`

本地证据：

- `workspace/src/robot_navigation/launch/cartographer_mapping.launch.py:59-60`
- `workspace/src/robot_navigation/launch/cartographer_localization.launch.py:79-80`

再结合 Lua：

- `num_laser_scans = 0`
- `num_point_clouds = 1`

所以当前 Cartographer 实际上只吃 fullframe `PointCloud2`。

### 16.2 分段 LiDAR 信息能否直接喂给 Cartographer

可以。  
但准确说法是：

- Cartographer 支持 `LaserScan`、`MultiEchoLaserScan`、`PointCloud2`
- 它不关心“这是分段还是整帧”，它只关心消息类型
- 只要驱动把分段数据发布成这些标准 ROS 消息，就能直接喂

依据：

- `cartographer_ros` 的 `SensorBridge::HandleLaserScanMessage()`
- `SensorBridge::HandleMultiEchoLaserScanMessage()`
- `SensorBridge::HandlePointCloud2Message()`

源码链接：

- `sensor_bridge.cc`  
  https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros/cartographer_ros/sensor_bridge.cc

### 16.3 SICK 驱动是否原生支持 fullframe / segmented 的 LaserScan 和 PointCloud2

支持。  
这是驱动原生能力，不是额外二次开发。

依据：

- `sick_scan_xd` 文档明确给出：
  - `publish_laserscan_segment_topic`
  - `publish_laserscan_fullframe_topic`
- custom pointcloud 明确支持：
  - `updateMethod=0`：fullframe
  - `updateMethod=1`：segmented

官方文档：

- https://docs.ros.org/en/kilted/p/sick_scan_xd/
- https://github.com/SICKAG/sick_scan_xd

### 16.4 为什么当前 fullframe `PointCloud2` 路径更容易在旋转时暴露畸变

当前风险来自 4 个层面。

#### 1. 整帧拼接的时间跨度

SICK 官方文档说明 `picoScan150` 支持 segmented output，设计目的之一就是降低输出延迟。  
官方操作说明中给出的参考量级是：

- `<= 25 Hz` 时，segment 典型为 `30°`，处理延迟 `<= 10 ms`
- `>= 30 Hz` 时，segment 典型为 `60°`，处理延迟 `<= 15 ms`

官方文档：

- https://www.sick.com/media/docs/1/91/691/operating_instructions_picoscan150_2d_lidar_sensors_en_im0106691.pdf

这说明：

- segment 本来就是为了比 fullframe 更低延迟、更少运动畸变

#### 2. Cartographer 对 LaserScan 和 PointCloud2 的处理不同

`SensorBridge::HandleLaserScan()` 会做 subdivision，把一帧 scan 按时间拆分成多个小块后再送入 rangefinder。  
源码里甚至有直接注释：

- `TODO(gaschler): Use per-point time instead of subdivisions.`

而 `HandlePointCloud2Message()` 则直接把 `PointCloud2` 转成点云送入 `HandleRangefinder()`。

这意味着：

- `num_subdivisions_per_laser_scan` 对 `LaserScan` 路径有明确意义
- 对当前 `PointCloud2` 路径没有相同的收益

依据同上：

- `sensor_bridge.cc`

#### 3. SICK 点云里的 `t` / `ts` 字段不是 Cartographer 现成的 guarantee path

`sick_scan_xd` 文档明确写了：

- `t` 是相对 header 的纳秒时间偏移
- 文档中特别提到它“used by rtabmap for deskewing”

官方文档：

- https://docs.ros.org/en/kilted/p/sick_scan_xd/

而 `cartographer_ros` 的 `PointCloud2 -> PointCloudWithIntensities` 文档暴露的是 `PointXYZT` / `time` 路径。  
因此，在没有直接修改 `cartographer_ros` 的前提下，我不把当前 `cloud_all_fields_fullframe` 里的 `t/ts` 当作 Cartographer 已经稳定利用的 deskew 通路。

参考：

- https://docs.ros.org/en/iron/p/cartographer_ros/generated/function_msg__conversion_8h_1a1e15f1de53796da76c49a777d878ae66.html

#### 4. 你当前没让 Cartographer 直接用 IMU 稳住旋转

当前 `use_imu_data = false`：

- `workspace/src/robot_navigation/config/pico_2d.lua:52`
- `workspace/src/robot_navigation/config/pico_2d_localization.lua:52`

因此旋转时前端主要还是依赖 LiDAR 几何和 odom 先验。

### 16.5 畸变处理的优先级建议

对当前这台 2D LiDAR，优先级建议如下：

1. 优先切到 `LaserScan` 输入路径
2. 优先使用 segmented `LaserScan`
3. 如果暂时不切 segmented，也至少切到 fullframe `LaserScan + num_subdivisions_per_laser_scan > 1`
4. 保证 `base_link -> lidar_link` 外参、点云时间戳、`/odom` 时间同步稳定
5. 再考虑是否启用 Cartographer 原生 IMU

一句话：

- 先把畸变源减掉，再去调 matcher

---

## 17. IMU、tracking_frame、odom 先验与 LiDAR 修正权限

### 17.1 为什么“TF 连上了”仍然不等于“Cartographer 可以安全吃 IMU”

你当前驱动把：

- LiDAR frame 设为 `lidar_link`
- IMU frame 也设为 `lidar_link`

同时你又发布了：

- `base_link -> lidar_link`

这在 ROS TF 上完全合法。  
但 Cartographer 对 IMU 的要求更严格。

`SensorBridge::ToImuData()` 里有硬检查：

- IMU frame 到 tracking frame 的平移必须几乎为 `0`

源码原文：

- `CHECK(sensor_to_tracking->translation().norm() < 1e-5) << "The IMU frame must be colocated with the tracking frame.";`

源码链接：

- `sensor_bridge.cc`  
  https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros/cartographer_ros/sensor_bridge.cc

所以：

- TF 不是假的
- 但当前 `tracking_frame = base_link`
- IMU frame = `lidar_link`
- 且 `base_link -> lidar_link` 默认有 `0.254 m` 平移

这组 frame 对 Cartographer 原生 IMU 使用不合法。

### 17.2 当前 EKF 里 IMU 实际被怎么用

`workspace/src/robot_navigation/config/ekf_odom_base_imu.yaml`

当前 EKF 配置里：

- `/odom_raw` 提供 `vx, vy, wz`
- `/sick_scansegment_xd/imu` 提供 `yaw` 和 `wz`
- 没有融合 IMU 线加速度

这意味着：

- 当前 IMU 的主要作用是帮助 EKF 稳住朝向和角速度
- 再由 EKF 输出 `/odom`
- 再由 Cartographer 把 `/odom` 当先验

### 17.3 Cartographer 原生 IMU 实际吃什么

Cartographer 原生 IMU 路径不依赖 `orientation` 字段。  
`SensorBridge::ToImuData()` 只读：

- `linear_acceleration`
- `angular_velocity`

因此：

- “Cartographer 只用 IMU 的 yaw / pitch”这个说法并不准确
- 那是你当前 EKF 的用法，不是 Cartographer 原生 IMU 的用法

### 17.4 “依赖 odom 先验”在前端到底是什么意思

Cartographer 前端不是直接拿一帧 scan 在整张地图上暴力全局搜索。  
更准确的过程是：

1. pose extrapolator 用已有传感器预测当前位姿
2. front-end scan matcher 在该预测值附近搜索
3. 结果再送去局部建图和后端图优化

当 `use_odometry = true` 时，odom 会参与 pose extrapolator。  
官方 `pose_extrapolator.cc` 明确表明：

- odometry 会用于更新线速度和角速度估计

源码：

- https://raw.githubusercontent.com/cartographer-project/cartographer/master/cartographer/mapping/pose_extrapolator.cc

因此“信任 odom 先验”在工程上表示：

- 搜索中心落在 odom 预测附近
- 偏离 odom 预测会有额外代价

### 17.5 你当前把 LiDAR 的修正权限压小了

当前前端参数：

- `linear_search_window = 0.1`
- `translation_delta_cost_weight = 10.0`
- `rotation_delta_cost_weight = 0.1`

而 Cartographer 默认 `translation_delta_cost_weight` 是 `0.1`。  
你现在的平移代价相当于官方默认的 `100x`。

参考：

- https://raw.githubusercontent.com/cartographer-project/cartographer/master/configuration_files/trajectory_builder_2d.lua

这会带来一个直接后果：

- LiDAR 仍然可以修正 odom
- 但前端被配置成更偏“跟 odom，再做小修正”
- 不是偏“让 LiDAR 主导几何对齐”

### 17.6 在你说的“低速 7% 恒定误差”下，LiDAR 能修到什么程度

如果环境满足：

- 几何特征足
- 没有明显动态障碍
- 扫描畸变可控
- 每帧间位姿增量不大

那么 LiDAR 对这种低速恒定比例误差通常能修掉相当一部分。  
但修正能力上限受 5 件事约束：

1. 搜索窗大小
2. odom 代价权重
3. 环境几何唯一性
4. 旋转阶段畸变
5. 外参与时序质量

所以当前并不是“LiDAR 修不动 7%”，而是：

- 当前配置没有给它足够大的修正权限

---

## 18. 如何最大化利用参数提高 Cartographer 建图质量

### 18.1 优先级原则

对当前项目，建图质量优先级建议是：

1. 输入形式与畸变处理
2. 外参与时间同步
3. 前端 matcher 参数
4. submap / motion filter
5. 后端 pose graph 参数

不要先从 `min_score`、`optimize_every_n_nodes` 这种后端参数开始。

### 18.2 参数建议总表

| 项目 | 当前值 | 影响 | 建议值 / 方向 | 依据与逻辑 |
| --- | --- | --- | --- | --- |
| `num_laser_scans` | `0` | 当前没走 LaserScan 路径 | `1` | 让 Cartographer 用上 LaserScan subdivision |
| `num_point_clouds` | `1` | 当前完全依赖 PointCloud2 | `0` | 对 2D LiDAR 建图，更推荐 LaserScan |
| `num_subdivisions_per_laser_scan` | `1` | 没有抗运动畸变增益 | `4` 起步 | Cartographer 官方 subdivision 就是为运动中的 scan unwarp 设计 |
| `submaps.num_range_data` | `35` | submap 偏小、响应快、平滑性一般 | `50-70` | 建图优先时提高局部一致性 |
| `submaps.grid_options_2d.resolution` | 未显式设置 | 内部 submap 默认仍可能是 `0.05` | 显式设 `0.03` | 仅调 occupancy grid 发布分辨率不能提升内部 SLAM 分辨率 |
| `min_range` | `0.3` | 丢近场角点和门框信息 | `0.15-0.2` | 近场结构对 2D 匹配很值钱 |
| `max_range` | `8.0` | 浪费 Pro-1 的有效中远距信息 | `10-12` | 不建议直接吃满官方极限量程，但 `8 m` 明显偏保守 |
| `missing_data_ray_length` | `1.0` | 空闲空间证据被截断 | 与 `max_range` 同量级 | `max_range=10` 就建议 `missing_data_ray_length=10` |
| `linear_search_window` | `0.1` | 匹配搜索过窄 | `0.15-0.2` | 给 LiDAR 更大的局部修正空间 |
| `translation_delta_cost_weight` | `10.0` | 过度粘附 odom 先验 | `0.1-0.3` | 让 LiDAR 有真实几何纠偏能力 |
| `rotation_delta_cost_weight` | `0.1` | 当前并不极端 | 可先保留 `0.1` | 暂不是第一优先级 |
| `motion_filter.max_time_seconds` | 默认 | 可能丢太多高频 scan | `0.5` | 质量优先时保留更多有效观测 |
| `motion_filter.max_distance_meters` | 默认 | 直线段过度稀疏化 | `0.05` | 让高性能 LiDAR 的观测密度真正进入地图 |
| `motion_filter.max_angle_radians` | 默认 | 旋转时保留点不足 | `math.rad(0.5)` | 增强转弯区域图像稳定性 |
| `use_imu_data` | `false` | 当前不直接利用原生 IMU | 保持 `false`，直到 frame 合法后再试 `true` | 先修 frame 设计，再开 IMU |
| echo 策略 | all echoes | 容易引入多回波轮廓污染 | 单 echo 为基线；玻璃场景试 `last echo` | SICK 官方对玻璃 / 透明场景推荐 last echo |

### 18.3 我更推荐的一版“建图质量优先”参数方向

如果仅从当前仓库出发，不引入别的 SLAM，只优化 Cartographer，我更认可这组方向：

```lua
num_laser_scans = 1
num_multi_echo_laser_scans = 0
num_subdivisions_per_laser_scan = 4
num_point_clouds = 0

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
```

### 18.4 关于 `Dynamic Sensing Profile` 的保守建议

根据 `PICS150-01000 Pro-1` 官方资料，它支持：

- `15-50 Hz`
- `0.05°-1°`

如果你当前目标是建一张更干净的 2D 静态图，优先考虑：

- `30 Hz / 0.1°`

原因：

- 比 `20 Hz` 更短的整帧时间，更抗运动畸变
- 比 `1°` 明显更细的角分辨率
- 比 `0.05°` 的超高密度更容易保持总体算力和通路稳定

这是偏工程稳妥的基线，而不是理论极限值。

### 18.5 单 echo 是否会造成数据不合理

不会。  
对 2D Cartographer，单 echo 通常反而更合理。

理由：

- 2D SLAM 需要的是稳定、单值、连续的边界
- all echoes 更容易产生重复边界和不一致几何
- SICK 官方明确支持 `first / all / last echo`
- 官方对玻璃 / 透明防护件场景建议 `last echo`

因此推荐：

- 常规室内：单 echo
- 有玻璃：优先试 `last echo`

参考：

- https://support.sick.com/sick-knowledgebase/article/?id=d8f8234a-9f08-f011-bae3-000d3a206a36
- https://support.sick.com/sick-knowledgebase/article/?code=KA-10163

---

## 19. Relocalization 技术建议与比较

### 19.1 Cartographer localization、AMCL、ICP 的定位分工

| 方案 | 优势 | 短板 | 适合当前项目吗 |
| --- | --- | --- | --- |
| Cartographer localization | 连续跟踪时局部配准锐利；能复用 pbstream submap | 对初值、时序、外参更敏感；不擅长冷启动全局重定位；需同时维护 pbstream 与 yaml | 不适合作为长期生产运行时主定位 |
| AMCL | 粒子滤波，多假设搜索，适合静态 2D 地图和冷启动重定位；与 Nav2 工作流天然匹配 | 局部连续跟踪不一定像 Cartographer 那么“尖” | 最适合当前运行时定位 |
| ICP / NDT / GICP | 已有较好初值时精修精度高 | 强依赖初值；对旋转畸变、时序、外参敏感；不自带全局多假设管理 | 更适合做 AMCL 之后的可选精修，而不是替代 AMCL |

### 19.2 为什么当前项目更推荐 AMCL

因为你的主要痛点是：

- 开机位置可能不同
- 需要自动找到自己在静态地图中哪里
- 运行阶段希望稳定、低维护

这正是 AMCL 的典型强项。

Nav2 官方也明确给出了 `SLAM / Localization` 工作流：

- 建图阶段使用 SLAM
- 运行阶段使用 localization

参考：

- https://docs.nav2.org/setup_guides/sensors/mapping_localization.html
- https://docs.nav2.org/configuration/packages/configuring-amcl.html

### 19.3 推荐的运行时重定位架构

推荐目标栈：

```text
/odom_raw + /imu  -->  robot_localization EKF  -->  /odom
/scan_fullframe   -->  AMCL                     -->  map -> odom
static yaml map   -->  map_server
Nav2              -->  planner + controller + BT recovery
```

### 19.4 推荐的启动重定位流程

#### 场景 A：起点固定，例如充电桩 / 停靠位

流程：

1. 拉起 `map_server`
2. 拉起 `AMCL`
3. 自动发布 dock `initialpose`
4. 等待粒子云收敛
5. 再激活 planner / controller / bt_navigator

#### 场景 B：起点不固定

流程：

1. 拉起 `map_server`
2. 拉起 `AMCL`
3. 调用 `ReinitializeGlobalLocalization`
4. 执行低速原地扫描或短距离扫描动作
5. 监控粒子分布与 pose covariance
6. 定位收敛后再开放导航

Nav2 BT 官方节点：

- `ReinitializeGlobalLocalization`  
  https://docs.nav2.org/configuration/packages/bt-plugins/actions/ReinitializeGlobalLocalization.html

### 19.5 低特征 / 重复货架环境下的补充建议

如果环境存在：

- 长走廊
- 对称货架
- 重复门洞
- 大面积几何重复

那么单靠 LiDAR + 静态图，本来就可能没有唯一解。  
这不是把 `min_score`、`inflation_radius` 再调一轮就能彻底解决的。

更实际的增强手段有 3 类：

1. 固定已知起始位姿
2. 增加人工先验：AprilTag、反光柱、人工地标
3. 允许人工介入初始位姿，但运行中保持自动定位

### 19.6 地图资产一致性要求

当前代码同时独立加载：

- `pbstream`
- `yaml`

本地证据：

- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py:39-40`
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py:225-226`

这意味着必须强制要求：

- 每张运行时 `yaml/pgm` 必须从同源 `pbstream` 导出

否则你看到的任何 local/global 不重合，都不一定是定位器本身错了。

---

## 20. Nav2 不同规划器与控制器的技术比较和建议

### 20.1 当前仓库中的 Nav2 事实

当前参数文件：

- 控制器：`dwb_core::DWBLocalPlanner`
- 全局规划器：`nav2_navfn_planner::NavfnPlanner`
- `use_astar = false`

本地证据：

- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml:123`
- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml:143`
- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml:281`
- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml:283-285`

### 20.2 全局规划器比较

| 方案 | 优势 | 短板 | 适合当前项目吗 |
| --- | --- | --- | --- |
| `NavFn` Dijkstra | 稳、老牌、参数少 | 路径质量一般，效率不如新 A* 系列 | 可作为基线，不是长期最优 |
| `NavFn` A* | 改动最小，低风险 | 仍然是传统网格搜索，路径质量和障碍代价处理一般 | 适合低风险过渡 |
| `SmacPlanner2D` | 官方推荐的 2D A* 系列，代价感知更强，路径质量更好，性能也很好 | 参数比 NavFn 略多 | 最推荐 |
| `Theta*` | any-angle，路径可更直 | 更依赖 costmap / inflation / traversal cost 调得合适 | 可作为备选，不是第一优先级 |
| `SmacHybrid` / `SmacLattice` | 适合有运动学约束或非圆形 footprint 机器人 | 复杂度更高，当前底盘未必需要 | 暂不优先 |

依据：

- NavFn Planner  
  https://docs.nav2.org/configuration/packages/configuring-navfn.html
- Smac Planner  
  https://docs.nav2.org/configuration/packages/configuring-smac-planner.html
- Theta Star Planner  
  https://docs.nav2.org/configuration/packages/configuring-thetastar.html
- Nav2 algorithm selection  
  https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html

### 20.3 局部控制器比较

| 方案 | 优势 | 短板 | 适合当前项目吗 |
| --- | --- | --- | --- |
| `DWB` | 默认、成熟、资料多 | critic 调参多，对高速 / 复杂局部几何不如预测型方法灵活 | 当前可用，但不是用户目标 |
| `MPPI` | 预测型采样控制器，支持差速 / 全向 / Ackermann，局部轨迹选择能力强 | 对模型参数、代价权重和算力要求更高 | 用户目标方案，推荐 |
| `RPP` | 路径跟踪简单、稳、调参相对容易 | 更偏 tracking，不如 MPPI 灵活 | 可作回退控制器（暂时先不使用，此处我不需要回退计划） |

依据：

- MPPI  
  https://docs.nav2.org/configuration/packages/configuring-mppic.html
- DWB  
  https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html
- RPP  
  https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html

### 20.4 对当前项目的推荐组合

#### 最终推荐

- 全局规划：`SmacPlanner2D`
- 局部控制：`MPPIController`

#### 低风险过渡方案

- 先把 `NavFnPlanner.use_astar` 从 `false` 改成 `true`
- 再单独把 `DWB` 切成 `MPPI`

#### 运动学模型建议

如果你的底盘横移控制真的可靠：

- MPPI 使用 `Omni`

如果 `vy` 跟踪不稳定，或者底盘虽然是全向但控制层实际更接近差速：

- 先使用 `DiffDrive`

不要让控制器模型与真实底盘运动学不一致。

### 20.5 Nav2 行为树层面的建议

Nav2 官方默认恢复树已经很成熟，支持：

- planner / controller 的 contextual recoveries
- global recovery subtree
- clear costmap
- spin
- wait
- backup
- consistent replanning

参考：

- https://docs.nav2.org/behavior_trees/trees/nav_to_pose_recovery.html
- https://docs.nav2.org/behavior_trees/trees/nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.html
- https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough
- https://docs.nav2.org/configuration/packages/configuring-bt-xml.html

---

## 21. 后期自动恢复行为逻辑建议

### 21.1 为什么这里不能只靠默认 `BackUp`

默认 Nav2 恢复树已经很好，但你这个项目后期要面向：

- 自动探索
- 长时间运行
- 低特征区定位
- 可能有动态障碍

因此恢复逻辑不应只停留在：

- 清图
- 转圈
- 后退一点

而应该做成“分层恢复”。

### 21.2 SCURM 仓库里值得借鉴的点

`SCURM_SentryNavigation` 的 README 明确提到两点：

- 改进 navigation2 的故障恢复行为
- 提供了一个 “enhenced back_up action that move toward free space”

参考：

- https://github.com/PolarisXQ/SCURM_SentryNavigation/tree/master
- https://raw.githubusercontent.com/PolarisXQ/SCURM_SentryNavigation/master/README.md

这点很值得借鉴，因为对你的机器人来说：

- “向后退固定距离”不一定安全
- “朝无碰撞的自由空间方向退避”明显更工程化

### 21.3 建议的恢复层级

#### 第 0 层：上下文恢复

触发条件：

- `ComputePathToPose` 失败
- `FollowPath` 失败
- local progress checker 失败
- path invalid / new obstacle blocking

动作：

1. `ClearCostmapAroundRobot`
2. `ClearEntireCostmap` 或局部局清
3. 重新规划
4. 继续跟踪

#### 第 1 层：局部姿态恢复

触发条件：

- 控制器连续多次失败
- 障碍局部拥堵
- 需要新的 LiDAR 观测视角

动作：

1. `Spin`
2. `Wait`
3. 小幅 `BackUp`
4. 再次 `ComputePathToPose`

#### 第 2 层：自由空间退避恢复

这是建议你后续自定义的重点。

目标：

- 不要固定朝后退
- 而是根据 local costmap 或局部可通行扇区，选择“更自由”的退避方向

逻辑：

1. 读取 local costmap / local obstacle window
2. 评估机器人后方、左后、右后、侧向等扇区碰撞风险
3. 选择最近的自由空间方向
4. 执行短距离退避
5. 清理局部 costmap
6. 重规划

这类逻辑可以参考 SCURM 的 `enhenced back_up action` 思路。

#### 第 3 层：定位恢复

触发条件：

- `map -> odom` 明显跳变
- AMCL 粒子散度过大
- 启动阶段定位长时间不收敛
- 运行中出现 kidnapped-robot 症状

动作：

1. 暂停 planner / controller 目标执行
2. 调用 `ReinitializeGlobalLocalization`
3. 执行低速原地扫描
4. 等待 AMCL 收敛
5. 收敛后恢复任务

#### 第 4 层：任务级失败与人工接管

触发条件：

- 多轮恢复后仍失败
- 定位质量不达标
- 动态障碍长时间占用

动作：

1. 停车
2. 上报任务失败原因
3. 请求人工接管或重派任务

### 21.4 推荐的行为树骨架

可参考如下逻辑：

```xml
<RecoveryNode number_of_retries="6" name="NavigateRecovery">
  <PipelineSequence name="NavigateWithReplanning">
    <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased"/>
    <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath"/>
    <ComputePathToPose .../>
    <FollowPath .../>
  </PipelineSequence>

  <ReactiveFallback name="GlobalRecovery">
    <GoalUpdated/>
    <Sequence>
      <ClearEntireCostmap name="ClearLocal"/>
      <ClearEntireCostmap name="ClearGlobal"/>
      <Spin/>
      <Wait wait_duration="2.0"/>
      <BackUp/>
      <!-- custom free-space backup -->
      <!-- localization recovery subtree -->
      <ReinitializeGlobalLocalization service_name="reinitialize_global_localization"/>
    </Sequence>
  </ReactiveFallback>
</RecoveryNode>
```

这里真正建议你后续自定义的是两块：

1. free-space aware backup
2. localization recovery subtree

### 21.5 自动探索阶段的恢复建议

对 frontier exploration，还应补这些策略：

- 不可达 frontier 黑名单
- 黑名单超时后可重试
- frontier 信息增益太低时直接跳过
- frontier 附近若遇动态障碍，先 wait 再 replan
- 若局部恢复失败，切换到下一个 frontier
- 若连续多个 frontier 都失败，结束探索并上报

---

## 22. 参考资料与证据索引补充

### 22.1 本地代码证据索引

- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py`
  - `39-40`: 默认 `pbstream` / `yaml`
  - `151-152`: Cartographer localization 加载 pbstream 且关闭 occupancy grid
  - `164`: `map_server` 读取 `yaml`
  - `183`: 启动 `navigation_launch.py`
  - `225-226`: `pbstream_file` / `map_yaml` launch 参数
  - `257`: 固定 `2.0 s` 后启动 Nav2

- `workspace/src/robot_navigation/launch/cartographer_localization.launch.py`
  - `31-34`: `load_state_filename` / `load_frozen_state`
  - `59`: `publish_occupancy_grid`
  - `75-80`: Cartographer node remap `points2` 和 `imu`

- `workspace/src/robot_navigation/launch/cartographer_mapping.launch.py`
  - `59-60`: mapping 模式也只 remap `points2` 和 `imu`

- `workspace/src/robot_navigation/launch/slam_mapping_stack.launch.py`
  - `37-38`: `publish_frame_id` / `publish_imu_frame_id` 均为 `lidar_link`
  - `52-54`: 开启 `cloud_all_fields_fullframe` 和 `/scan_fullframe`
  - `174-179`: 默认 `base_link -> lidar_link` 外参

- `workspace/src/robot_navigation/config/pico_2d.lua`
  - `22-34`: tracking / frame / LaserScan / PointCloud 模式
  - `48-56`: 关键前端参数

- `workspace/src/robot_navigation/config/pico_2d_localization.lua`
  - `22-34`: localization 模式输入形式
  - `48-56`: localization 前端参数

- `workspace/src/robot_navigation/config/ekf_odom_base_imu.yaml`
  - `15-20`: `/odom_raw`
  - `26-32`: `/imu`

- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml`
  - `123`, `143`: `DWB`
  - `184-220`: local costmap
  - `227-251`: global costmap
  - `281-285`: `NavFn` + `use_astar=false`

### 22.2 SICK / PicoScan / 驱动文档

- PicoScan `PICS150-01000 Pro-1` datasheet  
  https://www.sick.com/media/pdf/0/50/850/dataSheet_PICS150-01000-Pro-1_1134610_en.pdf

- PicoScan150 operating instructions  
  https://www.sick.com/media/docs/1/91/691/operating_instructions_picoscan150_2d_lidar_sensors_en_im0106691.pdf

- `sick_scan_xd` 文档  
  https://docs.ros.org/en/kilted/p/sick_scan_xd/

- `sick_scan_xd` GitHub  
  https://github.com/SICKAG/sick_scan_xd

- echo / filter 说明  
  https://support.sick.com/sick-knowledgebase/article/?id=d8f8234a-9f08-f011-bae3-000d3a206a36

- 玻璃 / 透明场景建议  
  https://support.sick.com/sick-knowledgebase/article/?code=KA-10163

### 22.3 Cartographer 官方文档与源码

- Cartographer ROS configuration  
  https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html

- Cartographer ROS algorithm walkthrough  
  https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html

- Cartographer ROS `your_bag`  
  https://google-cartographer-ros.readthedocs.io/en/latest/your_bag.html

- Cartographer default `trajectory_builder_2d.lua`  
  https://raw.githubusercontent.com/cartographer-project/cartographer/master/configuration_files/trajectory_builder_2d.lua

- Cartographer default `pose_graph.lua`  
  https://raw.githubusercontent.com/cartographer-project/cartographer/master/configuration_files/pose_graph.lua

- `sensor_bridge.cc`  
  https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros/cartographer_ros/sensor_bridge.cc

- `pose_extrapolator.cc`  
  https://raw.githubusercontent.com/cartographer-project/cartographer/master/cartographer/mapping/pose_extrapolator.cc

- `cartographer_ros::ToPointCloudWithIntensities(const PointCloud2&)` 文档  
  https://docs.ros.org/en/iron/p/cartographer_ros/generated/function_msg__conversion_8h_1a1e15f1de53796da76c49a777d878ae66.html

### 22.4 Nav2 官方文档

- Mapping / Localization setup guide  
  https://docs.nav2.org/setup_guides/sensors/mapping_localization.html

- AMCL  
  https://docs.nav2.org/configuration/packages/configuring-amcl.html

- NavFn Planner  
  https://docs.nav2.org/configuration/packages/configuring-navfn.html

- Smac Planner  
  https://docs.nav2.org/configuration/packages/configuring-smac-planner.html

- Theta Star Planner  
  https://docs.nav2.org/configuration/packages/configuring-thetastar.html

- Algorithm selection guide  
  https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html

- MPPI Controller  
  https://docs.nav2.org/configuration/packages/configuring-mppic.html

- DWB Controller  
  https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html

- Regulated Pure Pursuit  
  https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html

- Behavior Server  
  https://docs.nav2.org/configuration/packages/configuring-behavior-server.html

- BT Navigator  
  https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html

- Behavior Tree XML Nodes  
  https://docs.nav2.org/configuration/packages/configuring-bt-xml.html

- Detailed Behavior Tree Walkthrough  
  https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough

- Navigate To Pose with recovery  
  https://docs.nav2.org/behavior_trees/trees/nav_to_pose_recovery.html

- Navigate To Pose with consistent replanning and invalid-path recovery  
  https://docs.nav2.org/behavior_trees/trees/nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.html

- ReinitializeGlobalLocalization  
  https://docs.nav2.org/configuration/packages/bt-plugins/actions/ReinitializeGlobalLocalization.html

### 22.5 业界常见建图 / 定位路线参考

- `slam_toolbox`  
  https://docs.ros.org/en/ros2_packages/jazzy/api/slam_toolbox/

- `RTAB-Map`  
  https://introlab.github.io/rtabmap/

- `rtabmap_ros`  
  https://github.com/introlab/rtabmap_ros

- `LIO-SAM`  
  https://github.com/TixiaoShan/LIO-SAM

### 22.6 自动恢复行为参考

- `SCURM_SentryNavigation`  
  https://github.com/PolarisXQ/SCURM_SentryNavigation/tree/master

- `SCURM_SentryNavigation` README  
  https://raw.githubusercontent.com/PolarisXQ/SCURM_SentryNavigation/master/README.md
