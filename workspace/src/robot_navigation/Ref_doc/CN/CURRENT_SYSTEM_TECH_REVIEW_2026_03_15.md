# 当前系统技术审查

日期：2026-03-15

范围：
- 本文描述的是 2026-03-15 这次代码快照下仓库的当前实现状态。
- 本文按“代码真实行为”写，不按“原始设计意图”写。
- 覆盖导航栈、semantic map、任务执行链路，以及当前前端/后端围绕地图与 SLAM 页的连接关系。

本文可以看作对 `Ref_doc/CN` 与 `Ref_doc/EN` 里旧导航 review 的当前态补充。旧文档里很多分析仍有参考价值，但本文优先反映现有代码到底在做什么。

## 0. 执行摘要

当前系统已经不只是一个 ROS 导航工程，而是一套多层系统：

- 以 SICK PicoScan、Cartographer、EKF、Nav2 为核心的 ROS 2 导航与定位栈。
- 一层独立的 semantic overlay，既在 ROS 里存在，也在员工网页 UI 里存在。
- 一个 PostgreSQL + Node.js 后端，负责库存、员工认证、订单接口、semantic map 接口和 semantic map 版本历史。
- 一个基于 Vue 的员工端 dashboard，其中：
  - `Store Map` 是自定义可编辑 semantic map 页面。
  - `SLAM Map` 是把 vendored 的 ROS Web GUI 工具嵌进 iframe 的外层壳。

当前最关键的系统事实如下：

- mapping 和 localization 都是基于 PointCloud2 的 Cartographer 管线。
- localization 用的是 Cartographer localization + static map server，不是 AMCL。
- Nav2 当前主栈是 `SmacPlanner2D + MPPIController`，并且启用了 omni motion model。
- semantic map 当前在线主源仍然是 YAML。
- PostgreSQL 里的 semantic version 表目前是版本快照，不是主读源。
- `Store Map` 页面可以编辑 semantic map，并同时保存到 YAML 和数据库版本表。
- `SLAM Map` 页面不是自绘 canvas，而是一个外层壳，里面嵌的是 vendored `ros_web_gui_app`，由 iframe 内部直接连 `rosbridge`。

当前已经能工作，但“设计目标”和“当前真实行为”之间还有几个明显断点：

- 默认 BT executor 只轮询 customer order，不轮询 employee restock。
- customer 树当前只做导航，不改库存。
- restock 树当前反而调用了“decrement inventory”接口，语义上是反的。
- robot status 页面仍然是后端占位数据，不是 ROS 实时状态。
- 嵌入式 SLAM 页面必须额外跑 `rosbridge_server`，不会经过 Node 后端转发。

## 1. 代码边界与当前相关包

### 1.1 ROS 包

当前 `workspace/src` 里与本 review 直接相关的包：

- `robot_navigation`
  - launch 组合
  - Cartographer 配置
  - Nav2 参数
  - serial bridge
  - base-link crop filter
  - semantic map server
  - `nav_assistant` CLI
- `robot_task_manager`
  - blackboard
  - BT executor
  - BT nodes
  - customer / restock 树
  - 另一套 ViperX BT 路径
- `robot_interfaces`
  - `Order`、`OrderItem`
  - `NewOrder.srv`
  - `ResolveSemanticTarget.srv`
  - `MoveRack.action`
  - `PickArm.action`
- `robot_manipulation`
  - VX300 MoveIt launch
  - `pick_arm` action server

### 1.2 Web / backend 部分

当前 Web / backend 相关目录：

- `order-api-postgre`
  - Express 后端
  - PostgreSQL 接入
  - semantic map 文件解析与保存逻辑
- `order-api-postgre/fleet-manager`
  - 员工端 Vue dashboard
  - Store Map 页面
  - SLAM 页面外层壳
- `third_party/ros_web_gui_app`
  - 从 `StarLionJiang/ros_web_gui_app` vendored 下来的源码
  - 已做英文 UI 文案适配，用于嵌入式 SLAM 页面

### 1.3 地图资产与 semantic 资产

当前地图资产：

- `Maps/testmapMain.pbstream`
- `Maps/testmapMain.yaml`
- `Maps/testmapMain.pgm`

当前 semantic overlay 资产：

- `workspace/src/robot_navigation/config/semantic_map_testmapMain.yaml`

当前重要地图事实：

- 地图名：`testmapMain`
- 地图分辨率：`0.03 m/pixel`
- 地图在 `map` 坐标系里的 origin：`[-2.797711, -10.398200, 0.0]`
- semantic map id：`testmapMain`
- semantic home anchor 当前配置为 `(0.0, 0.0, 0.0)`

## 2. 当前导航栈

### 2.1 Mapping 栈

当前 mapping 启动入口：

- `ros2 launch robot_navigation slam_mapping_stack.launch.py`
- 或 `ros2 run robot_navigation nav_assistant mapping-stack`

当前 mapping 栈组成：

- SICK PicoScan 驱动 `sick_generic_caller`
- 静态 TF：`base_link -> lidar_link`
- 静态 TF：`lidar_link -> imu_link`
- `base_link_crop_filter`
- Cartographer mapping
- serial bridge
- 可选 EKF
- 可选 ultrasonic collision launch
- 可选 RViz

当前 mapping 传感器链路：

- LiDAR 原始点云：`/cloud_all_fields_fullframe`
- 给 Cartographer 的裁剪后点云：`/cloud_all_fields_fullframe_filtered`
- IMU：`/sick_scansegment_xd/imu`

当前 mapping 行为：

- Cartographer mapping 使用的是裁剪后的点云。
- Cartographer 通过 `cartographer_occupancy_grid_node` 发布 occupancy grid。
- crop filter 在 mapping 里默认开启。
- crop filter 的主要作用是把机器人车体自击中的点从 Cartographer 输入里去掉。

当前 Cartographer mapping 配置事实：

- `tracking_frame = "imu_link"`
- `published_frame = "base_link"`
- `odom_frame = "odom"`
- `use_odometry = true`
- `use_imu_data = true`
- `num_point_clouds = 1`
- `num_laser_scans = 0`
- `publish_frame_projected_to_2d = true`

这说明当前 mapping 不是 LaserScan 主链路，而是 PointCloud2 + IMU + odom 主链路。

### 2.2 Localization + Nav2 栈

当前 localization 启动入口：

- `ros2 launch robot_navigation nav2_localization_stack.launch.py`
- 或 `ros2 run robot_navigation nav_assistant localization-stack --map-name testmapMain`

当前 localization 栈组成：

- SICK PicoScan 驱动
- 静态 TF
- serial bridge
- 可选 EKF
- 可选 crop filter，默认关闭
- Cartographer localization，加载 `pbstream`
- `nav2_map_server` 提供 `testmapMain.yaml`
- map server lifecycle manager
- 2 秒延迟后启动 Nav2 bringup
- 可选 Nav2 RViz
- semantic map server，默认开启

当前 localization 的重要行为：

- Cartographer localization 加载保存下来的 `pbstream` 状态。
- localization 模式下 `publish_occupancy_grid=false`。
- localization 模式里的 `/map` 来自 `nav2_map_server`，不是 Cartographer。
- Cartographer 在这里主要负责 localization / TF；static occupancy grid 由导出的 yaml / pgm 提供。

这点是当前结构里的关键区别：

- mapping 模式：Cartographer 生成地图
- localization 模式：Cartographer 负责定位，`map_server` 负责静态地图

### 2.3 EKF 路径

当前 EKF 配置：

- 输入 odom：`/odom_raw`
- 输入 IMU：`/sick_scansegment_xd/imu`
- 输出：`/odom`
- `publish_tf: false`
- `world_frame: odom`
- `two_d_mode: true`

当前实际链路：

`STM32 telemetry -> nav2_serial_bridge -> /odom_raw`

`/odom_raw + /sick_scansegment_xd/imu -> robot_localization EKF -> /odom`

### 2.4 Nav2 当前技术栈

当前 Nav2 参数文件：

- `workspace/src/robot_navigation/config/nav2_params_smac_mppi_omni.yaml`

当前 active planner / controller：

- planner：`nav2_smac_planner/SmacPlanner2D`
- controller：`nav2_mppi_controller::MPPIController`
- motion model：`Omni`

当前 costmap 行为：

- local costmap：
  - 在 `odom` 坐标系滚动窗口
  - `VoxelLayer + InflationLayer`
  - 点云源：`/cloud_all_fields_fullframe`
- global costmap：
  - 在 `map` 坐标系
  - `StaticLayer + ObstacleLayer + InflationLayer`
  - 障碍物源：`/cloud_all_fields_fullframe`

这里有一个当前实现层面的重要事实：

- Cartographer 在 mapping / localization 里吃的是裁剪后的 cloud。
- Nav2 costmap 目前仍然用的是原始 cloud topic，不是裁剪后的 cloud。
- 也就是说 crop filter 现在主要保护 Cartographer 输入，不直接保护 Nav2 obstacle layer。

### 2.5 Serial bridge 与控制链路

当前 serial bridge：

- 节点：`robot_navigation/nav2_serial_bridge.py`
- 作用：
  - 把 `/cmd_vel` 类命令经串口发给 STM32
  - 把 telemetry 解码成 odometry
  - telemetry 断流时可选 fallback odom

当前默认桥接命令 topic：

- mapping 栈：`["/cmd_vel", "/cmd_vel_nav", "/cmd_vel_smoothed"]`
- localization 栈：`["/cmd_vel"]`

当前 command authority 事实：

- 还没有真正实现 `cmd_vel` 仲裁器
- 多个命令源之间仍然是简单订阅
- 旧规划文档里提到的 arbiter 仍然没有进入当前主代码路径

## 3. 当前 Semantic Map 栈

### 3.1 Semantic map 表示方式

当前 semantic map 是一份手工维护的 YAML overlay，不是从 RViz 或 SLAM 自动提取出来的。

当前文件：

- `workspace/src/robot_navigation/config/semantic_map_testmapMain.yaml`

当前结构：

- `map`
- `anchors`
- `racks`
- `slots`

当前 semantic 内容：

- anchors：
  - `home`
  - `drinks_anchor`
  - `fruits_anchor`
  - `snacks_anchor`
- racks：
  - 每个 anchor 区域对应一组 rack
- slots：
  - 当前有 7 个 slot
  - 绑定到 seeded inventory 里的 product id 1 到 7

当前必须明确的事实：

- 这份 semantic map 是人工标注的
- 不是从 RViz 自动生成
- 当前也不是从 PostgreSQL 直接读出来当主源
- 现在它是围绕 seeded store example 写的一份示例化语义图

### 3.2 ROS semantic map server

当前 semantic ROS 节点：

- `robot_navigation/semantic_map_server.py`

当前职责：

- 读取 semantic YAML
- 发布 marker overlay
- 提供 semantic resolve service

当前接口：

- markers：`/semantic_map/markers`
- service：`/semantic_map/resolve_target`

当前解析顺序：

1. 显式 `slot_id`
2. 显式 `anchor_id`
3. `product_id`
4. `product_name`

### 3.3 Semantic map 在 Web 里的实现方式

当前 semantic map 已经能在员工 UI 里显示和编辑，但 Web 路径和 ROS 路径不是一回事。

当前 backend semantic 行为：

- 在线读取 YAML
- 用 PostgreSQL 的 `inventory` 信息 enrich slot 产品信息
- 通过 REST 暴露 bundle
- 保存编辑结果回 YAML
- 每次保存同时写数据库版本快照

当前最关键的实现事实：

- YAML 仍然是在线主源
- `semantic_map_versions` 目前只是版本历史
- 直接改数据库不会改变当前前端读到的 live semantic map，除非 YAML 也一起改

## 4. 当前任务执行与 BT 链路

### 4.1 默认 BT executor

当前默认 executor：

- `workspace/src/robot_task_manager/robot_task_manager/bt_executor.py`

当前行为：

- 提供 `/order/new` service
- 每秒轮询一次 Node backend
- 只轮询 `GET /api/order/latest`
- 根据 `order.role` 构造 customer 或 employee 树
- 以 10 Hz tick BT

当前 order intake 来源：

- 自动轮询只支持 `order_id` 里的 customer order
- 员工端提交到 `restock_id` 的 restock request 不会被默认 executor 拉取

这是一条非常重要的当前限制：员工 UI 可以提交 restock，但默认 executor 不会自动消费它。

### 4.2 Blackboard 状态

当前 blackboard 已包含：

- 订单上下文
- 当前 item 上下文
- arm 占位字段
- rack 状态
- navigation goal
- semantic 相关字段

当前新增的 semantic 字段包括：

- `slot_id`
- `anchor_id`
- `rack_id`
- `semantic_id`
- `semantic_target_label`
- `nav_goal_source`
- `nav_goal`
- `home_goal`
- `shelf_pose`

当前配置的 home navigation goal：

- `(0.0, 0.0, 0.0)`

但这里要注意：

- 这是 blackboard 默认值
- 不是说 localization 会自动把机器人初始化在 `(0,0,0)`

### 4.3 Customer 树

当前默认 customer 树流程：

1. `SetCurrentItem`
2. `ResolveCurrentItemSemanticTarget`
3. `NavigateToGoalPose(nav_goal)`
4. 对每个 item 重复
5. `SetHome`
6. `NavigateToGoalPose(nav_goal)`

当前真实行为：

- 先用 semantic map 解析目标
- semantic 解析失败时回退到 legacy `x/y/z`
- 当前不执行 manipulation
- 当前不调用 `ChangeInventory`

所以当前 customer 树本质上是一个“导航到点位再回 home”的树。

### 4.4 Restock 树

当前默认 restock 树流程：

1. `SetCurrentItem`
2. `ResolveCurrentItemSemanticTarget`
3. `NavigateToGoalPose(nav_goal)`
4. `ChangeInventory`
5. 重复
6. `SetHome`
7. `NavigateToGoalPose(nav_goal)`

当前真实行为：

- manipulation 相关步骤仍大量注释掉
- inventory 更新是打开的

但当前代码里有一个非常关键的真实问题：

- `ChangeInventory` 无论什么 mode 都调用 `/api/inventory/decrement`
- 它不会按 `mode` 分支
- 所以当前 restock 树实际上是在“减库存”，不是“加库存”

这不是设计推测，而是当前代码的真实行为。

### 4.5 Semantic target 节点

当前 semantic BT 节点：

- `ResolveCurrentItemSemanticTarget`

当前行为：

- 从当前 item 里取 `product_id` 和 `name`
- 调 `/semantic_map/resolve_target`
- 把解析结果写回 blackboard
- 如果服务不可用或找不到，就回退到 legacy 坐标继续走

当前 fallback 路径：

- `SetCurrentItem` 会先写入：
  - `bb.nav_goal = (x, y, 0.0)`，来自 legacy order item 字段
  - `bb.rack_goal = z`
- semantic 成功时再覆盖这些值
- semantic 失败时就继续用这些 legacy 值

### 4.6 Nav2 动作叶子

当前导航 BT 叶子：

- `NavigateToGoalPose`

当前行为：

- 从 `bb.nav_goal` 读目标
- 支持 tuple / dict / PoseStamped
- 调 Nav2 `NavigateToPose`
- 已经不再用旧的硬编码 `(2.0, 0.0)` demo 点

### 4.7 完成回报行为

当前 BT 完成时：

- executor 会向 `/api/order/complete` 上报完成状态

当前 backend 的 `/api/order/complete` 行为：

- 只更新 `order_id` 表

因此：

- customer 完成回写在表层面是成立的
- employee / restock 就算通过别的方式塞进树里，默认完成回报也没有对应 `restock_id` 更新路径

### 4.8 另一条 ViperX 路径

当前代码里还存在一条 ViperX 专用 BT 路径：

- `bt_executor_viperx.py`
- `trees/customer_viperx.py`
- `trees/restock_viperx.py`
- `bt_nodes/viperx_nodes.py`

当前状态：

- 代码存在
- 但不是上面那条默认执行链的主路径
- 仍然大量注释或未完成
- 当前也没有与员工 UI 主流程打通

## 5. 当前机械臂执行栈

仓库现在已经有比较真实的 VX300 侧 manipulation backend，但默认 BT 链还没有把它完整接起来。

当前已经实现的 manipulation backend：

- `robot_manipulation/src/arm_controller.cpp`
- launch：`robot_manipulation/launch/vx300_moveit.launch.py`
- action：`robot_interfaces/action/PickArm.action`

当前 manipulation backend 能力：

- MoveIt 规划
- `pick_arm` action server
- pre-grasp / approach / retreat
- 可选 Cartesian approach

当前 BT 侧状态：

- generic arm BT node 仍是 TODO
- 默认 customer / restock 树里大部分 manipulation 步骤仍然注释掉

也就是说当前仓库里已经出现了：

- 比之前更真实的 arm backend
- 但还没有完整接到默认业务执行链上的 end-to-end manipulation

## 6. 后端与数据库审查

### 6.1 Backend 结构

当前 backend：

- Express
- body-parser
- cors
- PostgreSQL via `pg`
- semantic map 文件逻辑在 `semantic_map.js`

当前 backend 启动时会做：

- 如果没有 `semantic_map_versions` 就创建
- 把旧的 `testmapMain_v1` 这类 semantic id 归一化到 `testmapMain`
- 如果版本历史为空，就从当前 YAML bootstrap 一条版本快照
- 最后监听 `3000` 端口

### 6.2 当前数据库模型

当前 `001_schema.sql` 里的表：

- `customer`
- `employee`
- `inventory`
- `order_id`
- `restock_id`
- `semantic_map_versions`

当前数据库层面的重要现实：

- `inventory` 里仍然有 legacy `x`, `y`, `z`
- customer order 仍会把这些 legacy 位置字段固化进 order JSON
- semantic map 还没有被完全拆成关系型表（rack / slot / placement 等）
- semantic history 当前是 snapshot 存储，不是结构化 semantic entity 存储

### 6.3 当前 semantic map API 行为

当前 semantic 相关接口：

- `GET /api/maps/base/:mapName.pgm`
- `GET /api/maps/semantic/current`
- `PUT /api/maps/semantic/current`

当前认证要求：

- semantic 接口需要 employee auth
- base-map 文件接口不强制 auth

当前 `GET /api/maps/semantic/current` 的行为：

- 从磁盘读取 YAML bundle
- 从 `Maps/testmapMain.yaml` 读取 static map 元数据
- 从 `Maps/testmapMain.pgm` 解析宽高
- 从 `semantic_map_versions` 里附上最新版本信息
- 用 `inventory` 给 slot 的 `product_ids` 做 enrich
- 返回 enriched JSON bundle

当前 `PUT /api/maps/semantic/current` 的行为：

- 接收 Store Map UI 发来的 bundle
- 先保存 YAML
- 再写数据库版本快照
- 如果 DB 写失败，就把旧 YAML 回滚回去

这段 rollback 很重要：当前实现至少在 UI save 这条链路上尽量保证 YAML 和 DB version history 一致。

### 6.4 Auth 与 session 模型

当前 employee auth 流程：

- `POST /api/employee/auth/login`
- 从数据库读取明文密码并比较
- 返回 bearer token
- token 保存在内存 session map 里，TTL 12 小时

当前限制：

- token 只存在进程内存里
- backend 重启后 session 全失效
- 当前 schema 与 seed data 里的密码还是明文

### 6.5 Order 与 restock 接口

当前机器人执行链实际使用的 order 接口：

- `GET /api/order/latest`
- `POST /api/order/ack`
- `POST /api/order/complete`

当前行为：

- 这些接口只围绕 `order_id`
- 没有一条对等的自动 fetch / ack / complete 路径给 `restock_id`

当前员工 UI 使用的 restock 接口：

- `POST /api/employee/restock/submit`

当前行为：

- 往 `restock_id` 表插记录
- 但没有打到默认 executor 的轮询链路里

所以当前 employee restock dashboard 和默认 robot polling path 仍然没有真正打通。

### 6.6 占位 robot status

当前 robot status 接口：

- `GET /api/employee/robot/status`

当前行为：

- 返回三台硬编码机器人
- 如果 semantic map 里有对应 anchor，就用 semantic anchor pose 作为展示位置
- 不订阅 ROS
- 只是 dashboard 的占位数据源

## 7. 前端审查与前后端连接关系

### 7.1 前端结构

当前员工 dashboard：

- Vue 3
- Vue Router
- Vite

当前受保护路由：

- `/app/inventory-report`
- `/app/location-map`
- `/app/slam-map`
- `/app/robot-status`
- `/app/restock`
- `/app/employee-accounts`

当前 auth guard：

- 检查 `localStorage` 里的 `fleet_token`

### 7.2 登录流

当前登录流程：

1. 员工输入 `employee_ID` 和 password
2. 前端调用 `POST /api/employee/auth/login`
3. token 与员工信息写入 `localStorage`
4. router 跳到 `/app/inventory-report`

### 7.3 Inventory report 页面

当前页面：

- `InventoryReportView.vue`

当前 backend contract：

- `GET /api/employee/inventory/report`

当前行为：

- 渲染 summary cards
- 渲染 category breakdown
- 渲染 inventory 明细表
- 明细表里仍然直接显示 legacy `x/y/z`

这说明当前 UI 层仍然同时暴露着旧位置模型和新 semantic 模型。

### 7.4 Store Map 页面

当前页面：

- `LocationMapView.vue`

当前绘图组件：

- `SemanticMapCanvas.vue`

当前行为：

- 从 backend 拉 semantic map bundle
- 解析并渲染底图 PGM
- 自动 fit 到内容范围
- 支持 zoom / pan
- 支持 edit mode
- 支持拖拽 anchor / rack / slot
- 支持手工数值编辑 pose
- 支持保存

当前 save flow：

1. 员工登录拿到 bearer token
2. 页面调用 `GET /api/maps/semantic/current`
3. backend 返回基于 YAML 的 bundle，并附上 DB version
4. 用户在页面本地编辑 geometry
5. 页面调用 `PUT /api/maps/semantic/current`
6. backend 写 YAML，并插入 semantic version 快照
7. 页面重新加载保存后的 bundle，并显示新的版本号

当前 Store Map 数据真相：

- 在线 geometry 主源：YAML
- 版本历史主源：PostgreSQL

### 7.5 SLAM 页面

当前页面：

- `SlamMapView.vue`

当前实现：

- 不是 Foxglove
- 也不是之前那套自绘 `/map + /tf` canvas
- 现在是一个 iframe 外层壳

当前嵌入应用来源：

- vendored fork：`third_party/ros_web_gui_app`
- 源仓库记录为：`https://github.com/StarLionJiang/ros_web_gui_app`

当前运行模型：

- 主 Vue app 提供 `/embedded/ros-web-gui/index.html`
- iframe 加载这个嵌入应用
- 嵌入应用自己管理 `rosbridge` websocket 连接
- 页面默认展示的 bridge 地址是 `ws://<host>:9090`

当前直接后果：

- Node backend 不在 live SLAM transport path 里
- SLAM 页不会通过 Express 代理 ROS topic
- live ROS visualization 依赖单独运行的 `rosbridge_server`

### 7.6 Robot status 页面

当前页面：

- `RobotStatusView.vue`

当前 backend contract：

- `GET /api/employee/robot/status`

当前行为：

- 只显示 dashboard cards
- 没有 live ROS 订阅
- 状态来源是 backend 生成的 placeholder 数据

### 7.7 Restock 与 accounts 页面

当前 restock 页面：

- 从 `GET /api/employee/inventory/options` 拉产品列表
- 调 `POST /api/employee/restock/submit` 提交

当前 employee accounts 页面：

- 从 `GET /api/employee/accounts` 读
- 调 `POST /api/employee/accounts/create` 写

这些页面和 backend 是连通的，但和 ROS 执行链还没有深度打通。

## 8. 信息流审查

### 8.1 传感器到地图

当前 mapping 信息流：

`SICK PicoScan point cloud -> /cloud_all_fields_fullframe`

`/cloud_all_fields_fullframe -> base_link_crop_filter -> /cloud_all_fields_fullframe_filtered`

`/cloud_all_fields_fullframe_filtered + /sick_scansegment_xd/imu + /odom -> Cartographer mapping`

`Cartographer -> live occupancy grid -> mapping visualization/export`

### 8.2 定位到规划执行

当前 localization 信息流：

`STM32 telemetry -> nav2_serial_bridge -> /odom_raw`

`/odom_raw + IMU -> EKF -> /odom`

`point cloud + IMU + /odom + pbstream -> Cartographer localization`

`map_server -> /map`

`Nav2 planner/controller -> NavigateToPose action execution`

### 8.3 Customer order 到机器人

当前 customer 信息流：

`customer order 写入 order_id`

`BT executor 轮询 /api/order/latest`

`backend 返回 items，里面带 legacy x/y/z 与 product metadata`

`BT 把 JSON 转成 Order / OrderItem`

`SetCurrentItem 先写 legacy nav goal`

`ResolveCurrentItemSemanticTarget 再尝试做 product -> slot / service pose 解析`

`NavigateToGoalPose 发 Nav2 目标`

`树结束后回 home`

当前 customer 默认流里没有发生的事：

- 没有 pick action
- 没有 gripper action
- 没有 rack action
- 没有库存扣减

### 8.4 Semantic edit 流

当前 semantic edit 信息流：

`employee login -> bearer token`

`LocationMapView -> GET /api/maps/semantic/current`

`backend -> YAML bundle + map metadata + DB version + inventory enrich`

`user edits geometry`

`LocationMapView -> PUT /api/maps/semantic/current`

`backend -> save YAML -> insert semantic_map_versions snapshot`

### 8.5 SLAM Web visualization 流

当前 SLAM Web 信息流：

`browser 打开 /app/slam-map`

`SlamMapView 加载 iframe /embedded/ros-web-gui/index.html`

`iframe 内部应用直接连接 rosbridge`

`嵌入应用自己渲染 ROS topic`

也就是说当前 live SLAM 页面在实时传输层面绕过了 Express backend。

## 9. 当前技术栈清单

ROS / navigation：

- ROS 2 Humble 风格 launch 与节点
- SICK `sick_scan_xd`
- Cartographer ROS
- Nav2
- robot_localization
- tf2
- 自定义 point-cloud crop filter
- 自定义 serial bridge

任务执行：

- `py_trees`
- Python BT executor
- 自定义 semantic service client
- 自定义 Nav2 action BT 叶子

机械臂：

- MoveIt
- 自定义 `pick_arm` action server
- Interbotix / ViperX 相关实验节点

Backend：

- Node.js
- Express
- PostgreSQL
- `pg`
- 可选 Python `deep-translator`，用于 inventory translation

Frontend：

- Vue 3
- Vue Router
- Vite
- 自定义 Store Map canvas
- 嵌入式 vendored React / Three.js ROS GUI

嵌入式 SLAM viewer：

- vendored `ros_web_gui_app`
- React
- Three.js
- `roslib`
- `rosbridge_server`

## 10. 当前缺口、风险与不一致点

当前最重要的代码真实缺口如下：

1. Restock UI 到 robot execution 仍未打通。
   - UI 写 `restock_id`
   - 默认 executor 只轮询 `order_id`

2. 库存更新语义当前是不一致的。
   - customer 树不改库存
   - restock 树反而减库存

3. Robot status 页面不是 live ROS。
   - 只是 dashboard placeholder 数据

4. Semantic map 主源当前仍是分裂的。
   - YAML 是在线主源
   - DB 是版本历史
   - 直接改 DB 不会驱动当前 UI live bundle

5. Manipulation 只接了一部分。
   - backend action server 已有
   - 默认 BT 路径还没真正 end-to-end 用起来

6. 嵌入式 SLAM 页依赖外部运行时。
   - 需要单独跑 `rosbridge_server`
   - iframe 里的连接不经过 Node backend

7. Nav2 的命令权仍是简单 topic subscription。
   - manual / autonomous 之间还没有真正实现 arbiter

## 11. 当前运行说明

当前推荐入口：

- mapping：
  - `ros2 run robot_navigation nav_assistant mapping-stack`
- localization + Nav2：
  - `ros2 run robot_navigation nav_assistant localization-stack --map-name testmapMain --with-nav2-rviz true`
- backend：
  - `cd order-api-postgre && npm run dev`
- employee UI：
  - `cd order-api-postgre/fleet-manager && npm run dev`
- SLAM 页 live bridge：
  - `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`

当前员工测试登录：

- `000AAA / team21`

## 12. 结论

当前仓库已经不再只是一个“能跑导航”的 ROS 项目，而是出现了一套真实的多表面系统：

- 可用的 mapping / localization / Nav2 栈，
- 在 ROS 和 Web 两侧都存在的 semantic overlay，
- 可编辑的 Store Map 工具，
- 以及一个基于 vendored ROS GUI 的浏览器 SLAM 页面。

但 end-to-end autonomy 还没有完全闭环。

现在导航栈本身比之前真实得多，主要薄弱点已经从“底层算法是否成立”转移到了“流程是否完全接通”上，尤其是：

- customer / restock 的执行语义，
- inventory side effect，
- dashboard 里的 live robot status，
- 以及 BT 到 arm execution 的完整接线。

## 13. 详细 Launch、节点、Topic、Service、Action 清单

### 13.1 Launch 文件清单

| Launch 文件 | 当前用途 | 包含的节点 / include | 关键默认值 |
| --- | --- | --- | --- |
| `launch/slam_mapping_stack.launch.py` | 完整 mapping 栈 | `sick_generic_caller`、2 个 static TF publisher、`base_link_crop_filter`、`cartographer_mapping.launch.py`、`nav2_serial_bridge`、可选 `ekf_node`、可选 collision launch、可选 RViz | crop 默认开启，`cmd_topics=["/cmd_vel","/cmd_vel_nav","/cmd_vel_smoothed"]`，`odom_topic=/odom_raw` |
| `launch/cartographer_mapping.launch.py` | 纯 Cartographer mapping 核心 | `cartographer_node`、`cartographer_occupancy_grid_node` | `configuration_basename=pico_2d_mapping_quality.lua`，`resolution=0.03` |
| `launch/nav2_localization_stack.launch.py` | localization + Nav2 + semantic map | `sick_generic_caller`、2 个 static TF publisher、`nav2_serial_bridge`、可选 `ekf_node`、可选 `base_link_crop_filter`、`cartographer_localization.launch.py`、`map_server`、map lifecycle manager、延迟启动的 `nav2_bringup/navigation_launch.py`、可选 Nav2 RViz、可选 `semantic_map_server` | 默认地图 `testmapMain`，crop 默认关闭，semantic map 默认开启，`cmd_topics=["/cmd_vel"]` |
| `launch/cartographer_localization.launch.py` | Cartographer localization 核心 | `cartographer_node`、可选 `cartographer_occupancy_grid_node` | `configuration_basename=pico_2d_localization.lua`，`load_frozen_state=true`，本地 occupancy grid 默认关闭 |

### 13.2 Runtime executable 清单

#### `robot_navigation` executables

| Executable | 角色 | 发布 | 订阅 | Services / actions | 当前状态 |
| --- | --- | --- | --- | --- | --- |
| `nav2_serial_bridge` | STM32 命令与 telemetry 桥 | 配置的 `odom_topic` 上的 `nav_msgs/Odometry`，可选 TF | 配置的 `cmd_topics`，`/front_alert`，`/back_alert`，`/left_alert`，`/right_alert` | 无 | mapping 与 localization 都在用 |
| `base_link_crop_filter` | 从点云里去掉底盘自击中区域 | 过滤后的 `PointCloud2` | 原始 `PointCloud2` | 无 | mapping 默认启用 |
| `semantic_map_server` | semantic overlay 发布与 semantic target 解析 | `/semantic_map/markers` | 无 | `/semantic_map/resolve_target` | localization 默认启用 |
| `nav_assistant` | launch / save-map / export-map / goal / teleop macro CLI | 在 motion 模式下会临时发 `Twist` | 无持久订阅 | 内部使用 `WriteState`、`NavigateToPose`、`FollowWaypoints` client | 操作员工具，不是常驻 runtime 节点 |
| `teleop_cmd_vel` | 简单键盘 teleop | `Twist` | 无 | 无 | 工具节点 |
| `teleop_cmd_vel_collision` | 带单个碰撞停机输入的键盘 teleop | `Twist` | `right_alert` | 无 | 工具节点，不在主 launch 栈里 |
| `teleop_ros` | 旧版可配置 teleop publisher | `Twist` | 无 | 无 | legacy 工具 |
| `wheel_motor` | 旧版 I2C mecanum 驱动路径 | 无 | 可配置 `cmd_vel`、`obstacle_alert` | 无 | legacy 路径，不是当前 Nav2 主栈的一部分 |

#### `robot_task_manager` executables

| Executable | 角色 | ROS 接口 | 外部接口 | 当前状态 |
| --- | --- | --- | --- | --- |
| `bt_executor` | 默认 customer / employee BT executor | `/order/new` service；BT 内部含 Nav2 action client；BT 内部含 semantic resolve client | 轮询 `GET /api/order/latest`；POST `/api/order/ack`；POST `/api/order/complete`；`ChangeInventory` POST `/api/inventory/decrement` | 当前默认路径 |
| `bt_executor_viperX` | 另一条偏 ViperX 的 executor | 同样提供 `/order/new` service；包含 ViperX 专用 BT nodes | 和默认 executor 一样的 backend polling 模式 | 存在，但不是主业务路径 |

#### `robot_manipulation` executables

| Executable | 角色 | 接口 | 当前状态 |
| --- | --- | --- | --- |
| `arm_controller` | MoveIt-backed arm action server | `PickArm` action、`JointTrajectory` publisher | 已实现且可用 |
| `viperx_arm_server` | MoveIt-backed ViperX arm action server | 默认 `pick_viperx` 的 `PickArm` action | 已实现 |
| `arm_waypoint_server` | waypoint 风格 arm action server | 默认 `pick_arm_waypoint` 的 `PickArm` action | 已实现 |
| `arm_demo_controller` | 简化版 arm demo action server | 默认 `pick_arm_demo` 的 `PickArm` action | 用于 demo / 测试 |
| `rack_controller` | rack lift action server | `move_rack` 上的 `MoveRack` action | 已实现 |
| `arm_motor` | I2C hardware bridge | `JointTrajectory` subscriber | 已实现 |
| `arm_to_gazebo` | 仿真桥 | Gazebo 集成相关 pub / sub | 已实现 |
| `vx300_quick_move.py` | 诊断 / helper 脚本 | publishers + service clients | 工具 |
| `vx300_hardware_diagnostics.py` | 诊断 / helper 脚本 | publishers + service clients + subscriptions | 工具 |

### 13.3 当前主流程里的 ROS topic 清单

#### 导航与定位相关 topics

| Topic | 类型 | Producer | 主要 Consumer | 说明 |
| --- | --- | --- | --- | --- |
| `/cloud_all_fields_fullframe` | `sensor_msgs/PointCloud2` | SICK driver | crop filter、Nav2 costmap | 原始点云 |
| `/cloud_all_fields_fullframe_filtered` | `sensor_msgs/PointCloud2` | crop filter | Cartographer mapping / localization | 当前 Cartographer 输入 |
| `/scan_fullframe` | `sensor_msgs/LaserScan` | SICK driver | 当前不是主 Cartographer 路径 | 仍然可用 |
| `/sick_scansegment_xd/imu` | IMU | SICK driver | EKF、Cartographer | 当前 IMU 路径 |
| `/odom_raw` | `nav_msgs/Odometry` | `nav2_serial_bridge` | EKF | 底盘 raw odom |
| `/odom` | `nav_msgs/Odometry` | EKF | Nav2、Cartographer localization | 过滤后的 odom |
| `/map` | `nav_msgs/OccupancyGrid` | mapping 时来自 Cartographer；localization 时来自 `map_server` | RViz、Nav2、嵌入式 ROS GUI | 来源取决于运行模式 |
| `/cmd_vel` | `geometry_msgs/Twist` | teleop 或 Nav2 路径 | serial bridge | localization 栈默认监听这个 |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | 可选 nav 路径 | mapping 栈默认桥接可监听 | localization 默认不监听 |
| `/cmd_vel_smoothed` | `geometry_msgs/Twist` | 可选 smoother 路径 | mapping 栈默认桥接可监听 | localization 默认不监听 |
| `/semantic_map/markers` | `visualization_msgs/MarkerArray` | semantic map server | RViz | semantic overlay 显示 |

#### 当前接进 bridge 的碰撞 / 安全 topics

| Topic | 类型 | Consumer | 效果 |
| --- | --- | --- | --- |
| `/front_alert` | `std_msgs/Bool` | `nav2_serial_bridge` | 为 true 时禁止前进 |
| `/back_alert` | `std_msgs/Bool` | `nav2_serial_bridge` | 为 true 时禁止后退 |
| `/left_alert` | `std_msgs/Bool` | `nav2_serial_bridge` | 为 true 时禁止向左 |
| `/right_alert` | `std_msgs/Bool` | `nav2_serial_bridge`、`teleop_cmd_vel_collision` | 为 true 时禁止向右 / 让 teleop 停车 |

### 13.4 ROS service 清单

| Service | 类型 | Server | 当前用途 |
| --- | --- | --- | --- |
| `/semantic_map/resolve_target` | `robot_interfaces/srv/ResolveSemanticTarget` | `semantic_map_server` | 把 product / slot / anchor 解析成 nav pose 与 service pose |
| `/order/new` | `robot_interfaces/srv/NewOrder` | `bt_executor` 或 `bt_executor_viperX` | 手工从 ROS 侧注入订单 |
| `/write_state` | `cartographer_ros_msgs/srv/WriteState` | Cartographer | `nav_assistant save-map` 用于保存 pbstream |

### 13.5 ROS action 清单

| Action | 类型 | Server | Client | 当前用途 |
| --- | --- | --- | --- | --- |
| `/navigate_to_pose` 或 `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 | `NavigateToGoalPose`、`nav_assistant goal` | active |
| `/follow_waypoints` | `nav2_msgs/action/FollowWaypoints` | Nav2 | `nav_assistant waypoints` | 工具能力 |
| `pick_arm` | `robot_interfaces/action/PickArm` | 默认由 `arm_controller` 提供 | 默认 BT 路径还没真正用上 | 后端已实现 |
| `pick_viperx` | `robot_interfaces/action/PickArm` | 默认由 `viperx_arm_server` 提供 | ViperX BT nodes | 另一条路径 |
| `move_rack` | `robot_interfaces/action/MoveRack` | `rack_controller` | rack BT node | 后端已实现；默认树路径仍注释中 |

当前实现里的一个重要细节：

- `robot_task_manager/bt_nodes/rack_nodes.py` 仍然在 import `rack_interfaces.action.MoveRack`，但这个仓库真正的接口是 `robot_interfaces/action/MoveRack`。不过这个 BT node 目前也不在默认树导入路径里，所以这个问题当前是“潜伏的”，不是主路径上的 runtime error。

## 14. 完整 Backend / Frontend API Contract

### 14.1 浏览器到 backend 的传输边界

当前前端传输模型：

- Vite dev server 跑在 `5174`
- Vite 把 `/api` proxy 到 `http://localhost:3000`
- 嵌入式 ROS GUI 的静态资源从前端 public 目录下的 `/embedded/ros-web-gui/...` 提供
- SLAM iframe 不通过 backend API 拉实时 ROS 数据，而是直接连 `rosbridge`

### 14.2 Fleet UI 当前实际使用的 endpoints

| Method | Path | Auth | 当前调用方 | 请求结构 | 响应结构 | 说明 |
| --- | --- | --- | --- | --- | --- | --- |
| `POST` | `/api/employee/auth/login` | 否 | `LoginView` | `{ employee_ID, password }` | `{ ok, token, employee }` 或 error | bearer token 来源 |
| `GET` | `/api/employee/inventory/report` | employee | `InventoryReportView` | 无 | `{ summary, category_summary, items }` | 仍然带 legacy `x/y/z` |
| `GET` | `/api/maps/semantic/current` | employee | `LocationMapView` | 无 | 含 `map`、`anchors`、`racks`、`slots`、`summary`、`version`、`generated_at` 的 semantic bundle | 基于 YAML 读出 |
| `PUT` | `/api/maps/semantic/current` | employee | `LocationMapView` | `{ bundle, change_summary }` | `{ ok, bundle, saved_at }` | 同时写 YAML 和 DB version |
| `GET` | `/api/employee/robot/status` | employee | `RobotStatusView` | 无 | `{ robots: [...] }` | 当前是 placeholder 数据 |
| `GET` | `/api/employee/inventory/options` | employee | `RestockView` | 无 | `{ items: [...] }` | restock 下拉产品列表 |
| `POST` | `/api/employee/restock/submit` | employee | `RestockView` | `{ items: [{ product_id, qty }] }` | `{ status, restock_ID }` | 只写 `restock_id` |
| `GET` | `/api/employee/accounts` | employee | `EmployeeAccountsView` | 无 | `{ items: [...] }` | 员工列表 |
| `POST` | `/api/employee/accounts/create` | employee | `EmployeeAccountsView` | `{ employee_ID, first_name, last_name, password }` | `{ ok, employee_ID, first_name, last_name }` | 明文密码入库 |

### 14.3 机器人执行链实际使用的 backend endpoints

| Method | Path | Auth | 当前调用方 | 请求结构 | 响应结构 | 说明 |
| --- | --- | --- | --- | --- | --- | --- |
| `GET` | `/api/order/latest` | 否 | `bt_executor`、`bt_executor_viperX` | 无 | `204` 或 `{ order_id, role, requester_id, items }` | 只读 `order_id`，不读 `restock_id` |
| `POST` | `/api/order/ack` | 否 | executors | `{ order_id }` | `{ ok }` | 把 `order_id.status` 置为 `IN_PROGRESS` |
| `POST` | `/api/order/complete` | 否 | executors | `{ order_id, result }` | `{ ok }` | 更新 `order_id`，不更新 `restock_id` |
| `POST` | `/api/inventory/decrement` | 否 | `ChangeInventory` BT node | `{ product_id, qty }` | `{ ok }` | 当前 restock 树也在错误地用这条 |

### 14.4 Public / customer endpoints

| Method | Path | Auth | 请求结构 | 响应结构 | 说明 |
| --- | --- | --- | --- | --- | --- |
| `POST` | `/api/auth/login` | 否 | `{ member_ID }` | customer identity block | member 登录，不要密码 |
| `POST` | `/api/account/create_customer` | 否 | `{ first_name, last_name }` | `{ ok, member_ID, first_name, last_name }` | 自动生成 member ID |
| `GET` | `/api/inventory/list` | 否 | query `lang` 可选 | `{ items }` | 可通过 `deep-translator` 翻译 |
| `POST` | `/api/order/customer` | 否 | `{ member_ID or id, guest, items }` | `{ status, order_id }` | 当前 customer 下单主路径 |
| `GET` | `/api/order/status/:id` | 否 | path ID | `{ order_id, status, result, timestamp }` | `/api` 版本 |

### 14.5 Semantic 与地图资产接口

| Method | Path | Auth | 请求 | 响应 | 说明 |
| --- | --- | --- | --- | --- | --- |
| `GET` | `/api/maps/base/:mapName.pgm` | 否 | path map name | PGM 文件字节流 | Store Map 底图 |
| `GET` | `/api/maps/semantic/current` | employee | 无 | semantic bundle | Store Map 主数据源 |
| `PUT` | `/api/maps/semantic/current` | employee | semantic bundle | saved bundle + version | 双向保存路径 |

### 14.6 仍然保留的 legacy compatibility endpoints

这些接口在 `server.js` 里还存在，但它们不是当前 fleet-manager 的主 contract：

| Method | Path | 当前角色 |
| --- | --- | --- |
| `POST` | `/order/customer/new` | 旧 customer 下单路径 |
| `POST` | `/order/employee/new` | 旧 employee restock 路径 |
| `GET` | `/inventory/list/all` | 原始库存列表 |
| `GET` | `/inventory/list/category` | 分类过滤库存列表 |
| `POST` | `/inventory/add` | 新增库存行 |
| `POST` | `/inventory/update` | 更新库存行 |
| `POST` | `/inventory/delete` | 删除库存行 |
| `GET` | `/order/status/:id` | 旧 order-status 路径 |
| `POST` | `/order/complete` | 旧 completion 路径 |

## 15. 文字版时序流

### 15.1 Employee auth 流

1. 浏览器打开 `/login`
2. 用户输入 `employee_ID` 和 `password`
3. `LoginView` 调 `POST /api/employee/auth/login`
4. Backend 在 `employee` 表里校验账号密码
5. Backend 生成一个 12 小时 TTL 的内存 session token
6. 前端把 `fleet_token`、`fleet_employee_id`、`fleet_first_name`、`fleet_last_name` 写进 `localStorage`
7. Vue Router guard 允许用户进入 `/app/...`

失败边界：

- backend 重启后，内存里的 employee session 会消失
- 浏览器 `localStorage` 里还保留旧 token，直到下一次请求收到 `401`

### 15.2 Store Map 加载流

1. 用户进入 `/app/location-map`
2. 浏览器带 bearer token 调 `GET /api/maps/semantic/current`
3. Backend 读取：
   - `workspace/src/robot_navigation/config/semantic_map_testmapMain.yaml`
   - `Maps/testmapMain.yaml`
   - `Maps/testmapMain.pgm`
   - `semantic_map_versions` 最新版本行
   - 相关 `inventory` 行
4. Backend 返回一个 enriched bundle
5. `LocationMapView` 把它 clone 成：
   - `savedBundle`
   - 可编辑的 `bundle`
6. `SemanticMapCanvas` 再通过 `/api/maps/base/testmapMain.pgm` 拉取底图
7. Canvas 用以下公式把 map 坐标转成像素：
   - `px = (x - origin_x) / resolution`
   - `py = height_px - (y - origin_y) / resolution`
8. Canvas 再把内容或整图 auto-fit 到 viewport

### 15.3 Store Map 保存流

1. 用户拖拽或手工数值编辑 anchor / rack / slot
2. 页面本地状态把 bundle 标记为 dirty
3. 用户点击 `Save`
4. 前端调 `PUT /api/maps/semantic/current`，发 `{ bundle, change_summary }`
5. Backend 先把当前 YAML 文本读出来，作为 rollback backup
6. Backend 把 bundle 序列化回 YAML 并写盘
7. Backend 往 `semantic_map_versions` 里插一条新版本
8. 如果 DB insert 失败，backend 会把旧 YAML 文本写回去
9. Backend 返回标准化后的 saved bundle 和 version
10. 前端用返回值替换本地 draft，并清掉 dirty 状态

### 15.4 SLAM 页面加载流

1. 用户打开 `/app/slam-map`
2. `SlamMapView` 加载 iframe 源 `/embedded/ros-web-gui/index.html`
3. 这个 iframe 应用来自前端静态资源，不是 backend API
4. 嵌入式 ROS GUI 自己初始化连接页
5. 用户在里面连接 `ws://<host>:9090`
6. `rosbridge_server` 成为 map、TF、scan、overlay 等 live ROS 数据的传输层

重要边界：

- 主 Vue app 不参与 SLAM 页 live ROS 数据的中转

### 15.5 Restock submit 流

1. 员工打开 `/app/restock`
2. 前端先调用 `/api/employee/inventory/options` 拉产品列表
3. 员工把选中的 product id 和 qty 加入本地 cart
4. 前端调 `POST /api/employee/restock/submit`
5. Backend 校验 product id 与 qty
6. Backend 插入一条新的 `restock_id`
7. Backend 同步更新 `employee.restock_ID`
8. 前端显示生成的 `restock_ID`

当前更大流程里的断点：

- 默认 `bt_executor` 不轮询 `restock_id`
- 所以这个提交动作目前只走到数据库持久化，不会自动进入机器人执行

## 16. 参数解释与技术说明

### 16.1 Cartographer mapping / localization 参数解释

当前 mapping 文件：

- `config/pico_2d_mapping_quality.lua`

当前 localization 文件：

- `config/pico_2d_localization.lua`

关键参数组及其当前含义：

| 参数 / 参数组 | 当前取值方向 | 当前实际含义 |
| --- | --- | --- |
| `map_frame = "map"` | 固定 | 全局地图坐标系 |
| `tracking_frame = "imu_link"` | 固定 | Cartographer 跟踪的是 IMU frame，不是 `base_link` |
| `published_frame = "base_link"` | 固定 | 下游仍以 `base_link` 作为机器人平面位姿输出 |
| `odom_frame = "odom"` | 固定 | 和 EKF 输出链一致 |
| `provide_odom_frame = true` | 开启 | Cartographer 可以维护 map/odom 关系 |
| `publish_frame_projected_to_2d = true` | 开启 | 给平面导航消费者输出 2D projected 机器人位姿 |
| `use_odometry = true` | 开启 | Cartographer 使用 odom 作为运动先验 |
| `use_imu_data = true` | 开启 | IMU 用于提升姿态稳定性 |
| `num_point_clouds = 1` | 开启 | PointCloud2 是当前主输入路径 |
| `num_laser_scans = 0` | 关闭 | LaserScan 不是当前主 mapping/localization 输入 |
| `submaps.num_range_data = 60` | 中等偏质量 | 每个 submap 累积 60 组 range data 再向前滚动 |
| `grid_options_2d.resolution = 0.03` | 固定 | 与导出地图分辨率对齐 |
| `min_range = 0.15`，`max_range = 10.0` | 较宽 | 接受较近回波与完整 aisle 范围回波 |
| `missing_data_ray_length = 10.0` | 较宽 | unknown-space ray 长度与最大有效距离一致 |
| `use_online_correlative_scan_matching = true` | 开启 | 提高局部 scan matching 鲁棒性，但算力更高 |
| `linear_search_window = 0.2` | 中等 | 平移方向搜索窗口 |
| `translation_delta_cost_weight = 0.1` | 惩罚较低 | 允许 translational search 较自由 |
| `rotation_delta_cost_weight = 0.1` | 惩罚较低 | rotation search 也较自由 |
| `motion_filter.*`（mapping） | 阈值较严 | 避免插入过多重复观测 |
| `pure_localization_trimmer.max_submaps_to_keep = 3`（localization） | 开启 | localization 模式下控制内存中的 submap 数量 |
| `constraint_builder.min_score = 0.62` | 中等偏严格 | 拒绝较弱的匹配约束 |
| `odometry_translation_weight = 1e3`，`odometry_rotation_weight = 1e3` | 较高 | 优化时对 odom 约束信任较强 |

当前实际解释：

- 整套配置偏向“odom + IMU + cloud 一起工作”的稳定 store / warehouse localization
- 它不是一个轻量的 scan-only 配置

### 16.2 EKF 参数解释

当前 EKF 文件：

- `config/ekf_odom_base_imu.yaml`

重要参数组：

| 参数 / 参数组 | 当前行为 | 实际效果 |
| --- | --- | --- |
| `frequency = 50.0` | 较高 | EKF 以 50 Hz 输出 |
| `sensor_timeout = 0.3` | 中等 | 输入 stale 后较快剔除 |
| `two_d_mode = true` | 开启 | 把机器人状态限制在平面 |
| `publish_tf = false` | 关闭 | EKF 不成为另一套 TF authority |
| `world_frame = odom` | 固定 | 过滤后的状态保持 odom 参考系 |
| `odom0 = /odom_raw` | active | bridge raw odom 是运动输入 |
| `odom0_config` | 偏 twist 选择 | 更信任平面速度和角速度，而不是 raw absolute pose |
| `imu0 = /sick_scansegment_xd/imu` | active | 融合 IMU yaw 与 yaw rate |
| `imu0_remove_gravitational_acceleration = false` | 保持默认 | 不做额外重力加速度移除预处理 |
| `imu0_config` | 只用 yaw + yaw rate | IMU 主要负责 heading 稳定 |
| `process_noise_covariance` | 已调矩阵 | 决定平滑与响应之间的取舍 |

当前实际解释：

- EKF 当前本质上是对 bridge odom 与 IMU heading 做一个平面滤波
- 不是一套完整的 3D inertial fusion 栈

### 16.3 Nav2 参数解释

当前 Nav2 文件：

- `config/nav2_params_smac_mppi_omni.yaml`

主要 section：

| Section | 当前角色 | 说明 |
| --- | --- | --- |
| `amcl` | 文件里有，但运行栈里没启 | 当前 localization 栈没有 launch AMCL，这一段实际上是 dormant 配置 |
| `bt_navigator` | Nav2 的行为树导航器 | 使用 Nav2 默认 BT XML 和 plugin 集合 |
| `controller_server` | 局部控制 | 当前插件是 `MPPIController` |
| `planner_server` | 全局路径规划 | 当前插件是 `SmacPlanner2D` |
| `local_costmap` | `odom` 坐标系下的短程 obstacle space | `VoxelLayer + InflationLayer` |
| `global_costmap` | `map` 坐标系下的规划 costmap | static map + obstacle + inflation |
| `behavior_server` | recovery 与简单 behavior action | spin、backup、drive-on-heading、assisted teleop、wait |
| `waypoint_follower` | waypoint 执行 | 配置了 wait-at-waypoint plugin |
| `velocity_smoother` | 速度平滑 | 配置已在，但 localization 栈的 bridge 默认仍只监听 `/cmd_vel` |

关键 planner / controller 参数：

| 参数 / 参数组 | 当前取值方向 | 实际效果 |
| --- | --- | --- |
| `FollowPath.plugin = nav2_mppi_controller::MPPIController` | active | 采用优化式局部控制 |
| `motion_model = "Omni"` | active | 允许横移 |
| `vx_max = 0.26`，`vy_max = 0.20`，`wz_max = 1.0` | 偏保守 | 限制底盘最大速度 |
| `batch_size = 1000`，`time_steps = 56` | 较重 | MPPI 会采样较多轨迹 |
| `VelocityDeadbandCritic.deadband_velocities = [0.08, 0.07, 0.12]` | active | 避免控制输出落在底盘 deadband 内 |
| `GridBased.plugin = nav2_smac_planner/SmacPlanner2D` | active | 基于 occupancy costmap 做 2D 图搜索规划 |
| `allow_unknown = true` | active | planner 可穿过 unknown 区域 |
| `cost_travel_multiplier = 2.0` | 中等 | 障碍物附近 cost 会明显影响路径 |

关键 costmap 参数：

| 参数 / 参数组 | 当前设置 | 实际含义 |
| --- | --- | --- |
| local costmap `global_frame = odom` | rolling local frame | 反应式控制在 odom 参考系下工作 |
| global costmap `global_frame = map` | map-fixed | 长程规划使用全局地图坐标 |
| `robot_radius = 0.40` | 共用 | 当前碰撞包络假设 |
| obstacle topic `/cloud_all_fields_fullframe` | local/global 都在用 | Nav2 obstacle 仍直接看 raw cloud |
| inflation radius `0.55` | 中等 | 让规划路径离障碍物更远一些 |

### 16.4 前端地图坐标转换

当前 Store Map 页面使用以下 ROS map 坐标到 raster 像素的转换：

- `px = (x - origin_x) / resolution`
- `py = map_height_px - (y - origin_y) / resolution`

逆变换：

- `x = origin_x + px * resolution`
- `y = origin_y + (map_height_px - py) * resolution`

这也是为什么：

- map YAML 里的 `origin` 不等于“机器人 home pose”
- 即便 raster origin 不是 `(0,0,0)`，semantic anchor 也完全可以合法地使用 `(0,0,0)`

## 17. 当前未完成项与风险矩阵

| Area | 当前代码现实 | 风险 | 严重度 | 建议下一步 |
| --- | --- | --- | --- | --- |
| Restock execution intake | employee UI 写 `restock_id`，默认 executor 只轮询 `order_id` | restock request 不会自动进入机器人执行 | High | 增加 `/api/restock/latest|ack|complete` 或统一 polling contract |
| Inventory side effects | customer 树不改库存；restock 树反而减库存 | 业务语义与库存状态会持续偏离 | High | 拆分 increment / decrement endpoint，并按 workflow 类型调用 |
| Robot status page | backend 里是占位坐标和电量 | UI 容易被误认为是真实 fleet telemetry | Medium | 接 ROS 真数据，或明确标注为 simulated |
| Semantic source-of-truth | YAML 是 live source，DB 只是 version history | 直接改 DB 不会影响当前 active semantic map | Medium | 选定 DB-primary 或保持 YAML-primary 但提供正式 sync 工具 |
| SLAM page runtime dependency | 嵌入式 app 依赖额外 `rosbridge_server` | 操作员若不知道额外步骤，页面会像“坏掉” | Medium | 加 diagnostics banner 或可选 launch helper |
| Store Map help copy | `i18n.js` 仍写着 Store Map / SLAM 是 placeholders | UI 文案和真实功能不一致 | Low | 更新 help 文案和翻译 |
| Rack BT node | `rack_nodes.py` import 的是 `rack_interfaces` 而不是 `robot_interfaces`，且当前不在默认导入路径 | 后面若启用 rack BT 可能直接 import/runtime 失败 | Medium | 修正 import 并在启用前补测试 |
| Default arm BT path | generic arm nodes 仍是 TODO / 注释状态 | 机器人能导航到位，但 manipulation 流不会闭环 | High | 把 `pick_arm` 正式接进默认 customer / restock 树 |
| Costmap obstacle source | Nav2 costmap 用 raw cloud，不是 filtered cloud | 某些姿态下自击中点仍可能影响规划 | Medium | 明确决定 Nav2 是否也切到 filtered cloud |
| Auth model | 明文密码 + 内存 session | 安全性弱，backend 重启即掉登录态 | Medium | 密码哈希化，并改为 JWT 或持久 session |

## 18. 功能到文件的直接映射

| 功能 | 主要文件 |
| --- | --- |
| Mapping bringup | `launch/slam_mapping_stack.launch.py`、`launch/cartographer_mapping.launch.py`、`config/pico_2d_mapping_quality.lua` |
| Localization + Nav2 bringup | `launch/nav2_localization_stack.launch.py`、`launch/cartographer_localization.launch.py`、`config/pico_2d_localization.lua`、`config/nav2_params_smac_mppi_omni.yaml` |
| Odom bridge 与 telemetry | `robot_navigation/nav2_serial_bridge.py`、`config/ekf_odom_base_imu.yaml` |
| Semantic map runtime | `robot_navigation/semantic_map_server.py`、`config/semantic_map_testmapMain.yaml`、`robot_interfaces/srv/ResolveSemanticTarget.srv` |
| 默认 BT 执行链 | `robot_task_manager/bt_executor.py`、`trees/customer.py`、`trees/restock.py`、`bt_nodes/semantic_nodes.py`、`bt_nodes/navigation_nodes.py`、`bt_nodes/inventory_nodes.py` |
| Store Map UI | `fleet-manager/src/views/LocationMapView.vue`、`fleet-manager/src/components/SemanticMapCanvas.vue`、`order-api-postgre/semantic_map.js`、`order-api-postgre/server.js` |
| SLAM UI 外层壳 | `fleet-manager/src/views/SlamMapView.vue`、`fleet-manager/public/embedded/ros-web-gui`、`third_party/ros_web_gui_app` |
| 员工认证与 dashboard APIs | `order-api-postgre/server.js`、`fleet-manager/src/api.js`、`fleet-manager/src/router/index.js`、`fleet-manager/src/layouts/FleetLayout.vue` |
