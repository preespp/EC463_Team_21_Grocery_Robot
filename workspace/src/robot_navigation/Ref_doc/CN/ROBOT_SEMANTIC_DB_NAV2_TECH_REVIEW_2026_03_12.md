# Robot Semantic / DB / Nav2 Tech Review

日期：2026-03-12

---

## 1. 目的与范围

本文基于当前仓库代码状态，对下列模块做联合技术评审：

- `robot_navigation`
- `robot_task_manager`
- `robot_interfaces`
- `robot_manipulation`
- `robot_vision`
- `robot_perception`
- `order-api-postgre`
- `order-api`（Firebase 旧路径）
- `mvp_robot`（历史 MVP 包）

评审目标：

1. 梳理当前“地图、导航、库存、订单、货架、商品识别、执行器”之间的真实耦合关系。
2. 明确各模块当前功能、字段定义、语义边界和实现状态。
3. 以 `PostgreSQL` 为主数据源，给出后续“语义层地图 + Nav2 + 任务树”统一方案。
4. 说明用户逻辑（customer / employee）和运行时启动逻辑应如何适配。

---

## 2. 执行摘要

当前仓库已经具备以下基础：

1. 运行时定位主线已经可以切到 `EKF + AMCL + map_server + Nav2`。
2. `order-api-postgre` 已能提供订单、库存列表、库存扣减等基础 HTTP 接口。
3. `robot_task_manager` 已经能接单、构造行为树、处理 item 列表。
4. `robot_vision` 已经能输出商品和条码在 `camera/base/map` 下的位置 payload。
5. `robot_manipulation` 已经有货架升降 action server，可按 `shelf_level` 控制升降高度。

但当前系统还没有真正形成“可长期维护的语义地图层”。核心缺口有四个：

1. 数据语义混乱：`x/y/z` 与 `aisle/rack/shelf_level` 被混用。
2. 货架实体缺失：数据库里只有商品位置，没有稳定的 `shelf_id` 实体层。
3. 任务树仍在直接吃订单里的临时坐标，而不是通过语义层查询导航目标。
4. 语义叠加模块 `semantic_overlay` 仅在文档中规划，尚未实现。

因此，下一步最合理的工程路线不是继续往库存表里堆坐标字段，而是分成两层：

1. 静态语义层：货架/货位在地图中的稳定定义。
2. 动态业务层：每个货位当前放什么商品、库存多少、最近是否被视觉确认。

在此基础上，Nav2 只负责“去哪里”，任务树只负责“拿什么、升到哪层”，数据库只负责“货架上当前有什么”。

---

## 3. 当前模块总览

| 模块 | 主要职责 | 当前状态 | 与语义地图的关系 |
|---|---|---|---|
| `robot_navigation` | 建图、定位、Nav2 运行时栈、串口桥、操作助手 | 主链已较完整 | 未来承载 `semantic_overlay` 最合适 |
| `robot_task_manager` | 订单接入、黑板、行为树执行 | 逻辑存在，但目标语义仍旧混乱 | 必须改为通过语义层查目标 |
| `robot_interfaces` | ROS 消息 / action / service 契约 | 已有基础定义 | 需要新增语义查询服务 |
| `robot_manipulation` | 货架升降、机械臂相关 | 货架升降已可用 | 可直接消费 `shelf_level` |
| `robot_vision` | 商品识别、条码识别、map/base/cam 位姿输出 | 能提供商品观测 | 适合作为动态语义修正来源 |
| `robot_perception` | 超声碰撞检测 | 已接入 teleop / navigation 安全链 | 与语义层弱耦合 |
| `order-api-postgre` | PostgreSQL 主 API | 已具备库存和订单主线 | 推荐作为主业务数据源 |
| `order-api` | Firebase 旧 API | 仍可运行，但与 Postgres 字段体系不一致 | 应降级为兼容/淘汰路径 |
| `mvp_robot` | 旧 MVP 原型包 | 主要是历史遗留 | 不应作为当前架构 owner |

---

## 4. 各模块现状与修改建议

### 4.1 `robot_navigation`

#### 当前功能

- 提供一键 mapping / localization / Nav2 运行时命令。
- 封装 SICK LiDAR、IMU、EKF、Cartographer、AMCL、Nav2 启动链。
- 当前默认运行时定位主线已向 `AMCL + Nav2` 迁移。

关键入口：

- `workspace/src/robot_navigation/robot_navigation/nav_assistant.py`
- `workspace/src/robot_navigation/launch/slam_mapping_stack.launch.py`
- `workspace/src/robot_navigation/launch/nav2_amcl_localization_stack.launch.py`

#### 当前字段/接口

- 地图资产：
  - `.pbstream`
  - `.yaml`
  - `.pgm`
- 运行时定位：
  - `map_yaml`
  - `startup_mode`
  - `nav2_params_file`
- 运行时验证：
  - `verify-localization`
  - `verify-nav-profile`

#### 观察与问题

1. Nav2 主链已经可作为语义层宿主。
2. 但当前没有：
   - `semantic_overlay` 节点
   - `shelves.yaml/json`
   - 语义查询服务
   - RViz 货架/商品 marker 发布链

#### 修改建议

1. 在 `robot_navigation` 中新增：
   - `semantic_overlay.py`
   - `config/semantic/<map_stem>.shelves.yaml`
2. 在 `nav2_amcl_localization_stack.launch.py` 中把 `semantic_overlay` 作为运行时可选/默认节点拉起。
3. 新增只读验证命令：
   - `verify-semantic-overlay`
4. 语义层应使用 `map` 作为唯一空间基准，不重新定义世界坐标系。

---

### 4.2 `robot_task_manager`

#### 当前功能

- 通过 `/order/new` 或 HTTP 轮询接单。
- 把订单转成 `Order` / `OrderItem`。
- 以 `py_trees` 构建 customer / employee 行为树。
- 维护 blackboard。

关键入口：

- `workspace/src/robot_task_manager/robot_task_manager/bt_executor.py`
- `workspace/src/robot_task_manager/robot_task_manager/blackboard.py`
- `workspace/src/robot_task_manager/robot_task_manager/bt_nodes/state_update.py`
- `workspace/src/robot_task_manager/robot_task_manager/bt_nodes/navigation_nodes.py`
- `workspace/src/robot_task_manager/robot_task_manager/bt_nodes/inventory_nodes.py`

#### 当前字段定义

当前 blackboard 主要字段：

- `bb.mode`
- `bb.order`
- `bb.order_id_text`
- `bb.items`
- `bb.item_index`
- `bb.current_item`
- `bb.num_current_item`
- `bb.current_rack`
- `bb.rack_goal`
- `bb.home_rack`
- `bb.nav_goal`
- `bb.home_goal`

#### 当前实现中的关键问题

1. `bt_executor` 把 HTTP 中的 `x/y/z` 重新映射为：
   - `item.aisle = x`
   - `item.rack = y`
   - `item.shelf_level = z`
2. `state_update.py` 又把：
   - `aisle -> nav_goal.x`
   - `rack -> nav_goal.y`
   - `shelf_level -> rack_goal`
3. 这意味着：
   - `rack` 既像业务语义里的货架编号
   - 又像平面地图 `y`
   - 已经发生语义污染
4. 导航节点当前仍是 demo 硬编码：
   - `self.x = 2.0`
   - `self.y = 0.0`
   并没有真正消费黑板里的导航目标

#### 修改建议

1. `robot_task_manager` 不再直接依赖订单中的绝对坐标。
2. 订单 item 最低只应保留：
   - `product_id`
   - `qty`
   - 可选 `shelf_id`
   - 可选 `shelf_level`
3. `SetCurrentItem` 应改为：
   - 先根据 `product_id` 查语义服务
   - 再设置 `bb.nav_goal`
   - 再设置 `bb.rack_goal`
4. `NavigateToGoalPose` 必须改回真正使用 `bb.nav_goal`。
5. 后续任务树中 `customer` 与 `employee` 的导航逻辑都应统一走语义查询层，而不是直接用订单里带的坐标。

---

### 4.3 `robot_interfaces`

#### 当前功能

- 定义订单消息和货架升降 action。

当前主要接口：

- `msg/Order.msg`
- `msg/OrderItem.msg`
- `action/MoveRack.action`
- `action/PickArm.action`
- `srv/NewOrder.srv`

#### 当前字段定义

`Order.msg`

- `int64 order_id`
- `string role`
- `string requester_id`
- `OrderItem[] items`

`OrderItem.msg`

- `string product_id`
- `string name`
- `string aisle`
- `int32 rack`
- `int32 shelf_level`
- `int32 qty`
- `float32 price`
- `int32 stock`

`MoveRack.action`

- Goal:
  - `int32 shelf_level`

#### 观察与问题

`OrderItem.msg` 当前把：

- 商品身份信息
- 货架语义
- 平面导航坐标
- 价格/库存展示信息

混在同一个消息里。

尤其 `aisle` 和 `rack` 这两个字段在代码里既被当业务标签，也被当物理坐标使用。

#### 修改建议

建议保留兼容但新增更清晰的语义接口。

V1 兼容做法：

1. `OrderItem.msg` 先不强改，避免大面积打断当前调用链。
2. 新增 service：
   - `GetShelfPose.srv`
   - `GetProductLocation.srv`
3. 语义查询结果返回：
   - `shelf_id`
   - `approach_pose`
   - `shelf_level`
   - `slot_id`
   - `label`

V2 结构化做法：

1. 新增 `SemanticItem.msg` 或 `ShelfLocation.msg`
2. 逐步让任务层改为使用新消息，而不是继续滥用 `OrderItem`

---

### 4.4 `robot_manipulation`

#### 当前功能

- `rack_controller` 已作为 `move_rack` action server 运行。
- 通过 I2C 向 ESP32 发送货架目标高度。
- 通过 `shelf_level -> height_mm` 映射控制三层货架高度。

当前参数：

- `shelf1_mm = 0.0`
- `shelf2_mm = 240.0`
- `shelf3_mm = 480.0`

关键文件：

- `workspace/src/robot_manipulation/src/rack_controller.cpp`
- `workspace/src/robot_manipulation/config/rack_position.yaml`

#### 观察

这一层的语义反而是清楚的：

- 输入是离散 `shelf_level`
- 输出是具体升降高度

#### 修改建议

1. 语义层只需要继续向 manipulation 输出 `shelf_level`。
2. 不建议把 manipulation 直接绑定到数据库绝对坐标。
3. 后续如果有多种货架，才考虑引入：
   - `shelf_type`
   - `level_height_map`

---

### 4.5 `robot_vision`

#### 当前功能

- `realsense_combination_node` 集成：
  - YOLO 商品检测
  - 条码检测
  - 深度
  - `camera/base/map` 位置估计
- 输出：
  - `realsense_combination/payload_json`
  - 可选图像显示
  - UDP payload

关键文件：

- `workspace/src/robot_vision/robot_vision/realsense_combination.py`
- `workspace/src/robot_vision/robot_vision/realsense_combination_node.py`

#### 当前 payload 关键字段

- `base_pose_map`
- `camera_pose_base`
- `products[]`
  - `name`
  - `confidence`
  - `pos_cam_m`
  - `pos_base_m`
  - `pos_map_m`
  - `yaw_map_deg`
- `barcodes[]`
  - `text`
  - `pos_cam_m`
  - `map_pose`

#### 观察

视觉层已经具备“更新动态语义”的潜力，但不适合做开机启动后的唯一语义真值。

原因：

1. 视觉会受遮挡、视角、光照影响。
2. 你要求“每次开机都能在固定位置显示货架和货物类别消息”，这应该优先依赖静态语义文件 + DB snapshot。
3. 视觉更适合做：
   - 补货确认
   - 错放检测
   - 货位占用确认
   - 商品类别交叉校验

#### 修改建议

1. 语义层把视觉作为“动态校验输入”，不是 boot-time source of truth。
2. 后续可增加 `semantic_reconciler`：
   - 把视觉结果和库存库比对
   - 标记 `confirmed / suspected_mismatch / unknown`
3. 商品 marker 可带三种状态颜色：
   - 绿色：DB 与视觉一致
   - 黄色：仅 DB 已知
   - 红色：视觉发现与 DB 不一致

---

### 4.6 `robot_perception`

#### 当前功能

- 超声波碰撞检测
- 提供前后左右告警 topic

关键文件：

- `workspace/src/robot_perception/robot_perception/distance_sensor.py`
- `workspace/src/robot_perception/launch/ultrasonic_launch.py`

#### 语义层相关性

弱耦合。

#### 修改建议

1. 不需要把它接入语义地图主链。
2. 只需在运行时 launch 中继续作为安全辅助层存在。

---

### 4.7 `order-api-postgre`

#### 当前功能

- PostgreSQL 连接与账户逻辑
- `/api/inventory/list`
- `/api/order/customer`
- `/api/order/latest`
- `/api/order/ack`
- `/api/inventory/decrement`
- `/api/order/status/:id`
- `/api/order/complete`

关键文件：

- `order-api-postgre/server.js`

#### 当前字段定义

库存列表输出字段：

- `id`
- `name`
- `price`
- `stock`
- `x`
- `y`
- `z`

订单标准化 item：

- `product_id`
- `name`
- `price`
- `stock`
- `qty`
- `x`
- `y`
- `z`

#### 观察与问题

Postgres 路径目前更接近当前 ROS 侧实现，但仍有明显问题：

1. `inventory` 直接把商品坐标存成 `x/y/z`。
2. 这会把“商品坐标”误当成“稳定语义位置”。
3. 没有显式的：
   - `shelf_id`
   - `slot_id`
   - `approach_pose`
   - `semantic_map_version`

#### 修改建议

以 PostgreSQL 为主时，推荐把 schema 重构成：

1. `shelves`
   - `shelf_id`
   - `map_stem`
   - `label`
   - `pose_x`
   - `pose_y`
   - `pose_yaw`
   - `approach_x`
   - `approach_y`
   - `approach_yaw`
   - `levels_json`

2. `inventory`
   - `product_id`
   - `product_name`
   - `category_id`
   - `category_name`
   - `price`
   - `stock`
   - `shelf_id`
   - `shelf_level`
   - `slot_id`
   - `last_seen_ts`
   - `vision_status`

3. 兼容期内保留 `x/y/z`，但应明确：
   - 它们是派生缓存字段
   - 不再作为主维护字段

4. 新增 API：
   - `GET /api/semantic/shelves`
   - `GET /api/semantic/product/:product_id`
   - `GET /api/semantic/inventory_snapshot`

---

### 4.8 `order-api`（Firebase 旧路径）

#### 当前功能

- 旧版订单和库存 UI/API
- Firestore 账户与库存

#### 当前字段定义

库存主字段：

- `aisle`
- `rack`
- `shelf_level`
- `price`
- `stock`

#### 观察与问题

这条路径和 Postgres 路径的字段体系不一致：

- Firebase：`aisle/rack/shelf_level`
- Postgres：`x/y/z`

这会导致：

1. 上层逻辑要写双套适配。
2. 订单语义无法稳定。
3. 语义层难以选定唯一 owner。

#### 修改建议

1. 以 Postgres 作为主路径。
2. Firebase 只保留为历史兼容或演示路径。
3. 后续不再新增 Firebase 侧语义字段。

---

### 4.9 `mvp_robot`

#### 当前功能

- 旧阶段原型包，包含导航、相机、轮控、UI 等 MVP 级脚本。

#### 观察

当前主系统已经有：

- `robot_navigation`
- `robot_task_manager`
- `robot_vision`
- `robot_manipulation`

因此 `mvp_robot` 更适合视为历史包，而非生产链 owner。

#### 修改建议

1. 不把语义地图功能落在 `mvp_robot`。
2. 如需保留，仅作为原型演示或旧功能回溯参考。

---

## 5. 当前用户逻辑（customer / employee）

### 5.1 customer 逻辑

当前逻辑：

1. 用户在前端页面选择商品和数量。
2. 前端通过 `order-api-postgre` 提交订单。
3. API 生成 `order_id` 并写入 `order_id` 表。
4. `BTExecutor` 轮询 `/api/order/latest`。
5. 拿到订单后构造 `Order` 和 `OrderItem`。
6. 行为树逐 item 执行：
   - 设定当前 item
   - 设置导航目标
   - 设置货架层级
   - 导航
   - 升货架/抓取/更新库存

当前问题：

1. customer 下发的 item 数据语义过重。
2. 订单不应该直接携带最终导航绝对坐标。
3. 用户只应表达“我要什么”，不应表达“机器人该去地图上哪里”。

推荐逻辑：

1. customer UI 只传：
   - `product_id`
   - `qty`
2. 后端根据 `product_id` 查库存和语义位置。
3. ROS 侧或 API 侧补全：
   - `shelf_id`
   - `shelf_level`
   - `approach_pose`

---

### 5.2 employee / restock 逻辑

当前逻辑：

1. employee 通过 UI 维护库存项。
2. 可提交补货/员工任务。
3. `BTExecutor` 走 employee 模式树。

推荐逻辑：

1. employee UI 维护的不是“商品绝对坐标”，而是：
   - 商品属于哪个 `shelf_id`
   - 哪个 `shelf_level`
   - 哪个 `slot_id`
2. 如果货架布局变化，应由语义层文件或语义管理接口更新，而不是让 employee 直接改商品 `x/y/z`。

---

## 6. 当前字段体系问题总结

### 6.1 现有字段语义冲突

| 字段 | 当前出现位置 | 当前含义 | 问题 |
|---|---|---|---|
| `x/y/z` | Postgres inventory, order payload | 有时表示 map 坐标，有时表示货架层 | 语义不稳定 |
| `aisle` | Firebase inventory, ROS `OrderItem` | 有时像 aisle 字符串，有时像 map x | 被误用为导航坐标 |
| `rack` | Firebase inventory, ROS `OrderItem` | 有时像货架编号，有时像 map y | 被误用为平面坐标 |
| `shelf_level` | Firebase, ROS, manipulation | 货架层级 | 这个字段相对清楚，应保留 |

### 6.2 推荐字段体系

建议拆成三类字段：

1. 业务标识字段
   - `product_id`
   - `product_name`
   - `category_id`
   - `category_name`
   - `stock`

2. 语义定位字段
   - `shelf_id`
   - `slot_id`
   - `shelf_level`

3. 派生空间字段
   - `pose_x`
   - `pose_y`
   - `pose_yaw`
   - `approach_x`
   - `approach_y`
   - `approach_yaw`

原则：

1. 用户侧和订单侧优先用 `product_id`。
2. 任务侧优先用 `shelf_id + shelf_level`。
3. Nav2 只吃 `approach_pose`。

---

## 7. 语义层地图建议架构

### 7.1 目标

实现“每次开机后都能在固定位置显示货架、货物类别和库存消息”。

### 7.2 架构分层

#### A. 静态语义层

使用文件：

- `Maps/<map_stem>.semantic.yaml`

内容只描述稳定设施：

- `map_stem`
- `frame_id`
- `shelves[]`
  - `shelf_id`
  - `label`
  - `shelf_pose`
  - `approach_pose`
  - `levels`
  - `slots`

#### B. 动态业务层

使用 PostgreSQL：

- 当前库存
- 商品类别
- 补货状态
- 最近视觉确认时间

#### C. 运行时叠加层

新增节点：

- `semantic_overlay`

职责：

1. 读取 `shelves.yaml`
2. 检查 map stem 是否匹配当前运行地图
3. 从 Postgres 拉取库存快照
4. 发布：
   - `MarkerArray`
   - 商品类别文本
   - 查询服务

---

### 7.3 人工标定与坐标读取指南

人工标定货架和商品位置时，必须先区分三种不同的 `x/y/z`，不能混着记录：

1. `approach_pose`
   - 含义：机器人停靠位，也就是 `map -> base_link`
   - 用途：给 `Nav2` 做到达货架前的导航目标
   - 在 2D 导航里真正需要记录的是 `x / y / yaw`，不是 `z`
2. `tag_point_camera`
   - 含义：视觉看到的条码、Tag 或商品中心点相对 `camera_link` 的位置
   - 用途：用来计算货架或商品在地图中的最终位置
   - 这里的 `point.z` 通常表示相机前方深度，不是地图高度
3. `semantic pose in map`
   - 含义：货架中心、货位中心或商品语义点在 `map` 下的最终位置
   - 用途：写入语义层和数据库，供后续任务查询

当前系统中，只要 `map -> odom -> base_link` 这条 TF 链是通的，就一定能读到 `base_link` 在地图中的坐标。因此，人工记“机器人停在货架前的那个点”时，可以直接把 `map -> base_link` 作为 `approach_pose` 候选值。

### 7.4 当前可直接使用的查看命令

1. 查看机器人当前在地图中的位置：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

或：

```bash
ros2 topic echo /amcl_pose --once
```

应该记录：

- `x`
- `y`
- `yaw`

说明：

- `z` 在当前 2D Nav2 链路里通常接近固定值，没有实际导航意义
- 如果当前运行的是 `AMCL`，这组坐标本质上来自：
  - `AMCL` 提供的 `map -> odom`
  - 再乘 `EKF / base odom` 提供的 `odom -> base_link`

2. 查看相机相对机器人的外参：

```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

这一步用于确认 `camera_extrinsic`，也就是后续把视觉点从 `camera_link` 变换到 `base_link / map` 所需的刚体外参。

3. 查看条码或 Tag 相对相机的位置：

```bash
ros2 topic echo /barcode_point --once
```

应该读取：

- `point.x`
- `point.y`
- `point.z`

注意：

- 这 3 个数位于 `camera_link` 下
- 不是 `map` 下的货架坐标
- 这里的 `z` 是深度方向，不是货架层高

4. 如果使用商品检测而不是条码检测：

- 参考 `camera_ROS2.txt` 中的 `/detections_json`
- 该链当前主要提供 `camera_link` 下的点位语义
- 若走 `realsense_combination.py`，其 UDP payload 已经包含：
  - `products[].pos_cam_m`
  - `products[].pos_base_m`
  - `products[].pos_map_m`
- 但该链当前是 UDP payload，不是标准 ROS topic，不能直接当作稳定的语义标定接口

### 7.5 当前视觉链中的一个关键限制

当前 `bar_code.py` 里的 `/barcode_pose` 不是“视觉实时反推出的未知货架地图坐标”，而是从 `BARCODE_MAP` 这个静态表里查出来再发布的已知 `map pose`。

因此：

- `/barcode_pose` 可以用于已注册条码的已知位置发布
- 不能直接用于未知货架或未知商品位置的人工标定

这意味着当前人工标定不能简单地“看 `/barcode_pose` 然后抄下来”，而应该记录下面这组中间量：

- `approach_pose_map`: 机器人停靠时的 `map -> base_link`
- `tag_point_camera`: `/barcode_point`
- `camera_extrinsic`: `base_link -> camera_link`
- `tag_to_shelf_offset`: 人工量得的 Tag 到货架中心偏移
- `slot_offset`: 人工量得的货位相对货架原点偏移

### 7.6 推荐的人工货架标定流程

1. 固定地图版本，只使用同一张 `testmap1.yaml`
2. 先让 `AMCL` 稳定，再开始记录坐标；定位未收敛前不要标定
3. 给每个货架贴一个临时视觉锚点，推荐 `AprilTag / ArUco / QR / barcode`
4. 手动把机器人停到货架正前方，保证姿态基本摆正，让相机正视锚点
5. 记录当时的 `map -> base_link`，作为 `approach_pose` 候选值
6. 同时记录 `/barcode_point`，得到 `camera -> tag`
7. 结合 `base_link -> camera_link` 外参，把视觉点转换到机器人坐标系和地图坐标系
8. 换 3 到 5 个视角重复采样，取中位数或均值，得到稳定的 `shelf_pose`
9. 再人工量一次 `tag -> shelf center` 的固定偏移，补到结果里，不要直接把 Tag 点当作货架原点

需要明确：

- `approach_pose` 是机器人停靠点，不是货架中心
- `shelf_pose` 是货架语义原点，应该由视觉点和人工偏移共同确定

### 7.7 推荐的人工商品位置标定方式

商品位置不应再以“绝对 `map x/y/z`”作为主维护字段，而应先绑定到货架语义层。

推荐记录方式：

- `shelf_id`
- `level`
- `slot_id`

然后人工量出货位相对货架原点的偏移，例如：

```yaml
slot_id: A_L1_S2
offset: {x: 0.10, y: 0.00, z: 0.12}
```

最终商品绝对位置由：

`shelf_pose + slot_offset`

推导得到，而不是由人工直接维护一份容易过期的 `product map x/y/z`。

这里还要继续区分：

- `level` 或 `shelf_level`：离散层号，供 manipulation 使用
- `offset.z` 或 `z_mm`：货位或层高在货架局部坐标系中的垂直偏移
- `map z`：当前 2D 导航场景里通常不作为语义主字段维护

### 7.8 推荐的人工标定记录模板

每个货架至少应记录：

```yaml
shelf_id: shelf_A
approach_pose: {x: ..., y: ..., yaw: ...}
shelf_pose: {x: ..., y: ..., yaw: ...}
tag_to_shelf_offset: {x: ..., y: ..., z: ...}
levels:
  - level: 1
    z_mm: ...
```

说明：

- `approach_pose`：从 `map -> base_link` 记录
- `shelf_pose`：由视觉点、TF 变换和人工偏移共同计算
- `z_mm`：来自层高定义，不来自 Nav2 地图本身

一句话总结：

人工标定时，不是在抄一份统一的 `x/y/z`，而是在分别记录：

- `map -> base_link` 的 `x / y / yaw`
- `camera -> tag` 的 `x / y / z`
- 货架局部层高和货位偏移

只有把这三类量分开，后续 `PostgreSQL + semantic map + Nav2 + task manager` 的语义才会稳定。

## 8. 对 Nav2 的适配建议

### 8.1 Nav2 应该吃什么

Nav2 不应直接吃“商品点”，而应吃“货架接近点”。

即：

- 不是 `product_pose`
- 而是 `approach_pose`

### 8.2 为什么必须有 `approach_pose`

原因：

1. 货架中心不一定可停。
2. 货架前方需要预留操控距离。
3. 视觉抓取视角通常要求固定朝向。
4. 对全向底盘和差速近似 profile，停靠点约束不同。

### 8.3 Nav2 侧推荐适配

当前运行时主线已适合接入语义层：

- `AMCL + map_server + Nav2`

推荐做法：

1. `semantic_overlay` 查询返回：
   - `approach_pose`
   - `target_shelf_id`
   - `target_level`
2. `robot_task_manager` 用该 pose 填 `bb.nav_goal`
3. `NavigateToGoalPose` 把该 pose 发给 Nav2

### 8.4 与当前 Nav2 profile 的关系

当前推荐 profile 是：

- `smac_mppi_omni`

这对“停靠到货架前固定姿态”是有利的，但前提是：

1. `vy` 真的可控
2. 货架接近动作不是硬靠墙

如果后续发现：

- 横移抖动大
- 货架前窄位停靠不稳

则建议改成：

- `smac_mppi_diff`

即让语义层稳定、而 Nav2 profile 可单独试验。

---

## 9. 建议新增接口

### 9.1 文件

1. `Maps/testmap1.semantic.yaml`
2. `workspace/src/robot_navigation/robot_navigation/semantic_overlay.py`

### 9.2 ROS service

建议新增：

1. `GetShelfPose.srv`
   - request:
     - `string shelf_id`
   - response:
     - `bool found`
     - `geometry_msgs/PoseStamped approach_pose`
     - `geometry_msgs/PoseStamped shelf_pose`
     - `int32[] levels`
     - `string label`

2. `GetProductLocation.srv`
   - request:
     - `string product_id`
   - response:
     - `bool found`
     - `string shelf_id`
     - `int32 shelf_level`
     - `string slot_id`
     - `geometry_msgs/PoseStamped approach_pose`
     - `string category_name`
     - `int32 stock`

### 9.3 HTTP API

建议新增：

1. `GET /api/semantic/shelves`
2. `GET /api/semantic/inventory_snapshot`
3. `GET /api/semantic/product/:product_id`

---

## 10. 分阶段实施计划

### Phase 0：字段冻结与语义统一

目标：

- 停止继续扩大 `x/y/z` 与 `aisle/rack/shelf_level` 混用

动作：

1. 选定 PostgreSQL 为唯一主业务库。
2. 约定：
   - `shelf_level` 保留
   - `rack` 不再表示 map `y`
   - `aisle` 不再表示 map `x`
3. 开始引入 `shelf_id`

### Phase 1：静态语义文件

目标：

- 先把货架固定到地图上

动作：

1. 为每张运行时地图建立 `semantic.yaml`
2. 为每个货架定义：
   - `shelf_pose`
   - `approach_pose`
   - `levels`

### Phase 2：`semantic_overlay`

目标：

- 开机自动显示货架与商品类别

动作：

1. 读 `semantic.yaml`
2. 拉 PostgreSQL 库存快照
3. 发布 marker
4. 提供查询服务

### Phase 3：任务层改造

目标：

- BT 不再直接吃订单坐标

动作：

1. `SetCurrentItem` 改为通过 `product_id` 查询语义层
2. `bb.nav_goal` 来自语义查询结果
3. `NavigateToGoalPose` 去掉 demo 硬编码

### Phase 4：视觉回填

目标：

- 用视觉确认动态库存状态

动作：

1. 对接 `realsense_combination/payload_json`
2. 做 DB vs vision reconcile
3. 标记 `confirmed/stale/mismatch`

### Phase 5：验证

检查项：

1. 开机后 10 秒内语义 marker 可见
2. 地图 stem 不匹配时显式报警
3. 按 `product_id` 能查到：
   - `shelf_id`
   - `approach_pose`
   - `shelf_level`
4. Nav2 能成功导航到 `approach_pose`
5. Manipulation 能按 `shelf_level` 动作

---

## 11. 我对当前代码状态的最终判断

### 已经具备的条件

1. 运行时 `AMCL + Nav2` 主链已具备。
2. PostgreSQL 主 API 已可作为后续主业务路径。
3. 视觉与货架升降模块都已有可复用能力。

### 当前最大的结构性问题

1. 语义层缺席。
2. 订单字段承载了过多不该由用户输入的空间语义。
3. 任务树与导航节点还未真正通过语义服务解耦。

### 推荐结论

后续主线应明确为：

`PostgreSQL inventory + static semantic shelf map + semantic_overlay + AMCL/Nav2 + task manager`

而不是继续沿用：

`订单直接携带 x/y/z -> BT 直接拿坐标 -> Nav2`

前者是可以长期维护的架构；后者在 demo 阶段可用，但会在地图更新、货架调整、视觉更新和多数据源并存时持续失稳。

---

## 12. 推荐的下一步文档与代码任务

建议紧接着做两件事：

1. 新写一份：
   - `SEMANTIC_OVERLAY_DESIGN_2026_03_12.md`
   内容包括：
   - `shelves.yaml` schema
   - ROS service 定义
   - marker 设计
   - 启动流程

2. 开始代码实现：
   - `robot_navigation/semantic_overlay.py`
   - `robot_interfaces/srv/GetShelfPose.srv`
   - `robot_interfaces/srv/GetProductLocation.srv`
   - `nav2_amcl_localization_stack.launch.py` 接入 overlay
