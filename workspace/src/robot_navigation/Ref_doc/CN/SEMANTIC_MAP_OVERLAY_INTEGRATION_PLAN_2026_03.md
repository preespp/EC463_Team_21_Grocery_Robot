# Semantic Map Overlay Integration Plan (2026-03-15)

## 0. 文档目标

本文档用于把当前 Grocery Robot 的 `testmapMain` 静态地图，扩展成一套可落地的语义地图系统，覆盖以下四层：

1. UI 编辑与运行态显示
2. Node.js 后端与 PostgreSQL 数据模型
3. ROS2 导航 / 任务树 / 动作执行接入
4. 地图版本、坐标变换、兼容迁移策略

本文基于两份代码基线整理：

- 当前主仓库：`F:\EC463\EC463_Team_21_Grocery_Robot`
- 旁边带 UI / PostgreSQL / fleet manager 的分支仓库：`F:\EC463\WithUI branch\EC463_Team_21_Grocery_Robot`

本文是正式实施计划，不是只给讨论用的草稿。

---

## 1. 当前代码事实

### 1.1 地图资产事实

当前主仓库已存在可直接用于 overlay 的静态地图资产：

- `Maps/testmapMain.pbstream`
- `Maps/testmapMain.yaml`
- `Maps/testmapMain.pgm`

`testmapMain.yaml` 当前关键参数：

- `resolution = 0.03 m/pixel`
- `origin = [-2.797711, -10.398200, 0.0]`

`testmapMain.pgm` 当前像素尺寸：

- `width = 428 px`
- `height = 840 px`

这意味着语义层不需要修改 occupancy map 本体，只需要在同一 `map` 坐标系上叠加额外对象即可。

### 1.2 导航主链路已具备

当前导航主链路已经可以支撑语义地图接入：

1. `slam_mapping_stack.launch.py`
   - 启动 SICK LiDAR、静态 TF、点云裁剪、Cartographer、串口桥、EKF
2. `cartographer_mapping.launch.py`
   - Cartographer 当前吃 `PointCloud2`，remap 到 `points2`
3. `nav2_localization_stack.launch.py`
   - 加载 `pbstream + yaml`
   - 启动 `map_server`
   - 启动 Cartographer localization
   - 启动 Nav2

当前运行态定位 / 导航事实：

- `map -> odom -> base_link`
- `odom_raw + imu -> EKF -> /odom`
- Nav2 已可接受 `NavigateToPose`
- 当前 localization 栈默认使用 `testmap1`，但 `nav_assistant` 已支持 `testmapMain`

### 1.3 当前任务执行链路缺少语义层

当前订单和补货任务仍然是“把商品位置直接当坐标”：

1. API `/api/order/latest`
   - 返回 `x / y / z`
2. `bt_executor.py`
   - 把 JSON 转成 `OrderItem`
3. `state_update.py`
   - 直接把 `aisle -> nav_goal.x`
   - 直接把 `rack -> nav_goal.y`
   - 直接把 `shelf_level -> rack_goal`

当前问题：

- `x / y / z` 同时扮演了商品位置、导航目标、层高语义三种角色
- 没有 aisle / rack / slot / service pose 的正式模型
- 没有地图版本号
- 没有语义图与静态图的显式绑定
- 没有执行时快照

### 1.4 BT 与动作执行仍处于半接通状态

当前 BT 事实：

1. `customer.py`
   - 已挂上 `NavigateToGoalPose`
   - 机械臂流程仍为注释 / TODO
2. `restock.py`
   - 仍在用 `MoveDistanceForCurrentItem` demo 叶子
3. `navigation_nodes.py`
   - `NavigateToGoalPose` 当前仍然硬编码发 `(2.0, 0.0)`，还没有真正消费 blackboard 里的 `nav_goal`

当前机械臂事实：

1. `robot_manipulation/src/arm_controller.cpp`
   - 已有真实 `pick_arm` action server
2. `robot_interfaces/action/PickArm.action`
   - 已定义正式 pick 接口
3. `vx300_moveit.launch.py`
   - 已能拉起 VX300 / VX300S MoveIt

因此：

- “动作执行后端”不是空白
- 真正缺的是“语义目标解析 + BT 叶子接入 + service pose 数据来源”

### 1.5 UI / 后端 / 数据库仍是占位和兼容态

带 UI 分支中的当前事实：

1. `LocationMapView.vue`
   - 仍是 placeholder
2. `SlamMapView.vue`
   - 仍是 placeholder
3. `fleet-manager/src/router/index.js`
   - 两个页面路由已预留
4. `server.js`
   - `inventory` 仍只暴露 `x / y / z`
   - `robot/status` 仍是假数据
   - `restock/submit` 仅写数据库，没有连接 ROS 任务执行

### 1.6 现有文档里已有局部思路，但没有跨栈闭环

`ROBOT_AUTONOMY_FRONTIER_ADAPTATION_PLAN.md` 已提出：

- `semantic_overlay`
- `shelves.yaml/json`
- `T_ref_map_to_new_map`

但该文档主要停留在导航侧，没有定义：

- PostgreSQL 真正数据模型
- API contract
- UI 编辑器数据流
- BT / arm / rack 与语义目标的统一接入

---

## 2. 核心问题定义

当前系统最大的问题不是“地图上还没画货架”，而是缺少一层正式的“空间业务语义”。

目前系统存在以下结构性缺口：

1. 静态地图和商品/货架数据没有正式绑定关系
2. `inventory.x/y/z` 被滥用，无法表达真实 store topology
3. UI 无法编辑 aisle / rack / slot / approach pose
4. BT 无法通过 `product_id -> slot -> service pose` 解析动作目标
5. 运行态没有语义图版本快照，任务执行不可追溯
6. 无法支撑后续多地图、多版本、位置重分配、货架调整

因此，本项目不应做成“在前端地图上随便画几个框”，而应做成：

`Static Occupancy Map + Versioned Semantic Overlay + Runtime Overlay`

---

## 3. 设计原则

### 3.1 基础地图不改写

`testmapMain.pgm / yaml / pbstream` 是导航基础资产，不直接存业务对象。

### 3.2 语义图单独版本化

语义层必须有独立 revision，支持：

- draft
- published
- archived

### 3.3 PostgreSQL 为业务真源

业务语义真源应在数据库中，不应散落在：

- Vue 页面本地状态
- ROS blackboard
- 临时 JSON 文件

ROS 只加载 runtime cache，不做长期真源。

### 3.4 执行目标必须从语义对象解析

任何可执行目标都必须走统一解析链：

`product -> placement -> slot -> nav_anchor/service_pose -> Nav2/arm/rack`

### 3.5 兼容迁移优先

第一阶段可以保留旧的 `x/y/z` 字段和旧接口，但它们必须退化成兼容层，而不是继续作为系统真源。

### 3.6 运行时必须绑定语义快照

任务一旦下发，必须记录执行时使用的：

- `map_asset_id`
- `semantic_revision_id`
- slot / anchor / service_pose 快照

防止任务执行过程中地图被改版导致漂移。

---

## 4. 目标架构

```text
                         +----------------------+
                         |  Maps/testmapMain.*  |
                         |  pgm/yaml/pbstream   |
                         +----------+-----------+
                                    |
                                    v
                         +----------------------+
                         |      map_asset       |
                         |  resolution/origin   |
                         |  width/height/path   |
                         +----------+-----------+
                                    |
                                    v
                         +----------------------+
                         |   semantic_revision  |
                         +----------+-----------+
                                    |
          +-------------------------+-------------------------+
          |                         |                         |
          v                         v                         v
   +-------------+           +-------------+           +-------------+
   | semantic_   |           | semantic_   |           | nav_anchor  |
   | zone        |           | rack/slot   |           | service_pose|
   +------+------+           +------+------+           +------+------+ 
          |                         |                         |
          +--------------------+----+-------------------------+
                               |
                               v
                     +----------------------+
                     |  product_placement   |
                     +----------+-----------+
                                |
                 +--------------+--------------+
                 |                             |
                 v                             v
         +---------------+             +---------------+
         | task_request  |             | inventory     |
         | task_item     |             | stock/price   |
         +-------+-------+             +---------------+
                 |
                 v
         +---------------+      query/resolve      +----------------------+
         | bt_executor   +------------------------>+ semantic_map_server   |
         +-------+-------+                         +----------+-----------+
                 |                                            |
                 | Nav2 goal / arm action / rack action       | markers / target lookup
                 v                                            v
         +---------------+                            +----------------------+
         | Nav2 + VX300  |                            | UI Location / SLAM   |
         +---------------+                            +----------------------+
```

---

## 5. 三层地图模型

### 5.1 Layer A: Base Map

内容：

- `pgm`
- `yaml`
- `pbstream`
- `resolution`
- `origin`
- `width_px`
- `height_px`

用途：

- Nav2
- Cartographer localization
- UI 背景图

### 5.2 Layer B: Semantic Overlay

内容：

- aisle / zone polygon
- rack centerline / rack box
- slot / shelf cell
- docking / staging / home / approach anchor
- pick / place / observe / retreat service pose
- product placement

用途：

- UI 编辑
- 任务解析
- 机器人执行目标生成

### 5.3 Layer C: Runtime Overlay

内容：

- robot 当前 pose
- active path
- current goal
- blocked zone
- selected rack / slot
- task progress

用途：

- `SlamMapView`
- operator monitoring
- debug / recovery

---

## 6. 坐标系统一规范

### 6.1 统一真源坐标

所有语义对象统一存储在 ROS `map` 坐标系下：

- `x`
- `y`
- `yaw`
- 对机械臂 service pose 再附加 `z` 和 quaternion

### 6.2 UI 像素坐标转换

前端显示时统一使用：

```text
px = (x - origin_x) / resolution
py = height_px - (y - origin_y) / resolution
```

反变换：

```text
x = px * resolution + origin_x
y = (height_px - py) * resolution + origin_y
```

### 6.3 当前 `testmapMain` 参数

当前 V1 方案应固定使用：

- `origin_x = -2.797711`
- `origin_y = -10.398200`
- `resolution = 0.03`
- `width_px = 428`
- `height_px = 840`

### 6.4 地图底图格式

建议后端在发布给 UI 时同时提供：

- `pgm` 原始路径
- 转换后的 `png` 静态资源
- `width_px`
- `height_px`
- `origin`
- `resolution`

V1 阶段推荐直接把 `testmapMain.pgm` 转为 `png`，由 UI 叠加 SVG / Canvas overlay。

---

## 7. 数据库重构方案

### 7.1 `map_asset`

用途：保存基础地图元数据。

建议字段：

- `id`
- `map_name`
- `yaml_path`
- `pgm_path`
- `png_path`
- `pbstream_path`
- `resolution_m`
- `origin_x`
- `origin_y`
- `origin_yaw`
- `width_px`
- `height_px`
- `is_active`
- `checksum`
- `created_at`

### 7.2 `semantic_revision`

用途：语义图版本管理。

建议字段：

- `id`
- `map_asset_id`
- `version_tag`
- `status` (`draft/published/archived`)
- `parent_revision_id`
- `notes`
- `created_by`
- `created_at`
- `published_at`

### 7.3 `semantic_zone`

用途：描述 aisle、入口、装货区、禁行区等区域对象。

建议字段：

- `id`
- `semantic_revision_id`
- `zone_code`
- `zone_type`
- `label`
- `polygon_json`
- `priority`
- `metadata_json`

### 7.4 `semantic_rack`

用途：描述货架级对象。

建议字段：

- `id`
- `semantic_revision_id`
- `zone_id`
- `rack_code`
- `label`
- `center_x`
- `center_y`
- `yaw`
- `width_m`
- `depth_m`
- `height_m`
- `levels`
- `side`
- `metadata_json`

### 7.5 `semantic_slot`

用途：描述可绑定商品的最小货位单元。

建议字段：

- `id`
- `rack_id`
- `slot_code`
- `level_index`
- `bay_index`
- `face`
- `center_x`
- `center_y`
- `center_z`
- `width_m`
- `height_m`
- `reach_depth_m`
- `occupancy_state`
- `metadata_json`

### 7.6 `nav_anchor`

用途：导航目标点，不直接等于货位中心。

建议字段：

- `id`
- `semantic_revision_id`
- `anchor_code`
- `anchor_type` (`approach/queue/home/docking/staging/inspect`)
- `x`
- `y`
- `yaw`
- `tolerance_xy`
- `tolerance_yaw`
- `enabled`
- `metadata_json`

### 7.7 `service_pose`

用途：机械臂与升降结构的动作执行位姿。

建议字段：

- `id`
- `target_type` (`rack/slot/product`)
- `target_id`
- `pose_role` (`pick/place/observe/retreat/pregrasp`)
- `frame_id`
- `x`
- `y`
- `z`
- `qx`
- `qy`
- `qz`
- `qw`
- `rack_level`
- `arm_group`
- `ee_link`
- `metadata_json`

### 7.8 `product_placement`

用途：把商品绑定到真实语义货位。

建议字段：

- `id`
- `product_id`
- `semantic_revision_id`
- `slot_id`
- `primary_nav_anchor_id`
- `pick_service_pose_id`
- `place_service_pose_id`
- `is_primary`
- `priority`
- `valid_from`
- `valid_to`
- `metadata_json`

### 7.9 `task_request`

用途：统一 customer / restock / audit / escort 任务模型。

建议字段：

- `id`
- `task_type` (`customer_pick/restock/audit/escort`)
- `requester_type`
- `requester_id`
- `status`
- `priority`
- `map_asset_id`
- `semantic_revision_id`
- `source_order_id`
- `source_restock_id`
- `created_at`
- `claimed_at`
- `completed_at`
- `result_json`

### 7.10 `task_item`

用途：任务内每个商品或动作子项的执行快照。

建议字段：

- `id`
- `task_request_id`
- `sequence_no`
- `product_id`
- `qty`
- `slot_id`
- `nav_anchor_id`
- `service_pose_id`
- `execution_state`
- `snapshot_json`

### 7.11 `robot_state_snapshot`

用途：给 UI 运行态看板提供 live overlay 数据。

建议字段：

- `id`
- `robot_id`
- `map_asset_id`
- `semantic_revision_id`
- `pose_x`
- `pose_y`
- `pose_yaw`
- `battery_pct`
- `mode`
- `task_request_id`
- `current_anchor_id`
- `updated_at`
- `source`

### 7.12 `map_alignment`

用途：存语义图参考系与当前基础地图的对齐信息。

建议字段：

- `id`
- `map_asset_id`
- `semantic_revision_id`
- `reference_name`
- `method`
- `transform_json`
- `anchor_pairs_json`
- `error_m`
- `active`

---

## 8. 旧数据兼容策略

### 8.1 保留 `inventory.x/y/z`，但降级为兼容字段

在迁移期内：

- `inventory.x` 不再代表“商品真位置”
- `inventory.y` 不再代表“rack 真结构”
- `inventory.z` 不再代表“slot 真语义”

它们改为：

- 从 `product_placement + nav_anchor/slot` 派生出的兼容投影值

### 8.2 旧接口兼容策略

第一阶段保留以下接口可用：

- `/api/inventory/list`
- `/api/order/customer`
- `/api/order/latest`

但内部改成：

1. 先查 `product_placement`
2. 生成 `slot / anchor / service_pose`
3. 再回填 legacy `x/y/z`

### 8.3 兼容视图建议

可增加数据库 view：

- `inventory_location_compat`

把：

- `product_id`
- `slot_code`
- `rack_code`
- `anchor_x`
- `anchor_y`
- `rack_level`

映射为旧客户端可读格式。

---

## 9. API Contract 方案

## 9.1 地图基础接口

### `GET /api/maps/active`

返回当前 active map 与 published semantic revision：

```json
{
  "map_asset": {
    "id": 1,
    "map_name": "testmapMain",
    "png_url": "/static/maps/testmapMain.png",
    "resolution_m": 0.03,
    "origin": [-2.797711, -10.3982, 0.0],
    "width_px": 428,
    "height_px": 840
  },
  "semantic_revision": {
    "id": 12,
    "version_tag": "main_store_v1",
    "status": "published"
  }
}
```

### `GET /api/maps/:mapId/semantic`

返回该地图当前 revision 的 zones / racks / slots / anchors / service poses / placements。

### `POST /api/maps/:mapId/semantic/revisions`

创建新 draft revision。

### `PUT /api/maps/:mapId/semantic/revisions/:revisionId`

保存语义图编辑结果。

### `POST /api/maps/:mapId/semantic/revisions/:revisionId/publish`

发布当前 draft。

## 9.2 商品位置解析接口

### `POST /api/semantic/resolve-product`

输入：

- `product_id`
- `qty`
- `task_type`

输出：

- `slot`
- `rack`
- `nav_anchor`
- `service_pose`
- `semantic_revision_id`

### `GET /api/tasks/:taskId/execution-plan`

返回任务执行快照，供机器人和 UI 查看。

## 9.3 任务接口统一化

### 新建议

新增统一任务接口：

- `POST /api/task/submit`
- `GET /api/task/next`
- `POST /api/task/:id/ack`
- `POST /api/task/:id/progress`
- `POST /api/task/:id/complete`

### 旧接口保留但改为适配层

- `/api/order/customer`
- `/api/employee/restock/submit`

内部都应转写为 `task_request + task_item`。

## 9.4 机器人运行态接口

### `GET /api/robots/live`

返回：

- 当前位置
- 当前任务
- 当前目标 anchor / slot
- 电量
- mode

V1 可先用 REST polling；
V2 再升级为 WebSocket / SSE。

---

## 10. UI 方案

## 10.1 `LocationMapView` 目标

`LocationMapView` 不应是“看地图”的页面，而应是“语义地图编辑器”。

### 页面结构建议

1. 左侧：Layer / Revision / Filter 面板
2. 中间：基于 `testmapMain.png` 的主地图画布
3. 右侧：属性面板 Inspector
4. 顶部：发布 / 回滚 / 自动生成 slot / 保存草稿

### 第一阶段功能

1. 加载 base map
2. 显示已发布 semantic overlay
3. 支持创建 / 编辑：
   - zone
   - rack
   - slot
   - nav anchor
4. 支持把商品绑定到 slot
5. 支持批量生成 rack 下的 slot
6. 支持保存 draft revision
7. 支持 publish

### 关键交互

1. 点击 rack -> 右侧显示 rack 参数
2. rack 内自动生成层位
3. 拖拽商品到 slot
4. 选 slot 自动显示：
   - 对应 nav anchor
   - 对应 pick pose
   - 对应 rack level

### UI 数据流

1. `GET /api/maps/active`
2. `GET /api/maps/:mapId/semantic`
3. 编辑本地 draft
4. `PUT /api/maps/:mapId/semantic/revisions/:revisionId`
5. `POST .../publish`

## 10.2 `SlamMapView` 目标

`SlamMapView` 是运行态看板，不是编辑器。

### 页面结构建议

1. 左上：图层开关
2. 右上：robot card / battery / current task
3. 中间：与 `LocationMapView` 共用底图和 semantic layer
4. 底部：任务时间线 / 状态流

### 第一阶段功能

1. 显示 base map
2. 显示已发布 semantic layer
3. 显示 robot pose / heading
4. 显示当前 Nav2 目标
5. 显示当前任务项对应 slot / anchor
6. 显示 active path

### 刷新策略

V1：

- 1 Hz REST polling

V2：

- WebSocket / SSE 推送 live state

## 10.3 现有页面复用建议

当前 `fleet-manager/src/router/index.js` 已预留：

- `/app/location-map`
- `/app/slam-map`

因此可以直接在现有 Vue 路由体系内扩展，不需要重建页面导航。

---

## 11. ROS / BT 接入方案

## 11.1 新增 `semantic_map_server`

建议放在：

- `workspace/src/robot_navigation`

职责：

1. 从后端拉取或从导出 JSON 读取当前 published semantic revision
2. 发布 `MarkerArray`
3. 提供目标解析服务：
   - `product_id -> nav pose + service pose + rack level`
   - `slot_id -> nav pose + service pose`
4. 缓存当前 `semantic_revision_id`

V1 不做自动配准，直接使用 `testmapMain` 固定坐标。

## 11.2 新增 `robot_state_bridge`

建议放在：

- `workspace/src/robot_navigation`

职责：

1. 读取 `map -> base_link`
2. 读取当前 Nav2 action 状态
3. 读取 battery / mode
4. 定时上报到 `/api/robots/live` 对应的数据库表

## 11.3 BT 执行链路重构

### 新叶子建议

1. `ResolveCurrentItemTarget`
2. `NavigateToSemanticAnchor`
3. `MoveRackToLevel`
4. `ExecutePickArm`
5. `ExecutePlaceArm`
6. `ReportTaskProgress`

### `ResolveCurrentItemTarget` 输入输出

输入：

- `bb.current_item.product_id`
- 当前 `semantic_revision_id`

输出写回 blackboard：

- `bb.nav_goal`
- `bb.rack_goal`
- `bb.pick_pose`
- `bb.place_pose`
- `bb.slot_id`
- `bb.anchor_id`

### 现有缺口必须修复

1. `NavigateToGoalPose` 必须去掉硬编码 `(2.0, 0.0)`
2. `restock.py` 必须去掉 `MoveDistanceForCurrentItem`
3. `arm_nodes.py` 必须从 TODO 变成真正 action client

## 11.4 接口建议

建议在 `robot_interfaces` 增加服务，例如：

- `ResolveSemanticTarget.srv`
- `SemanticTarget.msg`

建议返回内容至少包括：

- `slot_code`
- `rack_code`
- `nav_pose`
- `service_pose`
- `rack_level`
- `semantic_revision_id`

---

## 12. 机械臂 / 升降结构接入策略

## 12.1 VX300 接入

当前可复用：

- `pick_arm` action
- MoveIt bringup

语义图需要补充的不是新的 arm backend，而是：

1. `slot -> pick_service_pose`
2. `slot -> place_service_pose`
3. `slot -> retreat_pose`
4. `slot -> pregrasp_offset`

## 12.2 Rack 接入

当前可复用：

- `move_rack` action server
- `rack_position.yaml`

因此语义图只需要确定：

- slot 对应 `rack_level`

## 12.3 Customer / Restock 两类动作路径

### Customer

`product -> slot -> nav_anchor -> rack_level -> pick_pose -> basket/drop_pose`

### Restock

V1 先约束为：

- 人工已把待补货商品装入机器人篮筐
- 机器人只负责去目标 slot 执行放置

因此：

`product -> destination slot -> nav_anchor -> rack_level -> place_pose`

---

## 13. 分阶段实施计划

## Phase 0: Contract 与 schema 固化

目标：

- 固定数据模型
- 固定 API 契约
- 固定坐标换算规则

输出：

- PostgreSQL migration 草案
- API contract 文档
- map asset 元数据导入脚本

验收：

- 所有后续模块都按同一字段命名

## Phase 1: Backend 兼容层

目标：

- 在不破坏现有 customer/employee UI 的前提下引入 semantic schema

任务：

1. 新建语义表
2. 初始化导入 `testmapMain`
3. 实现 `GET /api/maps/active`
4. 实现 `GET /api/maps/:id/semantic`
5. 让 `/api/inventory/list` 返回：
   - 新的语义位置字段
   - 旧 `x/y/z` 兼容字段

验收：

- 旧页面继续可用
- 新页面能拿到 semantic JSON

## Phase 2: `LocationMapView` 编辑器

目标：

- 在 UI 上可视化编辑 rack / slot / placement

任务：

1. 渲染 `testmapMain.png`
2. 加载 semantic draft
3. rack/slot 编辑
4. 商品绑定
5. publish revision

验收：

- 能用 UI 完成一张最小可用 store semantic map

## Phase 3: `semantic_map_server` + BT 解析链

目标：

- 让机器人真正消费 semantic target

任务：

1. 新建 `semantic_map_server`
2. 新建 resolve service
3. 改 `bt_executor`
4. 改 `SetCurrentItem`
5. 改 `NavigateToGoalPose`

验收：

- 输入 `product_id` 后能拿到真实 Nav2 goal 和 rack level

## Phase 4: 动作执行闭环

目标：

- customer / restock 都能走真实执行链

任务：

1. `arm_nodes.py` 实现 action client
2. `rack_nodes.py` 接上 `move_rack`
3. 引入 service pose
4. 任务进度回写 backend

验收：

- 至少一类任务能从 semantic map 一路执行到动作层

## Phase 5: `SlamMapView` 运行态 overlay

目标：

- UI 显示机器人 live state

任务：

1. `robot_state_bridge`
2. `/api/robots/live`
3. map 上显示：
   - robot pose
   - active path
   - active slot
   - current task

验收：

- 运行态页面可用于演示和调试

## Phase 6: 版本与对齐增强

目标：

- 支持 map refresh 和语义对齐

任务：

1. `map_alignment`
2. 两点人工校准
3. 后续评估自动对齐

验收：

- 语义层对齐误差可控

---

## 14. 文件级落地建议

### `order-api-postgre`

建议新增 / 改造：

- DB migration 目录
- semantic map endpoints
- inventory compatibility adapter
- live robot status endpoints

### `fleet-manager`

建议重点改造：

- `src/views/LocationMapView.vue`
- `src/views/SlamMapView.vue`
- `src/api.js`
- 新增 map canvas / overlay composables

### `workspace/src/robot_navigation`

建议新增：

- `semantic_map_server.py`
- `robot_state_bridge.py`
- 对应 launch 文件

### `workspace/src/robot_task_manager`

建议改造：

- `bt_executor.py`
- `state_update.py`
- `navigation_nodes.py`
- `arm_nodes.py`
- `customer.py`
- `restock.py`

### `workspace/src/robot_interfaces`

建议新增：

- semantic target resolve service / msg

### `workspace/src/robot_manipulation`

当前尽量复用现有：

- `pick_arm`
- `move_rack`
- `vx300_moveit.launch.py`

不建议第一阶段重写 manipulation backend。

---

## 15. 风险与缓解

### 风险 R1: 语义图与地图坐标不对齐

缓解：

- V1 固定 `testmapMain`
- map metadata 从 `yaml + pgm` 自动读取
- 先不引入多地图自动配准

### 风险 R2: 旧接口改坏现有 UI

缓解：

- 保留 `x/y/z` 兼容层
- 新接口先平行上线

### 风险 R3: 任务执行时 revision 被修改

缓解：

- `task_request` 固定 `semantic_revision_id`
- `task_item.snapshot_json` 固定执行快照

### 风险 R4: service pose 不可执行

缓解：

- V1 仅做少量 rack/slot 标定
- 先跑通一条 aisle 的 pick/place

### 风险 R5: 运行态 UI 延迟或丢状态

缓解：

- V1 用 polling
- 状态以数据库 snapshot 为准

---

## 16. Definition of Done

V1 语义地图 overlay 完成的最低标准：

1. `testmapMain` 可在 UI 中作为底图显示
2. 可在 `LocationMapView` 上编辑至少一组 aisle / rack / slot
3. 商品可绑定到 slot，而不是只写 `x/y/z`
4. 后端能通过 `product_id` 解析出：
   - slot
   - nav anchor
   - rack level
   - service pose
5. BT 能从语义目标生成真实 Nav2 goal
6. `NavigateToGoalPose` 不再使用硬编码坐标
7. 至少一条 customer 或 restock 流程接到真实语义目标
8. `SlamMapView` 能显示 live robot pose 和当前目标 slot

---

## 17. 当前建议的第一落地点

如果按工程收益排序，下一步最该做的是：

1. 先写 DB schema 和 API contract
2. 再做 `LocationMapView` 编辑器
3. 再做 `semantic_map_server + BT resolve`

不要一开始就直接去改 BT 或机械臂动作流程，因为当前最大的缺口不是执行器，而是“目标语义真源”。

---

## 18. 与已有文档的关系

本文与现有文档关系如下：

1. 继承 `ROBOT_AUTONOMY_FRONTIER_ADAPTATION_PLAN.md` 里关于 `semantic_overlay` 的导航侧想法
2. 扩展到 UI / backend / PostgreSQL / BT / manipulation 的完整闭环
3. 作为后续语义地图实施的主计划文档

