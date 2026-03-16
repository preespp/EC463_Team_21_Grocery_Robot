# Current System Tech Review

Date: 2026-03-15

Scope:
- This review describes the current code as it exists on 2026-03-15.
- It is intentionally code-faithful, not design-intent-faithful.
- It covers the navigation stack, semantic map layer, task execution path, and the current web/backend integration around Store Map and SLAM viewing.

This document should be read as a current-state companion to the older navigation reviews in `Ref_doc/CN` and `Ref_doc/EN`. Where older reviews describe historical states, this document describes what the code is doing now.

## 0. Executive Summary

The current system is now a multi-layer stack made of:

- ROS 2 navigation and localization built around SICK PicoScan, Cartographer, EKF, and Nav2.
- A semantic overlay layer implemented in ROS and mirrored into the employee web UI.
- A PostgreSQL + Node.js backend serving inventory, employee auth, order APIs, semantic map APIs, and semantic-map version history.
- A Vue-based employee dashboard with:
  - a custom editable Store Map page, and
  - a SLAM page that embeds a vendored ROS web GUI app through an iframe.

The most important current architectural facts are:

- Mapping and localization are both PointCloud2-based Cartographer pipelines.
- Localization uses Cartographer localization plus a static map server, not AMCL.
- Nav2 uses `SmacPlanner2D + MPPIController` with omni motion enabled.
- The semantic map is currently read from YAML as the live source of truth.
- Database semantic versions are history snapshots, not the primary read source.
- The Store Map page can edit the semantic map and save back to both YAML and PostgreSQL version history.
- The SLAM page is not a custom canvas anymore; it is a wrapper around a vendored `ros_web_gui_app` fork that connects to `rosbridge` directly inside the iframe.

The system is functional, but several workflow mismatches remain:

- The default BT executor only polls customer orders, not employee restock tasks.
- The customer BT tree currently navigates but does not update inventory.
- The restock BT tree currently calls the inventory decrement endpoint, which is backward for restocking.
- The robot status page still uses placeholder backend data, not live ROS telemetry.
- The embedded SLAM page needs a separate `rosbridge_server`; it is not routed through the Node backend.

## 1. Codebase Boundary and Current Relevant Packages

### 1.1 ROS packages

Relevant ROS packages in `workspace/src`:

- `robot_navigation`
  - launch composition
  - Cartographer configs
  - Nav2 params
  - serial bridge
  - base-link crop filter
  - semantic map server
  - helper CLI (`nav_assistant`)
- `robot_task_manager`
  - blackboard
  - BT executor
  - BT nodes
  - customer/restock trees
  - alternate ViperX BT path
- `robot_interfaces`
  - `Order`, `OrderItem`
  - `NewOrder.srv`
  - `ResolveSemanticTarget.srv`
  - `MoveRack.action`
  - `PickArm.action`
- `robot_manipulation`
  - MoveIt launch for VX300
  - `pick_arm` action server

### 1.2 Web/backend packages

Relevant web/backend folders:

- `order-api-postgre`
  - Express backend
  - PostgreSQL access
  - semantic map file parsing and save logic
- `order-api-postgre/fleet-manager`
  - employee-facing Vue dashboard
  - Store Map page
  - SLAM page shell
- `third_party/ros_web_gui_app`
  - vendored source from `StarLionJiang/ros_web_gui_app`
  - translated to English for the embedded SLAM view

### 1.3 Map assets and semantic assets

Current map assets:

- `Maps/testmapMain.pbstream`
- `Maps/testmapMain.yaml`
- `Maps/testmapMain.pgm`

Current semantic overlay asset:

- `workspace/src/robot_navigation/config/semantic_map_testmapMain.yaml`

Important current map facts:

- map name: `testmapMain`
- map resolution: `0.03 m/pixel`
- map origin in `map` frame: `[-2.797711, -10.398200, 0.0]`
- semantic map id: `testmapMain`
- semantic home anchor is configured as `(0.0, 0.0, 0.0)`

## 2. Current Navigation Stack

### 2.1 Mapping stack

Current mapping launch entry:

- `ros2 launch robot_navigation slam_mapping_stack.launch.py`
- or `ros2 run robot_navigation nav_assistant mapping-stack`

Current mapping stack composition:

- SICK PicoScan driver via `sick_generic_caller`
- static TF `base_link -> lidar_link`
- static TF `lidar_link -> imu_link`
- `base_link_crop_filter`
- Cartographer mapping
- serial bridge
- optional EKF
- optional ultrasonic collision launch
- optional RViz

Current mapping sensor path:

- LiDAR raw point cloud: `/cloud_all_fields_fullframe`
- cropped point cloud for Cartographer: `/cloud_all_fields_fullframe_filtered`
- IMU topic: `/sick_scansegment_xd/imu`

Current mapping behavior:

- Cartographer mapping consumes the cropped point cloud.
- Cartographer publishes occupancy grid through `cartographer_occupancy_grid_node`.
- The crop filter is enabled by default in mapping.
- The crop filter exists to remove self-hits from the robot body before Cartographer sees the cloud.

Current Cartographer mapping configuration facts:

- `tracking_frame = "imu_link"`
- `published_frame = "base_link"`
- `odom_frame = "odom"`
- `use_odometry = true`
- `use_imu_data = true`
- `num_point_clouds = 1`
- `num_laser_scans = 0`
- `publish_frame_projected_to_2d = true`

This means the current mapping path is not a LaserScan-first pipeline. It is a PointCloud2 + IMU + odom pipeline.

### 2.2 Localization + Nav2 stack

Current localization entry:

- `ros2 launch robot_navigation nav2_localization_stack.launch.py`
- or `ros2 run robot_navigation nav_assistant localization-stack --map-name testmapMain`

Current localization stack composition:

- SICK PicoScan driver
- static TFs
- serial bridge
- optional EKF
- optional crop filter, disabled by default
- Cartographer localization with `pbstream`
- `nav2_map_server` serving `testmapMain.yaml`
- lifecycle manager for map server
- Nav2 bringup delayed by 2 seconds
- optional Nav2 RViz
- semantic map server enabled by default

Important localization behavior:

- Cartographer localization loads the saved `pbstream` state.
- `publish_occupancy_grid` is disabled in localization mode.
- `/map` in localization mode comes from `nav2_map_server`, not from Cartographer.
- Cartographer still provides localization and TF behavior; the static occupancy grid comes from the exported map YAML/PGM.

This is the key distinction:

- mapping mode: Cartographer generates the map
- localization mode: Cartographer localizes against `pbstream`, while `map_server` serves the exported static map

### 2.3 EKF path

Current EKF config:

- input odom: `/odom_raw`
- input IMU: `/sick_scansegment_xd/imu`
- output: `/odom`
- `publish_tf: false`
- `world_frame: odom`
- `two_d_mode: true`

Current effective flow:

`STM32 telemetry -> nav2_serial_bridge -> /odom_raw`

`/odom_raw + /sick_scansegment_xd/imu -> robot_localization EKF -> /odom`

### 2.4 Nav2 stack

Current Nav2 parameter file:

- `workspace/src/robot_navigation/config/nav2_params_smac_mppi_omni.yaml`

Current active planner/controller stack:

- planner: `nav2_smac_planner/SmacPlanner2D`
- controller: `nav2_mppi_controller::MPPIController`
- motion model: `Omni`

Current costmap behavior:

- local costmap:
  - rolling window in `odom`
  - `VoxelLayer + InflationLayer`
  - point cloud source: `/cloud_all_fields_fullframe`
- global costmap:
  - static map in `map`
  - `StaticLayer + ObstacleLayer + InflationLayer`
  - obstacle source: `/cloud_all_fields_fullframe`

Important current behavior:

- Cartographer receives the cropped cloud in mapping and optionally in localization.
- Nav2 costmaps still use the raw point cloud topic, not the cropped topic.
- So the crop filter currently protects Cartographer input, not Nav2 obstacle layers.

### 2.5 Serial bridge and command transport

Current serial bridge:

- node: `robot_navigation/nav2_serial_bridge.py`
- purpose:
  - forward `/cmd_vel`-style commands to STM32 over serial
  - decode telemetry into odometry
  - optionally publish fallback odom if telemetry stalls

Current default bridged topics:

- mapping stack: `["/cmd_vel", "/cmd_vel_nav", "/cmd_vel_smoothed"]`
- localization stack: `["/cmd_vel"]`

Current authority model:

- there is no command arbiter yet
- whichever subscribed command arrives last can win
- older planning documents mention a future arbiter, but it is not implemented in the current code path

## 3. Current Semantic Map Stack

### 3.1 Semantic map representation

The semantic map is currently a manually curated YAML overlay, not something extracted automatically from RViz or SLAM.

Current file:

- `workspace/src/robot_navigation/config/semantic_map_testmapMain.yaml`

Current structure:

- `map`
- `anchors`
- `racks`
- `slots`

Current semantic content:

- anchors:
  - `home`
  - `drinks_anchor`
  - `fruits_anchor`
  - `snacks_anchor`
- racks:
  - one rack per anchor area
- slots:
  - 7 product slots
  - linked to current seeded inventory product IDs 1 through 7

Important current truth:

- this semantic map is hand-authored
- it is not automatically generated from RViz
- it is not pulled from PostgreSQL as the live source
- it is currently tailored around the seeded store example

### 3.2 ROS semantic map server

Current semantic ROS node:

- `robot_navigation/semantic_map_server.py`

Current responsibilities:

- load semantic YAML
- publish marker overlays
- provide a semantic resolution service

Current interface:

- markers: `/semantic_map/markers`
- service: `/semantic_map/resolve_target`

Current resolution order:

1. explicit `slot_id`
2. explicit `anchor_id`
3. `product_id`
4. `product_name`

### 3.3 Semantic map in the web stack

The semantic map is now visible and editable in the employee UI, but the web path is different from the ROS path.

Current backend semantic behavior:

- read live semantic content from YAML
- enrich slots with inventory metadata from PostgreSQL
- expose the bundle through REST
- save edits back to YAML
- snapshot each save into PostgreSQL version history

Current crucial implementation fact:

- YAML is still the primary live source
- `semantic_map_versions` is version history only
- editing the DB directly does not change the currently served semantic map unless YAML is also updated

## 4. Current Task Execution and BT Flow

### 4.1 Default BT executor

Current default executor:

- `workspace/src/robot_task_manager/robot_task_manager/bt_executor.py`

Current behavior:

- offers `/order/new` service
- polls the Node backend every second
- only polls `GET /api/order/latest`
- builds either a customer or employee tree based on `order.role`
- ticks the tree at 10 Hz

Current order intake source:

- automatic polling only supports customer orders from `order_id`
- employee restock requests stored in `restock_id` are not polled by the default executor

This is an important current limitation: the employee dashboard can submit restock requests, but the default executor does not fetch them.

### 4.2 Blackboard state

Current blackboard includes:

- order context
- current item context
- arm placeholders
- rack state
- navigation goals
- semantic identifiers

Important semantic fields now present:

- `slot_id`
- `anchor_id`
- `rack_id`
- `semantic_id`
- `semantic_target_label`
- `nav_goal_source`
- `nav_goal`
- `home_goal`
- `shelf_pose`

Current configured home navigation goal:

- `(0.0, 0.0, 0.0)`

This is a blackboard default, not proof that localization automatically initializes the robot at `(0,0,0)`.

### 4.3 Customer tree

Current default customer tree:

1. `SetCurrentItem`
2. `ResolveCurrentItemSemanticTarget`
3. `NavigateToGoalPose(nav_goal)`
4. repeat for each item
5. `SetHome`
6. `NavigateToGoalPose(nav_goal)`

Current actual behavior:

- the tree resolves item targets through semantic map first
- if semantic resolution fails, it falls back to legacy `x/y/z`
- the tree currently does not run manipulation
- the tree currently does not call `ChangeInventory`

This means the current customer tree is effectively a navigation-only tree.

### 4.4 Restock tree

Current default restock tree:

1. `SetCurrentItem`
2. `ResolveCurrentItemSemanticTarget`
3. `NavigateToGoalPose(nav_goal)`
4. `ChangeInventory`
5. repeat
6. `SetHome`
7. `NavigateToGoalPose(nav_goal)`

Current actual behavior:

- manipulation steps are still commented out
- inventory update is active

But there is a critical code-faithful issue:

- `ChangeInventory` always calls `/api/inventory/decrement`
- it does not branch by `mode`
- so the current restock tree decrements stock instead of increasing it

This is not a theoretical concern. It is the current code behavior.

### 4.5 Semantic target resolution node

Current BT semantic node:

- `ResolveCurrentItemSemanticTarget`

Current behavior:

- build request from current item `product_id` and `name`
- call `/semantic_map/resolve_target`
- write resolved data into the blackboard
- if unavailable or unresolved, keep running by falling back to legacy coordinates

Current fallback path:

- `SetCurrentItem` first writes:
  - `bb.nav_goal = (x, y, 0.0)` using legacy order item fields
  - `bb.rack_goal = z`
- semantic node can overwrite these with semantic targets
- if it cannot, the legacy values remain usable

### 4.6 NavigateToPose integration

Current navigation BT leaf:

- `NavigateToGoalPose`

Current behavior:

- reads `bb.nav_goal`
- accepts tuple/dict/PoseStamped
- sends `NavigateToPose` action goal to Nav2
- no longer uses the old hard-coded `(2.0, 0.0)` demo target

### 4.7 Completion reporting behavior

Current BT completion behavior:

- when the tree finishes, executor posts to `/api/order/complete`

Current backend completion behavior:

- `/api/order/complete` only updates the `order_id` table

Therefore:

- customer completion round-trip works at the table level
- employee/restock completion does not currently have a matching table update path through this endpoint

### 4.8 Alternate ViperX path

There is now a separate ViperX-specific BT path:

- `bt_executor_viperx.py`
- `trees/customer_viperx.py`
- `trees/restock_viperx.py`
- `bt_nodes/viperx_nodes.py`

Current state of that path:

- present in code
- not the main default path described above
- still largely commented or incomplete
- not currently integrated into the employee web workflow

## 5. Current Manipulation Stack

The codebase now contains a real VX300-side manipulation backend, but the default BT path still does not fully use it.

Current implemented manipulation backend:

- `robot_manipulation/src/arm_controller.cpp`
- launch: `robot_manipulation/launch/vx300_moveit.launch.py`
- action: `robot_interfaces/action/PickArm.action`

Current manipulation backend capabilities:

- MoveIt planning
- action server `pick_arm`
- pre-grasp / approach / retreat logic
- optional Cartesian approach

Current BT-side status:

- generic arm BT node remains TODO
- default customer/restock trees still have most manipulation steps commented out

So the repository now has:

- a stronger backend manipulation implementation than before
- but not a fully wired end-to-end default manipulation workflow

## 6. Backend and Database Review

### 6.1 Backend structure

Current backend:

- Express
- body-parser
- cors
- PostgreSQL via `pg`
- semantic map file parsing/saving in `semantic_map.js`

Current backend startup behavior:

- create `semantic_map_versions` if needed
- migrate legacy semantic IDs like `testmapMain_v1` into `testmapMain`
- bootstrap a version snapshot from the current YAML if no history exists
- start HTTP server on port `3000`

### 6.2 Current database model

Current tables from `001_schema.sql`:

- `customer`
- `employee`
- `inventory`
- `order_id`
- `restock_id`
- `semantic_map_versions`

Current important reality:

- `inventory` still contains legacy `x`, `y`, `z`
- customer orders still persist those legacy position values into order JSON
- semantic map is not yet normalized into relational tables like racks/slots/placements
- semantic history is stored as version snapshots, not decomposed semantic entities

### 6.3 Current semantic map API behavior

Current semantic endpoints:

- `GET /api/maps/base/:mapName.pgm`
- `GET /api/maps/semantic/current`
- `PUT /api/maps/semantic/current`

Current auth requirement:

- semantic endpoints require employee auth except the raw base-map file route

Current `GET /api/maps/semantic/current` behavior:

- read YAML bundle from disk
- read static map metadata from `Maps/testmapMain.yaml`
- parse PGM size from `Maps/testmapMain.pgm`
- attach current version info from `semantic_map_versions`
- join slot product IDs against `inventory`
- return enriched JSON bundle

Current `PUT /api/maps/semantic/current` behavior:

- accept bundle from Store Map UI
- save YAML to disk
- record version snapshot in DB
- if DB version write fails, restore the previous YAML text

That rollback logic is important: the code tries to keep YAML and DB history consistent for UI-driven saves.

### 6.4 Auth and session model

Current employee auth flow:

- `POST /api/employee/auth/login`
- compare plain-text password from DB
- return bearer token
- store token in in-memory session map with 12-hour TTL

Current limitations:

- tokens are process-memory only
- restarting the backend invalidates sessions
- passwords are plain text in the current schema and seed data

### 6.5 Order and restock endpoints

Current order endpoints used by robot path:

- `GET /api/order/latest`
- `POST /api/order/ack`
- `POST /api/order/complete`

Current behavior:

- these endpoints operate on `order_id`
- they do not provide an equivalent automatic fetch/ack/complete path for `restock_id`

Current employee UI endpoint:

- `POST /api/employee/restock/submit`

Current behavior:

- writes a restock request to `restock_id`
- does not connect to the default executor polling path

This means the employee restock dashboard and the default robot polling path are currently not aligned end-to-end.

### 6.6 Placeholder robot status

Current robot status endpoint:

- `GET /api/employee/robot/status`

Current behavior:

- returns three hard-coded robots
- uses semantic anchor poses for display positions when available
- does not subscribe to ROS
- is a placeholder dashboard source

## 7. Frontend Review and Frontend/Backend Connections

### 7.1 Frontend structure

Current employee dashboard:

- Vue 3
- Vue Router
- Vite

Current authenticated routes:

- `/app/inventory-report`
- `/app/location-map`
- `/app/slam-map`
- `/app/robot-status`
- `/app/restock`
- `/app/employee-accounts`

Current auth guard:

- route guard checks `fleet_token` in `localStorage`

### 7.2 Login flow

Current login flow:

1. employee enters `employee_ID` and password
2. frontend calls `POST /api/employee/auth/login`
3. token and employee info are written to `localStorage`
4. router sends user to `/app/inventory-report`

### 7.3 Inventory report page

Current page:

- `InventoryReportView.vue`

Current backend contract:

- `GET /api/employee/inventory/report`

Current behavior:

- render summary cards
- render category breakdown
- render raw inventory table including legacy `x/y/z`

This page still exposes the legacy location model even though semantic mapping now exists.

### 7.4 Store Map page

Current page:

- `LocationMapView.vue`

Current canvas:

- `SemanticMapCanvas.vue`

Current behavior:

- fetch semantic map bundle from backend
- parse and render the base PGM raster
- auto-fit the content into the viewport
- support zoom in/out and pan
- support edit mode
- drag anchors, racks, and slots
- allow manual numeric pose editing
- save back to backend

Current save flow:

1. employee logs in and gets bearer token
2. page calls `GET /api/maps/semantic/current`
3. backend serves YAML-derived bundle with DB version attached
4. user edits geometry locally in the page
5. page calls `PUT /api/maps/semantic/current`
6. backend writes YAML and records semantic version snapshot
7. page reloads the saved bundle and shows new version info

Current Store Map data source truth:

- live geometry source: YAML
- version history source: PostgreSQL

### 7.5 SLAM page

Current page:

- `SlamMapView.vue`

Current implementation:

- not Foxglove
- not a custom `/map + /tf` canvas anymore
- it embeds a child application through an iframe

Current embedded app source:

- vendored fork: `third_party/ros_web_gui_app`
- source repository recorded as `https://github.com/StarLionJiang/ros_web_gui_app`

Current runtime model:

- main Vue app serves `/embedded/ros-web-gui/index.html`
- iframe loads the embedded app
- embedded app manages its own `rosbridge` websocket connection
- default bridge target shown in UI is `ws://<host>:9090`

Current consequence:

- the Node backend is not in the live SLAM transport path
- the SLAM page does not proxy ROS topics through Express
- live ROS visualization depends on a separately running `rosbridge_server`

### 7.6 Robot status page

Current page:

- `RobotStatusView.vue`

Current backend contract:

- `GET /api/employee/robot/status`

Current behavior:

- display dashboard cards
- no live ROS subscription
- status is backend-generated placeholder data

### 7.7 Restock and accounts pages

Current restock page:

- loads products from `GET /api/employee/inventory/options`
- submits to `POST /api/employee/restock/submit`

Current employee accounts page:

- reads from `GET /api/employee/accounts`
- writes to `POST /api/employee/accounts/create`

These pages are fully backend-connected, but they are not deeply integrated with the ROS execution path yet.

## 8. Information Flow Review

### 8.1 Sensor to map flow

Current mapping information flow:

`SICK PicoScan point cloud -> /cloud_all_fields_fullframe`

`/cloud_all_fields_fullframe -> base_link_crop_filter -> /cloud_all_fields_fullframe_filtered`

`/cloud_all_fields_fullframe_filtered + /sick_scansegment_xd/imu + /odom -> Cartographer mapping`

`Cartographer -> live occupancy grid -> mapping visualization/export path`

### 8.2 Localization to planner flow

Current localization information flow:

`STM32 telemetry -> nav2_serial_bridge -> /odom_raw`

`/odom_raw + IMU -> EKF -> /odom`

`point cloud + IMU + /odom + pbstream -> Cartographer localization`

`map_server -> /map`

`Nav2 planner/controller -> NavigateToPose action execution`

### 8.3 Customer order to robot flow

Current customer information flow:

`customer order saved in order_id`

`BT executor polls /api/order/latest`

`backend returns items with legacy x/y/z and product metadata`

`BT converts JSON -> Order / OrderItem`

`SetCurrentItem writes legacy nav goal`

`ResolveCurrentItemSemanticTarget tries to resolve product -> slot/service pose`

`NavigateToGoalPose sends Nav2 goal`

`tree returns home`

What does not currently happen in the default customer flow:

- no pick action
- no gripper action
- no rack action
- no inventory decrement

### 8.4 Semantic edit flow

Current semantic edit flow:

`employee login -> bearer token`

`LocationMapView -> GET /api/maps/semantic/current`

`backend -> YAML bundle + map metadata + DB version + inventory enrichment`

`user edits geometry`

`LocationMapView -> PUT /api/maps/semantic/current`

`backend -> save YAML -> insert semantic_map_versions snapshot`

### 8.5 SLAM web visualization flow

Current SLAM web flow:

`browser opens /app/slam-map`

`SlamMapView loads iframe /embedded/ros-web-gui/index.html`

`embedded app connects directly to rosbridge`

`embedded app renders ROS topics`

This means the live SLAM page bypasses the Express backend for real-time ROS transport.

## 9. Current Technical Stack Inventory

ROS/navigation stack:

- ROS 2 Humble-style launch and nodes
- SICK `sick_scan_xd`
- Cartographer ROS
- Nav2
- robot_localization
- tf2
- custom point-cloud crop filter
- custom serial bridge

Task stack:

- `py_trees`
- Python BT executor
- custom semantic service client node
- custom Nav2 action BT leaf

Manipulation stack:

- MoveIt
- custom `pick_arm` action server
- Interbotix/ViperX-related experimental nodes

Backend stack:

- Node.js
- Express
- PostgreSQL
- `pg`
- optional Python `deep-translator` helper for inventory translation

Frontend stack:

- Vue 3
- Vue Router
- Vite
- custom Store Map canvas component
- embedded vendored React/Three.js ROS web GUI

Embedded SLAM viewer stack:

- vendored `ros_web_gui_app`
- React
- Three.js
- `roslib`
- `rosbridge_server`

## 10. Current Gaps, Risks, and Mismatches

The most important current code-faithful gaps are:

1. Restock UI to robot execution is incomplete.
   - UI writes `restock_id`.
   - default executor only polls `order_id`.

2. Inventory update semantics are inconsistent.
   - customer tree does not update inventory
   - restock tree decrements inventory

3. Robot status page is not live ROS.
   - it is placeholder dashboard data

4. Semantic map source-of-truth remains split.
   - YAML is live source
   - DB is version history
   - direct DB edits do not drive the current UI bundle

5. Manipulation is only partially wired.
   - backend action server exists
   - default BT path still does not use it end-to-end

6. Embedded SLAM page depends on external runtime setup.
   - requires `rosbridge_server`
   - embedded app connection is not brokered by the Node backend

7. Nav2 command authority is still simple topic subscription.
   - no implemented arbiter between manual and autonomous command sources

## 11. Operational Notes

Current recommended stack entrypoints:

- mapping:
  - `ros2 run robot_navigation nav_assistant mapping-stack`
- localization + Nav2:
  - `ros2 run robot_navigation nav_assistant localization-stack --map-name testmapMain --with-nav2-rviz true`
- backend:
  - `cd order-api-postgre && npm run dev`
- employee UI:
  - `cd order-api-postgre/fleet-manager && npm run dev`
- SLAM page live bridge:
  - `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`

Current employee test login:

- `000AAA / team21`

## 12. Bottom Line

The repository now contains a real, multi-surface navigation product:

- a usable mapping/localization/Nav2 stack,
- a semantic overlay layer in both ROS and web UI,
- editable store-map tooling with YAML + DB version persistence,
- and a browser-based SLAM viewer built around a vendored ROS GUI app.

But the end-to-end autonomy story is still only partially closed.

The navigation stack itself is substantially more real than before. The weak points are now mostly in workflow integration:

- customer and restock execution semantics,
- inventory side effects,
- live robot status in the dashboard,
- and full manipulation wiring from BT to arm execution.

## 13. Detailed Launch, Node, Topic, Service, and Action Inventory

### 13.1 Launch file inventory

| Launch file | Current purpose | Included nodes / includes | Important defaults |
| --- | --- | --- | --- |
| `launch/slam_mapping_stack.launch.py` | full mapping stack | `sick_generic_caller`, 2 static TF publishers, `base_link_crop_filter`, `cartographer_mapping.launch.py`, `nav2_serial_bridge`, optional `ekf_node`, optional collision launch, optional RViz | crop enabled, `cmd_topics=["/cmd_vel","/cmd_vel_nav","/cmd_vel_smoothed"]`, `odom_topic=/odom_raw` |
| `launch/cartographer_mapping.launch.py` | pure Cartographer mapping core | `cartographer_node`, `cartographer_occupancy_grid_node` | `configuration_basename=pico_2d_mapping_quality.lua`, `resolution=0.03` |
| `launch/nav2_localization_stack.launch.py` | localization + Nav2 + semantic map | `sick_generic_caller`, 2 static TF publishers, `nav2_serial_bridge`, optional `ekf_node`, optional `base_link_crop_filter`, `cartographer_localization.launch.py`, `map_server`, map lifecycle manager, delayed `nav2_bringup/navigation_launch.py`, optional Nav2 RViz, optional `semantic_map_server` | default map `testmapMain`, crop disabled, semantic map enabled, `cmd_topics=["/cmd_vel"]` |
| `launch/cartographer_localization.launch.py` | Cartographer localization core | `cartographer_node`, optional `cartographer_occupancy_grid_node` | `configuration_basename=pico_2d_localization.lua`, `load_frozen_state=true`, local occupancy grid off by default |

### 13.2 Runtime executable inventory

#### `robot_navigation` executables

| Executable | Role | Publishes | Subscribes | Services / actions | Current status |
| --- | --- | --- | --- | --- | --- |
| `nav2_serial_bridge` | STM32 command + telemetry bridge | `nav_msgs/Odometry` on configured `odom_topic`, optional TF | configured `cmd_topics`, `/front_alert`, `/back_alert`, `/left_alert`, `/right_alert` | none | active in mapping and localization |
| `base_link_crop_filter` | remove chassis self-hits from point cloud | filtered `PointCloud2` | raw `PointCloud2` | none | active in mapping by default |
| `semantic_map_server` | semantic overlay publisher + semantic target resolver | `/semantic_map/markers` | none | `/semantic_map/resolve_target` | active in localization by default |
| `nav_assistant` | CLI helper for launch, save-map, export-map, goal, teleop macros | transient `Twist` in motion modes | none persistent | uses `WriteState`, `NavigateToPose`, `FollowWaypoints` clients | operator tool, not a persistent runtime node |
| `teleop_cmd_vel` | simple keyboard teleop | `Twist` | none | none | utility |
| `teleop_cmd_vel_collision` | keyboard teleop with one collision stop input | `Twist` | `right_alert` | none | utility, not part of launch stacks |
| `teleop_ros` | older configurable teleop publisher | `Twist` | none | none | legacy utility |
| `wheel_motor` | older I2C mecanum driver path | none | configurable `cmd_vel`, `obstacle_alert` | none | legacy path, not used by current Nav2 stack |

#### `robot_task_manager` executables

| Executable | Role | ROS interfaces | External interfaces | Current status |
| --- | --- | --- | --- | --- |
| `bt_executor` | default customer / employee BT executor | `/order/new` service; Nav2 action client inside BT; semantic resolve client inside BT | polls backend `GET /api/order/latest`; posts `/api/order/ack`; posts `/api/order/complete`; `ChangeInventory` posts `/api/inventory/decrement` | current default path |
| `bt_executor_viperX` | alternate ViperX-focused executor | same `/order/new` service pattern; ViperX-specific BT nodes | same backend polling pattern as default executor | present, but not the main workflow path |

#### `robot_manipulation` executables

| Executable | Role | Interfaces | Current status |
| --- | --- | --- | --- |
| `arm_controller` | MoveIt-backed arm action server | `PickArm` action, `JointTrajectory` publisher | implemented and usable |
| `viperx_arm_server` | MoveIt-backed ViperX arm action server | `PickArm` action on default `pick_viperx` | implemented |
| `arm_waypoint_server` | waypoint-style arm action server | `PickArm` action on default `pick_arm_waypoint` | implemented |
| `arm_demo_controller` | simplified arm demo action server | `PickArm` action on default `pick_arm_demo` | implemented for demo/testing |
| `rack_controller` | rack lift action server | `MoveRack` action on `move_rack` | implemented |
| `arm_motor` | I2C hardware bridge | `JointTrajectory` subscriber | implemented |
| `arm_to_gazebo` | simulation bridge | publishers/subscribers for Gazebo integration | implemented |
| `vx300_quick_move.py` | diagnostic/helper script | publishers + service clients | utility |
| `vx300_hardware_diagnostics.py` | diagnostic/helper script | publishers + service clients + subscriptions | utility |

### 13.3 ROS topic inventory by current main workflow

#### Navigation and localization topics

| Topic | Type | Producer | Primary consumers | Notes |
| --- | --- | --- | --- | --- |
| `/cloud_all_fields_fullframe` | `sensor_msgs/PointCloud2` | SICK driver | crop filter, Nav2 costmaps | raw point cloud |
| `/cloud_all_fields_fullframe_filtered` | `sensor_msgs/PointCloud2` | crop filter | Cartographer mapping/localization | current Cartographer input |
| `/scan_fullframe` | `sensor_msgs/LaserScan` | SICK driver | currently not the main Cartographer path | still available |
| `/sick_scansegment_xd/imu` | IMU | SICK driver | EKF, Cartographer | current IMU path |
| `/odom_raw` | `nav_msgs/Odometry` | `nav2_serial_bridge` | EKF | raw base odom |
| `/odom` | `nav_msgs/Odometry` | EKF | Nav2, Cartographer localization | filtered odom |
| `/map` | `nav_msgs/OccupancyGrid` | Cartographer in mapping; `map_server` in localization | RViz, Nav2 global stack, embedded ROS GUI | source changes by mode |
| `/cmd_vel` | `geometry_msgs/Twist` | teleop or Nav2 path | serial bridge | localization stack bridge listens here by default |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | optional nav path | serial bridge in mapping only by default | not listened to by localization stack default |
| `/cmd_vel_smoothed` | `geometry_msgs/Twist` | optional smoother path | serial bridge in mapping only by default | same note as above |
| `/semantic_map/markers` | `visualization_msgs/MarkerArray` | semantic map server | RViz | semantic overlay display |

#### Collision / safety topics currently wired into the bridge

| Topic | Type | Consumer | Effect |
| --- | --- | --- | --- |
| `/front_alert` | `std_msgs/Bool` | `nav2_serial_bridge` | blocks forward motion when true |
| `/back_alert` | `std_msgs/Bool` | `nav2_serial_bridge` | blocks backward motion when true |
| `/left_alert` | `std_msgs/Bool` | `nav2_serial_bridge` | blocks left motion when true |
| `/right_alert` | `std_msgs/Bool` | `nav2_serial_bridge`, `teleop_cmd_vel_collision` | blocks right motion / teleop output |

### 13.4 ROS service inventory

| Service | Type | Server | Current purpose |
| --- | --- | --- | --- |
| `/semantic_map/resolve_target` | `robot_interfaces/srv/ResolveSemanticTarget` | `semantic_map_server` | resolve product / slot / anchor into nav and service poses |
| `/order/new` | `robot_interfaces/srv/NewOrder` | `bt_executor` or `bt_executor_viperX` | manual ROS-side order injection |
| `/write_state` | `cartographer_ros_msgs/srv/WriteState` | Cartographer | save pbstream through `nav_assistant save-map` |

### 13.5 ROS action inventory

| Action | Type | Server | Client(s) | Current use |
| --- | --- | --- | --- | --- |
| `/navigate_to_pose` or `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 | `NavigateToGoalPose`, `nav_assistant goal` | active |
| `/follow_waypoints` | `nav2_msgs/action/FollowWaypoints` | Nav2 | `nav_assistant waypoints` | utility |
| `pick_arm` | `robot_interfaces/action/PickArm` | `arm_controller` by default | not used by default BT path yet | implemented backend |
| `pick_viperx` | `robot_interfaces/action/PickArm` | `viperx_arm_server` default | ViperX BT nodes | alternate path |
| `move_rack` | `robot_interfaces/action/MoveRack` | `rack_controller` | rack BT node | backend exists; default tree path still commented |

Important implementation note:

- `robot_task_manager/bt_nodes/rack_nodes.py` still imports `rack_interfaces.action.MoveRack`, but the actual interface in this repo is `robot_interfaces/action/MoveRack`. That BT node is also not part of the current default tree import path, so this mismatch is currently dormant rather than runtime-active.

## 14. Full Backend and Frontend API Contract Table

### 14.1 Browser-to-backend transport boundary

Current frontend transport model:

- Vite dev server runs on `5174`.
- Vite proxies `/api` to `http://localhost:3000`.
- Static embedded ROS GUI assets are served from frontend public files under `/embedded/ros-web-gui/...`.
- The SLAM iframe does not call backend APIs for live ROS data; it connects to `rosbridge` directly.

### 14.2 Endpoints currently used by the fleet UI

| Method | Path | Auth | Current caller | Request shape | Response shape | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| `POST` | `/api/employee/auth/login` | no | `LoginView` | `{ employee_ID, password }` | `{ ok, token, employee }` or error | bearer token source |
| `GET` | `/api/employee/inventory/report` | employee | `InventoryReportView` | none | `{ summary, category_summary, items }` | still includes legacy `x/y/z` |
| `GET` | `/api/maps/semantic/current` | employee | `LocationMapView` | none | semantic bundle with `map`, `anchors`, `racks`, `slots`, `summary`, `version`, `generated_at` | YAML-derived |
| `PUT` | `/api/maps/semantic/current` | employee | `LocationMapView` | `{ bundle, change_summary }` | `{ ok, bundle, saved_at }` | writes YAML + DB version |
| `GET` | `/api/employee/robot/status` | employee | `RobotStatusView` | none | `{ robots: [...] }` | placeholder data |
| `GET` | `/api/employee/inventory/options` | employee | `RestockView` | none | `{ items: [...] }` | product dropdown |
| `POST` | `/api/employee/restock/submit` | employee | `RestockView` | `{ items: [{ product_id, qty }] }` | `{ status, restock_ID }` | writes `restock_id` only |
| `GET` | `/api/employee/accounts` | employee | `EmployeeAccountsView` | none | `{ items: [...] }` | employee list |
| `POST` | `/api/employee/accounts/create` | employee | `EmployeeAccountsView` | `{ employee_ID, first_name, last_name, password }` | `{ ok, employee_ID, first_name, last_name }` | plain-text password persistence |

### 14.3 Robot-facing backend endpoints

| Method | Path | Auth | Current caller | Request shape | Response shape | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| `GET` | `/api/order/latest` | no | `bt_executor`, `bt_executor_viperX` | none | `204` or `{ order_id, role, requester_id, items }` | only polls `order_id`, not `restock_id` |
| `POST` | `/api/order/ack` | no | executors | `{ order_id }` | `{ ok }` | sets `order_id.status='IN_PROGRESS'` |
| `POST` | `/api/order/complete` | no | executors | `{ order_id, result }` | `{ ok }` | updates `order_id`, not `restock_id` |
| `POST` | `/api/inventory/decrement` | no | `ChangeInventory` BT node | `{ product_id, qty }` | `{ ok }` | currently used by restock tree too |

### 14.4 Public / customer endpoints

| Method | Path | Auth | Request shape | Response shape | Notes |
| --- | --- | --- | --- | --- | --- |
| `POST` | `/api/auth/login` | no | `{ member_ID }` | customer identity block | member login, no password |
| `POST` | `/api/account/create_customer` | no | `{ first_name, last_name }` | `{ ok, member_ID, first_name, last_name }` | auto-generates member ID |
| `GET` | `/api/inventory/list` | no | query `lang` optional | `{ items }` | can translate with `deep-translator` |
| `POST` | `/api/order/customer` | no | `{ member_ID or id, guest, items }` | `{ status, order_id }` | current customer order creation path |
| `GET` | `/api/order/status/:id` | no | path ID | `{ order_id, status, result, timestamp }` | `/api` version |

### 14.5 Semantic and map asset endpoints

| Method | Path | Auth | Request | Response | Notes |
| --- | --- | --- | --- | --- | --- |
| `GET` | `/api/maps/base/:mapName.pgm` | no | path map name | PGM file bytes | base raster for Store Map |
| `GET` | `/api/maps/semantic/current` | employee | none | semantic bundle | main Store Map source |
| `PUT` | `/api/maps/semantic/current` | employee | semantic bundle | saved bundle + version | two-way UI save path |

### 14.6 Legacy compatibility endpoints still present

These endpoints are still implemented in `server.js`, but they are not the main fleet-manager contract:

| Method | Path | Current role |
| --- | --- | --- |
| `POST` | `/order/customer/new` | older customer order create path |
| `POST` | `/order/employee/new` | older employee restock create path |
| `GET` | `/inventory/list/all` | raw inventory list |
| `GET` | `/inventory/list/category` | category-filtered inventory list |
| `POST` | `/inventory/add` | add inventory row |
| `POST` | `/inventory/update` | update inventory row |
| `POST` | `/inventory/delete` | delete inventory row |
| `GET` | `/order/status/:id` | older order-status path |
| `POST` | `/order/complete` | older completion path |

## 15. Textual Sequence Flows

### 15.1 Employee auth flow

1. Browser opens `/login`.
2. User enters `employee_ID` and `password`.
3. `LoginView` calls `POST /api/employee/auth/login`.
4. Backend verifies the row in table `employee`.
5. Backend creates an in-memory session token with 12-hour TTL.
6. Frontend stores `fleet_token`, `fleet_employee_id`, `fleet_first_name`, and `fleet_last_name` in `localStorage`.
7. Vue Router guard allows navigation into `/app/...` routes.

Failure boundary:

- if backend restarts, in-memory employee sessions disappear, but browser `localStorage` still holds the old token until the next request fails with `401`

### 15.2 Store Map load flow

1. User enters `/app/location-map`.
2. Browser sends `GET /api/maps/semantic/current` with bearer token.
3. Backend reads:
   - semantic YAML from `workspace/src/robot_navigation/config/semantic_map_testmapMain.yaml`
   - map YAML from `Maps/testmapMain.yaml`
   - map raster header from `Maps/testmapMain.pgm`
   - latest version row from `semantic_map_versions`
   - inventory rows for slot product IDs
4. Backend returns one enriched bundle.
5. `LocationMapView` clones the bundle into:
   - `savedBundle`
   - editable `bundle`
6. `SemanticMapCanvas` fetches the base PGM through `/api/maps/base/testmapMain.pgm`.
7. Canvas converts map coordinates to pixels using:
   - `px = (x - origin_x) / resolution`
   - `py = height_px - (y - origin_y) / resolution`
8. Canvas auto-fits either content or full-map bounds into the viewport.

### 15.3 Store Map save flow

1. User drags or numerically edits anchor / rack / slot geometry.
2. Local page state marks the bundle as dirty.
3. User presses `Save`.
4. Frontend calls `PUT /api/maps/semantic/current` with `{ bundle, change_summary }`.
5. Backend reads current YAML text as rollback backup.
6. Backend serializes the submitted bundle back to YAML and writes it to disk.
7. Backend inserts a new row into `semantic_map_versions`.
8. If DB insert fails, backend writes the previous YAML text back to disk.
9. Backend responds with the normalized saved bundle plus version info.
10. Frontend replaces local draft state with the saved bundle and clears dirty state.

### 15.4 SLAM page load flow

1. User opens `/app/slam-map`.
2. `SlamMapView` loads iframe source `/embedded/ros-web-gui/index.html`.
3. The iframe app is served from frontend static assets, not backend APIs.
4. The embedded ROS GUI app initializes its own ROS connection UI.
5. User connects the embedded app to `ws://<host>:9090`.
6. `rosbridge_server` becomes the live transport for ROS topics, TF, map, scans, and overlays.

Important boundary:

- the main Vue app does not mediate live ROS traffic for the SLAM page

### 15.5 Restock submit flow

1. Employee opens `/app/restock`.
2. Frontend loads dropdown options from `/api/employee/inventory/options`.
3. Employee adds selected product IDs and quantities to a local cart.
4. Frontend submits `POST /api/employee/restock/submit`.
5. Backend validates product IDs and quantities.
6. Backend writes one new `restock_id` row and stores the JSON items payload.
7. Backend updates `employee.restock_ID`.
8. Frontend shows the generated `restock_ID`.

Current break in the larger workflow:

- default `bt_executor` does not poll `restock_id`, so the submission stops at database persistence unless another consumer is added

## 16. Parameter Review and Explanations

### 16.1 Cartographer mapping and localization parameters

Current mapping file:

- `config/pico_2d_mapping_quality.lua`

Current localization file:

- `config/pico_2d_localization.lua`

Important parameter groups and what they mean in the current code:

| Parameter / group | Current value direction | Practical meaning |
| --- | --- | --- |
| `map_frame = "map"` | fixed | global map frame |
| `tracking_frame = "imu_link"` | fixed | Cartographer tracks the IMU frame rather than `base_link` |
| `published_frame = "base_link"` | fixed | downstream consumers still see planar robot pose in `base_link` |
| `odom_frame = "odom"` | fixed | odom frame aligns with EKF output chain |
| `provide_odom_frame = true` | enabled | Cartographer can maintain map/odom relationship |
| `publish_frame_projected_to_2d = true` | enabled | planar navigation consumers receive 2D-projected robot pose |
| `use_odometry = true` | enabled | Cartographer uses odom as a motion prior |
| `use_imu_data = true` | enabled | IMU contributes orientation stability |
| `num_point_clouds = 1` | enabled | PointCloud2 is the intended live sensor path |
| `num_laser_scans = 0` | disabled | LaserScan is not the main mapping/localization input |
| `submaps.num_range_data = 60` | moderate / quality-biased | each submap integrates 60 range observations before rolling forward |
| `grid_options_2d.resolution = 0.03` | fixed | aligns with exported map resolution |
| `min_range = 0.15`, `max_range = 10.0` | broad | accepts close returns and full aisle-range returns |
| `missing_data_ray_length = 10.0` | broad | unknown-space ray extension equals max usable range |
| `use_online_correlative_scan_matching = true` | enabled | stronger local scan matching at higher compute cost |
| `linear_search_window = 0.2` | moderate | local translational search window |
| `translation_delta_cost_weight = 0.1` | low penalty | allows search freedom for translation |
| `rotation_delta_cost_weight = 0.1` | low penalty | allows rotation search freedom too |
| `motion_filter.*` in mapping | strict small motion thresholds | avoids inserting nearly duplicate observations |
| `pure_localization_trimmer.max_submaps_to_keep = 3` in localization | enabled | keeps localization memory bounded against loaded map state |
| `constraint_builder.min_score = 0.62` | moderately selective | rejects weak loop/constraint matches |
| `odometry_translation_weight = 1e3`, `odometry_rotation_weight = 1e3` | high | gives odometry strong influence during optimization |

Current practical interpretation:

- the stack is biased toward stable warehouse/store localization using odom + IMU + cloud together
- it is not configured as a light-weight scan-only setup

### 16.2 EKF parameter explanation

Current EKF file:

- `config/ekf_odom_base_imu.yaml`

Important parameter groups:

| Parameter / group | Current behavior | Practical effect |
| --- | --- | --- |
| `frequency = 50.0` | high | EKF publishes at 50 Hz |
| `sensor_timeout = 0.3` | moderate | stale inputs are dropped relatively quickly |
| `two_d_mode = true` | enabled | suppresses 3D motion states for planar robot use |
| `publish_tf = false` | disabled | EKF does not become another TF authority |
| `world_frame = odom` | fixed | filtered state remains odom-relative |
| `odom0 = /odom_raw` | active | raw bridge odom is the motion input |
| `odom0_config` | twist-dominant selection | odom contributes planar velocities and yaw rate rather than full absolute pose trust |
| `imu0 = /sick_scansegment_xd/imu` | active | IMU yaw + yaw rate are fused |
| `imu0_remove_gravitational_acceleration = false` | unchanged | raw IMU preprocessing left as-is |
| `imu0_config` | yaw + yaw rate only | IMU is used mainly for heading stabilization |
| `process_noise_covariance` | tuned nonzero matrix | determines smoothing vs responsiveness tradeoff |

Current practical interpretation:

- EKF is acting as a planar motion filter on top of bridge odom and IMU heading data
- it is not being used as a full 3D inertial fusion stack

### 16.3 Nav2 parameter explanation

Current Nav2 file:

- `config/nav2_params_smac_mppi_omni.yaml`

Important sections:

| Section | Current role | Notes |
| --- | --- | --- |
| `amcl` | present in file only | current localization stack does not launch AMCL; this block is effectively dormant |
| `bt_navigator` | Nav2 behavior-tree navigator | uses Nav2 default BT XMLs and plugin library set |
| `controller_server` | local motion control | current plugin is `MPPIController` |
| `planner_server` | global path planning | current plugin is `SmacPlanner2D` |
| `local_costmap` | short-range obstacle space in `odom` | uses `VoxelLayer` + `InflationLayer` |
| `global_costmap` | map-space planning costmap | uses static map + obstacle + inflation layers |
| `behavior_server` | recovery and simple behavior actions | spin, backup, drive-on-heading, assisted teleop, wait |
| `waypoint_follower` | waypoint task execution | configured with wait-at-waypoint plugin |
| `velocity_smoother` | command smoothing | configured but the bridge in localization still listens only to `/cmd_vel` by default |

Key controller and planner details:

| Parameter / group | Current value direction | Practical effect |
| --- | --- | --- |
| `FollowPath.plugin = nav2_mppi_controller::MPPIController` | active | optimization-based local control |
| `motion_model = "Omni"` | active | allows lateral motion planning/control |
| `vx_max = 0.26`, `vy_max = 0.20`, `wz_max = 1.0` | conservative | limits live chassis speed |
| `batch_size = 1000`, `time_steps = 56` | substantial | MPPI considers many sampled trajectories |
| `VelocityDeadbandCritic.deadband_velocities = [0.08, 0.07, 0.12]` | active | discourages tiny commands in the drivetrain deadband |
| `GridBased.plugin = nav2_smac_planner/SmacPlanner2D` | active | graph-based 2D planning on occupancy costmap |
| `allow_unknown = true` | active | planner can pass through unknown map cells if needed |
| `cost_travel_multiplier = 2.0` | moderate | obstacle proximity meaningfully affects path cost |

Key costmap details:

| Parameter / group | Current setting | Practical meaning |
| --- | --- | --- |
| local costmap `global_frame = odom` | rolling local frame | reactive control uses odom-relative obstacle space |
| global costmap `global_frame = map` | map-fixed | long-range planning uses global map coordinates |
| `robot_radius = 0.40` | shared | collision envelope assumption |
| obstacle topic `/cloud_all_fields_fullframe` | active in both costmaps | Nav2 obstacle processing still sees raw cloud |
| inflation radius `0.55` | moderate | keeps paths away from obstacles |

### 16.4 Frontend map coordinate conversion

The Store Map page currently uses the following conversion between ROS map coordinates and raster pixels:

- `px = (x - origin_x) / resolution`
- `py = map_height_px - (y - origin_y) / resolution`

Inverse conversion:

- `x = origin_x + px * resolution`
- `y = origin_y + (map_height_px - py) * resolution`

This is why:

- the map YAML `origin` is not the same thing as "robot home pose"
- semantic anchors may legally use `(0,0,0)` even when the raster origin is not `(0,0,0)`

## 17. Current Unfinished Items and Risk Matrix

| Area | Current code reality | Risk | Severity | Recommended next step |
| --- | --- | --- | --- | --- |
| Restock execution intake | employee UI writes `restock_id`, default executor polls only `order_id` | restock requests never reach robot execution automatically | High | add `/api/restock/latest|ack|complete` or unify polling contract |
| Inventory side effects | customer tree does not change stock; restock tree decrements stock | store state diverges from intended business semantics | High | split increment/decrement endpoints and call them by workflow type |
| Robot status page | placeholder backend positions and batteries | UI may be mistaken for live fleet telemetry | Medium | replace with ROS-fed status source or label page clearly as simulated |
| Semantic source-of-truth | YAML is live source, DB is only version history | direct DB edits do not affect active semantic map | Medium | choose DB-primary or keep YAML-primary but document and enforce sync tools |
| SLAM page runtime dependency | embedded app requires external `rosbridge_server` | page appears broken unless operator knows extra launch step | Medium | add diagnostics banner or launch integration helper |
| Store Map help copy | `i18n.js` still says Store Map and SLAM pages are placeholders | UI copy contradicts real implemented features | Low | update help text and translations |
| Rack BT node | `rack_nodes.py` imports `rack_interfaces` instead of `robot_interfaces` and is not in current tree import path | future rack activation may fail at import/runtime | Medium | fix import and add tests before enabling rack nodes |
| Default arm BT path | generic arm nodes are still TODO / commented | navigation reaches target but manipulation chain does not complete | High | wire `pick_arm` into default customer/restock trees |
| Costmap obstacle source | Nav2 costmaps use raw cloud, not cropped cloud | self-hits may still influence planning in some robot poses | Medium | decide whether Nav2 should also consume filtered cloud |
| Auth model | plain-text passwords and in-memory sessions | weak security and token loss on restart | Medium | hash passwords and move sessions to signed JWT or persistent store |

## 18. Direct File-to-Feature Crosswalk

| Feature | Primary files |
| --- | --- |
| Mapping bringup | `launch/slam_mapping_stack.launch.py`, `launch/cartographer_mapping.launch.py`, `config/pico_2d_mapping_quality.lua` |
| Localization + Nav2 bringup | `launch/nav2_localization_stack.launch.py`, `launch/cartographer_localization.launch.py`, `config/pico_2d_localization.lua`, `config/nav2_params_smac_mppi_omni.yaml` |
| Odom bridge and telemetry | `robot_navigation/nav2_serial_bridge.py`, `config/ekf_odom_base_imu.yaml` |
| Semantic map runtime | `robot_navigation/semantic_map_server.py`, `config/semantic_map_testmapMain.yaml`, `robot_interfaces/srv/ResolveSemanticTarget.srv` |
| Default BT execution | `robot_task_manager/bt_executor.py`, `trees/customer.py`, `trees/restock.py`, `bt_nodes/semantic_nodes.py`, `bt_nodes/navigation_nodes.py`, `bt_nodes/inventory_nodes.py` |
| Store Map UI | `fleet-manager/src/views/LocationMapView.vue`, `fleet-manager/src/components/SemanticMapCanvas.vue`, `order-api-postgre/semantic_map.js`, `order-api-postgre/server.js` |
| SLAM UI shell | `fleet-manager/src/views/SlamMapView.vue`, `fleet-manager/public/embedded/ros-web-gui`, `third_party/ros_web_gui_app` |
| Employee auth and dashboard APIs | `order-api-postgre/server.js`, `fleet-manager/src/api.js`, `fleet-manager/src/router/index.js`, `fleet-manager/src/layouts/FleetLayout.vue` |
