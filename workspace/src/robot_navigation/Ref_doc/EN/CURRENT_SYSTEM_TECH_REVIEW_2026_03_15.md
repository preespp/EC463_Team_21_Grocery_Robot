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
