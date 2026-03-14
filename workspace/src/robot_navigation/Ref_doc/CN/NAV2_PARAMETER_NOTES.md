# NAV2 Parameter Notes (Current MPPI + Smac Stack)

## 0. 当前 Nav2 栈上下文

- 当前 localization + Nav2 默认参数文件:
  `workspace/src/robot_navigation/config/nav2_params_smac_mppi_omni.yaml`
- 当前全局规划器: `nav2_smac_planner/SmacPlanner2D`
- 当前局部控制器: `nav2_mppi_controller::MPPIController`
- 当前运动模型: `Omni`
- 当前常用启动命令:

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
colcon build --symlink-install --packages-select robot_navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmapMain --with-nav2-rviz true
```

- 等价的直接 launch 版本:

```bash
ros2 launch robot_navigation nav2_localization_stack.launch.py \
  pbstream_file:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmapMain.pbstream \
  map_yaml:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmapMain.yaml \
  nav2_params_file:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/src/robot_navigation/config/nav2_params_smac_mppi_omni.yaml \
  with_nav2_rviz:=true
```

- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml`
  现在保留为旧的 DWB/NavFn 参考文件，不再是默认 localization bringup 参数文件。

## 1. 你当前代码里的实际值（已对照）

### Resolution 相关
- `Maps/testmap1.yaml`: `resolution: 0.050000`（历史导出文件，非最新默认值）
- `robot_navigation/robot_navigation/nav_assistant.py`: `export-map --resolution` 默认 `0.03`
- `robot_navigation/config/nav2_params_smac_mppi_omni.yaml`:
  - `local_costmap.local_costmap.ros__parameters.resolution: 0.03`
  - `global_costmap.global_costmap.ros__parameters.resolution: 0.03`
- `robot_navigation/launch/cartographer_mapping.launch.py`: occupancy grid `resolution` 默认 `0.03`
- `robot_navigation/launch/cartographer_localization.launch.py`: occupancy grid `resolution` 默认 `0.03`（且你当前 localization stack 里默认关闭该 grid 发布）

## 2. 你问的 NAV2 关键参数（单独列出）

### 参数名
- `local_costmap/global_costmap -> resolution`

### 作用
- 决定 Nav2 代价地图每个栅格大小（单位 m/cell）。
- 数值越小，地图越细，但 CPU/内存占用更高、更新负载更大。

### 你的现状
- 代码默认已是 `0.03m`（3cm）；如果你还在用旧地图文件（例如 `testmap1.yaml`），其分辨率仍可能是 `0.05m`。

### 建议值（按风险从低到高）
- 稳定优先: `0.03`
- 激进精细: `0.025`

### 修改位置
- 文件: `workspace/src/robot_navigation/config/nav2_params_smac_mppi_omni.yaml`
- 同时改两处，保持一致:
  - `local_costmap...resolution`
  - `global_costmap...resolution`

### 配套建议（否则会“看起来调了，但效果有限”）
- 导图时也用更细分辨率:
```bash
ros2 run robot_navigation nav_assistant export-map --map-name testmapMain
```
- 如果只改 Nav2 costmap，不重导 `.yaml/.pgm`，全局规划仍受原图精度上限限制。

## 3. LiDAR 被电池盒遮挡，会不会影响导航？

结论: 会，有概率显著影响，主要有两类风险。

- 盲区风险: 被遮挡角度上的障碍会更晚被看到，局部规划在该方向更保守或更不稳定。
- 自身误检风险: 若电池盒边缘被打到，可能在近场产生持续“假障碍”。

## 4. 遮挡场景下建议优先调整的 NAV2 参数

文件: `workspace/src/robot_navigation/config/nav2_params_smac_mppi_omni.yaml`

- `robot_radius`:
  - 你现在是 `0.40`
  - 若前向被挡且贴边风险高，可先试 `0.42 ~ 0.45`
- `inflation_radius`:
  - 你现在是 `0.55`
  - 可试 `0.60 ~ 0.70`，让绕障更保守
- `obstacle_min_range`:
  - 你现在是 `0.0`
  - 若确认近场假障碍来自机体遮挡，可试 `0.10 ~ 0.20`
  - 注意: 这个值过大，会削弱近距离障碍响应

说明:
- Nav2 当前 costmap 仍然直接观察 `/cloud_all_fields_fullframe`。
- `base_link` 裁剪框主要是给 Cartographer 点云输入做清理，不是 Nav2 的主自车过滤方案。
- Nav2 还是依赖自己的 footprint、`robot_radius` 和 inflation 参数来处理避障。

## 5. 最小验证清单（改完参数后）

```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmapMain --with-nav2-rviz true
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
ros2 run robot_navigation nav_assistant goal --x 1.0 --y 0.0 --yaw 0.0
```

观察重点:
- 贴近货架转弯是否更平滑。
- 电池盒遮挡方向是否仍频繁“擦边”或“突然急停”。
- 控制器是否出现明显频繁 oscillation（若有，先回退到更保守参数）。
