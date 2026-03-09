# NAV2 Parameter Notes (Resolution + LiDAR Occlusion)

## 1. 你当前代码里的实际值（已对照）

### Resolution 相关
- `Maps/testmap1.yaml`: `resolution: 0.050000`（历史导出文件，非最新默认值）
- `robot_navigation/robot_navigation/nav_assistant.py`: `export-map --resolution` 默认 `0.03`
- `robot_navigation/config/nav2_params_cartographer.yaml`:
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
- 文件: `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml`
- 同时改两处，保持一致:
  - `local_costmap...resolution`
  - `global_costmap...resolution`

### 配套建议（否则会“看起来调了，但效果有限”）
- 导图时也用更细分辨率:
```bash
ros2 run robot_navigation nav_assistant export-map --map-name testmap1
```
- 如果只改 Nav2 costmap，不重导 `.yaml/.pgm`，全局规划仍受原图精度上限限制。

## 3. LiDAR 被电池盒遮挡，会不会影响导航？

结论: 会，有概率显著影响，主要有两类风险。

- 盲区风险: 被遮挡角度上的障碍会更晚被看到，局部规划在该方向更保守或更不稳定。
- 自身误检风险: 若电池盒边缘被打到，可能在近场产生持续“假障碍”。

## 4. 遮挡场景下建议优先调整的 NAV2 参数

文件: `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml`

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

## 5. 最小验证清单（改完参数后）

```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmap1 --with-nav2-rviz true
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
ros2 run robot_navigation nav_assistant goal --x 1.0 --y 0.0 --yaw 0.0
```

观察重点:
- 贴近货架转弯是否更平滑。
- 电池盒遮挡方向是否仍频繁“擦边”或“突然急停”。
- 控制器是否出现明显频繁 oscillation（若有，先回退到更保守参数）。
