# NAV2 Parameter Notes (Resolution + LiDAR Occlusion)

## 1. Current values in this codebase (verified)

### Resolution-related values
- `Maps/testmap1.yaml`: `resolution: 0.050000` (legacy exported map file)
- `robot_navigation/robot_navigation/nav_assistant.py`: default `export-map --resolution` is `0.03`
- `robot_navigation/config/nav2_params_cartographer.yaml`:
  - `local_costmap.local_costmap.ros__parameters.resolution: 0.03`
  - `global_costmap.global_costmap.ros__parameters.resolution: 0.03`
- `robot_navigation/launch/cartographer_mapping.launch.py`: occupancy grid default `resolution` is `0.03`
- `robot_navigation/launch/cartographer_localization.launch.py`: occupancy grid default `resolution` is `0.03` (and this grid is disabled by default in localization stack)

## 2. NAV2 key parameter for your resolution question

### Parameter
- `local_costmap/global_costmap -> resolution`

### Meaning
- Grid cell size in meters per cell.
- Smaller values increase map detail, but increase CPU and memory load.

### Current status
- Code defaults are already set to `0.03m`.
- If you still use old exported maps, effective map detail can still be `0.05m`.

### Recommended values
- Stable default: `0.03`
- More aggressive detail: `0.025` (only if compute headroom is confirmed)

### Where to edit
- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml`
  - `local_costmap...resolution`
  - `global_costmap...resolution`

### Important pairing rule
- Also export maps at matching resolution:
```bash
ros2 run robot_navigation nav_assistant export-map --map-name testmap1
```
- If only NAV2 costmap resolution is changed but the saved map remains coarse, global planning quality is still bounded by the map file resolution.

## 3. LiDAR blocked by battery box: does it affect navigation?

Yes. It can affect behavior in two major ways:

- Blind sector risk: obstacles in blocked angles are detected later or not detected.
- Self-echo / false obstacle risk: robot body parts can appear as near-field clutter if filtering is insufficient.

## 4. NAV2 parameters to tune first for partial LiDAR occlusion

File: `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml`

- `robot_radius`:
  - Current: `0.40`
  - For safer clearance under occlusion, test `0.42 ~ 0.45`
- `inflation_radius`:
  - Current: `0.55`
  - For more conservative obstacle avoidance, test `0.60 ~ 0.70`
- `obstacle_min_range`:
  - Current: `0.0`
  - If confirmed self-returns are near-field clutter, test `0.10 ~ 0.20`
  - Warning: setting this too high weakens close-range obstacle response

## 5. Minimal validation checklist after tuning

```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmap1 --with-nav2-rviz true
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
ros2 run robot_navigation nav_assistant goal --x 1.0 --y 0.0 --yaw 0.0
```

Focus points:
- Smoothness around shelf corners and narrow aisles
- Whether blocked-angle direction still causes frequent edge-scraping or sudden stops
- Oscillation frequency during dynamic obstacle encounters
