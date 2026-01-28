# ROS2 SLAM Bringup (Cartographer + SICK PicoScan)

This document explains how to start our SLAM stack on the Jetson and what each command does.

We are using:

- **ROS2** (Humble)
- **Cartographer** for 2D SLAM
- **SICK PicoScan** LiDAR (`sick_scan_xd`)
- **RViz2** for visualization

---

## Network Setup

- **LiDAR IP (hostname)**: `192.168.8.150`  
- **Jetson IP (UDP receiver)**: `192.168.8.143`

The LiDAR sends UDP packets to the Jetson, where `sick_scan_xd` publishes point clouds to ROS2.

If these IPs ever change, update them in the LiDAR launch command below.

---

## 1. Start RViz2

RViz lets us see the map, robot pose, and point clouds.

```bash
ros2 run rviz2 rviz2
```

Tips:

- Load our saved RViz config (if we have one) so the **map**, **TF**, and **point cloud** displays are already set up.
- Make sure the **fixed frame** is set correctly (usually `map` or `world`).

---

## 2. Launch Cartographer

Cartographer subscribes to the LiDAR data and IMU (if available) and builds the 2D map.  
Our launch file is in `~/carto_cfg/my_carto.launch.py`.

```bash
ros2 launch ~/carto_cfg/my_carto.launch.py
```

What this does:

- Starts `cartographer_node`
- Sets up the required frames (e.g., `map -> odom`)
- Subscribes to the point cloud topic (e.g., `/cloud_all_fields_fullframe` – check the launch file)

If Cartographer is running correctly, you’ll see:

- The **TF tree** including `map`, `odom`, and `world`
- Submaps and the global map being updated in RViz

---

## 3. Start the SICK PicoScan LiDAR Driver

This brings up the LiDAR, receives data from the physical sensor, and publishes it into ROS2.

```bash
ros2 launch sick_scan_xd sick_picoscan.launch.py   hostname:=192.168.8.150   udp_receiver_ip:=192.168.8.143
```

Explanation:

- `hostname`: IP address of the LiDAR itself
- `udp_receiver_ip`: IP address of the Jetson that will receive the packets

Once this is running, you should see:

- A point cloud topic like `/cloud_all_fields_fullframe` in `ros2 topic list`
- Data moving in RViz (if visualized)

---

## Typical Bringup Order

In practice, you can start them in any order, but this sequence is easy to follow:

1. **Open RViz** (so you can see everything):
   ```bash
   ros2 run rviz2 rviz2
   ```
2. **Launch Cartographer**:
   ```bash
   ros2 launch ~/carto_cfg/my_carto.launch.py
   ```
3. **Start the LiDAR driver**:
   ```bash
   ros2 launch sick_scan_xd sick_picoscan.launch.py      hostname:=192.168.8.150      udp_receiver_ip:=192.168.8.143
   ```

After a few seconds of moving the robot, the map should gradually appear in RViz.

---

## TF and Node Graph (Reference)

We have two reference diagrams:

- `frames.png` – TF tree showing `map -> odom -> world`
- `rosgraph.png` – Node graph showing connections between:
  - `/sick_scansegment_xd`
  - `/cloud_all_fields_fullframe`
  - `/cartographer_node`
  - `/submap_list`
  - `/carto_grid`
  - `/map`
  - `/tf` and transform listeners

These are useful for debugging when something looks wrong in RViz (e.g., multiple TF arrows, jumps in pose, etc.).

---

## Quick Debug Checklist

If SLAM isn’t working:

1. **Check topics**  
   ```bash
   ros2 topic list
   ```
   Make sure the LiDAR and Cartographer topics (e.g., `/cloud_all_fields_fullframe`, `/submap_list`, `/map`) are present.

2. **Check TF**  
   ```bash
   ros2 run tf2_tools view_frames
   ```
   Then open the generated PDF/PNG and verify that `map`, `odom`, and `world` are connected as expected.

3. **Check RViz Fixed Frame**  
   Set it to `map` or `world` (depending on our config) to avoid “No transform” errors.

---

## Summary of Commands

```bash
# 1. Open RViz
ros2 run rviz2 rviz2

# 2. Launch Cartographer
ros2 launch ~/carto_cfg/my_carto.launch.py

# 3. Start LiDAR and send UDP to Jetson
ros2 launch sick_scan_xd sick_picoscan.launch.py   hostname:=192.168.8.150   udp_receiver_ip:=192.168.8.143
```

Keep this README with the project so anyone on the team can bring up SLAM on the Jetson quickly.
---

## Save the Cartographer Map (pbstream)

When you are happy with the map, save Cartographer's current state to our `Maps/` folder.

Example (`testmap1`):

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.pbstream', include_unfinished_submaps: true}"
```

Notes:
- This saves Cartographer’s internal state (`.pbstream`), which is what we load later for localization.

---

## Export pbstream to a Nav2-friendly Map (YAML + PGM)

Nav2 can also use a classic occupancy map (`.yaml + .pgm`). Convert like this:

```bash
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.pbstream \
  -map_filestem /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1 \
  -resolution 0.05
```

This will create:

- `.../Maps/testmap1.yaml`
- `.../Maps/testmap1.pgm`

---

## Run Cartographer in Localization Mode (Frozen Map)

To reuse a saved map, start Cartographer with the saved `.pbstream` and freeze the loaded state so it **localizes** instead of continuing to build/change the map.

We run our localization launch:

```bash
ros2 launch ~/carto_cfg/my_carto_Localization.launch.py
```

What this launch should do (high level):
- Start `cartographer_node` with:
  - `-load_state_filename .../Maps/<map>.pbstream`
  - `-load_frozen_state true`
- Start `cartographer_occupancy_grid_node` to publish `/map`

If localization is working, the TF tree should still look like:

- `map -> odom -> world`

And RViz **Fixed Frame** should typically be set to `map`.

---

## Start Nav2 Using Cartographer’s Map

Once Cartographer localization is running (publishing `/map` and `map->odom`), start Nav2:

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  autostart:=true \
  params_file:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Nav/nav2_params_cartographer.yaml
```

Then start Nav2’s RViz config:

```bash
ros2 launch nav2_bringup rviz_launch.py
```

### How to know Nav2 is working
- In the Nav2 terminal, you should see:
  - `Managed nodes are active`
- In RViz:
  - Fixed Frame = `map`
  - You can click **“Nav2 Goal”** and a path should appear.
- `cmd_vel` should publish when a goal is active:
  ```bash
  ros2 topic echo /cmd_vel
  ```

---

## Typical Bringup Order (Mapping vs Localization)

### Mapping run (build a new map)
1. Start RViz:
   ```bash
   ros2 run rviz2 rviz2
   ```
2. Start Cartographer (mapping):
   ```bash
   ros2 launch ~/carto_cfg/my_carto.launch.py
   ```
3. Start the LiDAR driver:
   ```bash
   ros2 launch sick_scan_xd sick_picoscan.launch.py   hostname:=192.168.8.150   udp_receiver_ip:=192.168.8.143
   ```
4. Drive the robot, then save the map:
   ```bash
   ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.pbstream', include_unfinished_submaps: true}"
   ```

### Localization + Navigation run (reuse a saved map + Nav2)
1. Start LiDAR driver:
   ```bash
   ros2 launch sick_scan_xd sick_picoscan.launch.py   hostname:=192.168.8.150   udp_receiver_ip:=192.168.8.143
   ```
2. Start Cartographer localization (frozen pbstream):
   ```bash
   ros2 launch ~/carto_cfg/my_carto_Localization.launch.py
   ```
3. Start Nav2:
   ```bash
   ros2 launch nav2_bringup navigation_launch.py \
     use_sim_time:=false \
     autostart:=true \
     params_file:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Nav/nav2_params_cartographer.yaml
   ```
4. Start Nav2 RViz:
   ```bash
   ros2 launch nav2_bringup rviz_launch.py
   ```

