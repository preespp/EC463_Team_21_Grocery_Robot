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
