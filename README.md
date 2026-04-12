# Team 21 GOFR: Grocery Operations & Fulfillment Robot
<p align="center">
<img src="./images/Logo_grey.png" width="50%">
</p>
Our group’s vision is to make everyday shopping more efficient and inclusive. We aim to address issues that would benefit both owners and customers through an intelligent, multi-functional robotic system to improve shelf management and customer experience. Our final delivery would be a multi-wheel robot with functional robotic arm(s), sensors, cameras, and artificial intelligence to assist in employee stocking and customer shopping.

<p align="center">
<img src="./images/GOFR.png" width="50%">
</p>
<p align="center">
GOFR
</p>

<p align="center">
<img src="./images/Team 21.jpg" width="50%">
</p>
<p align="center">
Darren Figo Sajino, Pree Simphliphan, Bach Thien Nguyen, Bernie Xu, Xingjian Jiang, Feng Tai
</p>

## Table of Contents
1. [Mission](#mission)
2. [Project Summary](#project-summary)
3. [Problems & Market Analysis](#problems-and-market-analysis)
4. [Goals](#goals)
5. [System Architecture](#system-architecture)
6. [Repository Guide](#repository-guide)
7. [Setup and Instructions](#setup-and-instructions)
8. [Future Development](#future-development)
9. [Team Members](#team-members)
10. [Advisor](#advisor)
11. [Additional Documentations](#additional-documentations)
12. [Additional Links](#team-links)

---
## Mission
To design and develop an intelligent, multi-functional robotic system that streamlines shelf management and enhances customer experience in a grocery store. Our mission is to integrate robotics, sensors, and artificial intelligence to assist employees in stocking tasks, ensure product availability, and provide customers with inclusive, efficient shopping support to make daily activities more convenient.

## Project Summary
In recent years, more and more robots are being employed in order to make daily life more convenient. These systems benefit the users by freeing up time and performing tasks that require precision. The Grocery Robot aims to build a robust robotic system that will perform a wide variety of assistive functions to make grocery shopping more accessible and convenient.

GOFR (Previously, GroceryBot), Grocery Operations & Fulfillment Robot, is an autonomous retail assistant built to support both shopping and restocking workflows. The goal is not to replace store employees, but to reduce repetitive operational burden and improve customer access in environments where reliable staffing and quick item fulfillment matter every day.

At a system level, GOFR combines store-scale mobility with item-level interaction. Customer or staff requests originate in the web app and backend, are resolved against a semantic store map, and are then executed by the ROS 2 autonomy stack. The robot localizes with LiDAR, odometry, and IMU inputs, navigates using Cartographer and Nav2, and uses camera-based perception to verify and align on target items before manipulation. This creates one continuous pipeline from store request to in-aisle execution.

## Problems and Market Analysis
### Market Research
Grocery is one of the largest everyday retail sectors in the world. Based on the team presentation research, the global grocery retail market was valued at roughly $12 trillion in 2023 and is projected to approach $15 trillion by 2030. That scale means even small improvements in stocking efficiency, order fulfillment speed, or shopping accessibility can have meaningful operational impact.

At the same time, grocery stores continue to face persistent labor and workflow pressure. The project research found that 68% of grocery managers report difficulty finding labor for ongoing day-to-day operations. Grocery retail depends on constant movement: checking shelves, restocking items, answering shopper questions, and preparing online or delivery orders. These tasks are repetitive, time-sensitive, and difficult to sustain when staffing is limited. When labor falls short, shelves remain empty longer, order preparation slows down, and employees are forced to divide attention between logistics and customer support.

Accessibility is another major gap in the sector. About 1 in 4 U.S. adults live with a disability, and the presentation research notes that more than 27% of people with disabilities experience a shopping barrier at least occasionally more than once per month. In a grocery setting, that can mean difficulty locating items, reaching products, navigating crowded aisles, or shopping independently without assistance. This makes grocery automation not just an efficiency problem, but also an inclusion problem.

These conditions create a clear opportunity for an assistive retail robot. GOFR is positioned to address both sides of the market need: operational support for stores and improved shopping support for customers. Unlike systems focused only on monitoring inventory, GOFR is intended to connect navigation, perception, and manipulation into one platform that can move through a store, identify a target area, verify products, and participate directly in fulfillment or restocking tasks.

### User Stories and User Requirement
- As a store owner, I want the robot to automatically restock the product to reduce the cost of human labor. 
- As a store owner, I want to have a real-time digital inventory report so I can save time waiting for the report.
- As a store employee, I want the robot to reduce my workload and focus more on more important things or serve the customers.
- As a shop customer, I want the robot to look for the product and navigate to the aisle for me, so that I will not get lost in the store.
- As a customer with special needs, I want to shop independently so that I do not have to ask my friends/relatives for help.
- As a delivery driver, I want to have my customer’s order be ready and packed when I arrive to save time. 

## Goals
### Minimum Viable Product
- Single-Aisle Navigation: fixed route in a mock aisle using line tracing mechanism for navigation with integration of LiDAR and IR Sensors. with 90% success rate under 5 minutes per operation
- Place, Verify & Recovery: Predefined motion of control of robotics arm with range of operation 2-meter height and 2-pound of payload per picking
- Approach & Alignment: Computer Vision to change the orientation of the gripper to pick up product accurately with KPI of 90% success rate
- Safety feature: distance sensor to prevent collision

### Stretch Goals
- Real-time Inventory Report with Database
- Multiple-Aisle Navigation using SLAM algorithms
- Robotics Arm Motions: Integrate Computer Vision or VLA for motion control
- Two modes for customers and store employees
- Online Order Prep from delivery drivers
- Additional Assitance feature on screen for customers
- Safety feature: avoid collision and re-route new navigation

## System Architecture
<p align="center">
<img src="./images/main_arch.png" width="50%">
</p>
<p align="center">
Overall Software System Architecture
</p>

<p align="center">
<img src="./images/power_diagram.png" width="50%">
</p>
<p align="center">
Power Diagram
</p>

<p align="center">
<img src="./images/communication_diagram.png" width="50%">
</p>
<p align="center">
Communication Diagram
</p>

<p align="center">
<img src="./images/CAD_final.png" width="50%">
</p>
<p align="center">
CAD Mechanical Design
</p>

The current production software path is:

1. Customer or employee requests are created from the app and backend in `order-api-postgre`.
2. The backend stores inventory, user, and semantic-map data in PostgreSQL and exposes APIs used by the web app and the robot.
3. `robot_task_manager` polls or receives orders, resolves semantic targets, and runs the ViperX/customer or restock Behavior Tree.
4. `robot_navigation` localizes the robot with LiDAR, odometry, and IMU, then executes mobile-base navigation through Cartographer and Nav2.
5. `robot_vision` publishes RealSense detections and camera TF data on `/detections_json` and related topics.
6. `robot_manipulation` executes ViperX arm motions through MoveIt and the `/pick_viperx` action server.
7. `robot_perception` publishes ultrasonic collision-alert topics used for safety.
8. The embedded layer is split between `STM32` for the mobile base and `ESP32` projects for ultrasonic sensing, servo experiments, and actuator-side prototyping.

### Software Stack
- Python
- C++
- ROS2 Humble
- Behavior Tree (pytrees)
- OpenCV
- YOLO Model
- IntelRealSense SDK
- Node.js
- Vue.js
- HTML
- CSS
- Javascript
- Gazebo
- Moveit2
- C
- ESP-IDF (Espressif)
- Database (Postgresql)
- Gemini API

### Hardware (Mechanical)
- 12V DC motors -> DJI Motors
- High-torque servo motors
- 3D printing components (PETG)
- Mecanum wheels
- Aluminum framing (80/20 profiles and tubing)
- Acylic Plate
- ViperX 300S Robotic Arm

### Hardware (Electrical & Sensing)
- 2D LiDAR Sensor (SICK picoscan150)
- Odometry Sensor (MCU Gyroscope)
- IntelRealSense Camera
- Ultrasonic Sensors
- 12V lead-acid batteries (+ Fuse and Power Distribution)
- ESP32
- STM32
- Remote E-Stop
- NVIDIA Jetson Nano

### Current Status Notes

- The ViperX arm path is the most complete end-to-end manipulation path in the repository today.
- The custom servo-arm and rack path exists in software and mechanical design, but it is still under hardware development and is not yet the main validated runtime path.
- The app, backend, navigation, camera, BT, and ViperX arm flow are already integrated in the repository as the primary demoable full-stack system.

## Repository Guide

The repository is organized around those major subsystems:

- `workspace/src/robot_navigation`: ROS 2 navigation, mapping, localization, Nav2 tuning, semantic map serving, teleoperation, and STM32 serial bridging
- `workspace/src/robot_interfaces`: shared ROS 2 custom messages, services, and actions used across the robot software stack
- `workspace/src/robot_manipulation`: ViperX / VX300 manipulation, MoveIt integration, simulation helpers, BT-facing arm action servers, and rack-related code
- `workspace/src/robot_task_manager`: Behavior Tree orchestration, order intake, blackboard state, semantic target resolution, and task sequencing
- `workspace/src/robot_vision`: RealSense, YOLO-based detection, barcode utilities, and camera TF publishing
- `workspace/src/robot_perception`: ultrasonic collision sensing nodes
- `workspace/src/mvp_robot`: older MVP prototype package kept for reference and legacy experiments
- `order-api-postgre`: Node.js + PostgreSQL backend for orders, inventory, authentication, and semantic-map data
- `order-api-postgre/fleet-manager`: Vue-based operations and fleet dashboard
- `ESP32`: actuator and sensor firmware projects such as ultrasonic I2C, arm servo I2C, and linear-actuator experiments
- `STM32`: mobile-base motor-control firmware and base-control logic
- `Maps`: saved `.pbstream`, `.yaml`, and `.pgm` navigation maps
- `files`: proposal, report PDFs, and exported architecture graphs
- `images`: diagrams, logos, CAD renders, and presentation assets
- `third_party`: vendored dependencies used by the dashboards and robot stack

Sections 5 and 6 have been updated to match the current repository structure and the present validated software flow.

## Setup and Instructions

This section describes the typical software bring-up after the hardware build is complete.

### 1. Finish Hardware Integration First

Before software bring-up, make sure the robot hardware is physically assembled and wired:

- Jetson Nano or robot computer installed and powered
- STM32 base controller connected to motor drivers and serial link
- LiDAR and IMU mounted and wired
- Intel RealSense camera mounted on the ViperX end effector
- ViperX / VX300S arm powered and connected by USB
- ESP32 boards connected for the ultrasonic and actuator subsystems that you plan to use

### 2. Flash Embedded Controllers

#### ESP32

Use the ESP32 folder that matches the hardware subsystem you are flashing:

- `ESP32/ultrasonic_I2C`: ultrasonic-sensor firmware used by the ROS ultrasonic collision stack
- `ESP32/arm_servo_I2C`: custom servo-arm I2C firmware prototype
- `ESP32/linear_actuator`: linear-actuator firmware prototype
- `ESP32/ultrasonic` and `ESP32/servo-component-testing`: Arduino-based bench tests

Typical ESP-IDF flow:

```bash
cd ESP32/ultrasonic_I2C
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Repeat the same pattern for `arm_servo_I2C` or `linear_actuator`, changing the folder and serial port as needed.

For Arduino sketches, flash them from the Arduino IDE with ESP32 board support.

#### STM32

The current mobile-base firmware is in:

```text
STM32/Base_Control_v2/robowalker2024bottominfantry-main/test
```

Open the project from:

```text
STM32/Base_Control_v2/robowalker2024bottominfantry-main/test/test.ioc
```

Then build and flash it with STM32CubeIDE or a compatible STM32 toolchain. This firmware is the base motor-control side used by the ROS navigation serial bridge.

### 3. Install Robot Software On The Jetson / Linux Host

At minimum, install and prepare:

- ROS 2 Humble
- PostgreSQL
- Node.js and npm
- Python 3
- Interbotix dependencies for the ViperX / VX300S arm
- Intel RealSense SDK dependencies

### 4. Build The ROS 2 Workspaces

Build the Interbotix workspace first, then the main robot workspace.

Example:

```bash
cd /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws
source /opt/ros/humble/setup.bash
colcon build
```

```bash
cd /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
colcon build --base-paths src
source install/setup.bash
```

### 5. Set Up The App And Backend

Follow the backend setup in `order-api-postgre/README.md`.

Typical order:

1. create the PostgreSQL database and seed it
2. start the Node.js backend
3. start the Vue fleet-manager app

Typical commands:

```bash
cd order-api-postgre
npm install
npm run dev
```

```bash
cd order-api-postgre/fleet-manager
npm install
npm run dev -- --host 0.0.0.0
```

### 6. Run The Robot Software Stack

For the current ViperX-based integrated stack with the app/backend, use separate terminals and source the environment in each one:

```bash
source /opt/ros/humble/setup.bash
source /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
```

If you want the one-command integrated ROS stack:

```bash
ros2 launch robot_task_manager viperx_nav_camera_bt.launch.py \
  motor_port:=/dev/ttyUSB1 \
  serial_port:=/dev/ttyUSB0 \
  include_ultrasonic:=true
```

This launch is the current combined ROS bring-up for:

- ViperX arm + MoveIt + `/pick_viperx`
- navigation + semantic map
- camera vision
- task-manager BT
- optional ultrasonic sensing

Then run the backend and app in separate terminals:

```bash
cd order-api-postgre
npm run dev
```

```bash
cd order-api-postgre/fleet-manager
npm run dev -- --host 0.0.0.0
```

### 7. Recommended Bring-Up Order For The ViperX + App Demo

1. power the robot and verify STM32 / ESP32 boards are flashed
2. start PostgreSQL, backend, and app
3. source the ROS environments
4. launch `robot_task_manager viperx_nav_camera_bt.launch.py`
5. verify LiDAR, Nav2, camera, and `/pick_viperx` are up
6. log in to the app and create an order or restock task
7. let `robot_task_manager` consume the order and run the BT

### 8. Notes

- The ViperX path is the most complete software path today.
- The custom servo-arm and rack path is still under active hardware development, so its full hardware bring-up is not yet the default deployment workflow.
- If you only want a quick arm-and-camera test, use the package-level instructions in `workspace/src/robot_task_manager/README.md` and `workspace/src/robot_manipulation/README.md`.

## Future Development

- Custom robotic servo arm with racking system: the custom lightweight arm and rack system is still under development and needs components that are light enough for the platform while still handling grocery payloads and arm loads. The related BT logic for the custom rack and servo arm exists in the repository but remains untested on final hardware because that hardware is still under development.
- Integration of camera with the custom servo arm: after the custom arm hardware stabilizes, the next step is to mount and calibrate the camera pipeline for that arm the way the ViperX path already does today.
- Integration of VLA/VLM with the robotic arm: future manipulation work includes using vision-language or vision-language-action systems for higher-level scene understanding, grasp selection, and more adaptive arm behavior.
- Cloud system for the app and our own AI model without relying on Gemini API: the current app uses a Gemini-based integration path, but a future goal is to migrate toward a cloud-hosted system owned by the project, including our own model-serving pipeline.
- Multiple robots working together under one fleet manager: the current software is centered on one robot, but the long-term direction is coordinated multi-robot operation under a single fleet-management interface.

## Team Members
- Bach Thien Nguyen, Mechanical Engineering
- Bernie Xu, Mechanical Engineering
- Darren Figo Sajino, Mechanical Engineering
- Feng Tai, Computer Engineering
- Pree Simphliphan, Computer Engineering
- Xingjian Jiang, Computer Engineering

## Advisor
- Professor Thomas Little

### EC463-464 Instuctors
- Professor Thomas Little
- Professor Ryan Lagoy
- Professor Osama Alshaykh

## Additional Documentations

### Embedded And Backend

- [ESP32 README](./ESP32/README.md)
- [STM32 README](./STM32/README.md)
- [Order API + PostgreSQL README](./order-api-postgre/README.md)

### ROS 2 Packages In `workspace/src`

- [mvp_robot README](./workspace/src/mvp_robot/README.md)
- [robot_interfaces README](./workspace/src/robot_interfaces/README.md)
- [robot_manipulation README](./workspace/src/robot_manipulation/README.md)
- [robot_navigation README](./workspace/src/robot_navigation/README.md)
- [robot_perception README](./workspace/src/robot_perception/README.md)
- [robot_task_manager README](./workspace/src/robot_task_manager/README.md)
- [robot_vision README](./workspace/src/robot_vision/README.md)

### Reports And Project Files In `files`

- [Final Report PDF](./files/BU_GOFR_Grocery_Operations_Fulfillment_Robot.pdf)
- [Project Proposal PDF](./files/Proposal%20for%20EC463.pdf)

## Additional Links

## Team links
- [Team Google Drive](https://drive.google.com/drive/folders/1yiAgVb-4LaUo8HKmD3yormvIIuaboWg9)

## Course links
- [ECE Senior Design Piazza Site](https://piazza.com/bu/fall2025/ec463/home)
- [Blackboard](http://learn.bu.edu/)

## Optional features links
- [Team Jira](https://seniordesign-team-21.atlassian.net/jira/software/projects/SCRUM/summary)
