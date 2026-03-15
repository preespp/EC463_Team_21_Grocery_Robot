cmake_minimum_required(VERSION 3.8)
project(robot_manipulation)

option(BUILD_LEGACY_MANIPULATION_CPP "Build the legacy non-VX300 C++ manipulation nodes." OFF)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_LEGACY_MANIPULATION_CPP)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(trajectory_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(moveit_msgs REQUIRED)
  find_package(robot_interfaces REQUIRED)
  find_package(moveit_ros_planning_interface REQUIRED)
  find_package(moveit_core REQUIRED)

  add_executable(rack_controller src/rack_controller.cpp)
  ament_target_dependencies(rack_controller
    rclcpp
    rclcpp_action
    robot_interfaces
  )

  add_executable(arm_motor src/arm_bridge_I2C.cpp)
  ament_target_dependencies(arm_motor
    rclcpp
    std_msgs
    trajectory_msgs
  )

  add_executable(arm_controller src/arm_controller.cpp)
  ament_target_dependencies(arm_controller
    rclcpp
    rclcpp_action
    geometry_msgs
    trajectory_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    moveit_msgs
    robot_interfaces
    moveit_ros_planning_interface
    moveit_core
  )

  add_executable(arm_to_gazebo src/arm_to_gazebo.cpp)
  ament_target_dependencies(arm_to_gazebo
    rclcpp
    std_msgs
    sensor_msgs
  )

  install(TARGETS
    rack_controller
    arm_motor
    arm_controller
    arm_to_gazebo
    DESTINATION lib/${PROJECT_NAME}
  )

  message(STATUS "Building legacy robot_manipulation C++ nodes.")
else()
  message(STATUS "Skipping legacy robot_manipulation C++ nodes; VX300 ROS 2 bringup only.")
endif()

install(PROGRAMS
  scripts/vx300_quick_move.py
  scripts/vx300_hardware_diagnostics.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
