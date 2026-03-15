cmake_minimum_required(VERSION 3.8)
project(robot_manipulation)

option(BUILD_LEGACY_MANIPULATION_CPP "Build the legacy non-VX300 C++ manipulation nodes." OFF)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_ROBOT_MANIPULATION_NODES "Build robot_manipulation C++ node executables" ON)
option(BUILD_ARM_MOTOR_NODE "Build arm_motor (I2C hardware bridge)" ON)
option(BUILD_ARM_CONTROLLER_NODE "Build arm_controller (autonomous control)" ON)
option(BUILD_ARM_DEMO_CONTROLLER_NODE "Build arm_demo_controller (no-MoveIt demo action server)" ON)
option(BUILD_ARM_TO_GAZEBO_NODE "Build arm_to_gazebo (sim bridge)" ON)
option(BUILD_ARM_WAYPOINT_SERVER_NODE "Build arm_waypoint_server" ON)
option(BUILD_RACK_CONTROLLER_NODE "Build rack_controller" ON)

find_package(ament_cmake REQUIRED)

if(BUILD_ROBOT_MANIPULATION_NODES)
  if(BUILD_RACK_CONTROLLER_NODE)
    add_executable(rack_controller src/rack_controller.cpp)
    ament_target_dependencies(rack_controller
      rclcpp
      rclcpp_action
      robot_interfaces
    )
    install(TARGETS rack_controller DESTINATION lib/${PROJECT_NAME})
  endif()

  if(BUILD_ARM_MOTOR_NODE)
    add_executable(arm_motor src/arm_bridge_I2C.cpp)
    ament_target_dependencies(arm_motor
      rclcpp
      std_msgs
      trajectory_msgs
    )
    install(TARGETS arm_motor DESTINATION lib/${PROJECT_NAME})
  endif()

  if(BUILD_ARM_CONTROLLER_NODE)
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
    install(TARGETS arm_controller DESTINATION lib/${PROJECT_NAME})
  endif()

  if(BUILD_ARM_DEMO_CONTROLLER_NODE)
    add_executable(arm_demo_controller src/arm_demo_controller.cpp)
    ament_target_dependencies(arm_demo_controller
      rclcpp
      rclcpp_action
      geometry_msgs
      trajectory_msgs
      robot_interfaces
    )
    install(TARGETS arm_demo_controller DESTINATION lib/${PROJECT_NAME})
  endif()

  if(BUILD_ARM_TO_GAZEBO_NODE)
    add_executable(arm_to_gazebo src/arm_to_gazebo.cpp)
    ament_target_dependencies(arm_to_gazebo
      rclcpp
      std_msgs
      sensor_msgs
      trajectory_msgs
    )
    install(TARGETS arm_to_gazebo DESTINATION lib/${PROJECT_NAME})
  endif()

  if(BUILD_ARM_WAYPOINT_SERVER_NODE)
    add_executable(arm_waypoint_server src/arm_waypoint_server.cpp)
    ament_target_dependencies(arm_waypoint_server
      rclcpp
      rclcpp_action
      trajectory_msgs
      robot_interfaces
    )
    install(TARGETS arm_waypoint_server DESTINATION lib/${PROJECT_NAME})
  endif()
endif()
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
