cmake_minimum_required(VERSION 3.8)
project(mvp_robot)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
