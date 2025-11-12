cmake_minimum_required(VERSION 3.8)
project(mvp_robot)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/ProductInfo.srv"
    "srv/ProductRequest.srv"
)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch srv
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
