Source repository: https://github.com/StarLionJiang/ros_web_gui_app
Source snapshot commit: f5249e3651fe8253d84522ec6b70ab2562ead37a
License: CC-BY-NC-SA-4.0

This directory vendors the forked ROS Web GUI source used by [SlamMapView.vue](/f:/EC463/EC463_Team_21_Grocery_Robot/order-api-postgre/fleet-manager/src/views/SlamMapView.vue).

Build and sync flow:
1. `npm --prefix third_party/ros_web_gui_app install`
2. `npm --prefix third_party/ros_web_gui_app run build:embed`
3. `npm --prefix order-api-postgre/fleet-manager run build:ros-web-gui`
