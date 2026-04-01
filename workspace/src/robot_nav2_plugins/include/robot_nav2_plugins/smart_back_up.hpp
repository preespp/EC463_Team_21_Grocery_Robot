#ifndef ROBOT_NAV2_PLUGINS__SMART_BACK_UP_HPP_
#define ROBOT_NAV2_PLUGINS__SMART_BACK_UP_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace robot_nav2_plugins
{

class SmartBackUp : public nav2_behaviors::DriveOnHeading<nav2_msgs::action::BackUp>
{
public:
  using BackUpAction = nav2_msgs::action::BackUp;
  using Costmap = nav2_msgs::msg::Costmap;
  using Status = nav2_behaviors::Status;

  Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;
  Status onCycleUpdate() override;

protected:
  void onConfigure() override;

private:
  bool waitForCostmapService() const;
  bool findFreeSpaceCentroid(
    const Costmap & costmap,
    double pose_x,
    double pose_y,
    geometry_msgs::msg::Point & centroid,
    std::vector<geometry_msgs::msg::Point> & free_points,
    double & selected_radius) const;
  bool isHolonomicCollisionFree(
    double distance_traveled,
    geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d) const;
  void publishVisualization(
    const std::vector<geometry_msgs::msg::Point> & free_points,
    const geometry_msgs::msg::Point & target_point,
    double pose_x,
    double pose_y) const;

  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::string service_name_;
  double robot_radius_{0.40};
  double max_radius_{1.00};
  double cost_threshold_{0.1};
  double radius_step_{0.1};
  int free_threshold_{3};
  bool visualization_{false};
  double direction_x_{-1.0};
  double direction_y_{0.0};
};

}  // namespace robot_nav2_plugins

#endif  // ROBOT_NAV2_PLUGINS__SMART_BACK_UP_HPP_
