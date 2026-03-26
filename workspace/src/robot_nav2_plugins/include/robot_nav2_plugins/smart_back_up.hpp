#ifndef ROBOT_NAV2_PLUGINS__SMART_BACK_UP_HPP_
#define ROBOT_NAV2_PLUGINS__SMART_BACK_UP_HPP_

#include <memory>
#include <string>
#include <vector>

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
  struct CandidateDirection
  {
    std::string name;
    double dir_x;
    double dir_y;
    double free_distance{0.0};
    double average_cost{255.0};
    double score{-1.0e9};
    bool valid{false};
  };

  bool waitForCostmapService() const;
  bool worldToMap(
    const Costmap & costmap,
    double world_x,
    double world_y,
    unsigned int & map_x,
    unsigned int & map_y) const;
  unsigned char getCellCost(const Costmap & costmap, double world_x, double world_y) const;
  CandidateDirection evaluateCandidate(
    const Costmap & costmap,
    const CandidateDirection & candidate,
    double pose_x,
    double pose_y,
    double yaw,
    double requested_distance) const;
  bool selectDirection(
    const Costmap & costmap,
    double pose_x,
    double pose_y,
    double yaw,
    double requested_distance,
    CandidateDirection & selected,
    std::vector<CandidateDirection> & evaluated) const;
  bool isHolonomicCollisionFree(
    double distance_traveled,
    geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d) const;
  void publishVisualization(
    const std::vector<CandidateDirection> & candidates,
    const CandidateDirection & selected,
    double pose_x,
    double pose_y,
    double yaw) const;

  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::string service_name_;
  double robot_radius_{0.40};
  double max_radius_{1.00};
  double cost_threshold_{253.0};
  double sample_step_{0.05};
  double min_free_distance_{0.20};
  bool visualization_{false};
  double direction_x_{-1.0};
  double direction_y_{0.0};
};

}  // namespace robot_nav2_plugins

#endif  // ROBOT_NAV2_PLUGINS__SMART_BACK_UP_HPP_
