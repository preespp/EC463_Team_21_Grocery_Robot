#include "robot_nav2_plugins/smart_back_up.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "visualization_msgs/msg/marker.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

double clampValue(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

geometry_msgs::msg::Point makePoint(double x, double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}

}  // namespace

namespace robot_nav2_plugins
{

void SmartBackUp::onConfigure()
{
  DriveOnHeading<BackUpAction>::onConfigure();

  auto node = this->node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  const std::string param_prefix = this->behavior_name_ + ".";

  nav2_util::declare_parameter_if_not_declared(
    node, param_prefix + "robot_radius", rclcpp::ParameterValue(0.40));
  nav2_util::declare_parameter_if_not_declared(
    node, param_prefix + "max_radius", rclcpp::ParameterValue(1.00));
  nav2_util::declare_parameter_if_not_declared(
    node, param_prefix + "service_name",
    rclcpp::ParameterValue(std::string("local_costmap/get_costmap")));
  nav2_util::declare_parameter_if_not_declared(
    node, param_prefix + "cost_threshold", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, param_prefix + "radius_step", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, param_prefix + "free_threshold", rclcpp::ParameterValue(3));
  nav2_util::declare_parameter_if_not_declared(
    node, param_prefix + "visualization", rclcpp::ParameterValue(false));
  // Kept for compatibility with older parameter files that used the
  // candidate-direction implementation.
  nav2_util::declare_parameter_if_not_declared(
    node, param_prefix + "sample_step", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, param_prefix + "min_free_distance", rclcpp::ParameterValue(0.20));

  node->get_parameter(param_prefix + "robot_radius", robot_radius_);
  node->get_parameter(param_prefix + "max_radius", max_radius_);
  node->get_parameter(param_prefix + "service_name", service_name_);
  node->get_parameter(param_prefix + "cost_threshold", cost_threshold_);
  node->get_parameter(param_prefix + "radius_step", radius_step_);
  node->get_parameter(param_prefix + "free_threshold", free_threshold_);
  node->get_parameter(param_prefix + "visualization", visualization_);

  max_radius_ = std::max(max_radius_, robot_radius_);
  radius_step_ = std::max(radius_step_, 0.01);
  cost_threshold_ = clampValue(cost_threshold_, 0.0, 255.0);
  free_threshold_ = std::max(free_threshold_, 0);

  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);
  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "smart_back_up_markers", 1);
}

bool SmartBackUp::waitForCostmapService() const
{
  auto node = this->node_.lock();
  if (!node) {
    return false;
  }

  while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for %s", service_name_.c_str());
      return false;
    }
    RCLCPP_WARN(node->get_logger(), "Waiting for %s", service_name_.c_str());
  }

  return true;
}

bool SmartBackUp::findFreeSpaceCentroid(
  const Costmap & costmap,
  double pose_x,
  double pose_y,
  geometry_msgs::msg::Point & centroid,
  std::vector<geometry_msgs::msg::Point> & free_points,
  double & selected_radius) const
{
  if (costmap.metadata.size_x == 0 || costmap.metadata.size_y == 0 || costmap.data.empty()) {
    return false;
  }

  const double resolution = costmap.metadata.resolution;
  const double origin_x = costmap.metadata.origin.position.x;
  const double origin_y = costmap.metadata.origin.position.y;

  for (double radius = robot_radius_; radius <= max_radius_ + 1.0e-6; radius += radius_step_) {
    std::vector<geometry_msgs::msg::Point> current_points;

    for (std::size_t ix = 0; ix < costmap.metadata.size_x; ++ix) {
      for (std::size_t iy = 0; iy < costmap.metadata.size_y; ++iy) {
        const auto index = ix + iy * costmap.metadata.size_x;
        const double x = static_cast<double>(ix) * resolution + origin_x;
        const double y = static_cast<double>(iy) * resolution + origin_y;

        if (std::hypot(x - pose_x, y - pose_y) > radius) {
          continue;
        }

        if (static_cast<double>(costmap.data.at(index)) <= cost_threshold_) {
          current_points.push_back(makePoint(x, y));
        }
      }
    }

    if (static_cast<int>(current_points.size()) <= free_threshold_) {
      continue;
    }

    centroid = makePoint(0.0, 0.0);
    for (const auto & point : current_points) {
      centroid.x += point.x;
      centroid.y += point.y;
    }
    centroid.x /= static_cast<double>(current_points.size());
    centroid.y /= static_cast<double>(current_points.size());
    free_points = current_points;
    selected_radius = radius;
    return true;
  }

  return false;
}

void SmartBackUp::publishVisualization(
  const std::vector<geometry_msgs::msg::Point> & free_points,
  const geometry_msgs::msg::Point & target_point,
  double pose_x,
  double pose_y) const
{
  if (!visualization_) {
    return;
  }

  auto node = this->node_.lock();
  if (!node) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header.frame_id = this->global_frame_;
  delete_marker.header.stamp = node->now();
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_marker);

  visualization_msgs::msg::Marker free_space_marker;
  free_space_marker.header.frame_id = this->global_frame_;
  free_space_marker.header.stamp = node->now();
  free_space_marker.ns = "smart_back_up_free_space";
  free_space_marker.id = 0;
  free_space_marker.type = visualization_msgs::msg::Marker::POINTS;
  free_space_marker.action = visualization_msgs::msg::Marker::ADD;
  free_space_marker.pose.orientation.w = 1.0;
  free_space_marker.scale.x = 0.03;
  free_space_marker.scale.y = 0.03;
  free_space_marker.color.r = 1.0F;
  free_space_marker.color.a = 1.0F;
  free_space_marker.points = free_points;
  markers.markers.push_back(free_space_marker);

  visualization_msgs::msg::Marker target_marker;
  target_marker.header.frame_id = this->global_frame_;
  target_marker.header.stamp = node->now();
  target_marker.ns = "smart_back_up_target";
  target_marker.id = 1;
  target_marker.type = visualization_msgs::msg::Marker::POINTS;
  target_marker.action = visualization_msgs::msg::Marker::ADD;
  target_marker.pose.orientation.w = 1.0;
  target_marker.scale.x = 0.08;
  target_marker.scale.y = 0.08;
  target_marker.color.g = 1.0F;
  target_marker.color.a = 1.0F;
  target_marker.points.push_back(target_point);
  target_marker.points.push_back(makePoint(pose_x, pose_y));
  markers.markers.push_back(target_marker);

  marker_pub_->publish(markers);
}

SmartBackUp::Status SmartBackUp::onRun(const std::shared_ptr<const BackUpAction::Goal> command)
{
  auto node = this->node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_ERROR(this->logger_, "SmartBackUp only supports planar x/y retreat selection.");
    return Status::FAILED;
  }

  if (!((command->target.x > 0.0) == (command->speed > 0.0))) {
    RCLCPP_ERROR(this->logger_, "Speed and command sign did not match");
    return Status::FAILED;
  }

  if (!waitForCostmapService()) {
    return Status::FAILED;
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto future = costmap_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
    RCLCPP_ERROR(this->logger_, "Timed out waiting for %s", service_name_.c_str());
    return Status::FAILED;
  }

  const auto costmap = future.get()->map;

  if (!nav2_util::getCurrentPose(
      this->initial_pose_, *this->tf_, this->global_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
    RCLCPP_ERROR(this->logger_, "Initial robot pose is not available.");
    return Status::FAILED;
  }

  const double pose_x = this->initial_pose_.pose.position.x;
  const double pose_y = this->initial_pose_.pose.position.y;
  const double yaw = tf2::getYaw(this->initial_pose_.pose.orientation);
  const double requested_distance = std::abs(command->target.x);

  geometry_msgs::msg::Point centroid;
  std::vector<geometry_msgs::msg::Point> free_points;
  double selected_radius = 0.0;
  if (!findFreeSpaceCentroid(costmap, pose_x, pose_y, centroid, free_points, selected_radius)) {
    RCLCPP_WARN(this->logger_, "SmartBackUp could not find enough free space.");
    return Status::FAILED;
  }

  const double angle_to_free_space = std::atan2(centroid.y - pose_y, centroid.x - pose_x);
  const double angle_diff = normalizeAngle(angle_to_free_space - yaw);

  direction_x_ = std::cos(angle_diff);
  direction_y_ = std::sin(angle_diff);
  this->command_x_ = requested_distance;
  this->command_speed_ = std::abs(command->speed);
  this->command_time_allowance_ = command->time_allowance;
  this->end_time_ = this->clock_->now() + this->command_time_allowance_;
  this->last_vel_ = std::numeric_limits<double>::max();

  publishVisualization(free_points, centroid, pose_x, pose_y);

  RCLCPP_INFO(
    this->logger_,
    "SmartBackUp selected centroid retreat | radius=%.2f m | angle=%.1f deg | command_distance=%.2f m",
    selected_radius, angle_diff * 180.0 / kPi, this->command_x_);

  return Status::SUCCEEDED;
}

bool SmartBackUp::isHolonomicCollisionFree(
  double distance_traveled,
  geometry_msgs::msg::Twist * cmd_vel,
  geometry_msgs::msg::Pose2D & pose2d) const
{
  const double remaining_distance = std::fabs(this->command_x_) - distance_traveled;
  const int max_cycle_count = static_cast<int>(this->cycle_frequency_ * this->simulate_ahead_time_);
  geometry_msgs::msg::Pose2D sim_pose = pose2d;
  bool fetch_data = true;

  for (int cycle_count = 1; cycle_count <= max_cycle_count; ++cycle_count) {
    const double dt = static_cast<double>(cycle_count) / this->cycle_frequency_;
    const double body_x = cmd_vel->linear.x * dt;
    const double body_y = cmd_vel->linear.y * dt;
    const double world_x = body_x * std::cos(pose2d.theta) - body_y * std::sin(pose2d.theta);
    const double world_y = body_x * std::sin(pose2d.theta) + body_y * std::cos(pose2d.theta);
    const double sim_distance = std::hypot(body_x, body_y);

    sim_pose.x = pose2d.x + world_x;
    sim_pose.y = pose2d.y + world_y;

    if (remaining_distance - sim_distance <= 0.0) {
      break;
    }

    if (!this->collision_checker_->isCollisionFree(sim_pose, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }

  return true;
}

SmartBackUp::Status SmartBackUp::onCycleUpdate()
{
  rclcpp::Duration time_remaining = this->end_time_ - this->clock_->now();
  if (time_remaining.seconds() < 0.0 && this->command_time_allowance_.seconds() > 0.0) {
    this->stopRobot();
    RCLCPP_WARN(this->logger_, "Exceeded time allowance before finishing SmartBackUp");
    return Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
    RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
    return Status::FAILED;
  }

  const double diff_x = this->initial_pose_.pose.position.x - current_pose.pose.position.x;
  const double diff_y = this->initial_pose_.pose.position.y - current_pose.pose.position.y;
  const double distance = std::hypot(diff_x, diff_y);

  this->feedback_->distance_traveled = distance;
  this->action_server_->publish_feedback(this->feedback_);

  if (distance >= this->command_x_) {
    this->stopRobot();
    return Status::SUCCEEDED;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->angular.z = 0.0;

  double speed = this->command_speed_;
  if (this->acceleration_limit_ != 0.0 && this->deceleration_limit_ != 0.0) {
    const double current_speed =
      this->last_vel_ == std::numeric_limits<double>::max() ? 0.0 : this->last_vel_;
    const double min_feasible_speed = current_speed + this->deceleration_limit_ / this->cycle_frequency_;
    const double max_feasible_speed = current_speed + this->acceleration_limit_ / this->cycle_frequency_;
    speed = clampValue(this->command_speed_, min_feasible_speed, max_feasible_speed);

    const double remaining_distance = std::fabs(this->command_x_) - distance;
    const double max_speed_to_stop = std::sqrt(-2.0 * this->deceleration_limit_ * remaining_distance);
    if (max_speed_to_stop < std::abs(speed)) {
      speed = max_speed_to_stop;
    }
  }

  if (std::abs(speed) < this->minimum_speed_) {
    speed = this->minimum_speed_;
  }

  cmd_vel->linear.x = direction_x_ * speed;
  cmd_vel->linear.y = direction_y_ * speed;

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isHolonomicCollisionFree(distance, cmd_vel.get(), pose2d)) {
    this->stopRobot();
    RCLCPP_WARN(this->logger_, "Collision predicted during SmartBackUp");
    return Status::FAILED;
  }

  this->last_vel_ = speed;
  this->vel_pub_->publish(std::move(cmd_vel));
  return Status::RUNNING;
}

}  // namespace robot_nav2_plugins

PLUGINLIB_EXPORT_CLASS(robot_nav2_plugins::SmartBackUp, nav2_core::Behavior)
