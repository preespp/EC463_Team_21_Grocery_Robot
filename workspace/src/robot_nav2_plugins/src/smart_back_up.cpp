#include "robot_nav2_plugins/smart_back_up.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "visualization_msgs/msg/marker.hpp"

namespace
{

constexpr double kDiagonal = 0.7071067811865476;

double clampValue(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
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
    node, param_prefix + "cost_threshold", rclcpp::ParameterValue(253.0));
  nav2_util::declare_parameter_if_not_declared(
    node, param_prefix + "sample_step", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, param_prefix + "min_free_distance", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node, param_prefix + "visualization", rclcpp::ParameterValue(false));

  node->get_parameter(param_prefix + "robot_radius", robot_radius_);
  node->get_parameter(param_prefix + "max_radius", max_radius_);
  node->get_parameter(param_prefix + "service_name", service_name_);
  node->get_parameter(param_prefix + "cost_threshold", cost_threshold_);
  node->get_parameter(param_prefix + "sample_step", sample_step_);
  node->get_parameter(param_prefix + "min_free_distance", min_free_distance_);
  node->get_parameter(param_prefix + "visualization", visualization_);

  max_radius_ = std::max(max_radius_, min_free_distance_);
  sample_step_ = std::max(sample_step_, 0.01);
  cost_threshold_ = clampValue(cost_threshold_, 0.0, 255.0);

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

bool SmartBackUp::worldToMap(
  const Costmap & costmap,
  double world_x,
  double world_y,
  unsigned int & map_x,
  unsigned int & map_y) const
{
  const double origin_x = costmap.metadata.origin.position.x;
  const double origin_y = costmap.metadata.origin.position.y;
  const double resolution = costmap.metadata.resolution;

  if (world_x < origin_x || world_y < origin_y) {
    return false;
  }

  map_x = static_cast<unsigned int>((world_x - origin_x) / resolution);
  map_y = static_cast<unsigned int>((world_y - origin_y) / resolution);

  return map_x < costmap.metadata.size_x && map_y < costmap.metadata.size_y;
}

unsigned char SmartBackUp::getCellCost(
  const Costmap & costmap,
  double world_x,
  double world_y) const
{
  unsigned int map_x = 0;
  unsigned int map_y = 0;
  if (!worldToMap(costmap, world_x, world_y, map_x, map_y)) {
    return 255;
  }

  const auto index = static_cast<std::size_t>(map_x) +
    static_cast<std::size_t>(map_y) * costmap.metadata.size_x;
  return costmap.data.at(index);
}

SmartBackUp::CandidateDirection SmartBackUp::evaluateCandidate(
  const Costmap & costmap,
  const CandidateDirection & candidate,
  double pose_x,
  double pose_y,
  double yaw,
  double requested_distance) const
{
  CandidateDirection evaluated = candidate;
  const double evaluation_distance = std::max(max_radius_, requested_distance);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double offset_scale = robot_radius_ * 0.80;
  const std::array<double, 3> lateral_offsets{{-offset_scale, 0.0, offset_scale}};
  const double perp_x = -candidate.dir_y;
  const double perp_y = candidate.dir_x;

  double accumulated_cost = 0.0;
  int samples = 0;

  for (double distance = sample_step_; distance <= evaluation_distance + 1.0e-6; distance += sample_step_) {
    bool blocked = false;

    for (double offset : lateral_offsets) {
      const double body_x = candidate.dir_x * distance + perp_x * offset;
      const double body_y = candidate.dir_y * distance + perp_y * offset;
      const double world_x = pose_x + body_x * cos_yaw - body_y * sin_yaw;
      const double world_y = pose_y + body_x * sin_yaw + body_y * cos_yaw;
      const unsigned char cost = getCellCost(costmap, world_x, world_y);
      if (static_cast<double>(cost) >= cost_threshold_) {
        blocked = true;
        break;
      }
      accumulated_cost += static_cast<double>(cost);
      ++samples;
    }

    if (blocked) {
      break;
    }

    evaluated.free_distance = distance;
  }

  if (samples > 0) {
    evaluated.average_cost = accumulated_cost / static_cast<double>(samples);
  }

  evaluated.valid = evaluated.free_distance >= min_free_distance_;

  const double rear_bias = 0.15 * std::max(0.0, -candidate.dir_x);
  const double reach_bonus = evaluated.free_distance + 1.0e-6 >= requested_distance ? 0.25 : 0.0;
  const double cost_penalty = 0.50 * (evaluated.average_cost / 255.0);
  evaluated.score = evaluated.free_distance + rear_bias + reach_bonus - cost_penalty;

  return evaluated;
}

bool SmartBackUp::selectDirection(
  const Costmap & costmap,
  double pose_x,
  double pose_y,
  double yaw,
  double requested_distance,
  CandidateDirection & selected,
  std::vector<CandidateDirection> & evaluated) const
{
  const std::array<CandidateDirection, 5> candidates{{
    {"back", -1.0, 0.0},
    {"back_left", -kDiagonal, kDiagonal},
    {"back_right", -kDiagonal, -kDiagonal},
    {"left", 0.0, 1.0},
    {"right", 0.0, -1.0},
  }};

  bool found = false;
  for (const auto & candidate : candidates) {
    auto result = evaluateCandidate(costmap, candidate, pose_x, pose_y, yaw, requested_distance);
    evaluated.push_back(result);

    if (!result.valid) {
      continue;
    }

    if (!found || result.score > selected.score) {
      selected = result;
      found = true;
    }
  }

  return found;
}

void SmartBackUp::publishVisualization(
  const std::vector<CandidateDirection> & candidates,
  const CandidateDirection & selected,
  double pose_x,
  double pose_y,
  double yaw) const
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

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  int marker_id = 0;

  for (const auto & candidate : candidates) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = this->global_frame_;
    marker.header.stamp = node->now();
    marker.ns = "smart_back_up_candidates";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    marker.color.a = 1.0;
    marker.color.r = candidate.valid ? 0.2F : 1.0F;
    marker.color.g = candidate.valid ? 0.9F : 0.1F;
    marker.color.b = 0.2F;
    marker.points.push_back(makePoint(pose_x, pose_y));

    const double body_x = candidate.dir_x * candidate.free_distance;
    const double body_y = candidate.dir_y * candidate.free_distance;
    const double end_x = pose_x + body_x * cos_yaw - body_y * sin_yaw;
    const double end_y = pose_y + body_x * sin_yaw + body_y * cos_yaw;
    marker.points.push_back(makePoint(end_x, end_y));
    markers.markers.push_back(marker);
  }

  visualization_msgs::msg::Marker chosen;
  chosen.header.frame_id = this->global_frame_;
  chosen.header.stamp = node->now();
  chosen.ns = "smart_back_up_selected";
  chosen.id = marker_id;
  chosen.type = visualization_msgs::msg::Marker::SPHERE;
  chosen.action = visualization_msgs::msg::Marker::ADD;
  chosen.pose.orientation.w = 1.0;
  chosen.scale.x = 0.10;
  chosen.scale.y = 0.10;
  chosen.scale.z = 0.10;
  chosen.color.a = 1.0;
  chosen.color.g = 1.0F;
  const double chosen_body_x = selected.dir_x * selected.free_distance;
  const double chosen_body_y = selected.dir_y * selected.free_distance;
  chosen.pose.position.x = pose_x + chosen_body_x * cos_yaw - chosen_body_y * sin_yaw;
  chosen.pose.position.y = pose_y + chosen_body_x * sin_yaw + chosen_body_y * cos_yaw;
  markers.markers.push_back(chosen);

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

  CandidateDirection selected;
  std::vector<CandidateDirection> evaluated;
  if (!selectDirection(costmap, pose_x, pose_y, yaw, requested_distance, selected, evaluated)) {
    RCLCPP_WARN(this->logger_, "SmartBackUp could not find enough free space.");
    return Status::FAILED;
  }

  direction_x_ = selected.dir_x;
  direction_y_ = selected.dir_y;
  this->command_x_ = std::min(
    requested_distance,
    std::max(min_free_distance_, selected.free_distance - sample_step_));
  this->command_speed_ = std::abs(command->speed);
  this->command_time_allowance_ = command->time_allowance;
  this->end_time_ = this->clock_->now() + this->command_time_allowance_;
  this->last_vel_ = std::numeric_limits<double>::max();

  publishVisualization(evaluated, selected, pose_x, pose_y, yaw);

  RCLCPP_INFO(
    this->logger_,
    "SmartBackUp selected %s | free_distance=%.2f m | command_distance=%.2f m | avg_cost=%.1f",
    selected.name.c_str(), selected.free_distance, this->command_x_, selected.average_cost);

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
