#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "robot_interfaces/action/pick_arm.hpp"

class ViperXArmServer : public rclcpp::Node
{
public:
  using PickArm = robot_interfaces::action::PickArm;
  using GoalHandlePickArm = rclcpp_action::ServerGoalHandle<PickArm>;

  ViperXArmServer()
  : Node("viperx_arm_server"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    goal_active_(false)
  {
    action_name_ = this->declare_parameter<std::string>("action_name", "pick_viperx");
    arm_group_name_ = this->declare_parameter<std::string>("arm_group_name", "interbotix_arm");
    gripper_group_name_ = this->declare_parameter<std::string>(
      "gripper_group_name",
      "interbotix_gripper");
    planning_time_sec_ = this->declare_parameter<double>("planning_time_sec", 3.0);
    max_velocity_scaling_ = this->declare_parameter<double>("max_velocity_scaling", 0.2);
    max_acceleration_scaling_ = this->declare_parameter<double>("max_acceleration_scaling", 0.2);
    open_gripper_pos_ = this->declare_parameter<double>("open_gripper_pos", 0.035);
    closed_gripper_pos_ = this->declare_parameter<double>("closed_gripper_pos", 0.0);
    open_gripper_named_target_ = this->declare_parameter<std::string>(
      "open_gripper_named_target",
      "Released");
    close_gripper_named_target_ = this->declare_parameter<std::string>(
      "close_gripper_named_target",
      "Grasping");

    action_server_ = rclcpp_action::create_server<PickArm>(
      this,
      action_name_,
      std::bind(&ViperXArmServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ViperXArmServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ViperXArmServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "viperx_arm_server ready on action '%s'", action_name_.c_str());
  }

  void initialize_moveit()
  {
    arm_move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(),
      arm_group_name_);
    gripper_move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(),
      gripper_group_name_);

    arm_move_group_->setPlanningTime(planning_time_sec_);
    arm_move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_);
    arm_move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);

    gripper_move_group_->setPlanningTime(planning_time_sec_);
    gripper_move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_);
    gripper_move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);

    RCLCPP_INFO(
      this->get_logger(),
      "MoveIt initialized (arm_group=%s, gripper_group=%s, planning_frame=%s)",
      arm_group_name_.c_str(),
      gripper_group_name_.c_str(),
      arm_move_group_->getPlanningFrame().c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PickArm::Goal> goal)
  {
    if (goal_active_.load()) {
      RCLCPP_WARN(this->get_logger(), "ViperX goal rejected: another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }

    const auto command = goal->planning_group;
    if (command != "open_gripper" && command != "close_gripper" &&
      goal->target_pose.header.frame_id.empty())
    {
      RCLCPP_WARN(this->get_logger(), "ViperX goal rejected: target_pose.frame_id is empty");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePickArm>)
  {
    RCLCPP_INFO(this->get_logger(), "ViperX goal cancellation requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePickArm> goal_handle)
  {
    goal_active_.store(true);
    std::thread{std::bind(&ViperXArmServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  geometry_msgs::msg::PoseStamped transform_pose_to_planning_frame(
    const geometry_msgs::msg::PoseStamped & pose_in)
  {
    const auto planning_frame = arm_move_group_->getPlanningFrame();
    if (pose_in.header.frame_id == planning_frame) {
      return pose_in;
    }
    return tf_buffer_.transform(pose_in, planning_frame, tf2::durationFromSec(0.3));
  }

  bool plan_and_execute_arm(
    const geometry_msgs::msg::PoseStamped & target_pose,
    const std::string & ee_link)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!ee_link.empty()) {
      arm_move_group_->setPoseTarget(target_pose.pose, ee_link);
    } else {
      arm_move_group_->setPoseTarget(target_pose.pose);
    }
    const bool ok = arm_move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    arm_move_group_->clearPoseTargets();
    if (!ok) {
      return false;
    }
    return arm_move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool execute_gripper_named(const std::string & named_target)
  {
    if (named_target.empty()) {
      return false;
    }
    if (!gripper_move_group_->setNamedTarget(named_target)) {
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool ok = gripper_move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (!ok) {
      return false;
    }
    return gripper_move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool execute_gripper_position(double target_pos)
  {
    auto joint_values = gripper_move_group_->getCurrentJointValues();
    if (joint_values.empty()) {
      return false;
    }
    joint_values[0] = target_pos;
    gripper_move_group_->setJointValueTarget(joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool ok = gripper_move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (!ok) {
      return false;
    }
    return gripper_move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool execute_gripper_command(const std::string & command, float gripper_close_position)
  {
    if (command == "open_gripper") {
      if (execute_gripper_named(open_gripper_named_target_)) {
        return true;
      }
      return execute_gripper_position(open_gripper_pos_);
    }

    if (command == "close_gripper") {
      if (execute_gripper_named(close_gripper_named_target_)) {
        return true;
      }
      const double close_target =
        (gripper_close_position >= 0.0f) ? static_cast<double>(gripper_close_position) :
        closed_gripper_pos_;
      return execute_gripper_position(close_target);
    }

    return false;
  }

  void execute(const std::shared_ptr<GoalHandlePickArm> goal_handle)
  {
    auto result = std::make_shared<PickArm::Result>();
    auto feedback = std::make_shared<PickArm::Feedback>();

    if (!arm_move_group_ || !gripper_move_group_) {
      result->success = false;
      result->message = "MoveIt not initialized";
      result->final_position_error_m = -1.0f;
      goal_handle->abort(result);
      goal_active_.store(false);
      return;
    }

    try {
      const auto goal = goal_handle->get_goal();
      const auto command = goal->planning_group;

      if (command == "open_gripper" || command == "close_gripper") {
        feedback->stage = command;
        feedback->position_error_m = 0.0f;
        goal_handle->publish_feedback(feedback);

        if (!execute_gripper_command(command, goal->gripper_close_position)) {
          result->success = false;
          result->message = "Failed to execute gripper command";
          result->final_position_error_m = -1.0f;
          goal_handle->abort(result);
          goal_active_.store(false);
          return;
        }

        result->success = true;
        result->message = "Gripper command complete";
        result->final_position_error_m = 0.0f;
        goal_handle->succeed(result);
        goal_active_.store(false);
        return;
      }

      feedback->stage = "planning_arm";
      feedback->position_error_m = 0.0f;
      goal_handle->publish_feedback(feedback);

      const auto target_pose = transform_pose_to_planning_frame(goal->target_pose);
      if (!plan_and_execute_arm(target_pose, goal->ee_link)) {
        result->success = false;
        result->message = "Failed to plan/execute arm pose";
        result->final_position_error_m = -1.0f;
        goal_handle->abort(result);
        goal_active_.store(false);
        return;
      }

      if (goal->gripper_close_position >= 0.0f) {
        feedback->stage = "closing_gripper";
        goal_handle->publish_feedback(feedback);
        if (!execute_gripper_command("close_gripper", goal->gripper_close_position)) {
          result->success = false;
          result->message = "Arm moved, but failed to close gripper";
          result->final_position_error_m = -1.0f;
          goal_handle->abort(result);
          goal_active_.store(false);
          return;
        }
      }

      feedback->stage = "completed";
      goal_handle->publish_feedback(feedback);
      result->success = true;
      result->message = "ViperX motion complete";
      result->final_position_error_m = 0.0f;
      goal_handle->succeed(result);
    } catch (const std::exception & ex) {
      result->success = false;
      result->message = ex.what();
      result->final_position_error_m = -1.0f;
      goal_handle->abort(result);
    }

    goal_active_.store(false);
  }

  std::string action_name_;
  std::string arm_group_name_;
  std::string gripper_group_name_;
  std::string open_gripper_named_target_;
  std::string close_gripper_named_target_;

  double planning_time_sec_;
  double max_velocity_scaling_;
  double max_acceleration_scaling_;
  double open_gripper_pos_;
  double closed_gripper_pos_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;

  std::atomic<bool> goal_active_;
  rclcpp_action::Server<PickArm>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ViperXArmServer>();
  node->initialize_moveit();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
