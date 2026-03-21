#include <atomic>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit/robot_state/conversions.h"
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
    cartesian_eef_step_m_ = this->declare_parameter<double>("cartesian_eef_step_m", 0.01);
    cartesian_min_fraction_ = this->declare_parameter<double>("cartesian_min_fraction", 0.9);
    preview_only_ = this->declare_parameter<bool>("preview_only", false);
    preview_step_delay_sec_ = this->declare_parameter<double>("preview_step_delay_sec", 1.0);
    open_gripper_pos_ = this->declare_parameter<double>("open_gripper_pos", 0.035);
    closed_gripper_pos_ = this->declare_parameter<double>("closed_gripper_pos", 0.0);
    open_gripper_named_target_ = this->declare_parameter<std::string>(
      "open_gripper_named_target",
      "Released");
    close_gripper_named_target_ = this->declare_parameter<std::string>(
      "close_gripper_named_target",
      "Grasping");
    return_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "return_joint_names",
      std::vector<std::string>{
        "waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"});
    return_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "return_joint_positions",
      std::vector<double>{0.0, -1.85004901, 1.53588974, 0.0, 0.80285146, 0.0});
    place_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "place_joint_names",
      std::vector<std::string>{
        "waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"});
    place_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "place_joint_positions",
      std::vector<double>{1.81514242, -0.2268928, 1.53588974, 0.06981317, -1.34390352, -0.01745329});

    action_server_ = rclcpp_action::create_server<PickArm>(
      this,
      action_name_,
      std::bind(&ViperXArmServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ViperXArmServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ViperXArmServer::handle_accepted, this, std::placeholders::_1));
    display_traj_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path",
      rclcpp::QoS(1).reliable().transient_local());

    RCLCPP_INFO(
      this->get_logger(),
      "viperx_arm_server ready on action '%s' (preview_only=%s)",
      action_name_.c_str(),
      preview_only_ ? "true" : "false");
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
    if (
      command != "open_gripper" && command != "close_gripper" &&
      command != "return_arm_pose" && command != "place_arm_pose" &&
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
    const std::string & ee_link,
    bool use_cartesian)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (use_cartesian) {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(target_pose.pose);

      moveit_msgs::msg::RobotTrajectory robot_traj;
      const double fraction = arm_move_group_->computeCartesianPath(
        waypoints,
        cartesian_eef_step_m_,
        0.0,
        robot_traj,
        true);
      if (fraction < cartesian_min_fraction_ || robot_traj.joint_trajectory.points.empty()) {
        RCLCPP_WARN(
          this->get_logger(),
          "Cartesian path fraction too low: %.3f < %.3f",
          fraction,
          cartesian_min_fraction_);
        return false;
      }
      plan.trajectory_ = robot_traj;
    } else {
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
    }
    if (preview_only_) {
      publish_preview(plan);
      return true;
    }
    return arm_move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  void publish_preview(const moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    moveit_msgs::msg::DisplayTrajectory display_msg;
    display_msg.model_id = arm_move_group_->getRobotModel()->getName();
    if (const auto current_state = arm_move_group_->getCurrentState(0.5)) {
      moveit::core::robotStateToRobotStateMsg(*current_state, display_msg.trajectory_start);
    }
    display_msg.trajectory.push_back(plan.trajectory_);
    display_traj_pub_->publish(display_msg);
    RCLCPP_INFO(this->get_logger(), "Published preview trajectory on /display_planned_path");
  }

  void maybe_preview_delay() const
  {
    if (!preview_only_) {
      return;
    }
    if (preview_step_delay_sec_ <= 0.0) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(preview_step_delay_sec_));
  }

  bool execute_gripper_named(const std::string & named_target)
  {
    if (named_target.empty()) {
      return false;
    }
    if (!gripper_move_group_->setNamedTarget(named_target)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to set gripper named target '%s'.",
        named_target.c_str());
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool ok = gripper_move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (!ok) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to plan gripper named target '%s'.",
        named_target.c_str());
      return false;
    }
    if (preview_only_) {
      publish_preview(plan);
      return true;
    }
    return gripper_move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool execute_gripper_position(double target_pos)
  {
    const auto active_joints = gripper_move_group_->getActiveJoints();
    if (active_joints.empty()) {
      RCLCPP_WARN(this->get_logger(), "Gripper group has no active joints.");
      return false;
    }

    std::map<std::string, double> joint_targets;
    for (const auto & joint_name : active_joints) {
      if (joint_name.find("left_finger") != std::string::npos) {
        joint_targets[joint_name] = std::abs(target_pos);
      } else if (joint_name.find("right_finger") != std::string::npos) {
        joint_targets[joint_name] = -std::abs(target_pos);
      } else {
        joint_targets[joint_name] = target_pos;
      }
    }

    if (!gripper_move_group_->setJointValueTarget(joint_targets)) {
      RCLCPP_WARN(this->get_logger(), "Failed to set gripper joint value target.");
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool ok = gripper_move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (!ok) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to plan gripper joint value target at %.4f.",
        target_pos);
      return false;
    }
    if (preview_only_) {
      publish_preview(plan);
      return true;
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

  bool execute_arm_joint_target(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & joint_positions)
  {
    if (joint_names.empty() || joint_names.size() != joint_positions.size()) {
      RCLCPP_WARN(this->get_logger(), "Return arm joint target config is empty or mismatched.");
      return false;
    }

    std::map<std::string, double> joint_targets;
    for (size_t i = 0; i < joint_names.size(); ++i) {
      joint_targets[joint_names[i]] = joint_positions[i];
    }

    if (!arm_move_group_->setJointValueTarget(joint_targets)) {
      RCLCPP_WARN(this->get_logger(), "Failed to set arm joint value target for return pose.");
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool ok = arm_move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (!ok) {
      RCLCPP_WARN(this->get_logger(), "Failed to plan configured return arm pose.");
      return false;
    }
    if (preview_only_) {
      publish_preview(plan);
      return true;
    }
    return arm_move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
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
        maybe_preview_delay();
        goal_handle->succeed(result);
        goal_active_.store(false);
        return;
      }

      if (command == "return_arm_pose" || command == "place_arm_pose") {
        feedback->stage = command;
        feedback->position_error_m = 0.0f;
        goal_handle->publish_feedback(feedback);

        const auto & joint_names = (command == "place_arm_pose") ? place_joint_names_ : return_joint_names_;
        const auto & joint_positions =
          (command == "place_arm_pose") ? place_joint_positions_ : return_joint_positions_;

        if (!execute_arm_joint_target(joint_names, joint_positions)) {
          result->success = false;
          result->message =
            (command == "place_arm_pose") ?
            "Failed to execute configured place arm pose" :
            "Failed to execute configured return arm pose";
          result->final_position_error_m = -1.0f;
          goal_handle->abort(result);
          goal_active_.store(false);
          return;
        }

        result->success = true;
        result->message =
          (command == "place_arm_pose") ? "Place arm pose complete" : "Return arm pose complete";
        result->final_position_error_m = 0.0f;
        maybe_preview_delay();
        goal_handle->succeed(result);
        goal_active_.store(false);
        return;
      }

      feedback->stage = "planning_arm";
      feedback->position_error_m = 0.0f;
      goal_handle->publish_feedback(feedback);

      const auto target_pose = transform_pose_to_planning_frame(goal->target_pose);
      if (!plan_and_execute_arm(target_pose, goal->ee_link, goal->use_cartesian_approach)) {
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
      maybe_preview_delay();
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
  double cartesian_eef_step_m_;
  double cartesian_min_fraction_;
  bool preview_only_;
  double preview_step_delay_sec_;
  double open_gripper_pos_;
  double closed_gripper_pos_;
  std::vector<std::string> return_joint_names_;
  std::vector<double> return_joint_positions_;
  std::vector<std::string> place_joint_names_;
  std::vector<double> place_joint_positions_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_traj_pub_;

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
