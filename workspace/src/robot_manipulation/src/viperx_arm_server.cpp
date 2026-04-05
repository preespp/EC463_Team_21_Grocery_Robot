#include <algorithm>
#include <atomic>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit/robot_state/conversions.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
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
    enforce_orientation_path_constraint_ = this->declare_parameter<bool>(
      "enforce_orientation_path_constraint",
      false);
    enforce_orientation_path_constraint_on_pose_moves_ = this->declare_parameter<bool>(
      "enforce_orientation_path_constraint_on_pose_moves",
      false);
    use_position_target_with_orientation_constraint_ = this->declare_parameter<bool>(
      "use_position_target_with_orientation_constraint",
      true);
    allow_orientation_constraint_fallback_ = this->declare_parameter<bool>(
      "allow_orientation_constraint_fallback",
      false);
    orientation_constraint_x_tolerance_rad_ = this->declare_parameter<double>(
      "orientation_constraint_x_tolerance_rad",
      0.12);
    orientation_constraint_y_tolerance_rad_ = this->declare_parameter<double>(
      "orientation_constraint_y_tolerance_rad",
      0.12);
    orientation_constraint_z_tolerance_rad_ = this->declare_parameter<double>(
      "orientation_constraint_z_tolerance_rad",
      1.57);
    orientation_constraint_weight_ = this->declare_parameter<double>(
      "orientation_constraint_weight",
      1.0);
    preview_only_ = this->declare_parameter<bool>("preview_only", false);
    preview_step_delay_sec_ = this->declare_parameter<double>("preview_step_delay_sec", 1.0);
    verify_final_orientation_match_ = this->declare_parameter<bool>(
      "verify_final_orientation_match",
      false);
    orientation_check_settle_sec_ = this->declare_parameter<double>(
      "orientation_check_settle_sec",
      0.15);
    open_gripper_pos_ = this->declare_parameter<double>("open_gripper_pos", 0.035);
    closed_gripper_pos_ = this->declare_parameter<double>("closed_gripper_pos", 0.0);
    open_gripper_named_target_ = this->declare_parameter<std::string>(
      "open_gripper_named_target",
      "Released");
    close_gripper_named_target_ = this->declare_parameter<std::string>(
      "close_gripper_named_target",
      "Grasping");
    scan_center_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "scan_center_joint_names",
      std::vector<std::string>{
        "waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"});
    scan_center_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "scan_center_joint_positions",
      std::vector<double>{0.0, -1.34390352, 1.57079633, 0.0, -0.22689280, 0.0});
    scan_left_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "scan_left_joint_names",
      std::vector<std::string>{
        "waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"});
    scan_left_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "scan_left_joint_positions",
      std::vector<double>{0.78539816, -1.34390352, 1.57079633, 0.0, -0.22689280, 0.0});
    scan_right_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "scan_right_joint_names",
      std::vector<std::string>{
        "waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"});
    scan_right_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "scan_right_joint_positions",
      std::vector<double>{-0.78539816, -1.34390352, 1.57079633, 0.0, -0.22689280, 0.0});
    startup_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "startup_joint_names",
      std::vector<std::string>{
        "waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"});
    startup_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "startup_joint_positions",
      std::vector<double>{0.0, -1.85004901, 1.57079633, 0.0, 0.40142573, 0.0});
    restock_pick_ready_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "restock_pick_ready_joint_positions",
      std::vector<double>{0.0, -1.06465084, 1.02974426, 0.0, 0.54105207, 0.0});
    restock_pick_approach_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "restock_pick_approach_joint_positions",
      std::vector<double>{2.39110108, -1.23918377, 1.30899694, 1.46607657, -0.83775804, -1.43116999});
    restock_pick_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "restock_pick_joint_positions",
      std::vector<double>{1.98967535, 0.05235988, 1.55334303, 0.43633231, -1.62315620, 0.01745329});
    restock_post_pick_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "restock_post_pick_joint_positions",
      std::vector<double>{2.42600766, -1.23918377, 1.09955743, 1.65806279, -0.87266463, -1.72787596});
    restock_transfer_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "restock_transfer_joint_positions",
      std::vector<double>{0.12217305, -0.54105207, 0.85521133, 0.41887902, -0.33161256, -0.36651914});
    restock_place_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "restock_place_joint_positions",
      std::vector<double>{0.15707963, 0.19198622, 0.29670597, 0.29670597, -0.61086524, -0.22689280});
    restock_home_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "restock_home_joint_positions",
      std::vector<double>{0.0, -1.85004901, 1.57079633, 0.0, 0.73303829, 0.0});
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
      std::vector<double>{2.63544717, -0.43633231, 1.22173048, 1.23918377, -1.25663706, -0.76794487});
    place_2_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "place_2_joint_positions",
      std::vector<double>{3.14159265, -0.50614548, 1.25663706, 1.58824962, -1.60570291, -0.83775804});
    place_3_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "place_3_joint_positions",
      std::vector<double>{1.76278254, -1.01229097, 1.43116999, 0.40142573, -0.45378561, -0.38397244});
    place_4_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "place_4_joint_positions",
      std::vector<double>{3.05432619, -1.34390352, 1.48352986, 1.57079633, -1.51843645, -1.44862328});
    place_5_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "place_5_joint_positions",
      std::vector<double>{0.57595865, -0.68067841, 1.32645023, -1.22173048, -1.13446401, 0.82030475});
    place_6_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "place_6_joint_positions",
      std::vector<double>{-0.27925268, -0.75049158, 1.34390352, -1.74532925, -1.79768913, 0.90757121});
    post_place_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "post_place_joint_names",
      std::vector<std::string>{
        "waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"});
    post_place_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "post_place_joint_positions",
      std::vector<double>{1.81514242, -1.06465084, 1.16937060, 0.43633231, -0.15707963, -0.41887902});
    pre_return_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "pre_return_joint_names",
      std::vector<std::string>{
        "waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"});
    pre_return_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "pre_return_joint_positions",
      std::vector<double>{0.0, -1.37881011, 1.20427718, 0.0, 0.27925268, 0.0});
    lift_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "lift_joint_names",
      std::vector<std::string>{
        "waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"});
    lift_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "lift_joint_positions",
      std::vector<double>{0.0, -0.17453293, 1.39626340, 0.0, -1.20427718, 0.0});
    post_lift_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "post_lift_joint_names",
      std::vector<std::string>{
        "waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"});
    post_lift_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "post_lift_joint_positions",
      std::vector<double>{0.0, -0.82030475, 1.06465084, 0.0, -0.24434610, 0.0});

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
  static bool is_gripper_command(const std::string & command)
  {
    return command == "open_gripper" || command == "close_gripper";
  }

  static bool is_named_arm_command(const std::string & command)
  {
    return
      command == "scan_center_arm_pose" || command == "scan_left_arm_pose" ||
      command == "scan_right_arm_pose" ||
      command == "startup_arm_pose" || command == "return_arm_pose" ||
      command == "restock_pick_ready_arm_pose" ||
      command == "restock_pick_approach_arm_pose" ||
      command == "restock_pick_arm_pose" ||
      command == "restock_post_pick_arm_pose" ||
      command == "restock_transfer_arm_pose" ||
      command == "restock_place_arm_pose" ||
      command == "restock_home_arm_pose" ||
      command == "lift_arm_pose" || command == "post_lift_arm_pose" ||
      command == "place_arm_pose" || command == "place_arm_pose_1" ||
      command == "place_arm_pose_2" || command == "place_arm_pose_3" ||
      command == "place_arm_pose_4" || command == "place_arm_pose_5" ||
      command == "place_arm_pose_6" || command == "post_place_arm_pose" ||
      command == "pre_return_arm_pose";
  }

  static bool is_waist_delta_command(const std::string & command)
  {
    return command == "waist_delta_arm_pose";
  }

  static bool is_pose_goal_command(const std::string & command)
  {
    return command.empty() || command == "arm";
  }

  const std::vector<std::string> & joint_names_for_command(const std::string & command) const
  {
    if (command == "scan_center_arm_pose") {
      return scan_center_joint_names_;
    }
    if (command == "scan_left_arm_pose") {
      return scan_left_joint_names_;
    }
    if (command == "scan_right_arm_pose") {
      return scan_right_joint_names_;
    }
    if (command == "startup_arm_pose") {
      return startup_joint_names_;
    }
    if (
      command == "restock_pick_ready_arm_pose" ||
      command == "restock_pick_approach_arm_pose" ||
      command == "restock_pick_arm_pose" ||
      command == "restock_post_pick_arm_pose" ||
      command == "restock_transfer_arm_pose" ||
      command == "restock_place_arm_pose" ||
      command == "restock_home_arm_pose")
    {
      return startup_joint_names_;
    }
    if (command == "lift_arm_pose") {
      return lift_joint_names_;
    }
    if (command == "post_lift_arm_pose") {
      return post_lift_joint_names_;
    }
    if (
      command == "place_arm_pose" || command == "place_arm_pose_1" ||
      command == "place_arm_pose_2" || command == "place_arm_pose_3" ||
      command == "place_arm_pose_4" || command == "place_arm_pose_5" ||
      command == "place_arm_pose_6")
    {
      return place_joint_names_;
    }
    if (command == "post_place_arm_pose") {
      return post_place_joint_names_;
    }
    if (command == "pre_return_arm_pose") {
      return pre_return_joint_names_;
    }
    return return_joint_names_;
  }

  const std::vector<double> & joint_positions_for_command(const std::string & command) const
  {
    if (command == "scan_center_arm_pose") {
      return scan_center_joint_positions_;
    }
    if (command == "scan_left_arm_pose") {
      return scan_left_joint_positions_;
    }
    if (command == "scan_right_arm_pose") {
      return scan_right_joint_positions_;
    }
    if (command == "startup_arm_pose") {
      return startup_joint_positions_;
    }
    if (command == "restock_pick_ready_arm_pose") {
      return restock_pick_ready_joint_positions_;
    }
    if (command == "restock_pick_approach_arm_pose") {
      return restock_pick_approach_joint_positions_;
    }
    if (command == "restock_pick_arm_pose") {
      return restock_pick_joint_positions_;
    }
    if (command == "restock_post_pick_arm_pose") {
      return restock_post_pick_joint_positions_;
    }
    if (command == "restock_transfer_arm_pose") {
      return restock_transfer_joint_positions_;
    }
    if (command == "restock_place_arm_pose") {
      return restock_place_joint_positions_;
    }
    if (command == "restock_home_arm_pose") {
      return restock_home_joint_positions_;
    }
    if (command == "lift_arm_pose") {
      return lift_joint_positions_;
    }
    if (command == "post_lift_arm_pose") {
      return post_lift_joint_positions_;
    }
    if (command == "place_arm_pose" || command == "place_arm_pose_1") {
      return place_joint_positions_;
    }
    if (command == "place_arm_pose_2") {
      return place_2_joint_positions_;
    }
    if (command == "place_arm_pose_3") {
      return place_3_joint_positions_;
    }
    if (command == "place_arm_pose_4") {
      return place_4_joint_positions_;
    }
    if (command == "place_arm_pose_5") {
      return place_5_joint_positions_;
    }
    if (command == "place_arm_pose_6") {
      return place_6_joint_positions_;
    }
    if (command == "post_place_arm_pose") {
      return post_place_joint_positions_;
    }
    if (command == "pre_return_arm_pose") {
      return pre_return_joint_positions_;
    }
    return return_joint_positions_;
  }

  std::string failure_message_for_named_command(const std::string & command) const
  {
    if (command == "scan_center_arm_pose") {
      return "Failed to execute configured center scan arm pose";
    }
    if (command == "scan_left_arm_pose") {
      return "Failed to execute configured left scan arm pose";
    }
    if (command == "scan_right_arm_pose") {
      return "Failed to execute configured right scan arm pose";
    }
    if (command == "startup_arm_pose") {
      return "Failed to execute configured startup arm pose";
    }
    if (command == "restock_pick_ready_arm_pose") {
      return "Failed to execute configured restock pick-ready arm pose";
    }
    if (command == "restock_pick_approach_arm_pose") {
      return "Failed to execute configured restock pick-approach arm pose";
    }
    if (command == "restock_pick_arm_pose") {
      return "Failed to execute configured restock pick arm pose";
    }
    if (command == "restock_post_pick_arm_pose") {
      return "Failed to execute configured restock post-pick arm pose";
    }
    if (command == "restock_transfer_arm_pose") {
      return "Failed to execute configured restock transfer arm pose";
    }
    if (command == "restock_place_arm_pose") {
      return "Failed to execute configured restock place arm pose";
    }
    if (command == "restock_home_arm_pose") {
      return "Failed to execute configured restock home arm pose";
    }
    if (command == "lift_arm_pose") {
      return "Failed to execute configured lift arm pose";
    }
    if (command == "post_lift_arm_pose") {
      return "Failed to execute configured post-lift arm pose";
    }
    if (command == "place_arm_pose" || command == "place_arm_pose_1") {
      return "Failed to execute configured basket 1 place arm pose";
    }
    if (command == "place_arm_pose_2") {
      return "Failed to execute configured basket 2 place arm pose";
    }
    if (command == "place_arm_pose_3") {
      return "Failed to execute configured basket 3 place arm pose";
    }
    if (command == "place_arm_pose_4") {
      return "Failed to execute configured basket 4 place arm pose";
    }
    if (command == "place_arm_pose_5") {
      return "Failed to execute configured basket 5 place arm pose";
    }
    if (command == "place_arm_pose_6") {
      return "Failed to execute configured basket 6 place arm pose";
    }
    if (command == "post_place_arm_pose") {
      return "Failed to execute configured post-place arm pose";
    }
    if (command == "pre_return_arm_pose") {
      return "Failed to execute configured pre-return arm pose";
    }
    return "Failed to execute configured return arm pose";
  }

  std::string success_message_for_named_command(const std::string & command) const
  {
    if (command == "scan_center_arm_pose") {
      return "Center scan arm pose complete";
    }
    if (command == "scan_left_arm_pose") {
      return "Left scan arm pose complete";
    }
    if (command == "scan_right_arm_pose") {
      return "Right scan arm pose complete";
    }
    if (command == "startup_arm_pose") {
      return "Startup arm pose complete";
    }
    if (command == "restock_pick_ready_arm_pose") {
      return "Restock pick-ready arm pose complete";
    }
    if (command == "restock_pick_approach_arm_pose") {
      return "Restock pick-approach arm pose complete";
    }
    if (command == "restock_pick_arm_pose") {
      return "Restock pick arm pose complete";
    }
    if (command == "restock_post_pick_arm_pose") {
      return "Restock post-pick arm pose complete";
    }
    if (command == "restock_transfer_arm_pose") {
      return "Restock transfer arm pose complete";
    }
    if (command == "restock_place_arm_pose") {
      return "Restock place arm pose complete";
    }
    if (command == "restock_home_arm_pose") {
      return "Restock home arm pose complete";
    }
    if (command == "lift_arm_pose") {
      return "Lift arm pose complete";
    }
    if (command == "post_lift_arm_pose") {
      return "Post-lift arm pose complete";
    }
    if (command == "place_arm_pose" || command == "place_arm_pose_1") {
      return "Basket 1 place arm pose complete";
    }
    if (command == "place_arm_pose_2") {
      return "Basket 2 place arm pose complete";
    }
    if (command == "place_arm_pose_3") {
      return "Basket 3 place arm pose complete";
    }
    if (command == "place_arm_pose_4") {
      return "Basket 4 place arm pose complete";
    }
    if (command == "place_arm_pose_5") {
      return "Basket 5 place arm pose complete";
    }
    if (command == "place_arm_pose_6") {
      return "Basket 6 place arm pose complete";
    }
    if (command == "post_place_arm_pose") {
      return "Post-place arm pose complete";
    }
    if (command == "pre_return_arm_pose") {
      return "Pre-return arm pose complete";
    }
    return "Return arm pose complete";
  }

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
      is_gripper_command(command) || is_named_arm_command(command) ||
      is_waist_delta_command(command))
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    if (!is_pose_goal_command(command)) {
      RCLCPP_WARN(
        this->get_logger(),
        "ViperX goal rejected: unsupported planning_group '%s'",
        command.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->target_pose.header.frame_id.empty()) {
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

  moveit_msgs::msg::Constraints make_orientation_constraints(
    const geometry_msgs::msg::PoseStamped & target_pose,
    const std::string & target_link) const
  {
    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    orientation_constraint.header.frame_id = target_pose.header.frame_id;
    orientation_constraint.link_name = target_link;
    orientation_constraint.orientation = target_pose.pose.orientation;
    orientation_constraint.absolute_x_axis_tolerance = orientation_constraint_x_tolerance_rad_;
    orientation_constraint.absolute_y_axis_tolerance = orientation_constraint_y_tolerance_rad_;
    orientation_constraint.absolute_z_axis_tolerance = orientation_constraint_z_tolerance_rad_;
    orientation_constraint.weight = orientation_constraint_weight_;

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(orientation_constraint);
    return path_constraints;
  }

  static double shortest_angle_distance(double from, double to)
  {
    return std::atan2(std::sin(to - from), std::cos(to - from));
  }

  bool pose_is_within_orientation_tolerance(
    const geometry_msgs::msg::PoseStamped & target_pose,
    const std::string & target_link)
  {
    if (target_link.empty()) {
      return true;
    }

    const auto actual_pose = arm_move_group_->getCurrentPose(target_link);

    tf2::Quaternion desired_q;
    desired_q.setX(target_pose.pose.orientation.x);
    desired_q.setY(target_pose.pose.orientation.y);
    desired_q.setZ(target_pose.pose.orientation.z);
    desired_q.setW(target_pose.pose.orientation.w);
    desired_q.normalize();

    tf2::Quaternion actual_q;
    actual_q.setX(actual_pose.pose.orientation.x);
    actual_q.setY(actual_pose.pose.orientation.y);
    actual_q.setZ(actual_pose.pose.orientation.z);
    actual_q.setW(actual_pose.pose.orientation.w);
    actual_q.normalize();

    double desired_roll = 0.0;
    double desired_pitch = 0.0;
    double desired_yaw = 0.0;
    tf2::Matrix3x3(desired_q).getRPY(desired_roll, desired_pitch, desired_yaw);

    double actual_roll = 0.0;
    double actual_pitch = 0.0;
    double actual_yaw = 0.0;
    tf2::Matrix3x3(actual_q).getRPY(actual_roll, actual_pitch, actual_yaw);

    const auto abs_roll_error =
      std::abs(shortest_angle_distance(desired_roll, actual_roll));
    const auto abs_pitch_error =
      std::abs(shortest_angle_distance(desired_pitch, actual_pitch));
    const auto abs_yaw_error =
      std::abs(shortest_angle_distance(desired_yaw, actual_yaw));

    if (
      abs_roll_error <= orientation_constraint_x_tolerance_rad_ &&
      abs_pitch_error <= orientation_constraint_y_tolerance_rad_ &&
      abs_yaw_error <= orientation_constraint_z_tolerance_rad_)
    {
      return true;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "End-effector orientation drift for link '%s': roll=%.3f pitch=%.3f yaw=%.3f rad",
      target_link.c_str(),
      abs_roll_error,
      abs_pitch_error,
      abs_yaw_error);
    return false;
  }

  double pose_goal_orientation_tolerance_rad() const
  {
    return std::min(
      {std::max(0.0, orientation_constraint_x_tolerance_rad_),
        std::max(0.0, orientation_constraint_y_tolerance_rad_),
        std::max(0.0, orientation_constraint_z_tolerance_rad_)});
  }

  bool plan_and_execute_arm(
    const geometry_msgs::msg::PoseStamped & target_pose,
    const std::string & ee_link,
    bool use_cartesian,
    bool require_orientation_match)
  {
    last_arm_failure_reason_.clear();
    const std::string target_link =
      !ee_link.empty() ? ee_link : arm_move_group_->getEndEffectorLink();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (use_cartesian) {
      arm_move_group_->setStartStateToCurrentState();
      const auto previous_pose_reference_frame = arm_move_group_->getPoseReferenceFrame();
      const auto previous_end_effector_link = arm_move_group_->getEndEffectorLink();
      const auto restore_move_group_context = [&]() {
        arm_move_group_->setPoseReferenceFrame(previous_pose_reference_frame);
        if (
          !previous_end_effector_link.empty() &&
          previous_end_effector_link != arm_move_group_->getEndEffectorLink())
        {
          arm_move_group_->setEndEffectorLink(previous_end_effector_link);
        }
      };

      arm_move_group_->setPoseReferenceFrame(target_pose.header.frame_id);
      if (!target_link.empty() && target_link != arm_move_group_->getEndEffectorLink()) {
        if (!arm_move_group_->setEndEffectorLink(target_link)) {
          RCLCPP_WARN(
            this->get_logger(),
            "Failed to set Cartesian end-effector link to '%s'.",
            target_link.c_str());
          last_arm_failure_reason_ = "Failed to set Cartesian end-effector link";
          restore_move_group_context();
          return false;
        }
      }

      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(target_pose.pose);

      moveit_msgs::msg::RobotTrajectory robot_traj;
      const bool use_orientation_constraint =
        require_orientation_match && enforce_orientation_path_constraint_ && !target_link.empty();
      const auto required_fraction = use_orientation_constraint ?
        std::max(cartesian_min_fraction_, 0.999) : cartesian_min_fraction_;
      const double fraction = use_orientation_constraint ?
        arm_move_group_->computeCartesianPath(
          waypoints,
          cartesian_eef_step_m_,
          0.0,
          robot_traj,
          make_orientation_constraints(target_pose, target_link),
          true) :
        arm_move_group_->computeCartesianPath(
          waypoints,
          cartesian_eef_step_m_,
          0.0,
          robot_traj,
          true);
      restore_move_group_context();
      if (fraction < required_fraction || robot_traj.joint_trajectory.points.empty()) {
        RCLCPP_WARN(
          this->get_logger(),
          "Cartesian path fraction too low: %.3f < %.3f",
          fraction,
          required_fraction);
        last_arm_failure_reason_ = "Failed to plan Cartesian arm pose";
        return false;
      }
      plan.trajectory_ = robot_traj;
    } else {
      arm_move_group_->setStartStateToCurrentState();
      const auto plan_pose_target = [&](bool use_orientation_constraint) -> bool {
        bool path_constraints_set = false;
        bool position_target_set = false;
        const bool use_goal_orientation_tolerance =
          require_orientation_match && !use_orientation_constraint;
        const double previous_goal_orientation_tolerance =
          arm_move_group_->getGoalOrientationTolerance();
        const double goal_orientation_tolerance =
          pose_goal_orientation_tolerance_rad();

        if (use_goal_orientation_tolerance) {
          arm_move_group_->setGoalOrientationTolerance(goal_orientation_tolerance);
          if (
            std::abs(orientation_constraint_x_tolerance_rad_ - goal_orientation_tolerance) > 1e-9 ||
            std::abs(orientation_constraint_y_tolerance_rad_ - goal_orientation_tolerance) > 1e-9 ||
            std::abs(orientation_constraint_z_tolerance_rad_ - goal_orientation_tolerance) > 1e-9)
          {
            RCLCPP_INFO(
              this->get_logger(),
              "Using goal-only orientation tolerance %.3f rad for pose planning "
              "(MoveIt goal tolerance is shared across roll/pitch/yaw; configured x=%.3f y=%.3f z=%.3f).",
              goal_orientation_tolerance,
              orientation_constraint_x_tolerance_rad_,
              orientation_constraint_y_tolerance_rad_,
              orientation_constraint_z_tolerance_rad_);
          }
        }

        if (use_orientation_constraint && !target_link.empty()) {
          arm_move_group_->setPathConstraints(
            make_orientation_constraints(target_pose, target_link));
          path_constraints_set = true;
        }

        arm_move_group_->setPoseReferenceFrame(target_pose.header.frame_id);
        if (!ee_link.empty()) {
          arm_move_group_->setPoseTarget(target_pose.pose, ee_link);
        } else {
          arm_move_group_->setPoseTarget(target_pose.pose);
        }

        if (
          use_orientation_constraint &&
          use_position_target_with_orientation_constraint_ &&
          !target_link.empty())
        {
          position_target_set = arm_move_group_->setPositionTarget(
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z,
            target_link);
          if (!position_target_set) {
            RCLCPP_WARN(
              this->get_logger(),
              "Failed to set constrained position target for link '%s'.",
              target_link.c_str());
          }
        }

        const bool ok = arm_move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        arm_move_group_->clearPoseTargets();
        if (use_goal_orientation_tolerance) {
          arm_move_group_->setGoalOrientationTolerance(previous_goal_orientation_tolerance);
        }
        if (path_constraints_set) {
          arm_move_group_->clearPathConstraints();
        }
        if (use_orientation_constraint && !position_target_set && !target_link.empty()) {
          last_arm_failure_reason_ = "Failed to set constrained position target";
          return false;
        }
        return ok;
      };

      const bool use_orientation_constraint =
        require_orientation_match &&
        enforce_orientation_path_constraint_ &&
        enforce_orientation_path_constraint_on_pose_moves_ &&
        !target_link.empty();

      if (!plan_pose_target(use_orientation_constraint)) {
        if (last_arm_failure_reason_.empty()) {
          last_arm_failure_reason_ = "Failed to plan arm pose";
        }
        return false;
      }
    }
    if (preview_only_) {
      publish_preview(plan);
      return true;
    }
    const bool execute_ok =
      arm_move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (!execute_ok) {
      last_arm_failure_reason_ = "Failed to execute arm pose";
      return false;
    }
    if (!require_orientation_match || !verify_final_orientation_match_) {
      return true;
    }
    if (orientation_check_settle_sec_ > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(orientation_check_settle_sec_));
    }
    if (!pose_is_within_orientation_tolerance(target_pose, target_link)) {
      last_arm_failure_reason_ = "Pose exceeded orientation tolerance";
      return false;
    }
    return true;
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
      const double open_target =
        (gripper_close_position >= 0.0f) ? static_cast<double>(gripper_close_position) :
        open_gripper_pos_;
      return execute_gripper_position(open_target);
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
      RCLCPP_WARN(this->get_logger(), "Arm joint target config is empty or mismatched.");
      return false;
    }

    std::map<std::string, double> joint_targets;
    for (size_t i = 0; i < joint_names.size(); ++i) {
      joint_targets[joint_names[i]] = joint_positions[i];
    }

    if (!arm_move_group_->setJointValueTarget(joint_targets)) {
      RCLCPP_WARN(this->get_logger(), "Failed to set arm joint value target.");
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool ok = arm_move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (!ok) {
      RCLCPP_WARN(this->get_logger(), "Failed to plan configured arm pose.");
      return false;
    }
    if (preview_only_) {
      publish_preview(plan);
      return true;
    }
    return arm_move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool execute_waist_delta(double delta_rad)
  {
    if (std::abs(delta_rad) < 1e-6) {
      RCLCPP_INFO(this->get_logger(), "Waist centering delta is near zero; skipping motion.");
      return true;
    }

    arm_move_group_->setStartStateToCurrentState();

    const auto joint_names = arm_move_group_->getActiveJoints();
    const auto joint_values = arm_move_group_->getCurrentJointValues();
    if (joint_names.empty() || joint_names.size() != joint_values.size()) {
      RCLCPP_WARN(this->get_logger(), "Unable to read current arm joints for waist-only move.");
      return false;
    }

    std::map<std::string, double> joint_targets;
    bool found_waist = false;
    for (size_t i = 0; i < joint_names.size(); ++i) {
      double target_value = joint_values[i];
      if (joint_names[i].find("waist") != std::string::npos) {
        target_value += delta_rad;
        found_waist = true;
      }
      joint_targets[joint_names[i]] = target_value;
    }

    if (!found_waist) {
      RCLCPP_WARN(this->get_logger(), "Active arm joints do not contain a waist joint.");
      return false;
    }

    if (!arm_move_group_->setJointValueTarget(joint_targets)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to set waist-only arm target (delta=%.3f rad).",
        delta_rad);
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool ok = arm_move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (!ok) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to plan waist-only arm target (delta=%.3f rad).",
        delta_rad);
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

      if (is_gripper_command(command)) {
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

      if (is_named_arm_command(command)) {
        feedback->stage = command;
        feedback->position_error_m = 0.0f;
        goal_handle->publish_feedback(feedback);

        const auto & joint_names = joint_names_for_command(command);
        const auto & joint_positions = joint_positions_for_command(command);

        if (!execute_arm_joint_target(joint_names, joint_positions)) {
          result->success = false;
          result->message = failure_message_for_named_command(command);
          result->final_position_error_m = -1.0f;
          goal_handle->abort(result);
          goal_active_.store(false);
          return;
        }

        result->success = true;
        result->message = success_message_for_named_command(command);
        result->final_position_error_m = 0.0f;
        maybe_preview_delay();
        goal_handle->succeed(result);
        goal_active_.store(false);
        return;
      }

      if (is_waist_delta_command(command)) {
        feedback->stage = command;
        feedback->position_error_m = 0.0f;
        goal_handle->publish_feedback(feedback);

        if (!execute_waist_delta(static_cast<double>(goal->waist_delta_rad))) {
          result->success = false;
          result->message = "Failed to execute waist-only arm move";
          result->final_position_error_m = -1.0f;
          goal_handle->abort(result);
          goal_active_.store(false);
          return;
        }

        result->success = true;
        result->message = "Waist-only arm move complete";
        result->final_position_error_m = 0.0f;
        maybe_preview_delay();
        goal_handle->succeed(result);
        goal_active_.store(false);
        return;
      }

      if (!is_pose_goal_command(command)) {
        result->success = false;
        result->message = "Unsupported planning_group";
        result->final_position_error_m = -1.0f;
        goal_handle->abort(result);
        goal_active_.store(false);
        return;
      }

      feedback->stage = "planning_arm";
      feedback->position_error_m = 0.0f;
      goal_handle->publish_feedback(feedback);

      const auto target_pose = transform_pose_to_planning_frame(goal->target_pose);
      if (
        !plan_and_execute_arm(
          target_pose,
          goal->ee_link,
          goal->use_cartesian_approach,
          goal->require_orientation_match))
      {
        result->success = false;
        result->message = last_arm_failure_reason_.empty() ?
          "Failed to plan/execute arm pose" : last_arm_failure_reason_;
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
  bool enforce_orientation_path_constraint_;
  bool enforce_orientation_path_constraint_on_pose_moves_;
  bool use_position_target_with_orientation_constraint_;
  bool allow_orientation_constraint_fallback_;
  double orientation_constraint_x_tolerance_rad_;
  double orientation_constraint_y_tolerance_rad_;
  double orientation_constraint_z_tolerance_rad_;
  double orientation_constraint_weight_;
  bool preview_only_;
  double preview_step_delay_sec_;
  bool verify_final_orientation_match_;
  double orientation_check_settle_sec_;
  std::string last_arm_failure_reason_;
  double open_gripper_pos_;
  double closed_gripper_pos_;
  std::vector<std::string> scan_center_joint_names_;
  std::vector<double> scan_center_joint_positions_;
  std::vector<std::string> scan_left_joint_names_;
  std::vector<double> scan_left_joint_positions_;
  std::vector<std::string> scan_right_joint_names_;
  std::vector<double> scan_right_joint_positions_;
  std::vector<std::string> startup_joint_names_;
  std::vector<double> startup_joint_positions_;
  std::vector<double> restock_pick_ready_joint_positions_;
  std::vector<double> restock_pick_approach_joint_positions_;
  std::vector<double> restock_pick_joint_positions_;
  std::vector<double> restock_post_pick_joint_positions_;
  std::vector<double> restock_transfer_joint_positions_;
  std::vector<double> restock_place_joint_positions_;
  std::vector<double> restock_home_joint_positions_;
  std::vector<std::string> return_joint_names_;
  std::vector<double> return_joint_positions_;
  std::vector<std::string> place_joint_names_;
  std::vector<double> place_joint_positions_;
  std::vector<double> place_2_joint_positions_;
  std::vector<double> place_3_joint_positions_;
  std::vector<double> place_4_joint_positions_;
  std::vector<double> place_5_joint_positions_;
  std::vector<double> place_6_joint_positions_;
  std::vector<std::string> post_place_joint_names_;
  std::vector<double> post_place_joint_positions_;
  std::vector<std::string> pre_return_joint_names_;
  std::vector<double> pre_return_joint_positions_;
  std::vector<std::string> lift_joint_names_;
  std::vector<double> lift_joint_positions_;
  std::vector<std::string> post_lift_joint_names_;
  std::vector<double> post_lift_joint_positions_;

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
