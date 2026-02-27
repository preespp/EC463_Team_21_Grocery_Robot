#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "robot_interfaces/action/pick_arm.hpp"

class ArmActionServer : public rclcpp::Node
{
public:
  using PickArm = robot_interfaces::action::PickArm;
  using GoalHandlePickArm = rclcpp_action::ServerGoalHandle<PickArm>;

  ArmActionServer()
  : Node("arm_controller"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    goal_active_(false)
  {
    action_name_ = this->declare_parameter<std::string>("action_name", "pick_arm");
    planning_group_ = this->declare_parameter<std::string>("planning_group", "arm");
    default_ee_link_ = this->declare_parameter<std::string>("ee_link", "gripper_base");

    traj_topic_ = this->declare_parameter<std::string>(
      "joint_trajectory_topic", "/arm/joint_trajectory_cmd");

    controlled_joints_ = this->declare_parameter<std::vector<std::string>>(
      "controlled_joints",
      {
        "joint1_base_yaw",
        "joint2_shoulder",
        "joint3_elbow",
        "joint4_wrist_roll",
        "joint5_gripper"
      });

    default_pregrasp_offset_m_ = this->declare_parameter<double>("default_pregrasp_offset_m", 0.10);
    default_retreat_offset_m_ = this->declare_parameter<double>("default_retreat_offset_m", 0.10);
    correction_threshold_m_ = this->declare_parameter<double>("correction_threshold_m", 0.02);
    correction_target_frame_ = this->declare_parameter<std::string>("correction_target_frame", "");

    planning_time_sec_ = this->declare_parameter<double>("planning_time_sec", 3.0);
    max_velocity_scaling_ = this->declare_parameter<double>("max_velocity_scaling", 0.25);
    max_acceleration_scaling_ = this->declare_parameter<double>("max_acceleration_scaling", 0.25);

    cartesian_eef_step_m_ = this->declare_parameter<double>("cartesian_eef_step_m", 0.01);
    cartesian_min_fraction_ = this->declare_parameter<double>("cartesian_min_fraction", 0.9);
    execute_wait_padding_sec_ = this->declare_parameter<double>("execute_wait_padding_sec", 0.25);

    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(traj_topic_, 10);

    action_server_ = rclcpp_action::create_server<PickArm>(
      this,
      action_name_,
      std::bind(&ArmActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ArmActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ArmActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "arm_controller ready. Call initialize_moveit() from main.");
  }

  void initialize_moveit()
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(),
      planning_group_);

    move_group_->setPlanningTime(planning_time_sec_);
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);

    if (!default_ee_link_.empty()) {
      move_group_->setEndEffectorLink(default_ee_link_);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "MoveIt initialized (group=%s, frame=%s, ee=%s)",
      planning_group_.c_str(),
      move_group_->getPlanningFrame().c_str(),
      move_group_->getEndEffectorLink().c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PickArm::Goal> goal)
  {
    if (goal_active_.load()) {
      RCLCPP_WARN(this->get_logger(), "Arm goal rejected: another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->target_pose.header.frame_id.empty()) {
      RCLCPP_WARN(this->get_logger(), "Arm goal rejected: target_pose.frame_id is empty");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePickArm>)
  {
    RCLCPP_INFO(this->get_logger(), "Arm goal cancellation requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePickArm> goal_handle)
  {
    goal_active_.store(true);
    std::thread{std::bind(&ArmActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePickArm> goal_handle)
  {
    auto result = std::make_shared<PickArm::Result>();
    auto feedback = std::make_shared<PickArm::Feedback>();

    if (!move_group_) {
      result->success = false;
      result->message = "MoveIt not initialized";
      result->final_position_error_m = -1.0f;
      goal_handle->abort(result);
      goal_active_.store(false);
      return;
    }

    try {
      const auto goal = goal_handle->get_goal();

      if (!goal->planning_group.empty() && goal->planning_group != planning_group_) {
        result->success = false;
        result->message = "planning_group mismatch. Restart node with matching parameter.";
        result->final_position_error_m = -1.0f;
        goal_handle->abort(result);
        goal_active_.store(false);
        return;
      }

      const auto ee_link = goal->ee_link.empty() ? move_group_->getEndEffectorLink() : goal->ee_link;
      const auto planning_frame = move_group_->getPlanningFrame();

      auto target_pose = transform_pose(goal->target_pose, planning_frame);

      if (!correction_target_frame_.empty()) {
        apply_tf_correction(target_pose, planning_frame, feedback, goal_handle);
      }

      const double pregrasp_offset =
        (goal->pregrasp_offset_m > 0.0f) ? goal->pregrasp_offset_m : default_pregrasp_offset_m_;
      const double retreat_offset =
        (goal->retreat_offset_m > 0.0f) ? goal->retreat_offset_m : default_retreat_offset_m_;

      const auto pregrasp_pose = offset_pose_along_tool_z(target_pose, -pregrasp_offset);

      feedback->stage = "planning_pregrasp";
      goal_handle->publish_feedback(feedback);

      trajectory_msgs::msg::JointTrajectory pregrasp_traj;
      if (!plan_to_pose(pregrasp_pose, ee_link, pregrasp_traj)) {
        result->success = false;
        result->message = "Failed to plan pre-grasp";
        result->final_position_error_m = -1.0f;
        goal_handle->abort(result);
        goal_active_.store(false);
        return;
      }

      if (!publish_and_wait(pregrasp_traj, goal_handle, feedback, "executing_pregrasp")) {
        result->success = false;
        result->message = "Canceled during pre-grasp execution";
        result->final_position_error_m = -1.0f;
        goal_handle->canceled(result);
        goal_active_.store(false);
        return;
      }

      trajectory_msgs::msg::JointTrajectory approach_traj;
      bool approach_ok = false;

      feedback->stage = "planning_approach";
      goal_handle->publish_feedback(feedback);

      if (goal->use_cartesian_approach) {
        approach_ok = plan_cartesian_to_pose(target_pose, approach_traj);
      } else {
        approach_ok = plan_to_pose(target_pose, ee_link, approach_traj);
      }

      if (!approach_ok) {
        result->success = false;
        result->message = "Failed to plan approach";
        result->final_position_error_m = -1.0f;
        goal_handle->abort(result);
        goal_active_.store(false);
        return;
      }

      if (!publish_and_wait(approach_traj, goal_handle, feedback, "executing_approach")) {
        result->success = false;
        result->message = "Canceled during approach execution";
        result->final_position_error_m = -1.0f;
        goal_handle->canceled(result);
        goal_active_.store(false);
        return;
      }

      if (goal->gripper_close_position >= 0.0f) {
        feedback->stage = "closing_gripper";
        goal_handle->publish_feedback(feedback);

        auto gripper_traj = build_gripper_trajectory(goal->gripper_close_position);
        if (!publish_and_wait(gripper_traj, goal_handle, feedback, "executing_gripper")) {
          result->success = false;
          result->message = "Canceled during gripper close";
          result->final_position_error_m = -1.0f;
          goal_handle->canceled(result);
          goal_active_.store(false);
          return;
        }
      }

      const auto retreat_pose = offset_pose_along_tool_z(target_pose, retreat_offset);
      trajectory_msgs::msg::JointTrajectory retreat_traj;

      feedback->stage = "planning_retreat";
      goal_handle->publish_feedback(feedback);

      if (!plan_to_pose(retreat_pose, ee_link, retreat_traj)) {
        result->success = false;
        result->message = "Failed to plan retreat";
        result->final_position_error_m = -1.0f;
        goal_handle->abort(result);
        goal_active_.store(false);
        return;
      }

      if (!publish_and_wait(retreat_traj, goal_handle, feedback, "executing_retreat")) {
        result->success = false;
        result->message = "Canceled during retreat execution";
        result->final_position_error_m = -1.0f;
        goal_handle->canceled(result);
        goal_active_.store(false);
        return;
      }

      feedback->stage = "completed";
      feedback->position_error_m = 0.0f;
      goal_handle->publish_feedback(feedback);

      result->success = true;
      result->message = "Pick sequence complete";
      result->final_position_error_m = feedback->position_error_m;
      goal_handle->succeed(result);

    } catch (const std::exception & ex) {
      result->success = false;
      result->message = ex.what();
      result->final_position_error_m = -1.0f;
      goal_handle->abort(result);
    }

    goal_active_.store(false);
  }

  geometry_msgs::msg::PoseStamped transform_pose(
    const geometry_msgs::msg::PoseStamped & in,
    const std::string & target_frame)
  {
    if (in.header.frame_id == target_frame) {
      return in;
    }

    return tf_buffer_.transform(in, target_frame, tf2::durationFromSec(0.25));
  }

  void apply_tf_correction(
    geometry_msgs::msg::PoseStamped & target_pose,
    const std::string & planning_frame,
    const std::shared_ptr<PickArm::Feedback> & feedback,
    const std::shared_ptr<GoalHandlePickArm> & goal_handle)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(
        planning_frame,
        correction_target_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF correction skipped: %s", ex.what());
      return;
    }

    const double dx = tf_msg.transform.translation.x - target_pose.pose.position.x;
    const double dy = tf_msg.transform.translation.y - target_pose.pose.position.y;
    const double dz = tf_msg.transform.translation.z - target_pose.pose.position.z;
    const double err = std::sqrt(dx * dx + dy * dy + dz * dz);

    feedback->stage = "tf_correction";
    feedback->position_error_m = static_cast<float>(err);
    goal_handle->publish_feedback(feedback);

    if (err > correction_threshold_m_) {
      target_pose.pose.position.x = tf_msg.transform.translation.x;
      target_pose.pose.position.y = tf_msg.transform.translation.y;
      target_pose.pose.position.z = tf_msg.transform.translation.z;
      RCLCPP_INFO(this->get_logger(), "Applied TF correction (%.3f m)", err);
    }
  }

  geometry_msgs::msg::PoseStamped offset_pose_along_tool_z(
    const geometry_msgs::msg::PoseStamped & in,
    double offset_m) const
  {
    geometry_msgs::msg::PoseStamped out = in;

    tf2::Quaternion q;
    tf2::fromMsg(in.pose.orientation, q);
    tf2::Matrix3x3 basis(q);
    const auto z_axis = basis.getColumn(2);

    out.pose.position.x += offset_m * z_axis.x();
    out.pose.position.y += offset_m * z_axis.y();
    out.pose.position.z += offset_m * z_axis.z();
    return out;
  }

  void set_start_state_from_last_command()
  {
    if (!have_last_commanded_state_) {
      move_group_->setStartStateToCurrentState();
      return;
    }

    moveit::core::RobotState state(move_group_->getRobotModel());
    state.setToDefaultValues();
    const auto & variable_names = state.getVariableNames();

    for (const auto & kv : last_commanded_state_) {
      if (std::find(variable_names.begin(), variable_names.end(), kv.first) != variable_names.end()) {
        state.setVariablePosition(kv.first, kv.second);
      }
    }

    move_group_->setStartState(state);
  }

  bool plan_to_pose(
    const geometry_msgs::msg::PoseStamped & target,
    const std::string & ee_link,
    trajectory_msgs::msg::JointTrajectory & out)
  {
    set_start_state_from_last_command();
    move_group_->setPoseTarget(target.pose, ee_link);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool ok = move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;

    move_group_->clearPoseTargets();
    if (!ok || plan.trajectory_.joint_trajectory.points.empty()) {
      return false;
    }

    out = plan.trajectory_.joint_trajectory;
    return true;
  }

  bool plan_cartesian_to_pose(
    const geometry_msgs::msg::PoseStamped & target,
    trajectory_msgs::msg::JointTrajectory & out)
  {
    set_start_state_from_last_command();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target.pose);

    moveit_msgs::msg::RobotTrajectory robot_traj;
    const double fraction = move_group_->computeCartesianPath(
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

    out = robot_traj.joint_trajectory;
    return true;
  }

  trajectory_msgs::msg::JointTrajectory build_gripper_trajectory(float gripper_position)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = controlled_joints_;

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.resize(controlled_joints_.size(), 0.0);

    for (size_t i = 0; i < controlled_joints_.size(); ++i) {
      const auto it = last_commanded_state_.find(controlled_joints_[i]);
      p.positions[i] = (it != last_commanded_state_.end()) ? it->second : 0.0;
    }

    for (size_t i = 0; i < controlled_joints_.size(); ++i) {
      if (controlled_joints_[i] == "joint5_gripper") {
        p.positions[i] = gripper_position;
      }
    }

    p.time_from_start.sec = 1;
    p.time_from_start.nanosec = 0;
    traj.points.push_back(p);
    return traj;
  }

  bool publish_and_wait(
    const trajectory_msgs::msg::JointTrajectory & traj,
    const std::shared_ptr<GoalHandlePickArm> & goal_handle,
    const std::shared_ptr<PickArm::Feedback> & feedback,
    const std::string & stage)
  {
    traj_pub_->publish(traj);
    update_last_commanded_state(traj);

    const auto & last_point = traj.points.back();
    const double sec =
      static_cast<double>(last_point.time_from_start.sec) +
      static_cast<double>(last_point.time_from_start.nanosec) * 1e-9 +
      execute_wait_padding_sec_;

    const auto end_time = std::chrono::steady_clock::now() + std::chrono::duration<double>(sec);
    rclcpp::Rate loop_rate(40.0);

    while (rclcpp::ok() && std::chrono::steady_clock::now() < end_time) {
      if (goal_handle->is_canceling()) {
        return false;
      }
      feedback->stage = stage;
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }

    return true;
  }

  void update_last_commanded_state(const trajectory_msgs::msg::JointTrajectory & traj)
  {
    if (traj.points.empty()) {
      return;
    }

    const auto & p = traj.points.back();
    const auto n = std::min(traj.joint_names.size(), p.positions.size());
    for (size_t i = 0; i < n; ++i) {
      last_commanded_state_[traj.joint_names[i]] = p.positions[i];
    }
    have_last_commanded_state_ = true;
  }

  std::string action_name_;
  std::string planning_group_;
  std::string default_ee_link_;
  std::string traj_topic_;
  std::string correction_target_frame_;

  std::vector<std::string> controlled_joints_;

  double default_pregrasp_offset_m_;
  double default_retreat_offset_m_;
  double correction_threshold_m_;
  double planning_time_sec_;
  double max_velocity_scaling_;
  double max_acceleration_scaling_;
  double cartesian_eef_step_m_;
  double cartesian_min_fraction_;
  double execute_wait_padding_sec_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  std::map<std::string, double> last_commanded_state_;
  bool have_last_commanded_state_ {false};

  std::atomic<bool> goal_active_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp_action::Server<PickArm>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ArmActionServer>();
  node->initialize_moveit();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
