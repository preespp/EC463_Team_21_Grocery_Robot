#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "robot_interfaces/action/pick_arm.hpp"

class ArmDemoController : public rclcpp::Node
{
public:
  using PickArm = robot_interfaces::action::PickArm;
  using GoalHandlePickArm = rclcpp_action::ServerGoalHandle<PickArm>;

  ArmDemoController()
  : Node("arm_demo_controller"), goal_active_(false)
  {
    action_name_ = this->declare_parameter<std::string>("action_name", "pick_arm_demo");
    traj_topic_ = this->declare_parameter<std::string>(
      "joint_trajectory_topic", "/arm/joint_trajectory_cmd");

    controlled_joints_ = this->declare_parameter<std::vector<std::string>>(
      "controlled_joints",
      {
        "joint1_base_yaw",
        "joint2_shoulder",
        "joint3_elbow"
      });

    std::vector<double> default_joint_min;
    std::vector<double> default_joint_max;
    default_joint_min.reserve(controlled_joints_.size());
    default_joint_max.reserve(controlled_joints_.size());
    for (const auto & name : controlled_joints_) {
      if (name == "joint1_base_yaw") {
        default_joint_min.push_back(-3.14159);
        default_joint_max.push_back(3.14159);
      } else if (name == "joint2_shoulder") {
        default_joint_min.push_back(-1.8);
        default_joint_max.push_back(1.8);
      } else if (name == "joint3_elbow") {
        default_joint_min.push_back(0.0);
        default_joint_max.push_back(2.6);
      } else if (name == "joint4_wrist_roll") {
        default_joint_min.push_back(-3.14159);
        default_joint_max.push_back(3.14159);
      } else if (name == "joint5_gripper") {
        default_joint_min.push_back(0.0);
        default_joint_max.push_back(1.0);
      } else {
        default_joint_min.push_back(-3.14159);
        default_joint_max.push_back(3.14159);
      }
    }

    joint_min_ = this->declare_parameter<std::vector<double>>("joint_min", default_joint_min);
    joint_max_ = this->declare_parameter<std::vector<double>>("joint_max", default_joint_max);

    link1_length_m_ = this->declare_parameter<double>("link1_length_m", 0.27);
    link2_length_m_ = this->declare_parameter<double>("link2_length_m", 0.27);
    base_height_m_ = this->declare_parameter<double>("base_height_m", 0.08);

    point_duration_sec_ = this->declare_parameter<double>("point_duration_sec", 1.5);
    wait_padding_sec_ = this->declare_parameter<double>("wait_padding_sec", 0.2);
    default_pregrasp_offset_m_ = this->declare_parameter<double>("default_pregrasp_offset_m", 0.08);
    default_retreat_offset_m_ = this->declare_parameter<double>("default_retreat_offset_m", 0.10);
    open_gripper_pos_ = this->declare_parameter<double>("open_gripper_pos", 1.0);
    closed_gripper_pos_ = this->declare_parameter<double>("closed_gripper_pos", 0.0);

    validate_config_sizes();

    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(traj_topic_, 10);

    action_server_ = rclcpp_action::create_server<PickArm>(
      this,
      action_name_,
      std::bind(&ArmDemoController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ArmDemoController::handle_cancel, this, std::placeholders::_1),
      std::bind(&ArmDemoController::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "arm_demo_controller ready (action=%s, traj_topic=%s)",
      action_name_.c_str(),
      traj_topic_.c_str());
  }

private:
  void validate_config_sizes() const
  {
    if (joint_min_.size() != controlled_joints_.size()) {
      throw std::runtime_error("joint_min size must match controlled_joints size");
    }
    if (joint_max_.size() != controlled_joints_.size()) {
      throw std::runtime_error("joint_max size must match controlled_joints size");
    }
    if (link1_length_m_ <= 0.0 || link2_length_m_ <= 0.0) {
      throw std::runtime_error("link lengths must be > 0");
    }
  }

  bool has_joint(const std::string & name) const
  {
    return std::find(controlled_joints_.begin(), controlled_joints_.end(), name) != controlled_joints_.end();
  }

  std::vector<double> solve_joint_targets(
    double x,
    double y,
    double z,
    double gripper_override) const
  {
    const double yaw = std::atan2(y, x);
    const double r = std::sqrt(x * x + y * y);
    const double z_rel = z - base_height_m_;

    const double d = std::sqrt(r * r + z_rel * z_rel);
    const double d_min = std::max(std::fabs(link1_length_m_ - link2_length_m_) + 1e-4, 1e-4);
    const double d_max = std::max(link1_length_m_ + link2_length_m_ - 1e-4, d_min + 1e-4);
    const double d_use = std::clamp(d, d_min, d_max);

    const double phi = std::atan2(z_rel, std::max(r, 1e-6));
    double cos_elbow =
      (d_use * d_use - link1_length_m_ * link1_length_m_ - link2_length_m_ * link2_length_m_) /
      (2.0 * link1_length_m_ * link2_length_m_);
    cos_elbow = std::clamp(cos_elbow, -1.0, 1.0);
    const double elbow = std::acos(cos_elbow);
    const double shoulder = phi - std::atan2(
      link2_length_m_ * std::sin(elbow),
      link1_length_m_ + link2_length_m_ * std::cos(elbow));

    std::vector<double> joints(controlled_joints_.size(), 0.0);
    for (size_t i = 0; i < controlled_joints_.size(); ++i) {
      const auto & name = controlled_joints_[i];
      if (name == "joint1_base_yaw") {
        joints[i] = yaw;
      } else if (name == "joint2_shoulder") {
        joints[i] = shoulder;
      } else if (name == "joint3_elbow") {
        joints[i] = elbow;
      } else if (name == "joint5_gripper") {
        joints[i] = (gripper_override >= 0.0) ? gripper_override : open_gripper_pos_;
      } else {
        joints[i] = 0.0;
      }

      joints[i] = std::clamp(joints[i], joint_min_[i], joint_max_[i]);
    }

    return joints;
  }

  trajectory_msgs::msg::JointTrajectory make_trajectory(
    const std::vector<double> & joints,
    double duration_sec) const
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = controlled_joints_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joints;

    const auto sec_part = static_cast<int32_t>(duration_sec);
    const auto nsec_part = static_cast<uint32_t>((duration_sec - static_cast<double>(sec_part)) * 1e9);
    point.time_from_start.sec = sec_part;
    point.time_from_start.nanosec = nsec_part;

    traj.points.push_back(point);
    return traj;
  }

  bool publish_and_wait(
    const trajectory_msgs::msg::JointTrajectory & traj,
    const std::shared_ptr<GoalHandlePickArm> & goal_handle,
    const std::shared_ptr<PickArm::Feedback> & feedback,
    const std::string & stage)
  {
    if (traj.points.empty()) {
      return false;
    }

    traj_pub_->publish(traj);

    const auto & p = traj.points.back();
    const double duration =
      static_cast<double>(p.time_from_start.sec) +
      static_cast<double>(p.time_from_start.nanosec) * 1e-9 +
      wait_padding_sec_;

    const auto end_time = std::chrono::steady_clock::now() + std::chrono::duration<double>(duration);
    rclcpp::Rate rate(40.0);
    size_t tick = 0;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end_time) {
      if (goal_handle->is_canceling()) {
        return false;
      }

      if ((tick % 10) == 0) {
        feedback->stage = stage;
        feedback->position_error_m = 0.0f;
        goal_handle->publish_feedback(feedback);
      }
      ++tick;
      rate.sleep();
    }
    return true;
  }

  bool execute_pose_stage(
    double x,
    double y,
    double z,
    double gripper_override,
    const std::string & stage,
    const std::shared_ptr<GoalHandlePickArm> & goal_handle,
    const std::shared_ptr<PickArm::Feedback> & feedback)
  {
    const auto joints = solve_joint_targets(x, y, z, gripper_override);
    const auto traj = make_trajectory(joints, point_duration_sec_);
    return publish_and_wait(traj, goal_handle, feedback, stage);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PickArm::Goal> goal)
  {
    if (goal_active_.load()) {
      RCLCPP_WARN(this->get_logger(), "Arm demo goal rejected: another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->target_pose.header.frame_id.empty()) {
      RCLCPP_WARN(this->get_logger(), "Arm demo goal rejected: target_pose.frame_id is empty");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePickArm>)
  {
    RCLCPP_INFO(this->get_logger(), "Arm demo cancellation requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePickArm> goal_handle)
  {
    goal_active_.store(true);
    std::thread{std::bind(&ArmDemoController::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePickArm> goal_handle)
  {
    auto result = std::make_shared<PickArm::Result>();
    auto feedback = std::make_shared<PickArm::Feedback>();

    try {
      const auto goal = goal_handle->get_goal();
      const auto & target = goal->target_pose.pose.position;

      const double pregrasp_offset =
        (goal->pregrasp_offset_m > 0.0f) ? goal->pregrasp_offset_m : default_pregrasp_offset_m_;
      const double retreat_offset =
        (goal->retreat_offset_m > 0.0f) ? goal->retreat_offset_m : default_retreat_offset_m_;

      const double xy_norm = std::sqrt(target.x * target.x + target.y * target.y);
      const double pregrasp_scale = (xy_norm > pregrasp_offset) ? ((xy_norm - pregrasp_offset) / xy_norm) : 0.5;
      const double retreat_scale = (xy_norm > retreat_offset) ? ((xy_norm - retreat_offset) / xy_norm) : 0.6;

      const double pre_x = target.x * pregrasp_scale;
      const double pre_y = target.y * pregrasp_scale;
      const double ret_x = target.x * retreat_scale;
      const double ret_y = target.y * retreat_scale;

      if (!execute_pose_stage(
          pre_x,
          pre_y,
          target.z,
          -1.0,
          "executing_pregrasp",
          goal_handle,
          feedback)) {
        result->success = false;
        result->message = "Canceled during pregrasp";
        result->final_position_error_m = -1.0f;
        goal_handle->canceled(result);
        goal_active_.store(false);
        return;
      }

      if (!execute_pose_stage(
          target.x,
          target.y,
          target.z,
          -1.0,
          "executing_approach",
          goal_handle,
          feedback)) {
        result->success = false;
        result->message = "Canceled during approach";
        result->final_position_error_m = -1.0f;
        goal_handle->canceled(result);
        goal_active_.store(false);
        return;
      }

      if (has_joint("joint5_gripper")) {
        const double close_pos =
          (goal->gripper_close_position >= 0.0f) ? goal->gripper_close_position : closed_gripper_pos_;
        if (!execute_pose_stage(
            target.x,
            target.y,
            target.z,
            close_pos,
            "executing_gripper_close",
            goal_handle,
            feedback)) {
          result->success = false;
          result->message = "Canceled during gripper close";
          result->final_position_error_m = -1.0f;
          goal_handle->canceled(result);
          goal_active_.store(false);
          return;
        }
      }

      if (!execute_pose_stage(
          ret_x,
          ret_y,
          target.z,
          -1.0,
          "executing_retreat",
          goal_handle,
          feedback)) {
        result->success = false;
        result->message = "Canceled during retreat";
        result->final_position_error_m = -1.0f;
        goal_handle->canceled(result);
        goal_active_.store(false);
        return;
      }

      feedback->stage = "completed";
      feedback->position_error_m = 0.0f;
      goal_handle->publish_feedback(feedback);

      result->success = true;
      result->message = "Demo pick sequence complete";
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
  std::string traj_topic_;
  std::vector<std::string> controlled_joints_;
  std::vector<double> joint_min_;
  std::vector<double> joint_max_;

  double link1_length_m_;
  double link2_length_m_;
  double base_height_m_;
  double point_duration_sec_;
  double wait_padding_sec_;
  double default_pregrasp_offset_m_;
  double default_retreat_offset_m_;
  double open_gripper_pos_;
  double closed_gripper_pos_;

  std::atomic<bool> goal_active_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp_action::Server<PickArm>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmDemoController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
