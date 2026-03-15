#include <atomic>
#include <cstdint>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "robot_interfaces/action/pick_arm.hpp"

class ArmWaypointServer : public rclcpp::Node
{
public:
  using PickArm = robot_interfaces::action::PickArm;
  using GoalHandlePickArm = rclcpp_action::ServerGoalHandle<PickArm>;

  ArmWaypointServer()
  : Node("arm_waypoint_server"), goal_active_(false)
  {
    action_name_ = this->declare_parameter<std::string>("action_name", "pick_arm_waypoint");
    traj_topic_ = this->declare_parameter<std::string>("joint_trajectory_topic", "/arm/joint_trajectory_cmd");

    joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "joint_names",
      {
        "joint1_base_yaw",
        "joint2_shoulder",
        "joint3_elbow",
        "joint5_gripper"
      });

    point_duration_sec_ = this->declare_parameter<double>("point_duration_sec", 1.5);
    wait_padding_sec_ = this->declare_parameter<double>("wait_padding_sec", 0.2);

    open_gripper_pos_ = this->declare_parameter<double>("open_gripper_pos", 1.0);
    closed_gripper_pos_ = this->declare_parameter<double>("closed_gripper_pos", 0.0);

    waypoint_home_ = this->declare_parameter<std::vector<double>>(
      "waypoint_home", {0.0, 0.4, 0.2, 1.0});
    waypoint_pregrasp_ = this->declare_parameter<std::vector<double>>(
      "waypoint_pregrasp", {0.0, 0.8, 0.5, 1.0});
    waypoint_grasp_ = this->declare_parameter<std::vector<double>>(
      "waypoint_grasp", {0.0, 1.0, 0.7, 1.0});
    waypoint_retreat_ = this->declare_parameter<std::vector<double>>(
      "waypoint_retreat", {0.0, 0.6, 0.4, 0.0});

    validate_waypoint_sizes();

    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(traj_topic_, 10);

    action_server_ = rclcpp_action::create_server<PickArm>(
      this,
      action_name_,
      std::bind(&ArmWaypointServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ArmWaypointServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ArmWaypointServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "arm_waypoint_server ready on action '%s'", action_name_.c_str());
  }

private:
  void validate_waypoint_sizes() const
  {
    const size_t n = joint_names_.size();
    auto check = [n](const std::vector<double> & v, const std::string & name) {
        if (v.size() != n) {
          throw std::runtime_error(name + " size must match joint_names size");
        }
      };

    check(waypoint_home_, "waypoint_home");
    check(waypoint_pregrasp_, "waypoint_pregrasp");
    check(waypoint_grasp_, "waypoint_grasp");
    check(waypoint_retreat_, "waypoint_retreat");
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PickArm::Goal>)
  {
    if (goal_active_.load()) {
      RCLCPP_WARN(this->get_logger(), "Arm waypoint goal rejected: another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePickArm>)
  {
    RCLCPP_INFO(this->get_logger(), "Arm waypoint cancellation requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePickArm> goal_handle)
  {
    goal_active_.store(true);
    std::thread{std::bind(&ArmWaypointServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  trajectory_msgs::msg::JointTrajectory make_trajectory(
    const std::vector<double> & joints,
    double duration_sec) const
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joint_names_;

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

    while (rclcpp::ok() && std::chrono::steady_clock::now() < end_time) {
      if (goal_handle->is_canceling()) {
        return false;
      }

      feedback->stage = stage;
      feedback->position_error_m = 0.0f;
      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }

    return true;
  }

  std::vector<double> with_gripper(const std::vector<double> & base, double gripper_pos) const
  {
    std::vector<double> out = base;

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (joint_names_[i] == "joint5_gripper") {
        out[i] = gripper_pos;
      }
    }

    return out;
  }

  bool execute_single_waypoint(
    const std::string & command,
    const std::shared_ptr<GoalHandlePickArm> & goal_handle,
    const std::shared_ptr<PickArm::Feedback> & feedback)
  {
    std::vector<double> target;

    if (command == "home") {
      target = waypoint_home_;
    } else if (command == "pregrasp") {
      target = waypoint_pregrasp_;
    } else if (command == "grasp") {
      target = waypoint_grasp_;
    } else if (command == "retreat") {
      target = waypoint_retreat_;
    } else if (command == "open_gripper") {
      target = with_gripper(waypoint_retreat_, open_gripper_pos_);
    } else if (command == "close_gripper") {
      target = with_gripper(waypoint_retreat_, closed_gripper_pos_);
    } else {
      return false;
    }

    const auto traj = make_trajectory(target, point_duration_sec_);
    return publish_and_wait(traj, goal_handle, feedback, "executing_" + command);
  }

  bool execute_pickup_sequence(
    const std::shared_ptr<const PickArm::Goal> & goal,
    const std::shared_ptr<GoalHandlePickArm> & goal_handle,
    const std::shared_ptr<PickArm::Feedback> & feedback)
  {
    if (!publish_and_wait(
        make_trajectory(with_gripper(waypoint_pregrasp_, open_gripper_pos_), point_duration_sec_),
        goal_handle,
        feedback,
        "executing_pregrasp")) {
      return false;
    }

    if (!publish_and_wait(
        make_trajectory(with_gripper(waypoint_grasp_, open_gripper_pos_), point_duration_sec_),
        goal_handle,
        feedback,
        "executing_grasp")) {
      return false;
    }

    double close_pos = closed_gripper_pos_;
    if (goal->gripper_close_position >= 0.0f) {
      close_pos = static_cast<double>(goal->gripper_close_position);
    }

    if (!publish_and_wait(
        make_trajectory(with_gripper(waypoint_grasp_, close_pos), point_duration_sec_),
        goal_handle,
        feedback,
        "executing_gripper_close")) {
      return false;
    }

    if (!publish_and_wait(
        make_trajectory(with_gripper(waypoint_retreat_, close_pos), point_duration_sec_),
        goal_handle,
        feedback,
        "executing_retreat")) {
      return false;
    }

    return true;
  }

  void execute(const std::shared_ptr<GoalHandlePickArm> goal_handle)
  {
    auto result = std::make_shared<PickArm::Result>();
    auto feedback = std::make_shared<PickArm::Feedback>();

    try {
      const auto goal = goal_handle->get_goal();
      const std::string command = goal->planning_group.empty() ? "pickup" : goal->planning_group;

      bool ok = false;
      if (command == "pickup") {
        ok = execute_pickup_sequence(goal, goal_handle, feedback);
      } else {
        ok = execute_single_waypoint(command, goal_handle, feedback);
      }

      if (!ok) {
        if (goal_handle->is_canceling()) {
          result->success = false;
          result->message = "Goal canceled";
          result->final_position_error_m = 0.0f;
          goal_handle->canceled(result);
        } else {
          result->success = false;
          result->message = "Unknown command or execution failed. Use planning_group in {pickup,home,pregrasp,grasp,retreat,open_gripper,close_gripper}.";
          result->final_position_error_m = -1.0f;
          goal_handle->abort(result);
        }
        goal_active_.store(false);
        return;
      }

      feedback->stage = "completed";
      feedback->position_error_m = 0.0f;
      goal_handle->publish_feedback(feedback);

      result->success = true;
      result->message = "Waypoint command complete";
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

  std::vector<std::string> joint_names_;
  std::vector<double> waypoint_home_;
  std::vector<double> waypoint_pregrasp_;
  std::vector<double> waypoint_grasp_;
  std::vector<double> waypoint_retreat_;

  double point_duration_sec_;
  double wait_padding_sec_;
  double open_gripper_pos_;
  double closed_gripper_pos_;

  std::atomic<bool> goal_active_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp_action::Server<PickArm>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmWaypointServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
