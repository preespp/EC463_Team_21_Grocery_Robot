#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_interfaces/action/move_rack.hpp"

namespace
{
constexpr int STATE_IDLE = 0;
constexpr int STATE_MOVING = 1;
constexpr int STATE_DONE = 2;
constexpr int STATE_ERROR = 3;
constexpr double DEFAULT_TIMEOUT_SEC = 25.0;
}  // namespace

class RackActionServer : public rclcpp::Node
{
public:
  using MoveRack = robot_interfaces::action::MoveRack;
  using GoalHandleMoveRack = rclcpp_action::ServerGoalHandle<MoveRack>;

  RackActionServer()
  : Node("rack_controller"), bus_fd_(-1), goal_active_(false)
  {
    bus_id_ = this->declare_parameter<int>("bus", 7);
    addr_ = this->declare_parameter<int>("addr", 0x13);

    shelf_map_[1] = static_cast<float>(this->declare_parameter<double>("shelf1_mm", 0.0));
    shelf_map_[2] = static_cast<float>(this->declare_parameter<double>("shelf2_mm", 240.0));
    shelf_map_[3] = static_cast<float>(this->declare_parameter<double>("shelf3_mm", 480.0));

    timeout_sec_ = this->declare_parameter<double>("timeout_sec", DEFAULT_TIMEOUT_SEC);

    open_i2c();

    action_server_ = rclcpp_action::create_server<MoveRack>(
      this,
      "move_rack",
      std::bind(&RackActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RackActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&RackActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Rack Action Server Ready (bus=%d addr=0x%02X)", bus_id_, addr_);
  }

  ~RackActionServer() override
  {
    if (bus_fd_ >= 0) {
      close(bus_fd_);
      bus_fd_ = -1;
    }
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveRack::Goal>)
  {
    if (goal_active_.load()) {
      RCLCPP_WARN(this->get_logger(), "Rack action rejected: another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveRack>)
  {
    RCLCPP_INFO(this->get_logger(), "Rack goal cancellation requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveRack> goal_handle)
  {
    goal_active_.store(true);
    std::thread{std::bind(&RackActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveRack> goal_handle)
  {
    auto result = std::make_shared<MoveRack::Result>();
    auto feedback = std::make_shared<MoveRack::Feedback>();
    const auto goal = goal_handle->get_goal();

    try {
      const auto it = shelf_map_.find(goal->shelf_level);
      if (it == shelf_map_.end()) {
        result->success = false;
        result->message = "Invalid shelf level";
        goal_handle->abort(result);
        goal_active_.store(false);
        return;
      }

      const float target_height = it->second;
      feedback->target_height_mm = target_height;

      RCLCPP_INFO(
        this->get_logger(),
        "Move rack -> shelf %d (%.1f mm)",
        goal->shelf_level,
        target_height);

      send_height(target_height);

      const auto start_time = std::chrono::steady_clock::now();
      rclcpp::Rate loop_rate(20.0);

      while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
          result->success = false;
          result->message = "Goal canceled";
          goal_handle->canceled(result);
          goal_active_.store(false);
          return;
        }

        const int state = read_state();
        feedback->state = state;
        goal_handle->publish_feedback(feedback);

        if (state == STATE_DONE) {
          result->success = true;
          result->message = "Rack motion complete";
          goal_handle->succeed(result);
          goal_active_.store(false);
          return;
        }

        if (state == STATE_ERROR) {
          result->success = false;
          result->message = "ESP32 reported error";
          goal_handle->abort(result);
          goal_active_.store(false);
          return;
        }

        const auto elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > timeout_sec_) {
          result->success = false;
          result->message = "Rack timeout";
          goal_handle->abort(result);
          goal_active_.store(false);
          return;
        }

        loop_rate.sleep();
      }

      result->success = false;
      result->message = "ROS shutdown";
      goal_handle->abort(result);
    } catch (const std::exception & ex) {
      result->success = false;
      result->message = ex.what();
      goal_handle->abort(result);
    }

    goal_active_.store(false);
  }

  void open_i2c()
  {
    const std::string dev = "/dev/i2c-" + std::to_string(bus_id_);
    bus_fd_ = open(dev.c_str(), O_RDWR);
    if (bus_fd_ < 0) {
      throw std::runtime_error("Failed to open " + dev);
    }

    if (ioctl(bus_fd_, I2C_SLAVE, addr_) < 0) {
      close(bus_fd_);
      bus_fd_ = -1;
      throw std::runtime_error("Failed to set I2C address");
    }
  }

  void send_height(float height_mm)
  {
    std::array<uint8_t, sizeof(float)> payload{};
    std::memcpy(payload.data(), &height_mm, sizeof(float));

    const auto bytes = write(bus_fd_, payload.data(), payload.size());
    if (bytes != static_cast<ssize_t>(payload.size())) {
      throw std::runtime_error("I2C write failed while sending rack height");
    }
  }

  int read_state() const
  {
    uint8_t state = 0;
    const auto bytes = read(bus_fd_, &state, sizeof(state));
    if (bytes != static_cast<ssize_t>(sizeof(state))) {
      throw std::runtime_error("I2C read failed while reading rack state");
    }
    return static_cast<int>(state);
  }

  int bus_id_;
  int addr_;
  int bus_fd_;
  double timeout_sec_;
  std::unordered_map<int, float> shelf_map_;
  std::atomic<bool> goal_active_;

  rclcpp_action::Server<MoveRack>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RackActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
