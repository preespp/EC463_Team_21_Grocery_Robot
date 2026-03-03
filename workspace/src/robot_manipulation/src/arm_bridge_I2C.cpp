#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace
{
constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
}

class ArmMotorBridge : public rclcpp::Node
{
public:
  ArmMotorBridge()
  : Node("arm_motor"), bus_fd_(-1)
  {
    bus_id = this->declare_parameter<int>("i2c_bus", 7);
    addr = this->declare_parameter<int>("i2c_addr", 0x08);
    joint_count = static_cast<size_t>(this->declare_parameter<int>("joint_count", 4));

    trajectory_topic_= this->declare_parameter<std::string>(
      "trajectory_topic", "/arm/joint_trajectory_cmd");
    direct_topic = this->declare_parameter<std::string>("direct_topic", "/arm/joint_cmd");

    joint_names = this->declare_parameter<std::vector<std::string>>(
      "joint_names",
      {
        "joint1_base_yaw",
        "joint2_shoulder",
        "joint3_elbow",
        //"joint4_wrist_roll",
        "joint5_gripper"
      });

    servo_offset_deg = this->declare_parameter<std::vector<double>>(
      "servo_offset_deg",
      {0.0, 0.0, 0.0, 0.0, 0.0});

    servo_min_deg = this->declare_parameter<std::vector<double>>(
      "servo_min_deg",
      std::vector<double>(joint_count_, 0.0));

    servo_max_deg = this->declare_parameter<std::vector<double>>(
      "servo_max_deg",
      std::vector<double>(joint_count_, 180.0));

    const auto dir_vals = this->declare_parameter<std::vector<int64_t>>(
      "servo_direction", std::vector<int64_t>(joint_count_, 1));

    default_point_dt_sec = this->declare_parameter<double>("default_point_dt_sec", 0.05);

    servo_direction_.reserve(dir_vals.size());
    for (const auto v : dir_vals) {
      servo_direction_.push_back(static_cast<int>(v));
    }

    validate_config_sizes();
    open_i2c();

    traj_sub = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      trajectory_topic,
      10,
      std::bind(&ArmMotorBridge::trajectory_callback, this, std::placeholders::_1));

    direct_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      direct_topic,
      10,
      std::bind(&ArmMotorBridge::direct_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "arm_motor started on /dev/i2c-%d addr=0x%02X",
      bus_id,
      addr);
  }

  ~ArmMotorBridge() override
  {
    if (bus_fd_ >= 0) {
      close(bus_fd);
      bus_fd = -1;
    }
  }

private:
  void validate_config_sizes() const
  {
    const auto expected = joint_count_;
    auto check = [expected](size_t actual, const std::string & name) {
        if (actual != expected) {
          throw std::runtime_error(
                  "Parameter '" + name + "' size must be " + std::to_string(expected));
        }
      };

    check(joint_names_.size(), "joint_names");
    check(servo_offset_deg_.size(), "servo_offset_deg");
    check(servo_min_deg_.size(), "servo_min_deg");
    check(servo_max_deg_.size(), "servo_max_deg");
    check(servo_direction_.size(), "servo_direction");
  }

  void open_i2c()
  {
    const std::string dev = "/dev/i2c-" + std::to_string(bus_id);
    bus_fd = open(dev.c_str(), O_RDWR);
    if (bus_fd < 0) {
      throw std::runtime_error("Failed to open " + dev);
    }

    if (ioctl(bus_fd_, I2C_SLAVE, addr) < 0) {
      close(bus_fd_);
      bus_fd_ = -1;
      throw std::runtime_error("Failed to set I2C address");
    }
  }

  void direct_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::vector<float> targets(joint_count_, 0.0f);
    const size_t count = std::min(joint_count_, msg->data.size());
    for (size_t i = 0; i < count; ++i) {
      targets[i] = msg->data[i];
    }
    send_servo_degrees(targets);
  }

  void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    if (msg->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
      return;
    }

    std::vector<int> indices;
    if (!build_index_map(msg->joint_names, indices)) {
      RCLCPP_WARN(this->get_logger(), "Trajectory missing required joint names");
      return;
    }

    double prev_time_sec = 0.0;
    for (const auto & point : msg->points) {
      if (point.positions.size() < msg->joint_names.size()) {
        RCLCPP_WARN(this->get_logger(), "Trajectory point has insufficient positions");
        return;
      }

      std::vector<float> targets(joint_count, 0.0f);
      for (size_t joint_i = 0; joint_i < joint_count; ++joint_i) {
        const auto msg_idx = static_cast<size_t>(indices[joint_i]);
        const double rad = point.positions[msg_idx];

        double deg = servo_offset_deg_[joint_i] +
          static_cast<double>(servo_direction_[joint_i]) * rad * RAD_TO_DEG;

        deg = std::clamp(deg, servo_min_deg_[joint_i], servo_max_deg_[joint_i]);
        targets[joint_i] = static_cast<float>(deg);
      }

      send_servo_degrees(targets);

      const double current_time_sec =
        static_cast<double>(point.time_from_start.sec) +
        static_cast<double>(point.time_from_start.nanosec) * 1e-9;

      double sleep_dt = current_time_sec - prev_time_sec;
      if (sleep_dt <= 0.0) {
        sleep_dt = default_point_dt_sec_;
      }
      rclcpp::sleep_for(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(sleep_dt)));
      prev_time_sec = current_time_sec;
    }
  }

  bool build_index_map(const std::vector<std::string> & incoming, std::vector<int> & indices) const
  {
    indices.assign(joint_count, -1);

    for (size_t i = 0; i < joint_count; ++i) {
      const auto & desired = joint_names_[i];
      for (size_t j = 0; j < incoming.size(); ++j) {
        if (incoming[j] == desired) {
          indices[i] = static_cast<int>(j);
          break;
        }
      }
      if (indices[i] < 0) {
        return false;
      }
    }

    return true;
  }

  void send_servo_degrees(const std::vector<float> & target_deg)
  {
    std::vector<uint8_t> bytes(target_deg.size() * sizeof(float));
    std::memcpy(bytes.data(), target_deg.data(), bytes.size());

    const auto sent = write(bus_fd_, bytes.data(), bytes.size());
    if (sent != static_cast<ssize_t>(bytes.size())) {
      RCLCPP_WARN(this->get_logger(), "I2C write failed (%zd/%zu)", sent, bytes.size());
      return;
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Servo TX [deg]: %.1f %.1f %.1f %.1f",// %.1f",
      target_deg[0],
      target_deg[1],
      target_deg[2],
      target_deg[3],
      //target_deg[4]
    );
  }

  int bus_id_
  int addr;
  int bus_fd;
  size_t joint_count;
  double default_point_dt_sec;

  std::string trajectory_topic;
  std::string direct_topic;

  std::vector<std::string> joint_names;
  std::vector<double> servo_offset_deg;
  std::vector<double> servo_min_deg;
  std::vector<double> servo_max_deg;
  std::vector<int> servo_direction;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr direct_sub;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmMotorBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
