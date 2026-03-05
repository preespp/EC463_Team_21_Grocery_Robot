#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

class ArmJointStateToGazebo : public rclcpp::Node
{
public:
  ArmJointStateToGazebo()
  : Node("arm_to_gazebo")
  {
    joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "joint_names",
      {
        "joint1_base_yaw",
        "joint2_shoulder",
        "joint3_elbow",
        "joint4_wrist_roll",
        "joint5_gripper"
      });

    input_topic_ = this->declare_parameter<std::string>("input_topic", "/arm/joint_state");

    const auto topic_defaults = std::vector<std::string>{
      "/arm/joint1_cmd",
      "/arm/joint2_cmd",
      "/arm/joint3_cmd",
      "/arm/joint4_cmd",
      "/arm/joint5_cmd"
    };

    output_topics_ = this->declare_parameter<std::vector<std::string>>(
      "output_topics", topic_defaults);

    if (output_topics_.size() != joint_names_.size()) {
      throw std::runtime_error("output_topics size must match joint_names size");
    }

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      publishers_[joint_names_[i]] =
        this->create_publisher<std_msgs::msg::Float64>(output_topics_[i], 10);
    }

    subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      input_topic_,
      10,
      std::bind(&ArmJointStateToGazebo::joint_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "arm_to_gazebo started (%s)", input_topic_.c_str());
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::unordered_map<std::string, double> positions;
    const auto count = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < count; ++i) {
      positions[msg->name[i]] = msg->position[i];
    }

    for (const auto & joint : joint_names_) {
      const auto it = positions.find(joint);
      if (it == positions.end()) {
        continue;
      }

      std_msgs::msg::Float64 out;
      out.data = it->second;
      publishers_[joint]->publish(out);
    }
  }

  std::string input_topic_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> output_topics_;

  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmJointStateToGazebo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
