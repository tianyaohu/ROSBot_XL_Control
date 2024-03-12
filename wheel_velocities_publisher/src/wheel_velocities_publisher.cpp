#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>
#include <vector>

class WheelVelocityPublisher : public rclcpp::Node {
public:
  WheelVelocityPublisher(const rclcpp::NodeOptions &options)
      : Node("wheel_velocity_publisher", options) {
    wheel_speed_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);
  }

  void testMovements() {
    performMovement({1.0, 1.0, 1.0, 1.0}, "Moving forward");
    performMovement({-1.0, -1.0, -1.0, -1.0}, "Moving backward");
    performMovement({1.0, -1.0, 1.0, -1.0}, "Moving sideways to the left"); 
    performMovement({-1.0, 1.0, -1.0, 1.0}, "Moving sideways to the right"); 
    performMovement({1.0, -1.0, -1.0, 1.0}, "Turning clockwise"); 
    performMovement({-1.0, 1.0, 1.0, -1.0}, "Turning counter-clockwise"); 
    performMovement({0.0, 0.0, 0.0, 0.0}, "Stopping");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      wheel_speed_pub;

  void publishWheelSpeeds(const std::vector<float> &speeds) {
    auto msg = std::make_shared<std_msgs::msg::Float32MultiArray>();
    msg->data = speeds;
    wheel_speed_pub->publish(*msg);
  }

  void performMovement(const std::vector<float> &speeds,
                       const std::string &action = "", float time = 3.0) {
    if (!action.empty()) {
      RCLCPP_INFO(this->get_logger(), "%s", action.c_str());
    }

    rclcpp::Rate loop_rate(10); // 10Hz
    auto start = this->now();

    // Continuously execute until the specified time has elapsed
    while (rclcpp::ok() && (this->now() - start).seconds() < time) {
      publishWheelSpeeds(speeds);
      loop_rate.sleep();
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WheelVelocityPublisher>(rclcpp::NodeOptions());
  node->testMovements(); // shutting down right after
  rclcpp::shutdown();
  return 0;
}
