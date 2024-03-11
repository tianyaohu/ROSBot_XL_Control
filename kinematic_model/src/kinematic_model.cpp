#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>

#include <armadillo>
#include <iostream>

class KinematicController : public rclcpp::Node {
public:
  KinematicController() : Node("kinematic_model") {
    // r is the radius of the wheels
    double r = 100 / 2;
    // w is half of track width
    double w = 269.69 / 2;
    // l is half of of the wheel base distanc
    double l = 170 / 2;
    // init kinematic_matrix
    this->init_kinematic_matrix(l, w, r);

    // Subscriber to the /wheel_speed topic
    wheel_speed_subscriber_ =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_speed", 10,
            std::bind(&KinematicController::wheelSpeedCallback, this,
                      std::placeholders::_1));

    // Publisher to the /cmd_vel topic
    velocity_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  void init_kinematic_matrix(double l, double w, double r) {
    A = {{-l - w, 1, -1},
         {l + w, 1, 1},
         {l + w, 1, -1},
         {-l - w, 1, 1}}; // 4x3 matrix

    A *= 1 / r;

    A_pinv = arma::pinv(A); // Compute the pseudoinverse
  }

  void
  wheelSpeedCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // msg->data contains wheel the order of [front_left, front_right,
    // rear_left, rear_right]
    // calc ang_z, lin_x, lin_y velocities
    arma::vec wheel_vel_vec = {msg->data[0], msg->data[1], msg->data[2],
                               msg->data[3]};

    arma::vec vel_vec = A_pinv * wheel_vel_vec;

    // TESTING output
    // std::cout << "vel_vec:\n" << vel_vec << std::endl;

    // Create and publish Twist message
    auto twist_msg = geometry_msgs::msg::Twist();

    twist_msg.angular.z = vel_vec[0];
    twist_msg.linear.x = vel_vec[1];
    twist_msg.linear.y = -vel_vec[2];
    velocity_publisher_->publish(twist_msg);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      wheel_speed_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

  arma::mat A;
  arma::mat A_pinv;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv); // Initialize ROS 2

  auto kinematic_controller =
      std::make_shared<KinematicController>(); // Create a KinematicController
                                               // node

  rclcpp::spin(
      kinematic_controller); // Spin the node so it can process callbacks

  rclcpp::shutdown(); // Shutdown ROS 2 when done
  return 0;
}