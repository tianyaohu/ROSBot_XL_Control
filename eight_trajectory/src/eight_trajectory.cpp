#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include <armadillo>
#include <cmath>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include <thread>

using namespace std;
using namespace std::chrono_literals;

class EightTrajectory : public rclcpp::Node {
public:
  EightTrajectory(double wb, double wd, double tw)
      : Node("eight_trajectory"), wheel_base(wb), wheel_diameter(wd),
        track_width(tw) {

    // init publishers and subscribers
    wheel_speed_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&EightTrajectory::odom_callback, this,
                  std::placeholders::_1));

    // init kinematic matrix
    init_kinematic_matrix();

    // init way point index
    cur_waypt_index = 0;
  }

  void move_through_waypoints() {
    // loop to move through all the way points
    W.each_row([&](arma::rowvec &way_pt) {
      cout << "Moving Towards " << way_pt << endl;
      move_to_goal(way_pt.t());
    });
  }

private:
  void move_to_goal(arma::vec goal) {
    // threshhold betwen goal and cur_pos
    const double THRESH = 0.05;    
    arma::vec error;

    do {
      // proccess odoms
      rclcpp::spin_some(shared_from_this());

      // get current pos
      arma::vec cur_pos = {cur_theta, cur_x, cur_y};
      // calc error
      error = goal - cur_pos;

      // add a proportional gain error;
      double Kp = 0.55;
      // recalc error with kp
      error *= Kp;

      // move based on error
      this->abs_motion(error);

    } while (rclcpp::ok() && !arma::all(arma::abs(error) < THRESH));
    // stop after reaching goal
    stop();
  }

  void stop() {
    arma::vec stop(4, arma::fill::zeros);
    publishWheelSpeeds(stop);
  }

  // {dphi, dx, dy}
  void abs_motion(arma::vec d) {
    //
    arma::vec twist = velocity2twist(d);
    arma::vec wheel_speeds = twist2wheels(twist);

    // reverse wheel speeds
    std::reverse(wheel_speeds.begin(), wheel_speeds.end());

    this->publishWheelSpeeds(wheel_speeds);
    rclcpp::sleep_for(chrono::milliseconds(25));
  }

  void publishWheelSpeeds(arma::vec speeds) {
    // convert arma::vec into a std::vector<float>
    std::vector<float> stdVec = arma::conv_to<std::vector<float>>::from(speeds);

    // init message
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = stdVec;
    wheel_speed_pub_->publish(msg);
  }

  static inline double normalize(double angle) {
    const double TWO_PI = 2.0 * M_PI;
    angle = fmod(angle + M_PI, TWO_PI);
    if (angle < 0)
      angle += TWO_PI;
    return angle - M_PI;
  }

  void init_waypoints(double start_x, double start_y, double start_theta) {
    // convert the input into a vec
    arma::rowvec start_vec = {start_theta, start_x, start_y};

    // add the start vec to every row of way_pts
    W.each_row() += start_vec;

    // Apply the test function to the first column
    W.col(0).transform(normalize);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double roll, pitch; // roll pitch is not used here
    // Convert quaternion to Euler angles
    tf2::Quaternion quat;
    // Get qua from msg
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, cur_theta);

    // get x,y
    cur_x = msg->pose.pose.position.x;
    cur_y = msg->pose.pose.position.y;

    // if running for the very first time, init way points
    if (first_odom_call_) {
      init_waypoints(cur_x, cur_y, cur_theta);
      first_odom_call_ = false;
    }
  }

  inline void init_kinematic_matrix() {
    // half wheel base
    double l = wheel_base / 2;
    // half track_width
    double w = track_width / 2;

    K = {
        {-l - w, 1, -1},
        {l + w, 1, 1},
        {l + w, 1, -1},
        {-l - w, 1, 1},
    };

    K = K / (wheel_diameter / 2);
  }

  // input twist vec: {wz, vx, vy}
  inline arma::vec twist2wheels(arma::vec twist) { return K * twist; }

  // input vector {dphi, dx, dy}
  inline arma::vec velocity2twist(arma::vec d) {
    arma::mat R = {
        {1, 0, 0},
        {0, cos(cur_theta), sin(cur_theta)},
        {0, -sin(cur_theta), cos(cur_theta)},
    };
    return R * d; // 3x1 vec {wz, vx, vy}
  }

  // ROS2 sub & pub
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      wheel_speed_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  arma::mat W = {{0.0, 1, -1},     {0.0, 2, 0}, {0.0, 3, 1}, {1.5708, 4, 0},
                 {-3.1415, 3, -1}, {0.0, 2, 0}, {0.0, 1, 1}, {0.0, 0, 0}};

  int cur_waypt_index;

  // twist2wheel matrix
  arma::mat K;

  double cur_x, cur_y, cur_theta;
  bool first_odom_call_ = true;

  // robot physical info
  double wheel_base;
  double wheel_diameter;
  double track_width;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // the following are ROSBot XL's parameter
  double wheel_base = 170;
  double wheel_diameter = 100;
  double track_width = 269.69;

  std::shared_ptr<EightTrajectory> node = std::make_shared<EightTrajectory>(
      wheel_base, wheel_diameter, track_width);

  node->move_through_waypoints();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
