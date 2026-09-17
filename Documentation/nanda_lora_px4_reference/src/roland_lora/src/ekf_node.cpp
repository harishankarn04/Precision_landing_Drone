#include "roland_lora/ekf_node.hpp"
#include <iostream>

using namespace std::chrono_literals;

EkfNode::EkfNode() : Node("ekf_node"), init_(false) {
  this->declare_parameter("lora_range_sigma", 1.5);
  lora_range_sigma_ = this->get_parameter("lora_range_sigma").as_double();

  lora_sub_ = this->create_subscription<roland_lora_msgs::msg::LoRaRanging>(
    "/lora/ranging", 10, std::bind(&EkfNode::lora_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/mavros/local_position/odom", rclcpp::SensorDataQoS(), std::bind(&EkfNode::odom_callback, this, std::placeholders::_1));

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ekf/relative_platform_pose", 10);

  // Initialize state (Target X, Y, Z) to 0,0,0
  T_ = Eigen::MatrixXf::Zero(3, 1);
  P_ = Eigen::MatrixXf::Identity(3, 3) * 100.0;
  
  // Process noise (target moving slowly)
  Q_ = Eigen::MatrixXf::Identity(3, 3) * 0.01;

  timer_ = this->create_wall_timer(50ms, std::bind(&EkfNode::timer_callback, this));
}

void EkfNode::lora_callback(const roland_lora_msgs::msg::LoRaRanging::SharedPtr msg) {
  if (!init_) return;
  
  int id = msg->anchor_id;
  
  // Anchor offsets relative to target center (from Gazebo SDF)
  float dx = 0, dy = 0, dz = 0.1;
  if (id == 0) { dx = 1.5; dy = 1.5; }
  else if (id == 1) { dx = 1.5; dy = -1.5; }
  else if (id == 2) { dx = -1.5; dy = 1.5; }
  else if (id == 3) { dx = -1.5; dy = -1.5; }
  else return; // Unknown anchor

  // Drone position
  float xd = D_(0);
  float yd = D_(1);
  float zd = D_(2);

  // Target position (Prior)
  float xt = T_(0);
  float yt = T_(1);
  float zt = T_(2);

  // Anchor position
  float xa = xt + dx;
  float ya = yt + dy;
  float za = zt + dz;

  // Expected range h(T)
  float h = std::sqrt(std::pow(xa - xd, 2) + std::pow(ya - yd, 2) + std::pow(za - zd, 2));
  if (h < 0.1) return;

  // Jacobian H = d(h)/dT
  Eigen::MatrixXf H(1, 3);
  H << (xa - xd) / h, (ya - yd) / h, (za - zd) / h;

  // Measurement innovation
  float z = msg->range;
  float y = z - h; // Correct non-linear innovation!

  // Measurement variance
  float R = std::pow(msg->range_error, 2);

  // EKF Update
  Eigen::MatrixXf S = H * P_ * H.transpose() + Eigen::MatrixXf::Constant(1, 1, R);
  Eigen::MatrixXf K = P_ * H.transpose() * S.inverse();

  T_ = T_ + K * y;
  P_ = (Eigen::MatrixXf::Identity(3, 3) - K * H) * P_;
  
  // Time update (Process noise)
  P_ = P_ + Q_;
}

void EkfNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  D_ = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  if (!init_) {
    init_ = true;
  }
}

void EkfNode::timer_callback() {
  if (!init_) return;
  geometry_msgs::msg::PoseStamped out;
  out.header.stamp = this->now();
  out.header.frame_id = "map";
  out.pose.position.x = T_(0) - D_(0);
  out.pose.position.y = T_(1) - D_(1);
  out.pose.position.z = T_(2) - D_(2);
  pose_pub_->publish(out);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfNode>());
  rclcpp::shutdown();
  return 0;
}
