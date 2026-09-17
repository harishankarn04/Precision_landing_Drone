#ifndef ROLAND_LORA_EKF_NODE_HPP
#define ROLAND_LORA_EKF_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <Eigen/Dense>
#include <roland_lora_msgs/msg/lo_ra_ranging.hpp>
#include "roland_lora/msg/bboxes.hpp"

class EkfNode : public rclcpp::Node {
public:
  EkfNode();

private:
  void lora_callback(const roland_lora_msgs::msg::LoRaRanging::SharedPtr msg);
  void bbox_callback(const roland_lora::msg::Bboxes::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timer_callback();

  rclcpp::Subscription<roland_lora_msgs::msg::LoRaRanging>::SharedPtr lora_sub_;
  rclcpp::Subscription<roland_lora::msg::Bboxes>::SharedPtr bbox_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  Eigen::MatrixXf T_; // Target State: [target_xyz]
  Eigen::Vector3f D_; // Drone Odom: [drone_xyz]
  Eigen::MatrixXf P_; // Covariance
  Eigen::MatrixXf Q_; // Process noise
  
  bool init_;
  double lora_range_sigma_;
};

#endif
