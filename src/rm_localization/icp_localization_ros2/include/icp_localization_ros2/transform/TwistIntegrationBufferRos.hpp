/*
 * TwistIntegrationBuffer.hpp
 *
 *  Created on: Apr 26, 2021
 *      Author: jelavice
 */

#pragma once
#include "icp_localization_ros2/transform/TwistIntegrationBuffer.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/subscription.hpp>

namespace icp_loco {

class TwistIntegrationBufferRos : public TwistIntegrationBuffer {

 public:

  TwistIntegrationBufferRos(std::shared_ptr<rclcpp::Node> nh);

  void initialize();

  void setOdometryTopic(const std::string &topic);

 private:

  void odometryCallback(const nav_msgs::msg::Odometry &msg);

  std::string odometryTopic_;
  std::shared_ptr<rclcpp::Node> nh_;
  // ros::NodeHandle nh_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscriber_;
};



} // namespace icp_loco
