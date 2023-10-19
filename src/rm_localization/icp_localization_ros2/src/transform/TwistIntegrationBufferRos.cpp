/*
 * TwistIntegrationBuffer.cpp
 *
 *  Created on: Apr 26, 2021
 *      Author: jelavice
 */

#include "icp_localization_ros2/transform/TwistIntegrationBufferRos.hpp"
#include "icp_localization_ros2/common/assert.hpp"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <algorithm>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/qos.hpp>
#include <string>

namespace icp_loco {

TwistIntegrationBufferRos::TwistIntegrationBufferRos(
    std::shared_ptr<rclcpp::Node> nh)
    : nh_(nh) {
  odometryTopic_ = nh_->declare_parameter("icp_localization_ros2.odometry_data_topic", "");
  // nh_.param<std::string>("odometry_topic", odometryTopic_, "");
}

void TwistIntegrationBufferRos::setOdometryTopic(const std::string &topic) {
  odometryTopic_ = topic;
}

void TwistIntegrationBufferRos::initialize() {
  odomSubscriber_ = nh_->create_subscription<nav_msgs::msg::Odometry>(odometryTopic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&TwistIntegrationBufferRos::odometryCallback, this, std::placeholders::_1));
}

void TwistIntegrationBufferRos::odometryCallback(
    const nav_msgs::msg::Odometry &msg) {
  const auto twist = fromRos(msg.twist.twist);

  const auto time = fromRos(msg.header.stamp);

  push(time, twist);
}

} // namespace icp_loco
