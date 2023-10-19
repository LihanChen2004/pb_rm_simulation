/*
 * TfPublisher.hpp
 *
 *  Created on: Apr 27, 2021
 *      Author: jelavice
 */

#pragma once
#include "icp_localization_ros2/transform/RigidTransform.hpp"
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace icp_loco {

class FrameTracker;
class ImuTracker;

class TfPublisher {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TfPublisher(std::shared_ptr<rclcpp::Node> nh,
              std::shared_ptr<FrameTracker> frameTracker,
              std::shared_ptr<ImuTracker> imuTracker);
  ~TfPublisher() = default;
  void setOdometryTopic(const std::string &topic);
  void setImuTopic(const std::string &topic);

  void initialize();
  void publishMapToOdom(const Time &time);
  void publishMapToRangeSensor(const Time &time);
  void setIsProvideOdomFrame(bool value);
  void setIsUseOdometry(bool value);
  void setInitialPose(const Eigen::Vector3d &p, const Eigen::Quaterniond &q);

private:
  void odometryCallback(const nav_msgs::msg::Odometry &msg);
  void imuCallback(const sensor_msgs::msg::Imu &msg);

  std::shared_ptr<rclcpp::Node> nh_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  std::string odometryTopic_;
  std::string imuTopic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscriber_;
  std::shared_ptr<FrameTracker> frameTracker_;
  std::shared_ptr<ImuTracker> imuTracker_;
  bool isProvideOdomFrame_ = false;
  bool isUseOdometry_ = false;
  Eigen::Vector3d initPosition_;
  Eigen::Quaterniond initOrientation_;
};

} // namespace icp_loco
