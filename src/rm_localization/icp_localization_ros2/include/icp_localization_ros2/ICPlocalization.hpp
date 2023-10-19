/*
 * ICPlocalization.hpp
 *
 *  Created on: Apr 23, 2021
 *      Author: jelavice
 */

#pragma once
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
// #include <ros/ros.h>
#include "pointmatcher/PointMatcher.h"
#include <thread>

#include "icp_localization_ros2/RangeDataAccumulator.hpp"
#include "icp_localization_ros2/common/typedefs.hpp"
#include "icp_localization_ros2/helpers.hpp"
#include "pointmatcher/IO.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <eigen_conversions/eigen_msg.h>
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

namespace icp_loco {

class TfPublisher;
class FrameTracker;
class ImuTracker;

class ICPlocalization : public rclcpp::Node {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ICPlocalization(const rclcpp::NodeOptions &options);
  ~ICPlocalization();
  void setMapCloud(const Pointcloud &map);
  void setInitialPose(const Eigen::Vector3d &p, const Eigen::Quaterniond &q);
  void initialize();
  DP fromPCL(const Pointcloud &pcl);
  void matchScans();

  void icpWorker();
  void publishPose() const;
  void publishRegisteredCloud() const;
  const std::string &getFixedFrame() const;
  void set2DPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstPtr &msg);

  void initializeInternal();

private:
  Eigen::Vector3d userSetPosition_;
  Eigen::Quaterniond userSetQuaternion_;
  Eigen::Vector3d lastPosition_;
  Eigen::Quaterniond lastOrientation_;
  Eigen::Vector3d currentPosition_;
  Eigen::Quaterniond currentOrientation_;
  Pointcloud mapCloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      registeredCloudPublisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initialPose_;
  // ros::Publisher registeredCloudPublisher_;
  // ros::Publisher posePub_;
  // ros::Subscriber initialPose_;
  PM::ICPSequence icp_;
  PM::DataPointsFilters inputFilters_;
  DP refCloud_;
  DP regCloud_;
  bool isMapSet_ = false;
  mutable int seq_ = 0;
  std::shared_ptr<RangeDataAccumulatorRos> rangeDataAccumulator_;
  std::thread icpWorker_;
  PM::TransformationParameters optimizedPose_;
  Time optimizedPoseTimestamp_;
  Time lastOptimizedPoseTimestamp_;
  Time regCloudTimestamp_;
  std::shared_ptr<TfPublisher> tfPublisher_;
  std::shared_ptr<FrameTracker> frameTracker_;
  std::shared_ptr<ImuTracker> imuTracker_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  bool isFirstScanMatch_ = true;
  bool isUseOdometry_ = false;
  bool isSetPoseFromUser_ = false;
  std::string fixedFrame_ = "map";
};

} // namespace icp_loco
