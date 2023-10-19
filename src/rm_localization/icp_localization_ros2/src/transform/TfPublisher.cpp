/*
 * TfPublisher.cpp
 *
 *  Created on: Apr 27, 2021
 *      Author: jelavice
 */

#include "icp_localization_ros2/transform/TfPublisher.hpp"
#include "icp_localization_ros2/common/assert.hpp"
#include "icp_localization_ros2/transform/FrameTracker.hpp"
#include "icp_localization_ros2/transform/ImuTracker.hpp"
#include <Eigen/Dense>
// #include <eigen_conversions/eigen_msg.h>
#include <memory>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

namespace icp_loco {

namespace {

constexpr bool isDebug = false;

geometry_msgs::msg::TransformStamped toRos(const Rigid3d &T, const Time &time,
                                           const std::string &frame,
                                           const std::string &childFrame) {
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = toRos(time);
  transformStamped.header.frame_id = frame;
  transformStamped.child_frame_id = childFrame;
  transformStamped.transform.translation.x = T.translation().x();
  transformStamped.transform.translation.y = T.translation().y();
  transformStamped.transform.translation.z = T.translation().z();
  transformStamped.transform.rotation.w = T.rotation().w();
  transformStamped.transform.rotation.x = T.rotation().x();
  transformStamped.transform.rotation.y = T.rotation().y();
  transformStamped.transform.rotation.z = T.rotation().z();
  return transformStamped;
}

} // namespace

TfPublisher::TfPublisher(std::shared_ptr<rclcpp::Node> nh,
                         std::shared_ptr<FrameTracker> frameTracker,
                         std::shared_ptr<ImuTracker> imuTracker)
    : nh_(nh), frameTracker_(frameTracker), imuTracker_(imuTracker),
      initPosition_(Eigen::Vector3d::Zero()),
      initOrientation_(Eigen::Quaterniond::Identity()) {}

void TfPublisher::setOdometryTopic(const std::string &topic) {
  odometryTopic_ = topic;
}

void TfPublisher::setImuTopic(const std::string &topic) { imuTopic_ = topic; }

void TfPublisher::initialize() {

  odomSubscriber_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
      odometryTopic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&TfPublisher::odometryCallback, this, std::placeholders::_1));
  imuSubscriber_ = nh_->create_subscription<sensor_msgs::msg::Imu>(
      imuTopic_, rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&TfPublisher::imuCallback, this, std::placeholders::_1));
      tfBroadcaster_= std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
}

void TfPublisher::publishMapToRangeSensor(const Time &t) {
  const auto mapToLidar = frameTracker_->getTransformMapToRangeSensor(t);
  geometry_msgs::msg::TransformStamped transformStamped =
      toRos(mapToLidar, t, "map", "range_sensor");
  tfBroadcaster_->sendTransform(transformStamped);
}

void TfPublisher::setIsProvideOdomFrame(bool value) {
  isProvideOdomFrame_ = value;
}

void TfPublisher::setIsUseOdometry(bool value) { isUseOdometry_ = value; }

void TfPublisher::setInitialPose(const Eigen::Vector3d &p,
                                 const Eigen::Quaterniond &q) {
  initPosition_ = p;
  initOrientation_ = q;
}

void TfPublisher::publishMapToOdom(const Time &t) {
  const auto mapToOdom = frameTracker_->getTransformMapToOdom(t);
  geometry_msgs::msg::TransformStamped transformStamped =
      toRos(mapToOdom, t, "map", "odom");
  tfBroadcaster_->sendTransform(transformStamped);
  // just for debug

  if (isDebug) {
    const auto mapToLidar = frameTracker_->getTransformMapToRangeSensor(t);
    transformStamped = toRos(mapToLidar, t, "map", "rs_check");
    tfBroadcaster_->sendTransform(transformStamped);

    const auto lidarToCam =
        (frameTracker_->getTransformOdomSourceToRangeSensor(t)).inverse();
    geometry_msgs::msg::TransformStamped transformStamped =
        toRos(lidarToCam, t, "rs_check", "os_check");
    tfBroadcaster_->sendTransform(transformStamped);

    const auto check = mapToLidar * lidarToCam;
    transformStamped = toRos(check, t, "map", "rs_os_check");
    tfBroadcaster_->sendTransform(transformStamped);
  }
}

void TfPublisher::odometryCallback(const nav_msgs::msg::Odometry &msg) {
  Eigen::Vector3d t{msg.pose.pose.position.x, msg.pose.pose.position.y,
                    msg.pose.pose.position.z};
  Eigen::Quaterniond q{msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z};
  TimestampedTransform odomToTracking;
  odomToTracking.transform_ = Rigid3d(t, q);
  odomToTracking.time_ = fromRos(msg.header.stamp);
  frameTracker_->setTransformOdomToOdomSource(odomToTracking);

  if (isProvideOdomFrame_) {
    geometry_msgs ::msg::TransformStamped transformStamped = toRos(
        odomToTracking.transform_, odomToTracking.time_, "odom", "odom_source");
    tfBroadcaster_->sendTransform(transformStamped);

    const auto cameraToLidar =
        frameTracker_->getTransformOdomSourceToRangeSensor(
            odomToTracking.time_);
    transformStamped = toRos(cameraToLidar, odomToTracking.time_, "odom_source",
                             "range_sensor");
    tfBroadcaster_->sendTransform(transformStamped);

    // just visualize raw odometry
    const Rigid3d initPose(initPosition_, initOrientation_);
    const Rigid3d lidarToOdomSource =
        frameTracker_->getTransformOdomSourceToRangeSensor(Time(fromSeconds(0)))
            .inverse();
    transformStamped = toRos(initPose * lidarToOdomSource, odomToTracking.time_,
                             "map", "init_odom_source");
    tfBroadcaster_->sendTransform(transformStamped);
    transformStamped = toRos(odomToTracking.transform_, odomToTracking.time_,
                             "init_odom_source", "raw_odometry_source");
    tfBroadcaster_->sendTransform(transformStamped);
  }
}

void TfPublisher::imuCallback(const sensor_msgs::msg::Imu &msg) {
  TimestampedImuReading imuReading = fromRos(msg);
  imuTracker_->addReading(imuReading.time_, imuReading.imu_);
  //  std::cout <<"Adding imu reading: \n" << imuReading.imu_.asString() <<
  //  std::endl;
  const auto lidarToImu =
      frameTracker_->getTransformImuToRangeSensor(imuReading.time_).inverse();
  geometry_msgs::msg::TransformStamped transformStamped =
      toRos(lidarToImu, imuReading.time_, "range_sensor", "inertial_sensor");
  tfBroadcaster_->sendTransform(transformStamped);

  //	{
  //	Rigid3d odomToOdomSource = imuTracker_->getLatestOdometry();
  ////	std::cout << "IMu odom: " << odomToOdomSource.asString() << std::endl;
  //	geometry_msgs::TransformStamped transformStamped =
  // toRos(odomToOdomSource, 			imuReading.time_, "map",
  // "odom_imu"); 	tfBroadcaster_->sendTransform(transformStamped);
  //	}
  // todo I should fuse those two somehow
  if (isProvideOdomFrame_ && !isUseOdometry_) {
  }

  //  const auto end = imuTracker_->getBuffer().latest_time();
  //  const auto start = end - fromSeconds(1.0);
  //  std::cout << "dXYZ: " << imuTracker_->getLienarPositionChange(start, end,
  //  Eigen::Vector3d::Ones()).transpose() << std::endl; const auto
  //  imuOrientation = imuTracker_->orientation(end); const auto q =
  //  imuTracker_->getOrientationChange(start, end) * imuOrientation.inverse();
  //  ROS_INFO_STREAM_THROTTLE(1.0, "Imu orientation: " << 180. / M_PI *
  //  toRPY(imuOrientation).transpose()); std::cout << "d RPY: " << 180. / M_PI
  //  * toRPY(q).transpose() << "\n \n";
}

} // namespace icp_loco
