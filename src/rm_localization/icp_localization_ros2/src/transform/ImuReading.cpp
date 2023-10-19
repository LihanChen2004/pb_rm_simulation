/*
 * Imu.cpp
 *
 *  Created on: Apr 30, 2021
 *      Author: jelavice
 */

#include "icp_localization_ros2/transform/ImuReading.hpp"

#include "icp_localization_ros2/common/math.hpp"

// #include <eigen_conversions/eigen_msg.h>

namespace icp_loco {

TimestampedImuReading interpolate(const TimestampedImuReading &start,
                                  const TimestampedImuReading &end,
                                  const Time &time) {
  if (time > end.time_ || time < start.time_) {
    throw std::runtime_error(
        "imu interpolate: query time is not between start and end time");
  }

  const auto &s = start.imu_;
  const auto &e = end.imu_;
  TimestampedImuReading retVal;
  retVal.imu_.acceleration() = interpolateVector(
      s.acceleration(), e.acceleration(), start.time_, end.time_, time);
  retVal.imu_.angularVelocity() = interpolateVector(
      s.angularVelocity(), e.angularVelocity(), start.time_, end.time_, time);
  retVal.imu_.rotation() = interpolateQuaternion(s.rotation(), e.rotation(),
                                                 start.time_, end.time_, time);
  return retVal;
}

TimestampedImuReading fromRos(const sensor_msgs::msg::Imu &msg) {
  Eigen::Vector3d lin{msg.linear_acceleration.x, msg.linear_acceleration.y,
                      msg.linear_acceleration.z},
      ang{msg.angular_velocity.x, msg.angular_velocity.y,
          msg.angular_velocity.z};
  Eigen::Quaterniond q{msg.orientation.w, msg.orientation.x, msg.orientation.y,
                       msg.orientation.z};
  // tf::vectorMsgToEigen(msg.linear_acceleration, lin);
  // tf::vectorMsgToEigen(msg.angular_velocity, ang);
  // tf::quaternionMsgToEigen(msg.orientation, q);
  TimestampedImuReading retVal;
  retVal.imu_ = ImuReadingd(lin, ang, q);
  retVal.time_ = fromRos(msg.header.stamp);
  return retVal;
}

} // namespace icp_loco
