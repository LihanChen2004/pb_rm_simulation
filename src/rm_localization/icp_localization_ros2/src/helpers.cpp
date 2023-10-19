/*
 * helpers.cpp
 *
 *  Created on: Apr 22, 2021
 *      Author: jelavice
 */

#include "icp_localization_ros2/helpers.hpp"
#include "icp_localization_ros2/common/math.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
namespace icp_loco {

namespace {
const double kDegToRad = M_PI / 180.0;
} // namespace

Eigen::Quaternionf toFloat(const Eigen::Quaterniond &q) {
  return q.cast<float>();
  //  return Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z());
}

Eigen::Quaterniond toDouble(const Eigen::Quaternionf &q) {
  return q.cast<double>();
  //  return Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
}

Eigen::MatrixXf toFloat(const Eigen::MatrixXd &m) { return m.cast<float>(); }
Eigen::MatrixXd toDouble(const Eigen::MatrixXf &m) { return m.cast<double>(); }

Eigen::Vector3d getPositionFromParameterServer(std::shared_ptr<rclcpp::Node> nh,
                                               const std::string &prefix) {
  double x = 0, y = 0, z = 0;
  //  const std::string prefix = "icp_localization_ros2/initial_pose/";
  try {
    if (!nh->has_parameter(prefix + "x")) {
      nh->declare_parameter(prefix + "x", 0.0);
    }
    if (!nh->has_parameter(prefix + "y")) {
      nh->declare_parameter(prefix + "y", 0.0);
    }
    if (!nh->has_parameter(prefix + "z")) {
      nh->declare_parameter(prefix + "z", 0.0);
    }
    x = nh->get_parameter(prefix + "x").as_double();
    y = nh->get_parameter(prefix + "y").as_double();
    z = nh->get_parameter(prefix + "z").as_double(); 
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(nh->get_logger(), "Failed to load position "
                                              << prefix << " " << e.what());
  }
  // bool res = nh.param(prefix + "x", x, 0.0) && nh.param(prefix + "y", y, 0.0)
  // &&
  //            nh.param(prefix + "z", z, 0.0);
  // if (!res) {
  //   ROS_ERROR_STREAM("Failed to load position " << prefix);
  // }
  return Eigen::Vector3d(x, y, z);
}
Eigen::Quaterniond
getOrientationFromParameterServer(std::shared_ptr<rclcpp::Node> nh,
                                  const std::string &prefix,
                                  bool isRPYInDegrees) {
  double r = 0, p = 0, y = 0;
  //  const std::string prefix = "icp_localization_ros2/initial_pose/";
  try {
    if (!nh->has_parameter(prefix + "roll")) {
      nh->declare_parameter(prefix + "roll", 0.0);
    }
    if (!nh->has_parameter(prefix + "pitch")) {
      nh->declare_parameter(prefix + "pitch", 0.0);
    }
    if (!nh->has_parameter(prefix + "yaw")) {
      nh->declare_parameter(prefix + "yaw", 0.0);
    }
    r = nh->get_parameter(prefix + "roll").as_double();
    p = nh->get_parameter(prefix + "pitch").as_double();
    y = nh->get_parameter(prefix + "yaw").as_double();
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(nh->get_logger(), "Failed to load orientation "
                                              << prefix << " " << e.what());
  }
  // bool res = nh.param(prefix + "roll", r, 0.0) &&
  //            nh.param(prefix + "pitch", p, 0.0) &&
  //            nh.param(prefix + "yaw", y, 0.0);
  // if (!res) {
  //   ROS_ERROR_STREAM("Failed to load orientation : " << prefix);
  // }

  if (isRPYInDegrees) {
    r *= kDegToRad;
    p *= kDegToRad;
    y *= kDegToRad;
  }
  return fromRPY(r, p, y).normalized();
}

} // namespace icp_loco
