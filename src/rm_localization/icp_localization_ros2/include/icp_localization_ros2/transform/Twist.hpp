/*
 * Twist.hpp
 *
 *  Created on: Apr 26, 2021
 *      Author: jelavice
 */

#pragma once
#include "icp_localization_ros2/common/time.hpp"
#include <Eigen/Dense>

#include <geometry_msgs/msg/twist.hpp>

namespace icp_loco {

template <typename FloatType> class Twist3 {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Vector = Eigen::Matrix<FloatType, 3, 1>;

  Twist3() : linear_(Vector::Zero()), angular_(Vector::Zero()) {}
  Twist3(const Vector &lin, const Vector &ang) : linear_(lin), angular_(ang) {}

  template <typename OtherType> Twist3<OtherType> cast() const {
    return Twist3<OtherType>(linear_.template cast<OtherType>(),
                             angular_.template cast<OtherType>());
  }

  const Vector &linear() const { return linear_; }
  const Vector &angular() const { return angular_; }

private:
  Vector linear_;
  Vector angular_;
};

using Twist3d = Twist3<double>;
using Twist3f = Twist3<float>;

struct TimestampedTwist {
  Time time_;
  Twist3d twist_;
};

TimestampedTwist interpolate(const TimestampedTwist &start,
                             const TimestampedTwist &end, const Time &time);

Twist3d fromRos(const geometry_msgs::msg::Twist &msg);

} // namespace icp_loco
