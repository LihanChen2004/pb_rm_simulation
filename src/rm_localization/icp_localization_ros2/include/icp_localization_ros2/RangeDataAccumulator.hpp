/*
 * RangeDataAccumulator.hpp
 *
 *  Created on: Apr 23, 2021
 *      Author: jelavice
 */

#pragma once

#include "icp_localization_ros2/Parameters.hpp"
#include "icp_localization_ros2/common/time.hpp"
#include "icp_localization_ros2/common/typedefs.hpp"
#include "icp_localization_ros2/helpers.hpp"
#include <memory>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

namespace icp_loco {

struct TimedRangeData {
  Time timestamp_;
  DP data_;
};

class RangeDataAccumulator {

public:
  RangeDataAccumulator() = default;
  virtual ~RangeDataAccumulator() = default;

  void setParam(const RangeDataAccumulatorParam &p);
  const RangeDataAccumulatorParam &getParam() const;

  bool isAccumulatedRangeDataReady() const;
  void addRangeData(const DP &rangeData, const Time &t);
  const TimedRangeData &getAccumulatedRangeData() const;
  const TimedRangeData &popAccumulatedRangeData() const;
  const Time &getAccumulatedRangeDataTimestamp() const;
  void resetAccumulatedRangeData() const;

protected:
  bool isAccumulatedTargetNumRangeData() const;

private:
  TimedRangeData workingRangeData_;
  RangeDataAccumulatorParam param_;
  int currentNumRangeDataAccumulated_ = 0;
  TimedRangeData accumulatedRangeData_;
  mutable bool isRangeDataReady_ = false;
  mutable std::mutex accumulatedDataMutex_;
};

class RangeDataAccumulatorRos : public RangeDataAccumulator {
public:
  RangeDataAccumulatorRos(const std::shared_ptr<rclcpp::Node> nh);
  ~RangeDataAccumulatorRos() override;

  void initialize();
  void cloudCallback(const sensor_msgs::msg::PointCloud2 &msg);
  void setParam(const RangeDataAccumulatorParamRos &p);
  const RangeDataAccumulatorParamRos &getParam() const;
  void publishAccumulatedRangeDataWorker() const;

private:
  std::shared_ptr<rclcpp::Node> nh_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSubscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulatedRangeDataPub_;
  //  ros::NodeHandle nh_;
  //  ros::Subscriber cloudSubscriber_;
  //  ros::Publisher accumulatedRangeDataPub_;
  RangeDataAccumulatorParamRos param_;
  std::thread publisherWorker_;
  std::string frameId_ = "map";
};

} // namespace icp_loco
