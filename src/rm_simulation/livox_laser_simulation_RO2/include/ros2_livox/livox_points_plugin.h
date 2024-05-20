//
// Created by lfc on 2021/2/28.
//

#ifndef SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H
#define SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo_ros/node.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "livox_ode_multiray_shape.h"

namespace gazebo
{
   struct AviaRotateInfo
   {
      double time;
      double azimuth;
      double zenith;
   };


   class LivoxPointsPlugin : public RayPlugin
   {
   public:
      LivoxPointsPlugin();

      virtual ~LivoxPointsPlugin();

      void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

   private:
      ignition::math::Angle AngleMin() const;

      ignition::math::Angle AngleMax() const;

      double GetAngleResolution() const GAZEBO_DEPRECATED(7.0);

      double AngleResolution() const;

      double GetRangeMin() const GAZEBO_DEPRECATED(7.0);

      double RangeMin() const;

      double GetRangeMax() const GAZEBO_DEPRECATED(7.0);

      double RangeMax() const;

      double GetRangeResolution() const GAZEBO_DEPRECATED(7.0);

      double RangeResolution() const;

      int GetRayCount() const GAZEBO_DEPRECATED(7.0);

      int RayCount() const;

      int GetRangeCount() const GAZEBO_DEPRECATED(7.0);

      int RangeCount() const;

      int GetVerticalRayCount() const GAZEBO_DEPRECATED(7.0);

      int VerticalRayCount() const;

      int GetVerticalRangeCount() const GAZEBO_DEPRECATED(7.0);

      int VerticalRangeCount() const;

      ignition::math::Angle VerticalAngleMin() const;

      ignition::math::Angle VerticalAngleMax() const;

      double GetVerticalAngleResolution() const GAZEBO_DEPRECATED(7.0);

      double VerticalAngleResolution() const;

   protected:
      virtual void OnNewLaserScans();

   private:
      void InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                          boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape);

      void InitializeScan(msgs::LaserScan *&scan);

      void SendRosTf(const ignition::math::Pose3d &pose, const std::string &father_frame, const std::string &child_frame);

      boost::shared_ptr<physics::LivoxOdeMultiRayShape> rayShape;
      gazebo::physics::CollisionPtr laserCollision;
      physics::EntityPtr parentEntity;
      transport::PublisherPtr scanPub;

      sdf::ElementPtr sdfPtr;
      transport::NodePtr node;
      msgs::LaserScanStamped laserMsg;
      gazebo::sensors::SensorPtr raySensor;
      std::vector<AviaRotateInfo> aviaInfos;

      gazebo_ros::Node::SharedPtr node_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2_pub;
      rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub;
      
      std::string parent_name;
      std::string child_name;
      int64_t samplesStep = 0;
      int64_t currStartIndex = 0;
      int64_t maxPointSize = 1000;
      int64_t downSample = 1;

      double maxDist = 400.0;
      double minDist = 0.1;
   };

} // namespace gazebo

#endif // SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H


