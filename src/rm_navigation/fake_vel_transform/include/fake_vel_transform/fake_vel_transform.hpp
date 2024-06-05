#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fake_vel_transform
{
class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void localPoseCallback(const nav_msgs::msg::Path::SharedPtr msg);

  void publishTransform();

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;

  // Broadcast tf from base_link to base_link_fake
  rclcpp::TimerBase::SharedPtr tf_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  geometry_msgs::msg::PoseStamped planner_local_pose_;
  double current_angle_;
  double base_link_angle_;
  float spin_speed_;
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
