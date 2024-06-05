#include "fake_vel_transform/fake_vel_transform.hpp"

#include <tf2/utils.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

namespace fake_vel_transform
{
const std::string CMD_VEL_TOPIC = "/cmd_vel";
const std::string AFTER_TF_CMD_VEL = "/cmd_vel_chassis";
const std::string TRAJECTORY_TOPIC = "/local_plan";
const int TF_PUBLISH_FREQUENCY = 20;  // base_link to base_link_fake. Frequency in Hz.

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  // Declare and get the spin speed parameter
  this->declare_parameter<float>("spin_speed", -6.0);
  this->get_parameter("spin_speed", spin_speed_);

  // TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // Create Publisher and Subscriber
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    CMD_VEL_TOPIC, 1, std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  cmd_vel_chassis_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    AFTER_TF_CMD_VEL, rclcpp::QoS(rclcpp::KeepLast(1)));
  local_pose_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    TRAJECTORY_TOPIC, 1,
    std::bind(&FakeVelTransform::localPoseCallback, this, std::placeholders::_1));

  // Create a timer to publish the transform regularly
  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / TF_PUBLISH_FREQUENCY),
    std::bind(&FakeVelTransform::publishTransform, this));
}

// Get the local pose from planner
void FakeVelTransform::localPoseCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!msg || msg->poses.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty or invalid PoseArray message");
    return;
  }

  // Choose the pose based on the size of the poses array
  size_t index = std::min(msg->poses.size() / 4, msg->poses.size() - 1);
  const geometry_msgs::msg::Pose & selected_pose = msg->poses[index].pose;

  // Update current angle based on the difference between teb_angle and base_link_angle_
  double teb_angle = tf2::getYaw(selected_pose.orientation);
  current_angle_ = teb_angle - base_link_angle_;
}

// Transform the velocity from base_link to base_link_fake
void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf2_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    base_link_angle_ = tf2::getYaw(transform_stamped.transform.rotation);

    double angle_diff = -current_angle_;

    geometry_msgs::msg::Twist aft_tf_vel;
    aft_tf_vel.angular.z = (msg->angular.z != 0) ? spin_speed_ : 0;
    aft_tf_vel.linear.x = msg->linear.x * cos(angle_diff) + msg->linear.y * sin(angle_diff);
    aft_tf_vel.linear.y = -msg->linear.x * sin(angle_diff) + msg->linear.y * cos(angle_diff);

    cmd_vel_chassis_pub_->publish(aft_tf_vel);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
  }
}

// Publish transform from base_link to base_link_fake
void FakeVelTransform::publishTransform()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = get_clock()->now();
  t.header.frame_id = "base_link";
  t.child_frame_id = "base_link_fake";
  tf2::Quaternion q;
  q.setRPY(0, 0, current_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)