#ifndef ICP_REGISTRATION_HPP
#define ICP_REGISTRATION_HPP

// std
#include <filesystem>
#include <mutex>

// ros
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

namespace icp {
using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;

// Get initial map to odom pose estimation using ICP algorithm
class IcpNode : public rclcpp::Node {
public:
  IcpNode(const rclcpp::NodeOptions &options);
  ~IcpNode();
private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void initialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  static PointCloudXYZIN::Ptr addNorm(PointCloudXYZI::Ptr cloud);

  // Eigen::Matrix4d align(PointCloudXYZI::Ptr source,
  //                       const Eigen::Matrix4d &init_guess);

  Eigen::Matrix4d multiAlignSync(PointCloudXYZI::Ptr source,
                                 const Eigen::Matrix4d &init_guess);

  // ROS2 part
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
  // rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mutex_;
  std::unique_ptr<std::thread> tf_publisher_thread_;

  // Voxelfilter used to downsample the pointcloud
  pcl::VoxelGrid<pcl::PointXYZI> voxel_rough_filter_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_refine_filter_;

  // ICP
  int rough_iter_;
  int refine_iter_;
  pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_rough_;
  pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_refine_;

  // Store
  PointCloudXYZI::Ptr cloud_in_;
  PointCloudXYZIN::Ptr refine_map_;
  PointCloudXYZIN::Ptr rough_map_;
  geometry_msgs::msg::TransformStamped map_to_odom_;
  std::filesystem::path pcd_path_;
  std::string map_frame_id_, odom_frame_id_, range_odom_frame_id_, laser_frame_id_;
  bool success_;
  double score_;
  double thresh_;
  double xy_offset_;
  double yaw_offset_;
  double yaw_resolution_;
  geometry_msgs::msg::Pose initial_pose_;

  bool is_ready_;
  bool first_scan_;
};
} // namespace icp
#endif