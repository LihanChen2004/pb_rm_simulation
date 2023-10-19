/*
 * pcl_loader_node.cpp
 *
 *  Created on: Mar 4, 2021
 *      Author: jelavice
 */
#include "icp_localization_ros2/ICPlocalization.hpp"
#include "icp_localization_ros2/RangeDataAccumulator.hpp"
#include "icp_localization_ros2/common/typedefs.hpp"
#include "icp_localization_ros2/helpers.hpp"
#include <memory>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <thread>

using namespace icp_loco;
Pointcloud::Ptr mapCloud;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub;

const double kRadToDeg = 180.0 / M_PI;

Pointcloud::Ptr loadPointcloudFromPcd(const std::string &filename) {
  Pointcloud::Ptr cloud(new Pointcloud);
  pcl::PCLPointCloud2 cloudBlob;
  pcl::io::loadPCDFile(filename, cloudBlob);
  pcl::fromPCLPointCloud2(cloudBlob, *cloud);
  return cloud;
}

void publishCloud(
    std::shared_ptr<rclcpp::Node> node,
    Pointcloud::Ptr cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
    const std::string &frameId) {
  cloud->header.frame_id = frameId;
  cloud->header.seq = 0;
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.stamp = node->now(); 
  pub->publish(msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ICPlocalization>(rclcpp::NodeOptions{});
  node->initializeInternal();

  // ros::init(argc, argv, "tree_detection_eval_node");
  // ros::NodeHandle nh("~");

  // cloudPub = nh.advertise<sensor_msgs::PointCloud2>("icp_map", 1, true);
  cloudPub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "icp_map", rclcpp::QoS(rclcpp::KeepLast(1)));
  const std::string pclFilename =
      node->declare_parameter("pcd_file_path", "");

  mapCloud = loadPointcloudFromPcd(pclFilename);
  const Eigen::Quaterniond qInit = icp_loco::getOrientationFromParameterServer(
      node, "initial_pose.", true);
  const Eigen::Vector3d pInit = icp_loco::getPositionFromParameterServer(
      node, "initial_pose.");
  // ICPlocalization icp(nh);
  node->setMapCloud(*mapCloud);
  node->setInitialPose(pInit, qInit);
  node->initialize();
  std::cout << "succesfully initialized icp" << std::endl;

  publishCloud(node->shared_from_this(),mapCloud, cloudPub, node->getFixedFrame());

  // ros::AsyncSpinner spinner(3);
  // spinner.start();
  // ros::waitForShutdown();
  rclcpp::spin(node);
  return 0;
}
