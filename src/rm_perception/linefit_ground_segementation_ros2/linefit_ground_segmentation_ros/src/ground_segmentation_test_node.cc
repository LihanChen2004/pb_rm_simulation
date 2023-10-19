#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ground_segmentation/ground_segmentation.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ground_segmentation");
  std::string cloud_file;
  if (node->get_parameter("point_cloud_file", cloud_file)) {
    std::cout << "Point cloud file is \"" << cloud_file << "\"\n";
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::io::loadPLYFile(cloud_file, cloud);

    GroundSegmentationParams params;
    params.visualize = node->declare_parameter("visualize", params.visualize);
    params.n_bins = node->declare_parameter("n_bins", params.n_bins);
    params.n_segments =
        node->declare_parameter("n_segments", params.n_segments);
    params.max_dist_to_line =
        node->declare_parameter("max_dist_to_line", params.max_dist_to_line);
    params.max_slope = node->declare_parameter("max_slope", params.max_slope);
    params.min_slope = node->declare_parameter("min_slope", params.min_slope);
    params.long_threshold =
        node->declare_parameter("long_threshold", params.long_threshold);
    params.max_long_height =
        node->declare_parameter("max_long_height", params.max_long_height);
    params.max_start_height =
        node->declare_parameter("max_start_height", params.max_start_height);
    params.sensor_height =
        node->declare_parameter("sensor_height", params.sensor_height);
    params.line_search_angle =
        node->declare_parameter("line_search_angle", params.line_search_angle);
    params.n_threads = node->declare_parameter("n_threads", params.n_threads);
        // Params that need to be squared.
    double r_min, r_max, max_fit_error;
    if (node->get_parameter("r_min", r_min)) {
      params.r_min_square = r_min * r_min;
    }
    if (node->get_parameter("r_max", r_max)) {
      params.r_max_square = r_max * r_max;
    }
    if (node->get_parameter("max_fit_error", max_fit_error)) {
      params.max_error_square = max_fit_error * max_fit_error;
    }

    GroundSegmentation segmenter(params);
    std::vector<int> labels;

    segmenter.segment(cloud, &labels);

    rclcpp::spin(node);
  } else {
    std::cerr << "No point cloud file given\n";
  }
  rclcpp::shutdown();
  return 0;
}
