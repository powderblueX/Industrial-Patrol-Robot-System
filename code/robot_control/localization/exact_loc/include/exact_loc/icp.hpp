#ifndef ICP_NODE_HPP
#define ICP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/float32.hpp>
#include <Eigen/Geometry>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include <atomic>
#include <mutex>
#include <queue>
#include <chrono>
class ICPNode : public rclcpp::Node
{
public:
  ICPNode();

private:
  void switch_symbol();
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  Eigen::Matrix4f genTransformation(const std::vector<float> &params);
  void decomposeMatrix4f(const Eigen::Matrix4f &matrix);
  Eigen::Matrix4f icp_point_to_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI>::Ptr tar,
                                     Eigen::Matrix4f &guess);
  void init_map2odom_();
  void broadcast_transform();
  void filter_points_by_height(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

  float voxel_grid_size_;
  std::string pcd_target_path_;
  std::string topic_name_;
  int max_iter_;
  std::vector<float> init_params_;
  rclcpp::TimerBase::SharedPtr timer_relocalization_;
  rclcpp::TimerBase::SharedPtr timer_broadcast_;
  std::atomic<bool> symbol_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_target_{new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source_{new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_target_filtered_{new pcl::PointCloud<pcl::PointXYZI>};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source_filtered_{new pcl::PointCloud<pcl::PointXYZI>};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::TransformStamped map2lidar_;
  geometry_msgs::msg::TransformStamped map2icp_lidar_;
  geometry_msgs::msg::TransformStamped odom2base_;
  geometry_msgs::msg::TransformStamped base2livox_;
  geometry_msgs::msg::TransformStamped map2livox_;
  double icp_score_;
  double max_score_;
  bool height_flag_;
  double height_threshold_;
  std::mutex mtx_; // 互斥锁
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr score_publisher_;
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> point_cloud_queue_;
};

#endif // ICP_NODE_HPP
