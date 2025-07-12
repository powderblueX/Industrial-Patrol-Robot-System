#include "relocalization/icp.hpp"
// #include "icp.hpp"
using std::placeholders::_1;

ICPNode::ICPNode() : Node("icp_node")
{
  RCLCPP_INFO(this->get_logger(), "relocalization start");
  std::string config_file;
  this->declare_parameter<std::string>("config_file", "");
  this->get_parameter("config_file", config_file);
  // Load configuration from YAML file
  const auto config = YAML::LoadFile(config_file);
  pcd_target_path_ = config["pcd_target_path"].as<std::string>();
  topic_name_ = config["topic_name"].as<std::string>();
  init_params_ = config["init_params"].as<std::vector<float>>();
  voxel_grid_size_ = config["voxel_grid_size"].as<float>();
  height_flag_ = config["height_flag"].as<bool>();
  max_score_ = config["max_score"].as<double>();
  height_threshold_ = config["height_threshold"].as<double>();
  use_best_result_ = config["use_best_result"].as<bool>();
  // Load target PCD file
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_target_path_, *cloud_target_) == -1)
  {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read target PCD file.");
    return;
  }
  // Filter target point cloud
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
  voxel_grid.setInputCloud(cloud_target_);
  voxel_grid.filter(*cloud_target_filtered_);

  RCLCPP_INFO(this->get_logger(), "Loaded and filtered target PCD file.");
  symbol_.store(false);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_name_, 10, std::bind(&ICPNode::topic_callback, this, _1));
  init_map2odom_();
  register_ = std::make_shared<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
  timer_broadcast_ =
      this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ICPNode::broadcast_transform, this));
  timer_relocalization_ =
      this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&ICPNode::switch_symbol, this));
  score_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/relocalization_score", 10); 
}

void ICPNode::switch_symbol()
{
  symbol_.store(true);
}
void ICPNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  bool expected = symbol_.load();
  if (expected)
  {
    pcl::fromROSMsg(*msg, *cloud_source_);
    if (height_flag_)
    {
      filter_points_by_height(cloud_source_);
    }
    auto start = std::chrono::steady_clock::now();
    icp_point_to_plane(cloud_source_);
    auto end = std::chrono::steady_clock::now();
    auto diff = end - start;
    RCLCPP_INFO(this->get_logger(), "ICP time: %f ms", std::chrono::duration<double, std::milli>(diff).count());
    if (use_best_result_)
    {
      update_best_result(result_t_);
      update_map2odom(best_result_t_);
    }
    else if (icp_score_ < max_score_)
      update_map2odom(result_t_);
    decomposeMatrix4f(result_t_);
    symbol_.store(false);
  }
}
Eigen::Matrix4f ICPNode::genTransformation(const std::vector<float> &params)
{
  Eigen::Vector3f r(params[0], params[1], params[2]);
  Eigen::Vector3f t(params[3], params[4], params[5]);
  Eigen::AngleAxisf init_rotation_x(r.x(), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(r.y(), Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(r.z(), Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(t.x(), t.y(), t.z());
  return (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
}

void ICPNode::decomposeMatrix4f(const Eigen::Isometry3d &result)
{
  Eigen::Vector3d translation = result.translation();
  Eigen::Vector3d eulerAngles = result.rotation().eulerAngles(2, 1, 0) * (180.0 / M_PI);
  std::cout << "rotate: " << std::endl
            << eulerAngles << std::endl;
  std::cout << "p: " << std::endl
            << translation << std::endl;
}
void ICPNode::update_best_result(Eigen::Isometry3d result_t)
{
  if (icp_score_ < best_score_)
  {
    best_score_ = icp_score_;
    best_result_t_ = result_t;
  }
}
void ICPNode::icp_point_to_plane(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &raw_source)
{
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
  voxel_grid.setInputCloud(raw_source);
  voxel_grid.filter(*cloud_source_filtered_);
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setTransformationEpsilon(1e-10);
  icp.setMaxCorrespondenceDistance(2.0);
  icp.setMaximumIterations(50);
  icp.setInputSource(cloud_source_filtered_);
  icp.setInputTarget(cloud_target_filtered_);
  pcl::PointCloud<pcl::PointXYZI> final_cloud;
  Eigen::Matrix4f guess = previous_result_t_.matrix().cast<float>();
  icp.align(final_cloud, guess);
  if (icp.hasConverged())
  {
    icp_score_ = icp.getFitnessScore();
    RCLCPP_INFO(this->get_logger(), "ICP has converged with score: %f", icp.getFitnessScore());
    auto score_msg = std_msgs::msg::Float32();
    score_msg.data = icp_score_; // 设置浮点数值
    score_publisher_->publish(score_msg);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
  }
  Eigen::Matrix4f transformation = icp.getFinalTransformation();
  // Eigen::Matrix4f transformation转化为Eigen::Isometry3d
  result_t_.matrix() = transformation.cast<double>();
  previous_result_t_ = result_t_;
}
void ICPNode::init_map2odom_()
{
  std::lock_guard<std::mutex> lock(mtx_); // 加锁
  map2odom_.header.stamp = this->get_clock()->now();
  map2odom_.header.frame_id = "map";
  map2odom_.child_frame_id = "lidar_odom";
  map2odom_.transform.translation.x = init_params_[0];
  map2odom_.transform.translation.y = init_params_[1];
  map2odom_.transform.translation.z = init_params_[2];
  tf2::Quaternion q;
  q.setRPY(init_params_[5] * M_PI / 180.0, init_params_[4] * M_PI / 180.0, init_params_[3] * M_PI / 180.0);
  map2odom_.transform.rotation.x = q.x();
  map2odom_.transform.rotation.y = q.y();
  map2odom_.transform.rotation.z = q.z();
  map2odom_.transform.rotation.w = q.w();
  previous_result_t_ = tf2::transformToEigen(map2odom_.transform);
  result_t_ = tf2::transformToEigen(map2odom_.transform);
}
void ICPNode::broadcast_transform()
{
  std::lock_guard<std::mutex> lock(mtx_); // 加锁
  map2odom_.header.stamp = this->get_clock()->now();
  tf_broadcaster_->sendTransform(map2odom_);
}
void ICPNode::filter_points_by_height(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_by_height(new pcl::PointCloud<pcl::PointXYZI>);
  for (const auto &point : cloud->points)
  {
    if (point.z <= height_threshold_)
    {
      filtered_cloud_by_height->points.push_back(point);
    }
  }
  cloud = filtered_cloud_by_height;
  cloud->width = cloud_source_filtered_->points.size();
  cloud->height = 1; // 无序点云可以将 height 设置为 1
  RCLCPP_INFO(this->get_logger(), "size of filtered point cloud by height : %d", cloud->width);
}
void ICPNode::update_map2odom(Eigen::Isometry3d icp_result)
{
  std::lock_guard<std::mutex> lock(mtx_); // 加锁
  map2odom_.header.stamp = this->get_clock()->now();
  map2odom_.transform.translation.x = icp_result.translation().x();
  map2odom_.transform.translation.y = icp_result.translation().y();
  map2odom_.transform.translation.z = icp_result.translation().z();
  Eigen::Quaterniond q(icp_result.rotation());
  map2odom_.transform.rotation.x = q.x();
  map2odom_.transform.rotation.y = q.y();
  map2odom_.transform.rotation.z = q.z();
  map2odom_.transform.rotation.w = q.w();
}
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ICPNode>());
  rclcpp::shutdown();
  return 0;
}
