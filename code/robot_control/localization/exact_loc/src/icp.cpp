#include "exact_loc/icp.hpp"
using std::placeholders::_1;

ICPNode::ICPNode() : Node("icp_node")
{
  RCLCPP_INFO(this->get_logger(), "start");
  std::string config_file;
  this->declare_parameter<std::string>("config_file", "");
  this->get_parameter("config_file", config_file);
  // Load configuration from YAML file
  const auto config = YAML::LoadFile(config_file);
  pcd_target_path_ = config["pcd_target_path"].as<std::string>();
  topic_name_ = config["topic_name"].as<std::string>();
  max_iter_ = config["max_iter"].as<int>();
  init_params_ = config["init_params"].as<std::vector<float>>();
  voxel_grid_size_ = config["voxel_grid_size"].as<float>();
  max_score_ = config["max_score"].as<float>();
  height_flag_ = config["height_flag"].as<bool>();
  height_threshold_ = config["height_threshold"].as<double>();
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
  timer_broadcast_ =
      this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ICPNode::broadcast_transform, this));
  timer_relocalization_ =
      this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&ICPNode::switch_symbol, this));

  score_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/exact_score", 10);
}

void ICPNode::switch_symbol()
{
  symbol_.store(true);
}
void ICPNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // 更新队列
  if (point_cloud_queue_.size() >= max_iter_)
  {
    point_cloud_queue_.pop();
  }
  point_cloud_queue_.push(msg);
  bool expected = symbol_.load();
  if (expected)
  {
    try
    {
      //计时
      auto start = std::chrono::high_resolution_clock::now();
      map2lidar_ = tf_buffer_->lookupTransform("map", "livox_frame", tf2::TimePointZero);
      while (!point_cloud_queue_.empty())
      {
        // 获取队列中的点云消息
        sensor_msgs::msg::PointCloud2::SharedPtr msg = point_cloud_queue_.front();
        pcl::PointCloud<pcl::PointXYZI> cloud_temp;
        pcl::fromROSMsg(*msg, cloud_temp);
        *cloud_source_ += cloud_temp;
        point_cloud_queue_.pop();
      }
      pcl::fromROSMsg(*msg, *cloud_source_);
      pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
      voxel_grid.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);

      voxel_grid.setInputCloud(cloud_source_);

      voxel_grid.filter(*cloud_source_filtered_);
      RCLCPP_INFO(this->get_logger(), "Received and filtered source point cloud.");
      cloud_source_->clear();
      // Perform ICP
      Eigen::Matrix4f guess = Eigen::Matrix4f::Identity(); // 初始化为4x4单位矩阵
      // 从 TransformStamped 中提取平移
      const auto &translation = map2lidar_.transform.translation;
      guess(0, 3) = translation.x;
      guess(1, 3) = translation.y;
      guess(2, 3) = translation.z;
      // 从 TransformStamped 中提取旋转
      const auto &rotation = map2lidar_.transform.rotation;
      Eigen::Quaternionf quat(rotation.w, rotation.x, rotation.y, rotation.z);
      // 将旋转转换为 3x3 矩阵并嵌入到 4x4 矩阵中
      guess.block<3, 3>(0, 0) = quat.toRotationMatrix();
      // Eigen::Matrix4f guess = genTransformation(init_params_);
      if(height_flag_)
      {
        filter_points_by_height(cloud_source_);
      }
      Eigen::Matrix4f transformation = icp_point_to_plane(cloud_source_filtered_, cloud_target_filtered_, guess);
      RCLCPP_INFO(this->get_logger(), "ICP completed. Transformation matrix:");
      std::cout << transformation << std::endl;
      decomposeMatrix4f(transformation);
      std::cout<<"Exact_loc score: "<<icp_score_<<std::endl;
      auto score_msg = std_msgs::msg::Float32();
      score_msg.data = icp_score_; // 设置浮点数值
      score_publisher_->publish(score_msg);
      
      if (icp_score_ < max_score_)
      {
        std::lock_guard<std::mutex> lock(mtx_); // 加锁

        tf2::Matrix3x3 tf2_rotation(transformation(0, 0), transformation(0, 1), transformation(0, 2),
                                    transformation(1, 0), transformation(1, 1), transformation(1, 2),
                                    transformation(2, 0), transformation(2, 1), transformation(2, 2));
        // 提取平移部分 (3x1 向量)
        tf2::Vector3 tf2_translation(transformation(0, 3), transformation(1, 3), transformation(2, 3));
        // 组合旋转和平移部分为 tf2::Transform
        tf2::Transform tf2_transform(tf2_rotation, tf2_translation);
        map2icp_lidar_.transform = tf2::toMsg(tf2_transform);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "ICP :bad result");
      }
      symbol_.store(false);
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = end - start;
      RCLCPP_INFO(this->get_logger(), "ICP took %f seconds", elapsed.count());
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
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

void ICPNode::decomposeMatrix4f(const Eigen::Matrix4f &matrix)
{
  Eigen::Vector3f translation;
  Eigen::Vector3f eulerAngles;
  // 提取平移部分 (前三个元素)
  translation = matrix.block<3, 1>(0, 3);
  // 提取旋转部分 (左上角 3x3 子矩阵)
  Eigen::Matrix3f rotationMatrix = matrix.block<3, 3>(0, 0);
  // 使用 Eigen 的函数将旋转矩阵转换为欧拉角 (ZYX 角，即绕 Z 轴、Y 轴、X 轴的旋转顺序)
  Eigen::Vector3f euler = rotationMatrix.eulerAngles(2, 1, 0);
  // 将欧拉角返回
  eulerAngles = euler * (180.0 / M_PI);
  std::cout << "rotate: " << std::endl
            << eulerAngles << std::endl;
  std::cout << "p: " << std::endl
            << translation << std::endl;
}

Eigen::Matrix4f ICPNode::icp_point_to_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr src,
                                            pcl::PointCloud<pcl::PointXYZI>::Ptr tar, Eigen::Matrix4f &guess)
{
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setTransformationEpsilon(1e-10);
  icp.setMaxCorrespondenceDistance(2.0);
  icp.setMaximumIterations(50);
  icp.setInputSource(src);
  icp.setInputTarget(tar);
  pcl::PointCloud<pcl::PointXYZI> final_cloud;
  icp.align(final_cloud, guess);

  if (icp.hasConverged())
  {
    icp_score_ = icp.getFitnessScore();
    RCLCPP_INFO(this->get_logger(), "ICP has converged with score: %f", icp.getFitnessScore());
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
  }

  return icp.getFinalTransformation();
}

void ICPNode::init_map2odom_()
{
  std::lock_guard<std::mutex> lock(mtx_); // 加锁
  map2icp_lidar_.header.stamp = this->get_clock()->now();
  map2icp_lidar_.header.frame_id = "map";
  map2icp_lidar_.child_frame_id = "icp_lidar";
  map2icp_lidar_.transform.translation.x = init_params_[0];
  map2icp_lidar_.transform.translation.y = init_params_[1];
  map2icp_lidar_.transform.translation.z = init_params_[2];
  tf2::Quaternion q;
  q.setRPY(init_params_[5] * M_PI / 180.0, init_params_[4] * M_PI / 180.0, init_params_[3] * M_PI / 180.0);
  map2icp_lidar_.transform.rotation.x = q.x();
  map2icp_lidar_.transform.rotation.y = q.y();
  map2icp_lidar_.transform.rotation.z = q.z();
  map2icp_lidar_.transform.rotation.w = q.w();
}
void ICPNode::broadcast_transform()
{
  map2icp_lidar_.header.stamp = this->get_clock()->now();
  tf_broadcaster_->sendTransform(map2icp_lidar_);
  RCLCPP_INFO(this->get_logger(), "Broadcasted transform.");
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ICPNode>());
  rclcpp::shutdown();
  return 0;
}
