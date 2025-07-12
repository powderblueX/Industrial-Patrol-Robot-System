/*转发/path到map坐标系*/

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/convert.h"  // 对于需要转换的类型
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"  // 对于 geometry_msgs::PoseStamped


class PathTransformer : public rclcpp::Node
{
public:
    PathTransformer() : Node("path_transformer")
    {
        // 创建 tf2 缓冲区和监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 订阅 /path 话题
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&PathTransformer::path_callback, this, std::placeholders::_1));

        // 发布到 /map/path 话题
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/map/path", 10);
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        try
        {
            // 获取从当前坐标系到 map 坐标系的变换
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "map",        // 目标坐标系
                msg->header.frame_id, // 原始坐标系
                tf2::TimePointZero); // 当前时刻

            nav_msgs::msg::Path transformed_path;
            transformed_path.header.stamp = this->now();
            transformed_path.header.frame_id = "map";

            // 遍历路径点，进行坐标系变换
            for (const auto &pose : msg->poses)
            {
                geometry_msgs::msg::PoseStamped transformed_pose;
                tf2::doTransform(pose, transformed_pose, transform);
                transformed_path.poses.push_back(transformed_pose);
            }

            // 发布转换后的路径
            path_pub_->publish(transformed_path);
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", e.what());
        }
    }

    // 订阅者和发布者
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // tf2 相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathTransformer>());
    rclcpp::shutdown();
    return 0;
}
