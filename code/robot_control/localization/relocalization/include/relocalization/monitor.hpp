#pragma once

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "robot_msg/msg/log_msg.hpp"
#include "robot_msg/msg/lio_state_msg.hpp"
#include <yaml-cpp/yaml.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_srvs/srv/trigger.hpp"
#include "robot_msg/srv/global_relocalization.hpp"
#include "robot_msg/msg/global_relocalization_msg.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <atomic>
#include <iostream>
#include <unistd.h>    // fork, exec
#include <sys/types.h> // pid_t
#include <sys/wait.h>  // waitpid
#include <signal.h>    // kill
#include <cstdlib>
#include <thread>

#define SP_LOOP_DURATION 500
enum class MonitorState
{
    ACTIVE,
    OUT_OF_BOUNDS,
    RESTARTING
};

class MonitorNode : public rclcpp::Node
{
public:
    MonitorNode();
    ~MonitorNode();

private:
    void timer_callback();
    bool is_point_in_polygon(double px, double py, const std::vector<std::pair<double, double>> &poly);
    bool is_out_of_bounds(const geometry_msgs::msg::TransformStamped &transform,
                          const std::vector<std::pair<double, double>> &polygon);
    void transition_to(MonitorState new_state);
    void pid_start();
    void pid_stop();
    void pid_status_callback();
    rclcpp::Publisher<robot_msg::msg::LogMsg>::SharedPtr log_publisher_;
    rclcpp::Publisher<robot_msg::msg::LioStateMsg>::SharedPtr lio_state_publisher_;
    std::vector<std::pair<double, double>> bounds_;
    double z_min_, z_max_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::chrono::time_point<std::chrono::system_clock> last_time_;
    MonitorState current_state_;
    std::atomic<bool> sim_out_of_bounds_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;

    int wait_count_ = 0; // lio重启后等待时间
    //////////////////////////////////////////////////////////////////////////
    pid_t pid_;                                // 子进程ID
    int pid_status_;                           // 子进程状态
    std::atomic<bool> pid_running_{false};     // 子进程是否在运行
    rclcpp::TimerBase::SharedPtr timer_check_; // 定时器
    //////////////////////////////////////////////////////////////////////////
    rclcpp::Client<robot_msg::srv::GlobalRelocalization>::SharedPtr global_relocalization_client_;
    void send_global_relocalization_request();
    rclcpp::Subscription<robot_msg::msg::GlobalRelocalizationMsg>::SharedPtr global_relocalization_sub_;
    void global_relocalization_callback(const robot_msg::msg::GlobalRelocalizationMsg::SharedPtr msg);
    std::atomic<double> x_from_referee_{0.0}, y_from_referee_{0.0}, yaw_from_cboard_{0.0};
    bool color_is_match_map_;
};
