#include <memory>
#include <vector>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class WaypointNavigator : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    WaypointNavigator() : Node("waypoint_navigator")
    {
        // 创建Action客户端
        this->action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose");

        // 初始化导航点
        initializeWaypoints();

        // 创建定时器，延迟2秒后开始导航
        timer_ = this->create_wall_timer(
            2s,
            std::bind(&WaypointNavigator::startNavigation, this));
    }

private:
    void initializeWaypoints()
    {
        // 预设的导航点列表
        // 格式: [x, y, z, qx, qy, qz, qw]
        waypoints_ = {
            {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},    // 第一个点
            {2.0, 1.0, 0.0, 0.0, 0.0, 0.707, 0.707}, // 第二个点
            {1.0, 2.0, 0.0, 0.0, 0.0, 1.0, 0.0},    // 第三个点
            {0.0, 1.0, 0.0, 0.0, 0.0, -0.707, 0.707} // 第四个点
        };
        current_waypoint_index_ = 0;
    }

    void startNavigation()
    {
        if (!action_client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        sendGoal();
    }

    void sendGoal()
    {
        auto goal_msg = NavigateToPose::Goal();
        auto& pose = goal_msg.pose;

        // 设置目标点
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = waypoints_[current_waypoint_index_][0];
        pose.pose.position.y = waypoints_[current_waypoint_index_][1];
        pose.pose.position.z = waypoints_[current_waypoint_index_][2];
        pose.pose.orientation.x = waypoints_[current_waypoint_index_][3];
        pose.pose.orientation.y = waypoints_[current_waypoint_index_][4];
        pose.pose.orientation.z = waypoints_[current_waypoint_index_][5];
        pose.pose.orientation.w = waypoints_[current_waypoint_index_][6];

        RCLCPP_INFO(this->get_logger(), "Sending goal to waypoint %d", current_waypoint_index_);

        // 发送目标
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&WaypointNavigator::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&WaypointNavigator::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&WaypointNavigator::resultCallback, this, std::placeholders::_1);
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedbackCallback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Distance remaining: %.2f",
            feedback->distance_remaining);
    }

    void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        // 移动到下一个导航点
        current_waypoint_index_ = (current_waypoint_index_ + 1) % waypoints_.size();
        
        // 等待一段时间后发送下一个目标
        timer_ = this->create_wall_timer(
            2s,
            std::bind(&WaypointNavigator::sendGoal, this));
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::array<double, 7>> waypoints_;
    size_t current_waypoint_index_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}