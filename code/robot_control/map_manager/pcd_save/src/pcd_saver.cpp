#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class MapSaverComponent : public rclcpp::Node
{
public:
  MapSaverComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("map_saver_component", options)
  {
    // 创建服务客户端
    client_ = this->create_client<std_srvs::srv::Trigger>("map_save");
    // 等待服务可用
    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for the map_save service to be available...");
    }

    // 创建请求对象
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // 异步调用服务
    auto result_future =
        client_->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          try
          {
            auto response = future.get();
            if (response->success)
            {
              RCLCPP_INFO(this->get_logger(), "Map save service called successfully: %s", response->message.c_str());
            }
            else
            {
              RCLCPP_WARN(this->get_logger(), "Map save service failed: %s", response->message.c_str());
            }
          }
          catch (const std::exception& e)
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to call map_save service: %s", e.what());
          }
          // 无论成功或失败，调用rclcpp::shutdown()以关闭程序
          // rclcpp::shutdown();
        });
  }

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(MapSaverComponent)