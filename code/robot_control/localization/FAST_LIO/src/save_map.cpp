#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using std_srvs::srv::Trigger;
using namespace std::chrono_literals;

class MapSaveClient : public rclcpp::Node
{
public:
  MapSaveClient() : Node("map_save_client")
  {
    client_ = this->create_client<Trigger>("map_save");

    // 等待服务可用
    while (!client_->wait_for_service(1s))
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for the map_save service to be available...");
    }

    // 创建请求对象
    auto request = std::make_shared<Trigger::Request>();

    // 异步发送请求
    using ServiceResponseFuture = rclcpp::Client<Trigger>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
      auto response = future.get();
      if (response->success)
      {
        RCLCPP_INFO(this->get_logger(), "Map saved successfully: %s", response->message.c_str());
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to save map: %s", response->message.c_str());
      }
      rclcpp::shutdown();
    };

    // 发送请求并注册回调函数
    auto result = client_->async_send_request(request, response_received_callback);
  }

private:
  rclcpp::Client<Trigger>::SharedPtr client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapSaveClient>());
  rclcpp::shutdown();
  return 0;
}
