#include "relocalization/monitor.hpp"
using std::placeholders::_1;
MonitorNode::MonitorNode()
    : Node("monitor_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      current_state_(MonitorState::ACTIVE)
{
    std::string config_file;
    this->declare_parameter<std::string>("config_file", "");
    this->get_parameter("config_file", config_file);
    // Load configuration from YAML file
    const auto config = YAML::LoadFile(config_file);
    if (config["team_color"] && config["map_color"])
    {
        std::string team_color = config["team_color"].as<std::string>();
        std::string map_color = config["map_color"].as<std::string>();
        if (team_color == "red" && map_color == "red")
        {
            color_is_match_map_ = true;
        }
        else if (team_color == "blue" && map_color == "blue")
        {
            color_is_match_map_ = true;
        }
        else
        {
            color_is_match_map_ = false;
        }
        std::cout << "team_color: " << team_color << ", map_color: " << map_color << std::endl;
        std::cout << "color_is_match_map_: " << (color_is_match_map_ ? "true" : "false") << std::endl;
    }
    else
    {
        std::cerr << "team_color or map_color not found in the YAML file." << std::endl;
    }
    // 读取 z_min 和 z_max
    if (config["min_z"] && config["max_z"])
    {
        z_min_ = config["min_z"].as<double>();
        z_max_ = config["max_z"].as<double>();
        std::cout << "z_min: " << z_min_ << ", z_max: " << z_max_ << std::endl;
    }
    else
    {
        std::cerr << "z_min or z_max not found in the YAML file." << std::endl;
    }
    // 读取 bounds_point 数组
    if (config["bounds_point"])
    {
        const auto &bounds_point = config["bounds_point"];
        for (size_t i = 0; i < bounds_point.size(); ++i)
        {
            double x = bounds_point[i]["x"].as<double>();
            double y = bounds_point[i]["y"].as<double>();
            bounds_.emplace_back(x, y);
            std::cout << "Point " << i + 1 << ": x = " << x << ", y = " << y << std::endl;
        }
    }
    else
    {
        std::cerr << "bounds_point not found in the YAML file." << std::endl;
    }
    sim_out_of_bounds_.store(false);
    srv_reset_ = this->create_service<std_srvs::srv::Trigger>(
        "sim_out_of_bounds/reset", // 推荐相对路径
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            (void)request;
            sim_out_of_bounds_.store(true);
            RCLCPP_INFO(this->get_logger(), "Reset signal received.");
            response->success = true;
            response->message = "Reset signal sent.";
        });

    if (!srv_reset_)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create reset service.");
    }
    global_relocalization_client_ = this->create_client<robot_msg::srv::GlobalRelocalization>("/global_relocalization");
    if (!global_relocalization_client_)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create global relocalization client.");
    }
    log_publisher_ = this->create_publisher<robot_msg::msg::LogMsg>("/log_topic", 100);
    lio_state_publisher_ = this->create_publisher<robot_msg::msg::LioStateMsg>("/lio_state", 10);
    global_relocalization_sub_ = this->create_subscription<robot_msg::msg::GlobalRelocalizationMsg>(
        "/global_relocalization_param", 10, std::bind(&MonitorNode::global_relocalization_callback, this, _1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(SP_LOOP_DURATION),
        std::bind(&MonitorNode::timer_callback, this));

    pid_start();
    timer_check_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&MonitorNode::pid_status_callback, this));
    RCLCPP_INFO(this->get_logger(), "MonitorNode 启动完成");
}
void MonitorNode::global_relocalization_callback(const robot_msg::msg::GlobalRelocalizationMsg::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Global Relocalization Callback");
    double x = msg->translation.x;
    double y = msg->translation.y;
    double yaw = msg->rotation.x/180*M_PI; // todo 根据红蓝方将角度进行转换
    if (!color_is_match_map_)
    {
        yaw = -yaw;
    }
    x_from_referee_.store(x);
    y_from_referee_.store(y);
    yaw_from_cboard_.store(yaw);
    // RCLCPP_INFO(this->get_logger(), "Translation: x = %f, y = %f", x, y);
    // RCLCPP_INFO(this->get_logger(), "Rotation: z = %f", yaw);
}
void MonitorNode::send_global_relocalization_request()
{
    // 等待服务启动
    while (rclcpp::ok() && !global_relocalization_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for service to become available...");
    }

    auto request = std::make_shared<robot_msg::srv::GlobalRelocalization::Request>();

    request->translation.x = x_from_referee_.load();
    request->translation.y = y_from_referee_.load();
    request->translation.z = 0.0;

    // 欧拉角转四元数
    double yaw = yaw_from_cboard_.load();
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    request->rotation.x = quaternion.x();
    request->rotation.y = quaternion.y();
    request->rotation.z = quaternion.z();
    request->rotation.w = quaternion.w();
    std::cout<<"--------------------x: "<<quaternion.x()<<std::endl;
    auto future = global_relocalization_client_->async_send_request(
        request,
        [this](rclcpp::Client<robot_msg::srv::GlobalRelocalization>::SharedFuture future_result)
        {
            auto response = future_result.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Service succeeded: %s", response->message.c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Service failed: %s", response->message.c_str());
            }
        });
}
MonitorNode::~MonitorNode()
{
    if (pid_ > 0)
    {
        kill(pid_, SIGINT);
        waitpid(pid_, nullptr, 0);
    }
    RCLCPP_INFO(this->get_logger(), "MonitorNode 销毁");
}
void MonitorNode::timer_callback()
{
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "获取变换失败: %s", ex.what());
        return;
    }
    switch (current_state_)
    {
    case MonitorState::ACTIVE:
    {
        bool sim_flag = sim_out_of_bounds_.load();
        if (is_out_of_bounds(transform, bounds_) || sim_flag)
        {
            sim_out_of_bounds_.store(false);
            /////// todo 停止运动
            /////// todo 关闭里程计，获取里程计状态
            pid_stop();
            last_time_ = std::chrono::system_clock::now();
            transition_to(MonitorState::OUT_OF_BOUNDS);
        }
        break;
    }
    case MonitorState::OUT_OF_BOUNDS:
    {
        bool running = pid_running_.load();
        if (std::chrono::system_clock::now() - last_time_ > std::chrono::seconds(5) && running == false) ///// todo 里程计成功关闭
        {
            ///// todo 重启里程计
            pid_start();
            send_global_relocalization_request();
            transition_to(MonitorState::RESTARTING);
        }
        break;
    }
    case MonitorState::RESTARTING:
    { ///// todo 检测到机器人回到边界内，恢复正常状态
        double wait_time = static_cast<double>(wait_count_ * SP_LOOP_DURATION / 1000.0);
        std::cout << "wait_time: " << wait_time << std::endl;
        if (!is_out_of_bounds(transform, bounds_) && wait_time < 7)
        {
            wait_count_++;
        }
        else if (!is_out_of_bounds(transform, bounds_) && wait_time >= 7)
        {
            wait_count_ = 0;
            transition_to(MonitorState::ACTIVE);
        }
        else if (is_out_of_bounds(transform, bounds_) && wait_time >= 7)
        {
            wait_count_ = 0;
            pid_stop();
            last_time_ = std::chrono::system_clock::now();
            transition_to(MonitorState::OUT_OF_BOUNDS);
        }
        else
        {
            wait_count_++;
        }
        break;
    }
    default:
    {
        RCLCPP_ERROR(this->get_logger(), "error state");
        break;
    }
    }
    robot_msg::msg::LioStateMsg lio_state_msg;
    lio_state_msg.state = static_cast<int>(current_state_);
    lio_state_publisher_->publish(lio_state_msg);
}
bool MonitorNode::is_point_in_polygon(double px, double py, const std::vector<std::pair<double, double>> &poly)
{
    if (poly.size() != 4)
    {
        RCLCPP_ERROR(this->get_logger(), "多边形顶点数量必须为4");
        return false;
    }

    auto cross = [](double x1, double y1, double x2, double y2)
    {
        return x1 * y2 - y1 * x2;
    };

    bool all_positive = true;
    bool all_negative = true;

    for (size_t i = 0; i < 4; ++i)
    {
        double x1 = poly[i].first;
        double y1 = poly[i].second;
        double x2 = poly[(i + 1) % 4].first;
        double y2 = poly[(i + 1) % 4].second;

        // 向量 AB 和 AP 的叉积
        double dx1 = x2 - x1;
        double dy1 = y2 - y1;
        double dx2 = px - x1;
        double dy2 = py - y1;

        double cp = cross(dx1, dy1, dx2, dy2);

        if (cp <= 0)
            all_positive = false;
        if (cp > 0)
            all_negative = false;
    }

    return all_positive || all_negative; // 全正或全负 ⇒ 点在多边形内
}

bool MonitorNode::is_out_of_bounds(const geometry_msgs::msg::TransformStamped &transform,
                                   const std::vector<std::pair<double, double>> &polygon)
{
    double px = transform.transform.translation.x;
    double py = transform.transform.translation.y;
    double pz = transform.transform.translation.z;
    if (pz < z_min_ || pz > z_max_)
    {
        RCLCPP_WARN(this->get_logger(), "z out of bounds: %f", pz);
        return true;
    }
    // 点在多边形内部 ⇒ 返回 false（没有越界）
    return !is_point_in_polygon(px, py, polygon);
}

void MonitorNode::transition_to(MonitorState new_state)
{
    robot_msg::msg::LogMsg log_msg;
    log_msg.node_name = "monitor_node";
    log_msg.level = "WARNING";
    std::string state_str;
    switch (new_state)
    {
    case MonitorState::ACTIVE:
        state_str = "ACTIVE";
        RCLCPP_WARN(this->get_logger(), "switch to ACTIVE state");
        break;
    case MonitorState::RESTARTING:
        state_str = "RESTARTING";
        RCLCPP_WARN(this->get_logger(), "switch to RESTARTING state");
        break;
    case MonitorState::OUT_OF_BOUNDS:
        state_str = "OUT_OF_BOUNDS";
        RCLCPP_WARN(this->get_logger(), "switch to OUT_OF_BOUNDS state");
        break;
    }

    log_msg.message = "State changed to " + state_str;
    log_msg.stamp = this->now();
    log_publisher_->publish(log_msg);
    current_state_ = new_state;
}

void MonitorNode::pid_start()
{
    pid_ = fork();
    if (pid_ < 0)
    {
        std::cerr << "Failed to fork" << std::endl;
    }
    if (pid_ == 0)
    {
        // 子进程：执行 ROS2 launch 命令，自动 source 环境
        execlp("bash", "bash", "-c",
               "source /opt/ros/humble/setup.bash && "
               "ros2 launch point_lio mapping_mid360.launch.py",
               nullptr);
        std::cerr << "Failed to exec ros2 launch" << std::endl;
        exit(1); // exec 失败必须退出子进程
    }
    else if (pid_ > 0)
    {
        std::cout << "[Parent] ROS2 process started, pid = " << pid_ << std::endl;
    }
    else
    {
        std::cerr << "[Parent] Failed to fork" << std::endl;
    }
    pid_running_.store(true);
}

void MonitorNode::pid_stop()
{
    pid_t result = waitpid(pid_, &pid_status_, WNOHANG);
    if (result == 0)
    {
        // 子进程还在运行
        kill(pid_, SIGINT);
        std::cout << "Stopping child process with PID: " << pid_ << std::endl;
    }
}

void MonitorNode::pid_status_callback()
{
    bool running = pid_running_.load();
    std::cout << "monitor running ...." << std::endl;
    if (running == false)
    {
        std::cout << "Child process is not running." << std::endl;
        return;
    }
    pid_t result = waitpid(pid_, &pid_status_, WNOHANG);
    if (result == 0)
    {
        // 子进程还在运行，继续等
        std::cout << "[INFO] ROS2 process still running..." << std::endl;
    }
    else if (result == pid_)
    {
        pid_running_.store(false);
        // 子进程已退出
        if (WIFEXITED(pid_status_))
        {
            std::cout << "[INFO] ROS2 process exited normally with code "
                      << WEXITSTATUS(pid_status_) << std::endl;
        }
        else if (WIFSIGNALED(pid_status_))
        {
            std::cout << "[INFO] ROS2 process terminated by signal "
                      << WTERMSIG(pid_status_) << std::endl;
        }
        else
        {
            std::cout << "[INFO] ROS2 process exited abnormally." << std::endl;
        }
    }
    else
    {
        std::cerr << "[ERROR] waitpid() failed" << std::endl;
    }
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonitorNode>());
    rclcpp::shutdown();
    return 0;
}