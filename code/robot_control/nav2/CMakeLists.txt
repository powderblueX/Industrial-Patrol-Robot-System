cmake_minimum_required(VERSION 3.8)
project(nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 添加可执行文件
add_executable(waypoint_navigator src/waypoint_navigator.cpp)
ament_target_dependencies(waypoint_navigator
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)

# 安装可执行文件
install(TARGETS
  waypoint_navigator
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()