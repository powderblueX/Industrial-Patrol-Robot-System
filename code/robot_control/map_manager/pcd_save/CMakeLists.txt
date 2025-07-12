cmake_minimum_required(VERSION 3.8)
project(pcd_save)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
# 创建库
add_library(pcd_saver_component SHARED src/pcd_saver.cpp)

# 添加依赖项
ament_target_dependencies(pcd_saver_component
  rclcpp
  rclcpp_components
  std_srvs
)

# 注册组件
rclcpp_components_register_node(pcd_saver_component
  PLUGIN "MapSaverComponent"
  EXECUTABLE pcd_saver_node)

# 安装
install(TARGETS
  pcd_saver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY  launch 
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
