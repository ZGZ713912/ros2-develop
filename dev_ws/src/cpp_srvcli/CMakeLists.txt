cmake_minimum_required(VERSION 3.5)
project(signal_filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 编译数据发生器节点
add_executable(data_generator src/data_generator_node.cpp)
ament_target_dependencies(data_generator
  rclcpp
  std_msgs
)

# 编译中值滤波器节点
add_executable(median_filter src/median_filter_node.cpp)
ament_target_dependencies(median_filter
  rclcpp
  std_msgs
)

# 编译低通滤波器节点
add_executable(lowpass_filter src/lowpass_filter_node.cpp)
ament_target_dependencies(lowpass_filter
  rclcpp
  std_msgs
)

# 安装可执行文件
install(TARGETS
  data_generator
  median_filter
  lowpass_filter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
    