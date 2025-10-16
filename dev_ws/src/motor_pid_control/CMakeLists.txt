cmake_minimum_required(VERSION 3.5)
project(motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 编译速度滤波器节点
add_executable(velocity_filter src/velocity_filter_node.cpp)
ament_target_dependencies(velocity_filter
  rclcpp
  std_msgs
)

# 编译PID控制器节点
add_executable(pid_controller src/pid_controller_node.cpp)
ament_target_dependencies(pid_controller
  rclcpp
  std_msgs
)

# 安装可执行文件
install(TARGETS
  velocity_filter
  pid_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
    