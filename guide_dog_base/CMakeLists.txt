cmake_minimum_required(VERSION 3.8)
project(guide_dog_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_api REQUIRED)

# /cmd_vel -> /api/sport/request (Move). Was cmd_vel_unitree_bridge in
# ~/go2_bringup_ws, run by the go2-cmdvel-bridge systemd service.
add_executable(cmd_vel_bridge src/cmd_vel_bridge.cpp)
ament_target_dependencies(cmd_vel_bridge
  rclcpp
  geometry_msgs
  unitree_api
)

install(TARGETS
  cmd_vel_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
