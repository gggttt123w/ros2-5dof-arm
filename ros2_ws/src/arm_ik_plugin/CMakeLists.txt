cmake_minimum_required(VERSION 3.10)
project(arm_ik_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_library(arm_ik_plugin SHARED
  src/arm_analytic_ik_plugin.cpp
)
target_include_directories(arm_ik_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(arm_ik_plugin
  moveit_core
  pluginlib
  rclcpp
  tf2_eigen
  geometry_msgs
  moveit_msgs
)

# kinematics::KinematicsBase 定义在 moveit_core，所以插件描述注册到 moveit_core
pluginlib_export_plugin_description_file(moveit_core arm_ik_plugin.xml)

install(TARGETS arm_ik_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_package()
