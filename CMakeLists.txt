cmake_minimum_required(VERSION 3.10)

project(CanServer VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ─────────── 依赖 ───────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ─────────── 库：不依赖 ROS 的 SocketCAN + 舵机协议层 ───────────
add_library(arm_can SHARED
    src/can_init.cpp
    src/Servo_ctrl.cpp
)
target_include_directories(arm_can PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(arm_can PUBLIC Threads::Threads)

# ─────────── 库：ros2_control 硬件接口插件 ───────────
add_library(arm_system_hardware SHARED
    src/arm_system_hardware.cpp
)
target_include_directories(arm_system_hardware PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(arm_system_hardware PUBLIC
    arm_can
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp_lifecycle::rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(
    hardware_interface arm_system_hardware.xml)

# ─────────── 可执行文件 ───────────
add_executable(arm_server src/arm_server_node.cpp)
target_link_libraries(arm_server arm_can rclcpp::rclcpp ${sensor_msgs_TARGETS})

add_executable(can_test ./main.cpp)
target_link_libraries(can_test arm_can)

# ─────────── 安装 ───────────
# 可执行文件
install(TARGETS arm_server can_test
        DESTINATION lib/${PROJECT_NAME})

# ★ 插件和它依赖的库都必须在 lib/（pluginlib 从这里加载）
install(TARGETS arm_system_hardware arm_can
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib)

# 头文件（给别包 include 用）
install(DIRECTORY include/
        DESTINATION include)

# ★★★ 就是这行被删了 —— launch 和 config 必须装到 share/ ★★★
install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

ament_package()