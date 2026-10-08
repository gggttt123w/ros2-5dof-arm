cmake_minimum_required(VERSION 3.10)

project(CanServer VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(arm_can SHARED
    ${PROJECT_SOURCE_DIR}/src/can_init.cpp
    ${PROJECT_SOURCE_DIR}/src/Servo_ctrl.cpp
    
)

target_include_directories(arm_can PUBLIC
    ${PROJECT_SOURCE_DIR}/include

)

target_link_libraries(arm_can PUBLIC Threads::Threads)

add_executable(arm_server src/arm_server_node.cpp)
target_link_libraries(arm_server arm_can rclcpp::rclcpp ${sensor_msgs_TARGETS})

add_executable(can_test ./main.cpp)
target_link_libraries(can_test arm_can)

install(TARGETS arm_server can_test DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()