cmake_minimum_required(VERSION 3.8)
project(my_robot_arm_moveit_config)

find_package(ament_cmake REQUIRED)

# ★★★ 这三行不能少，否则 launch / config 进不了 install ★★★
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
