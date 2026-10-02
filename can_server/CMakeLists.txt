cmake_minimum_required(VERSION 3.10)

project(CanServer VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(canlib
    ${PROJECT_SOURCE_DIR}/src/can_init.cpp
    ${PROJECT_SOURCE_DIR}/src/Servo_ctrl.cpp
    ${PROJECT_SOURCE_DIR}/src/myqueue.cpp
    
)

target_include_directories(canlib PUBLIC
    ${PROJECT_SOURCE_DIR}/include

)

add_executable(CanServer ./main.cpp)

target_link_libraries(CanServer PRIVATE canlib)