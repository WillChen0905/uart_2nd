cmake_minimum_required(VERSION 2.8.3)
project(uart_2nd)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    serial
    std_msgs
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(uart_2nd src/uart_2nd.cpp)
target_link_libraries(uart_2nd ${catkin_LIBRARIES})
