cmake_minimum_required(VERSION 3.5)
project(roar_robot_localization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
        include
)

set(dependencies
        rclcpp
        geometry_msgs
        visualization_msgs
        std_msgs
        message_filters
        sensor_msgs
)

add_executable(odom_publisher src/odom_publisher.cpp)

ament_target_dependencies(odom_publisher ${dependencies})

target_include_directories(odom_publisher PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS odom_publisher
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
ament_package()
