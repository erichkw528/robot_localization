cmake_minimum_required(VERSION 3.5)
project(roar_robot_localization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(pose_converter_node src/pose_converter_node.cpp)

ament_target_dependencies(pose_converter_node rclcpp geometry_msgs)
install(TARGETS pose_converter_node
        DESTINATION lib/${PROJECT_NAME})
        
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
