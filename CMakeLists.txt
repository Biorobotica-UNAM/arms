cmake_minimum_required(VERSION 3.8)
project(arms)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(arms_interfaces REQUIRED)

add_executable(arms_node src/arms_node.cpp)

target_include_directories(arms_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(arms_node
  rclcpp
  std_msgs
  dynamixel_sdk
  sensor_msgs
  trajectory_msgs
  tf2_ros
  arms_interfaces
)

install(TARGETS arms_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
