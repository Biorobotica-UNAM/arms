cmake_minimum_required(VERSION 3.8)
project(arms)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# For generating custom interfaces
find_package(rosidl_default_generators REQUIRED)

# ============================================
# Generate custom service interfaces
# ============================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ArmMovementPlan.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Get the typesupport target for C++
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# ============================================
# Executable 1: Arm Movement Planner Service
# ============================================
add_executable(arm_movement_planner_node
  src/plan_arm_movement_service.cpp
)

ament_target_dependencies(arm_movement_planner_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  tf2
  tf2_geometry_msgs
)

# Link the generated service interface
target_link_libraries(arm_movement_planner_node "${cpp_typesupport_target}")

# CRITICAL: Ensure service is generated before building executable
rosidl_get_typesupport_target(cpp_typesupport_target 
  "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(arm_movement_planner_node
  "${cpp_typesupport_target}")

# ============================================
# Executable 2: Arm Node
# ============================================

add_executable(arm_node
  src/arm_node.cpp
)

ament_target_dependencies(arm_node
  rclcpp
  std_msgs
  moveit_msgs
  dynamixel_sdk
  sensor_msgs
  trajectory_msgs
  tf2_ros
)

# ============================================
# Install targets
# ============================================
install(TARGETS
  arm_movement_planner_node
  arm_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()