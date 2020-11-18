################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(mastering_ros_robot_description_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

################################################################################
# Build
################################################################################

include_directories(
  include
)

set(DEPENDENCIES
  "rclcpp"
  "gazebo_msgs"
  "gazebo_ros"
)

add_library(joint_control SHARED "src/jointControl.cpp")
ament_target_dependencies(joint_control ${DEPENDENCIES})
ament_export_libraries(joint_control)

################################################################################
# Install
################################################################################

install(TARGETS joint_control
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib)

install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
