cmake_minimum_required(VERSION 3.8)
project(arm_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(control_msgs REQUIRED)  # Added
find_package(trajectory_msgs REQUIRED)  # Added
find_package(moveit_ros_planning_interface REQUIRED)  # Optional for MoveIt!

# include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
  ${xacro_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add any executables or libraries
# add_executable(${PROJECT_NAME}_node src/your_node.cpp)
# target_link_libraries(${PROJECT_NAME}_node
#   ${rclcpp_LIBRARIES}
#   ${std_msgs_LIBRARIES}
#   ${sensor_msgs_LIBRARIES}
#   ${geometry_msgs_LIBRARIES}
#   ${urdf_LIBRARIES}
#   ${xacro_LIBRARIES}
#   ${gazebo_ros_LIBRARIES}
# )
# install(TARGETS ${PROJECT_NAME}_node
#   DESTINATION lib/${PROJECT_NAME})

# Directories to install
set(DIRS config launch meshes urdf)

# Install each directory
foreach(dir ${DIRS})
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

ament_package()
