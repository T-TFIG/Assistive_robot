cmake_minimum_required(VERSION 3.8)
project(controller)

# 1. Specialized ROS 2 Control build tools
find_package(ros2_control_cmake REQUIRED)

# 2. Compiler flags for safety
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 3. List all dependencies in one place
set(PACKAGE_DEPENDENCIES
  generate_parameter_library
  control_msgs
  control_toolbox
  nav_msgs
  tf2
  tf2_geometry_msgs
  controller_interface
  geometry_msgs
  realtime_tools
  ament_cmake
  rclcpp
  pluginlib
)

# 4. Find the packages
foreach(Dependency IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# 5. Generate the Parameters Header FIRST
# This creates the "pid_controller_parameters" target
generate_parameter_library(pid_controller_parameters
  src/controllers.yaml
)

# 6. Create your Controller Library
add_library(velocity_pid_controller SHARED
  src/velocity_pid_controller.cpp
)

# 7. EXTREMELY IMPORTANT: Tell the library where to find the headers
# $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> is where the generated .hpp lives
target_include_directories(velocity_pid_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> 
  include
)

# 8. Link dependencies and the generated parameter target
ament_target_dependencies(velocity_pid_controller
  ${PACKAGE_DEPENDENCIES}
)

target_link_libraries(velocity_pid_controller
  pid_controller_parameters
)

# 9. Export the plugin (so ROS 2 can "discover" your controller)
pluginlib_export_plugin_description_file(controller_interface plugin.xml)
install(
  TARGETS velocity_pid_controller pid_controller_parameters
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(velocity_pid_controller)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()