cmake_minimum_required(VERSION 3.8)
project(controller)

# 1. Standard ROS 2 build tool
find_package(ament_cmake REQUIRED)

# 2. Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 3. List all dependencies (Removed ros2_control_cmake)
set(PACKAGE_DEPENDENCIES
  ros2_control
  generate_parameter_library
  control_msgs
  control_toolbox
  nav_msgs
  tf2
  tf2_geometry_msgs
  controller_interface
  geometry_msgs
  realtime_tools
  rclcpp
  pluginlib
)

# 4. Find the packages properly
foreach(Dependency IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# 5. Generate Parameters
generate_parameter_library(pid_controller_parameters
  src/controllers.yaml
)

# 6. Create Library
add_library(velocity_pid_controller SHARED
  src/velocity_pid_controller.cpp
)

# 7. Include Directories
target_include_directories(velocity_pid_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> 
)

# 8. Link dependencies
ament_target_dependencies(velocity_pid_controller
  ${PACKAGE_DEPENDENCIES}
)

target_link_libraries(velocity_pid_controller
  pid_controller_parameters
)

# 9. Export and Install
pluginlib_export_plugin_description_file(controller_interface plugin.xml)

install(
  TARGETS velocity_pid_controller pid_controller_parameters
  DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_libraries(velocity_pid_controller)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()