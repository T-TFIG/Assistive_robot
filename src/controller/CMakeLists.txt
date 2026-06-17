cmake_minimum_required(VERSION 3.8)
project(controller)

find_package(ament_cmake REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# คลีนเอาตัวแปรระบบพิกัดที่ไม่ได้ใช้ออกไปแล้ว
set(PACKAGE_DEPENDENCIES
  ros2_control
  generate_parameter_library
  control_msgs
  control_toolbox
  controller_interface
  geometry_msgs
  realtime_tools
  rclcpp
  pluginlib
)

foreach(Dependency IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(pid_controller_parameters
  config/velocity_pid_controller.yaml
)

add_library(velocity_pid_controller SHARED
  src/velocity_pid_controller.cpp
)

target_include_directories(velocity_pid_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> 
)

ament_target_dependencies(velocity_pid_controller
  ${PACKAGE_DEPENDENCIES}
)

target_link_libraries(velocity_pid_controller
  pid_controller_parameters
)

# ส่งออกปลั๊กอินให้ ros2_control รู้จัก
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