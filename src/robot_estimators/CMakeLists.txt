cmake_minimum_required(VERSION 3.8)
project(robot_estimators)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 1. ดึงคลังคณิตศาสตร์เมทริกซ์และคอร์สถาปัตยกรรมตัวกรอง EKF เข้ามาในระบบ
find_package(Eigen3 REQUIRED)
find_package(robot_math_core REQUIRED)

set(PACKAGE_DEPENDENCIES
  ros2_control
  generate_parameter_library
  control_msgs
  control_toolbox
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  controller_interface
  geometry_msgs
  realtime_tools
  rclcpp
  pluginlib
)

foreach(Dependency IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# 2. ปั่นแผงพารามิเตอร์อัตโนมัติจากไฟล์ YAML ทั้งสองตัว
generate_parameter_library(pure_encoder_yaml
  config/pure_encoder.yaml 
)

generate_parameter_library(ekf_estimator_yaml
  config/ekf_estimator.yaml 
)

# 3. สร้างแฝดห้องสมุดแชร์แรม (Shared Libraries)
add_library(pure_encoder SHARED
  src/pure_encoder.cpp
)

add_library(ekf_estimator SHARED
  src/ekf_estimator.cpp
)

# 4. ลิงก์และเปิดช่องทางวิ่งให้คอมไพเลอร์มองเห็น Header Files
foreach(Target IN ITEMS pure_encoder ekf_estimator)
  target_include_directories(${Target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    ${Eigen3_INCLUDE_DIRS}
    ${robot_math_core_INCLUDE_DIRS} # ดึงพาร์ทโฟลเดอร์ include ของตัว EKF Core มาใช้งานโดยตรงผ่านตัวแปรระบบ
  )

  # ผูก dependencies พื้นฐานของ ROS 2 เข้ากับตัวแปรเป้าหมาย
  ament_target_dependencies(${Target} ${PACKAGE_DEPENDENCIES} robot_math_core)
endforeach()

# 5. ลิงก์โครงสร้างพารามิเตอร์ของไลบรารีและเชื่อมสัญลักษณ์เชื่อมโยง (Link Libraries)
target_link_libraries(pure_encoder pure_encoder_yaml)
target_link_libraries(ekf_estimator ekf_estimator_yaml robot_math_core::robot_math_core)

# Ground truth publisher node
add_executable(ground_truth_publisher
  src/ground_truth_publisher.cpp
)

ament_target_dependencies(ground_truth_publisher
  rclcpp
  nav_msgs
  tf2_msgs
)

# 6. ส่งออกปลั๊กอินเพื่อให้ระบบ ros2_control สแกนหาตัวแปรเจอตอนสแกน Hardware Interface
pluginlib_export_plugin_description_file(controller_interface plugin.xml)

# 7. สั่งจัดวางและส่งออกไฟล์ลงระบบหลังจากคอมไพล์เสร็จ
install(
  TARGETS pure_encoder ekf_estimator
  EXPORT export_robot_estimators
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ground_truth_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_libraries(pure_encoder ekf_estimator)
ament_export_dependencies(${PACKAGE_DEPENDENCIES} robot_math_core)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()