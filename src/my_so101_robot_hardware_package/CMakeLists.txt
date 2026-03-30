cmake_minimum_required(VERSION 3.8)
project(so101_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# =========================
# Shared library
# =========================
add_library(${PROJECT_NAME} SHARED
  src/so101.cpp
  src/motors_bus.cpp
  src/FeetechMotorsBus.cpp
  src/scservo_sdk.cpp
  src/port_handler.cpp
  src/tables.cpp
  src/packet_handler.cpp
  src/protocol_packet_handler.cpp
  src/group_sync_read.cpp
  src/group_sync_write.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
)

target_link_libraries(${PROJECT_NAME}
  /usr/local/lib/libserial.so
)

# =========================
# Publisher executable
# =========================
add_executable(my_so101_ros2_pub_executable
  src/so101_ros2_pub.cpp
)

target_link_libraries(my_so101_ros2_pub_executable
  ${PROJECT_NAME}
)

ament_target_dependencies(my_so101_ros2_pub_executable
  rclcpp
  std_msgs
  sensor_msgs
)

# =========================
# Subscriber executable
# =========================
add_executable(my_so101_ros2_sub_executable
  src/so101_ros2_sub.cpp
)

target_link_libraries(my_so101_ros2_sub_executable
  ${PROJECT_NAME}
)

ament_target_dependencies(my_so101_ros2_sub_executable
  rclcpp
  std_msgs
  sensor_msgs
)

# =========================
# Calibration executable
# =========================
add_executable(my_so101_ros2_calib_executable
  src/so101_ros2_calib.cpp
)

target_link_libraries(my_so101_ros2_calib_executable
  ${PROJECT_NAME}
)

ament_target_dependencies(my_so101_ros2_calib_executable
  rclcpp
  std_msgs
  sensor_msgs
)

# =========================
# Install
# =========================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    my_so101_ros2_pub_executable
    my_so101_ros2_sub_executable
    my_so101_ros2_calib_executable
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Export
# =========================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
)

ament_package()