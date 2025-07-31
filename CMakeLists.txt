cmake_minimum_required(VERSION 3.8)
project(adi_imu)

#Default to C++17 - ROS2 Humble
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# Find dependencies
# ==============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_library(LIBIIO_LIBRARIES iio)
find_path(LIBIIO_INCLUDE_DIRS iio.h)

# ==============================================================================
# Create executable
# ==============================================================================

set(NODE_NAME ${PROJECT_NAME}_node)

file(GLOB FILES_H ${CMAKE_SOURCE_DIR}/include/adi_imu/*.h
                  ${CMAKE_SOURCE_DIR}/include/adi_imu/utils/*.h)

file(GLOB FILES_CPP ${CMAKE_SOURCE_DIR}/src/*.cpp
                    ${CMAKE_SOURCE_DIR}/src/utils/*.cpp)

add_executable(${NODE_NAME} ${FILES_H} ${FILES_CPP})

target_include_directories(${NODE_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${NODE_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${NODE_NAME}
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  ament_cmake
)

target_link_libraries(${NODE_NAME} iio)

install(TARGETS ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# Generate interfaces
# ==============================================================================
set(msg_files
  "msg/AccelGyroTempData.msg"
  "msg/ImuDiagDataADIS1646X.msg"
  "msg/ImuDiagDataADIS1647X.msg"
  "msg/ImuDiagDataADIS1650X.msg"
  "msg/ImuDiagDataADIS1654X.msg"
  "msg/ImuDiagDataADIS1655X.msg"
  "msg/ImuDiagDataADIS1657X.msg"
  "msg/ImuFullMeasuredData.msg"
  "msg/ImuIdentificationData.msg"
  "msg/VelAngTempData.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
)

rosidl_get_typesupport_target(${NODE_NAME}
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)
target_link_libraries(${NODE_NAME} "${adi_imu_node}")

# ==============================================================================
# Install targets
# ==============================================================================

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================================================
# Testing
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Only build integration tests when explicitly enabled
  if(DEFINED ENV{IMU_ROS2_ENABLE_HW_TESTS})
    set(BUILD_INTEGRATION_TESTS ON)
  else()
    set(BUILD_INTEGRATION_TESTS OFF)
  endif()

  if(BUILD_INTEGRATION_TESTS)
    message(STATUS "Building integration tests")
    add_subdirectory(test)
  else()
    message(STATUS "Integration tests disabled. Set IMU_ROS2_ENABLE_HW_TESTS=1 environment variable to enable")
  endif()
endif()

# ==============================================================================
# Export information
# ==============================================================================

ament_package()
