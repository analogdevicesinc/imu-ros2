cmake_minimum_required(VERSION 3.14)
project(adi_imu)

#Default to C++17
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

find_package(PkgConfig REQUIRED)
pkg_search_module(IIO REQUIRED IMPORTED_TARGET libiio iio)

set(LIBIIO_API_VERSION "AUTO" CACHE STRING "libiio API to target: AUTO, v0 or v1")
set_property(CACHE LIBIIO_API_VERSION PROPERTY STRINGS AUTO v0 v1)

if(IIO_VERSION VERSION_LESS 1.0)
  set(LIBIIO_DETECTED_API v0)
else()
  set(LIBIIO_DETECTED_API v1)
endif()

if(LIBIIO_API_VERSION STREQUAL "AUTO")
  set(LIBIIO_RESOLVED_API ${LIBIIO_DETECTED_API})
elseif(NOT LIBIIO_API_VERSION STREQUAL LIBIIO_DETECTED_API)
  message(FATAL_ERROR "LIBIIO_API_VERSION=${LIBIIO_API_VERSION} but found libiio ${IIO_VERSION} ")
else()
  set(LIBIIO_RESOLVED_API ${LIBIIO_API_VERSION})
endif()

message(STATUS "Using libiio ${IIO_VERSION} (${LIBIIO_RESOLVED_API} API)")

# ==============================================================================
# Create executable
# ==============================================================================

set(NODE_NAME ${PROJECT_NAME}_node)

file(GLOB FILES_H ${CMAKE_SOURCE_DIR}/include/adi_imu/*.h
                  ${CMAKE_SOURCE_DIR}/include/adi_imu/utils/*.h)

file(GLOB FILES_CPP ${CMAKE_SOURCE_DIR}/src/*.cpp
                    ${CMAKE_SOURCE_DIR}/src/utils/*.cpp)

add_executable(${NODE_NAME} ${FILES_H} ${FILES_CPP})

if(LIBIIO_RESOLVED_API STREQUAL "v1")
  target_compile_definitions(${NODE_NAME} PRIVATE LIBIIO_V1)
endif()

target_include_directories(${NODE_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${NODE_NAME} PUBLIC cxx_std_17)

target_link_libraries(${NODE_NAME}
  PkgConfig::IIO
  ${rclcpp_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

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
