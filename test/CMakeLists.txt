cmake_minimum_required(VERSION 3.14)
project(adi_imu_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

find_library(LIBIIO_LIBRARIES iio)
find_path(LIBIIO_INCLUDE_DIRS iio.h)

# Include all source files from the main project (including utils subdirectory)
file(GLOB FILES_H ${CMAKE_SOURCE_DIR}/include/adi_imu/*.h
                  ${CMAKE_SOURCE_DIR}/include/adi_imu/utils/*.h)

file(GLOB FILES_CPP ${CMAKE_SOURCE_DIR}/src/*.cpp
                    ${CMAKE_SOURCE_DIR}/src/utils/*.cpp)

# Remove the main node file from the list since it has its own main() function
list(REMOVE_ITEM FILES_CPP ${CMAKE_SOURCE_DIR}/src/imu_ros2_node.cpp)

include_directories(${CMAKE_SOURCE_DIR}/include)

ament_add_gtest(adi_imu_test_node utest_launch.test src/utest.cpp
                                      src/imu_subscriber_test.cpp
                                      src/imu_full_measured_data_subscriber_test.cpp
                                      src/imu_identification_subscriber_test.cpp
                                      src/imu_diag_adis1646X_subscriber_test.cpp
                                      src/imu_diag_adis1647X_subscriber_test.cpp
                                      src/imu_diag_adis1650X_subscriber_test.cpp
                                      src/imu_diag_adis1654X_subscriber_test.cpp
                                      src/imu_diag_adis1655X_subscriber_test.cpp
                                      src/imu_diag_adis1657X_subscriber_test.cpp
                                      src/velangtemp_subscriber_test.cpp
                                      src/accelgyrotemp_subscriber_test.cpp
                                      ${FILES_H} ${FILES_CPP})

target_include_directories(adi_imu_test_node PUBLIC
  ${LIBIIO_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(adi_imu_test_node PUBLIC cxx_std_17)  # Require C++17

target_link_libraries(adi_imu_test_node
  "${adi_imu_node}"
  ${LIBIIO_LIBRARIES}
  ${rclcpp_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${ament_cmake_TARGETS}
  ${ament_cmake_gtest_TARGETS}
  ${ament_lint_auto_TARGETS}
)

install(
  TARGETS adi_imu_test_node
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_get_typesupport_target(adi_imu_test_node adi_imu "rosidl_typesupport_cpp")
