cmake_minimum_required(VERSION 3.8)
project(imu_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_library(LIBIIO_LIBRARIES iio)
find_path(LIBIIO_INCLUDE_DIRS iio.h)


file(GLOB FILES_H ${CMAKE_SOURCE_DIR}/include/*.h
                    ${CMAKE_SOURCE_DIR}/include/imu_ros2/*.h)

file(GLOB FILES_CPP ${CMAKE_SOURCE_DIR}/src/*.cpp)



add_executable(imu_ros2_node ${FILES_H} ${FILES_CPP})


target_include_directories(imu_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_ros2_node PUBLIC cxx_std_17)  # Require C++17

ament_target_dependencies(imu_ros2_node rclcpp std_msgs sensor_msgs ament_cmake)

target_link_libraries(imu_ros2_node iio)

install(TARGETS imu_ros2_node
  DESTINATION lib/${PROJECT_NAME})


rosidl_generate_interfaces(imu_ros2
  "msg/StaticData.msg"
  "msg/GyroscopeData.msg"
  DEPENDENCIES builtin_interfaces
)

rosidl_get_typesupport_target(imu_ros2_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)
target_link_libraries(imu_ros2_node "${imu_ros2_node}")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  #ament_add_gtest(iio_ros2_test_node test/utest_launch.test test/src/utest.cpp)

  add_subdirectory(test)
endif()








ament_package()
