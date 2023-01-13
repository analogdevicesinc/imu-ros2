cmake_minimum_required(VERSION 3.8)
project(imu-ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

file(GLOB FILES_H ${CMAKE_SOURCE_DIR}/include/*.h
                    ${CMAKE_SOURCE_DIR}/include/imu_ros2/*.h)

file(GLOB FILES_CPP ${CMAKE_SOURCE_DIR}/src/*.cpp)

add_executable(imu-ros2-node ${FILES_H} ${FILES_CPP})

target_include_directories(imu-ros2-node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu-ros2-node PUBLIC cxx_std_17)  # Require C++17

ament_target_dependencies(imu-ros2-node rclcpp std_msgs sensor_msgs ament_cmake)

install(TARGETS imu-ros2-node
  DESTINATION lib/${PROJECT_NAME})

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
