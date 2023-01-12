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



add_executable(imu-ros2-node src/imu_ros2_node.cpp)

target_include_directories(imu-ros2-node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu-ros2-node PUBLIC cxx_std_17)  # Require C++17

ament_target_dependencies(imu-ros2-node rclcpp std_msgs sensor_msgs ament_cmake)

install(TARGETS imu-ros2-node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
