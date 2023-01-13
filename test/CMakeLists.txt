cmake_minimum_required(VERSION 3.8)
project(imu_ros2_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)



file(GLOB FILES_H ${CMAKE_SOURCE_DIR}/include/*.h
                    ${CMAKE_SOURCE_DIR}/include/iio_ros2_package/*.h)

file(GLOB FILES_CPP ${CMAKE_SOURCE_DIR}/src/*.cpp)

#add_executable(imu_ros2_node ${FILES_H} ${FILES_CPP})

#if(CATKIN_ENABLE_TESTING)
  #find_package(rostest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(imu_ros2_test_node utest_launch.test src/utest.cpp src/subscriberteststring.cpp)
  #target_link_libraries(utest_node ${catkin_LIBRARIES})
#endif()

target_include_directories(imu_ros2_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_ros2_test_node PUBLIC cxx_std_17)  # Require C++17

ament_target_dependencies(imu_ros2_test_node rclcpp std_msgs sensor_msgs ament_cmake ament_cmake_gtest ament_lint_auto)

install(TARGETS imu_ros2_test_node
  DESTINATION lib/${PROJECT_NAME})



ament_package()
