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
find_package(geometry_msgs REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_library(LIBIIO_LIBRARIES iio)
find_path(LIBIIO_INCLUDE_DIRS iio.h)

file(GLOB FILES_H ${CMAKE_SOURCE_DIR}/include/*.h
                    ${CMAKE_SOURCE_DIR}/include/imu_ros2/*.h
                    ${CMAKE_SOURCE_DIR}/include/imu_ros2/imudiag/*.h
                    )

file(GLOB FILES_CPP ${CMAKE_SOURCE_DIR}/src/*.cpp
    ${CMAKE_SOURCE_DIR}/src/imudiag/*.cpp
                  )

add_executable(imu_ros2_node ${FILES_H} ${FILES_CPP})

target_include_directories(imu_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_ros2_node PUBLIC cxx_std_17)  # Require C++17

ament_target_dependencies(imu_ros2_node rclcpp geometry_msgs std_msgs sensor_msgs ament_cmake )

target_link_libraries(imu_ros2_node iio)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS imu_ros2_node
  DESTINATION lib/${PROJECT_NAME})

rosidl_generate_interfaces(imu_ros2
  "msg/ImuIdentificationData.msg"
  "msg/VelAngTempData.msg"
  "msg/ImuFullMeasuredData.msg"
  "msg/AccelGyroTempData.msg"
  "msg/ImuDiagDataadis1646x.msg"
  "msg/ImuDiagDataadis1647x.msg"
  "msg/ImuDiagDataadis1650x.msg"
  "msg/ImuDiagDataadis1654x.msg"
  "msg/ImuDiagDataadis1655x.msg"
  "msg/ImuDiagDataadis1657x.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

rosidl_get_typesupport_target(imu_ros2_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)
target_link_libraries(imu_ros2_node "${imu_ros2_node}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

#install(FILES config/${DEVICE_PATH}/imu_config.yaml DESTINATION config)

ament_package()
