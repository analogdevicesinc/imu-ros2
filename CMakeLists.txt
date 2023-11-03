cmake_minimum_required(VERSION 3.8)
project(imu_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

string (COMPARE EQUAL "adis16465-1" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16465_1)
string (COMPARE EQUAL "adis16465-2" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16465_2)
string (COMPARE EQUAL "adis16465-3" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16465_3)
string (COMPARE EQUAL "adis16467-1" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16467_1)
string (COMPARE EQUAL "adis16467-2" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16467_2)
string (COMPARE EQUAL "adis16467-3" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16467_3)
string (COMPARE EQUAL "adis16470"   $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16470)
string (COMPARE EQUAL "adis16475-1" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16475_1)
string (COMPARE EQUAL "adis16475-2" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16475_2)
string (COMPARE EQUAL "adis16475-3" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16475_3)

string (COMPARE EQUAL "adis16477-1" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16477_1)
string (COMPARE EQUAL "adis16477-2" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16477_2)
string (COMPARE EQUAL "adis16477-3" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16477_3)

string (COMPARE EQUAL "adis16500"   $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16500)
string (COMPARE EQUAL "adis16505-1" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16505_1)
string (COMPARE EQUAL "adis16505-2" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16505_2)
string (COMPARE EQUAL "adis16505-3" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16505_3)
string (COMPARE EQUAL "adis16507-1" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16507_1)
string (COMPARE EQUAL "adis16507-2" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16507_2)
string (COMPARE EQUAL "adis16507-3" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16507_3)

string (COMPARE EQUAL "adis16575-2" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16575_2)
string (COMPARE EQUAL "adis16575-3" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16575_3)
string (COMPARE EQUAL "adis16576-2" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16576_2)
string (COMPARE EQUAL "adis16576-3" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16576_3)
string (COMPARE EQUAL "adis16577-2" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16577_2)
string (COMPARE EQUAL "adis16577-3" $ENV{DEVICE_ID} DEVICE_FOUND_ADIS16577_3)

if (${DEVICE_FOUND_ADIS16500}  OR
   ${DEVICE_FOUND_ADIS16505_1} OR
   ${DEVICE_FOUND_ADIS16505_2} OR
   ${DEVICE_FOUND_ADIS16505_3} OR
   ${DEVICE_FOUND_ADIS16507_1} OR
   ${DEVICE_FOUND_ADIS16507_2} OR
   ${DEVICE_FOUND_ADIS16507_3})
  message(AUTHOR_WARNING "DEVICE_ID=$ENV{DEVICE_ID} which is a valid value.")
  set(DEVICE_PATH adis1650x)
  add_compile_definitions(ADIS1650X)

elseif (${DEVICE_FOUND_ADIS16465_1} OR
        ${DEVICE_FOUND_ADIS16465_2} OR
        ${DEVICE_FOUND_ADIS16465_3} OR
        ${DEVICE_FOUND_ADIS16467_1} OR
        ${DEVICE_FOUND_ADIS16467_2} OR
        ${DEVICE_FOUND_ADIS16467_3} OR
        ${DEVICE_FOUND_ADIS16470}   OR
        ${DEVICE_FOUND_ADIS16475_1} OR
        ${DEVICE_FOUND_ADIS16475_2} OR
        ${DEVICE_FOUND_ADIS16475_3})
  message(AUTHOR_WARNING "DEVICE_ID=$ENV{DEVICE_ID} which is a valid value.")
  set(DEVICE_PATH adis1646x)
  add_compile_definitions(ADIS1646X)

elseif (${DEVICE_FOUND_ADIS16565_2} OR
        ${DEVICE_FOUND_ADIS16565_3} OR
        ${DEVICE_FOUND_ADIS16576_2} OR
        ${DEVICE_FOUND_ADIS16576_3} OR
        ${DEVICE_FOUND_ADIS16577_2} OR
        ${DEVICE_FOUND_ADIS16577_3})
  message(AUTHOR_WARNING "DEVICE_ID=$ENV{DEVICE_ID} which is a valid value.")
  set(DEVICE_PATH adis1657x)
  add_compile_definitions(ADIS1657X)

elseif (${DEVICE_FOUND_ADIS16477_1} OR
        ${DEVICE_FOUND_ADIS16477_2} OR
        ${DEVICE_FOUND_ADIS16477_3})
  message(AUTHOR_WARNING "DEVICE_ID=$ENV{DEVICE_ID} which is a valid value.")
  set(DEVICE_PATH adis1647x)
  add_compile_definitions(ADIS1647X)

else()
  message (FATAL_ERROR "
  DEVICE_ID=$ENV{DEVICE_ID} is not a valid value.
  Please use one of the following supported devices:
  adis16465-1, adis16465-2, adis16465-3,
  adis16467-1, adis16467-2, adis16467-3,
  adis16470,
  adis16475-1, adis16475-2, adis16475-3,
  adis16477-1, adis16477-2, adis16477-3,
  adis16500,
  adis16505-1, adis16505-2, adis16505-3,
  adis16507-1, adis16507-2, adis16507-3,
  adis16575-2, adis16575-3,
  adis16576-2, adis16576-3,
  adis16577-2, adis16577-3.")
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
                    ${CMAKE_SOURCE_DIR}/include/imu_ros2/${DEVICE_PATH}/*.h)

file(GLOB FILES_CPP ${CMAKE_SOURCE_DIR}/src/*.cpp
                      ${CMAKE_SOURCE_DIR}/src/${DEVICE_PATH}/*.cpp)

add_executable(imu_ros2_node ${FILES_H} ${FILES_CPP})

target_include_directories(imu_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_ros2_node PUBLIC cxx_std_17)  # Require C++17

ament_target_dependencies(imu_ros2_node rclcpp geometry_msgs std_msgs sensor_msgs ament_cmake )

target_link_libraries(imu_ros2_node iio)

install(TARGETS imu_ros2_node
  DESTINATION lib/${PROJECT_NAME})

rosidl_generate_interfaces(imu_ros2
  "msg/ImuIdentificationData.msg"
  "msg/VelAngTempData.msg"
  "msg/ImuFullMeasuredData.msg"
  "msg/AccelGyroTempData.msg"
  "msg/${DEVICE_PATH}/ImuDiagData.msg"
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

install(FILES config/${DEVICE_PATH}/imu_config.yaml DESTINATION config)

ament_package()
