/*******************************************************************************
*   @file   imu_1657x_diag_subscriber_test.cpp
*   @brief  Test imu 1657x diag data
*   @author Vasile Holonec (Vasile.Holonec@analog.com)
********************************************************************************
* Copyright 2023(c) Analog Devices, Inc.

* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <gtest/gtest.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "imu_ros2/msg/imu1657x_diag_data.hpp"

class Imu1657xDiagSubscriberTest : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  static void TearDownTestCase() { rclcpp::shutdown(); }
};

TEST(Imu1657xDiagSubscriberTest, test_imu_1657x_diag_data_publisher)
{
  auto node = rclcpp::Node::make_shared("imu1657xdiagdata");

  std::string topic = "imu1657xdiagdata";
  bool callbackExecuted = false;

  auto callback = [&callbackExecuted](imu_ros2::msg::Imu1657xDiagData msg) -> void {
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp_imu_16505_diag_data"),
      "diag_sensor_initialization_failure= %d \n"
      "diag data: diag_data_path_overrun = %d \n"
      "diag_flash_memory_update_error = %d \n"
      "diag_spi_communication_error = %d \n"
      "diag_standby_mode = %d \n"
      "diag_sensor_self_test_error = %d \n"
      "diag_flash_memory_test_error = %d \n"
      "diag_clock_error = %d \n"
      "diag_x_axis_gyroscope_failure = %d \n"
      "diag_y_axis_gyroscope_failure = %d \n"
      "diag_z_axis_gyroscope_failure = %d \n"
      "diag_x_axis_accelerometer_failure = %d \n"
      "diag_y_axis_accelerometer_failure = %d \n"
      "diag_z_axis_accelerometer_failure = %d \n"
      "diag_aduc_mcu_fault = %d \n"
      "diag_checksum_error_flag = %d \n"
      "diag_flash_memory_write_count_exceeded_error = %d \n"
      "flash_counter = %d \n"
      "lost_samples_count = %d \n",
      msg.diag_sensor_initialization_failure, msg.diag_data_path_overrun,
      msg.diag_flash_memory_update_error, msg.diag_spi_communication_error, msg.diag_standby_mode,
      msg.diag_sensor_self_test_error, msg.diag_flash_memory_test_error, msg.diag_clock_error,
      msg.diag_x_axis_gyroscope_failure, msg.diag_y_axis_gyroscope_failure,
      msg.diag_z_axis_gyroscope_failure, msg.diag_x_axis_accelerometer_failure,
      msg.diag_y_axis_accelerometer_failure, msg.diag_z_axis_accelerometer_failure,
      msg.diag_aduc_mcu_fault, msg.diag_checksum_error_flag,
      msg.diag_flash_memory_write_count_exceeded_error, msg.flash_counter, msg.lost_samples_count);

    ASSERT_TRUE(msg.diag_sensor_initialization_failure == false);
    ASSERT_TRUE(msg.diag_data_path_overrun == false);
    ASSERT_TRUE(msg.diag_flash_memory_update_error == false);
    ASSERT_TRUE(msg.diag_spi_communication_error == false);
    ASSERT_TRUE(msg.diag_standby_mode == false);
    ASSERT_TRUE(msg.diag_sensor_self_test_error == false);
    ASSERT_TRUE(msg.diag_flash_memory_test_error == false);
    ASSERT_TRUE(msg.diag_clock_error == false);
    ASSERT_TRUE(msg.diag_x_axis_gyroscope_failure == false);
    ASSERT_TRUE(msg.diag_y_axis_gyroscope_failure == false);
    ASSERT_TRUE(msg.diag_z_axis_gyroscope_failure == false);
    ASSERT_TRUE(msg.diag_x_axis_accelerometer_failure == false);
    ASSERT_TRUE(msg.diag_y_axis_accelerometer_failure == false);
    ASSERT_TRUE(msg.diag_z_axis_accelerometer_failure == false);
    ASSERT_TRUE(msg.diag_aduc_mcu_fault == false);
    ASSERT_TRUE(msg.diag_checksum_error_flag == false);
    ASSERT_TRUE(msg.diag_flash_memory_write_count_exceeded_error == false);
    ASSERT_TRUE(msg.flash_counter >= 0);
    ASSERT_TRUE(msg.lost_samples_count >= 0);

    callbackExecuted = true;
  };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto subscriber = node->create_subscription<imu_ros2::msg::Imu1657xDiagData>(topic, 10, callback);

  std::chrono::seconds sec(1);

  while (!callbackExecuted) executor.spin_once(sec);
}
