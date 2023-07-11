/*******************************************************************************
*   @file   imu_diag_subscriber_test.cpp
*   @brief  Test imu diag data
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

#include "imu_ros2/msg/imu16505_diag_data.hpp"

/**
 * \brief Class for testing the imu 16505 diag data
 *
 * This class instantiate a subscriber node and listen data
 * from topic and compare with default values.
 */
class Imu16505DiagSubscriberTest : public ::testing::Test
{
public:
  /**
   * \brief Set up the test case
   *
   * This class initialize variable before the tests
   */
  static void SetUpTestCase() {}

  /**
   * \brief Tear down the test case
   *
   * This class dealocate the data after tests
   */
  static void TearDownTestCase() { rclcpp::shutdown(); }
};

/**
 * \brief Imu16505DiagSubscriberTest
 *
 * This test instantiate a subscriber node and listen data
 * from topic and compare with default values.
 */
TEST(Imu16505DiagSubscriberTest, test_imu_16505_diag_data_publisher)
{
  auto node = rclcpp::Node::make_shared("imu16505diagdata");

  std::string topic = "imu16505diagdata";
  bool callbackExecuted = false;

  auto callback = [&callbackExecuted](imu_ros2::msg::Imu16505DiagData msg) -> void {
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp_imu_16505_diag_data"),
      " diag data: diag_data_path_overrun = %d \n"
      "diag_flash_memory_update_error = %d \n"
      "diag_spi_communication_error = %d \n"
      "diag_standby_mode = %d \n"
      "diag_sensor_self_test_error = %d \n"
      "diag_flash_memory_test_error = %d \n"
      "diag_clock_error = %d \n"
      "diag_acceleration_self_test_error = %d \n"
      "diag_gyroscope1_self_test_error = %d \n"
      "diag_gyroscope2_self_test_error = %d \n"
      "diag_checksum_error_flag = %d \n"
      "diag_flash_memory_write_count_exceeded_error = %d \n"
      "lost_samples_count = %d \n"
      "flash_counter = %d \n",
      msg.diag_data_path_overrun, msg.diag_flash_memory_update_error,
      msg.diag_spi_communication_error, msg.diag_standby_mode, msg.diag_sensor_self_test_error,
      msg.diag_flash_memory_test_error, msg.diag_clock_error, msg.diag_acceleration_self_test_error,
      msg.diag_gyroscope1_self_test_error, msg.diag_gyroscope2_self_test_error,
      msg.diag_checksum_error_flag, msg.diag_flash_memory_write_count_exceeded_error,
      msg.lost_samples_count, msg.flash_counter);

    ASSERT_TRUE(msg.diag_data_path_overrun == false);
    ASSERT_TRUE(msg.diag_flash_memory_update_error == false);
    ASSERT_TRUE(msg.diag_spi_communication_error == false);
    ASSERT_TRUE(msg.diag_standby_mode == false);
    ASSERT_TRUE(msg.diag_sensor_self_test_error == false);
    ASSERT_TRUE(msg.diag_flash_memory_test_error == false);
    ASSERT_TRUE(msg.diag_clock_error == false);
    ASSERT_TRUE(msg.diag_acceleration_self_test_error == false);
    ASSERT_TRUE(msg.diag_gyroscope1_self_test_error == false);
    ASSERT_TRUE(msg.diag_gyroscope2_self_test_error == false);
    ASSERT_TRUE(msg.diag_checksum_error_flag == false);
    ASSERT_TRUE(msg.diag_flash_memory_write_count_exceeded_error == false);

    ASSERT_TRUE(msg.lost_samples_count >= 0);
    ASSERT_TRUE(msg.flash_counter >= 0);
    callbackExecuted = true;
  };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto subscriber = node->create_subscription<imu_ros2::msg::Imu16505DiagData>(topic, 10, callback);

  std::chrono::seconds sec(1);

  while (!callbackExecuted) executor.spin_once(sec);
}
