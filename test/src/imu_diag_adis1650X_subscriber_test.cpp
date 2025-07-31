/*******************************************************************************
*   @file   imu__diag_subscriber_test.cpp
*   @brief  Test imu diag data
*   @author Vasile Holonec (Vasile.Holonec@analog.com)
*******************************************************************************/
// Copyright 2023 Analog Devices, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "adi_imu/adis_device_factory.h"
#include "adi_imu/adis_register_map.h"
#include "adi_imu/iio_wrapper.h"
#include "adi_imu/msg/imu_diag_data_adis1650_x.hpp"

/**
 * @brief Class for testing ImuDiagData.
 *
 * This class instantiates a subscriber node and listens to data on
 * ImuDiagData topic and compares it against a range of expected
 * values.
 */
class ImuDiagSubscriberTest : public ::testing::Test
{
public:
  /**
   * @brief Set up the test case.
   */
  static void SetUpTestCase() {}

  /**
   * @brief Tear down the test case.
   */
  static void TearDownTestCase() { rclcpp::shutdown(); }
};

/**
 * @brief ImuDiagSubscriberTest
 *
 * This test instantiates a subscriber node and listens to data on
 * ImuDiagData topic and compares it against a range of expected
 * values.
 */
TEST(ImuDiagSubscriberTest, test_imu_diag_data_publisher_adis1650x)
{
  auto node = rclcpp::Node::make_shared("test_imudiagdata_publisher");

  node->declare_parameter("iio_context_string", "local:");
  node->declare_parameter("imu_device_name", "unknown");

  std::string context =
    node->get_parameter("iio_context_string").get_parameter_value().get<std::string>();
  std::string device_name =
    node->get_parameter("imu_device_name").get_parameter_value().get<std::string>();

  auto device_descriptor = adi_imu::ADISDeviceFactory::make(device_name);
  if (device_descriptor->getDeviceFamily() != "adis1650x") {
    GTEST_SKIP() << "Test skipped, device is not " << device_descriptor->getDeviceFamily()
                 << "family.";
  }

  std::string topic = "imudiagdata";
  bool callbackExecuted = false;

  auto callback = [&callbackExecuted,
                   &device_descriptor](adi_imu::msg::ImuDiagDataADIS1650X msg) -> void {
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
    ASSERT_TRUE(msg.diag_flash_memory_write_count_exceeded_error == false);
    ASSERT_TRUE(
      msg.flash_counter < device_descriptor->get(adi_imu::ADISRegister::FLS_MEM_ENDURANCE));

    callbackExecuted = true;
  };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto subscriber =
    node->create_subscription<adi_imu::msg::ImuDiagDataADIS1650X>(topic, 10, callback);

  std::chrono::seconds sec(1);

  while (!callbackExecuted) executor.spin_once(sec);
}
