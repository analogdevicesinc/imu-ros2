/*******************************************************************************
*   @file   imu_identification_subscriber_test.cpp
*   @brief  Test imu identification data
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

#include "adi_imu/iio_wrapper.h"
#include "adi_imu/msg/imu_identification_data.hpp"

/**
 * @brief Class for testing ImuIdentificationData.
 *
 * This class instantiates a subscriber node and listens to data on
 * ImuIdentificationData topic and compares it against a range of expected
 * values.
 */
class ImuIdentificationSubscriberTest : public ::testing::Test
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
 * @brief ImuIdentificationSubscriberTest
 *
 * This test instantiates a subscriber node and listens to data on
 * ImuIdentificationData topic and compares it against a range of expected
 * values.
 */
TEST(ImuIdentificationSubscriberTest, test_imu_identification_publisher)
{
  IIOWrapper iio_wrapper;

  auto node = rclcpp::Node::make_shared("test_imuidentificationdata_publisher");

  node->declare_parameter("iio_context_string", "local:");

  std::string context =
    node->get_parameter("iio_context_string").get_parameter_value().get<std::string>();
  IIOWrapper m_iio_wrapper;
  m_iio_wrapper.createContext(context.c_str());

  std::string topic = "imuidentificationdata";

  bool callbackExecuted = false;
  adi_imu::msg::ImuIdentificationData imu_message;

  iio_wrapper.firmware_revision(imu_message.firmware_revision);
  iio_wrapper.firmware_date(imu_message.firmware_date);
  iio_wrapper.product_id(imu_message.product_id);
  iio_wrapper.serial_number(imu_message.serial_number);
  iio_wrapper.gyroscope_measurement_range(imu_message.gyroscope_measurement_range);

  auto callback = [&imu_message,
                   &callbackExecuted](adi_imu::msg::ImuIdentificationData msg) -> void {
    RCLCPP_INFO(
      rclcpp::get_logger("imu_identification_subscriber_test"),
      "\nproduct id: %d"
      "\nserial number: %d"
      "\nfirmware revision: %s"
      "\nfirmare date: %s"
      "\ngyroscope measurement range: %s",
      msg.product_id, msg.serial_number, msg.firmware_revision.c_str(), msg.firmware_date.c_str(),
      msg.gyroscope_measurement_range.c_str());
    ASSERT_TRUE(msg.firmware_revision == imu_message.firmware_revision);
    ASSERT_TRUE(msg.firmware_date == imu_message.firmware_date);
    ASSERT_TRUE(msg.product_id == imu_message.product_id);
    ASSERT_TRUE(msg.serial_number == imu_message.serial_number);
    ASSERT_TRUE(msg.gyroscope_measurement_range == imu_message.gyroscope_measurement_range);
    callbackExecuted = true;
  };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto subscriber =
    node->create_subscription<adi_imu::msg::ImuIdentificationData>(topic, 10, callback);

  std::chrono::seconds sec(1);

  while (!callbackExecuted) executor.spin_once(sec);
}
