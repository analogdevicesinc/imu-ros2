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

#include "imu_ros2/iio_wrapper.h"
#include "imu_ros2/msg/imu_identification_data.hpp"

class ImuIdentificationSubscriberTest : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  static void TearDownTestCase() { rclcpp::shutdown(); }
};

TEST(ImuIdentificationSubscriberTest, test_imu_identification_subscriber)
{
  IIOWrapper iio_wrapper;

  auto node = rclcpp::Node::make_shared("imuidentificationdata");

  std::string topic = "imuidentificationdata";

  bool callbackExecuted = false;
  imu_ros2::msg::ImuIdentificationData imu_message;

  iio_wrapper.firmware_revision(imu_message.firmware_revision);
  iio_wrapper.firmware_date(imu_message.firmware_date);
  iio_wrapper.product_id(imu_message.product_id);
  iio_wrapper.serial_number(imu_message.serial_number);
  iio_wrapper.gyroscope_measurement_range(imu_message.gyroscope_measurement_range);

  auto callback = [&imu_message,
                   &callbackExecuted](imu_ros2::msg::ImuIdentificationData msg) -> void {
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp_imu_identification_data"), " device info: %s %s %d  \n",
      msg.firmware_revision.c_str(), msg.firmware_date.c_str(), msg.product_id);
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
    node->create_subscription<imu_ros2::msg::ImuIdentificationData>(topic, 10, callback);

  std::chrono::seconds sec(1);

  while (!callbackExecuted) executor.spin_once(sec);
}
