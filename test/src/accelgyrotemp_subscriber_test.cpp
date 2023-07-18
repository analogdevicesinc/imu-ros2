/*******************************************************************************
*   @file   accelgyrotemp_subscriber_test.cpp
*   @brief  Test acceleration, gyroscope and temperature
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
#include "imu_ros2/msg/accel_gyro_temp_data.hpp"

/**
 * \brief Class for testing the accel gyro temp data
 *
 * This class instantiate a subscriber node and listen data
 * from topic and compare with a range of values.
 */
class AccelGyroTempSubscriberTest : public ::testing::Test
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
 * \brief AccelGyroTempSubscriberTest
 *
 * This test instantiate a subscriber node and listen data
 * from topic and compare with a range of values.
 */
TEST(AccelGyroTempSubscriberTest, test_accelgyrotemp_publisher)
{
  IIOWrapper iio_wrapper;

  auto node = rclcpp::Node::make_shared("accelgyrotempdata");

  std::string topic = "accelgyrotempdata";

  double scale_accel = iio_wrapper.get_scale_accel();
  double scale_angvel = iio_wrapper.get_scale_angvel();
  double scale_temp = iio_wrapper.get_scale_temp();
  bool callbackExecuted = false;

  auto callback = [&scale_accel, &scale_angvel, &scale_temp,
                   &callbackExecuted](imu_ros2::msg::AccelGyroTempData msg) -> void {
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp_test_accel_gyro_temp"),
      " acceleration: %f %f %f and gyroscope: %f %f %f \n", msg.linear_acceleration.x,
      msg.linear_acceleration.y, msg.linear_acceleration.z, msg.angular_velocity.x,
      msg.angular_velocity.y, msg.angular_velocity.z);

    int32_t minint = -2147483648;
    int32_t maxint = 2147483647;

    double maxRangeAccel = maxint * scale_accel;
    double minRangeAccel = minint * scale_accel;

    double maxRangeAngvel = maxint * scale_angvel;
    double minRangeAngvel = minint * scale_angvel;

    int32_t minint16 = -32768;
    int32_t maxint16 = 32767;

    double maxRangeTemp = maxint16 * scale_temp;
    double minRangeTemp = minint16 * scale_temp;

    ASSERT_TRUE(msg.linear_acceleration.x >= minRangeAccel);
    ASSERT_TRUE(msg.linear_acceleration.y >= minRangeAccel);
    ASSERT_TRUE(msg.linear_acceleration.z >= minRangeAccel);

    ASSERT_TRUE(msg.linear_acceleration.x <= maxRangeAccel);
    ASSERT_TRUE(msg.linear_acceleration.y <= maxRangeAccel);
    ASSERT_TRUE(msg.linear_acceleration.z <= maxRangeAccel);

    ASSERT_TRUE(msg.angular_velocity.x >= minRangeAngvel);
    ASSERT_TRUE(msg.angular_velocity.y >= minRangeAngvel);
    ASSERT_TRUE(msg.angular_velocity.z >= minRangeAngvel);

    ASSERT_TRUE(msg.angular_velocity.x <= maxRangeAngvel);
    ASSERT_TRUE(msg.angular_velocity.y <= maxRangeAngvel);
    ASSERT_TRUE(msg.angular_velocity.z <= maxRangeAngvel);

    ASSERT_TRUE(msg.temperature >= minRangeTemp);
    ASSERT_TRUE(msg.temperature <= maxRangeTemp);

    callbackExecuted = true;
  };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto subscriber =
    node->create_subscription<imu_ros2::msg::AccelGyroTempData>(topic, 10, callback);

  std::chrono::seconds sec(1);

  while (!callbackExecuted) executor.spin_once(sec);
}
