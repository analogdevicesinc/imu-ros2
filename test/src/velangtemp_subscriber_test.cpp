/*******************************************************************************
*   @file   velangtemp_subscriber_test.cpp
*   @brief  Test vel ang temp publisher
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
#include "imu_ros2/msg/vel_ang_temp_data.hpp"

class VelAngTempSubscriberTest : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  static void TearDownTestCase() { rclcpp::shutdown(); }
};

TEST(VelAngTempSubscriberTest, test_velangtemp_publisher)
{
  IIOWrapper iio_wrapper;

  auto node = rclcpp::Node::make_shared("velangtempdata");

  std::string topic = "velangtempdata";

  double scale_velocity = iio_wrapper.get_scale_velocity();
  double scale_rot = iio_wrapper.get_scale_rot();
  double scale_temp = iio_wrapper.get_scale_temp();
  bool callbackExecuted = false;

  auto callback = [&scale_velocity, &scale_rot, &scale_temp,
                   &callbackExecuted](imu_ros2::msg::VelAngTempData msg) -> void {
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp_test_adiimu_data"), " delta velocity value : %f %f %f \n",
      msg.delta_velocity.x, msg.delta_velocity.y, msg.delta_velocity.z);

    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp_test_adiimu_data"), " delta angle value : %f %f %f \n",
      msg.delta_angle.x, msg.delta_angle.y, msg.delta_angle.z);

    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp_test_adiimu_data"), " temperature value : %f \n", msg.temperature);

    int32_t minint = -2147483648;
    int32_t maxint = 2147483647;

    double maxRangeVelocity = maxint * scale_velocity;
    double minRangeVelocity = minint * scale_velocity;

    double maxRangeRot = maxint * scale_rot;
    double minRangeRot = minint * scale_rot;

    int32_t minint16 = -32768;
    int32_t maxint16 = 32767;

    double maxRangeTemp = maxint16 * scale_temp;
    double minRangeTemp = minint16 * scale_temp;

    ASSERT_TRUE(
      msg.delta_velocity.x >= minRangeVelocity && msg.delta_velocity.x <= maxRangeVelocity);
    ASSERT_TRUE(
      msg.delta_velocity.y >= minRangeVelocity && msg.delta_velocity.y <= maxRangeVelocity);
    ASSERT_TRUE(
      msg.delta_velocity.z >= minRangeVelocity && msg.delta_velocity.z <= maxRangeVelocity);

    ASSERT_TRUE(msg.delta_angle.x >= minRangeRot && msg.delta_angle.x <= maxRangeRot);
    ASSERT_TRUE(msg.delta_angle.y >= minRangeRot && msg.delta_angle.y <= maxRangeRot);
    ASSERT_TRUE(msg.delta_angle.z >= minRangeRot && msg.delta_angle.z <= maxRangeRot);

    ASSERT_TRUE(msg.temperature >= minRangeTemp && msg.temperature <= maxRangeTemp);
    callbackExecuted = true;
  };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto subscriber = node->create_subscription<imu_ros2::msg::VelAngTempData>(topic, 10, callback);

  std::chrono::seconds sec(1);

  while (!callbackExecuted) executor.spin_once(sec);
}
