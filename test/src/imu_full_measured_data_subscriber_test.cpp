/*******************************************************************************
*   @file   imu_full_measured_data_subscriber_test.cpp
*   @brief  Test all sensors data
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
#include "imu_ros2/msg/imu_full_measured_data.hpp"

class ImuFullMeasuredDataSubscriberTest : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  static void TearDownTestCase() { rclcpp::shutdown(); }
};

TEST(ImuFullMeasuredDataSubscriberTest, test_imu_full_measured_data_publisher)
{
  IIOWrapper iio_wrapper;

  auto node = rclcpp::Node::make_shared("imufullmeasureddata");

  std::string topic = "imufullmeasureddata";

  double scale_accel = iio_wrapper.get_scale_accel();
  double scale_angvel = iio_wrapper.get_scale_angvel();
  double scale_velocity = iio_wrapper.get_scale_velocity();
  double scale_rot = iio_wrapper.get_scale_rot();
  double scale_temp = iio_wrapper.get_scale_temp();
  bool callbackExecuted = false;

  auto callback = [&scale_accel, &scale_angvel, &scale_velocity, &scale_rot, &scale_temp,
                   &callbackExecuted](imu_ros2::msg::ImuFullMeasuredData msg) -> void {
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp_test_imu_full_measured_data"),
      " acceleration: %f %f %f and gyroscope: %f %f %f \n"
      " delta_velocity: %f %f %f delta_angle: %f %f %f and temperature: %f",
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
      msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.delta_velocity.x,
      msg.delta_velocity.y, msg.delta_velocity.z, msg.delta_angle.x, msg.delta_angle.y,
      msg.delta_angle.z, msg.temperature);

    int32_t minint = -2147483648;
    int32_t maxint = 2147483647;

    double maxRangeAccel = maxint * scale_accel;
    double minRangeAccel = minint * scale_accel;

    double maxRangeAngvel = maxint * scale_angvel;
    double minRangeAngvel = minint * scale_angvel;

    double maxRangeVelocity = maxint * scale_velocity;
    double minRangeVelocity = minint * scale_velocity;

    double maxRangeRot = maxint * scale_rot;
    double minRangeRot = minint * scale_rot;

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

    ASSERT_TRUE(msg.delta_velocity.x >= minRangeVelocity);
    ASSERT_TRUE(msg.delta_velocity.y >= minRangeVelocity);
    ASSERT_TRUE(msg.delta_velocity.z >= minRangeVelocity);

    ASSERT_TRUE(msg.delta_velocity.x <= maxRangeVelocity);
    ASSERT_TRUE(msg.delta_velocity.y <= maxRangeVelocity);
    ASSERT_TRUE(msg.delta_velocity.z <= maxRangeVelocity);

    ASSERT_TRUE(msg.delta_angle.x >= minRangeRot);
    ASSERT_TRUE(msg.delta_angle.y >= minRangeRot);
    ASSERT_TRUE(msg.delta_angle.z >= minRangeRot);

    ASSERT_TRUE(msg.delta_angle.x <= maxRangeRot);
    ASSERT_TRUE(msg.delta_angle.y <= maxRangeRot);
    ASSERT_TRUE(msg.delta_angle.z <= maxRangeRot);

    ASSERT_TRUE(msg.temperature >= minRangeTemp);
    ASSERT_TRUE(msg.temperature <= maxRangeTemp);
    callbackExecuted = true;
  };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto subscriber =
    node->create_subscription<imu_ros2::msg::ImuFullMeasuredData>(topic, 10, callback);

  std::chrono::seconds sec(1);

  while (!callbackExecuted) executor.spin_once(sec);
}
