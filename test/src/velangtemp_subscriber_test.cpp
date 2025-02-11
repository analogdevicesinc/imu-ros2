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

/**
 * @brief Class for testing VelAngTempData.
 *
 * This class instantiates a subscriber node and listens to data on
 * VelAngTempData topic and compares it against a range of expected
 * values.
 */
class VelAngTempSubscriberTest : public ::testing::Test
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
 * @brief VelAngTempSubscriberTest
 *
 * This test instantiates a subscriber node and listens to data on
 * VelAngTempData topic and compares it against a range of expected
 * values.
 */

TEST(VelAngTempSubscriberTest, test_velangtemp_publisher)
{
   if(AdisData::GetInstance()->getDeviceValue("ADIS_HAS_DELTA_BURST")) {
  IIOWrapper iio_wrapper;

  auto node = rclcpp::Node::make_shared("test_velangtempdata_publisher");

  node->declare_parameter("iio_context_string", "local:");

  std::string context =
    node->get_parameter("iio_context_string").get_parameter_value().get<std::string>();
  IIOWrapper m_iio_wrapper;
  m_iio_wrapper.createContext(context.c_str());

  std::string topic = "velangtempdata";

  double scale_deltavelocity = iio_wrapper.get_scale_deltavelocity();
  double scale_deltaangl = iio_wrapper.get_scale_deltaangl();
  double scale_temp = iio_wrapper.get_scale_temp();
  bool callbackExecuted = false;

  auto callback = [&scale_deltavelocity, &scale_deltaangl, &scale_temp,
                   &callbackExecuted](imu_ros2::msg::VelAngTempData msg) -> void {
    RCLCPP_INFO(
      rclcpp::get_logger("velangtemp_subscriber_test"),
      "delta velocity x axis: %f \ndelta velocity y axis: %f\ndelta velocity z axis: %f\n"
      "delta angle x axis: %f\ndelta angle y axis: %f\ndelta angle z axis: %f\n"
      "temperature: %f\n",
      msg.delta_velocity.x, msg.delta_velocity.y, msg.delta_velocity.z, msg.delta_angle.x,
      msg.delta_angle.y, msg.delta_angle.z, msg.temperature);

    int32_t minint = -2147483648;
    int32_t maxint = 2147483647;

    double maxRangeVelocity = maxint * scale_deltavelocity;
    double minRangeVelocity = minint * scale_deltavelocity;

    double maxRangeRot = maxint * scale_deltaangl;
    double minRangeRot = minint * scale_deltaangl;

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

}

