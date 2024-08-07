/*******************************************************************************
*   @file   imu__diag_subscriber_test.cpp
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

#include "imu_ros2/adis_data_access.h"
#include "imu_ros2/msg/imu_diag_data.hpp"

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
TEST(ImuDiagSubscriberTest, test_imu__diag_data_publisher)
{
  auto node = rclcpp::Node::make_shared("test_imudiagdata_publisher");

  std::string topic = "imudiagdata";
  bool callbackExecuted = false;

  auto callback = [&callbackExecuted](imu_ros2::msg::ImuDiagData msg) -> void {

#ifdef ADIS_SNSR_INIT_FAIL
    ASSERT_TRUE(msg.diag_sensor_initialization_failure == false);
#endif
#ifdef ADIS_DATA_PATH_OVERRUN
    ASSERT_TRUE(msg.diag_data_path_overrun == false);
#endif
#ifdef ADIS_WDG_TIMER_FLAG
    ASSERT_TRUE(msg.diag_automatic_reset == false);
#endif
#ifdef ADIS_FLS_MEM_UPDATE_FAIL
    ASSERT_TRUE(msg.diag_flash_memory_update_error == false);
#endif
#ifdef ADIS_SPI_COMM_ERR
    ASSERT_TRUE(msg.diag_spi_communication_error == false);
#endif
#ifdef ADIS_CRC_ERROR
    ASSERT_TRUE(msg.diag_crc_error == false);
#endif
#ifdef ADIS_STDBY_MODE
    ASSERT_TRUE(msg.diag_standby_mode == false);
#endif
#ifdef ADIS_SNSR_FAIL
    ASSERT_TRUE(msg.diag_sensor_self_test_error == false);
#endif
#ifdef ADIS_MEM_FAIL
    ASSERT_TRUE(msg.diag_flash_memory_test_error == false);
#endif
#ifdef ADIS_CLK_ERR
    ASSERT_TRUE(msg.diag_clock_error == false);
#endif
#ifdef ADIS_ACCEL_FAIL
    ASSERT_TRUE(msg.diag_acceleration_self_test_error == false);
#endif
#ifdef ADIS_GYRO1_FAIL
    ASSERT_TRUE(msg.diag_gyroscope1_self_test_error == false);
#endif
#ifdef ADIS_GYRO2_FAIL
    ASSERT_TRUE(msg.diag_gyroscope2_self_test_error == false);
#endif
#ifdef ADIS_GYRO_X_FAIL
    ASSERT_TRUE(msg.diag_x_axis_gyroscope_failure == false);
#endif
#ifdef ADIS_GYRO_Y_FAIL
    ASSERT_TRUE(msg.diag_y_axis_gyroscope_failure == false);
#endif
#ifdef ADIS_GYRO_Z_FAIL
    ASSERT_TRUE(msg.diag_z_axis_gyroscope_failure == false);
#endif
#ifdef ADIS_ACCEL_X_FAIL
    ASSERT_TRUE(msg.diag_x_axis_accelerometer_failure == false);
#endif
#ifdef ADIS_ACCEL_Y_FAIL
    ASSERT_TRUE(msg.diag_y_axis_accelerometer_failure == false);
#endif
#ifdef ADIS_ACCEL_Z_FAIL
    ASSERT_TRUE(msg.diag_z_axis_accelerometer_failure == false);
#endif
#ifdef ADIS_ADUC_MCU_FAULT
    ASSERT_TRUE(msg.diag_aduc_mcu_fault == false);
#endif
    ASSERT_TRUE(msg.diag_flash_memory_write_count_exceeded_error == false);

    ASSERT_TRUE(msg.flash_counter < ADIS_FLS_MEM_ENDURANCE);
    callbackExecuted = true;
  };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto subscriber = node->create_subscription<imu_ros2::msg::ImuDiagData>(topic, 10, callback);

  std::chrono::seconds sec(1);

  while (!callbackExecuted) executor.spin_once(sec);
}
