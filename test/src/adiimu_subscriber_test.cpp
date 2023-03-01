/***************************************************************************//**
*   @file   gyroscope_subscriber_test.cpp
*   @brief  Test gyroscope
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

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <chrono>
#include "imu_ros2/msg/adi_imu_data.hpp"

class AdiImuSubscriberTest  : public ::testing::Test
{
public:
    static void SetUpTestCase()
    {

    }

    static void TearDownTestCase()
    {
        rclcpp::shutdown();
    }
};

TEST(AdiImuSubscriberTest,test_adiimu_data_intervals)
{
    auto node = rclcpp::Node::make_shared("adiimudata");

    std::string topic = "adiimudata";

    int counter = 0;


    auto callback =
            [&counter]( imu_ros2::msg::AdiImuData msg) -> void
    {
        ++counter;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_test_adiimu_data"), " acceleration value : %f %f %f \n",
                    msg.accel.x, msg.accel.y, msg.accel.z );
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_test_adiimu_data"), " gyroscope value : %f %f %f \n",
                    msg.gyro.x, msg.gyro.y, msg.gyro.z );

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_test_adiimu_data"), " delta velocity value : %f %f %f \n",
                    msg.delta_vel.x, msg.delta_vel.y, msg.delta_vel.z );

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_test_adiimu_data"), " delta angle value : %f %f %f \n",
                    msg.delta_angle.x, msg.delta_angle.y, msg.delta_angle.z );

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_test_adiimu_data"), " temperature value : %f \n",
                    msg.temp);


        ASSERT_TRUE(msg.accel.x >= -10000 && msg.accel.x <= 10000);
        ASSERT_TRUE(msg.accel.y >= -10000 && msg.accel.y <= 10000);
        ASSERT_TRUE(msg.accel.z >= -10000 && msg.accel.z <= 10000);

        ASSERT_TRUE(msg.gyro.x >= -10000 && msg.gyro.x <= 10000);
        ASSERT_TRUE(msg.gyro.y >= -10000 && msg.gyro.y <= 10000);
        ASSERT_TRUE(msg.gyro.z >= -10000 && msg.gyro.z <= 10000);

        ASSERT_TRUE(msg.delta_vel.x >= -10000 && msg.delta_vel.x <= 10000);
        ASSERT_TRUE(msg.delta_vel.y >= -10000 && msg.delta_vel.y <= 10000);
        ASSERT_TRUE(msg.delta_vel.z >= -10000 && msg.delta_vel.z <= 10000);

        ASSERT_TRUE(msg.delta_angle.x >= -10000 && msg.delta_angle.x <= 10000);
        ASSERT_TRUE(msg.delta_angle.y >= -10000 && msg.delta_angle.y <= 10000);
        ASSERT_TRUE(msg.delta_angle.z >= -10000 && msg.delta_angle.z <= 10000);

        ASSERT_TRUE(msg.temp >= -100 && msg.temp <= 100);
    };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto subscriber = node->create_subscription<imu_ros2::msg::AdiImuData>(topic, 10, callback);

    std::chrono::seconds sec(1);


    for(int i=0;i<100;i++)
    {
    executor.spin_once(sec);
    }

    //executor.spin();

    ASSERT_TRUE(true);
}

