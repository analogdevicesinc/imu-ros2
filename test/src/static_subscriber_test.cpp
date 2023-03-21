/***************************************************************************//**
*   @file   static_subscriber_test.cpp
*   @brief  Test static data
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
#include "imu_ros2/msg/imu_identification_data.hpp"

class StaticSubscriberTest  : public ::testing::Test
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

TEST(StaticSubscriberTest,test_static_data_values1)
{
    auto node = rclcpp::Node::make_shared("imuidentificationdata");

    std::string topic = "imuidentificationdata";

    int counter = 0;


    auto callback =
            [&counter]( imu_ros2::msg::ImuIdentificationData msg) -> void
    {
        ++counter;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_imu_identification_data"), " device info: %s %s %d  \n",
                    msg.firmware_revision.c_str(), msg.firmware_date.c_str(), msg.product_id );
        ASSERT_TRUE(msg.firmware_revision == "1.6");
        ASSERT_TRUE(msg.firmware_date == "06-27-2019");
        ASSERT_TRUE(msg.product_id == 16505);


    };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto subscriber = node->create_subscription<imu_ros2::msg::ImuIdentificationData>(topic, 10, callback);

    std::chrono::seconds sec(1);


    for(int i=0;i<100;i++)
    {
    executor.spin_once(sec);
    }

    //executor.spin();

    ASSERT_TRUE(true);
}

