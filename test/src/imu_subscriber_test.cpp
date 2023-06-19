/***************************************************************************//**
*   @file   imu_subscriber_test.cpp
*   @brief  Test acceleration
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
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>


class ImuSubscriberTest  : public ::testing::Test
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

TEST(ImuSubscriberTest,test_acceleration_pozitive_values1)
{
    auto node = rclcpp::Node::make_shared("imuacceleration");

    std::string topic = "imuacceleration";

    int counter = 0;


    auto callback =
            [&counter]( sensor_msgs::msg::Imu msg) -> void
    {
        ++counter;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_test_acceleration"), " acceleration: %f %f %f  \n",
                    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z );
        ASSERT_TRUE(msg.linear_acceleration.x >= 0);
        ASSERT_TRUE(msg.linear_acceleration.y >= 0);
        ASSERT_TRUE(msg.linear_acceleration.z >= 0);


    };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto subscriber = node->create_subscription<sensor_msgs::msg::Imu>(topic, 10, callback);

    std::chrono::seconds sec(1);


    for(int i=0;i<100;i++)
    {
    executor.spin_once(sec);
    }

    //executor.spin();

    ASSERT_TRUE(true);
}

TEST(ImuSubscriberTest,test_acceleration_pozitive_values2)
{
    auto node = rclcpp::Node::make_shared("imuacceleration");

    std::string topic = "imuacceleration";

    int counter = 0;


    auto callback =
            [&counter]( sensor_msgs::msg::Imu msg) -> void
    {
        ++counter;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_test_acceleration"), " acceleration: %f %f %f  \n",
                    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z );
        ASSERT_TRUE(msg.linear_acceleration.x >= 0);
        ASSERT_TRUE(msg.linear_acceleration.y >= 0);
        ASSERT_TRUE(msg.linear_acceleration.z >= 0);


    };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto subscriber = node->create_subscription<sensor_msgs::msg::Imu>(topic, 10, callback);

    std::chrono::seconds sec(1);


    for(int i=0;i<100;i++)
    {
    executor.spin_once(sec);
    }

    //executor.spin();

    ASSERT_TRUE(true);
}
