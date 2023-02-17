/***************************************************************************//**
*   @file   subscriber_test_string.cpp
*   @brief  Test a string
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
#include <std_msgs/msg/string.hpp>
#include <chrono>


class SubscriberTestString  : public ::testing::Test
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

TEST(SubscriberTestString,test_string_that_is_publish_from_publisher_string)
{
    auto node = rclcpp::Node::make_shared("topicStr");

    std::string topic = "topicStr";

    int counter = 0;


    auto callback =
            [&counter]( std_msgs::msg::String msg) -> void
    {
        ++counter;
        //printf("  callback() %d with message data %s\n", counter, msg.data);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_test"), " callback() %d with message data %s \n", counter, msg.data.c_str());
        ASSERT_TRUE(msg.data.length() > 0);


    };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto subscriber = node->create_subscription<std_msgs::msg::String>(topic, 10, callback);

    std::chrono::seconds sec(1);
    //auto seconds = std::chrono::duration_cast<std::chrono::seconds>(0);

    for(int i=0;i<100;i++)
    {
    executor.spin_once(sec);
    }

    //executor.spin();

    ASSERT_TRUE(true);
}

TEST(SubscriberTestString,test_string_that_is_publish_from_publisher_string_second_test)
{
    auto node = rclcpp::Node::make_shared("topicStr");

    std::string topic = "topicStr";

    int counter = 0;


    auto callback =
            [&counter]( std_msgs::msg::String msg) -> void
    {
        ++counter;
        //printf("  callback() %d with message data %s\n", counter, msg.data);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_test"), " callback() %d with message data %s \n", counter, msg.data.c_str());
        ASSERT_TRUE(msg.data.length() > 0);


    };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto subscriber = node->create_subscription<std_msgs::msg::String>(topic, 10, callback);

    std::chrono::seconds sec(1);
    //auto seconds = std::chrono::duration_cast<std::chrono::seconds>(0);

    for(int i=0;i<100;i++)
    {
    executor.spin_once(sec);
    }

    //executor.spin();

    ASSERT_TRUE(true);
}
