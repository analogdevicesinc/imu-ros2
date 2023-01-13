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
