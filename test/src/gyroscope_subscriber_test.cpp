#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <chrono>
#include "imu_ros2/msg/gyroscope_data.hpp"

class GyroscopeSubscriberTest  : public ::testing::Test
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

TEST(GyroscopeSubscriberTest,test_gyroscope_pozitive_values1)
{
    auto node = rclcpp::Node::make_shared("imugyroscope");

    std::string topic = "imugyroscope";

    int counter = 0;


    auto callback =
            [&counter]( imu_ros2::msg::GyroscopeData msg) -> void
    {
        ++counter;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_test_gyroscope_data"), " gyroscope value : %f %f %f \n",
                    msg.anglvel_x, msg.anglvel_y, msg.anglvel_z );
        ASSERT_TRUE(msg.anglvel_x >= 0);
        ASSERT_TRUE(msg.anglvel_y >= 0);
        ASSERT_TRUE(msg.anglvel_z >= 0);



    };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto subscriber = node->create_subscription<imu_ros2::msg::GyroscopeData>(topic, 10, callback);

    std::chrono::seconds sec(1);


    for(int i=0;i<100;i++)
    {
    executor.spin_once(sec);
    }

    //executor.spin();

    ASSERT_TRUE(true);
}

