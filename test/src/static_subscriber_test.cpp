#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <chrono>
#include "imu_ros2/msg/static_data.hpp"

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

TEST(StaticSubscriberTest,test_acceleration_pozitive_values1)
{
    auto node = rclcpp::Node::make_shared("imudevicedata");

    std::string topic = "imudevicedata";

    int counter = 0;


    auto callback =
            [&counter]( imu_ros2::msg::StaticData msg) -> void
    {
        ++counter;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp_test_static_data"), " device info: %s %s %s  \n",
                    msg.firmware_revision.c_str(), msg.firmware_date.c_str(), msg.product_id.c_str() );
        ASSERT_TRUE(msg.firmware_revision == "FIRMWARE_REVISION_1");
        ASSERT_TRUE(msg.firmware_date == "19-01-2023");
        ASSERT_TRUE(msg.product_id == "PRODUCT_ID_1");


    };

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto subscriber = node->create_subscription<imu_ros2::msg::StaticData>(topic, 10, callback);

    std::chrono::seconds sec(1);


    for(int i=0;i<100;i++)
    {
    executor.spin_once(sec);
    }

    //executor.spin();

    ASSERT_TRUE(true);
}

