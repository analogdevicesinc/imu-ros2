#include "imu_ros2/rospublisher1.h"
#include <thread>


RosPublisher1::RosPublisher1(std::shared_ptr<rclcpp::Node>& node)
{
    init(node);
}

RosPublisher1::~RosPublisher1()
{
    delete m_dataProvider;
}

void RosPublisher1::init(std::shared_ptr<rclcpp::Node> &node)
{
    m_publisher = node->create_publisher<std_msgs::msg::String>("topicStr", 10);
}

void RosPublisher1::setMessageProvider(DataProviderInterface *dataProvider)
{
    m_dataProvider = dataProvider;
}

void RosPublisher1::run()
{
    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " started...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "startThread: '%d'", this_id);

    rclcpp::WallRate loopRate(10);

    int count = 0;
    while (rclcpp::ok()) {

       std::thread::id this_id = std::this_thread::get_id();
        std::cout << "thread " << this_id << " running...\n";
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "running: '%d'", this_id);

        m_message = m_dataProvider->getData(count);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing: '%s'", m_message.data.c_str());

        m_publisher->publish(m_message);
        count++;
        //rclcpp::spin_some(m_node);
        loopRate.sleep();
    }
    this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " ended...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "endThread: '%d'", this_id);
}
