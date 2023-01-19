#include "imu_ros2/static_ros_publisher.h"
#include <thread>


StaticRosPublisher::StaticRosPublisher(std::shared_ptr<rclcpp::Node>& node)
{
    init(node);
}

StaticRosPublisher::~StaticRosPublisher()
{
    delete m_dataProvider;
}

void StaticRosPublisher::init(std::shared_ptr<rclcpp::Node> &node)
{
    m_publisher = node->create_publisher<imu_ros2::msg::StaticData>("imudevicedata", 10);
}

void StaticRosPublisher::setMessageProvider(StaticDataProviderInterface *dataProvider)
{
    m_dataProvider = dataProvider;
}

void StaticRosPublisher::run()
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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing static data: '%s' '%s' '%s'",
                    m_message.firmware_revision.c_str(), m_message.firmware_date.c_str(), m_message.product_id.c_str());

        m_publisher->publish(m_message);
        count++;
        //rclcpp::spin_some(m_node);
        loopRate.sleep();
    }
    this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " ended...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "endThread: '%d'", this_id);
}
