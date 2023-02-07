#include "imu_ros2/gyroscope_ros_publisher.h"
#include <thread>


GyroscopeRosPublisher::GyroscopeRosPublisher(std::shared_ptr<rclcpp::Node>& node)
{
    init(node);
}

GyroscopeRosPublisher::~GyroscopeRosPublisher()
{
    delete m_dataProvider;
}

void GyroscopeRosPublisher::init(std::shared_ptr<rclcpp::Node> &node)
{
    m_node = node;
    m_publisher = node->create_publisher<imu_ros2::msg::GyroscopeData>("imugyroscope", 10);
}

void GyroscopeRosPublisher::setMessageProvider(GyroscopeDataProviderInterface *dataProvider)
{
    m_dataProvider = dataProvider;
}

void GyroscopeRosPublisher::run()
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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing gyroscope data: '%f' '%f' '%f'",
                    m_message.anglvel_x, m_message.anglvel_y, m_message.anglvel_z);

        m_publisher->publish(m_message);
        count++;
        //rclcpp::spin_some(m_node);
        loopRate.sleep();
    }
    this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " ended...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "endThread: '%d'", this_id);
}