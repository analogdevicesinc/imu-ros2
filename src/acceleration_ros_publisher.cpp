#include "imu_ros2/acceleration_ros_publisher.h"
#include <thread>


AccelerationRosPublisher::AccelerationRosPublisher(std::shared_ptr<rclcpp::Node>& node)
{
    init(node);
}

AccelerationRosPublisher::~AccelerationRosPublisher()
{
    delete m_dataProvider;
}

void AccelerationRosPublisher::init(std::shared_ptr<rclcpp::Node> &node)
{
    m_node = node;
    m_publisher = node->create_publisher<sensor_msgs::msg::Imu>("imuacceleration", 10);
}

void AccelerationRosPublisher::setMessageProvider(AccelerationDataProviderInterface *dataProvider)
{
    m_dataProvider = dataProvider;
}

void AccelerationRosPublisher::run()
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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing acceleration x y z: '%f' '%f' '%f'",
                    m_message.linear_acceleration.x, m_message.linear_acceleration.y, m_message.linear_acceleration.z);

        m_publisher->publish(m_message);
        count++;
        //rclcpp::spin_some(m_node);
        loopRate.sleep();
    }
    this_id = std::this_thread::get_id();
    std::cout << "thread " << this_id << " ended...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "endThread: '%d'", this_id);
}
