#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "imu_ros2/worker_thread.h"

#include "imu_ros2/data_provider_string.h"
#include "imu_ros2/ros_publisher_1.h"

#include "imu_ros2/acceleration_data_provider.h"
#include "imu_ros2/acceleration_ros_publisher.h"

#include "imu_ros2/static_data_provider.h"
#include "imu_ros2/static_ros_publisher.h"

using namespace std::chrono_literals;

void declareParameters(std::shared_ptr<rclcpp::Node>& node)
{
    node->declare_parameter("my_parameter", "world");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("imu_ros2_node");

    declareParameters(node);

    DataProviderInterface* dataStr = new DataProviderString();
    RosPublisherInterface * publisher1 = new RosPublisher1(node);
    publisher1->setMessageProvider(dataStr);

    RosTask* rosTask = dynamic_cast<RosTask*>(publisher1);

    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "mainthread " << this_id << " running...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_main"), "running: '%d'", this_id);

    AccelerationDataProviderInterface* accDataProv = new AccelerationDataProvider();
    AccelerationRosPublisherInterface * accPublisher = new AccelerationRosPublisher(node);
    accPublisher->setMessageProvider(accDataProv);

    RosTask* accRosTask = dynamic_cast<RosTask*>(accPublisher);

    StaticDataProviderInterface* staDataProv = new StaticDataProvider();
    StaticRosPublisherInterface * staPublisher = new StaticRosPublisher(node);
    staPublisher->setMessageProvider(staDataProv);

    RosTask* staRosTask = dynamic_cast<RosTask*>(staPublisher);

    WorkerThread wth(rosTask);
    WorkerThread accwth(accRosTask);
    WorkerThread stawth(staRosTask);
    wth.join();
    accwth.join();
    stawth.join();

    delete publisher1;
    delete accPublisher;
    delete staPublisher;

    return 0;
}
