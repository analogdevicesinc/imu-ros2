#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "imu_ros2/rospublisher1.h"
#include "imu_ros2/workerthread.h"
#include "imu_ros2/dataproviderstring.h"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("imu_ros2_node");

    DataProviderInterface* dataStr = new DataProviderString();
    RosPublisherInterface * publisher1 = new RosPublisher1(node);
    publisher1->setMessageProvider(dataStr)
            ;
    RosTask* rosTask = dynamic_cast<RosTask*>(publisher1);

    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "mainthread " << this_id << " running...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_main"), "running: '%d'", this_id);

    WorkerThread wth(rosTask);
    wth.join();

    delete publisher1;
    delete dataStr;

    return 0;
}
