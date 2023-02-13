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

#include "imu_ros2/gyroscope_data_provider.h"
#include "imu_ros2/gyroscope_ros_publisher.h"

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("imu_ros2_node")
  {
      declare_parameter("my_parameter", "world");
      declare_parameter("diag_checksum_error_flag", "val");
      declare_parameter("diag_flash_memory_write_count_exceeded_error", "val");
      declare_parameter("diag_acceleration_self_test_error", "val");
      declare_parameter("diag_gyroscope2_self_test_error", "val");
      declare_parameter("diag_gyroscope1_self_test_error", "val");
      declare_parameter("diag_clock_error", "val");
      declare_parameter("diag_flash_memory_test_error", "val");
      declare_parameter("diag_sensor_self_test_error", "val");
      declare_parameter("diag_standby_mode", "val");
      declare_parameter("diag_spi_communication_error", "val");
      declare_parameter("diag_flash_memory_update_error", "val");
      declare_parameter("diag_data_path_overrun", "val");
      declare_parameter("time_stamp", "val");
      declare_parameter("data_counter", "val");
      declare_parameter("filter_size", "val");
      declare_parameter("gyroscope_measurement_range", "val");
      declare_parameter("burst_size_selection", "val");
      declare_parameter("burst_data_selection", "val");
      declare_parameter("linear_acceleration_compensation", "val");
      declare_parameter("point_of_percussion_alignment", "val");
      declare_parameter("internal_sensor_bandwidth", "val");
      declare_parameter("sync_mode_select", "val");
      declare_parameter("sync_polarity", "val");
      declare_parameter("data_ready_polarity", "val");
      declare_parameter("sync_signal_scale", "val");
      declare_parameter("decimation_filter", "val");
      declare_parameter("software_reset", "val");
      declare_parameter("flash_memory_test", "val");
      declare_parameter("flash_memory_update", "val");
      declare_parameter("sensor_self_test", "val");
      declare_parameter("factory_calibration_restore", "val");
      declare_parameter("firmware_revison", "val");
      declare_parameter("firmware_date", "val");
      declare_parameter("product_id", "val");
      declare_parameter("serial_number", "val");
      declare_parameter("scratch_pad_register1", "val");
      declare_parameter("scratch_pad_register2", "val");
      declare_parameter("scratch_pad_register3", "val");
      declare_parameter("flash_counter", "val");
      declare_parameter("accel_calibbias_x", "val");
      declare_parameter("accel_calibbias_y", "val");
      declare_parameter("accel_calibbias_z", "val");
      declare_parameter("gyro_calibbias_x", "val");
      declare_parameter("gyro_calibbias_y", "val");
      declare_parameter("gyro_calibbias_z", "val");


  }

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = std::make_shared<MinimalParam>();

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

    GyroscopeDataProviderInterface* gyroDataProv = new GyroscopeDataProvider();
    GyroscopeRosPublisherInterface * gyroPublisher = new GyroscopeRosPublisher(node);
    gyroPublisher->setMessageProvider(gyroDataProv);

    RosTask* gyroRosTask = dynamic_cast<RosTask*>(gyroPublisher);

    WorkerThread wth(rosTask);
    WorkerThread accwth(accRosTask);
    WorkerThread stawth(staRosTask);
    WorkerThread gyrowth(gyroRosTask);
    wth.join();
    accwth.join();
    stawth.join();
    gyrowth.join();

    delete publisher1;
    delete accPublisher;
    delete staPublisher;
    delete gyroPublisher;

    return 0;
}
