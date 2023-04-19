/***************************************************************************//**
*   @file   imu_ros2_node.cpp
*   @brief  Implementation for main function
*   @author Vasile Holonec (Vasile.Holonec@analog.com)
********************************************************************************
* Copyright 2023(c) Analog Devices, Inc.

* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "imu_ros2/worker_thread.h"

#include "imu_ros2/acceleration_data_provider.h"
#include "imu_ros2/acceleration_ros_publisher.h"

#include "imu_ros2/imu_identification_data_provider.h"
#include "imu_ros2/imu_identification_ros_publisher.h"

#include "imu_ros2/adiimu_data_provider.h"
#include "imu_ros2/adiimu_ros_publisher.h"

#include "imu_ros2/imu_diag_data_provider.h"
#include "imu_ros2/imu_diag_ros_publisher.h"

using namespace std::chrono_literals;

void declareParameters(std::shared_ptr<rclcpp::Node>& node)
{
    node->declare_parameter("my_parameter", "world");
    node->declare_parameter("diag_checksum_error_flag", "val");
    node->declare_parameter("diag_flash_memory_write_count_exceeded_error", "val");
    node->declare_parameter("diag_acceleration_self_test_error", "val");
    node->declare_parameter("diag_gyroscope2_self_test_error", "val");
    node->declare_parameter("diag_gyroscope1_self_test_error", "val");
    node->declare_parameter("diag_clock_error", "val");
    node->declare_parameter("diag_flash_memory_test_error", "val");
    node->declare_parameter("diag_sensor_self_test_error", "val");
    node->declare_parameter("diag_standby_mode", "val");
    node->declare_parameter("diag_spi_communication_error", "val");
    node->declare_parameter("diag_flash_memory_update_error", "val");
    node->declare_parameter("diag_data_path_overrun", "val");
    node->declare_parameter("time_stamp", "val");
    node->declare_parameter("data_counter", "val");
    node->declare_parameter("filter_size", 0);
    node->declare_parameter("gyroscope_measurement_range", "val");
    node->declare_parameter("burst_size_selection", "val");
    node->declare_parameter("burst_data_selection", "val");
    node->declare_parameter("linear_acceleration_compensation", "val");
    node->declare_parameter("point_of_percussion_alignment", "val");
    node->declare_parameter("internal_sensor_bandwidth", "val");
    node->declare_parameter("sync_mode_select", "val");
    node->declare_parameter("sync_polarity", "val");
    node->declare_parameter("data_ready_polarity", "val");
    node->declare_parameter("sync_signal_scale", "val");
    node->declare_parameter("decimation_filter", "val");
    node->declare_parameter("software_reset", "val");
    node->declare_parameter("flash_memory_test", "val");
    node->declare_parameter("flash_memory_update", "val");
    node->declare_parameter("sensor_self_test", "val");
    node->declare_parameter("factory_calibration_restore", "val");
    node->declare_parameter("firmware_revison", "val");
    node->declare_parameter("firmware_date", "val");
    node->declare_parameter("product_id", "val");
    node->declare_parameter("serial_number", "val");
    node->declare_parameter("scratch_pad_register1", "val");
    node->declare_parameter("scratch_pad_register2", "val");
    node->declare_parameter("scratch_pad_register3", "val");
    node->declare_parameter("flash_counter", "val");
    node->declare_parameter("accel_calibbias_x", "val");
    node->declare_parameter("accel_calibbias_y", "val");
    node->declare_parameter("accel_calibbias_z", "val");
    node->declare_parameter("gyro_calibbias_x", "val");
    node->declare_parameter("gyro_calibbias_y", "val");
    node->declare_parameter("gyro_calibbias_z", "val");


}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("imu_ros2_node");

    declareParameters(node);

    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "mainthread " << this_id << " running...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_main"), "running: '%d'", this_id);

    AccelerationDataProviderInterface* accDataProv = new AccelerationDataProvider();
    AccelerationRosPublisherInterface * accPublisher = new AccelerationRosPublisher(node);
    accPublisher->setMessageProvider(accDataProv);

    RosTask* accRosTask = dynamic_cast<RosTask*>(accPublisher);

    ImuIdentificationDataProviderInterface* idenDataProv = new ImuIdentificationDataProvider();
    ImuIdentificationRosPublisherInterface * idenPublisher = new ImuIdentificationRosPublisher(node);
    idenPublisher->setMessageProvider(idenDataProv);

    RosTask* idenRosTask = dynamic_cast<RosTask*>(idenPublisher);

    AdiImuDataProviderInterface* aiDataProv = new AdiImuDataProvider();
    AdiImuRosPublisherInterface * aiPublisher = new AdiImuRosPublisher(node);
    aiPublisher->setMessageProvider(aiDataProv);

    RosTask* aiRosTask = dynamic_cast<RosTask*>(aiPublisher);

    ImuDiagDataProviderInterface* diagDataProv = new ImuDiagDataProvider();
    ImuDiagRosPublisherInterface * diagPublisher = new ImuDiagRosPublisher(node);
    diagPublisher->setMessageProvider(diagDataProv);

    RosTask* diagRosTask = dynamic_cast<RosTask*>(diagPublisher);

    WorkerThread accwth(accRosTask);
    WorkerThread idenwth(idenRosTask);
    WorkerThread aiwth(aiRosTask);
    WorkerThread diagwth(diagRosTask);

    accwth.join();
    idenwth.join();
    aiwth.join();
    diagwth.join();

    delete accPublisher;
    delete idenPublisher;
    delete aiPublisher;
    delete diagPublisher;

    rclcpp::shutdown();

    return 0;
}
