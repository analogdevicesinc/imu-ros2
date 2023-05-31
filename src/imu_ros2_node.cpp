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

#include "imu_ros2/velangtemp_data_provider.h"
#include "imu_ros2/velangtemp_ros_publisher.h"

#include "imu_ros2/imu_diag_data_provider.h"
#include "imu_ros2/imu_diag_ros_publisher.h"

#include "imu_ros2/imu_control_parameters.h"
#include "imu_ros2/setting_declarations.h"

#include "imu_ros2/imu_full_measured_data_provider.h"
#include "imu_ros2/imu_full_measured_data_ros_publisher.h"

#include "imu_ros2/accelgyrotemp_data_provider.h"
#include "imu_ros2/accelgyrotemp_ros_publisher.h"

using namespace std::chrono_literals;

void declareParameters(std::shared_ptr<rclcpp::Node>& node)
{  
    node->declare_parameter("operation_mode", USER_PARAM_SETTING_MODE);
    node->declare_parameter("buffered_data_selection", ACCEL_GYRO_BUFFERED_DATA);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("imu_ros2_node");

    declareParameters(node);

    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "mainthread " << this_id << " running...\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_main"), "running: '%d'", this_id);

    ImuControlParameters * ctrlParam = new ImuControlParameters(node);
    RosTask* ctrlParamTask = dynamic_cast<RosTask*>(ctrlParam);

    AccelerationDataProviderInterface* accDataProv = new AccelerationDataProvider();
    AccelerationRosPublisherInterface * accPublisher = new AccelerationRosPublisher(node);
    accPublisher->setMessageProvider(accDataProv);

    RosTask* accRosTask = dynamic_cast<RosTask*>(accPublisher);

    ImuIdentificationDataProviderInterface* idenDataProv = new ImuIdentificationDataProvider();
    ImuIdentificationRosPublisherInterface * idenPublisher = new ImuIdentificationRosPublisher(node);
    idenPublisher->setMessageProvider(idenDataProv);

    RosTask* idenRosTask = dynamic_cast<RosTask*>(idenPublisher);

    VelAngTempDataProviderInterface* vatDataProv = new VelAngTempDataProvider();
    VelAngTempRosPublisherInterface * vatPublisher = new VelAngTempRosPublisher(node);
    vatPublisher->setMessageProvider(vatDataProv);

    RosTask* vatRosTask = dynamic_cast<RosTask*>(vatPublisher);

    AccelGyroTempDataProviderInterface* accelgyrotempDataProv = new AccelGyroTempDataProvider();
    AccelGyroTempRosPublisherInterface * accelgyrotempPublisher = new AccelGyroTempRosPublisher(node);
    accelgyrotempPublisher->setMessageProvider(accelgyrotempDataProv);

    RosTask* accelgyrotempRosTask = dynamic_cast<RosTask*>(accelgyrotempPublisher);

    ImuDiagDataProviderInterface* diagDataProv = new ImuDiagDataProvider();
    ImuDiagRosPublisherInterface * diagPublisher = new ImuDiagRosPublisher(node);
    diagPublisher->setMessageProvider(diagDataProv);

    RosTask* diagRosTask = dynamic_cast<RosTask*>(diagPublisher);

    ImuFullMeasuredDataProviderInterface* imuFullMeasuredDataProv = new ImuFullMeasuredDataProvider();
    ImuFullMeasuredDataRosPublisherInterface * imuFullMeasuredDataPublisher = new ImuFullMeasuredDataRosPublisher(node);
    imuFullMeasuredDataPublisher->setMessageProvider(imuFullMeasuredDataProv);

    RosTask* imuFullMeasuredDataRosTask = dynamic_cast<RosTask*>(imuFullMeasuredDataPublisher);

    WorkerThread ctrlParamwth(ctrlParamTask);
    WorkerThread accwth(accRosTask);
    WorkerThread idenwth(idenRosTask);
    WorkerThread vatwth(vatRosTask);
    WorkerThread accelgyrotempwth(accelgyrotempRosTask);
    WorkerThread diagwth(diagRosTask);
    WorkerThread imuFullMeasuredDatawth(imuFullMeasuredDataRosTask);

    ctrlParamwth.join();
    accwth.join();
    idenwth.join();
    vatwth.join();
    accelgyrotempwth.join();
    diagwth.join();
    imuFullMeasuredDatawth.join();

    delete accPublisher;
    delete idenPublisher;
    delete vatPublisher;
    delete accelgyrotempPublisher;
    delete diagPublisher;
    delete ctrlParam;
    delete imuFullMeasuredDataPublisher;

    rclcpp::shutdown();

    return 0;
}
