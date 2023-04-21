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

#include "imu_ros2/imu_control_parameters.h"
#include "imu_ros2/setting_declarations.h"

using namespace std::chrono_literals;

void declareParameters(std::shared_ptr<rclcpp::Node>& node)
{  
    node->declare_parameter("operation_mode", USER_PARAM_SETTING_MODE);
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

    ImuControlParameters * ctrlParam = new ImuControlParameters(node);
    RosTask* ctrlParamTask = dynamic_cast<RosTask*>(ctrlParam);

    WorkerThread accwth(accRosTask);
    WorkerThread idenwth(idenRosTask);
    WorkerThread aiwth(aiRosTask);
    WorkerThread diagwth(diagRosTask);
    WorkerThread ctrlParamwth(ctrlParamTask);

    accwth.join();
    idenwth.join();
    aiwth.join();
    diagwth.join();
    ctrlParamwth.join();

    delete accPublisher;
    delete idenPublisher;
    delete aiPublisher;
    delete diagPublisher;
    delete ctrlParam;

    rclcpp::shutdown();

    return 0;
}
