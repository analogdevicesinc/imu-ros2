/***************************************************************************/ /**
 *   @file   imu_ros2_node.cpp
 *   @brief  Implementation for IMU node.
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

#include "imu_ros2/accelgyrotemp_data_provider.h"
#include "imu_ros2/accelgyrotemp_ros_publisher.h"
#include "imu_ros2/imu_1650x_diag_data_provider.h"
#include "imu_ros2/imu_1650x_diag_ros_publisher.h"
#include "imu_ros2/imu_1657x_diag_data_provider.h"
#include "imu_ros2/imu_1657x_diag_ros_publisher.h"
#include "imu_ros2/imu_control_parameters.h"
#include "imu_ros2/imu_data_provider.h"
#include "imu_ros2/imu_full_measured_data_provider.h"
#include "imu_ros2/imu_full_measured_data_ros_publisher.h"
#include "imu_ros2/imu_identification_data_provider.h"
#include "imu_ros2/imu_identification_ros_publisher.h"
#include "imu_ros2/imu_ros_publisher.h"
#include "imu_ros2/ros_publisher_group.h"
#include "imu_ros2/ros_publisher_group_interface.h"
#include "imu_ros2/setting_declarations.h"
#include "imu_ros2/velangtemp_data_provider.h"
#include "imu_ros2/velangtemp_ros_publisher.h"
#include "imu_ros2/worker_thread.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> imu_node = rclcpp::Node::make_shared("imu_ros2_node");

  std::thread::id this_id = std::this_thread::get_id();
  std::cout << "mainthread " << this_id << " running...\n";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp_main"), "running: main thread");

  ImuControlParameters * ctrl_params = new ImuControlParameters(imu_node);
  RosTask * ctrl_params_task = dynamic_cast<RosTask *>(ctrl_params);

  AccelGyroTempDataProviderInterface * accel_gyro_data_provider = new AccelGyroTempDataProvider();
  AccelGyroTempRosPublisherInterface * accel_gyro_publisher =
    new AccelGyroTempRosPublisher(imu_node);
  accel_gyro_publisher->setMessageProvider(accel_gyro_data_provider);

  ImuDataProviderInterface * imu_std_data_provider = new ImuDataProvider();
  ImuRosPublisherInterface * imu_std_publisher = new ImuRosPublisher(imu_node);
  imu_std_publisher->setMessageProvider(imu_std_data_provider);

  VelAngTempDataProviderInterface * vel_ang_data_provider = new VelAngTempDataProvider();
  VelAngTempRosPublisherInterface * vel_ang_publisher = new VelAngTempRosPublisher(imu_node);
  vel_ang_publisher->setMessageProvider(vel_ang_data_provider);

  ImuFullMeasuredDataProviderInterface * full_data_provider = new ImuFullMeasuredDataProvider();
  ImuFullMeasuredDataRosPublisherInterface * full_data_publisher =
    new ImuFullMeasuredDataRosPublisher(imu_node);
  full_data_publisher->setMessageProvider(full_data_provider);

  RosPublisherGroupInterface * publisher_group = new RosPublisherGroup(imu_node);
  publisher_group->setAccelGyroTempRosPublisher(accel_gyro_publisher);
  publisher_group->setVelAngTempRosPublisher(vel_ang_publisher);
  publisher_group->setImuRosPublisher(imu_std_publisher);
  publisher_group->setImuFullMeasuredDataRosPublisher(full_data_publisher);

  RosTask * publisher_group_task = dynamic_cast<RosTask *>(publisher_group);

  ImuIdentificationDataProviderInterface * ident_data_provider =
    new ImuIdentificationDataProvider();
  ImuIdentificationRosPublisherInterface * ident_publisher =
    new ImuIdentificationRosPublisher(imu_node);
  ident_publisher->setMessageProvider(ident_data_provider);
  RosTask * ident_task = dynamic_cast<RosTask *>(ident_publisher);

  Imu1650xDiagDataProviderInterface * diag1650x_data_provider = nullptr;
  Imu1650xDiagRosPublisherInterface * diag1650x_publisher = nullptr;
  RosTask * diag1650x_task = nullptr;

  Imu1657xDiagDataProviderInterface * diag1657x_data_provider = nullptr;
  Imu1657xDiagRosPublisherInterface * diag1657x_publisher = nullptr;
  RosTask * diag1657x_task = nullptr;
  switch (IIOWrapper::s_device_name_enum) {
    case IIODeviceName::ADIS1650X:
      diag1650x_data_provider = new Imu1650xDiagDataProvider();
      diag1650x_publisher = new Imu1650xDiagRosPublisher(imu_node);
      diag1650x_publisher->setMessageProvider(diag1650x_data_provider);

      diag1650x_task = dynamic_cast<RosTask *>(diag1650x_publisher);
      break;
    case IIODeviceName::ADIS1657X:
      diag1657x_data_provider = new Imu1657xDiagDataProvider();
      diag1657x_publisher = new Imu1657xDiagRosPublisher(imu_node);
      diag1657x_publisher->setMessageProvider(diag1657x_data_provider);

      diag1657x_task = dynamic_cast<RosTask *>(diag1657x_publisher);
      break;
    default: {
      break;
    }
  }

  WorkerThread ctrl_param_thread(ctrl_params_task);
  WorkerThread publisher_group_thread(publisher_group_task);

  WorkerThread ident_thread(ident_task);
  if (diag1650x_task) {
    WorkerThread diag1650x_thread(diag1650x_task);
    diag1650x_thread.join();
  }

  if (diag1657x_task) {
    WorkerThread diag1657x_thread(diag1657x_task);
    diag1657x_thread.join();
  }

  ctrl_param_thread.join();
  publisher_group_thread.join();
  ident_thread.join();

  delete ctrl_params;
  delete accel_gyro_publisher;
  delete vel_ang_publisher;
  delete imu_std_publisher;
  delete full_data_publisher;
  delete ident_publisher;

  if (diag1650x_publisher) delete diag1650x_publisher;
  if (diag1657x_publisher) delete diag1657x_publisher;

  rclcpp::shutdown();

  return 0;
}
