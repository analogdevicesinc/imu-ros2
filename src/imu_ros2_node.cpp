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
#include "imu_ros2/imu_16505_diag_data_provider.h"
#include "imu_ros2/imu_16505_diag_ros_publisher.h"
#include "imu_ros2/imu_1657x_diag_data_provider.h"
#include "imu_ros2/imu_1657x_diag_ros_publisher.h"
#include "imu_ros2/imu_control_parameters.h"
#include "imu_ros2/imu_data_provider.h"
#include "imu_ros2/imu_full_measured_data_provider.h"
#include "imu_ros2/imu_full_measured_data_ros_publisher.h"
#include "imu_ros2/imu_identification_data_provider.h"
#include "imu_ros2/imu_identification_ros_publisher.h"
#include "imu_ros2/imu_ros_publisher.h"
#include "imu_ros2/setting_declarations.h"
#include "imu_ros2/velangtemp_data_provider.h"
#include "imu_ros2/velangtemp_ros_publisher.h"
#include "imu_ros2/worker_thread.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

void declareParameters(std::shared_ptr<rclcpp::Node> & node)
{
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description =
    "measured_data_topic_selection values:\n \
    0: measured data is published on /accelgyrotempdata topic \n \
    1: measured data is published on /velangtempdata topic \n \
    2: measured data is published on /imu topic \n \
    3: measured data is published on /imufullmeasureddata topic (default)";
  node->declare_parameter("measured_data_topic_selection", FULL_MEASURED_DATA, param_desc);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> imu_node = rclcpp::Node::make_shared("imu_ros2_node");
  declareParameters(imu_node);

  std::thread::id this_id = std::this_thread::get_id();
  std::cout << "mainthread " << this_id << " running...\n";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp_main"), "running: '%d'", this_id);

  ImuControlParameters * ctrl_params = new ImuControlParameters(imu_node);
  RosTask * ctrl_params_task = dynamic_cast<RosTask *>(ctrl_params);

  AccelGyroTempDataProviderInterface * accel_gyro_data_provider = new AccelGyroTempDataProvider();
  AccelGyroTempRosPublisherInterface * accel_gyro_publisher =
    new AccelGyroTempRosPublisher(imu_node);
  accel_gyro_publisher->setMessageProvider(accel_gyro_data_provider);
  RosTask * accel_gyro_task = dynamic_cast<RosTask *>(accel_gyro_publisher);

  VelAngTempDataProviderInterface * vel_ang_data_provider = new VelAngTempDataProvider();
  VelAngTempRosPublisherInterface * vel_ang_publisher = new VelAngTempRosPublisher(imu_node);
  vel_ang_publisher->setMessageProvider(vel_ang_data_provider);
  RosTask * vel_ang_task = dynamic_cast<RosTask *>(vel_ang_publisher);

  ImuDataProviderInterface * imu_std_data_provider = new ImuDataProvider();
  ImuRosPublisherInterface * imu_std_publisher = new ImuRosPublisher(imu_node);
  imu_std_publisher->setMessageProvider(imu_std_data_provider);
  RosTask * imu_std_task = dynamic_cast<RosTask *>(imu_std_publisher);

  ImuFullMeasuredDataProviderInterface * full_data_provider = new ImuFullMeasuredDataProvider();
  ImuFullMeasuredDataRosPublisherInterface * full_data_publisher =
    new ImuFullMeasuredDataRosPublisher(imu_node);
  full_data_publisher->setMessageProvider(full_data_provider);
  RosTask * full_data_task = dynamic_cast<RosTask *>(full_data_publisher);

  ImuIdentificationDataProviderInterface * ident_data_provider =
    new ImuIdentificationDataProvider();
  ImuIdentificationRosPublisherInterface * ident_publisher =
    new ImuIdentificationRosPublisher(imu_node);
  ident_publisher->setMessageProvider(ident_data_provider);
  RosTask * ident_task = dynamic_cast<RosTask *>(ident_publisher);

  Imu16505DiagDataProviderInterface * diag16505_data_provider = nullptr;
  Imu16505DiagRosPublisherInterface * diag16505_publisher = nullptr;
  RosTask * diag16505_task = nullptr;

  Imu1657xDiagDataProviderInterface * diag1657x_data_provider = nullptr;
  Imu1657xDiagRosPublisherInterface * diag1657x_publisher = nullptr;
  RosTask * diag1657x_task = nullptr;
  switch (IIOWrapper::s_device_name_enum) {
    case IIODeviceName::ADIS16505:
      diag16505_data_provider = new Imu16505DiagDataProvider();
      diag16505_publisher = new Imu16505DiagRosPublisher(imu_node);
      diag16505_publisher->setMessageProvider(diag16505_data_provider);

      diag16505_task = dynamic_cast<RosTask *>(diag16505_publisher);
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
  WorkerThread accel_gyro_thread(accel_gyro_task);
  WorkerThread vel_ang_thread(vel_ang_task);
  WorkerThread imu_std_thread(imu_std_task);
  WorkerThread full_data_thread(full_data_task);
  WorkerThread ident_thread(ident_task);
  if (diag16505_task) {
    WorkerThread diag16505_thread(diag16505_task);
    diag16505_thread.join();
  }

  if (diag1657x_task) {
    WorkerThread diag1657x_thread(diag1657x_task);
    diag1657x_thread.join();
  }

  ctrl_param_thread.join();
  accel_gyro_thread.join();
  vel_ang_thread.join();
  imu_std_thread.join();
  full_data_thread.join();
  ident_thread.join();

  delete ctrl_params;
  delete accel_gyro_publisher;
  delete vel_ang_publisher;
  delete imu_std_publisher;
  delete full_data_publisher;
  delete ident_publisher;

  if (diag16505_publisher) delete diag16505_publisher;
  if (diag1657x_publisher) delete diag1657x_publisher;

  rclcpp::shutdown();

  return 0;
}
