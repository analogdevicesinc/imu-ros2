/***************************************************************************/ /**
 *   @file   adi_imu_node.cpp
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

#include "adi_imu/accelgyrotemp_data_provider.h"
#include "adi_imu/accelgyrotemp_ros_publisher.h"
#include "adi_imu/imu_control_parameters.h"
#include "adi_imu/imu_data_provider.h"
#include "adi_imu/imu_diag_data_provider.h"
#include "adi_imu/imu_diag_ros_publisher.h"
#include "adi_imu/imu_full_measured_data_provider.h"
#include "adi_imu/imu_full_measured_data_ros_publisher.h"
#include "adi_imu/imu_identification_data_provider.h"
#include "adi_imu/imu_identification_ros_publisher.h"
#include "adi_imu/imu_ros_publisher.h"
#include "adi_imu/ros_publisher_group.h"
#include "adi_imu/ros_publisher_group_interface.h"
#ifdef ADIS_HAS_DELTA_BURST
#include "adi_imu/velangtemp_data_provider.h"
#include "adi_imu/velangtemp_ros_publisher.h"
#endif

#include "adi_imu/worker_thread.h"
#include "rclcpp/rclcpp.hpp"

/**
   * @brief main function to run imu-ros2
   * @param argc argument count
   * @param argv arguments value array
   * @return Should not return if successful (will run continuously), error code
   * otherwise.
   */
int main(int argc, char * argv[])
{
  int ret;
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> imu_node = rclcpp::Node::make_shared("adi_imu_node");

  std::thread::id this_id = std::this_thread::get_id();
  std::cout << "mainthread " << this_id << " running...\n";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp_main"), "running: main thread");

  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description =
    "\nDefault value is \'local:\', to be used when the adi_imu node is running locally."
    "\nIf the adi_imu node is running remotely, please use \'ip:rpi_ip_address\',";

  imu_node->declare_parameter("iio_context_string", "local:", param_desc);

  /* First make sure IIO context is available */
  std::string context =
    imu_node->get_parameter("iio_context_string").get_parameter_value().get<std::string>();
  IIOWrapper m_iio_wrapper;
  ret = m_iio_wrapper.createContext(context.c_str());

  if (ret) {
    std::runtime_error("Error IIO context, exiting ROS2 node");
    rclcpp::shutdown();
    return 0;
  }
  ImuControlParameters * ctrl_params = new ImuControlParameters(imu_node);

  AccelGyroTempDataProviderInterface * accel_gyro_data_provider = new AccelGyroTempDataProvider();
  AccelGyroTempRosPublisherInterface * accel_gyro_publisher =
    new AccelGyroTempRosPublisher(imu_node);
  accel_gyro_publisher->setMessageProvider(accel_gyro_data_provider);

  ImuDataProviderInterface * imu_std_data_provider = new ImuDataProvider();
  ImuRosPublisherInterface * imu_std_publisher = new ImuRosPublisher(imu_node);
  imu_std_publisher->setMessageProvider(imu_std_data_provider);

#ifdef ADIS_HAS_DELTA_BURST
  VelAngTempDataProviderInterface * vel_ang_data_provider = new VelAngTempDataProvider();
  VelAngTempRosPublisherInterface * vel_ang_publisher = new VelAngTempRosPublisher(imu_node);
  vel_ang_publisher->setMessageProvider(vel_ang_data_provider);
#endif

  ImuFullMeasuredDataProviderInterface * full_data_provider = new ImuFullMeasuredDataProvider();
  ImuFullMeasuredDataRosPublisherInterface * full_data_publisher =
    new ImuFullMeasuredDataRosPublisher(imu_node);
  full_data_publisher->setMessageProvider(full_data_provider);

  RosPublisherGroupInterface * publisher_group = new RosPublisherGroup(imu_node);
  publisher_group->setAccelGyroTempRosPublisher(accel_gyro_publisher);
#ifdef ADIS_HAS_DELTA_BURST
  publisher_group->setVelAngTempRosPublisher(vel_ang_publisher);
#endif
  publisher_group->setImuRosPublisher(imu_std_publisher);
  publisher_group->setImuFullMeasuredDataRosPublisher(full_data_publisher);
  publisher_group->setImuControlParameters(ctrl_params);

  RosTask * publisher_group_task = dynamic_cast<RosTask *>(publisher_group);

  ImuIdentificationDataProviderInterface * ident_data_provider =
    new ImuIdentificationDataProvider();
  ImuIdentificationRosPublisherInterface * ident_publisher =
    new ImuIdentificationRosPublisher(imu_node);
  ident_publisher->setMessageProvider(ident_data_provider);
  RosTask * ident_task = dynamic_cast<RosTask *>(ident_publisher);

  ImuDiagDataProviderInterface * diag_data_provider = nullptr;
  ImuDiagRosPublisherInterface * diag_publisher = nullptr;
  RosTask * diag_task = nullptr;

  diag_data_provider = new ImuDiagDataProvider();
  diag_publisher = new ImuDiagRosPublisher(imu_node);
  diag_publisher->setMessageProvider(diag_data_provider);

  diag_task = dynamic_cast<RosTask *>(diag_publisher);

  WorkerThread publisher_group_thread(publisher_group_task);
  WorkerThread ident_thread(ident_task);
  WorkerThread diag_thread(diag_task);

  diag_thread.join();
  ident_thread.join();
  publisher_group_thread.join();

  delete ctrl_params;
  delete accel_gyro_publisher;
#ifdef ADIS_HAS_DELTA_BURST
  delete vel_ang_publisher;
#endif
  delete imu_std_publisher;
  delete full_data_publisher;
  delete ident_publisher;
  delete diag_publisher;

  rclcpp::shutdown();

  return 0;
}
