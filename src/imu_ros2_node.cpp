/*******************************************************************************
 *   @file   imu_ros2_node.cpp
 *   @brief  Implementation for IMU node.
 *   @author Vasile Holonec (Vasile.Holonec@analog.com)
*******************************************************************************/
// Copyright 2023 Analog Devices, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "adi_imu/accelgyrotemp_data_provider.h"
#include "adi_imu/accelgyrotemp_ros_publisher.h"
#include "adi_imu/adis_device_factory.h"
#include "adi_imu/adis_register_map.h"
#include "adi_imu/imu_control_parameters.h"
#include "adi_imu/imu_covariance_factory.h"  // New covariance feature
#include "adi_imu/imu_data_provider.h"
#include "adi_imu/imu_diag_data_provider.h"
#include "adi_imu/imu_diag_ros_publisher.h"
#include "adi_imu/imu_diag_ros_publisher_factory.h"
#include "adi_imu/imu_full_measured_data_provider.h"
#include "adi_imu/imu_full_measured_data_ros_publisher.h"
#include "adi_imu/imu_identification_data_provider.h"
#include "adi_imu/imu_identification_ros_publisher.h"
#include "adi_imu/imu_ros_publisher.h"
#include "adi_imu/ros_publisher_group.h"
#include "adi_imu/ros_publisher_group_interface.h"
#include "adi_imu/velangtemp_data_provider.h"
#include "adi_imu/velangtemp_ros_publisher.h"
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

  auto iio_context_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  iio_context_param_desc.description =
    "\nDefault value is \'local:\', to be used when the adi_imu node is running locally."
    "\nIf the adi_imu node is running remotely, please use \'ip:rpi_ip_address\',";

  imu_node->declare_parameter("iio_context_string", "local:", iio_context_param_desc);

  auto imu_device_name_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  imu_device_name_param_desc.description =
    "\nSet the IMU device name from the list of supported devices, e.g. 'adis16545-3'.";
  imu_node->declare_parameter("imu_device_name", "unknown", imu_device_name_param_desc);

  auto imu_device_name =
    imu_node->get_parameter("imu_device_name").get_parameter_value().get<std::string>();
  auto device_descriptor = adi_imu::ADISDeviceFactory::make(imu_device_name);

  auto diag_data_enable_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  diag_data_enable_param_desc.description =
    "\nWhether to enable the publisher of IMU diagnostic data."
    "\nWhen enabled, the node will continuously poll diagnostic registers to read and publish "
    "device status.";
  imu_node->declare_parameter("diag_data_enable", true, diag_data_enable_param_desc);

  auto diag_data_enable =
    imu_node->get_parameter("diag_data_enable").get_parameter_value().get<bool>();

  auto ident_data_enable_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  ident_data_enable_param_desc.description =
    "\nWhether to enable the publisher of IMU identification data."
    "\nWhen enabled, the node will read and publish device identification";
  imu_node->declare_parameter("ident_data_enable", true, ident_data_enable_param_desc);

  auto ident_data_enable =
    imu_node->get_parameter("ident_data_enable").get_parameter_value().get<bool>();

  auto frame_id_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  frame_id_param_desc.description =
    "\nThe TF frame ID for the IMU sensor (e.g., source_x/imu). Useful when multiple IMUs are "
    "used.";
  imu_node->declare_parameter("frame_id", "imu", frame_id_param_desc);

  auto frame_id = imu_node->get_parameter("frame_id").get_parameter_value().get<std::string>();

  /* First make sure IIO context is available */
  std::string context =
    imu_node->get_parameter("iio_context_string").get_parameter_value().get<std::string>();
  adi_imu::IIOWrapper m_iio_wrapper;
  m_iio_wrapper.setDeviceDescriptor(device_descriptor);
  ret = m_iio_wrapper.createContext(context.c_str());

  if (ret) {
    std::runtime_error("Error IIO context, exiting ROS2 node");
    rclcpp::shutdown();
    return 0;
  }
  adi_imu::ImuControlParameters * ctrl_params =
    new adi_imu::ImuControlParameters(imu_node, device_descriptor);

  adi_imu::AccelGyroTempDataProviderInterface * accel_gyro_data_provider =
    new adi_imu::AccelGyroTempDataProvider();
  adi_imu::AccelGyroTempRosPublisherInterface * accel_gyro_publisher =
    new adi_imu::AccelGyroTempRosPublisher(imu_node);
  accel_gyro_publisher->setMessageProvider(accel_gyro_data_provider);

  adi_imu::ImuDataProviderInterface * imu_std_data_provider = new adi_imu::ImuDataProvider();
  imu_std_data_provider->setFrameId(frame_id);
  adi_imu::ImuRosPublisherInterface * imu_std_publisher = new adi_imu::ImuRosPublisher(imu_node);
  imu_std_publisher->setMessageProvider(imu_std_data_provider);

  //==============Covariance provider setup================
  // Create covariance provider from ROS2 parameters.
  // The factory declares parameters and creates the appropriate provider
  std::unique_ptr<adi_imu::ImuCovarianceInterface> covariance_provider =
    adi_imu::ImuCovarianceFactory::createFromParameters(imu_node);

  // Inject covariance provider into IMU data provider if enabled
  if (covariance_provider) {
    imu_std_data_provider->setCovarianceProvider(covariance_provider.get());
    RCLCPP_INFO(imu_node->get_logger(), "Covariance provider attached to ImuDataProvider.");
  } else {
    RCLCPP_INFO(
      imu_node->get_logger(), "Covariance computation disabled (covariance.enable = false).");
  }
  //=======================================================

  adi_imu::VelAngTempDataProviderInterface * vel_ang_data_provider = nullptr;
  adi_imu::VelAngTempRosPublisherInterface * vel_ang_publisher = nullptr;

  if (device_descriptor->has(adi_imu::ADISRegister::HAS_DELTA_BURST)) {
    vel_ang_data_provider = new adi_imu::VelAngTempDataProvider();
    vel_ang_publisher = new adi_imu::VelAngTempRosPublisher(imu_node);
    vel_ang_publisher->setMessageProvider(vel_ang_data_provider);
  } else {
    RCLCPP_INFO(
      imu_node->get_logger(),
      "Device does not support delta burst, skipping VelAngTemp publisher.");
  }

  adi_imu::ImuFullMeasuredDataProviderInterface * full_data_provider =
    new adi_imu::ImuFullMeasuredDataProvider();
  adi_imu::ImuFullMeasuredDataRosPublisherInterface * full_data_publisher =
    new adi_imu::ImuFullMeasuredDataRosPublisher(imu_node);
  full_data_publisher->setMessageProvider(full_data_provider);

  adi_imu::RosPublisherGroupInterface * publisher_group = new adi_imu::RosPublisherGroup(imu_node);
  publisher_group->setAccelGyroTempRosPublisher(accel_gyro_publisher);
  if (vel_ang_publisher != nullptr) {
    publisher_group->setVelAngTempRosPublisher(vel_ang_publisher);
  } else {
    RCLCPP_INFO(imu_node->get_logger(), "VelAngTemp publisher is null, skipping.");
  }

  publisher_group->setImuRosPublisher(imu_std_publisher);
  publisher_group->setImuFullMeasuredDataRosPublisher(full_data_publisher);
  publisher_group->setImuControlParameters(ctrl_params);

  adi_imu::RosTask * publisher_group_task = dynamic_cast<adi_imu::RosTask *>(publisher_group);

  adi_imu::ImuIdentificationDataProviderInterface * ident_data_provider = nullptr;
  adi_imu::ImuIdentificationRosPublisherInterface * ident_publisher = nullptr;
  adi_imu::RosTask * ident_task = nullptr;
  if (ident_data_enable) {
    ident_data_provider = new adi_imu::ImuIdentificationDataProvider();
    ident_publisher = new adi_imu::ImuIdentificationRosPublisher(imu_node);
    ident_publisher->setMessageProvider(ident_data_provider);
    ident_task = dynamic_cast<adi_imu::RosTask *>(ident_publisher);
  } else {
    RCLCPP_INFO(imu_node->get_logger(), "Identification data publishing is disabled.");
  }

  adi_imu::ImuDiagDataProviderInterface * diag_data_provider = nullptr;
  std::unique_ptr<adi_imu::ImuDiagRosPublisherInterface> diag_publisher = nullptr;
  adi_imu::RosTask * diag_task = nullptr;
  if (diag_data_enable) {
    diag_data_provider = new adi_imu::ImuDiagDataProvider();

    try {
      diag_publisher = adi_imu::ImuDiagPublisherFactory::make(device_descriptor, imu_node);
      diag_publisher->setMessageProvider(diag_data_provider);
      diag_publisher->setDeviceDescriptor(device_descriptor);

      diag_task = dynamic_cast<adi_imu::RosTask *>(diag_publisher.get());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(imu_node->get_logger(), "Failed to create diag publisher: %s", e.what());
    }
  } else {
    RCLCPP_INFO(imu_node->get_logger(), "Diagnostic data publishing is disabled.");
  }

  adi_imu::WorkerThread publisher_group_thread(publisher_group_task);
  std::unique_ptr<adi_imu::WorkerThread> ident_thread = nullptr;
  std::unique_ptr<adi_imu::WorkerThread> diag_thread = nullptr;
  if (ident_task != nullptr) {
    ident_thread = std::make_unique<adi_imu::WorkerThread>(ident_task);
  }
  if (diag_task != nullptr) {
    diag_thread = std::make_unique<adi_imu::WorkerThread>(diag_task);
  }

  // Join all threads after they have started
  if (ident_thread) {
    ident_thread->join();
  }
  if (diag_thread) {
    diag_thread->join();
  }
  publisher_group_thread.join();

  // Cleanup resources. Deleting publishers also deletes their respective providers
  if (vel_ang_publisher != nullptr) {
    delete vel_ang_publisher;
  }

  if (ident_publisher != nullptr) {
    delete ident_publisher;
  }

  delete accel_gyro_publisher;
  delete imu_std_publisher;
  delete full_data_publisher;

  if (publisher_group != nullptr) {
    delete publisher_group;
  }

  return 0;
}
