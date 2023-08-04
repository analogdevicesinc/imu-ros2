/*******************************************************************************
 *   @file   ros_publisher_group.cpp
 *   @brief  Implementation for group of publishers.
 *   @author Vasile Holonec (Vasile.Holonec@analog.com)
 *******************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
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
 ******************************************************************************/

#include "imu_ros2/ros_publisher_group.h"

#include <chrono>
#include <thread>

#include "imu_ros2/accelgyrotemp_ros_publisher_interface.h"
#include "imu_ros2/imu_full_measured_data_ros_publisher_interface.h"
#include "imu_ros2/imu_ros_publisher_interface.h"
#include "imu_ros2/setting_declarations.h"
#include "imu_ros2/velangtemp_ros_publisher_interface.h"

RosPublisherGroup::RosPublisherGroup(std::shared_ptr<rclcpp::Node> & node) { init(node); }

RosPublisherGroup::~RosPublisherGroup() {}

void RosPublisherGroup::init(std::shared_ptr<rclcpp::Node> & node) { m_node = node; }

void RosPublisherGroup::setAccelGyroTempRosPublisher(
  AccelGyroTempRosPublisherInterface * accelGyroTempRosPublisher)
{
  m_accelGyroTempRosPublisher = accelGyroTempRosPublisher;
}

void RosPublisherGroup::setVelAngTempRosPublisher(
  VelAngTempRosPublisherInterface * velAngTempRosPublisher)
{
  m_velAngTempRosPublisher = velAngTempRosPublisher;
}

void RosPublisherGroup::setImuRosPublisher(ImuRosPublisherInterface * imuRosPublisher)
{
  m_imuRosPublisher = imuRosPublisher;
}

void RosPublisherGroup::setImuFullMeasuredDataRosPublisher(
  ImuFullMeasuredDataRosPublisherInterface * imuFullMeasuredDataRosPublisher)
{
  m_imuFullMeasuredDataRosPublisher = imuFullMeasuredDataRosPublisher;
}

void RosPublisherGroup::run()
{
  std::thread::id this_id = std::this_thread::get_id();
  std::cout << "thread " << this_id << " started...\n";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp_rosPublisherGroup"), "startThread: RosPublisherGroup");

  int32_t measuredDataSelection;
  int32_t previousSelection = -1;

  while (rclcpp::ok()) {
    measuredDataSelection =
      m_node->get_parameter("measured_data_topic_selection").get_parameter_value().get<int32_t>();

    switch (measuredDataSelection) {
      case ACCEL_GYRO_BUFFERED_DATA:
        if (previousSelection != ACCEL_GYRO_BUFFERED_DATA)
          if (m_imuRosPublisher->configureBufferedDataOutput())
            previousSelection = ACCEL_GYRO_BUFFERED_DATA;
        m_accelGyroTempRosPublisher->run();
        break;
      case DELTAVEL_DELTAANG_BUFFERED_DATA:
        if (previousSelection != DELTAVEL_DELTAANG_BUFFERED_DATA)
          if (m_velAngTempRosPublisher->configureBufferedDataOutput())
            previousSelection = DELTAVEL_DELTAANG_BUFFERED_DATA;
        m_velAngTempRosPublisher->run();
        break;
      case IMU_STD_MSG_DATA:
        if (previousSelection != IMU_STD_MSG_DATA)
          if (m_imuRosPublisher->configureBufferedDataOutput())
            previousSelection = IMU_STD_MSG_DATA;
        m_imuRosPublisher->run();
        break;
      case FULL_MEASURED_DATA:
        if (previousSelection != FULL_MEASURED_DATA)
          if (m_imuFullMeasuredDataRosPublisher->configureBufferedDataOutput())
            previousSelection = FULL_MEASURED_DATA;
        m_imuFullMeasuredDataRosPublisher->run();
        break;
      default: {
        break;
      }
    }
  }

  this_id = std::this_thread::get_id();
  std::cout << "thread " << this_id << " ended...\n";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp_rosPublisherGroup"), "endThread: RosPublisherGroup");
}
