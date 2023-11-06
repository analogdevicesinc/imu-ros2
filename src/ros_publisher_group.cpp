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
#include "imu_ros2/imu_control_parameters.h"
#include "imu_ros2/imu_full_measured_data_ros_publisher_interface.h"
#include "imu_ros2/imu_ros_publisher_interface.h"
#ifdef ADIS_HAS_DELTA_BURST
#include "imu_ros2/velangtemp_ros_publisher_interface.h"
#endif

RosPublisherGroup::RosPublisherGroup(std::shared_ptr<rclcpp::Node> & node) { m_node = node; }

RosPublisherGroup::~RosPublisherGroup() {}

void RosPublisherGroup::setAccelGyroTempRosPublisher(
  AccelGyroTempRosPublisherInterface * accelGyroTempRosPublisher)
{
  m_accelGyroTempRosPublisher = accelGyroTempRosPublisher;
}

#ifdef ADIS_HAS_DELTA_BURST
void RosPublisherGroup::setVelAngTempRosPublisher(
  VelAngTempRosPublisherInterface * velAngTempRosPublisher)
{
  m_velAngTempRosPublisher = velAngTempRosPublisher;
}
#endif

void RosPublisherGroup::setImuRosPublisher(ImuRosPublisherInterface * imuRosPublisher)
{
  m_imuRosPublisher = imuRosPublisher;
}

void RosPublisherGroup::setImuFullMeasuredDataRosPublisher(
  ImuFullMeasuredDataRosPublisherInterface * imuFullMeasuredDataRosPublisher)
{
  m_imuFullMeasuredDataRosPublisher = imuFullMeasuredDataRosPublisher;
}

void RosPublisherGroup::setImuControlParameters(ImuControlParameters * imuControlParameters)
{
  m_imuControlParameters = imuControlParameters;
}

void RosPublisherGroup::run()
{
  std::thread::id this_id = std::this_thread::get_id();
  std::cout << "thread " << this_id << " started...\n";
  RCLCPP_INFO(rclcpp::get_logger("ros_publisher_group"), "startThread: RosPublisherGroup");

  int32_t measuredDataSelection;

  while (rclcpp::ok()) {
    measuredDataSelection =
      m_node->get_parameter("measured_data_topic_selection").get_parameter_value().get<int32_t>();

    switch (measuredDataSelection) {
      case ACCEL_GYRO_BUFFERED_DATA:
        m_accelGyroTempRosPublisher->publish();
        break;
#ifdef ADIS_HAS_DELTA_BURST
      case DELTAVEL_DELTAANG_BUFFERED_DATA:
        m_velAngTempRosPublisher->publish();
        break;
#endif
      case IMU_STD_MSG_DATA:
        m_imuRosPublisher->publish();
        break;
      case FULL_MEASURED_DATA:
        m_imuFullMeasuredDataRosPublisher->publish();
        break;
      default: {
        RCLCPP_INFO(
          rclcpp::get_logger("ros_publisher_group"),
          "Invalid value for measured_data_topic_selection parameter.");
        break;
      }
    }

    m_imuControlParameters->handleControlParams();

    rclcpp::spin_some(m_node);
  }

  this_id = std::this_thread::get_id();
  std::cout << "thread " << this_id << " ended...\n";
  RCLCPP_INFO(rclcpp::get_logger("ros_publisher_group"), "endThread: RosPublisherGroup");
}
