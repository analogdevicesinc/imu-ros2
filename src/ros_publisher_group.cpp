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
#include "imu_ros2/accelgyrotemp_ros_publisher_interface.h"
#include "imu_ros2/velangtemp_ros_publisher_interface.h"
#include "imu_ros2/imu_ros_publisher_interface.h"
#include "imu_ros2/imu_full_measured_data_ros_publisher_interface.h"
#include "imu_ros2/setting_declarations.h"

#include <chrono>
#include <thread>

RosPublisherGroup::RosPublisherGroup(std::shared_ptr<rclcpp::Node> & node)
{
  init(node);
}

RosPublisherGroup::~RosPublisherGroup() {  }

void RosPublisherGroup::init(std::shared_ptr<rclcpp::Node> & node)
{
  m_node = node;
}

void RosPublisherGroup::setAccelGyroTempRosPublisher(AccelGyroTempRosPublisherInterface * accelGyroTempRosPublisher)
{
  m_accelGyroTempRosPublisher = accelGyroTempRosPublisher;
}

void RosPublisherGroup::setVelAngTempRosPublisher(VelAngTempRosPublisherInterface * velAngTempRosPublisher)
{
  m_velAngTempRosPublisher = velAngTempRosPublisher;
}

void RosPublisherGroup::setImuRosPublisher(ImuRosPublisherInterface * imuRosPublisher)
{
  m_imuRosPublisher = imuRosPublisher;
}

void RosPublisherGroup::setImuFullMeasuredDataRosPublisher(ImuFullMeasuredDataRosPublisherInterface * imuFullMeasuredDataRosPublisher)
{
  m_imuFullMeasuredDataRosPublisher = imuFullMeasuredDataRosPublisher;
}


void RosPublisherGroup::run()
{
  std::thread::id this_id = std::this_thread::get_id();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp_rosPublisherGroup"), "startThread: '%d'", this_id);

  bool bufferedDataEnabledAccel = false;
  bool bufferedDataEnabledVelAng = false;
  bool bufferedDataEnabledImu = false;
  int32_t measuredDataSelection = ACCEL_GYRO_BUFFERED_DATA;

  while (rclcpp::ok()) {

    measuredDataSelection =
        m_node->get_parameter("measured_data_topic_selection").get_parameter_value().get<int32_t>();

    switch (measuredDataSelection) {
    case ACCEL_GYRO_BUFFERED_DATA:
      if (!bufferedDataEnabledAccel)
        if (m_accelGyroTempRosPublisher->enableBufferedDataOutput()) bufferedDataEnabledAccel = true;

      m_accelGyroTempRosPublisher->run();
      bufferedDataEnabledVelAng = false;
      bufferedDataEnabledImu = false;
      break;
    case DELTAVEL_DELTAANG_BUFFERED_DATA:
      if (!bufferedDataEnabledVelAng)
        if (m_velAngTempRosPublisher->enableBufferedDataOutput()) bufferedDataEnabledVelAng = true;

      m_velAngTempRosPublisher->run();
      bufferedDataEnabledAccel = false;
      bufferedDataEnabledImu = false;
      break;
    case IMU_STD_MSG_DATA:
      if (!bufferedDataEnabledImu)
        if (m_imuRosPublisher->enableBufferedDataOutput()) bufferedDataEnabledImu = true;
      m_imuRosPublisher->run();
      bufferedDataEnabledAccel = false;
      bufferedDataEnabledVelAng = false;
      break;
    case FULL_MEASURED_DATA:
      m_imuFullMeasuredDataRosPublisher->run();
      bufferedDataEnabledAccel = false;
      bufferedDataEnabledVelAng = false;
      bufferedDataEnabledImu = false;
      break;
    default: {
      bufferedDataEnabledAccel = false;
      bufferedDataEnabledVelAng = false;
      bufferedDataEnabledImu = false;
      break;
    }
    }

  }

  this_id = std::this_thread::get_id();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp_rosPublisherGroup"), "endThread: '%d'", this_id);
}
