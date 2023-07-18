/***************************************************************************/ /**
 *   @file   iio_wrapper.cpp
 *   @brief  Implementation for iio wrapper library
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

#include "imu_ros2/iio_wrapper.h"

#include <unistd.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

#define MAX_NO_OF_SAMPLES 4000

enum
{
  CHAN_GYRO_X,
  CHAN_GYRO_Y,
  CHAN_GYRO_Z,
  CHAN_ACCEL_X,
  CHAN_ACCEL_Y,
  CHAN_ACCEL_Z,
  CHAN_TEMP,
  CHAN_DELTA_ANGL_X,
  CHAN_DELTA_ANGL_Y,
  CHAN_DELTA_ANGL_Z,
  CHAN_DELTA_VEL_X,
  CHAN_DELTA_VEL_Y,
  CHAN_DELTA_VEL_Z,
  CHAN_DATA_TIMESTAMP,
  NO_OF_CHANS,
};

struct iio_context * IIOWrapper::m_local_context = nullptr;

struct iio_device * IIOWrapper::m_dev = nullptr;
struct iio_device * IIOWrapper::m_devtrigger = nullptr;
struct iio_buffer * IIOWrapper::m_device_buffer = nullptr;

struct iio_channel * IIOWrapper::m_channel_accel_x = nullptr;
struct iio_channel * IIOWrapper::m_channel_accel_y = nullptr;
struct iio_channel * IIOWrapper::m_channel_accel_z = nullptr;
struct iio_channel * IIOWrapper::m_channel_anglvel_x = nullptr;
struct iio_channel * IIOWrapper::m_channel_anglvel_y = nullptr;
struct iio_channel * IIOWrapper::m_channel_anglvel_z = nullptr;
struct iio_channel * IIOWrapper::m_channel_rot_x = nullptr;
struct iio_channel * IIOWrapper::m_channel_rot_y = nullptr;
struct iio_channel * IIOWrapper::m_channel_rot_z = nullptr;
struct iio_channel * IIOWrapper::m_channel_velocity_x = nullptr;
struct iio_channel * IIOWrapper::m_channel_velocity_y = nullptr;
struct iio_channel * IIOWrapper::m_channel_velocity_z = nullptr;
struct iio_channel * IIOWrapper::m_channel_temp = nullptr;
struct iio_channel * IIOWrapper::m_channel_timestamp = nullptr;

double IIOWrapper::m_scale_accel_x = 0;
double IIOWrapper::m_scale_accel_y = 0;
double IIOWrapper::m_scale_accel_z = 0;
double IIOWrapper::m_scale_angvel_x = 0;
double IIOWrapper::m_scale_angvel_y = 0;
double IIOWrapper::m_scale_angvel_z = 0;
double IIOWrapper::m_scale_rot_x = 0;
double IIOWrapper::m_scale_rot_y = 0;
double IIOWrapper::m_scale_rot_z = 0;
double IIOWrapper::m_scale_velocity_x = 0;
double IIOWrapper::m_scale_velocity_y = 0;
double IIOWrapper::m_scale_velocity_z = 0;
double IIOWrapper::m_scale_temp = 0;

uint32_t write_buffer_idx = 0;
uint32_t read_buffer_idx = MAX_NO_OF_SAMPLES;
uint32_t buffered_data[NO_OF_CHANS + 1][MAX_NO_OF_SAMPLES];
double samp_freq = 2000.0;
uint32_t no_of_samp = 2000;

std::string IIOWrapper::s_device_name;
std::string IIOWrapper::s_device_trigger_name;
IIODeviceName IIOWrapper::s_device_name_enum = IIODeviceName::ADIS1657X;

IIOWrapper::IIOWrapper()
{
  // create context
  if (m_local_context == nullptr) {
    m_local_context = iio_create_local_context();
    if (!m_local_context) throw std::runtime_error("Exception: local context is null");

    int count = iio_context_get_devices_count(m_local_context);
    if (count > 0) {
      struct iio_device * dev = iio_context_get_device(m_local_context, 0);
      const char * name = iio_device_get_name(dev);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp_iiowrapper"), "device name: %s", name);

      IIOWrapper::s_device_name = std::string(name);

      std::list<std::string> adis1650x_x{"adis16505-1", "adis16505-2", "adis16505-3"};

      bool found =
        (std::find(adis1650x_x.begin(), adis1650x_x.end(), IIOWrapper::s_device_name) !=
         adis1650x_x.end());
      if (found) {
        IIOWrapper::s_device_name_enum = IIODeviceName::ADIS1650X;
      }

      std::list<std::string> adis1657x{"adis16575-2", "adis16575-3", "adis16576-2",
                                       "adis16576-3", "adis16577-2", "adis16577-3"};

      bool found7x =
        (std::find(adis1657x.begin(), adis1657x.end(), IIOWrapper::s_device_name) !=
         adis1657x.end());
      if (found7x) {
        IIOWrapper::s_device_name_enum = IIODeviceName::ADIS1657X;
      }

      if (!found && !found7x)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp_iiowrapper"), "device not found");
    } else
      throw std::runtime_error("Exception: no device found");
  }

  if (m_dev == nullptr) {
    m_dev = iio_context_find_device(m_local_context, IIOWrapper::s_device_name.c_str());
    if (!m_dev) throw std::runtime_error("Exception: device data is null");
  }

  if (m_devtrigger == nullptr) {
    std::string triggnerName = IIOWrapper::s_device_name + "-dev0";
    m_devtrigger = iio_context_find_device(m_local_context, triggnerName.c_str());
    if (!m_devtrigger) throw std::runtime_error("Exception: device trigger data is null");
  }

  iio_device_set_trigger(m_dev, m_devtrigger);

  if (m_channel_accel_x == nullptr)
    m_channel_accel_x = iio_device_find_channel(m_dev, "accel_x", false);
  if (m_channel_accel_y == nullptr)
    m_channel_accel_y = iio_device_find_channel(m_dev, "accel_y", false);
  if (m_channel_accel_z == nullptr)
    m_channel_accel_z = iio_device_find_channel(m_dev, "accel_z", false);

  if (m_channel_anglvel_x == nullptr)
    m_channel_anglvel_x = iio_device_find_channel(m_dev, "anglvel_x", false);
  if (m_channel_anglvel_y == nullptr)
    m_channel_anglvel_y = iio_device_find_channel(m_dev, "anglvel_y", false);
  if (m_channel_anglvel_z == nullptr)
    m_channel_anglvel_z = iio_device_find_channel(m_dev, "anglvel_z", false);

  if (m_channel_rot_x == nullptr) m_channel_rot_x = iio_device_find_channel(m_dev, "rot_x", false);
  if (m_channel_rot_y == nullptr) m_channel_rot_y = iio_device_find_channel(m_dev, "rot_y", false);
  if (m_channel_rot_z == nullptr) m_channel_rot_z = iio_device_find_channel(m_dev, "rot_z", false);

  if (m_channel_velocity_x == nullptr)
    m_channel_velocity_x = iio_device_find_channel(m_dev, "velocity_x", false);
  if (m_channel_velocity_y == nullptr)
    m_channel_velocity_y = iio_device_find_channel(m_dev, "velocity_y", false);
  if (m_channel_velocity_z == nullptr)
    m_channel_velocity_z = iio_device_find_channel(m_dev, "velocity_z", false);

  if (m_channel_temp == nullptr) m_channel_temp = iio_device_find_channel(m_dev, "temp", false);

  if (m_channel_timestamp == nullptr)
    m_channel_timestamp = iio_device_find_channel(m_dev, "timestamp", false);

  iio_channel_enable(m_channel_accel_x);
  iio_channel_enable(m_channel_accel_y);
  iio_channel_enable(m_channel_accel_z);
  iio_channel_enable(m_channel_anglvel_x);
  iio_channel_enable(m_channel_anglvel_y);
  iio_channel_enable(m_channel_anglvel_z);
  iio_channel_enable(m_channel_rot_x);
  iio_channel_enable(m_channel_rot_y);
  iio_channel_enable(m_channel_rot_z);
  iio_channel_enable(m_channel_velocity_x);
  iio_channel_enable(m_channel_velocity_y);
  iio_channel_enable(m_channel_velocity_z);
  iio_channel_enable(m_channel_temp);
  iio_channel_enable(m_channel_timestamp);

  iio_channel_attr_read_double(m_channel_accel_x, "scale", &m_scale_accel_x);
  iio_channel_attr_read_double(m_channel_accel_y, "scale", &m_scale_accel_y);
  iio_channel_attr_read_double(m_channel_accel_z, "scale", &m_scale_accel_z);
  iio_channel_attr_read_double(m_channel_anglvel_x, "scale", &m_scale_angvel_x);
  iio_channel_attr_read_double(m_channel_anglvel_y, "scale", &m_scale_angvel_y);
  iio_channel_attr_read_double(m_channel_anglvel_z, "scale", &m_scale_angvel_z);
  iio_channel_attr_read_double(m_channel_rot_x, "scale", &m_scale_rot_x);
  iio_channel_attr_read_double(m_channel_rot_y, "scale", &m_scale_rot_y);
  iio_channel_attr_read_double(m_channel_rot_z, "scale", &m_scale_rot_z);
  iio_channel_attr_read_double(m_channel_velocity_x, "scale", &m_scale_velocity_x);
  iio_channel_attr_read_double(m_channel_velocity_y, "scale", &m_scale_velocity_y);
  iio_channel_attr_read_double(m_channel_velocity_z, "scale", &m_scale_velocity_z);
  iio_channel_attr_read_double(m_channel_temp, "scale", &m_scale_temp);
}

IIOWrapper::~IIOWrapper()
{
  if (m_device_buffer != nullptr) {
    iio_buffer_destroy(m_device_buffer);
    m_device_buffer = nullptr;
  }
  if (m_local_context != nullptr) {
    iio_context_destroy(m_local_context);
    m_local_context = nullptr;
  }
}

void IIOWrapper::stopBufferAcquisition()
{
  if (m_device_buffer != nullptr) {
    iio_buffer_destroy(m_device_buffer);
    m_device_buffer = nullptr;
  }
}

static ssize_t demux_sample(const struct iio_channel * chn, void * sample, size_t size, void * d)
{
  uint64_t val;
  iio_channel_convert(chn, &val, sample);

  buffered_data[iio_channel_get_index(chn)][write_buffer_idx] = val;

  if (iio_channel_get_index(chn) == CHAN_DATA_TIMESTAMP) {
    buffered_data[CHAN_DATA_TIMESTAMP + 1][write_buffer_idx] = val >> 32;
    write_buffer_idx++;
  }

  if (write_buffer_idx == no_of_samp) write_buffer_idx = 0;

  return size;
}

bool IIOWrapper::updateBuffer()
{
  ssize_t ret;

  if (m_device_buffer == nullptr) {
    no_of_samp = samp_freq;
    m_device_buffer = iio_device_create_buffer(m_dev, no_of_samp, false);
    if (!m_device_buffer) throw std::runtime_error("Exception: device buffer is null");
  }

  read_buffer_idx++;

  if (read_buffer_idx < no_of_samp) return true;

  ret = iio_buffer_refill(m_device_buffer);
  if (ret == 0) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_iiowrapper"), "no samples available yet, retrying");
    return false;
  }
  if (ret < 0) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_iiowrapper"), "buffer refill error status %d", ret);
    stopBufferAcquisition();
    return false;
  }

  iio_buffer_foreach_sample(m_device_buffer, demux_sample, NULL);

  read_buffer_idx = 0;

  return true;
}

double IIOWrapper::getBuffLinearAccelerationX()
{
  return (int32_t)buffered_data[CHAN_ACCEL_X][read_buffer_idx] * m_scale_accel_x;
}

double IIOWrapper::getBuffLinearAccelerationY()
{
  return (int32_t)buffered_data[CHAN_ACCEL_Y][read_buffer_idx] * m_scale_accel_y;
}

double IIOWrapper::getBuffLinearAccelerationZ()
{
  return (int32_t)buffered_data[CHAN_ACCEL_Z][read_buffer_idx] * m_scale_accel_z;
}

double IIOWrapper::getBuffAngularVelocityX()
{
  return (int32_t)buffered_data[CHAN_GYRO_X][read_buffer_idx] * m_scale_angvel_x;
}

double IIOWrapper::getBuffAngularVelocityY()
{
  return (int32_t)buffered_data[CHAN_GYRO_Y][read_buffer_idx] * m_scale_angvel_y;
}

double IIOWrapper::getBuffAngularVelocityZ()
{
  return (int32_t)buffered_data[CHAN_GYRO_Z][read_buffer_idx] * m_scale_angvel_z;
}

double IIOWrapper::getBuffDeltaAngleX()
{
  return (int32_t)buffered_data[CHAN_DELTA_ANGL_X][read_buffer_idx] * m_scale_rot_x;
}

double IIOWrapper::getBuffDeltaAngleY()
{
  return (int32_t)buffered_data[CHAN_DELTA_ANGL_Y][read_buffer_idx] * m_scale_rot_y;
}

double IIOWrapper::getBuffDeltaAngleZ()
{
  return (int32_t)buffered_data[CHAN_DELTA_ANGL_Z][read_buffer_idx] * m_scale_rot_z;
}

double IIOWrapper::getBuffDeltaVelocityX()
{
  return (int32_t)buffered_data[CHAN_DELTA_VEL_X][read_buffer_idx] * m_scale_velocity_x;
}

double IIOWrapper::getBuffDeltaVelocityY()
{
  return (int32_t)buffered_data[CHAN_DELTA_VEL_Y][read_buffer_idx] * m_scale_velocity_y;
}

double IIOWrapper::getBuffDeltaVelocityZ()
{
  return (int32_t)buffered_data[CHAN_DELTA_VEL_Z][read_buffer_idx] * m_scale_velocity_z;
}

double IIOWrapper::getBuffTemperature()
{
  return (int32_t)buffered_data[CHAN_TEMP][read_buffer_idx] * m_scale_temp / 1000.0;
}

int64_t IIOWrapper::getBuffSampleTimestamp()
{
  uint16_t timestamp_0_15 = buffered_data[CHAN_DATA_TIMESTAMP][read_buffer_idx];
  uint16_t timestamp_16_31 = buffered_data[CHAN_DATA_TIMESTAMP][read_buffer_idx] >> 16;
  uint16_t timestamp_32_47 = buffered_data[CHAN_DATA_TIMESTAMP + 1][read_buffer_idx];
  uint16_t timestamp_48_63 = buffered_data[CHAN_DATA_TIMESTAMP + 1][read_buffer_idx] >> 16;

  uint64_t timestamp = ((uint64_t)timestamp_48_63 << 48) | ((uint64_t)timestamp_32_47 << 32) |
                       ((uint64_t)timestamp_16_31 << 16) | timestamp_0_15;
  return (int64_t)timestamp;
}

bool IIOWrapper::getRegLinearAccelerationX(double & result)
{
  long long valueRaw;
  int ret = iio_channel_attr_read_longlong(m_channel_accel_x, "raw", &valueRaw);

  result = valueRaw * m_scale_accel_x;
  return (ret == 0);
}

bool IIOWrapper::getRegLinearAccelerationY(double & result)
{
  long long valueRaw;
  int ret = iio_channel_attr_read_longlong(m_channel_accel_y, "raw", &valueRaw);

  result = valueRaw * m_scale_accel_y;
  return (ret == 0);
}

bool IIOWrapper::getRegLinearAccelerationZ(double & result)
{
  long long valueRaw;
  int ret = iio_channel_attr_read_longlong(m_channel_accel_z, "raw", &valueRaw);

  result = valueRaw * m_scale_accel_z;
  return (ret == 0);
}

bool IIOWrapper::getRegAngularVelocityX(double & result)
{
  long long valueRaw;
  int ret = iio_channel_attr_read_longlong(m_channel_anglvel_x, "raw", &valueRaw);

  result = valueRaw * m_scale_angvel_x;
  return (ret == 0);
}

bool IIOWrapper::getRegAngularVelocityY(double & result)
{
  long long valueRaw;
  int ret = iio_channel_attr_read_longlong(m_channel_anglvel_y, "raw", &valueRaw);

  result = valueRaw * m_scale_angvel_y;
  return (ret == 0);
}

bool IIOWrapper::getRegAngularVelocityZ(double & result)
{
  long long valueRaw;
  int ret = iio_channel_attr_read_longlong(m_channel_anglvel_z, "raw", &valueRaw);

  result = valueRaw * m_scale_angvel_z;
  return (ret == 0);
}

bool IIOWrapper::getRegDeltaAngleX(double & result)
{
  long long valueRaw;
  int ret = iio_channel_attr_read_longlong(m_channel_rot_x, "raw", &valueRaw);

  result = valueRaw * m_scale_rot_x;
  return (ret == 0);
}

bool IIOWrapper::getRegDeltaAngleY(double & result)
{
  long long valueRaw;
  int ret = iio_channel_attr_read_longlong(m_channel_rot_y, "raw", &valueRaw);

  result = valueRaw * m_scale_rot_y;
  return (ret == 0);
}

bool IIOWrapper::getRegDeltaAngleZ(double & result)
{
  long long valueRaw;
  int ret = iio_channel_attr_read_longlong(m_channel_rot_z, "raw", &valueRaw);

  result = valueRaw * m_scale_rot_z;
  return (ret == 0);
}

bool IIOWrapper::getRegDeltaVelocityX(double & result)
{
  long long valueRaw;
  int ret = iio_channel_attr_read_longlong(m_channel_velocity_x, "raw", &valueRaw);

  result = valueRaw * m_scale_velocity_x;
  return (ret == 0);
}

bool IIOWrapper::getRegDeltaVelocityY(double & result)
{
  long long valueRaw;
  int ret = iio_channel_attr_read_longlong(m_channel_velocity_y, "raw", &valueRaw);

  result = valueRaw * m_scale_velocity_y;
  return (ret == 0);
}

bool IIOWrapper::getRegDeltaVelocityZ(double & result)
{
  long long valueRaw;
  int ret = iio_channel_attr_read_longlong(m_channel_velocity_z, "raw", &valueRaw);

  result = valueRaw * m_scale_velocity_z;
  return (ret == 0);
}

bool IIOWrapper::getRegTemperature(double & result)
{
  long long valueRaw;
  int ret = iio_channel_attr_read_longlong(m_channel_temp, "raw", &valueRaw);

  result = valueRaw * m_scale_temp / 1000.0;
  return (ret == 0);
}

bool IIOWrapper::anglvel_x_calibbias(int32_t & result)
{
  long long valuel;
  int ret = iio_channel_attr_read_longlong(m_channel_anglvel_x, "calibbias", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_anglvel_calibbias_x(int32_t val)
{
  return (iio_channel_attr_write_longlong(m_channel_anglvel_x, "calibbias", val) == 0);
}

bool IIOWrapper::anglvel_y_calibbias(int32_t & result)
{
  long long valuel;
  int ret = iio_channel_attr_read_longlong(m_channel_anglvel_y, "calibbias", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_anglvel_calibbias_y(int32_t val)
{
  return (iio_channel_attr_write_longlong(m_channel_anglvel_y, "calibbias", val) == 0);
}

bool IIOWrapper::anglvel_z_calibbias(int32_t & result)
{
  long long valuel;
  int ret = iio_channel_attr_read_longlong(m_channel_anglvel_z, "calibbias", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_anglvel_calibbias_z(int32_t val)
{
  return (iio_channel_attr_write_longlong(m_channel_anglvel_z, "calibbias", val) == 0);
}

bool IIOWrapper::accel_x_calibbias(int32_t & result)
{
  long long valuel;
  int ret = iio_channel_attr_read_longlong(m_channel_accel_x, "calibbias", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_accel_calibbias_x(int32_t val)
{
  return (iio_channel_attr_write_longlong(m_channel_accel_x, "calibbias", val) == 0);
}

bool IIOWrapper::accel_y_calibbias(int32_t & result)
{
  long long valuel;
  int ret = iio_channel_attr_read_longlong(m_channel_accel_y, "calibbias", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_accel_calibbias_y(int32_t val)
{
  return (iio_channel_attr_write_longlong(m_channel_accel_y, "calibbias", val) == 0);
}

bool IIOWrapper::accel_z_calibbias(int32_t & result)
{
  long long valuel;
  int ret = iio_channel_attr_read_longlong(m_channel_accel_z, "calibbias", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_accel_calibbias_z(int32_t val)
{
  return (iio_channel_attr_write_longlong(m_channel_accel_z, "calibbias", val) == 0);
}

bool IIOWrapper::sampling_frequency(double * result)
{
  int ret;

  ret = iio_device_attr_read_double(m_dev, "sampling_frequency", result);
  if (ret) return false;

  samp_freq = *result;

  return true;
}

bool IIOWrapper::update_sampling_frequency(double val)
{
  int ret;

  ret = iio_device_attr_write_double(m_dev, "sampling_frequency", val);
  if (ret) return false;

  samp_freq = val;

  return true;
}

bool IIOWrapper::diag_sensor_initialization_failure(bool & result)
{
  return (
    iio_device_debug_attr_read_bool(m_dev, "diag_sensor_initialization_failure", &result) == 0);
}

bool IIOWrapper::diag_data_path_overrun(bool & result)
{
  return (iio_device_debug_attr_read_bool(m_dev, "diag_data_path_overrun", &result) == 0);
}

bool IIOWrapper::diag_flash_memory_update_error(bool & result)
{
  return (iio_device_debug_attr_read_bool(m_dev, "diag_flash_memory_update_error", &result) == 0);
}

bool IIOWrapper::diag_spi_communication_error(bool & result)
{
  return (iio_device_debug_attr_read_bool(m_dev, "diag_spi_communication_error", &result) == 0);
}

bool IIOWrapper::diag_standby_mode(bool & result)
{
  return (iio_device_debug_attr_read_bool(m_dev, "diag_standby_mode", &result) == 0);
}

bool IIOWrapper::diag_sensor_self_test_error(bool & result)
{
  return (iio_device_debug_attr_read_bool(m_dev, "diag_sensor_self_test_error", &result) == 0);
}

bool IIOWrapper::diag_flash_memory_test_error(bool & result)
{
  return (iio_device_debug_attr_read_bool(m_dev, "diag_flash_memory_test_error", &result) == 0);
}

bool IIOWrapper::diag_clock_error(bool & result)
{
  return (iio_device_debug_attr_read_bool(m_dev, "diag_clk_error", &result) == 0);
}

bool IIOWrapper::diag_gyroscope1_self_test_error(bool & result)
{
  return (iio_device_debug_attr_read_bool(m_dev, "diag_gyroscope1_self_test_error", &result) == 0);
}

bool IIOWrapper::diag_gyroscope2_self_test_error(bool & result)
{
  return (iio_device_debug_attr_read_bool(m_dev, "diag_gyroscope2_self_test_error", &result) == 0);
}

bool IIOWrapper::diag_acceleration_self_test_error(bool & result)
{
  return (
    iio_device_debug_attr_read_bool(m_dev, "diag_acceleration_self_test_error", &result) == 0);
}

bool IIOWrapper::diag_x_axis_gyroscope_failure(bool & result)
{
  return (iio_device_debug_attr_read_bool(m_dev, "diag_x_axis_gyroscope_failure", &result) == 0);
}

bool IIOWrapper::diag_y_axis_gyroscope_failure(bool & result)
{
  return (iio_device_debug_attr_read_bool(m_dev, "diag_y_axis_gyroscope_failure", &result) == 0);
}

bool IIOWrapper::diag_z_axis_gyroscope_failure(bool & result)
{
  return (iio_device_debug_attr_read_bool(m_dev, "diag_z_axis_gyroscope_failure", &result) == 0);
}

bool IIOWrapper::diag_x_axis_accelerometer_failure(bool & result)
{
  return (
    iio_device_debug_attr_read_bool(m_dev, "diag_x_axis_accelerometer_failure", &result) == 0);
}

bool IIOWrapper::diag_y_axis_accelerometer_failure(bool & result)
{
  return (
    iio_device_debug_attr_read_bool(m_dev, "diag_y_axis_accelerometer_failure", &result) == 0);
}

bool IIOWrapper::diag_z_axis_accelerometer_failure(bool & result)
{
  return (
    iio_device_debug_attr_read_bool(m_dev, "diag_z_axis_accelerometer_failure", &result) == 0);
}

bool IIOWrapper::diag_aduc_mcu_fault(bool & result)
{
  return (iio_device_debug_attr_read_bool(m_dev, "diag_aduc_mcu_fault", &result) == 0);
}

bool IIOWrapper::diag_checksum_error_flag(bool & result)
{
  return (iio_device_debug_attr_read_bool(m_dev, "diag_checksum_error_flag", &result) == 0);
}

bool IIOWrapper::diag_flash_memory_write_count_exceeded_error(bool & result)
{
  return (
    iio_device_debug_attr_read_bool(
      m_dev, "diag_flash_memory_write_count_exceeded_error", &result) == 0);
}

bool IIOWrapper::lost_samples_count(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "lost_samples_count", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::fifo_enable(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "fifo_enable", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_fifo_enable(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "fifo_enable", val) == 0);
}

bool IIOWrapper::fifo_overflow_behavior(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "fifo_overflow_behavior", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_fifo_overflow_behavior(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "fifo_overflow_behavior", val) == 0);
}

bool IIOWrapper::fifo_watermark_interrupt_enable(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "fifo_watermark_interrupt_enable", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_fifo_watermark_interrupt_enable(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "fifo_watermark_interrupt_enable", val) == 0);
}

bool IIOWrapper::fifo_watermark_interrupt_polarity(uint32_t & result)
{
  long long valuel;
  int ret =
    iio_device_debug_attr_read_longlong(m_dev, "fifo_watermark_interrupt_polarity", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_fifo_watermark_interrupt_polarity(uint32_t val)
{
  return (
    iio_device_debug_attr_write_longlong(m_dev, "fifo_watermark_interrupt_polarity", val) == 0);
}

bool IIOWrapper::fifo_watermark_threshold_level(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "fifo_watermark_threshold_level", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_fifo_watermark_threshold_level(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "fifo_watermark_threshold_level", val) == 0);
}

bool IIOWrapper::filter_size(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "filter_size", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_filter_size(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "filter_size", val) == 0);
}

bool IIOWrapper::gyroscope_measurement_range(std::string & result)
{
  char valuec[32];
  int ret = iio_device_debug_attr_read(m_dev, "gyroscope_measurement_range", valuec, 32);

  result = valuec;
  return (ret > 0);
}

bool IIOWrapper::data_ready_polarity(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "data_ready_polarity", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_data_ready_polarity(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "data_ready_polarity", val) == 0);
}

bool IIOWrapper::sync_polarity(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "sync_polarity", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_sync_polarity(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "sync_polarity", val) == 0);
}

bool IIOWrapper::sync_mode_select(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "sync_mode_select", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_sync_mode_select(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "sync_mode_select", val) == 0);
}

bool IIOWrapper::internal_sensor_bandwidth(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "internal_sensor_bandwidth", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_internal_sensor_bandwidth(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "internal_sensor_bandwidth", val) == 0);
}

bool IIOWrapper::point_of_percussion_alignment(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "point_of_percussion_alignment", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_point_of_percussion_alignment(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "point_of_percussion_alignment", val) == 0);
}

bool IIOWrapper::update_linear_acceleration_compensation(uint32_t val)
{
  return (
    iio_device_debug_attr_write_longlong(m_dev, "linear_acceleration_compensation", val) == 0);
}

bool IIOWrapper::linear_acceleration_compensation(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "linear_acceleration_compensation", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::burst_data_selection(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "burst_data_selection", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_burst_data_selection(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "burst_data_selection", val) == 0);
}

bool IIOWrapper::burst_size_selection(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "burst_size_selection", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_burst_size_selection(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "burst_size_selection", val) == 0);
}

bool IIOWrapper::timestamp32(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "timestamp32", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_timestamp32(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "timestamp32", val) == 0);
}

bool IIOWrapper::internal_sync_enable_4khz(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "internal_sync_enable_4khz", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_internal_sync_enable_4khz(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "internal_sync_enable_4khz", val) == 0);
}

bool IIOWrapper::sync_signal_scale(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "sync_signal_scale", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_sync_signal_scale(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "sync_signal_scale", val) == 0);
}

bool IIOWrapper::decimation_filter(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "decimation_filter", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_decimation_filter(uint32_t val)
{
  return (iio_device_debug_attr_write_longlong(m_dev, "decimation_filter", val) == 0);
}

bool IIOWrapper::bias_correction_time_base_control(uint32_t & result)
{
  long long valuel;
  int ret =
    iio_device_debug_attr_read_longlong(m_dev, "bias_correction_time_base_control", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_bias_correction_time_base_control(uint32_t val)
{
  return (
    iio_device_debug_attr_write_longlong(m_dev, "bias_correction_time_base_control", val) == 0);
}

bool IIOWrapper::x_axis_gyroscope_bias_correction_enable(uint32_t & result)
{
  long long valuel;
  int ret =
    iio_device_debug_attr_read_longlong(m_dev, "x_axis_gyroscope_bias_correction_enable", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_x_axis_gyroscope_bias_correction_enable(uint32_t val)
{
  return (
    iio_device_debug_attr_write_longlong(m_dev, "x_axis_gyroscope_bias_correction_enable", val) ==
    0);
}

bool IIOWrapper::y_axis_gyroscope_bias_correction_enable(uint32_t & result)
{
  long long valuel;
  int ret =
    iio_device_debug_attr_read_longlong(m_dev, "y_axis_gyroscope_bias_correction_enable", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_y_axis_gyroscope_bias_correction_enable(uint32_t val)
{
  return (
    iio_device_debug_attr_write_longlong(m_dev, "y_axis_gyroscope_bias_correction_enable", val) ==
    0);
}

bool IIOWrapper::z_axis_gyroscope_bias_correction_enable(uint32_t & result)
{
  long long valuel;
  int ret =
    iio_device_debug_attr_read_longlong(m_dev, "z_axis_gyroscope_bias_correction_enable", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_z_axis_gyroscope_bias_correction_enable(uint32_t val)
{
  return (
    iio_device_debug_attr_write_longlong(m_dev, "z_axis_gyroscope_bias_correction_enable", val) ==
    0);
}

bool IIOWrapper::x_axis_accelerometer_bias_correction_enable(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(
    m_dev, "x_axis_accelerometer_bias_correction_enable", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_x_axis_accelerometer_bias_correction_enable(uint32_t val)
{
  return (
    iio_device_debug_attr_write_longlong(
      m_dev, "x_axis_accelerometer_bias_correction_enable", val) == 0);
}

bool IIOWrapper::y_axis_accelerometer_bias_correction_enable(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(
    m_dev, "y_axis_accelerometer_bias_correction_enable", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_y_axis_accelerometer_bias_correction_enable(uint32_t val)
{
  return (
    iio_device_debug_attr_write_longlong(
      m_dev, "y_axis_accelerometer_bias_correction_enable", val) == 0);
}

bool IIOWrapper::z_axis_accelerometer_bias_correction_enable(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(
    m_dev, "z_axis_accelerometer_bias_correction_enable", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_z_axis_accelerometer_bias_correction_enable(uint32_t val)
{
  return (
    iio_device_debug_attr_write_longlong(
      m_dev, "z_axis_accelerometer_bias_correction_enable", val) == 0);
}

bool IIOWrapper::bias_correction_update()
{
  return (iio_device_debug_attr_write_longlong(m_dev, "bias_correction_update", 1) == 0);
}

bool IIOWrapper::factory_calibration_restore()
{
  return (iio_device_debug_attr_write_longlong(m_dev, "factory_calibration_restore", 1) == 0);
}

bool IIOWrapper::sensor_self_test()
{
  return (iio_device_debug_attr_write_longlong(m_dev, "sensor_self_test", 1) == 0);
}

bool IIOWrapper::flash_memory_update()
{
  return (iio_device_debug_attr_write_longlong(m_dev, "flash_memory_update", 1) == 0);
}

bool IIOWrapper::flash_memory_test()
{
  return (iio_device_debug_attr_write_longlong(m_dev, "flash_memory_test", 1) == 0);
}

bool IIOWrapper::fifo_flush()
{
  return (iio_device_debug_attr_write_longlong(m_dev, "fifo_flush", 1) == 0);
}

bool IIOWrapper::software_reset()
{
  return (iio_device_debug_attr_write_longlong(m_dev, "software_reset", 1) == 0);
}

bool IIOWrapper::firmware_revision(std::string & result)
{
  char valuec[32];
  int ret = iio_device_debug_attr_read(m_dev, "firmware_revision", valuec, 32);

  result = valuec;
  return (ret > 0);
}

bool IIOWrapper::firmware_date(std::string & result)
{
  char valuec[32];
  int ret = iio_device_debug_attr_read(m_dev, "firmware_date", valuec, 32);

  result = valuec;
  return (ret > 0);
}

bool IIOWrapper::product_id(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "product_id", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::serial_number(uint32_t & result)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "serial_number", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::flash_counter(uint32_t & value)
{
  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "flash_counter", &valuel);

  value = valuel;
  return (ret == 0);
}

double IIOWrapper::get_scale_accel() { return m_scale_accel_x; }

double IIOWrapper::get_scale_angvel() { return m_scale_angvel_x; }

double IIOWrapper::get_scale_velocity() { return m_scale_velocity_x; }

double IIOWrapper::get_scale_rot() { return m_scale_rot_x; }

double IIOWrapper::get_scale_temp() { return m_scale_temp / 1000.0; }
