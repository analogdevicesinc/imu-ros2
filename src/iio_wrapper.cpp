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

#include <rclcpp/rclcpp.hpp>

/*! Maximum allowed number of samples in IIO buffer. */
#define MAX_NO_OF_SAMPLES 1

enum
{
  CHAN_GYRO_X,
  CHAN_GYRO_Y,
  CHAN_GYRO_Z,
  CHAN_ACCEL_X,
  CHAN_ACCEL_Y,
  CHAN_ACCEL_Z,
  CHAN_TEMP,
#ifdef ADIS_HAS_DELTA_BURST
  CHAN_DELTA_ANGL_X,
  CHAN_DELTA_ANGL_Y,
  CHAN_DELTA_ANGL_Z,
  CHAN_DELTA_VEL_X,
  CHAN_DELTA_VEL_Y,
  CHAN_DELTA_VEL_Z,
#endif
  CHAN_DATA_TIMESTAMP,
  NO_OF_CHANS,
};

struct iio_context * IIOWrapper::m_iio_context = nullptr;

struct iio_device * IIOWrapper::m_dev = nullptr;
struct iio_device * IIOWrapper::m_dev_trigger = nullptr;
struct iio_buffer * IIOWrapper::m_dev_buffer = nullptr;

struct iio_channel * IIOWrapper::m_channel_accel_x = nullptr;
struct iio_channel * IIOWrapper::m_channel_accel_y = nullptr;
struct iio_channel * IIOWrapper::m_channel_accel_z = nullptr;
struct iio_channel * IIOWrapper::m_channel_anglvel_x = nullptr;
struct iio_channel * IIOWrapper::m_channel_anglvel_y = nullptr;
struct iio_channel * IIOWrapper::m_channel_anglvel_z = nullptr;
struct iio_channel * IIOWrapper::m_channel_deltaangl_x = nullptr;
struct iio_channel * IIOWrapper::m_channel_deltaangl_y = nullptr;
struct iio_channel * IIOWrapper::m_channel_deltaangl_z = nullptr;
struct iio_channel * IIOWrapper::m_channel_deltavelocity_x = nullptr;
struct iio_channel * IIOWrapper::m_channel_deltavelocity_y = nullptr;
struct iio_channel * IIOWrapper::m_channel_deltavelocity_z = nullptr;
struct iio_channel * IIOWrapper::m_channel_temp = nullptr;
struct iio_channel * IIOWrapper::m_channel_timestamp = nullptr;

double IIOWrapper::m_scale_accel_x = 0;
double IIOWrapper::m_scale_accel_y = 0;
double IIOWrapper::m_scale_accel_z = 0;
double IIOWrapper::m_scale_anglvel_x = 0;
double IIOWrapper::m_scale_anglvel_y = 0;
double IIOWrapper::m_scale_anglvel_z = 0;
double IIOWrapper::m_scale_deltaangl_x = 0;
double IIOWrapper::m_scale_deltaangl_y = 0;
double IIOWrapper::m_scale_deltaangl_z = 0;
double IIOWrapper::m_scale_deltavelocity_x = 0;
double IIOWrapper::m_scale_deltavelocity_y = 0;
double IIOWrapper::m_scale_deltavelocity_z = 0;
double IIOWrapper::m_scale_temp = 0;

/*! Buffer write index.  */
uint32_t buff_write_idx = 0;
/*! Buffer read index.  */
uint32_t buff_read_idx = MAX_NO_OF_SAMPLES;
/*! Buffer containing the read data. */
uint32_t buff_data[NO_OF_CHANS + 1][MAX_NO_OF_SAMPLES];
/*! Sampling frequency of the device. */
double samp_freq = 2000.0;
/*! Number of samples to be read at once. */
uint32_t no_of_samp = MAX_NO_OF_SAMPLES;
/*! Current set data selection. */
uint32_t current_data_selection = FULL_MEASURED_DATA;

bool has_delta_channels = true;
bool has_timestamp_channel = false;

IIOWrapper::IIOWrapper() {}

IIOWrapper::~IIOWrapper()
{
  if (m_dev_buffer != nullptr) {
    iio_buffer_destroy(m_dev_buffer);
    m_dev_buffer = nullptr;
  }
  if (m_iio_context != nullptr) {
    iio_context_destroy(m_iio_context);
    m_iio_context = nullptr;
  }
}

void IIOWrapper::createContext(const char * context)
{
  if (m_iio_context) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_iiowrapper"), "IIO context already exists.");
    return;
  }

  if (!strcmp(context, "local:"))
    m_iio_context = iio_create_local_context();
  else
    m_iio_context = iio_create_context_from_uri(context);

  if (!m_iio_context) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_iiowrapper"), "IIO context is null");
    return;
  }

  iio_context_set_timeout(m_iio_context, 5000);

  std::list<std::string> supported_devices{
    "adis16465-1", "adis16465-2", "adis16465-3", "adis16467-1", "adis16467-2", "adis16467-3",
    "adis16470",   "adis16475-1", "adis16475-2", "adis16475-3", "adis16477-1", "adis16477-2",
    "adis16477-3", "adis16500",   "adis16501",   "adis16505-1", "adis16505-2", "adis16505-3",
    "adis16507-1", "adis16507-2", "adis16507-3", "adis16575-2", "adis16575-3", "adis16576-2",
    "adis16576-3", "adis16577-2", "adis16577-3"};

  uint8_t dev_id = 0;

  for (std::string const & devname : supported_devices) {
    m_dev = iio_context_find_device(m_iio_context, devname.c_str());
    if (m_dev) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp_iiowrapper"), "Found device: %s", devname.c_str());
      if (m_dev_trigger == nullptr) {
        const std::string devid = iio_device_get_id(m_dev);

        /* Get device number in the form of 0, 1, etc. to append to the trigger name. */
        std::string devnb;
        for (long unsigned int i = 0; i < devid.std::string::length(); i++)
          if (isdigit(devid.c_str()[i])) devnb.std::string::append(&devid.c_str()[i]);

        std::string triggerName = devname + "-dev" + devnb;
        m_dev_trigger = iio_context_find_device(m_iio_context, triggerName.c_str());
        RCLCPP_INFO(
          rclcpp::get_logger("rclcpp_iiowrapper"), "Found trigger: %s", triggerName.c_str());
        if (!m_dev_trigger) {
          RCLCPP_INFO(
            rclcpp::get_logger("rclcpp_iiowrapper"), "Did not find a trigger for device %s.",
            devname.c_str());
          return;
        }
      }
      break;
    }
    dev_id++;
  }

  if (!m_dev) {
    iio_context_destroy(m_iio_context);
    m_iio_context = nullptr;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_iiowrapper"), "No supported IIO device found.");
    return;
  }

  iio_device_set_trigger(m_dev, m_dev_trigger);

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

  if (m_channel_deltaangl_x == nullptr) {
    m_channel_deltaangl_x = iio_device_find_channel(m_dev, "deltaangl_x", false);
    if (m_channel_deltaangl_x == nullptr) has_delta_channels = false;
  }

  if (has_delta_channels) {
    if (m_channel_deltaangl_y == nullptr)
      m_channel_deltaangl_y = iio_device_find_channel(m_dev, "deltaangl_y", false);
    if (m_channel_deltaangl_z == nullptr)
      m_channel_deltaangl_z = iio_device_find_channel(m_dev, "deltaangl_z", false);

    if (m_channel_deltavelocity_x == nullptr)
      m_channel_deltavelocity_x = iio_device_find_channel(m_dev, "deltavelocity_x", false);
    if (m_channel_deltavelocity_y == nullptr)
      m_channel_deltavelocity_y = iio_device_find_channel(m_dev, "deltavelocity_y", false);
    if (m_channel_deltavelocity_z == nullptr)
      m_channel_deltavelocity_z = iio_device_find_channel(m_dev, "deltavelocity_z", false);
  }

  if (m_channel_temp == nullptr) m_channel_temp = iio_device_find_channel(m_dev, "temp0", false);

  if (m_channel_timestamp == nullptr)
    m_channel_timestamp = iio_device_find_channel(m_dev, "timestamp", false);

  if (m_channel_temp) iio_channel_enable(m_channel_temp);

  if (m_channel_timestamp) {
    iio_channel_enable(m_channel_timestamp);
    has_timestamp_channel = true;
  }

  if (m_channel_accel_x) iio_channel_attr_read_double(m_channel_accel_x, "scale", &m_scale_accel_x);

  if (m_channel_accel_y) iio_channel_attr_read_double(m_channel_accel_y, "scale", &m_scale_accel_y);

  if (m_channel_accel_z) iio_channel_attr_read_double(m_channel_accel_z, "scale", &m_scale_accel_z);

  if (m_channel_anglvel_x)
    iio_channel_attr_read_double(m_channel_anglvel_x, "scale", &m_scale_anglvel_x);

  if (m_channel_anglvel_y)
    iio_channel_attr_read_double(m_channel_anglvel_y, "scale", &m_scale_anglvel_y);

  if (m_channel_anglvel_z)
    iio_channel_attr_read_double(m_channel_anglvel_z, "scale", &m_scale_anglvel_z);

  if (has_delta_channels) {
    iio_channel_attr_read_double(m_channel_deltaangl_x, "scale", &m_scale_deltaangl_x);
    iio_channel_attr_read_double(m_channel_deltaangl_y, "scale", &m_scale_deltaangl_y);
    iio_channel_attr_read_double(m_channel_deltaangl_z, "scale", &m_scale_deltaangl_z);
    iio_channel_attr_read_double(m_channel_deltavelocity_x, "scale", &m_scale_deltavelocity_x);
    iio_channel_attr_read_double(m_channel_deltavelocity_y, "scale", &m_scale_deltavelocity_y);
    iio_channel_attr_read_double(m_channel_deltavelocity_z, "scale", &m_scale_deltavelocity_z);
  } else {
    /* Set scale manually in case delta channels are not available in the linux driver. */
    setDeltaAngleScales((enum adis_device_id)dev_id);
    setDeltaVelocityScales((enum adis_device_id)dev_id);
  }

  if (m_channel_temp) iio_channel_attr_read_double(m_channel_temp, "scale", &m_scale_temp);
}

bool IIOWrapper::updateField(uint32_t reg, uint32_t val, uint32_t mask)
{
  int ret;
  uint32_t __val;

  if (!m_dev) return false;

  ret = iio_device_reg_read(m_dev, reg, &__val);
  if (ret) return false;

  __val = (__val & ~mask) | (val & mask);

  return (iio_device_reg_write(m_dev, reg, __val) == 0);
}

void IIOWrapper::setDeltaAngleScales(enum adis_device_id id)
{
  switch (id) {
    case ADIS16465_1:
    case ADIS16467_1:
    case ADIS16475_1:
    case ADIS16477_1:
    case ADIS16505_1:
    case ADIS16507_1:
      m_scale_deltaangl_x = 0.000000002;
      m_scale_deltaangl_y = 0.000000002;
      m_scale_deltaangl_z = 0.000000002;
      return;
    case ADIS16465_2:
    case ADIS16467_2:
    case ADIS16475_2:
    case ADIS16477_2:
    case ADIS16501:
    case ADIS16505_2:
    case ADIS16507_2:
      m_scale_deltaangl_x = 0.000000006;
      m_scale_deltaangl_y = 0.000000006;
      m_scale_deltaangl_z = 0.000000006;
      return;
    case ADIS16465_3:
    case ADIS16467_3:
    case ADIS16470:
    case ADIS16475_3:
    case ADIS16477_3:
    case ADIS16500:
    case ADIS16505_3:
    case ADIS16507_3:
      m_scale_deltaangl_x = 0.000000017;
      m_scale_deltaangl_y = 0.000000017;
      m_scale_deltaangl_z = 0.000000017;
      return;
    case ADIS16575_2:
    case ADIS16576_2:
    case ADIS16577_2:
      m_scale_deltaangl_x = 0.000000003;
      m_scale_deltaangl_y = 0.000000003;
      m_scale_deltaangl_z = 0.000000003;
      return;
    case ADIS16575_3:
    case ADIS16576_3:
    case ADIS16577_3:
      m_scale_deltaangl_x = 0.000000016;
      m_scale_deltaangl_y = 0.000000016;
      m_scale_deltaangl_z = 0.000000016;
      return;
    default:
      return;
  }
}

void IIOWrapper::setDeltaVelocityScales(enum adis_device_id id)
{
  switch (id) {
    case ADIS16467_1:
    case ADIS16467_2:
    case ADIS16467_3:
    case ADIS16470:
    case ADIS16477_1:
    case ADIS16477_2:
    case ADIS16477_3:
    case ADIS16500:
    case ADIS16507_1:
    case ADIS16507_2:
    case ADIS16507_3:
      m_scale_deltavelocity_x = 0.000000186;
      m_scale_deltavelocity_y = 0.000000186;
      m_scale_deltavelocity_z = 0.000000186;
      return;
    case ADIS16465_1:
    case ADIS16465_2:
    case ADIS16465_3:
    case ADIS16475_1:
    case ADIS16475_2:
    case ADIS16475_3:
    case ADIS16505_1:
    case ADIS16505_2:
    case ADIS16505_3:
    case ADIS16575_2:
    case ADIS16575_3:
    case ADIS16576_2:
    case ADIS16576_3:
    case ADIS16577_2:
    case ADIS16577_3:
      m_scale_deltavelocity_x = 0.000000046;
      m_scale_deltavelocity_y = 0.000000046;
      m_scale_deltavelocity_z = 0.000000046;
      return;
    case ADIS16501:
      m_scale_deltavelocity_x = 0.000000058;
      m_scale_deltavelocity_y = 0.000000058;
      m_scale_deltavelocity_z = 0.000000058;
      return;
    default:
      return;
  }
}

void IIOWrapper::stopBufferAcquisition()
{
  if (m_dev_buffer != nullptr) {
    iio_buffer_destroy(m_dev_buffer);
    m_dev_buffer = nullptr;
  }
}

static ssize_t demux_sample(
  const struct iio_channel * chn, void * sample, size_t size, __attribute__((unused)) void * d)
{
  if (size == 2) {
    int16_t val;
    iio_channel_convert(chn, &val, sample);
    buff_data[iio_channel_get_index(chn)][buff_write_idx] = val;
    if (!has_timestamp_channel) {
      /* timestamp channel is not available, have to update buff_write_idx */
      buff_write_idx++;
    }
  } else if (size == 4) {
    int32_t val;
    iio_channel_convert(chn, &val, sample);
    buff_data[iio_channel_get_index(chn)][buff_write_idx] = val;
    if (!has_timestamp_channel) {
/* timestamp channel is not available, have to update buff_write_idx for last read channel */
#ifdef ADIS_HAS_DELTA_BURST
      if (iio_channel_get_index(chn) == CHAN_DELTA_VEL_Z) buff_write_idx++;
#else
      if (iio_channel_get_index(chn) == CHAN_ACCEL_Z) buff_write_idx++;
#endif
    }
  } else {
    int64_t val;
    iio_channel_convert(chn, &val, sample);
    buff_data[CHAN_DATA_TIMESTAMP][buff_write_idx] = val;
    buff_data[CHAN_DATA_TIMESTAMP + 1][buff_write_idx++] = val >> 32;
  }

  if (buff_write_idx == no_of_samp) buff_write_idx = 0;

  return size;
}

bool IIOWrapper::updateBuffer(uint32_t data_selection)
{
  ssize_t ret;

  if (current_data_selection != data_selection) {
    stopBufferAcquisition();
    if (data_selection == ACCEL_GYRO_BUFFERED_DATA) {
#ifdef ADIS_HAS_DELTA_BURST
      if (has_delta_channels) {
        iio_channel_disable(m_channel_deltaangl_x);
        iio_channel_disable(m_channel_deltaangl_y);
        iio_channel_disable(m_channel_deltaangl_z);
        iio_channel_disable(m_channel_deltavelocity_x);
        iio_channel_disable(m_channel_deltavelocity_y);
        iio_channel_disable(m_channel_deltavelocity_z);
      }
#endif
      iio_channel_enable(m_channel_accel_x);
      iio_channel_enable(m_channel_accel_y);
      iio_channel_enable(m_channel_accel_z);
      iio_channel_enable(m_channel_anglvel_x);
      iio_channel_enable(m_channel_anglvel_y);
      iio_channel_enable(m_channel_anglvel_z);
    } else {
      iio_channel_disable(m_channel_accel_x);
      iio_channel_disable(m_channel_accel_y);
      iio_channel_disable(m_channel_accel_z);
      iio_channel_disable(m_channel_anglvel_x);
      iio_channel_disable(m_channel_anglvel_y);
      iio_channel_disable(m_channel_anglvel_z);
#ifdef ADIS_HAS_DELTA_BURST
      if (has_delta_channels) {
        iio_channel_enable(m_channel_deltaangl_x);
        iio_channel_enable(m_channel_deltaangl_y);
        iio_channel_enable(m_channel_deltaangl_z);
        iio_channel_enable(m_channel_deltavelocity_x);
        iio_channel_enable(m_channel_deltavelocity_y);
        iio_channel_enable(m_channel_deltavelocity_z);
      } else {
        stopBufferAcquisition();
      }
#endif
    }

    current_data_selection = data_selection;
  }

  if (m_dev_buffer == nullptr) {
    sampling_frequency(&samp_freq);
    no_of_samp = MAX_NO_OF_SAMPLES;
    if (no_of_samp > samp_freq)
      /* Overwrite number of samples based on sampling frequency, to avoid timeout errors from LibIIO */
      no_of_samp = samp_freq;

    m_dev_buffer = iio_device_create_buffer(m_dev, no_of_samp, false);

    if (!m_dev_buffer) throw std::runtime_error("Exception: device buffer is null");
    buff_read_idx = 0;
    buff_write_idx = 0;
  } else {
    buff_read_idx++;
    if (buff_read_idx < no_of_samp) return true;
  }

  ret = iio_buffer_refill(m_dev_buffer);
  if ((ret == 0) || (ret == -110)) {
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp_iiowrapper"), "no samples available yet, retrying ret = %ld", ret);
    return false;
  }
  if (ret < 0) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp_iiowrapper"), "buffer refill error status %ld", ret);
    stopBufferAcquisition();
    return false;
  }
  iio_buffer_foreach_sample(m_dev_buffer, demux_sample, NULL);
  buff_read_idx = 0;
  return true;
}

double IIOWrapper::getBuffLinearAccelerationX()
{
  return (int32_t)buff_data[CHAN_ACCEL_X][buff_read_idx] * m_scale_accel_x;
}

double IIOWrapper::getBuffLinearAccelerationY()
{
  return (int32_t)buff_data[CHAN_ACCEL_Y][buff_read_idx] * m_scale_accel_y;
}

double IIOWrapper::getBuffLinearAccelerationZ()
{
  return (int32_t)buff_data[CHAN_ACCEL_Z][buff_read_idx] * m_scale_accel_z;
}

double IIOWrapper::getBuffAngularVelocityX()
{
  return (int32_t)buff_data[CHAN_GYRO_X][buff_read_idx] * m_scale_anglvel_x;
}

double IIOWrapper::getBuffAngularVelocityY()
{
  return (int32_t)buff_data[CHAN_GYRO_Y][buff_read_idx] * m_scale_anglvel_y;
}

double IIOWrapper::getBuffAngularVelocityZ()
{
  return (int32_t)buff_data[CHAN_GYRO_Z][buff_read_idx] * m_scale_anglvel_z;
}

#ifdef ADIS_HAS_DELTA_BURST
double IIOWrapper::getBuffDeltaAngleX()
{
  if (!has_delta_channels) {
    double result;
    if (getConvertedDeltaAngleXFromDebug(result))
      return result;
    else
      return 0.0;
  }

  return (int32_t)buff_data[CHAN_DELTA_ANGL_X][buff_read_idx] * m_scale_deltaangl_x;
}

double IIOWrapper::getBuffDeltaAngleY()
{
  if (!has_delta_channels) {
    double result;
    if (getConvertedDeltaAngleYFromDebug(result))
      return result;
    else
      return 0.0;
  }
  return (int32_t)buff_data[CHAN_DELTA_ANGL_Y][buff_read_idx] * m_scale_deltaangl_y;
}

double IIOWrapper::getBuffDeltaAngleZ()
{
  if (!has_delta_channels) {
    double result;
    if (getConvertedDeltaAngleZFromDebug(result))
      return result;
    else
      return 0.0;
  }
  return (int32_t)buff_data[CHAN_DELTA_ANGL_Z][buff_read_idx] * m_scale_deltaangl_z;
}

double IIOWrapper::getBuffDeltaVelocityX()
{
  if (!has_delta_channels) {
    double result;
    if (getConvertedDeltaVelocityXFromDebug(result))
      return result;
    else
      return 0.0;
  }
  return (int32_t)buff_data[CHAN_DELTA_VEL_X][buff_read_idx] * m_scale_deltavelocity_x;
}

double IIOWrapper::getBuffDeltaVelocityY()
{
  if (!has_delta_channels) {
    double result;
    if (getConvertedDeltaVelocityYFromDebug(result))
      return result;
    else
      return 0.0;
  }
  return (int32_t)buff_data[CHAN_DELTA_VEL_Y][buff_read_idx] * m_scale_deltavelocity_y;
}

double IIOWrapper::getBuffDeltaVelocityZ()
{
  if (!has_delta_channels) {
    double result;
    if (getConvertedDeltaVelocityZFromDebug(result))
      return result;
    else
      return 0.0;
  }
  return (int32_t)buff_data[CHAN_DELTA_VEL_Z][buff_read_idx] * m_scale_deltavelocity_z;
}
#endif

double IIOWrapper::getBuffTemperature()
{
  return (int32_t)buff_data[CHAN_TEMP][buff_read_idx] * m_scale_temp / 1000.0;
}

void IIOWrapper::getBuffSampleTimestamp(int32_t & sec, uint32_t & nanosec)
{
  if (has_timestamp_channel) {
    uint16_t timestamp_0_15 = buff_data[CHAN_DATA_TIMESTAMP][buff_read_idx];
    uint16_t timestamp_16_31 = buff_data[CHAN_DATA_TIMESTAMP][buff_read_idx] >> 16;
    uint16_t timestamp_32_47 = buff_data[CHAN_DATA_TIMESTAMP + 1][buff_read_idx];
    uint16_t timestamp_48_63 = buff_data[CHAN_DATA_TIMESTAMP + 1][buff_read_idx] >> 16;

    uint64_t timestamp = ((uint64_t)timestamp_48_63 << 48) | ((uint64_t)timestamp_32_47 << 32) |
                         ((uint64_t)timestamp_16_31 << 16) | timestamp_0_15;

    sec = timestamp / 1000000000;
    nanosec = timestamp % 1000000000;
  } else {
    sec = 0;
    nanosec = 0;
  }
}

bool IIOWrapper::getConvertedLinearAccelerationX(double & result)
{
  long long valueRaw;

  if (!m_channel_accel_x) return false;

  int ret = iio_channel_attr_read_longlong(m_channel_accel_x, "raw", &valueRaw);

  result = valueRaw * m_scale_accel_x;
  return (ret == 0);
}

bool IIOWrapper::getConvertedLinearAccelerationY(double & result)
{
  long long valueRaw;

  if (!m_channel_accel_y) return false;

  int ret = iio_channel_attr_read_longlong(m_channel_accel_y, "raw", &valueRaw);

  result = valueRaw * m_scale_accel_y;
  return (ret == 0);
}

bool IIOWrapper::getConvertedLinearAccelerationZ(double & result)
{
  long long valueRaw;

  if (!m_channel_accel_z) return false;

  int ret = iio_channel_attr_read_longlong(m_channel_accel_z, "raw", &valueRaw);

  result = valueRaw * m_scale_accel_z;
  return (ret == 0);
}

bool IIOWrapper::getConvertedAngularVelocityX(double & result)
{
  long long valueRaw;

  if (!m_channel_anglvel_x) return false;

  int ret = iio_channel_attr_read_longlong(m_channel_anglvel_x, "raw", &valueRaw);

  result = valueRaw * m_scale_anglvel_x;
  return (ret == 0);
}

bool IIOWrapper::getConvertedAngularVelocityY(double & result)
{
  long long valueRaw;

  if (!m_channel_anglvel_y) return false;

  int ret = iio_channel_attr_read_longlong(m_channel_anglvel_y, "raw", &valueRaw);

  result = valueRaw * m_scale_anglvel_y;
  return (ret == 0);
}

bool IIOWrapper::getConvertedAngularVelocityZ(double & result)
{
  long long valueRaw;

  if (!m_channel_anglvel_z) return false;

  int ret = iio_channel_attr_read_longlong(m_channel_anglvel_z, "raw", &valueRaw);

  result = valueRaw * m_scale_anglvel_z;
  return (ret == 0);
}

bool IIOWrapper::getRawDeltaAngleXFromDebug(int32_t & result)
{
  uint32_t reg_low;
  uint32_t reg_high;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DELTANG_X_LOW_REG, &reg_low);
  if (ret) return false;

  ret = iio_device_reg_read(m_dev, ADIS_DELTANG_X_OUT_REG, &reg_high);
  if (ret) return false;

  result = reg_high << 16 | reg_low;

  return true;
}

bool IIOWrapper::getConvertedDeltaAngleXFromDebug(double & result)
{
  int32_t reg_val = 0;

  bool ret = getRawDeltaAngleXFromDebug(reg_val);

  result = reg_val * m_scale_deltaangl_x;

  return ret;
}

bool IIOWrapper::getConvertedDeltaAngleX(double & result)
{
  long long valueRaw;

  if (!m_channel_deltaangl_x) return getConvertedDeltaAngleXFromDebug(result);

  int ret = iio_channel_attr_read_longlong(m_channel_deltaangl_x, "raw", &valueRaw);

  result = valueRaw * m_scale_deltaangl_x;
  return (ret == 0);
}

bool IIOWrapper::getRawDeltaAngleYFromDebug(int32_t & result)
{
  uint32_t reg_low;
  uint32_t reg_high;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DELTANG_Y_LOW_REG, &reg_low);
  if (ret) return false;

  ret = iio_device_reg_read(m_dev, ADIS_DELTANG_Y_OUT_REG, &reg_high);
  if (ret) return false;

  result = reg_high << 16 | reg_low;

  return true;
}

bool IIOWrapper::getConvertedDeltaAngleYFromDebug(double & result)
{
  int32_t reg_val = 0;

  bool ret = getRawDeltaAngleYFromDebug(reg_val);

  result = reg_val * m_scale_deltaangl_y;

  return ret;
}

bool IIOWrapper::getConvertedDeltaAngleY(double & result)
{
  long long valueRaw;

  if (!m_channel_deltaangl_y) return getConvertedDeltaAngleYFromDebug(result);

  int ret = iio_channel_attr_read_longlong(m_channel_deltaangl_y, "raw", &valueRaw);

  result = valueRaw * m_scale_deltaangl_y;
  return (ret == 0);
}

bool IIOWrapper::getRawDeltaAngleZFromDebug(int32_t & result)
{
  uint32_t reg_low;
  uint32_t reg_high;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DELTANG_Z_LOW_REG, &reg_low);
  if (ret) return false;

  ret = iio_device_reg_read(m_dev, ADIS_DELTANG_Z_OUT_REG, &reg_high);
  if (ret) return false;

  result = reg_high << 16 | reg_low;

  return true;
}

bool IIOWrapper::getConvertedDeltaAngleZFromDebug(double & result)
{
  int32_t reg_val = 0;

  bool ret = getRawDeltaAngleZFromDebug(reg_val);

  result = reg_val * m_scale_deltaangl_z;

  return ret;
}

bool IIOWrapper::getConvertedDeltaAngleZ(double & result)
{
  long long valueRaw;

  if (!m_channel_deltaangl_z) return getConvertedDeltaAngleZFromDebug(result);

  int ret = iio_channel_attr_read_longlong(m_channel_deltaangl_z, "raw", &valueRaw);

  result = valueRaw * m_scale_deltaangl_z;
  return (ret == 0);
}

bool IIOWrapper::getRawDeltaVelocityXFromDebug(int32_t & result)
{
  uint32_t reg_low;
  uint32_t reg_high;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DELTVEL_X_LOW_REG, &reg_low);
  if (ret) return false;

  ret = iio_device_reg_read(m_dev, ADIS_DELTVEL_X_OUT_REG, &reg_high);
  if (ret) return false;

  result = reg_high << 16 | reg_low;

  return true;
}

bool IIOWrapper::getConvertedDeltaVelocityXFromDebug(double & result)
{
  int32_t reg_val = 0;

  bool ret = getRawDeltaVelocityXFromDebug(reg_val);

  result = reg_val * m_scale_deltavelocity_x;

  return ret;
}

bool IIOWrapper::getConvertedDeltaVelocityX(double & result)
{
  long long valueRaw;

  if (!m_channel_deltavelocity_x) return getConvertedDeltaVelocityXFromDebug(result);

  int ret = iio_channel_attr_read_longlong(m_channel_deltavelocity_x, "raw", &valueRaw);

  result = valueRaw * m_scale_deltavelocity_x;
  return (ret == 0);
}

bool IIOWrapper::getRawDeltaVelocityYFromDebug(int32_t & result)
{
  uint32_t reg_low;
  uint32_t reg_high;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DELTVEL_Y_LOW_REG, &reg_low);
  if (ret) return false;

  ret = iio_device_reg_read(m_dev, ADIS_DELTVEL_Y_OUT_REG, &reg_high);
  if (ret) return false;

  result = reg_high << 16 | reg_low;

  return true;
}

bool IIOWrapper::getConvertedDeltaVelocityYFromDebug(double & result)
{
  int32_t reg_val = 0;

  bool ret = getRawDeltaVelocityYFromDebug(reg_val);

  result = reg_val * m_scale_deltavelocity_y;

  return ret;
}

bool IIOWrapper::getConvertedDeltaVelocityY(double & result)
{
  long long valueRaw;

  if (!m_channel_deltavelocity_y) return getConvertedDeltaVelocityYFromDebug(result);

  int ret = iio_channel_attr_read_longlong(m_channel_deltavelocity_y, "raw", &valueRaw);

  result = valueRaw * m_scale_deltavelocity_y;
  return (ret == 0);
}

bool IIOWrapper::getRawDeltaVelocityZFromDebug(int32_t & result)
{
  uint32_t reg_low;
  uint32_t reg_high;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DELTVEL_Z_LOW_REG, &reg_low);
  if (ret) return false;

  ret = iio_device_reg_read(m_dev, ADIS_DELTVEL_Z_OUT_REG, &reg_high);
  if (ret) return false;

  result = reg_high << 16 | reg_low;

  return true;
}

bool IIOWrapper::getConvertedDeltaVelocityZFromDebug(double & result)
{
  int32_t reg_val = 0;

  bool ret = getRawDeltaVelocityZFromDebug(reg_val);

  result = reg_val * m_scale_deltavelocity_z;

  return ret;
}

bool IIOWrapper::getConvertedDeltaVelocityZ(double & result)
{
  long long valueRaw;

  if (!m_channel_deltavelocity_z) return getConvertedDeltaVelocityZFromDebug(result);

  int ret = iio_channel_attr_read_longlong(m_channel_deltavelocity_z, "raw", &valueRaw);

  result = valueRaw * m_scale_deltavelocity_z;
  return (ret == 0);
}

bool IIOWrapper::getConvertedTemperature(double & result)
{
  long long valueRaw;

  if (!m_channel_temp) return false;

  int ret = iio_channel_attr_read_longlong(m_channel_temp, "raw", &valueRaw);

  result = valueRaw * m_scale_temp / 1000.0;
  return (ret == 0);
}

bool IIOWrapper::anglvel_x_calibbias(int32_t & result)
{
  long long valuel;

  if (!m_channel_anglvel_x) return false;

  int ret = iio_channel_attr_read_longlong(m_channel_anglvel_x, "calibbias", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_anglvel_calibbias_x(int32_t val)
{
  if (!m_channel_anglvel_x) return false;

  return (iio_channel_attr_write_longlong(m_channel_anglvel_x, "calibbias", val) == 0);
}

bool IIOWrapper::anglvel_y_calibbias(int32_t & result)
{
  long long valuel;

  if (!m_channel_anglvel_y) return false;

  int ret = iio_channel_attr_read_longlong(m_channel_anglvel_y, "calibbias", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_anglvel_calibbias_y(int32_t val)
{
  if (!m_channel_anglvel_y) return false;

  return (iio_channel_attr_write_longlong(m_channel_anglvel_y, "calibbias", val) == 0);
}

bool IIOWrapper::anglvel_z_calibbias(int32_t & result)
{
  long long valuel;

  if (!m_channel_anglvel_z) return false;

  int ret = iio_channel_attr_read_longlong(m_channel_anglvel_z, "calibbias", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_anglvel_calibbias_z(int32_t val)
{
  if (!m_channel_anglvel_z) return false;

  return (iio_channel_attr_write_longlong(m_channel_anglvel_z, "calibbias", val) == 0);
}

bool IIOWrapper::accel_x_calibbias(int32_t & result)
{
  long long valuel;

  if (!m_channel_accel_x) return false;

  int ret = iio_channel_attr_read_longlong(m_channel_accel_x, "calibbias", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_accel_calibbias_x(int32_t val)
{
  if (!m_channel_accel_x) return false;

  return (iio_channel_attr_write_longlong(m_channel_accel_x, "calibbias", val) == 0);
}

bool IIOWrapper::accel_y_calibbias(int32_t & result)
{
  long long valuel;

  if (!m_channel_accel_y) return false;

  int ret = iio_channel_attr_read_longlong(m_channel_accel_y, "calibbias", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_accel_calibbias_y(int32_t val)
{
  if (!m_channel_accel_y) return false;

  return (iio_channel_attr_write_longlong(m_channel_accel_y, "calibbias", val) == 0);
}

bool IIOWrapper::accel_z_calibbias(int32_t & result)
{
  long long valuel;

  if (!m_channel_accel_z) return false;

  int ret = iio_channel_attr_read_longlong(m_channel_accel_z, "calibbias", &valuel);

  result = valuel;
  return (ret == 0);
}

bool IIOWrapper::update_accel_calibbias_z(int32_t val)
{
  if (!m_channel_accel_z) return false;

  return (iio_channel_attr_write_longlong(m_channel_accel_z, "calibbias", val) == 0);
}

bool IIOWrapper::sampling_frequency(double * result)
{
  int ret;

  if (!m_dev) return false;

  ret = iio_device_attr_read_double(m_dev, "sampling_frequency", result);
  if (ret) return false;

  samp_freq = *result;

  return true;
}

bool IIOWrapper::update_sampling_frequency(double val)
{
  int ret;

  if (!m_dev) return false;

  ret = iio_device_attr_write_double(m_dev, "sampling_frequency", val);
  if (ret) return false;

  samp_freq = val;

  return true;
}

#ifdef ADIS_SNSR_INIT_FAIL
bool IIOWrapper::diag_sensor_initialization_failure(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_SNSR_INIT_FAIL) >> ADIS_SNSR_INIT_FAIL_POS;

  result = reg_val;

  return true;
}
#endif

bool IIOWrapper::diag_data_path_overrun(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_DATA_PATH_OVERRUN) >> ADIS_DATA_PATH_OVERRUN_POS;
  result = reg_val;

  return true;
}

bool IIOWrapper::diag_flash_memory_update_error(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_FLS_MEM_UPDATE_FAIL) >> ADIS_FLS_MEM_UPDATE_FAIL_POS;
  result = reg_val;

  return true;
}

bool IIOWrapper::diag_spi_communication_error(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_SPI_COMM_ERR) >> ADIS_SPI_COMM_ERR_POS;
  result = reg_val;

  return true;
}

bool IIOWrapper::diag_standby_mode(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_STDBY_MODE) >> ADIS_STDBY_MODE_POS;
  result = reg_val;

  return true;
}

bool IIOWrapper::diag_sensor_self_test_error(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & AIDS_SNSR_FAIL) >> AIDS_SNSR_FAIL_POS;
  result = reg_val;

  return true;
}

bool IIOWrapper::diag_flash_memory_test_error(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_MEM_FAIL) >> ADIS_MEM_FAIL_POS;
  result = reg_val;

  return true;
}

bool IIOWrapper::diag_clock_error(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_CLK_ERR) >> ADIS_CLK_ERR_POS;
  result = reg_val;

  return true;
}

#ifdef ADIS_GYRO1_FAIL
bool IIOWrapper::diag_gyroscope1_self_test_error(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_GYRO1_FAIL) >> ADIS_GYRO1_FAIL_POS;
  result = reg_val;

  return true;
}
#endif

#ifdef ADIS_GYRO2_FAIL
bool IIOWrapper::diag_gyroscope2_self_test_error(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_GYRO2_FAIL) >> ADIS_GYRO2_FAIL_POS;
  result = reg_val;

  return true;
}
#endif

#ifdef ADIS_ACCEL_FAIL
bool IIOWrapper::diag_acceleration_self_test_error(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_ACCEL_FAIL) >> ADIS_ACCEL_FAIL_POS;
  result = reg_val;

  return true;
}
#endif

#ifdef ADIS_GYRO_X_FAIL
bool IIOWrapper::diag_x_axis_gyroscope_failure(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_GYRO_X_FAIL) >> ADIS_GYRO_X_FAIL_POS;
  result = reg_val;

  return true;
}
#endif

#ifdef ADIS_GYRO_Y_FAIL
bool IIOWrapper::diag_y_axis_gyroscope_failure(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_GYRO_Y_FAIL) >> ADIS_GYRO_Y_FAIL_POS;
  result = reg_val;

  return true;
}
#endif

#ifdef ADIS_GYRO_Z_FAIL
bool IIOWrapper::diag_z_axis_gyroscope_failure(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_GYRO_Z_FAIL) >> ADIS_GYRO_Z_FAIL_POS;
  result = reg_val;

  return true;
}
#endif

#ifdef ADIS_ACCEL_X_FAIL
bool IIOWrapper::diag_x_axis_accelerometer_failure(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_ACCEL_X_FAIL) >> ADIS_ACCEL_X_FAIL_POS;
  result = reg_val;

  return true;
}
#endif

#ifdef ADIS_ACCEL_Y_FAIL
bool IIOWrapper::diag_y_axis_accelerometer_failure(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_ACCEL_Y_FAIL) >> ADIS_ACCEL_Y_FAIL_POS;
  result = reg_val;

  return true;
}
#endif

#ifdef ADIS_ACCEL_Z_FAIL
bool IIOWrapper::diag_z_axis_accelerometer_failure(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_ACCEL_Z_FAIL) >> ADIS_ACCEL_Z_FAIL_POS;
  result = reg_val;

  return true;
}
#endif

#ifdef ADIS_ADUC_MCU_FAULT
bool IIOWrapper::diag_aduc_mcu_fault(bool & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_DIAG_STAT_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_ADUC_MCU_FAULT) >> ADIS_ADUC_MCU_FAULT_POS;
  result = reg_val;

  return true;
}
#endif

bool IIOWrapper::diag_flash_memory_write_count_exceeded_error(bool & result)
{
  uint32_t reg_val;
  bool ret = flash_counter(reg_val);
  if (!ret) return false;

  result = reg_val > ADIS_FLS_MEM_ENDURANCE;
  return true;
}

bool IIOWrapper::filter_low_pass_3db_frequency(uint32_t & result)
{
  long long valuel;

  if (!m_dev) return false;

  int ret = iio_device_attr_read_longlong(m_dev, "filter_low_pass_3db_frequency", &valuel);
  if (ret) return false;

  result = valuel;
  return true;
}

bool IIOWrapper::update_filter_low_pass_3db_frequency(uint32_t val)
{
  if (!m_dev) return false;

  return (iio_device_attr_write_longlong(m_dev, "filter_low_pass_3db_frequency", val) == 0);
}

bool IIOWrapper::gyroscope_measurement_range(std::string & result)
{
  uint32_t reg_val;

  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_RANG_MDL_ADDR, &reg_val);
  if (ret) return false;

  reg_val = (reg_val & ADIS_GYRO_MEAS_RANG) >> ADIS_GYRO_MEAS_RANG_POS;

  switch (reg_val) {
    case 0:
      result = "+/-125_degrees_per_sec";
      return true;
    case 1:
      result = "+/-500_degrees_per_sec";
      return true;
    case 3:
      result = "+/-2000_degrees_per_sec";
      return true;
    default:
      return false;
  }
}

#ifdef ADIS_SENS_BW
bool IIOWrapper::internal_sensor_bandwidth(uint32_t & result)
{
  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_MSC_CTRL_ADDR, &result);
  if (ret) return false;

  result = (result & ADIS_SENS_BW) >> ADIS_SENS_BW_POS;
  return true;
}

bool IIOWrapper::update_internal_sensor_bandwidth(uint32_t val)
{
  return updateField(ADIS_MSC_CTRL_ADDR, val << ADIS_SENS_BW_POS, ADIS_SENS_BW);
}
#endif

bool IIOWrapper::point_of_percussion_alignment(uint32_t & result)
{
  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_MSC_CTRL_ADDR, &result);
  if (ret) return false;

  result = (result & ADIS_PT_OF_PERG_ALGNMNT) >> ADIS_PT_OF_PERG_ALGNMNT_POS;
  return true;
}

bool IIOWrapper::update_point_of_percussion_alignment(uint32_t val)
{
  return updateField(
    ADIS_MSC_CTRL_ADDR, val << ADIS_PT_OF_PERG_ALGNMNT_POS, ADIS_PT_OF_PERG_ALGNMNT);
}

bool IIOWrapper::linear_acceleration_compensation(uint32_t & result)
{
  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_MSC_CTRL_ADDR, &result);
  if (ret) return false;

  result = (result & ADIS_LN_ACCL_COMP) >> ADIS_LN_ACCL_COMP_POS;
  return true;
}

bool IIOWrapper::update_linear_acceleration_compensation(uint32_t val)
{
  return updateField(ADIS_MSC_CTRL_ADDR, val << ADIS_LN_ACCL_COMP_POS, ADIS_LN_ACCL_COMP);
}

#ifdef ADIS_NULL_CNFG_ADDR
bool IIOWrapper::bias_correction_time_base_control(uint32_t & result)
{
  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_NULL_CNFG_ADDR, &result);
  if (ret) return false;

  result = (result & ADIS_TIME_BASE_CONTROL) >> ADIS_TIME_BASE_CONTROL_POS;
  return true;
}

bool IIOWrapper::update_bias_correction_time_base_control(uint32_t val)
{
  return updateField(
    ADIS_NULL_CNFG_ADDR, val << ADIS_TIME_BASE_CONTROL_POS, ADIS_TIME_BASE_CONTROL);
}

bool IIOWrapper::x_axis_gyroscope_bias_correction_enable(uint32_t & result)
{
  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_NULL_CNFG_ADDR, &result);
  if (ret) return false;

  result = (result & ADIS_X_AXIS_GYRO_BIAS_CORR_EN) >> ADIS_X_AXIS_GYRO_BIAS_CORR_EN_POS;
  return true;
}

bool IIOWrapper::update_x_axis_gyroscope_bias_correction_enable(uint32_t val)
{
  return updateField(
    ADIS_NULL_CNFG_ADDR, val << ADIS_X_AXIS_GYRO_BIAS_CORR_EN_POS, ADIS_X_AXIS_GYRO_BIAS_CORR_EN);
}

bool IIOWrapper::y_axis_gyroscope_bias_correction_enable(uint32_t & result)
{
  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_NULL_CNFG_ADDR, &result);
  if (ret) return false;

  result = (result & ADIS_Y_AXIS_GYRO_BIAS_CORR_EN) >> ADIS_Y_AXIS_GYRO_BIAS_CORR_EN_POS;
  return true;
}

bool IIOWrapper::update_y_axis_gyroscope_bias_correction_enable(uint32_t val)
{
  return updateField(
    ADIS_NULL_CNFG_ADDR, val << ADIS_Y_AXIS_GYRO_BIAS_CORR_EN_POS, ADIS_Y_AXIS_GYRO_BIAS_CORR_EN);
}

bool IIOWrapper::z_axis_gyroscope_bias_correction_enable(uint32_t & result)
{
  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_NULL_CNFG_ADDR, &result);
  if (ret) return false;

  result = (result & ADIS_Z_AXIS_GYRO_BIAS_CORR_EN) >> ADIS_Z_AXIS_GYRO_BIAS_CORR_EN_POS;
  return true;
}

bool IIOWrapper::update_z_axis_gyroscope_bias_correction_enable(uint32_t val)
{
  return updateField(
    ADIS_NULL_CNFG_ADDR, val << ADIS_Z_AXIS_GYRO_BIAS_CORR_EN_POS, ADIS_Z_AXIS_GYRO_BIAS_CORR_EN);
}

bool IIOWrapper::x_axis_accelerometer_bias_correction_enable(uint32_t & result)
{
  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_NULL_CNFG_ADDR, &result);
  if (ret) return false;

  result = (result & ADIS_X_AXIS_ACCEL_BIAS_CORR_EN) >> ADIS_X_AXIS_ACCEL_BIAS_CORR_EN_POS;
  return true;
}

bool IIOWrapper::update_x_axis_accelerometer_bias_correction_enable(uint32_t val)
{
  return updateField(
    ADIS_NULL_CNFG_ADDR, val << ADIS_X_AXIS_ACCEL_BIAS_CORR_EN_POS, ADIS_X_AXIS_ACCEL_BIAS_CORR_EN);
}

bool IIOWrapper::y_axis_accelerometer_bias_correction_enable(uint32_t & result)
{
  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_NULL_CNFG_ADDR, &result);
  if (ret) return false;

  result = (result & ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN) >> ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN_POS;
  return true;
}

bool IIOWrapper::update_y_axis_accelerometer_bias_correction_enable(uint32_t val)
{
  return updateField(
    ADIS_NULL_CNFG_ADDR, val << ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN_POS, ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN);
}

bool IIOWrapper::z_axis_accelerometer_bias_correction_enable(uint32_t & result)
{
  if (!m_dev) return false;

  int ret = iio_device_reg_read(m_dev, ADIS_NULL_CNFG_ADDR, &result);
  if (ret) return false;

  result = (result & ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN) >> ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN_POS;
  return true;
}

bool IIOWrapper::update_z_axis_accelerometer_bias_correction_enable(uint32_t val)
{
  return updateField(
    ADIS_NULL_CNFG_ADDR, val << ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN_POS, ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN);
}
#endif

#ifdef ADIS_BIAS_CORRECTION_UPDATE
bool IIOWrapper::bias_correction_update()
{
  if (!m_dev) return false;

  uint16_t cmd = ADIS_BIAS_CORRECTION_UPDATE;
  return (iio_device_reg_write(m_dev, ADIS_GLOB_CMD_ADDR, cmd) == 0);
}
#endif

bool IIOWrapper::factory_calibration_restore()
{
  if (!m_dev) return false;

  uint16_t cmd = ADIS_FACTORY_CALIBRATION_RESTORE;
  return (iio_device_reg_write(m_dev, ADIS_GLOB_CMD_ADDR, cmd) == 0);
}

bool IIOWrapper::sensor_self_test()
{
  if (!m_dev) return false;

  uint16_t cmd = ADIS_SENSOR_SELF_TEST;
  return (iio_device_reg_write(m_dev, ADIS_GLOB_CMD_ADDR, cmd) == 0);
}

bool IIOWrapper::flash_memory_update()
{
  if (!m_dev) return false;

  uint16_t cmd = ADIS_FLASH_MEMORY_UPDATE;
  return (iio_device_reg_write(m_dev, ADIS_GLOB_CMD_ADDR, cmd) == 0);
}

bool IIOWrapper::flash_memory_test()
{
  if (!m_dev) return false;

  uint16_t cmd = ADIS_FLASH_MEMORY_TEST;
  return (iio_device_reg_write(m_dev, ADIS_GLOB_CMD_ADDR, cmd) == 0);
}

bool IIOWrapper::software_reset()
{
  if (!m_dev) return false;

  uint16_t cmd = ADIS_SOFTWARE_RESET_CMD;
  return (iio_device_reg_write(m_dev, ADIS_GLOB_CMD_ADDR, cmd) == 0);
}

bool IIOWrapper::firmware_revision(std::string & result)
{
  if (!m_dev) return false;

  char valuec[32];
  int ret = iio_device_debug_attr_read(m_dev, "firmware_revision", valuec, 32);
  if (ret < 0) return false;

  result = valuec;
  return true;
}

bool IIOWrapper::firmware_date(std::string & result)
{
  if (!m_dev) return false;

  char valuec[32];
  int ret = iio_device_debug_attr_read(m_dev, "firmware_date", valuec, 32);
  if (ret < 0) return false;

  result = valuec;
  return true;
}

bool IIOWrapper::product_id(uint32_t & result)
{
  if (!m_dev) return false;

  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "product_id", &valuel);
  if (ret) return false;

  result = valuel;
  return true;
}

bool IIOWrapper::serial_number(uint32_t & result)
{
  if (!m_dev) return false;

  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "serial_number", &valuel);
  if (ret) return false;

  result = valuel;
  return true;
}

bool IIOWrapper::flash_counter(uint32_t & result)
{
  if (!m_dev) return false;

  long long valuel;
  int ret = iio_device_debug_attr_read_longlong(m_dev, "flash_count", &valuel);
  if (ret) return false;

  result = valuel;
  return true;
}

double IIOWrapper::get_scale_accel() { return m_scale_accel_x; }

double IIOWrapper::get_scale_anglvel() { return m_scale_anglvel_x; }

double IIOWrapper::get_scale_deltavelocity() { return m_scale_deltavelocity_x; }

double IIOWrapper::get_scale_deltaangl() { return m_scale_deltaangl_x; }

double IIOWrapper::get_scale_temp() { return m_scale_temp / 1000.0; }
