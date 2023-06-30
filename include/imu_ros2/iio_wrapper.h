/*******************************************************************************
 *   @file   iio_wrapper.h
 *   @brief  Wrapper for iio library
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

#ifndef IIO_WRAPPER_H
#define IIO_WRAPPER_H

#include <iio.h>

#include <mutex>
#include <string>

#include "setting_declarations.h"

class IIOWrapper
{
public:
  IIOWrapper();
  ~IIOWrapper();

  bool updateBuffer();
  void stopBufferAcquisition();

  double getBuffLinearAccelerationX();
  double getBuffLinearAccelerationY();
  double getBuffLinearAccelerationZ();

  double getBuffAngularVelocityX();
  double getBuffAngularVelocityY();
  double getBuffAngularVelocityZ();

  double getBuffDeltaVelocityX();
  double getBuffDeltaVelocityY();
  double getBuffDeltaVelocityZ();

  double getBuffDeltaAngleX();
  double getBuffDeltaAngleY();
  double getBuffDeltaAngleZ();

  double getBuffTemperature();
  uint32_t getBuffSampleCount();

  bool getRegLinearAccelerationX(double & result);
  bool getRegLinearAccelerationY(double & result);
  bool getRegLinearAccelerationZ(double & result);

  bool getRegAngularVelocityX(double & result);
  bool getRegAngularVelocityY(double & result);
  bool getRegAngularVelocityZ(double & result);

  bool getRegDeltaAngleX(double & result);
  bool getRegDeltaAngleY(double & result);
  bool getRegDeltaAngleZ(double & result);

  bool getRegDeltaVelocityX(double & result);
  bool getRegDeltaVelocityY(double & result);
  bool getRegDeltaVelocityZ(double & result);

  bool getRegTemperature(double & result);

  bool anglvel_x_calibbias(int32_t & result);
  bool update_anglvel_calibbias_x(int32_t val);
  bool anglvel_y_calibbias(int32_t & result);
  bool update_anglvel_calibbias_y(int32_t val);
  bool anglvel_z_calibbias(int32_t & result);
  bool update_anglvel_calibbias_z(int32_t val);

  bool accel_x_calibbias(int32_t & result);
  bool update_accel_calibbias_x(int32_t val);
  bool accel_y_calibbias(int32_t & result);
  bool update_accel_calibbias_y(int32_t val);
  bool accel_z_calibbias(int32_t & result);
  bool update_accel_calibbias_z(int32_t val);

  bool sampling_frequency(double * result);
  bool update_sampling_frequency(double val);

  bool diag_sensor_initialization_failure(bool & result);
  bool diag_data_path_overrun(bool & result);
  bool diag_flash_memory_update_error(bool & result);
  bool diag_spi_communication_error(bool & result);
  bool diag_standby_mode(bool & result);
  bool diag_sensor_self_test_error(bool & result);
  bool diag_flash_memory_test_error(bool & result);
  bool diag_clock_error(bool & result);
  bool diag_gyroscope1_self_test_error(bool & result);
  bool diag_gyroscope2_self_test_error(bool & result);
  bool diag_acceleration_self_test_error(bool & result);
  bool diag_x_axis_gyroscope_failure(bool & result);
  bool diag_y_axis_gyroscope_failure(bool & result);
  bool diag_z_axis_gyroscope_failure(bool & result);
  bool diag_x_axis_accelerometer_failure(bool & result);
  bool diag_y_axis_accelerometer_failure(bool & result);
  bool diag_z_axis_accelerometer_failure(bool & result);
  bool diag_aduc_mcu_fault(bool & result);
  bool diag_checksum_error_flag(bool & result);
  bool diag_flash_memory_write_count_exceeded_error(bool & result);

  bool lost_samples_count(uint32_t & result);

  bool fifo_enable(uint32_t & result);
  bool update_fifo_enable(uint32_t val);
  bool fifo_overflow_behavior(uint32_t & result);
  bool update_fifo_overflow_behavior(uint32_t val);
  bool fifo_watermark_interrupt_enable(uint32_t & result);
  bool update_fifo_watermark_interrupt_enable(uint32_t val);
  bool fifo_watermark_interrupt_polarity(uint32_t & result);
  bool update_fifo_watermark_interrupt_polarity(uint32_t val);
  bool fifo_watermark_threshold_level(uint32_t & result);
  bool update_fifo_watermark_threshold_level(uint32_t val);
  bool filter_size(uint32_t & result);
  bool update_filter_size(uint32_t val);
  bool gyroscope_measurement_range(std::string & result);
  bool data_ready_polarity(uint32_t & result);
  bool update_data_ready_polarity(uint32_t val);
  bool sync_polarity(uint32_t & result);
  bool update_sync_polarity(uint32_t val);
  bool sync_mode_select(uint32_t & result);
  bool update_sync_mode_select(uint32_t val);
  bool internal_sensor_bandwidth(uint32_t & result);
  bool update_internal_sensor_bandwidth(uint32_t val);
  bool point_of_percussion_alignment(uint32_t & result);
  bool update_point_of_percussion_alignment(uint32_t val);
  bool update_linear_acceleration_compensation(uint32_t val);
  bool linear_acceleration_compensation(uint32_t & result);
  bool burst_data_selection(uint32_t & result);
  bool update_burst_data_selection(uint32_t val);
  bool burst_size_selection(uint32_t & result);
  bool update_burst_size_selection(uint32_t val);
  bool timestamp32(uint32_t & result);
  bool update_timestamp32(uint32_t val);
  bool internal_sync_enable_4khz(uint32_t & result);
  bool update_internal_sync_enable_4khz(uint32_t val);
  bool sync_signal_scale(uint32_t & result);
  bool update_sync_signal_scale(uint32_t val);
  bool decimation_filter(uint32_t & result);
  bool update_decimation_filter(uint32_t val);
  bool bias_correction_time_base_control(uint32_t & result);
  bool update_bias_correction_time_base_control(uint32_t val);
  bool x_axis_gyroscope_bias_correction_enable(uint32_t & result);
  bool update_x_axis_gyroscope_bias_correction_enable(uint32_t val);
  bool y_axis_gyroscope_bias_correction_enable(uint32_t & result);
  bool update_y_axis_gyroscope_bias_correction_enable(uint32_t val);
  bool z_axis_gyroscope_bias_correction_enable(uint32_t & result);
  bool update_z_axis_gyroscope_bias_correction_enable(uint32_t val);
  bool x_axis_accelerometer_bias_correction_enable(uint32_t & result);
  bool update_x_axis_accelerometer_bias_correction_enable(uint32_t val);
  bool y_axis_accelerometer_bias_correction_enable(uint32_t & result);
  bool update_y_axis_accelerometer_bias_correction_enable(uint32_t val);
  bool z_axis_accelerometer_bias_correction_enable(uint32_t & result);
  bool update_z_axis_accelerometer_bias_correction_enable(uint32_t val);

  bool bias_correction_update();
  bool factory_calibration_restore();
  bool sensor_self_test();
  bool flash_memory_update();
  bool flash_memory_test();
  bool fifo_flush();
  bool software_reset();

  bool firmware_revision(std::string & result);
  bool firmware_date(std::string & result);
  bool product_id(uint32_t & result);
  bool serial_number(uint32_t & result);
  bool flash_counter(uint32_t & value);

  double get_scale_accel();
  double get_scale_angvel();
  double get_scale_velocity();
  double get_scale_rot();
  double get_scale_temp();

private:
  static struct iio_context * m_local_context;
  static struct iio_device * m_dev;
  static struct iio_device * m_devtrigger;
  static struct iio_buffer * m_device_buffer;

  static struct iio_channel * m_channel_accel_x;
  static struct iio_channel * m_channel_accel_y;
  static struct iio_channel * m_channel_accel_z;
  static struct iio_channel * m_channel_anglvel_x;
  static struct iio_channel * m_channel_anglvel_y;
  static struct iio_channel * m_channel_anglvel_z;
  static struct iio_channel * m_channel_rot_x;
  static struct iio_channel * m_channel_rot_y;
  static struct iio_channel * m_channel_rot_z;
  static struct iio_channel * m_channel_velocity_x;
  static struct iio_channel * m_channel_velocity_y;
  static struct iio_channel * m_channel_velocity_z;
  static struct iio_channel * m_channel_temp;
  static struct iio_channel * m_channel_count;

  static double m_scale_accel_x;
  static double m_scale_accel_y;
  static double m_scale_accel_z;
  static double m_scale_angvel_x;
  static double m_scale_angvel_y;
  static double m_scale_angvel_z;
  static double m_scale_rot_x;
  static double m_scale_rot_y;
  static double m_scale_rot_z;
  static double m_scale_velocity_x;
  static double m_scale_velocity_y;
  static double m_scale_velocity_z;
  static double m_scale_temp;

public:
  static std::string s_device_name;
  static std::string s_device_trigger_name;
  static IIODeviceName s_device_name_enum;
};

#endif  // IIO_WRAPPER_H
