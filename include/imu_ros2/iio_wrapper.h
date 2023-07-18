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

/**
 * \brief Wrapper class for libiio library
 *
 * This class is a wrapper class for libiio library
 * and offer functions for accesing iio capabilities.
 */
class IIOWrapper
{
public:
  /**
   * \brief Constructor for IIOWrapper.
   *
   * This is the default constructor for class
   *  IIOWrapper.
   *
   */
  IIOWrapper();

  /**
   * \brief Destructor for IIOWrapper.
   *
   * This is a virtual destructor for IIOWrapper.
   *
   */
  ~IIOWrapper();

  /**
   * \brief Update buffer with data.
   *
   * This function update iio buffer with data.
   * If buffer not exists then it will be created,
   * and if buffer exists the call refill function
   * from libiio to receive the data.
   *
   * \return Return true if the buffer was updated with success
   * and false if not.
   */
  bool updateBuffer();

  /**
   * \brief Stop buffer acquisition.
   *
   * This function destroy the buffer.
   *
   */
  void stopBufferAcquisition();

  /**
   * \brief Get linear acceleration on x axis.
   *
   * This function gets the linear acceleration on x axis.
   * The reading is with buffer.
   *
   * \return Return the linear acceleration on x axis.
   */
  double getBuffLinearAccelerationX();

  /**
   * \brief Get linear acceleration on y axis.
   *
   * This function gets the linear acceleration on y axis.
   * The reading is with buffer.
   *
   * \return Return the linear acceleration on y axis.
   */
  double getBuffLinearAccelerationY();

  /**
   * \brief Get linear acceleration on z axis.
   *
   * This function gets the linear acceleration on z axis.
   * The reading is with buffer.
   *
   * \return Return the linear acceleration on z axis.
   */
  double getBuffLinearAccelerationZ();

  /**
   * \brief Get angular velocity on x axis.
   *
   * This function gets the angular velocity on x axis.
   * The reading is with buffer.
   *
   * \return Return the  angular velocity on x axis.
   */
  double getBuffAngularVelocityX();

  /**
   * \brief Get angular velocity on z axis.
   *
   * This function gets the angular velocity on z axis.
   * The reading is with buffer.
   *
   * \return Return the  angular velocity on z axis.
   */
  double getBuffAngularVelocityY();

  /**
   * \brief Get angular velocity on z axis.
   *
   * This function gets the angular velocity on z axis.
   * The reading is with buffer.
   *
   * \return Return the  angular velocity on z axis.
   */
  double getBuffAngularVelocityZ();

  /**
   * \brief Get delta velocity on x axis.
   *
   * This function gets the delta velocity on x axis.
   * The reading is with buffer.
   *
   * \return Return the  delta velocity on x axis.
   */
  double getBuffDeltaVelocityX();

  /**
   * \brief Get delta velocity on y axis.
   *
   * This function gets the delta velocity on y axis.
   * The reading is with buffer.
   *
   * \return Return the  delta velocity on y axis.
   */
  double getBuffDeltaVelocityY();

  /**
   * \brief Get delta velocity on z axis.
   *
   * This function gets the delta velocity on z axis.
   * The reading is with buffer.
   *
   * \return Return the  delta velocity on z axis.
   */
  double getBuffDeltaVelocityZ();

  /**
   * \brief Get delta angle on x axis.
   *
   * This function gets the delta angle on x axis.
   * The reading is with buffer.
   *
   * \return Return the  delta angle on x axis.
   */
  double getBuffDeltaAngleX();

  /**
   * \brief Get delta angle on y axis.
   *
   * This function gets the delta angle on y axis.
   * The reading is with buffer.
   *
   * \return Return the  delta angle on y axis.
   */
  double getBuffDeltaAngleY();

  /**
   * \brief Get delta angle on z axis.
   *
   * This function gets the delta angle on z axis.
   * The reading is with buffer.
   *
   * \return Return the  delta angle on z axis.
   */
  double getBuffDeltaAngleZ();

  /**
   * \brief Get temperature.
   *
   * This function gets the temperature.
   * The reading is with buffer.
   *
   * \return Return the temperature.
   */
  double getBuffTemperature();

  /**
   * \brief Get sample timestamp.
   *
   * This function gets the sample timestamp.
   * The reading is with buffer.
   *
   * \return Return the sample timestamp.
   */
  int64_t getBuffSampleTimestamp();

  /**
   * \brief Get linear acceleration on x axis.
   *
   * This function gets by parameter the linear acceleration on x axis.
   * The reading is with direct register access.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool getRegLinearAccelerationX(double & result);

  /**
   * \brief Get linear acceleration on y axis.
   *
   * This function gets by parameter the linear acceleration on y axis.
   * The reading is with direct register access.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool getRegLinearAccelerationY(double & result);

  /**
   * \brief Get linear acceleration on z axis.
   *
   * This function gets by parameter the linear acceleration on z axis.
   * The reading is with direct register access.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool getRegLinearAccelerationZ(double & result);

  /**
   * \brief Get angular velocity on x axis.
   *
   * This function gets by parameter the angular velocity on x axis.
   * The reading is with direct register access.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool getRegAngularVelocityX(double & result);

  /**
   * \brief Get angular velocity on y axis.
   *
   * This function gets by parameter the angular velocity on y axis.
   * The reading is with direct register access.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool getRegAngularVelocityY(double & result);

  /**
   * \brief Get angular velocity on z axis.
   *
   * This function gets by parameter the angular velocity on z axis.
   * The reading is with direct register access.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool getRegAngularVelocityZ(double & result);

  /**
   * \brief Get delta angle on x axis.
   *
   * This function gets by parameter the delta angle on x axis.
   * The reading is with direct register access.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool getRegDeltaAngleX(double & result);

  /**
   * \brief Get delta angle on y axis.
   *
   * This function gets by parameter the delta angle on y axis.
   * The reading is with direct register access.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool getRegDeltaAngleY(double & result);

  /**
   * \brief Get delta angle on z axis.
   *
   * This function gets by parameter the delta angle on z axis.
   * The reading is with direct register access.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool getRegDeltaAngleZ(double & result);

  /**
   * \brief Get delta velocity on x axis.
   *
   * This function gets by parameter the delta velocity on x axis.
   * The reading is with direct register access.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool getRegDeltaVelocityX(double & result);

  /**
   * \brief Get delta velocity on y axis.
   *
   * This function gets by parameter the delta velocity on y axis.
   * The reading is with direct register access.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool getRegDeltaVelocityY(double & result);

  /**
   * \brief Get delta velocity on z axis.
   *
   * This function gets by parameter the delta velocity on z axis.
   * The reading is with direct register access.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool getRegDeltaVelocityZ(double & result);

  /**
   * \brief Get temperature.
   *
   * This function gets by parameter the temperature.
   * The reading is with direct register access.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool getRegTemperature(double & result);

  /**
   * \brief Get angle velocity calibbias on x axis.
   *
   * This function gets by parameter
   * the angle velocity calibbias on x axis.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool anglvel_x_calibbias(int32_t & result);

  /**
   * \brief Update angle velocity calibbias on x axis.
   *
   * This function updates
   * the angle velocity calibbias on x axis.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_anglvel_calibbias_x(int32_t val);

  /**
   * \brief Get angle velocity calibbias on y axis.
   *
   * This function gets by parameter
   * the angle velocity calibbias on y axis.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool anglvel_y_calibbias(int32_t & result);

  /**
   * \brief Update angle velocity calibbias on y axis.
   *
   * This function updates
   * the angle velocity calibbias on y axis.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_anglvel_calibbias_y(int32_t val);

  /**
   * \brief Get angle velocity calibbias on z axis.
   *
   * This function gets by parameter
   * the angle velocity calibbias on z axis.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool anglvel_z_calibbias(int32_t & result);

  /**
   * \brief Update angle velocity calibbias on z axis.
   *
   * This function updates
   * the angle velocity calibbias on z axis.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_anglvel_calibbias_z(int32_t val);

  /**
   * \brief Get acceleration calibbias on x axis.
   *
   * This function gets by parameter
   * the acceleration calibbias on x axis.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool accel_x_calibbias(int32_t & result);

  /**
   * \brief Update acceleration calibbias on x axis.
   *
   * This function updates
   * the acceleration calibbias on x axis.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_accel_calibbias_x(int32_t val);

  /**
   * \brief Get acceleration calibbias on y axis.
   *
   * This function gets by parameter
   * the acceleration calibbias on y axis.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool accel_y_calibbias(int32_t & result);

  /**
   * \brief Update acceleration calibbias on y axis.
   *
   * This function updates
   * the acceleration calibbias on y axis.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_accel_calibbias_y(int32_t val);

  /**
   * \brief Get acceleration calibbias on z axis.
   *
   * This function gets by parameter
   * the acceleration calibbias on z axis.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool accel_z_calibbias(int32_t & result);

  /**
   * \brief Update acceleration calibbias on z axis.
   *
   * This function updates
   * the acceleration calibbias on z axis.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_accel_calibbias_z(int32_t val);

  /**
   * \brief Get sampling frequency.
   *
   * This function gets the sampling frequency.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool sampling_frequency(double * result);

  /**
   * \brief Update sampling frequency.
   *
   * This function updates the sampling frequency.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_sampling_frequency(double val);

  /**
   * \brief Get diag sensor initialization failure data.
   *
   * This function gets the diag sensor initialization failure data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_sensor_initialization_failure(bool & result);

  /**
   * \brief Get diag data path overrun data.
   *
   * This function gets the diag data path overrun data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_data_path_overrun(bool & result);

  /**
   * \brief Get diag flash memory update_error data.
   *
   * This function gets the diag flash memory update error data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_flash_memory_update_error(bool & result);

  /**
   * \brief Get diag spi communication error data.
   *
   * This function gets the diag spi communication error data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_spi_communication_error(bool & result);

  /**
   * \brief Get diag standby mode data.
   *
   * This function gets the diag standby mode data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_standby_mode(bool & result);

  /**
   * \brief Get diag sensor self test error data.
   *
   * This function gets the diag sensor self test error data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_sensor_self_test_error(bool & result);

  /**
   * \brief Get diag flash memory test error data.
   *
   * This function gets the diag flash memory test error data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_flash_memory_test_error(bool & result);

  /**
   * \brief Get diag clock error data.
   *
   * This function gets the diag clock error data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_clock_error(bool & result);

  /**
   * \brief Get diag gyroscope1 self test error data.
   *
   * This function gets the diag gyroscope1 self test error data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_gyroscope1_self_test_error(bool & result);

  /**
   * \brief Get diag gyroscope2 self test error data.
   *
   * This function gets the diag gyroscope2 self test error data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_gyroscope2_self_test_error(bool & result);

  /**
   * \brief Get diag acceleration self test error data.
   *
   * This function gets the diag acceleration self test error data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_acceleration_self_test_error(bool & result);

  /**
   * \brief Get diag x axis gyroscope failure data.
   *
   * This function gets the diag x axis gyroscope failure data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_x_axis_gyroscope_failure(bool & result);

  /**
   * \brief Get diag y axis gyroscope failure data.
   *
   * This function gets the diag y axis gyroscope failure data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_y_axis_gyroscope_failure(bool & result);

  /**
   * \brief Get diag z axis gyroscope failure data.
   *
   * This function gets the diag z axis gyroscope failure data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_z_axis_gyroscope_failure(bool & result);

  /**
   * \brief Get diag x axis accelerometer failure data.
   *
   * This function gets the diag x axis accelerometer failure data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_x_axis_accelerometer_failure(bool & result);

  /**
   * \brief Get diag y axis accelerometer failure data.
   *
   * This function gets the diag y axis accelerometer failure data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_y_axis_accelerometer_failure(bool & result);

  /**
   * \brief Get diag z axis accelerometer failure data.
   *
   * This function gets the diag z axis accelerometer failure data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_z_axis_accelerometer_failure(bool & result);

  /**
   * \brief Get diag aduc mcu fault data.
   *
   * This function gets the diag aduc mcu fault data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_aduc_mcu_fault(bool & result);

  /**
   * \brief Get diag checksum error flag data.
   *
   * This function gets the diag checksum error flag data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_checksum_error_flag(bool & result);

  /**
   * \brief Get diag flash memory write count exceeded error data.
   *
   * This function gets the diag flash memory write count exceeded error data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool diag_flash_memory_write_count_exceeded_error(bool & result);

  /**
   * \brief Get lost samples count data.
   *
   * This function gets the lost samples count data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool lost_samples_count(uint32_t & result);

  /**
   * \brief Get fifo enable data.
   *
   * This function gets the fifo enable data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool fifo_enable(uint32_t & result);

  /**
   * \brief Update fifo enable.
   *
   * This function updates the fifo enable.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_fifo_enable(uint32_t val);

  /**
   * \brief Get fifo overflow behavior data.
   *
   * This function gets the fifo overflow behavior data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool fifo_overflow_behavior(uint32_t & result);

  /**
   * \brief Update fifo overflow behavior.
   *
   * This function updates the fifo overflow behavior.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_fifo_overflow_behavior(uint32_t val);

  /**
   * \brief Get fifo watermark interrupt enable data.
   *
   * This function gets the fifo watermark interrupt enable data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool fifo_watermark_interrupt_enable(uint32_t & result);

  /**
   * \brief Update fifo watermark interrupt enable.
   *
   * This function updates the fifo watermark interrupt enable.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_fifo_watermark_interrupt_enable(uint32_t val);

  /**
   * \brief Get fifo watermark interrupt polarity data.
   *
   * This function gets the fifo watermark interrupt polarity data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool fifo_watermark_interrupt_polarity(uint32_t & result);

  /**
   * \brief Update fifo watermark interrupt polarity.
   *
   * This function updates the fifo watermark interrupt polarity.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_fifo_watermark_interrupt_polarity(uint32_t val);

  /**
   * \brief Get fifo watermark threshold level data.
   *
   * This function gets the fifo watermark threshold level data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool fifo_watermark_threshold_level(uint32_t & result);

  /**
   * \brief Update fifo watermark threshold level.
   *
   * This function updates the fifo watermark threshold level.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_fifo_watermark_threshold_level(uint32_t val);

  /**
   * \brief Get filter size data.
   *
   * This function gets the filter size data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool filter_size(uint32_t & result);

  /**
   * \brief Update filter size.
   *
   * This function updates the filter size.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_filter_size(uint32_t val);

  /**
   * \brief Get gyroscope measurement range data.
   *
   * This function gets the gyroscope measurement range data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool gyroscope_measurement_range(std::string & result);

  /**
   * \brief Get data ready polarity data.
   *
   * This function gets the data ready polarity data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool data_ready_polarity(uint32_t & result);

  /**
   * \brief Update data ready polarity.
   *
   * This function updates the data ready polarity.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_data_ready_polarity(uint32_t val);

  /**
   * \brief Get sync polarity data.
   *
   * This function gets the sync polarity data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool sync_polarity(uint32_t & result);

  /**
   * \brief Update sync polarity.
   *
   * This function updates the sync polarity.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_sync_polarity(uint32_t val);

  /**
   * \brief Get sync mode select data.
   *
   * This function gets the sync mode select data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool sync_mode_select(uint32_t & result);

  /**
   * \brief Update sync mode select.
   *
   * This function updates the sync mode select.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_sync_mode_select(uint32_t val);

  /**
   * \brief Get internal sensor bandwidth data.
   *
   * This function gets the internal sensor bandwidth data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool internal_sensor_bandwidth(uint32_t & result);

  /**
   * \brief Update internal sensor bandwidth.
   *
   * This function updates the internal sensor bandwidth.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_internal_sensor_bandwidth(uint32_t val);

  /**
   * \brief Get point of percussion alignment data.
   *
   * This function gets the point of percussion alignment data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool point_of_percussion_alignment(uint32_t & result);

  /**
   * \brief Update point of percussion alignment.
   *
   * This function updates the point of percussion alignment.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_point_of_percussion_alignment(uint32_t val);

  /**
   * \brief Update linear acceleration compensation.
   *
   * This function updates the linear acceleration compensation.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_linear_acceleration_compensation(uint32_t val);

  /**
   * \brief Get linear acceleration compensation data.
   *
   * This function gets the linear acceleration compensation data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool linear_acceleration_compensation(uint32_t & result);

  /**
   * \brief Get burst data selection data.
   *
   * This function gets the burst data selection data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool burst_data_selection(uint32_t & result);

  /**
   * \brief Update burst data selection.
   *
   * This function updates the burst data selection.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_burst_data_selection(uint32_t val);

  /**
   * \brief Get burst size selection data.
   *
   * This function gets the burst size selection data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool burst_size_selection(uint32_t & result);

  /**
   * \brief Update burst size selection.
   *
   * This function updates the burst size selection.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_burst_size_selection(uint32_t val);

  /**
   * \brief Get timestamp32 data.
   *
   * This function gets the timestamp32 data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool timestamp32(uint32_t & result);

  /**
   * \brief Update timestamp32.
   *
   * This function updates the timestamp32.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_timestamp32(uint32_t val);

  /**
   * \brief Get internal sync enable 4khz data.
   *
   * This function gets the internal sync enable 4khz data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool internal_sync_enable_4khz(uint32_t & result);

  /**
   * \brief Update internal sync enable 4khz.
   *
   * This function updates the internal sync enable 4khz.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_internal_sync_enable_4khz(uint32_t val);

  /**
   * \brief Get sync signal scale data.
   *
   * This function gets the sync signal scale data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool sync_signal_scale(uint32_t & result);

  /**
   * \brief Update update sync signal scale.
   *
   * This function updates the update sync signal scale.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_sync_signal_scale(uint32_t val);

  /**
   * \brief Get decimation filter data.
   *
   * This function gets the decimation filter data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool decimation_filter(uint32_t & result);

  /**
   * \brief Update decimation filter scale.
   *
   * This function updates the decimation filter.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_decimation_filter(uint32_t val);

  /**
   * \brief Get bias correction time base control data.
   *
   * This function gets the bias correction time base control data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool bias_correction_time_base_control(uint32_t & result);

  /**
   * \brief Update bias correction time base control.
   *
   * This function updates the bias correction time base control.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_bias_correction_time_base_control(uint32_t val);

  /**
   * \brief Get x axis gyroscope bias correction enable data.
   *
   * This function gets the x axis gyroscope bias correction enable data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool x_axis_gyroscope_bias_correction_enable(uint32_t & result);

  /**
   * \brief Update x axis gyroscope bias correction enable.
   *
   * This function updates the x axis gyroscope bias correction enable.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_x_axis_gyroscope_bias_correction_enable(uint32_t val);

  /**
   * \brief Get y axis gyroscope bias correction enable data.
   *
   * This function gets the y axis gyroscope bias correction enable data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool y_axis_gyroscope_bias_correction_enable(uint32_t & result);

  /**
   * \brief Update y axis gyroscope bias correction enable.
   *
   * This function updates the y axis gyroscope bias correction enable.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_y_axis_gyroscope_bias_correction_enable(uint32_t val);

  /**
   * \brief Get z axis gyroscope bias correction enable data.
   *
   * This function gets the z axis gyroscope bias correction enable data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool z_axis_gyroscope_bias_correction_enable(uint32_t & result);

  /**
   * \brief Update z axis gyroscope bias correction enable.
   *
   * This function updates the z axis gyroscope bias correction enable.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_z_axis_gyroscope_bias_correction_enable(uint32_t val);

  /**
   * \brief Get x axis accelerometer bias correction enable data.
   *
   * This function gets the x axis accelerometer bias correction enable data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool x_axis_accelerometer_bias_correction_enable(uint32_t & result);

  /**
   * \brief Update x axis accelerometer bias correction enable.
   *
   * This function updates the x axis accelerometer bias correction enable.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_x_axis_accelerometer_bias_correction_enable(uint32_t val);

  /**
   * \brief Get y axis accelerometer bias correction enable data.
   *
   * This function gets the y axis accelerometer bias correction enable data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool y_axis_accelerometer_bias_correction_enable(uint32_t & result);

  /**
   * \brief Update y axis accelerometer bias correction enable.
   *
   * This function updates the y axis accelerometer bias correction enable.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_y_axis_accelerometer_bias_correction_enable(uint32_t val);

  /**
   * \brief Get z axis accelerometer bias correction enable data.
   *
   * This function gets the z axis accelerometer bias correction enable data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool z_axis_accelerometer_bias_correction_enable(uint32_t & result);

  /**
   * \brief Update z axis accelerometer bias correction enable.
   *
   * This function updates the z axis accelerometer bias correction enable.
   *
   * \return Return true if writing was with success and
   * false if not.
   *
   * @param val The variable with which it will be modified
   */
  bool update_z_axis_accelerometer_bias_correction_enable(uint32_t val);

  /**
   * \brief Update bias correction.
   *
   * This function updates the bias correction.
   *
   * \return Return true if updating was with success and
   * false if not.
   *
   */
  bool bias_correction_update();

  /**
   * \brief Restore factory calibration.
   *
   * This function restores factory calibration.
   *
   * \return Return true if the restore was with success and
   * false if not.
   *
   */
  bool factory_calibration_restore();

  /**
   * \brief Test self sensor.
   *
   * This function tests self sensor.
   *
   * \return Return true if the test was with success and
   * false if not.
   *
   */
  bool sensor_self_test();

  /**
   * \brief Update flash memory.
   *
   * This function updates flash memory.
   *
   * \return Return true if the update was with success and
   * false if not.
   *
   */
  bool flash_memory_update();

  /**
   * \brief Test flash memory.
   *
   * This function tests flash memory.
   *
   * \return Return true if the test was with success and
   * false if not.
   *
   */
  bool flash_memory_test();

  /**
   * \brief Call fifo flush.
   *
   * This function calls fifo flush.
   *
   * \return Return true if the call was with success and
   * false if not.
   *
   */
  bool fifo_flush();

  /**
   * \brief Reset the software.
   *
   * This function resets the software.
   *
   * \return Return true if the reset was with success and
   * false if not.
   *
   */
  bool software_reset();

  /**
   * \brief Get firmware revision data.
   *
   * This function gets the firmware revision data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool firmware_revision(std::string & result);

  /**
   * \brief Get firmware date data.
   *
   * This function gets the firmware date data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool firmware_date(std::string & result);

  /**
   * \brief Get product id data.
   *
   * This function gets the product id data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool product_id(uint32_t & result);

  /**
   * \brief Get serial number data.
   *
   * This function gets the serial number data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param result This variable will contain the read data
   */
  bool serial_number(uint32_t & result);

  /**
   * \brief Get flash counter data.
   *
   * This function gets the flash counter data.
   *
   * \return Return true if reading was with success and
   * false if not.
   *
   * @param value This variable will contain the read data
   */
  bool flash_counter(uint32_t & value);

  /**
   * \brief Get scale value from acceleration.
   *
   * This function gets the scale value from acceleration.
   *
   * \return Return scale value from acceleration.
   */
  double get_scale_accel();

  /**
   * \brief Get scale value from angular velocity.
   *
   * This function gets the scale value from angular velocity.
   *
   * \return Return scale value from angular velocity.
   */
  double get_scale_angvel();

  /**
   * \brief Get scale value from delta velocity.
   *
   * This function gets the scale value from delta velocity.
   *
   * \return Return scale value from delta velocity.
   */
  double get_scale_velocity();

  /**
   * \brief Get scale value from delta anlge.
   *
   * This function gets the scale value from delta anlge.
   *
   * \return Return scale value from delta anlge.
   */
  double get_scale_rot();

  /**
   * \brief Get scale value from temperature.
   *
   * This function gets the scale value from temperature.
   *
   * \return Return scale value from temperature.
   */
  double get_scale_temp();

private:
  /*! This variable retains local context instance*/
  static struct iio_context * m_local_context;

  /*! This variable retains the device instance */
  static struct iio_device * m_dev;

  /*! This variable retains device trigger instance */
  static struct iio_device * m_devtrigger;

  /*! This variable retains device buffer instance */
  static struct iio_buffer * m_device_buffer;

  /*! This variable retains accel x channel */
  static struct iio_channel * m_channel_accel_x;

  /*! This variable retains accel y channel */
  static struct iio_channel * m_channel_accel_y;

  /*! This variable retains accel z channel */
  static struct iio_channel * m_channel_accel_z;

  /*! This variable retains angvel x channel */
  static struct iio_channel * m_channel_anglvel_x;

  /*! This variable retains angvel y channel */
  static struct iio_channel * m_channel_anglvel_y;

  /*! This variable retains angvel z channel */
  static struct iio_channel * m_channel_anglvel_z;

  /*! This variable retains rot, delta angle x channel */
  static struct iio_channel * m_channel_rot_x;

  /*! This variable retains rot, delta angle y channel */
  static struct iio_channel * m_channel_rot_y;

  /*! This variable retains rot, delta angle z channel */
  static struct iio_channel * m_channel_rot_z;

  /*! This variable retains delta velocity x channel */
  static struct iio_channel * m_channel_velocity_x;

  /*! This variable retains delta velocity y channel */
  static struct iio_channel * m_channel_velocity_y;

  /*! This variable retains delta velocity x channel */
  static struct iio_channel * m_channel_velocity_z;

  /*! This variable retains temperature channel */
  static struct iio_channel * m_channel_temp;

  /*! This variable retains timestamp channel */
  static struct iio_channel * m_channel_timestamp;

  /*! This variable retains scale accel x value */
  static double m_scale_accel_x;

  /*! This variable retains scale accel y value */
  static double m_scale_accel_y;

  /*! This variable retains scale accel z value */
  static double m_scale_accel_z;

  /*! This variable retains scale angvel x value */
  static double m_scale_angvel_x;

  /*! This variable retains scale angvel y value */
  static double m_scale_angvel_y;

  /*! This variable retains scale angvel z value */
  static double m_scale_angvel_z;

  /*! This variable retains scale rot, delta angle x value */
  static double m_scale_rot_x;

  /*! This variable retains scale rot, delta angle y value */
  static double m_scale_rot_y;

  /*! This variable retains scale rot, delta angle z value */
  static double m_scale_rot_z;

  /*! This variable retains scale delta velocity x value */
  static double m_scale_velocity_x;

  /*! This variable retains scale delta velocity y value */
  static double m_scale_velocity_y;

  /*! This variable retains scale delta velocity z value */
  static double m_scale_velocity_z;

  /*! This variable retains scale temperature value */
  static double m_scale_temp;

public:
  /*! This variable retains device name */
  static std::string s_device_name;

  /*! This variable retains device trigger name */
  static std::string s_device_trigger_name;

  /*! This variable retains device name enum*/
  static IIODeviceName s_device_name_enum;
};

#endif  // IIO_WRAPPER_H
