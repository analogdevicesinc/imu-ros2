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

#define IIO_CONTEXT_ERROR -1

#include <iio.h>

#include <string>

#include "adis_data_access.h"

/**
 * @brief Wrapper class for libiio library for IMU devices
 */
class IIOWrapper
{
public:
  /**
   * @brief Constructor for IIOWrapper.
   */
  IIOWrapper();

  /**
   * @brief Destructor for IIOWrapper.
   */
  ~IIOWrapper();

  /**
   * @brief create IIO context based on the given context string and search for
   * IIO ADIS device, enable trigger and map device attributes, device channels
   * and channels attributes.
   * @param context IIO context string
   * @return ERROR code on IIO context error, 0 otherwise
   */
  int createContext(const char * context);

  /**
   * @brief Update buffer data. This function should be called before
   * retrieving data using getBuff** APIs.
   * @param data_selection 0 for acceleration and gyroscope data, 1 for
   * delta angle and delta velocity data.
   * @return Return true if data was read successfully and can be retrieved,
   * false otherwise.
   */
  bool updateBuffer(uint32_t data_selection);

  /**
   * @brief Stops buffer acquisition.
   */
  void stopBufferAcquisition();

  /**
   * @brief Get linear acceleration on x axis with buffer reads; in this case
   * the retrieved samples are continuous if the function is called fast enough
   * and samples are not overwritten.
   * @return Return the linear acceleration on x axis in m / s^2.
   */
  double getBuffLinearAccelerationX();

  /**
   * @brief Get linear acceleration on y axis with buffer reads; in this case
   * the retrieved samples are continuous if the function is called fast enough
   * and samples are not overwritten.
   * @return Return the linear acceleration on y axis in m / s^2.
   */
  double getBuffLinearAccelerationY();

  /**
   * @brief Get linear acceleration on z axis with buffer reads; in this case
   * the retrieved samples are continuous if the function is called fast enough
   * and samples are not overwritten.
   * @return Return the linear acceleration on z axis in m / s^2.
   */
  double getBuffLinearAccelerationZ();

  /**
   * @brief Get angular velocity on x axis with buffer reads; in this case
   * the retrieved samples are continuous if the function is called fast enough
   * and samples are not overwritten.
   * @return Return the angular velocity on x axis in rad / s.
   */
  double getBuffAngularVelocityX();

  /**
   * @brief Get angular velocity on y axis with buffer reads; in this case
   * the retrieved samples are continuous if the function is called fast enough
   * and samples are not overwritten.
   * @return Return the angular velocity on y axis in rad / s.
   */
  double getBuffAngularVelocityY();

  /**
   * @brief Get angular velocity on z axis with buffer reads; in this case
   * the retrieved samples are continuous if the function is called fast enough
   * and samples are not overwritten.
   * @return Return the angular velocity on z axis in rad / s.
   */
  double getBuffAngularVelocityZ();

#ifdef ADIS_HAS_DELTA_BURST
  /**
   * @brief Get delta velocity on x axis with buffer reads; in this case
   * the retrieved samples are continuous if the function is called fast enough
   * and samples are not overwritten.
   * @return Return the delta velocity on x axis in m / s.
   */
  double getBuffDeltaVelocityX();

  /**
   * @brief Get delta velocity on y axis with buffer reads; in this case
   * the retrieved samples are continuous if the function is called fast enough
   * and samples are not overwritten.
   * @return Return the delta velocity on y axis in m / s.
   */
  double getBuffDeltaVelocityY();

  /**
   * @brief Get delta velocity on z axis with buffer reads; in this case
   * the retrieved samples are continuous if the function is called fast enough
   * and samples are not overwritten.
   * @return Return the delta velocity on z axis in m / s.
   */
  double getBuffDeltaVelocityZ();

  /**
   * @brief Get delta angle on x axis with buffer reads; in this case
   * the retrieved samples are continuous if the function is called fast enough
   * and samples are not overwritten.
   * @return Return the delta angle on x axis in radians.
   */
  double getBuffDeltaAngleX();

  /**
   * @brief Get delta angle on y axis with buffer reads; in this case
   * the retrieved samples are continuous if the function is called fast enough
   * and samples are not overwritten.
   * @return Return the delta angle on y axis in radians.
   */
  double getBuffDeltaAngleY();

  /**
   * @brief Get delta angle on z axis with buffer reads; in this case
   * the retrieved samples are continuous if the function is called fast enough
   * and samples are not overwritten.
   * @return Return the delta angle on z axis in radians.
   */
  double getBuffDeltaAngleZ();
#endif

  /**
   * @brief Get temperature with buffer reads; in this case
   * the retrieved samples are continuous if the function is called fast enough
   * and samples are not overwritten.
   * @return Return the temperature in degrees Celsius.
   */
  double getBuffTemperature();

  /**
   * @brief Get buffer timestamp when performing buffer reads; the timestamp
   * represent the time at which the samples from the devices were read over
   * SPI.
   * @param sec The second component of the timestamp
   * @param nanosec The nanosecond component of the timestamp
   */
  void getBuffSampleTimestamp(int32_t & sec, uint32_t & nanosec);

  /**
   * @brief Get linear acceleration on x axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The linear acceleration on x axis in m / s^2.
   */
  bool getConvertedLinearAccelerationX(double & result);

  /**
   * @brief Get linear acceleration on y axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The linear acceleration on y axis in m / s^2.
   */
  bool getConvertedLinearAccelerationY(double & result);

  /**
   * @brief Get linear acceleration on z axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The linear acceleration on z axis in m / s^2.
   */
  bool getConvertedLinearAccelerationZ(double & result);

  /**
   * @brief Get angular velocity on x axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The angular velocity on x axis in rad / s.
   */
  bool getConvertedAngularVelocityX(double & result);

  /**
   * @brief Get angular velocity on y axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The angular velocity on y axis in rad / s.
   */
  bool getConvertedAngularVelocityY(double & result);

  /**
   * @brief Get angular velocity on z axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The angular velocity on z axis in rad / s.
   */
  bool getConvertedAngularVelocityZ(double & result);

  /**
   * @brief Get delta angle on x axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta angle on x axis in radians.
   */
  bool getConvertedDeltaAngleX(double & result);

  /**
   * @brief Get delta angle on y axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta angle on y axis in radians.
   */
  bool getConvertedDeltaAngleY(double & result);

  /**
   * @brief Get delta angle on z axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta angle on z axis in radians.
   */
  bool getConvertedDeltaAngleZ(double & result);

  /**
   * @brief Get delta velocity on x axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta velocity on x axis in m / s.
   */
  bool getConvertedDeltaVelocityX(double & result);

  /**
   * @brief Get delta velocity on y axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta velocity on y axis in m / s.
   */
  bool getConvertedDeltaVelocityY(double & result);

  /**
   * @brief Get delta velocity on z axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta velocity on z axis in m / s.
   */
  bool getConvertedDeltaVelocityZ(double & result);

  /**
   * @brief Get temperature with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The temperature in degrees Celsius.
   */
  bool getConvertedTemperature(double & result);

  /**
   * @brief Get angular velocity calibration offset on x axis.
   * @param result angular velocity calibration offset on x axis.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool anglvel_x_calibbias(int32_t & result);

  /**
   * @brief Update angular velocity calibration offset on x axis.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_anglvel_calibbias_x(int32_t val);

  /**
   * @brief Get angular velocity calibration offset on y axis.
   * @param result angular velocity calibration offset on y axis.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool anglvel_y_calibbias(int32_t & result);

  /**
   * @brief Update angular velocity calibration offset on y axis.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_anglvel_calibbias_y(int32_t val);

  /**
   * @brief Get angular velocity calibration offset on z axis.
   * @param result angular velocity calibration offset on z axis.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool anglvel_z_calibbias(int32_t & result);

  /**
   * @brief Update angular velocity calibration offset on z axis.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_anglvel_calibbias_z(int32_t val);

  /**
   * @brief Get linear acceleration calibration offset on x axis.
   * @param result linear acceleration calibration offset on x axis.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool accel_x_calibbias(int32_t & result);

  /**
   * @brief Update linear acceleration calibration offset on x axis.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_accel_calibbias_x(int32_t val);

  /**
   * @brief Get linear acceleration calibration offset on y axis.
   * @param result linear acceleration calibration offset on y axis.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool accel_y_calibbias(int32_t & result);

  /**
   * @brief Update linear acceleration calibration offset on y axis.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_accel_calibbias_y(int32_t val);

  /**
   * @brief Get linear acceleration calibration offset on z axis.
   * @param result linear acceleration calibration offset on z axis.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool accel_z_calibbias(int32_t & result);

  /**
   * @brief Update linear acceleration calibration offset on z axis.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_accel_calibbias_z(int32_t val);

#if defined(ADIS1654X) || defined(ADIS1655X)
  /**
   * @brief Get low pass 3db frequency data for x angular velocity.
   * @param result low pass 3db frequency value.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool angvel_x_filter_low_pass_3db(uint32_t & result);
  /**
   * @brief Update low pass 3db frequency for x angular velocity.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_angvel_x_filter_low_pass_3db(uint32_t val);

  /**
   * @brief Get low pass 3db frequency data for y angular velocity.
   * @param result low pass 3db frequency value.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool angvel_y_filter_low_pass_3db(uint32_t & result);
  /**
   * @brief Update low pass 3db frequency for y angular velocity.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_angvel_y_filter_low_pass_3db(uint32_t val);

  /**
   * @brief Get low pass 3db frequency data for z angular velocity.
   * @param result low pass 3db frequency value.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool angvel_z_filter_low_pass_3db(uint32_t & result);
  /**
   * @brief Update low pass 3db frequency for z angular velocity.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_angvel_z_filter_low_pass_3db(uint32_t val);

  /**
   * @brief Get low pass 3db frequency data for x acceleration.
   * @param result low pass 3db frequency value.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool accel_x_filter_low_pass_3db(uint32_t & result);
  /**
   * @brief Update low pass 3db frequency for x acceleration.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_accel_x_filter_low_pass_3db(uint32_t val);

  /**
   * @brief Get low pass 3db frequency data for y acceleration.
   * @param result low pass 3db frequency value.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool accel_y_filter_low_pass_3db(uint32_t & result);
  /**
   * @brief Update low pass 3db frequency for y acceleration.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_accel_y_filter_low_pass_3db(uint32_t val);

  /**
   * @brief Get low pass 3db frequency data for z acceleration.
   * @param result low pass 3db frequency value.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool accel_z_filter_low_pass_3db(uint32_t & result);
  /**
   * @brief Update low pass 3db frequency for z acceleration.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_accel_z_filter_low_pass_3db(uint32_t val);

#else
  /**
   * @brief Get low pass 3db frequency data.
   * @param result low pass 3db frequency value.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool filter_low_pass_3db_frequency(uint32_t & result);

  /**
   * @brief Update low pass 3db frequency.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_filter_low_pass_3db_frequency(uint32_t val);
#endif

#ifdef ADIS_HAS_CALIB_SCALE
  /**
   * @brief Get linear acceleration calibration scale on x axis.
   * @param result linear acceleration calibration scale on x axis.
   * @return Return true if reading was with success and
   * false if not.
   */
  bool accel_x_calibscale(int32_t & result);
  /**
   * @brief Get linear acceleration calibration scale on y axis.
   * @param result linear acceleration calibration scale on y axis.
   * @return Return true if reading was with success and
   * false if not.
   */
  bool accel_y_calibscale(int32_t & result);
  /**
   * @brief Get linear acceleration calibration scale on z axis.
   * @param result linear acceleration calibration scale on z axis.
   * @return Return true if reading was with success and
   * false if not.
   */
  bool accel_z_calibscale(int32_t & result);
  /**
   * @brief Get linear angular velocity calibration scale on x axis.
   * @param result linear angular velocity calibration scale on x axis.
   * @return Return true if reading was with success and
   * false if not.
   */
  bool anglvel_x_calibscale(int32_t & result);
  /**
   * @brief Get linear angular velocity calibration scale on y axis.
   * @param result linear angular velocity calibration scale on y axis.
   * @return Return true if reading was with success and
   * false if not.
   */
  bool anglvel_y_calibscale(int32_t & result);
  /**
   * @brief Get linear angular velocity calibration scale on z axis.
   * @param result linear angular velocity calibration scale on z axis.
   * @return Return true if reading was with success and
   * false if not.
   */
  bool anglvel_z_calibscale(int32_t & result);

  /**
   * @brief Update linear acceleration calibration scale on x axis.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_accel_calibscale_x(int32_t val);
  /**
   * @brief Update linear acceleration calibration scale on y axis.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_accel_calibscale_y(int32_t val);
  /**
   * @brief Update linear acceleration calibration scale on y axis.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_accel_calibscale_z(int32_t val);
  /**
   * @brief Update angular velocity calibration scale on x axis.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_anglvel_calibscale_x(int32_t val);
  /**
   * @brief Update angular velocity calibration scale on y axis.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_anglvel_calibscale_y(int32_t val);
  /**
   * @brief Update angular velocity calibration scale on z axis.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_anglvel_calibscale_z(int32_t val);

#endif

  /**
   * @brief Get sampling frequency.
   * @param result current set sampling frequency
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool sampling_frequency(double * result);

  /**
   * @brief Update sampling frequency.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_sampling_frequency(double val);

  /**
   * @brief Get diag sensor initialization failure data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_sensor_initialization_failure(bool & result);

  /**
   * @brief Get diag data path overrun data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_data_path_overrun(bool & result);

  /**
   * @brief Get automatic reset data
   * @param result True if device reset has occured
   * @return true Return true if reading was successful and data is valid,
   * false otherwise.
   */
  bool diag_automatic_reset(bool & result);

  /**
   * @brief Get diag flash memory update_error data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_flash_memory_update_error(bool & result);

  /**
   * @brief Get diag spi communication error data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_spi_communication_error(bool & result);

  /**
   * @brief Get diag crc error.
   * @param result True if calculation failure occurred in a CRC
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_crc_error(bool & result);

  /**
   * @brief Get diag standby mode data.
   * @param result True if device is in standby mode, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_standby_mode(bool & result);

  /**
   * @brief Get diag sensor self test error data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_sensor_self_test_error(bool & result);

  /**
   * @brief Get diag flash memory test error data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_flash_memory_test_error(bool & result);

  /**
   * @brief Get diag clock error data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_clock_error(bool & result);

  /**
   * @brief Get diag gyroscope1 self test error data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_gyroscope1_self_test_error(bool & result);

  /**
   * @brief Get diag gyroscope2 self test error data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_gyroscope2_self_test_error(bool & result);

  /**
   * @brief Get diag acceleration self test error data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_acceleration_self_test_error(bool & result);

  /**
   * @brief Get diag x axis gyroscope failure data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_x_axis_gyroscope_failure(bool & result);

  /**
   * @brief Get diag y axis gyroscope failure data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_y_axis_gyroscope_failure(bool & result);

  /**
   * @brief Get diag z axis gyroscope failure data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_z_axis_gyroscope_failure(bool & result);

  /**
   * @brief Get diag x axis accelerometer failure data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_x_axis_accelerometer_failure(bool & result);

  /**
   * @brief Get diag y axis accelerometer failure data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_y_axis_accelerometer_failure(bool & result);

  /**
   * @brief Get diag z axis accelerometer failure data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_z_axis_accelerometer_failure(bool & result);

  /**
   * @brief Get diag aduc mcu fault data.
   * @param result True if failure occurred, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_aduc_mcu_fault(bool & result);

  /**
   * @brief Get diag flash memory write count exceeded error data.
   * @param result True if write count exceeded allowed value, false otherwise.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool diag_flash_memory_write_count_exceeded_error(bool & result);

  /**
   * @brief Get gyroscope measurement range data.
   * @param result gyroscope measurement range data.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool gyroscope_measurement_range(std::string & result);

  /**
   * @brief Get internal sensor bandwidth data.
   * @param result internal sensor bandwidth data.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool internal_sensor_bandwidth(uint32_t & result);

  /**
   * @brief Update internal sensor bandwidth.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_internal_sensor_bandwidth(uint32_t val);

  /**
   * @brief Get point of percussion alignment data.
   * @param result point of percussion alignment data.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool point_of_percussion_alignment(uint32_t & result);

  /**
   * @brief Update point of percussion alignment.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_point_of_percussion_alignment(uint32_t val);

  /**
   * @brief Get linear acceleration compensation data.
   * @param result linear acceleration compensation data.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool linear_acceleration_compensation(uint32_t & result);

  /**
   * @brief Update linear acceleration compensation.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_linear_acceleration_compensation(uint32_t val);

  /**
   * @brief Get bias correction time base control data.
   * @param result bias correction time base control data.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool bias_correction_time_base_control(uint32_t & result);

  /**
   * @brief Update bias correction time base control.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_bias_correction_time_base_control(uint32_t val);

  /**
   * @brief Get x axis gyroscope bias correction enable data.
   * @param result x axis gyroscope bias correction enable data.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool x_axis_gyroscope_bias_correction_enable(uint32_t & result);

  /**
   * @brief Update x axis gyroscope bias correction enable.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_x_axis_gyroscope_bias_correction_enable(uint32_t val);

  /**
   * @brief Get y axis gyroscope bias correction enable data.
   * @param result y axis gyroscope bias correction enable data.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool y_axis_gyroscope_bias_correction_enable(uint32_t & result);

  /**
   * @brief Update y axis gyroscope bias correction enable.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_y_axis_gyroscope_bias_correction_enable(uint32_t val);

  /**
   * @brief Get z axis gyroscope bias correction enable data.
   * @param result z axis gyroscope bias correction enable data.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool z_axis_gyroscope_bias_correction_enable(uint32_t & result);

  /**
   * @brief Update z axis gyroscope bias correction enable.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_z_axis_gyroscope_bias_correction_enable(uint32_t val);

  /**
   * @brief Get x axis accelerometer bias correction enable data.
   * @param result x axis accelerometer bias correction enable data.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool x_axis_accelerometer_bias_correction_enable(uint32_t & result);

  /**
   * @brief Update x axis accelerometer bias correction enable.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_x_axis_accelerometer_bias_correction_enable(uint32_t val);

  /**
   * @brief Get y axis accelerometer bias correction enable data.
   * @param result y axis accelerometer bias correction enable data.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool y_axis_accelerometer_bias_correction_enable(uint32_t & result);

  /**
   * @brief Update y axis accelerometer bias correction enable.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_y_axis_accelerometer_bias_correction_enable(uint32_t val);

  /**
   * @brief Get z axis accelerometer bias correction enable data.
   * @param result z axis accelerometer bias correction enable data.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool z_axis_accelerometer_bias_correction_enable(uint32_t & result);

  /**
   * @brief Update z axis accelerometer bias correction enable.
   * @param val value to update with.
   * @return Return true if writing was with success and
   * false if not.
   */
  bool update_z_axis_accelerometer_bias_correction_enable(uint32_t val);

  /**
   * @brief Trigger a bias correction update command.
   * @return Return true if the command was successfully triggered, false
   * otherwise.
   */
  bool bias_correction_update();

  /**
   * @brief Trigger a factory calibration restore command.
   * @return Return true if the command was successfully triggered, false
   * otherwise.
   */
  bool factory_calibration_restore();

  /**
   * @brief Trigger a sensor self test command.
   * @return Return true if the command was successfully triggered, false
   * otherwise.
   */
  bool sensor_self_test();

  /**
   * @brief Trigger a flash memory update command.
   * @return Return true if the command was successfully triggered, false
   * otherwise.
   */
  bool flash_memory_update();

  /**
   * @brief Trigger a flash memory test command.
   * @return Return true if the command was successfully triggered, false
   * otherwise.
   */
  bool flash_memory_test();

  /**
   * @brief Trigger a software reset command.
   * @return Return true if the command was successfully triggered, false
   * otherwise.
   */
  bool software_reset();

  /**
   * @brief Get firmware revision value.
   * @param result The firmware revision value.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool firmware_revision(std::string & result);

  /**
   * @brief Get firmware date value.
   * @param result The firmware data value.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool firmware_date(std::string & result);

  /**
   * @brief Get product id value.
   * @param result The product id value.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool product_id(uint32_t & result);

  /**
   * @brief Get serial number value.
   * @param result The serial number value.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool serial_number(uint32_t & result);

  /**
   * @brief Get flash counter value.
   * @param value The flash counter value.
   * @return Return true if reading was successful and data is valid, false
   * otherwise.
   */
  bool flash_counter(uint32_t & value);

  /**
   * @brief Get scale value for linear acceleration.
   * @return Return scale value for linear acceleration.
   */
  double get_scale_accel();

  /**
   * @brief Get scale value for angular velocity.
   * @return Return scale value for angular velocity.
   */
  double get_scale_anglvel();

  /**
   * @brief Get scale value for delta velocity.
   * @return Return scale value for delta velocity.
   */
  double get_scale_deltavelocity();

  /**
   * @brief Get scale value for delta angle.
   * @return Return scale value for delta angle.
   */
  double get_scale_deltaangl();

  /**
   * @brief Get scale value for temperature.
   * @return Return scale value for temperature.
   */
  double get_scale_temp();

private:
  /**
   * @brief Sets manually the delta angle scales based on the device id.
   * @param dev Device id for which the scales are set.
   */
  void setDeltaAngleScales(enum adis_device_id dev_id);
  /**
   * @brief Sets manually the delta velocity scales based on the device id.
   * @param dev Device id for which the scales are set.
   */
  void setDeltaVelocityScales(enum adis_device_id dev_id);
  /**
   * @brief Update a field in the register map.
   * @param reg The register address where the field is located
   * @param val The value to be written.
   * @param mask The bitmask to change
   * @return Return true if updated was successful, false otherwise.
   */
  bool updateField(uint32_t reg, uint32_t val, uint32_t mask);

  /**
   * @brief Get raw delta angle on x axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta angle on x axis in raw format.
   */
  bool getRawDeltaAngleXFromDebug(int32_t & result);

  /**
   * @brief Get raw delta angle on y axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta angle on y axis in raw format.
   */
  bool getRawDeltaAngleYFromDebug(int32_t & result);

  /**
   * @brief Get raw delta angle on z axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta angle on z axis in raw format.
   */
  bool getRawDeltaAngleZFromDebug(int32_t & result);

  /**
   * @brief Get raw delta velocity on x axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta velocity on x axis in raw format.
   */
  bool getRawDeltaVelocityXFromDebug(int32_t & result);

  /**
   * @brief Get raw delta velocity on y axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta velocity on y axis in raw format.
   */
  bool getRawDeltaVelocityYFromDebug(int32_t & result);

  /**
   * @brief Get raw delta velocity on z axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta velocity on z axis in raw format.
   */
  bool getRawDeltaVelocityZFromDebug(int32_t & result);

  /**
   * @brief Get delta angle on x axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta angle on x axis in radians.
   */
  bool getConvertedDeltaAngleXFromDebug(double & result);

  /**
   * @brief Get delta angle on y axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta angle on y axis in radians.
   */
  bool getConvertedDeltaAngleYFromDebug(double & result);

  /**
   * @brief Get delta angle on z axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta angle on z axis in radians.
   */
  bool getConvertedDeltaAngleZFromDebug(double & result);

  /**
   * @brief Get delta velocity on x axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta velocity on x axis in m / s.
   */
  bool getConvertedDeltaVelocityXFromDebug(double & result);

  /**
   * @brief Get delta velocity on y axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta velocity on y axis in m / s.
   */
  bool getConvertedDeltaVelocityYFromDebug(double & result);

  /**
   * @brief Get delta velocity on z axis with register reads; in this case
   * the retrieved samples are not necessary continuous.
   * @return Return true if the reading was successful and result is valid,
   * false otherwise.
   * @param result The delta velocity on z axis in m / s.
   */
  bool getConvertedDeltaVelocityZFromDebug(double & result);

  /*! This variable retains the IIO context instance */
  static struct iio_context * m_iio_context;

  /*! This variable retains the device instance */
  static struct iio_device * m_dev;

  /*! This variable retains device trigger instance */
  static struct iio_device * m_dev_trigger;

  /*! This variable retains device buffer instance */
  static struct iio_buffer * m_dev_buffer;

  /*! This variable retains the linear acceleration x channel */
  static struct iio_channel * m_channel_accel_x;

  /*! This variable retains the linear acceleration y channel */
  static struct iio_channel * m_channel_accel_y;

  /*! This variable retains the linear acceleration z channel */
  static struct iio_channel * m_channel_accel_z;

  /*! This variable retains the angular velocity x channel */
  static struct iio_channel * m_channel_anglvel_x;

  /*! This variable retains the angular velocity y channel */
  static struct iio_channel * m_channel_anglvel_y;

  /*! This variable retains the angular velocity z channel */
  static struct iio_channel * m_channel_anglvel_z;

  /*! This variable retains the delta angle x channel */
  static struct iio_channel * m_channel_deltaangl_x;

  /*! This variable retains the delta angle y channel */
  static struct iio_channel * m_channel_deltaangl_y;

  /*! This variable retains the delta angle z channel */
  static struct iio_channel * m_channel_deltaangl_z;

  /*! This variable retains the delta velocity x channel */
  static struct iio_channel * m_channel_deltavelocity_x;

  /*! This variable retains the delta velocity y channel */
  static struct iio_channel * m_channel_deltavelocity_y;

  /*! This variable retains the delta velocity x channel */
  static struct iio_channel * m_channel_deltavelocity_z;

  /*! This variable retains the temperature channel */
  static struct iio_channel * m_channel_temp;

  /*! This variable retains the timestamp channel */
  static struct iio_channel * m_channel_timestamp;

  /*! This variable retains the scale for the linear acceleration x raw value
   */
  static double m_scale_accel_x;

  /*! This variable retains the scale for the linear acceleration y raw value
   */
  static double m_scale_accel_y;

  /*! This variable retains the scale for the linear acceleration z raw value
   */
  static double m_scale_accel_z;

  /*! This variable retains the scale for the angular velocity  x raw value */
  static double m_scale_anglvel_x;

  /*! This variable retains the scale for the angular velocity  y raw value */
  static double m_scale_anglvel_y;

  /*! This variable retains the scale for the angular velocity z raw value */
  static double m_scale_anglvel_z;

  /*! This variable retains the scale for the delta angle x raw value */
  static double m_scale_deltaangl_x;

  /*! This variable retains the scale for the delta angle y raw value */
  static double m_scale_deltaangl_y;

  /*! This variable retains the scale for the delta angle z raw value */
  static double m_scale_deltaangl_z;

  /*! This variable retains the scale for the delta velocity x raw value */
  static double m_scale_deltavelocity_x;

  /*! This variable retains the scale for the delta velocity y raw value */
  static double m_scale_deltavelocity_y;

  /*! This variable retains the scale for the delta velocity z raw value */
  static double m_scale_deltavelocity_z;

  /*! This variable retains the scale for the temperature raw value */
  static double m_scale_temp;
};

#endif  // IIO_WRAPPER_H
