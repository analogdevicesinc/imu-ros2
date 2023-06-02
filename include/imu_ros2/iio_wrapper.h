/***************************************************************************//**
*   @file   iio_wrapper.h
*   @brief  Wrapper for iio library
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

#ifndef IIO_WRAPPER_H
#define IIO_WRAPPER_H

#include<string>
#include <iio.h>
#include <mutex>
#include "setting_declarations.h"

class IIOWrapper {
public:
    IIOWrapper();
    ~IIOWrapper();

    void update_buffer(bool& success);

    float getAccelerometerX();
    float getAccelerometerY();
    float getAccelerometerZ();

    float getGyroscopeX();
    float getGyroscopeY();
    float getGyroscopeZ();

    float getRotX();
    float getRotY();
    float getRotZ();

    float getVelocityX();
    float getVelocityY();
    float getVelocityZ();

    float getTemperature();

    bool getRegAccelerometerX(double& result);
    bool getRegAccelerometerY(double& result);
    bool getRegAccelerometerZ(double& result);

    bool getRegGyroscopeX(double& result);
    bool getRegGyroscopeY(double& result);
    bool getRegGyroscopeZ(double& result);

    bool getRegRotX(double& result);
    bool getRegRotY(double& result);
    bool getRegRotZ(double& result);

    bool getRegVelocityX(double& result);
    bool getRegVelocityY(double& result);
    bool getRegVelocityZ(double& result);

    bool getRegTemperature(double& result);

    int lost_samples_count();

    std::string firmware_revision();
    std::string firmware_date();

    int product_id();
    int serial_number();
    std::string gyroscope_measurement_range();
    int diag_checksum_error_flag();
    int diag_flash_memory_write_count_exceeded_error();
    int diag_acceleration_self_test_error();
    int diag_gyroscope2_self_test_error();
    int diag_gyroscope1_self_test_error();
    int diag_clock_error();
    int diag_flash_memory_test_error();
    int diag_sensor_self_test_error();
    int diag_standby_mode();
    int diag_spi_communication_error();
    int diag_flash_memory_update_error();
    int diag_data_path_overrun();
    int flash_counter();

    int32_t filter_size();

    int32_t anglvel_x_calibbias();
    int32_t anglvel_y_calibbias();
    int32_t anglvel_z_calibbias();
    int32_t accel_x_calibbias();
    int32_t accel_y_calibbias();
    int32_t accel_z_calibbias();
    int32_t burst_size_selection();
    int32_t burst_data_selection();
    int32_t linear_acceleration_compensation();
    int32_t point_of_percussion_alignment();
    int32_t internal_sensor_bandwidth();
    int32_t sync_mode_select();
    int32_t sync_polarity();
    int32_t data_ready_polarity();
    int32_t sync_signal_scale();
    int32_t decimation_filter();

    int32_t count();

    int update_filter_size(int32_t val);
    int update_burst_size_selection(int32_t val);
    int update_burst_data_selection(int32_t val);
    int update_linear_acceleration_compensation(int32_t val);
    int update_point_of_percussion_alignment(int32_t val);
    int update_internal_sensor_bandwidth(int32_t val);
    int update_sync_mode_select(int32_t val);
    int update_sync_polarity(int32_t val);
    int update_data_ready_polarity(int32_t val);
    int update_sync_signal_scale(int32_t val);
    int update_decimation_filter(int32_t val);
    int update_accel_calibbias_x(int32_t val);
    int update_accel_calibbias_y(int32_t val);
    int update_accel_calibbias_z(int32_t val);
    int update_anglvel_calibbias_x(int32_t val);
    int update_anglvel_calibbias_y(int32_t val);
    int update_anglvel_calibbias_z(int32_t val);

    int update_sampling_frequency(double val);
    double sampling_frequency();

    int software_reset();
    int flash_memory_test();
    int flash_memory_update();
    int sensor_self_test();
    int factory_calibration_restore();

    int fifo_flush();
    int bias_correction_update();

    void load();
    void unload();
    bool isBufferLoaded();

    int32_t z_axis_accelerometer_bias_correction_enable();
    int32_t y_axis_accelerometer_bias_correction_enable();
    int32_t x_axis_accelerometer_bias_correction_enable();
    int32_t z_axis_gyroscope_bias_correction_enable();
    int32_t y_axis_gyroscope_bias_correction_enable();
    int32_t x_axis_gyroscope_bias_correction_enable();
    int32_t internal_sync_enable_4khz();
    int32_t timestamp32();
    int32_t fifo_watermark_interrupt_polarity();
    int32_t fifo_watermark_interrupt_enable();
    int32_t fifo_overflow_behavior();
    int32_t fifo_enable();
    int32_t bias_correction_time_base_control();
    int32_t fifo_watermark_threshold_level();

    int update_z_axis_accelerometer_bias_correction_enable(int32_t val);
    int update_y_axis_accelerometer_bias_correction_enable(int32_t val);
    int update_x_axis_accelerometer_bias_correction_enable(int32_t val);
    int update_z_axis_gyroscope_bias_correction_enable(int32_t val);
    int update_y_axis_gyroscope_bias_correction_enable(int32_t val);
    int update_x_axis_gyroscope_bias_correction_enable(int32_t val);
    int update_internal_sync_enable_4khz(int32_t val);
    int update_timestamp32(int32_t val);
    int update_fifo_watermark_interrupt_polarity(int32_t val);
    int update_fifo_watermark_interrupt_enable(int32_t val);
    int update_fifo_overflow_behavior(int32_t val);
    int update_fifo_enable(int32_t val);
    int update_bias_correction_time_base_control(int32_t val);
    int update_fifo_watermark_threshold_level(int32_t val);

private:
    static   std::mutex m_mutex;

    static struct iio_context * m_local_context;
    static struct iio_device *m_dev;
    static struct iio_device *m_devtrigger;

    static struct iio_buffer *m_device_buffer;

    static struct iio_channel *m_channel_accel_x;
    static struct iio_channel *m_channel_accel_y;
    static struct iio_channel *m_channel_accel_z;

    static struct iio_channel *m_channel_anglvel_x;
    static struct iio_channel *m_channel_anglvel_y;
    static struct iio_channel *m_channel_anglvel_z;

    static struct iio_channel *m_channel_rot_x;
    static struct iio_channel *m_channel_rot_y;
    static struct iio_channel *m_channel_rot_z;

    static struct iio_channel *m_channel_velocity_x;
    static struct iio_channel *m_channel_velocity_y;
    static struct iio_channel *m_channel_velocity_z;

    static struct iio_channel *m_channel_temp;
    static struct iio_channel *m_channel_count;

    static float m_fvalScaleAccel_x;
    static float m_fvalScaleAccel_y;
    static float m_fvalScaleAccel_z;

    static float m_fvalScaleAngvel_x;
    static float m_fvalScaleAngvel_y;
    static float m_fvalScaleAngvel_z;

    static float m_fvalScaleRot_x;
    static float m_fvalScaleRot_y;
    static float m_fvalScaleRot_z;

    static float m_fvalScaleVelocity_x;
    static float m_fvalScaleVelocity_y;
    static float m_fvalScaleVelocity_z;

    static float m_fvalScaleTemp;

    // device sensors data member
    static float m_accelerometer_x;
    static float m_accelerometer_y;
    static float m_accelerometer_z;
    static float m_gyroscope_x;
    static float m_gyroscope_y;
    static float m_gyroscope_z;
    static float m_rot_x;
    static float m_rot_y;
    static float m_rot_z;
    static float m_velocity_x;
    static float m_velocity_y;
    static float m_velocity_z;
    static float m_temperature;
    static int32_t m_count;

public:
    static std::string s_device_name;
    static std::string s_device_trigger_name;
    static IIODeviceName s_device_name_enum;
};

#endif // IIO_WRAPPER_H
