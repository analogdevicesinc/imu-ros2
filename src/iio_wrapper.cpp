/***************************************************************************//**
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

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

struct iio_context* IIOWrapper::m_local_context = nullptr;

struct iio_device* IIOWrapper::m_dev = nullptr;
struct iio_device* IIOWrapper::m_devtrigger = nullptr;
struct iio_buffer* IIOWrapper::m_device_buffer = nullptr;

struct iio_channel *IIOWrapper::m_channel_accel_x = nullptr;
struct iio_channel *IIOWrapper::m_channel_accel_y = nullptr;
struct iio_channel *IIOWrapper::m_channel_accel_z = nullptr;

struct iio_channel *IIOWrapper::m_channel_anglvel_x = nullptr;
struct iio_channel *IIOWrapper::m_channel_anglvel_y = nullptr;
struct iio_channel *IIOWrapper::m_channel_anglvel_z = nullptr;

struct iio_channel *IIOWrapper::m_channel_rot_x = nullptr;
struct iio_channel *IIOWrapper::m_channel_rot_y = nullptr;
struct iio_channel *IIOWrapper::m_channel_rot_z = nullptr;

struct iio_channel *IIOWrapper::m_channel_velocity_x = nullptr;
struct iio_channel *IIOWrapper::m_channel_velocity_y = nullptr;
struct iio_channel *IIOWrapper::m_channel_velocity_z = nullptr;

struct iio_channel *IIOWrapper::m_channel_temp = nullptr;
struct iio_channel *IIOWrapper::m_channel_count = nullptr;

float IIOWrapper::m_fvalScaleAccel_x = 0;
float IIOWrapper::m_fvalScaleAccel_y = 0;
float IIOWrapper::m_fvalScaleAccel_z = 0;

float IIOWrapper::m_fvalScaleAngvel_x = 0;
float IIOWrapper::m_fvalScaleAngvel_y = 0;
float IIOWrapper::m_fvalScaleAngvel_z = 0;

float IIOWrapper::m_fvalScaleRot_x = 0;
float IIOWrapper::m_fvalScaleRot_y = 0;
float IIOWrapper::m_fvalScaleRot_z = 0;

float IIOWrapper::m_fvalScaleVelocity_x = 0;
float IIOWrapper::m_fvalScaleVelocity_y = 0;
float IIOWrapper::m_fvalScaleVelocity_z = 0;

float IIOWrapper::m_fvalScaleTemp = 0;

// device sensors data member
float IIOWrapper::m_accelerometer_x = 0;
float IIOWrapper::m_accelerometer_y = 0;
float IIOWrapper::m_accelerometer_z = 0;
float IIOWrapper::m_gyroscope_x = 0;
float IIOWrapper::m_gyroscope_y = 0;
float IIOWrapper::m_gyroscope_z = 0;
float IIOWrapper::m_rot_x = 0;
float IIOWrapper::m_rot_y = 0;
float IIOWrapper::m_rot_z = 0;
float IIOWrapper::m_velocity_x = 0;
float IIOWrapper::m_velocity_y = 0;
float IIOWrapper::m_velocity_z = 0;
float IIOWrapper::m_temperature = 0;
int32_t IIOWrapper::m_count = 0;

std::mutex IIOWrapper::m_mutex;

IIOWrapper::IIOWrapper()
{
    if(m_local_context == nullptr)
    {
        m_local_context = iio_create_local_context();
        if(!m_local_context)
        {
            throw std::runtime_error("Exception: local context is null");
        }
    }

    if(m_dev == nullptr)
    {
        m_dev = iio_context_find_device(m_local_context, "adis16505");
        if(!m_dev)
        {
            throw std::runtime_error("Exception: device data is null");
        }
    }

    if(m_devtrigger == nullptr)
    {
        m_devtrigger = iio_context_find_device(m_local_context, "adis16505-dev0");
        if(!m_devtrigger)
        {
            throw std::runtime_error("Exception: device trigger data is null");
        }
    }

    iio_device_set_trigger(m_dev, m_devtrigger);

    if(m_channel_accel_x == nullptr)
        m_channel_accel_x = iio_device_find_channel(m_dev, "accel_x", false);
    if(m_channel_accel_y == nullptr)
        m_channel_accel_y = iio_device_find_channel(m_dev, "accel_y", false);
    if(m_channel_accel_z == nullptr)
        m_channel_accel_z = iio_device_find_channel(m_dev, "accel_z", false);

    if(m_channel_anglvel_x == nullptr)
        m_channel_anglvel_x = iio_device_find_channel(m_dev, "anglvel_x", false);
    if(m_channel_anglvel_y == nullptr)
        m_channel_anglvel_y = iio_device_find_channel(m_dev, "anglvel_y", false);
    if(m_channel_anglvel_z == nullptr)
        m_channel_anglvel_z = iio_device_find_channel(m_dev, "anglvel_z", false);

    if(m_channel_rot_x == nullptr)
        m_channel_rot_x = iio_device_find_channel(m_dev, "rot_x", false);
    if(m_channel_rot_y == nullptr)
        m_channel_rot_y = iio_device_find_channel(m_dev, "rot_y", false);
    if(m_channel_rot_z == nullptr)
        m_channel_rot_z = iio_device_find_channel(m_dev, "rot_z", false);

    if(m_channel_velocity_x == nullptr)
        m_channel_velocity_x = iio_device_find_channel(m_dev, "velocity_x", false);
    if(m_channel_velocity_y == nullptr)
        m_channel_velocity_y = iio_device_find_channel(m_dev, "velocity_y", false);
    if(m_channel_velocity_z == nullptr)
        m_channel_velocity_z = iio_device_find_channel(m_dev, "velocity_z", false);

    if(m_channel_temp == nullptr)
        m_channel_temp = iio_device_find_channel(m_dev, "temp", false);

    if(m_channel_count == nullptr)
        m_channel_count = iio_device_find_channel(m_dev, "count", false);

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
    iio_channel_enable(m_channel_count);

    double valueScaleAccel_x;
    iio_channel_attr_read_double(m_channel_accel_x, "scale", &valueScaleAccel_x);
    m_fvalScaleAccel_x = valueScaleAccel_x;

    double valueScaleAccel_y;
    iio_channel_attr_read_double(m_channel_accel_y, "scale", &valueScaleAccel_y);
    m_fvalScaleAccel_y = valueScaleAccel_y;

    double valueScaleAccel_z;
    iio_channel_attr_read_double(m_channel_accel_z, "scale", &valueScaleAccel_z);
    m_fvalScaleAccel_z = valueScaleAccel_z;

    double valueScaleAngvel_x;
    iio_channel_attr_read_double(m_channel_anglvel_x, "scale", &valueScaleAngvel_x);
    m_fvalScaleAngvel_x = valueScaleAngvel_x;

    double valueScaleAngvel_y;
    iio_channel_attr_read_double(m_channel_anglvel_y, "scale", &valueScaleAngvel_y);
    m_fvalScaleAngvel_y = valueScaleAngvel_y;

    double valueScaleAngvel_z;
    iio_channel_attr_read_double(m_channel_anglvel_z, "scale", &valueScaleAngvel_z);
    m_fvalScaleAngvel_z = valueScaleAngvel_z;

    double valueScaleRot_x;
    iio_channel_attr_read_double(m_channel_rot_x, "scale", &valueScaleRot_x);
    m_fvalScaleRot_x = valueScaleRot_x;

    double valueScaleRot_y;
    iio_channel_attr_read_double(m_channel_rot_y, "scale", &valueScaleRot_y);
    m_fvalScaleRot_y = valueScaleRot_y;

    double valueScaleRot_z;
    iio_channel_attr_read_double(m_channel_rot_z, "scale", &valueScaleRot_z);
    m_fvalScaleRot_z = valueScaleRot_z;

    double valueScaleVelocity_x;
    iio_channel_attr_read_double(m_channel_velocity_x, "scale", &valueScaleVelocity_x);
    m_fvalScaleVelocity_x = valueScaleVelocity_x;

    double valueScaleVelocity_y;
    iio_channel_attr_read_double(m_channel_velocity_y, "scale", &valueScaleVelocity_y);
    m_fvalScaleVelocity_y = valueScaleVelocity_y;

    double valueScaleVelocity_z;
    iio_channel_attr_read_double(m_channel_velocity_z, "scale", &valueScaleVelocity_z);
    m_fvalScaleVelocity_z = valueScaleVelocity_z;

    double valueScaleTemp;
    iio_channel_attr_read_double(m_channel_temp, "scale", &valueScaleTemp);
    m_fvalScaleTemp = valueScaleTemp;

    if(m_device_buffer == nullptr)
    {
        m_device_buffer = iio_device_create_buffer(m_dev, 1, false);
        if(!m_device_buffer)
        {
            throw std::runtime_error("Exception: device buffer is null");
        }
        iio_buffer_set_blocking_mode(m_device_buffer, true);
    }
}

IIOWrapper::~IIOWrapper()
{
    if(m_device_buffer != nullptr)
    {
        iio_buffer_destroy(m_device_buffer);
        m_device_buffer = nullptr;
    }
    if(m_local_context != nullptr)
    {
        iio_context_destroy(m_local_context);
        m_local_context = nullptr;
    }  
}

void IIOWrapper::update_buffer(bool& success)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    ssize_t ret =  iio_buffer_refill(m_device_buffer);
    if(ret == 0)
    {
        success = false;
    }
    else
    {
        int32_t valueRaw = 0;
        iio_channel_read(m_channel_accel_x, m_device_buffer, &valueRaw, 4);
        float fvalRaw = valueRaw;
        m_accelerometer_x = fvalRaw * m_fvalScaleAccel_x;

        valueRaw = 0;
        iio_channel_read(m_channel_accel_y, m_device_buffer, &valueRaw, 4);
        fvalRaw = valueRaw;
        m_accelerometer_y = fvalRaw * m_fvalScaleAccel_y;

        valueRaw = 0;
        iio_channel_read(m_channel_accel_z, m_device_buffer, &valueRaw, 4);
        fvalRaw = valueRaw;
        m_accelerometer_z = fvalRaw * m_fvalScaleAccel_z;

        valueRaw = 0;
        iio_channel_read(m_channel_anglvel_x, m_device_buffer, &valueRaw, 4);
        fvalRaw = valueRaw;
        m_gyroscope_x = fvalRaw * m_fvalScaleAngvel_x;

        valueRaw = 0;
        iio_channel_read(m_channel_anglvel_y, m_device_buffer, &valueRaw, 4);
        fvalRaw = valueRaw;
        m_gyroscope_y = fvalRaw * m_fvalScaleAngvel_y;

        valueRaw = 0;
        iio_channel_read(m_channel_anglvel_z, m_device_buffer, &valueRaw, 4);
        fvalRaw = valueRaw;
        m_gyroscope_z = fvalRaw * m_fvalScaleAngvel_z;

        valueRaw = 0;
        iio_channel_read(m_channel_rot_x, m_device_buffer, &valueRaw, 4);
        fvalRaw = valueRaw;
        m_rot_x = fvalRaw * m_fvalScaleRot_x;

        valueRaw = 0;
        iio_channel_read(m_channel_rot_y, m_device_buffer, &valueRaw, 4);
        fvalRaw = valueRaw;
        m_rot_y = fvalRaw * m_fvalScaleRot_y;

        valueRaw = 0;
        iio_channel_read(m_channel_rot_z, m_device_buffer, &valueRaw, 4);
        fvalRaw = valueRaw;
        m_rot_z = fvalRaw * m_fvalScaleRot_z;

        valueRaw = 0;
        iio_channel_read(m_channel_velocity_x, m_device_buffer, &valueRaw, 4);
        fvalRaw = valueRaw;
        m_velocity_x = fvalRaw * m_fvalScaleVelocity_x;

        valueRaw = 0;
        iio_channel_read(m_channel_velocity_y, m_device_buffer, &valueRaw, 4);
        fvalRaw = valueRaw;
        m_velocity_y = fvalRaw * m_fvalScaleVelocity_y;

        valueRaw = 0;
        iio_channel_read(m_channel_velocity_z, m_device_buffer, &valueRaw, 4);
        fvalRaw = valueRaw;
        m_velocity_z = fvalRaw * m_fvalScaleVelocity_z;

        valueRaw = 0;
        iio_channel_read(m_channel_temp, m_device_buffer, &valueRaw, 4);
        fvalRaw = valueRaw;
        m_temperature = fvalRaw * m_fvalScaleTemp / 1000.0;

        valueRaw = 0;
        iio_channel_read(m_channel_count, m_device_buffer, &valueRaw, 4);
        m_count = valueRaw;
        success = true;
    }
}

int32_t IIOWrapper::count()
{
    return m_count;
}

float IIOWrapper::getAccelerometerX()
{
    return m_accelerometer_x;
}

float IIOWrapper::getAccelerometerY()
{
    return m_accelerometer_y;
}

float IIOWrapper::getAccelerometerZ()
{
    return m_accelerometer_z;
}

float IIOWrapper::getGyroscopeX()
{
    return m_gyroscope_x;
}

float IIOWrapper::getGyroscopeY()
{
    return m_gyroscope_y;
}

float IIOWrapper::getGyroscopeZ()
{
    return m_gyroscope_z;
}

float IIOWrapper::getRotX()
{
    return m_rot_x;
}

float IIOWrapper::getRotY()
{
    return m_rot_y;
}

float IIOWrapper::getRotZ()
{
    return m_rot_z;
}

float IIOWrapper::getVelocityX()
{
    return m_velocity_x;
}

float IIOWrapper::getVelocityY()
{
    return m_velocity_y;
}

float IIOWrapper::getVelocityZ()
{
    return m_velocity_z;
}

float IIOWrapper::getTemperature()
{
    return m_temperature;
}

int IIOWrapper::lost_samples_count()
{
   long long valuel;
   iio_device_debug_attr_read_longlong(m_dev,"lost_samples_count", &valuel);

   return valuel;
}

std::string IIOWrapper::firmware_revision()
{
    char valuec[1024];
    iio_device_debug_attr_read(m_dev,"firmware_revision", valuec, 1024);

    std::string str(valuec);

    return str;
}

std::string IIOWrapper::firmware_date()
{
    char valuec[1024];
    iio_device_debug_attr_read(m_dev,"firmware_date", valuec, 1024);

    std::string str(valuec);

    return str;
}

int IIOWrapper::product_id()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"product_id", &valuel);

    return valuel;
}

int IIOWrapper::serial_number()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"serial_number", &valuel);

    return valuel;
}

std::string IIOWrapper::gyroscope_measurement_range()
{
    char valuec[1024];
    iio_device_debug_attr_read(m_dev,"gyroscope_measurement_range", valuec, 1024);

    std::string str(valuec);

    return str;
}

int IIOWrapper::diag_checksum_error_flag()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"diag_checksum_error_flag", &valuel);

    return valuel;
}

int IIOWrapper::diag_flash_memory_write_count_exceeded_error()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"diag_flash_memory_write_count_exceeded_error", &valuel);

    return valuel;
}

int IIOWrapper::diag_acceleration_self_test_error()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"diag_acceleration_self_test_error", &valuel);

    return valuel;
}

int IIOWrapper::diag_gyroscope2_self_test_error()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"diag_gyroscope2_self_test_error", &valuel);

    return valuel;
}

int IIOWrapper::diag_gyroscope1_self_test_error()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"diag_gyroscope1_self_test_error", &valuel);

    return valuel;
}

int IIOWrapper::diag_clock_error()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"diag_clock_error", &valuel);

    return valuel;
}

int IIOWrapper::diag_flash_memory_test_error()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"diag_flash_memory_test_error", &valuel);

    return valuel;
}

int IIOWrapper::diag_sensor_self_test_error()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"diag_sensor_self_test_error", &valuel);

    return valuel;
}

int IIOWrapper::diag_standby_mode()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"diag_standby_mode", &valuel);

    return valuel;
}

int IIOWrapper::diag_spi_communication_error()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"diag_spi_communication_error", &valuel);

    return valuel;
}

int IIOWrapper::diag_flash_memory_update_error()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"diag_flash_memory_update_error", &valuel);

    return valuel;
}

int IIOWrapper::diag_data_path_overrun()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"diag_data_path_overrun", &valuel);

    return valuel;
}

int IIOWrapper::flash_counter()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"flash_counter", &valuel);

    return valuel;
}

//----------------------------------------------------------------------

int32_t IIOWrapper::filter_size()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"filter_size", &valuel);

    return valuel;
}

void IIOWrapper::set_filter_size(int32_t val)
{
    long long valuel = val;
    iio_device_debug_attr_write_longlong(m_dev,"filter_size", valuel);
}

int32_t IIOWrapper::anglvel_x_calibbias()
{
    long long valueC;
    iio_channel_attr_read_longlong(m_channel_anglvel_x, "calibbias", &valueC);
    return valueC;
}

int32_t IIOWrapper::anglvel_y_calibbias()
{
    long long valueC;
    iio_channel_attr_read_longlong(m_channel_anglvel_y, "calibbias", &valueC);
    return valueC;
}

int32_t IIOWrapper::anglvel_z_calibbias()
{
    long long valueC;
    iio_channel_attr_read_longlong(m_channel_anglvel_z, "calibbias", &valueC);
    return valueC;
}

int32_t IIOWrapper::accel_x_calibbias()
{
    long long valueC;
    iio_channel_attr_read_longlong(m_channel_accel_x, "calibbias", &valueC);
    return valueC;
}

int32_t IIOWrapper::accel_y_calibbias()
{
    long long valueC;
    iio_channel_attr_read_longlong(m_channel_accel_y, "calibbias", &valueC);
    return valueC;
}

int32_t IIOWrapper::accel_z_calibbias()
{
    long long valueC;
    iio_channel_attr_read_longlong(m_channel_accel_z, "calibbias", &valueC);
    return valueC;
}

int32_t IIOWrapper::burst_size_selection()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"burst_size_selection", &valuel);

    return valuel;
}

int32_t IIOWrapper::burst_data_selection()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"burst_data_selection", &valuel);

    return valuel;
}

int32_t IIOWrapper::linear_acceleration_compensation()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"linear_acceleration_compensation", &valuel);

    return valuel;
}

int32_t IIOWrapper::point_of_percussion_alignment()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"point_of_percussion_alignment", &valuel);

    return valuel;
}

int32_t IIOWrapper::internal_sensor_bandwidth()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"internal_sensor_bandwidth", &valuel);

    return valuel;
}

int32_t IIOWrapper::sync_mode_select()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"sync_mode_select", &valuel);

    return valuel;
}

int32_t IIOWrapper::sync_polarity()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"sync_polarity", &valuel);

    return valuel;
}

int32_t IIOWrapper::data_ready_polarity()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"data_ready_polarity", &valuel);

    return valuel;
}

int32_t IIOWrapper::sync_signal_scale()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"sync_signal_scale", &valuel);

    return valuel;
}

int32_t IIOWrapper::decimation_filter()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"decimation_filter", &valuel);

    return valuel;
}
