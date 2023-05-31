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

std::string IIOWrapper::s_device_name;
std::string IIOWrapper::s_device_trigger_name;
IIODeviceName IIOWrapper::s_device_name_enum = IIODeviceName::ADIS1657X;

IIOWrapper::IIOWrapper()
{
    if(m_local_context == nullptr)
    {
        m_local_context = iio_create_local_context();
        if(!m_local_context)
            throw std::runtime_error("Exception: local context is null");

        int count = iio_context_get_devices_count(m_local_context);
        if(count > 0)
        {
            struct iio_device *dev = iio_context_get_device(m_local_context, 0);
            const char *name = iio_device_get_name(dev);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp_device_name"), "device name: %s", name);

            IIOWrapper::s_device_name = std::string(name);

            std::list<std::string> adis16505_x {"adis16505", "adis16505-1", "adis16505-2", "adis16505-3"};

            bool found = (std::find(adis16505_x.begin(), adis16505_x.end(),
                                    IIOWrapper::s_device_name) != adis16505_x.end());
            if(found)
            {
                IIOWrapper::s_device_name_enum = IIODeviceName::ADIS16505;
            }

            std::list<std::string> adis1657x {"adis16577", "adis16577-1", "adis16577-2", "adis16577-3"};

            bool found7x = (std::find(adis1657x.begin(), adis1657x.end(),
                                      IIOWrapper::s_device_name) != adis1657x.end());
            if(found7x)
            {
                IIOWrapper::s_device_name_enum = IIODeviceName::ADIS1657X;
            }

            if(!found && !found7x)
                RCLCPP_INFO(rclcpp::get_logger("rclcpp_device"), "device not found");
        }
        else
            throw std::runtime_error("Exception: no device found");

    }

    if(m_dev == nullptr)
    {
        m_dev = iio_context_find_device(m_local_context, IIOWrapper::s_device_name.c_str());
        if(!m_dev)
            throw std::runtime_error("Exception: device data is null");
    }

    if(m_devtrigger == nullptr)
    {
        std::string triggnerName = IIOWrapper::s_device_name + "-dev0";
        m_devtrigger = iio_context_find_device(m_local_context, triggnerName.c_str());
        if(!m_devtrigger)
            throw std::runtime_error("Exception: device trigger data is null");
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
            throw std::runtime_error("Exception: device buffer is null");
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

void IIOWrapper::unload()
{
    if(m_device_buffer != nullptr)
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        iio_buffer_destroy(m_device_buffer);
        m_device_buffer = nullptr;
    }
}

void IIOWrapper::load()
{
    if(m_device_buffer == nullptr)
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_device_buffer = iio_device_create_buffer(m_dev, 1, false);
        if(!m_device_buffer)
            throw std::runtime_error("Exception: device buffer is null");
        iio_buffer_set_blocking_mode(m_device_buffer, true);
    }
}

bool IIOWrapper::isBufferLoaded()
{
    return (m_device_buffer != nullptr);
}

void IIOWrapper::update_buffer(bool& success)
{
    if(m_device_buffer == nullptr)
    {
        success = false;
        return;
    }
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

bool IIOWrapper::getRegAccelerometerX(double& result)
{
    long long valueRaw;
    int ret = iio_channel_attr_read_longlong(m_channel_accel_x, "raw", &valueRaw);

    result = valueRaw * m_fvalScaleAccel_x;
    return (ret == 0);
}

bool IIOWrapper::getRegAccelerometerY(double& result)
{
    long long valueRaw;
    int ret = iio_channel_attr_read_longlong(m_channel_accel_y, "raw", &valueRaw);

    result = valueRaw * m_fvalScaleAccel_y;
    return (ret == 0);
}

bool IIOWrapper::getRegAccelerometerZ(double& result)
{
    long long valueRaw;
    int ret = iio_channel_attr_read_longlong(m_channel_accel_z, "raw", &valueRaw);

    result = valueRaw * m_fvalScaleAccel_z;
    return (ret == 0);
}

bool IIOWrapper::getRegGyroscopeX(double& result)
{
    long long valueRaw;
    int ret = iio_channel_attr_read_longlong(m_channel_anglvel_x, "raw", &valueRaw);

    result = valueRaw * m_fvalScaleAngvel_x;
    return (ret == 0);
}

bool IIOWrapper::getRegGyroscopeY(double& result)
{
    long long valueRaw;
    int ret = iio_channel_attr_read_longlong(m_channel_anglvel_y, "raw", &valueRaw);

    result = valueRaw * m_fvalScaleAngvel_y;
    return (ret == 0);
}

bool IIOWrapper::getRegGyroscopeZ(double& result)
{
    long long valueRaw;
    int ret = iio_channel_attr_read_longlong(m_channel_anglvel_z, "raw", &valueRaw);

    result = valueRaw * m_fvalScaleAngvel_z;
    return (ret == 0);
}

bool IIOWrapper::getRegRotX(double& result)
{
    long long valueRaw;
    int ret = iio_channel_attr_read_longlong(m_channel_rot_x, "raw", &valueRaw);

    result = valueRaw * m_fvalScaleRot_x;
    return (ret == 0);
}

bool IIOWrapper::getRegRotY(double& result)
{
    long long valueRaw;
    int ret = iio_channel_attr_read_longlong(m_channel_rot_y, "raw", &valueRaw);

    result = valueRaw * m_fvalScaleRot_y;
    return (ret == 0);
}

bool IIOWrapper::getRegRotZ(double& result)
{
    long long valueRaw;
    int ret = iio_channel_attr_read_longlong(m_channel_rot_z, "raw", &valueRaw);

    result = valueRaw * m_fvalScaleRot_z;
    return (ret == 0);
}

bool IIOWrapper::getRegVelocityX(double& result)
{
    long long valueRaw;
    int ret = iio_channel_attr_read_longlong(m_channel_velocity_x, "raw", &valueRaw);

    result = valueRaw * m_fvalScaleVelocity_x;
    return (ret == 0);
}

bool IIOWrapper::getRegVelocityY(double& result)
{
    long long valueRaw;
    int ret = iio_channel_attr_read_longlong(m_channel_velocity_y, "raw", &valueRaw);

    result = valueRaw * m_fvalScaleVelocity_y;
    return (ret == 0);
}

bool IIOWrapper::getRegVelocityZ(double& result)
{
    long long valueRaw;
    int ret = iio_channel_attr_read_longlong(m_channel_velocity_z, "raw", &valueRaw);

    result = valueRaw * m_fvalScaleVelocity_z;
    return (ret == 0);
}

bool IIOWrapper::getRegTemperature(double& result)
{
    long long valueRaw;
    int ret = iio_channel_attr_read_longlong(m_channel_temp, "raw", &valueRaw);

    result = valueRaw * m_fvalScaleTemp / 1000.0;
    return (ret == 0);
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

int IIOWrapper::update_filter_size(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"filter_size", valuel);
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

// --------------------------------------------------------------------------

int IIOWrapper::update_burst_size_selection(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"burst_size_selection", valuel);
}

int IIOWrapper::update_burst_data_selection(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"burst_data_selection", valuel);
}

int IIOWrapper::update_linear_acceleration_compensation(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"linear_acceleration_compensation", valuel);
}

int IIOWrapper::update_point_of_percussion_alignment(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"point_of_percussion_alignment", valuel);
}

int IIOWrapper::update_internal_sensor_bandwidth(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"internal_sensor_bandwidth", valuel);
}

int IIOWrapper::update_sync_mode_select(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"sync_mode_select", valuel);
}

int IIOWrapper::update_sync_polarity(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"sync_polarity", valuel);
}

int IIOWrapper::update_data_ready_polarity(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"data_ready_polarity", valuel);
}

int IIOWrapper::update_sync_signal_scale(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"sync_signal_scale", valuel);
}

int IIOWrapper::update_decimation_filter(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"decimation_filter", valuel);
}

int IIOWrapper::update_accel_calibbias_x(int32_t val)
{
    long long valuel = val;
    return iio_channel_attr_write_longlong(m_channel_accel_x, "calibbias", valuel);
}

int IIOWrapper::update_accel_calibbias_y(int32_t val)
{
    long long valuel = val;
    return iio_channel_attr_write_longlong(m_channel_accel_y, "calibbias", valuel);
}

int IIOWrapper::update_accel_calibbias_z(int32_t val)
{
    long long valuel = val;
    return iio_channel_attr_write_longlong(m_channel_accel_z, "calibbias", valuel);
}

int IIOWrapper::update_anglvel_calibbias_x(int32_t val)
{
    long long valuel = val;
    return iio_channel_attr_write_longlong(m_channel_anglvel_x, "calibbias", valuel);
}

int IIOWrapper::update_anglvel_calibbias_y(int32_t val)
{
    long long valuel = val;
    return iio_channel_attr_write_longlong(m_channel_anglvel_y, "calibbias", valuel);
}

int IIOWrapper::update_anglvel_calibbias_z(int32_t val)
{
    long long valuel = val;
    return iio_channel_attr_write_longlong(m_channel_anglvel_z, "calibbias", valuel);
}

int IIOWrapper::update_sampling_frequency(double val)
{
    return iio_device_attr_write_double(m_dev, "sampling_frequency", val);
}

double IIOWrapper::sampling_frequency()
{
    double valuel;
    iio_device_attr_read_double(m_dev,"sampling_frequency", &valuel);
    return valuel;
}

int IIOWrapper::software_reset()
{
    return iio_device_debug_attr_write_longlong(m_dev,"software_reset", 1);
}

int IIOWrapper::flash_memory_test()
{
    return iio_device_debug_attr_write_longlong(m_dev,"flash_memory_test", 1);
}

int IIOWrapper::flash_memory_update()
{
    return iio_device_debug_attr_write_longlong(m_dev,"flash_memory_update", 1);
}

int IIOWrapper::sensor_self_test()
{
    return iio_device_debug_attr_write_longlong(m_dev,"sensor_self_test", 1);
}

int IIOWrapper::factory_calibration_restore()
{
    return iio_device_debug_attr_write_longlong(m_dev,"factory_calibration_restore", 1);
}

//-------------------------------------------

int32_t IIOWrapper::z_axis_accelerometer_bias_correction_enable()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"z_axis_accelerometer_bias_correction_enable", &valuel);

    return valuel;
}

int32_t IIOWrapper::y_axis_accelerometer_bias_correction_enable()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"y_axis_accelerometer_bias_correction_enable", &valuel);

    return valuel;
}

int32_t IIOWrapper::x_axis_accelerometer_bias_correction_enable()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"x_axis_accelerometer_bias_correction_enable", &valuel);

    return valuel;
}

int32_t IIOWrapper::z_axis_gyroscope_bias_correction_enable()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"z_axis_gyroscope_bias_correction_enable", &valuel);

    return valuel;
}

int32_t IIOWrapper::y_axis_gyroscope_bias_correction_enable()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"y_axis_gyroscope_bias_correction_enable", &valuel);

    return valuel;
}

int32_t IIOWrapper::x_axis_gyroscope_bias_correction_enable()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"x_axis_gyroscope_bias_correction_enable", &valuel);

    return valuel;
}

int32_t IIOWrapper::internal_sync_enable_4khz()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"internal_sync_enable_4khz", &valuel);

    return valuel;
}

int32_t IIOWrapper::timestamp32()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"timestamp32", &valuel);

    return valuel;
}

int32_t IIOWrapper::fifo_watermark_interrupt_polarity()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"fifo_watermark_interrupt_polarity", &valuel);

    return valuel;
}

int32_t IIOWrapper::fifo_watermark_interrupt_enable()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"fifo_watermark_interrupt_enable", &valuel);

    return valuel;
}

int32_t IIOWrapper::fifo_overflow_behavior()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"fifo_overflow_behavior", &valuel);

    return valuel;
}

int32_t IIOWrapper::fifo_enable()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"fifo_enable", &valuel);

    return valuel;
}

int32_t IIOWrapper::bias_correction_time_base_control()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"bias_correction_time_base_control", &valuel);

    return valuel;
}

int32_t IIOWrapper::fifo_watermark_threshold_level()
{
    long long valuel;
    iio_device_debug_attr_read_longlong(m_dev,"fifo_watermark_threshold_level", &valuel);

    return valuel;
}

int IIOWrapper::update_z_axis_accelerometer_bias_correction_enable(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"z_axis_accelerometer_bias_correction_enable", valuel);
}

int IIOWrapper::update_y_axis_accelerometer_bias_correction_enable(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"y_axis_accelerometer_bias_correction_enable", valuel);
}

int IIOWrapper::update_x_axis_accelerometer_bias_correction_enable(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"x_axis_accelerometer_bias_correction_enable", valuel);
}

int IIOWrapper::update_z_axis_gyroscope_bias_correction_enable(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"z_axis_gyroscope_bias_correction_enable", valuel);
}

int IIOWrapper::update_y_axis_gyroscope_bias_correction_enable(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"y_axis_gyroscope_bias_correction_enable", valuel);
}

int IIOWrapper::update_x_axis_gyroscope_bias_correction_enable(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"x_axis_gyroscope_bias_correction_enable", valuel);
}

int IIOWrapper::update_internal_sync_enable_4khz(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"internal_sync_enable_4khz", valuel);
}

int IIOWrapper::update_timestamp32(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"timestamp32", valuel);
}

int IIOWrapper::update_fifo_watermark_interrupt_polarity(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"fifo_watermark_interrupt_polarity", valuel);
}

int IIOWrapper::update_fifo_watermark_interrupt_enable(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"fifo_watermark_interrupt_enable", valuel);
}

int IIOWrapper::update_fifo_overflow_behavior(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"fifo_overflow_behavior", valuel);
}

int IIOWrapper::update_fifo_enable(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"fifo_enable", valuel);
}

int IIOWrapper::update_bias_correction_time_base_control(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"bias_correction_time_base_control", valuel);
}

int IIOWrapper::update_fifo_watermark_threshold_level(int32_t val)
{
    long long valuel = val;
    return iio_device_debug_attr_write_longlong(m_dev,"fifo_watermark_threshold_level", valuel);
}
