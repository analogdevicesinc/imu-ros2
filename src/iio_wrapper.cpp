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

struct iio_context* IIOWrapper::m_network_context = nullptr;

IIOWrapper::IIOWrapper()
{
    if(m_network_context == nullptr)
    {
        std::string uri = "ip:127.0.0.1"; // TODO: pass uri from cmd line

        m_network_context = iio_create_context_from_uri(uri.c_str());

        m_object_context =  iio_context_clone(m_network_context);
    }
    else
    {
        m_object_context =  iio_context_clone(m_network_context);
    }
    m_dev = iio_context_find_device(m_object_context, "adis16505");
    m_channel_accel_x = iio_device_find_channel(m_dev, "accel_x", false);
    m_channel_accel_y = iio_device_find_channel(m_dev, "accel_y", false);
    m_channel_accel_z = iio_device_find_channel(m_dev, "accel_z", false);



    m_channel_anglvel_x = iio_device_find_channel(m_dev, "anglvel_x", false);
    m_channel_anglvel_y = iio_device_find_channel(m_dev, "anglvel_y", false);
    m_channel_anglvel_z = iio_device_find_channel(m_dev, "anglvel_z", false);

    m_channel_rot_x = iio_device_find_channel(m_dev, "rot_x", false);
    m_channel_rot_y = iio_device_find_channel(m_dev, "rot_y", false);
    m_channel_rot_z = iio_device_find_channel(m_dev, "rot_z", false);

    m_channel_velocity_x = iio_device_find_channel(m_dev, "velocity_x", false);
    m_channel_velocity_y = iio_device_find_channel(m_dev, "velocity_y", false);
    m_channel_velocity_z = iio_device_find_channel(m_dev, "velocity_z", false);

    m_channel_temp = iio_device_find_channel(m_dev, "temp", false);

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
}

IIOWrapper::~IIOWrapper()
{
    iio_context_destroy(m_object_context);
    if(m_network_context != nullptr)
    {
        iio_context_destroy(m_network_context);
        m_network_context = nullptr;
    }

}


float IIOWrapper::getAccelerometerX()
{
    long long valueRaw;
    iio_channel_attr_read_longlong(m_channel_accel_x, "raw", &valueRaw);
    float fvalRaw = valueRaw;

    float result = fvalRaw * m_fvalScaleAccel_x;
    return result;
}

float IIOWrapper::getAccelerometerY()
{
    long long valueRaw;
    iio_channel_attr_read_longlong(m_channel_accel_y, "raw", &valueRaw);
    float fvalRaw = valueRaw;

    float result = fvalRaw * m_fvalScaleAccel_y;
    return result;
}

float IIOWrapper::getAccelerometerZ()
{
    long long valueRaw;
    iio_channel_attr_read_longlong(m_channel_accel_z, "raw", &valueRaw);
    float fvalRaw = valueRaw;

    float result = fvalRaw * m_fvalScaleAccel_z;
    return result;
}

float IIOWrapper::getGyroscopeX()
{
    long long valueRaw;
    iio_channel_attr_read_longlong(m_channel_anglvel_x, "raw", &valueRaw);
    float fvalRaw = valueRaw;

    float result = fvalRaw * m_fvalScaleAngvel_x;
    return result;
}

float IIOWrapper::getGyroscopeY()
{
    long long valueRaw;
    iio_channel_attr_read_longlong(m_channel_anglvel_y, "raw", &valueRaw);
    float fvalRaw = valueRaw;

    float result = fvalRaw * m_fvalScaleAngvel_y;
    return result;
}

float IIOWrapper::getGyroscopeZ()
{
    long long valueRaw;
    iio_channel_attr_read_longlong(m_channel_anglvel_z, "raw", &valueRaw);
    float fvalRaw = valueRaw;

    float result = fvalRaw * m_fvalScaleAngvel_z;
    return result;
}

float IIOWrapper::getRotX()
{
    long long valueRaw;
    iio_channel_attr_read_longlong(m_channel_rot_x, "raw", &valueRaw);
    float fvalRaw = valueRaw;

    float result = fvalRaw * m_fvalScaleRot_x;
    return result;
}

float IIOWrapper::getRotY()
{
    long long valueRaw;
    iio_channel_attr_read_longlong(m_channel_rot_y, "raw", &valueRaw);
    float fvalRaw = valueRaw;

    float result = fvalRaw * m_fvalScaleRot_y;
    return result;
}

float IIOWrapper::getRotZ()
{
    long long valueRaw;
    iio_channel_attr_read_longlong(m_channel_rot_z, "raw", &valueRaw);
    float fvalRaw = valueRaw;

    float result = fvalRaw * m_fvalScaleRot_z;
    return result;
}

float IIOWrapper::getVelocityX()
{
    long long valueRaw;
    iio_channel_attr_read_longlong(m_channel_velocity_x, "raw", &valueRaw);
    float fvalRaw = valueRaw;

    float result = fvalRaw * m_fvalScaleVelocity_x;
    return result;
}

float IIOWrapper::getVelocityY()
{
    long long valueRaw;
    iio_channel_attr_read_longlong(m_channel_velocity_y, "raw", &valueRaw);
    float fvalRaw = valueRaw;

    float result = fvalRaw * m_fvalScaleVelocity_y;
    return result;
}

float IIOWrapper::getVelocityZ()
{
    long long valueRaw;
    iio_channel_attr_read_longlong(m_channel_velocity_z, "raw", &valueRaw);
    float fvalRaw = valueRaw;

    float result = fvalRaw * m_fvalScaleVelocity_z;
    return result;
}

float IIOWrapper::getTemperature()
{
    long long valueRaw;
    iio_channel_attr_read_longlong(m_channel_temp, "raw", &valueRaw);
    float fvalRaw = valueRaw;

    float result = fvalRaw * m_fvalScaleTemp / 1000.0;
    return result;
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
