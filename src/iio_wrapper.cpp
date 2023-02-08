#include "imu_ros2/iio_wrapper.h"

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <sstream>

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
    long long value;
    iio_channel_attr_read_longlong(m_channel_accel_x, "raw", &value);
    float fval = value;
    return fval;
}

float IIOWrapper::getAccelerometerY()
{
    long long value;
    iio_channel_attr_read_longlong(m_channel_accel_y, "raw", &value);
    float fval = value;
    return fval;
}

float IIOWrapper::getAccelerometerZ()
{
    long long value;
    iio_channel_attr_read_longlong(m_channel_accel_z, "raw", &value);
    float fval = value;
    return fval;
}

float IIOWrapper::getGyroscopeX()
{
    long long value;
    iio_channel_attr_read_longlong(m_channel_anglvel_x, "raw", &value);
    float fval = value;
    return fval;
}

float IIOWrapper::getGyroscopeY()
{
    long long value;
    iio_channel_attr_read_longlong(m_channel_anglvel_y, "raw", &value);
    float fval = value;
    return fval;
}

float IIOWrapper::getGyroscopeZ()
{
    long long value;
    iio_channel_attr_read_longlong(m_channel_anglvel_z, "raw", &value);
    float fval = value;
    return fval;
}
