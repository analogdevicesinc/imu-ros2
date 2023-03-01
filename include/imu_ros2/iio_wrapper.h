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

class IIOWrapper {
public:
    IIOWrapper();
    ~IIOWrapper();

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

private:
    static struct iio_context * m_network_context;
    struct iio_context * m_object_context;
    struct iio_device *m_dev;

    struct iio_channel *m_channel_accel_x;
    struct iio_channel *m_channel_accel_y;
    struct iio_channel *m_channel_accel_z;

    struct iio_channel *m_channel_anglvel_x;
    struct iio_channel *m_channel_anglvel_y;
    struct iio_channel *m_channel_anglvel_z;

    struct iio_channel *m_channel_rot_x;
    struct iio_channel *m_channel_rot_y;
    struct iio_channel *m_channel_rot_z;

    struct iio_channel *m_channel_velocity_x;
    struct iio_channel *m_channel_velocity_y;
    struct iio_channel *m_channel_velocity_z;

    struct iio_channel *m_channel_temp;
};

#endif // IIO_WRAPPER_H
