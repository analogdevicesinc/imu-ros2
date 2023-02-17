/***************************************************************************//**
*   @file   gyroscope_data_provider.h
*   @brief  Provide gyroscope data
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

#ifndef GYROSCOPE_DATA_PROVIDER_H
#define GYROSCOPE_DATA_PROVIDER_H

#include "imu_ros2/gyroscope_data_provider_interface.h"
#include "imu_ros2/iio_wrapper.h"

class GyroscopeDataProvider : public GyroscopeDataProviderInterface {


public:
    GyroscopeDataProvider();
    ~GyroscopeDataProvider();

    void init() override;
    imu_ros2::msg::GyroscopeData getData(int count) override;

private:
    IIOWrapper m_iioWrapper;
};

#endif // GYROSCOPE_DATA_PROVIDER_STRING_H
