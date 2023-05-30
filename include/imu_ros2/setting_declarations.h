/***************************************************************************//**
*   @file   setting_declarations.h
*   @brief  Declare state for configuration mode
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

#ifndef SETTING_DECLARATIONS_H
#define SETTING_DECLARATIONS_H

#define USER_PARAM_SETTING_MODE 0
#define DEVICE_PARAM_SETTING_MODE 1
#define DEVICE_CONTINUOUS_SAMPLING_MODE 2

enum IIODeviceName {
    ADIS16505 = 0,
    ADIS1657X
};

#define ACCEL_GYRO_BUFFERED_DATA 0
#define DELTAVEL_DELTAANG_BUFFERED_DATA 1

#endif // SETTING_DECLARATIONS_H
