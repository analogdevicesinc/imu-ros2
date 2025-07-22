// Copyright 2025 Analog Devices, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ADI_IMU__ADIS_DEVICE_FACTORY_H_
#define ADI_IMU__ADIS_DEVICE_FACTORY_H_

#include <memory>
#include <string>
#include <vector>

#include "adi_imu/adis_register_map.h"
#include "adi_imu/utils/adis_register_definitions.h"

namespace adi_imu
{

class ADISDeviceFactory
{
public:
  static std::shared_ptr<ADISRegisterMap> make(adis_device_id device_id);
  static std::shared_ptr<ADISRegisterMap> make(const std::string & device_name);
};

}  // namespace adi_imu

#endif  // ADI_IMU__ADIS_DEVICE_FACTORY_H_
