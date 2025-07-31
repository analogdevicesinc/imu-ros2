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

#ifndef ADI_IMU__UTILS__ADIS_DEVICE_REGISTRY_H_
#define ADI_IMU__UTILS__ADIS_DEVICE_REGISTRY_H_

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "adi_imu/utils/adis_register_definitions.h"

namespace adi_imu
{

class ADISDeviceRegistry
{
public:
  // Device name <-> ID conversion
  static adis_device_id getDeviceIdFromString(const std::string & device_name);
  static std::string getDeviceNameFromId(adis_device_id device_id);

  // Device queries
  static std::vector<std::string> getSupportedDeviceNames();
  static std::vector<adis_device_id> getSupportedDeviceIds();
  static bool isDeviceSupported(const std::string & device_name);
  static bool isDeviceSupported(adis_device_id device_id);

  // Device family classification
  static std::string getDeviceFamily(adis_device_id device_id);
  static std::string getDeviceFamily(const std::string & device_name);

private:
  static const std::unordered_map<std::string, adis_device_id> m_supported_devices;
  // static const std::unordered_map<adis_device_id, std::string> m_device_families;
};

}  // namespace adi_imu

#endif  // ADI_IMU__UTILS__ADIS_DEVICE_REGISTRY_H_
