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

#ifndef ADI_IMU__ADIS_REGISTER_MAP_H_
#define ADI_IMU__ADIS_REGISTER_MAP_H_

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "adi_imu/utils/adis_register_definitions.h"

namespace adi_imu
{

class ADISRegisterMap
{
protected:
  explicit ADISRegisterMap(adis_device_id device_id);

public:
  void initialize();

  virtual ~ADISRegisterMap() = default;

  bool has(ADISRegister reg) const;
  uint32_t get(ADISRegister reg) const;

  bool hasDeltaBurst() const;
  std::string getDeviceFamily() const;
  std::string getDeviceName() const;
  adis_device_id getDeviceID() const;
  void log() const;

protected:
  void set(ADISRegister reg, uint32_t value);

  virtual void initSharedRegisters() final;
  virtual void computeBitMasks() final;
  virtual void postComputeBitmask() final;
  virtual void initializeConstants() = 0;
  virtual void overwriteRegisters() {}

protected:
  std::unordered_map<ADISRegister, uint32_t> m_register_map;

  adis_device_id m_device_id;
  std::string m_device_name;
  std::string m_device_family;
};

}  // namespace adi_imu

#endif  // ADI_IMU__ADIS_REGISTER_MAP_H_
