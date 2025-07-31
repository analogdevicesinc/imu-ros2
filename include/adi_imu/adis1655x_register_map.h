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

#ifndef ADI_IMU__ADIS1655X_REGISTER_MAP_H_
#define ADI_IMU__ADIS1655X_REGISTER_MAP_H_

#include "adi_imu/adis_register_map.h"

namespace adi_imu
{

class Adis1655xRegisterMap : public ADISRegisterMap
{
public:
  explicit Adis1655xRegisterMap(adis_device_id device_id);

private:
  void initializeConstants() override;
  void overwriteRegisters() override;
};

}  // namespace adi_imu

#endif  // ADI_IMU__ADIS1655X_REGISTER_MAP_H_
