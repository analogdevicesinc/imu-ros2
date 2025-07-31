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

#include "adi_imu/adis1655x_register_map.h"

namespace adi_imu
{

Adis1655xRegisterMap::Adis1655xRegisterMap(adis_device_id device_id) : ADISRegisterMap(device_id)
{
  initialize();
}

void Adis1655xRegisterMap::initializeConstants()
{
  set(ADISRegister::HAS_DELTA_BURST, 1);
  set(ADISRegister::HAS_CALIB_SCALE, 1);

  set(ADISRegister::FLS_MEM_ENDURANCE, 100000);
  set(ADISRegister::MAX_SAMP_FREQ, 4250);

  // value to add to reg addr per page
  set(ADISRegister::PAGE_ID_VAL, 0x80);

  // global commands
  set(ADISRegister::GLOB_CMD_PAGE_ID, 0x00);
  set(ADISRegister::GLOB_CMD_ADDR_WITHOUT_PAGE, 0x50);

  set(ADISRegister::SENSOR_SELF_TEST_POS, 1);
  set(ADISRegister::FLASH_MEMORY_UPDATE_POS, 3);
  set(ADISRegister::FACTORY_CALIBRATION_RESTORE_POS, 2);
  set(ADISRegister::SOFTWARE_RESET_CMD_POS, 5);

  // status and error flag indication
  set(ADISRegister::DIAG_STAT_PAGE_ID, 0x00);
  set(ADISRegister::DIAG_STAT_ADDR_WITHOUT_PAGE, 0x0E);

  set(ADISRegister::MEM_FAIL_POS, 0);
  set(ADISRegister::CRC_ERROR_POS, 1);
  set(ADISRegister::FLS_MEM_UPDATE_FAIL_POS, 2);
  set(ADISRegister::SNSR_FAIL_POS, 4);
  set(ADISRegister::SPI_COMM_ERR_POS, 6);
  set(ADISRegister::DATA_PATH_OVERRUN_POS, 7);
  set(ADISRegister::CLK_ERR_POS, 10);
  set(ADISRegister::WDG_TIMER_FLAG_POS, 15);

  // self test error flags
  set(ADISRegister::DIAG_STS_PAGE_ID, 0x00);
  set(ADISRegister::DIAG_STS_REG_WITHOUT_PAGE, 0x0F);

  set(ADISRegister::GYRO_X_FAIL_POS, 0);
  set(ADISRegister::GYRO_Y_FAIL_POS, 2);
  set(ADISRegister::GYRO_Z_FAIL_POS, 4);
  set(ADISRegister::ACCEL_X_FAIL_POS, 6);
  set(ADISRegister::ACCEL_Y_FAIL_POS, 8);
  set(ADISRegister::ACCEL_Z_FAIL_POS, 10);

  // measurement range identifier
  set(ADISRegister::RANG_MDL_PAGE_ID, 0x00);
  set(ADISRegister::RANG_MDL_ADDR_WITHOUT_PAGE, 0x10);

  // point of percussion
  set(ADISRegister::PT_OF_PERC_PAGE_ID, 0x00);
  set(ADISRegister::PT_OF_PERC_REG_ADDR_WITHOUT_PAGE, 0x52);

  set(ADISRegister::PT_OF_PERC_ALGNMNT_POS, 4);
}

void Adis1655xRegisterMap::overwriteRegisters()
{
  // For this ADIS device, the shift position works differently than in other devices.
  set(ADISRegister::GYRO_X_FAIL, 3 << get(ADISRegister::GYRO_X_FAIL_POS));
  set(ADISRegister::GYRO_Y_FAIL, 3 << get(ADISRegister::GYRO_Y_FAIL_POS));
  set(ADISRegister::GYRO_Z_FAIL, 3 << get(ADISRegister::GYRO_Z_FAIL_POS));
  set(ADISRegister::ACCEL_X_FAIL, 3 << get(ADISRegister::ACCEL_X_FAIL_POS));
  set(ADISRegister::ACCEL_Y_FAIL, 3 << get(ADISRegister::ACCEL_Y_FAIL_POS));
  set(ADISRegister::ACCEL_Z_FAIL, 3 << get(ADISRegister::ACCEL_Z_FAIL_POS));
}

}  // namespace adi_imu
