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

#include "adi_imu/adis1657x_register_map.h"

namespace adi_imu
{

Adis1657xRegisterMap::Adis1657xRegisterMap(adis_device_id device_id) : ADISRegisterMap(device_id)
{
  initialize();
}

void Adis1657xRegisterMap::initializeConstants()
{
  set(ADISRegister::HAS_DELTA_BURST, 1);

  set(ADISRegister::FLS_MEM_ENDURANCE, 100000);
  set(ADISRegister::MAX_SAMP_FREQ, 4100);

  set(ADISRegister::DIAG_STAT_ADDR, 0x02);
  set(ADISRegister::SNSR_INIT_FAIL_POS, 0);
  set(ADISRegister::DATA_PATH_OVERRUN_POS, 1);
  set(ADISRegister::FLS_MEM_UPDATE_FAIL_POS, 2);
  set(ADISRegister::SPI_COMM_ERR_POS, 3);
  set(ADISRegister::STDBY_MODE_POS, 4);
  set(ADISRegister::SNSR_FAIL_POS, 5);
  set(ADISRegister::MEM_FAIL_POS, 6);
  set(ADISRegister::CLK_ERR_POS, 7);

  set(ADISRegister::GYRO_X_FAIL_POS, 8);
  set(ADISRegister::GYRO_Y_FAIL_POS, 9);
  set(ADISRegister::GYRO_Z_FAIL_POS, 10);
  set(ADISRegister::ACCEL_X_FAIL_POS, 11);
  set(ADISRegister::ACCEL_Y_FAIL_POS, 12);
  set(ADISRegister::ACCEL_Z_FAIL_POS, 13);
  set(ADISRegister::ADUC_MCU_FAULT_POS, 15);

  set(ADISRegister::RANG_MDL_ADDR, 0x5E);
  set(ADISRegister::GYRO_MEAS_RANG_POS, 2);

  set(ADISRegister::MSC_CTRL_ADDR, 0x60);
  set(ADISRegister::DR_POL_POS, 0);
  set(ADISRegister::SYNC_POL_POS, 1);
  set(ADISRegister::LN_ACCL_COMP_POS, 7);
  set(ADISRegister::SENS_BW_POS, 12);

  set(ADISRegister::NULL_CNFG_ADDR, 0x66);
  set(ADISRegister::TIME_BASE_CONTROL_POS, 0);
  set(ADISRegister::X_AXIS_GYRO_BIAS_CORR_EN_POS, 8);
  set(ADISRegister::Y_AXIS_GYRO_BIAS_CORR_EN_POS, 9);
  set(ADISRegister::Z_AXIS_GYRO_BIAS_CORR_EN_POS, 10);
  set(ADISRegister::X_AXIS_ACCEL_BIAS_CORR_EN_POS, 11);
  set(ADISRegister::Y_AXIS_ACCEL_BIAS_CORR_EN_POS, 12);
  set(ADISRegister::Z_AXIS_ACCEL_BIAS_CORR_EN_POS, 13);

  set(ADISRegister::TIME_BASE_CONTROL, 0xF);

  set(ADISRegister::GLOB_CMD_ADDR, 0x68);

  set(ADISRegister::BIAS_CORRECTION_UPDATE_POS, 0);
  set(ADISRegister::FACTORY_CALIBRATION_RESTORE_POS, 1);
  set(ADISRegister::SENSOR_SELF_TEST_POS, 2);
  set(ADISRegister::FLASH_MEMORY_UPDATE_POS, 3);
  set(ADISRegister::FLASH_MEMORY_TEST_POS, 4);
  set(ADISRegister::SOFTWARE_RESET_CMD_POS, 7);

  set(ADISRegister::PT_OF_PERC_ALGNMNT_POS, 6);
}

void Adis1657xRegisterMap::overwriteRegisters()
{
  set(ADISRegister::GYRO_ACCEL_FAIL_REG, get(ADISRegister::DIAG_STAT_ADDR));
}

}  // namespace adi_imu
