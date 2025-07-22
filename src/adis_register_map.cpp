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

#include "adi_imu/adis_register_map.h"

#include <iostream>

#include "adi_imu/utils/adis_device_registry.h"
#include "adi_imu/utils/adis_register_definitions.h"

namespace adi_imu
{
extern const std::unordered_map<ADISRegister, std::string> registerNames;

ADISRegisterMap::ADISRegisterMap(adis_device_id device_id)
: m_device_id(device_id), m_device_family("unknown")
{
  m_device_name = ADISDeviceRegistry::getDeviceNameFromId(device_id);
  m_device_family = ADISDeviceRegistry::getDeviceFamily(device_id);
};

uint32_t ADISRegisterMap::get(ADISRegister reg) const
{
  auto it = m_register_map.find(reg);
  if (it == m_register_map.end()) {
    std::runtime_error error("Register not found in the register map.");
    throw error;
  }
  return it->second;
}

bool ADISRegisterMap::hasDeltaBurst() const
{
  return has(ADISRegister::HAS_DELTA_BURST) && get(ADISRegister::HAS_DELTA_BURST);
}

std::string ADISRegisterMap::getDeviceFamily() const { return m_device_family; }
std::string ADISRegisterMap::getDeviceName() const { return m_device_name; }
adis_device_id ADISRegisterMap::getDeviceID() const { return m_device_id; }

void ADISRegisterMap::log() const
{
  for (const auto & pair : m_register_map) {
    auto reg_name = registerNames.at(pair.first);
    std::cout << "[" << reg_name << "]: 0x" << std::hex << pair.second << std::dec << std::endl;
  }
}

void ADISRegisterMap::set(ADISRegister reg, uint32_t value) { m_register_map[reg] = value; }

void ADISRegisterMap::initialize()
{
  this->initSharedRegisters();
  this->initializeConstants();
  this->computeBitMasks();
  this->postComputeBitmask();
  this->overwriteRegisters();
}

void ADISRegisterMap::initSharedRegisters()
{
  set(ADISRegister::DELTANG_X_LOW_REG, 0x24);
  set(ADISRegister::DELTANG_X_OUT_REG, 0x26);
  set(ADISRegister::DELTANG_Y_LOW_REG, 0x28);
  set(ADISRegister::DELTANG_Y_OUT_REG, 0x2A);
  set(ADISRegister::DELTANG_Z_LOW_REG, 0x2C);
  set(ADISRegister::DELTANG_Z_OUT_REG, 0x2E);
  set(ADISRegister::DELTVEL_X_LOW_REG, 0x30);
  set(ADISRegister::DELTVEL_X_OUT_REG, 0x32);
  set(ADISRegister::DELTVEL_Y_LOW_REG, 0x34);
  set(ADISRegister::DELTVEL_Y_OUT_REG, 0x36);
  set(ADISRegister::DELTVEL_Z_LOW_REG, 0x38);
  set(ADISRegister::DELTVEL_Z_OUT_REG, 0x3A);
}

void ADISRegisterMap::computeBitMasks()
{
  if (has(ADISRegister::DATA_PATH_OVERRUN_POS)) {
    set(ADISRegister::DATA_PATH_OVERRUN, 1 << get(ADISRegister::DATA_PATH_OVERRUN_POS));
  }
  if (has(ADISRegister::WDG_TIMER_FLAG_POS)) {
    set(ADISRegister::WDG_TIMER_FLAG, 1 << get(ADISRegister::WDG_TIMER_FLAG_POS));
  }
  if (has(ADISRegister::FLS_MEM_UPDATE_FAIL_POS)) {
    set(ADISRegister::FLS_MEM_UPDATE_FAIL, 1 << get(ADISRegister::FLS_MEM_UPDATE_FAIL_POS));
  }
  if (has(ADISRegister::SPI_COMM_ERR_POS)) {
    set(ADISRegister::SPI_COMM_ERR, 1 << get(ADISRegister::SPI_COMM_ERR_POS));
  }
  if (has(ADISRegister::STDBY_MODE_POS)) {
    set(ADISRegister::STDBY_MODE, 1 << get(ADISRegister::STDBY_MODE_POS));
  }

  if (has(ADISRegister::SNSR_INIT_FAIL_POS)) {
    set(ADISRegister::SNSR_INIT_FAIL, 1 << get(ADISRegister::SNSR_INIT_FAIL_POS));
  }
  if (has(ADISRegister::SNSR_FAIL_POS)) {
    set(ADISRegister::SNSR_FAIL, 1 << get(ADISRegister::SNSR_FAIL_POS));
  }

  if (has(ADISRegister::MEM_FAIL_POS)) {
    set(ADISRegister::MEM_FAIL, 1 << get(ADISRegister::MEM_FAIL_POS));
  }
  if (has(ADISRegister::ADUC_MCU_FAULT_POS)) {
    set(ADISRegister::ADUC_MCU_FAULT, 1 << get(ADISRegister::ADUC_MCU_FAULT_POS));
  }
  if (has(ADISRegister::CRC_ERROR_POS)) {
    set(ADISRegister::CRC_ERROR, 1 << get(ADISRegister::CRC_ERROR_POS));
  }
  if (has(ADISRegister::CLK_ERR_POS)) {
    set(ADISRegister::CLK_ERR, 1 << get(ADISRegister::CLK_ERR_POS));
  }
  if (has(ADISRegister::GYRO1_FAIL_POS)) {
    set(ADISRegister::GYRO1_FAIL, 1 << get(ADISRegister::GYRO1_FAIL_POS));
  }
  if (has(ADISRegister::GYRO2_FAIL_POS)) {
    set(ADISRegister::GYRO2_FAIL, 1 << get(ADISRegister::GYRO2_FAIL_POS));
  }
  if (has(ADISRegister::ACCEL_FAIL_POS)) {
    set(ADISRegister::ACCEL_FAIL, 1 << get(ADISRegister::ACCEL_FAIL_POS));
  }
  if (has(ADISRegister::GYRO_MEAS_RANG_POS)) {
    set(
      ADISRegister::GYRO_MEAS_RANG,
      3 << get(ADISRegister::GYRO_MEAS_RANG_POS));  // 3 was first seen value
  }

  if (has(ADISRegister::DR_POL_POS)) {
    set(ADISRegister::DR_POL, 1 << get(ADISRegister::DR_POL_POS));
  }
  if (has(ADISRegister::SYNC_POL_POS)) {
    set(ADISRegister::SYNC_POL, 1 << get(ADISRegister::SYNC_POL_POS));
  }

  if (has(ADISRegister::SENS_BW_POS)) {
    set(ADISRegister::SENS_BW, 1 << get(ADISRegister::SENS_BW_POS));
  }

  if (has(ADISRegister::LN_ACCL_COMP_POS)) {
    set(ADISRegister::LN_ACCL_COMP, 1 << get(ADISRegister::LN_ACCL_COMP_POS));
  }

  if (has(ADISRegister::X_AXIS_GYRO_BIAS_CORR_EN_POS)) {
    set(
      ADISRegister::X_AXIS_GYRO_BIAS_CORR_EN, 1 << get(ADISRegister::X_AXIS_GYRO_BIAS_CORR_EN_POS));
  }
  if (has(ADISRegister::Y_AXIS_GYRO_BIAS_CORR_EN_POS)) {
    set(
      ADISRegister::Y_AXIS_GYRO_BIAS_CORR_EN, 1 << get(ADISRegister::Y_AXIS_GYRO_BIAS_CORR_EN_POS));
  }
  if (has(ADISRegister::Z_AXIS_GYRO_BIAS_CORR_EN_POS)) {
    set(
      ADISRegister::Z_AXIS_GYRO_BIAS_CORR_EN, 1 << get(ADISRegister::Z_AXIS_GYRO_BIAS_CORR_EN_POS));
  }

  if (has(ADISRegister::X_AXIS_ACCEL_BIAS_CORR_EN_POS)) {
    set(
      ADISRegister::X_AXIS_ACCEL_BIAS_CORR_EN,
      1 << get(ADISRegister::X_AXIS_ACCEL_BIAS_CORR_EN_POS));
  }
  if (has(ADISRegister::Y_AXIS_ACCEL_BIAS_CORR_EN_POS)) {
    set(
      ADISRegister::Y_AXIS_ACCEL_BIAS_CORR_EN,
      1 << get(ADISRegister::Y_AXIS_ACCEL_BIAS_CORR_EN_POS));
  }
  if (has(ADISRegister::Z_AXIS_ACCEL_BIAS_CORR_EN_POS)) {
    set(
      ADISRegister::Z_AXIS_ACCEL_BIAS_CORR_EN,
      1 << get(ADISRegister::Z_AXIS_ACCEL_BIAS_CORR_EN_POS));
  }

  if (has(ADISRegister::PT_OF_PERC_ALGNMNT_POS)) {
    set(ADISRegister::PT_OF_PERC_ALGNMNT, 1 << get(ADISRegister::PT_OF_PERC_ALGNMNT_POS));
  }
  if (
    has(ADISRegister::PAGE_ID_VAL) && has(ADISRegister::PT_OF_PERC_PAGE_ID) &&
    has(ADISRegister::GLOB_CMD_ADDR_WITHOUT_PAGE)) {
    auto value = get(ADISRegister::PAGE_ID_VAL) * get(ADISRegister::PT_OF_PERC_PAGE_ID) +
                 get(ADISRegister::GLOB_CMD_ADDR_WITHOUT_PAGE);
    set(ADISRegister::GLOB_CMD_ADDR, value);
  }
  if (
    has(ADISRegister::PAGE_ID_VAL) && has(ADISRegister::PT_OF_PERC_PAGE_ID) &&
    has(ADISRegister::PT_OF_PERC_REG_ADDR_WITHOUT_PAGE)) {
    auto value = get(ADISRegister::PAGE_ID_VAL) * get(ADISRegister::PT_OF_PERC_PAGE_ID) +
                 get(ADISRegister::PT_OF_PERC_REG_ADDR_WITHOUT_PAGE);
    set(ADISRegister::PT_OF_PERC_REG_ADDR, value);
  }
  if (
    has(ADISRegister::PAGE_ID_VAL) && has(ADISRegister::NULL_CNFG_PAGE_ID) &&
    has(ADISRegister::NULL_CNFG_ADDR_WITHOUT_PAGE)) {
    auto value = get(ADISRegister::PAGE_ID_VAL) * get(ADISRegister::NULL_CNFG_PAGE_ID) +
                 get(ADISRegister::NULL_CNFG_ADDR_WITHOUT_PAGE);
    set(ADISRegister::NULL_CNFG_ADDR, value);
  }

  if (has(ADISRegister::BIAS_CORRECTION_UPDATE_POS)) {
    set(ADISRegister::BIAS_CORRECTION_UPDATE, 1 << get(ADISRegister::BIAS_CORRECTION_UPDATE_POS));
  }
  if (has(ADISRegister::SENSOR_SELF_TEST_POS)) {
    set(ADISRegister::SENSOR_SELF_TEST, 1 << get(ADISRegister::SENSOR_SELF_TEST_POS));
  }
  if (has(ADISRegister::FLASH_MEMORY_UPDATE_POS)) {
    set(ADISRegister::FLASH_MEMORY_UPDATE, 1 << get(ADISRegister::FLASH_MEMORY_UPDATE_POS));
  }
  if (has(ADISRegister::FLASH_MEMORY_TEST_POS)) {
    set(ADISRegister::FLASH_MEMORY_TEST, 1 << get(ADISRegister::FLASH_MEMORY_TEST_POS));
  }
  if (has(ADISRegister::FACTORY_CALIBRATION_RESTORE_POS)) {
    set(
      ADISRegister::FACTORY_CALIBRATION_RESTORE,
      1 << get(ADISRegister::FACTORY_CALIBRATION_RESTORE_POS));
  }
  if (has(ADISRegister::SOFTWARE_RESET_CMD_POS)) {
    set(ADISRegister::SOFTWARE_RESET_CMD, 1 << get(ADISRegister::SOFTWARE_RESET_CMD_POS));
  }
  if (
    has(ADISRegister::PAGE_ID_VAL) && has(ADISRegister::DIAG_STAT_PAGE_ID) &&
    has(ADISRegister::DIAG_STAT_ADDR_WITHOUT_PAGE)) {
    auto value = get(ADISRegister::PAGE_ID_VAL) * get(ADISRegister::DIAG_STAT_PAGE_ID) +
                 get(ADISRegister::DIAG_STAT_ADDR_WITHOUT_PAGE);
    set(ADISRegister::DIAG_STAT_ADDR, value);
  }
  if (
    has(ADISRegister::PAGE_ID_VAL) && has(ADISRegister::DIAG_STS_PAGE_ID) &&
    has(ADISRegister::DIAG_STS_REG_WITHOUT_PAGE)) {
    auto value = get(ADISRegister::PAGE_ID_VAL) * get(ADISRegister::DIAG_STS_PAGE_ID) +
                 get(ADISRegister::DIAG_STS_REG_WITHOUT_PAGE);
    set(ADISRegister::DIAG_STS_REG, value);
  }

  if (has(ADISRegister::GYRO_X_FAIL_POS)) {
    set(ADISRegister::GYRO_X_FAIL, 1 << get(ADISRegister::GYRO_X_FAIL_POS));
  }
  if (has(ADISRegister::GYRO_Y_FAIL_POS)) {
    set(ADISRegister::GYRO_Y_FAIL, 1 << get(ADISRegister::GYRO_Y_FAIL_POS));
  }
  if (has(ADISRegister::GYRO_Z_FAIL_POS)) {
    set(ADISRegister::GYRO_Z_FAIL, 1 << get(ADISRegister::GYRO_Z_FAIL_POS));
  }

  if (has(ADISRegister::ACCEL_X_FAIL_POS)) {
    set(ADISRegister::ACCEL_X_FAIL, 1 << get(ADISRegister::ACCEL_X_FAIL_POS));
  }
  if (has(ADISRegister::ACCEL_Y_FAIL_POS)) {
    set(ADISRegister::ACCEL_Y_FAIL, 1 << get(ADISRegister::ACCEL_Y_FAIL_POS));
  }
  if (has(ADISRegister::ACCEL_Z_FAIL_POS)) {
    set(ADISRegister::ACCEL_Z_FAIL, 1 << get(ADISRegister::ACCEL_Z_FAIL_POS));
  }

  if (
    has(ADISRegister::PAGE_ID_VAL) && has(ADISRegister::RANG_MDL_PAGE_ID) &&
    has(ADISRegister::RANG_MDL_ADDR_WITHOUT_PAGE)) {
    auto value = get(ADISRegister::PAGE_ID_VAL) * get(ADISRegister::RANG_MDL_PAGE_ID) +
                 get(ADISRegister::RANG_MDL_ADDR_WITHOUT_PAGE);
    set(ADISRegister::RANG_MDL_ADDR, value);
  }
}

void ADISRegisterMap::postComputeBitmask()
{
  if (has(ADISRegister::DIAG_STS_REG)) {
    set(ADISRegister::GYRO_ACCEL_FAIL_REG, get(ADISRegister::DIAG_STS_REG));
  }

  if (has(ADISRegister::MSC_CTRL_ADDR)) {
    set(ADISRegister::PT_OF_PERC_REG_ADDR, get(ADISRegister::MSC_CTRL_ADDR));
  }
}

bool ADISRegisterMap::has(ADISRegister reg) const
{
  return m_register_map.find(reg) != m_register_map.end();
}

}  // namespace adi_imu
