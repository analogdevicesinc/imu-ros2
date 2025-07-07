#include "adi_imu/adis1650x_register_map.h"

namespace adi_imu
{

Adis1650xRegisterMap::Adis1650xRegisterMap(adis_device_id device_id) : ADISRegisterMap(device_id)
{
  initialize();
}

void Adis1650xRegisterMap::initializeConstants()
{
  set(ADISRegister::HAS_DELTA_BURST, 1);

  set(ADISRegister::FLS_MEM_ENDURANCE, 10000);
  set(ADISRegister::MAX_SAMP_FREQ, 2100);

  set(ADISRegister::DIAG_STAT_ADDR, 0x02);
  set(ADISRegister::DATA_PATH_OVERRUN_POS, 1);
  set(ADISRegister::FLS_MEM_UPDATE_FAIL_POS, 2);
  set(ADISRegister::SPI_COMM_ERR_POS, 3);
  set(ADISRegister::STDBY_MODE_POS, 4);
  set(ADISRegister::SNSR_FAIL_POS, 5);
  set(ADISRegister::MEM_FAIL_POS, 6);
  set(ADISRegister::CLK_ERR_POS, 7);
  set(ADISRegister::GYRO1_FAIL_POS, 8);
  set(ADISRegister::GYRO2_FAIL_POS, 9);
  set(ADISRegister::ACCEL_FAIL_POS, 10);

  set(ADISRegister::RANG_MDL_ADDR, 0x5E);
  set(ADISRegister::GYRO_MEAS_RANG_POS, 2);

  set(ADISRegister::MSC_CTRL_ADDR, 0x60);
  set(ADISRegister::DR_POL_POS, 0);
  set(ADISRegister::SYNC_POL_POS, 1);
  set(ADISRegister::SENS_BW_POS, 4);
  set(ADISRegister::LN_ACCL_COMP_POS, 7);

  set(ADISRegister::GLOB_CMD_ADDR, 0x68);
  set(ADISRegister::FACTORY_CALIBRATION_RESTORE_POS, 1);
  set(ADISRegister::SENSOR_SELF_TEST_POS, 2);
  set(ADISRegister::FLASH_MEMORY_UPDATE_POS, 3);
  set(ADISRegister::FLASH_MEMORY_TEST_POS, 4);
  set(ADISRegister::SOFTWARE_RESET_CMD_POS, 7);

  set(ADISRegister::PT_OF_PERC_ALGNMNT_POS, 6);
};

}  // namespace adi_imu