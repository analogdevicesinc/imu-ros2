/*******************************************************************************
 *   @file   adis_data.cpp
 *   @brief  Implementation for ros imu threads.
 *   @author Vasile Holonec (Vasile.Holonec@analog.com)
 *******************************************************************************
 * Copyright 2023(c) Analog Devices Inc.
 *
 * Licensed under the Apache License] = Version 2.0 (the "License";
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing] = software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND] = either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

#include "imu_ros2/adis_data.h"

AdisData* AdisData::singleton_= nullptr;

AdisData *AdisData::GetInstance()
{
    if(singleton_==nullptr){
        singleton_ = new AdisData();
    }
    return singleton_;
}

AdisData::AdisData()
{
  // adis1650x --------------------------------
  m_adis1650x_data["ADIS_HAS_DELTA_BURST"] = 1;

  m_adis1650x_data["ADIS_FLS_MEM_ENDURANCE"] = 10000;
  m_adis1650x_data["ADIS_MAX_SAMP_FREQ"] = 2100.0;

  m_adis1650x_data["ADIS_DIAG_STAT_ADDR"] = 0x02;
  m_adis1650x_data["ADIS_DATA_PATH_OVERRUN_POS"] = 1;
  m_adis1650x_data["ADIS_FLS_MEM_UPDATE_FAIL_POS"] = 2;
  m_adis1650x_data["ADIS_SPI_COMM_ERR_POS"] = 3;
  m_adis1650x_data["ADIS_STDBY_MODE_POS"] = 4;
  m_adis1650x_data["ADIS_SNSR_FAIL_POS"] = 5;
  m_adis1650x_data["ADIS_MEM_FAIL_POS"] = 6;
  m_adis1650x_data["ADIS_CLK_ERR_POS"] = 7;
  m_adis1650x_data["ADIS_GYRO1_FAIL_POS"] = 8;
  m_adis1650x_data["ADIS_GYRO2_FAIL_POS"] = 9;
  m_adis1650x_data["ADIS_ACCEL_FAIL_POS"] = 10;

  m_adis1650x_data["ADIS_DATA_PATH_OVERRUN"] =
                         (1 << m_adis1650x_data["ADIS_DATA_PATH_OVERRUN_POS"]);
  m_adis1650x_data["ADIS_FLS_MEM_UPDATE_FAIL"] =
                         (1 << m_adis1650x_data["ADIS_FLS_MEM_UPDATE_FAIL_POS"]);
  m_adis1650x_data["ADIS_SPI_COMM_ERR"] =
                         (1 << m_adis1650x_data["ADIS_SPI_COMM_ERR_POS"]);
  m_adis1650x_data["ADIS_STDBY_MODE"] =
                         (1 << m_adis1650x_data["ADIS_STDBY_MODE_POS"]);
  m_adis1650x_data["ADIS_SNSR_FAIL"] =
                         (1 << m_adis1650x_data["ADIS_SNSR_FAIL_POS"]);
  m_adis1650x_data["ADIS_MEM_FAIL"] =
                         (1 << m_adis1650x_data["ADIS_MEM_FAIL_POS"]);
  m_adis1650x_data["ADIS_CLK_ERR"] =
                         (1 << m_adis1650x_data["ADIS_CLK_ERR_POS"]);
  m_adis1650x_data["ADIS_GYRO1_FAIL"] =
                         (1 << m_adis1650x_data["ADIS_GYRO1_FAIL_POS"]);
  m_adis1650x_data["ADIS_GYRO2_FAIL"] =
                         (1 << m_adis1650x_data["ADIS_GYRO2_FAIL_POS"]);
  m_adis1650x_data["ADIS_ACCEL_FAIL"] =
                         (1 << m_adis1650x_data["ADIS_ACCEL_FAIL_POS"]);

  m_adis1650x_data["ADIS_RANG_MDL_ADDR"] = 0x5E;
  m_adis1650x_data["ADIS_GYRO_MEAS_RANG_POS"] = 2;

  m_adis1650x_data["ADIS_GYRO_MEAS_RANG"] = (3 << m_adis1650x_data["ADIS_GYRO_MEAS_RANG_POS"]);

  m_adis1650x_data["ADIS_MSC_CTRL_ADDR"] = 0x60;
  m_adis1650x_data["ADIS_DR_POL_POS"] = 0;
  m_adis1650x_data["ADIS_SYNC_POL_POS"] = 1;
  m_adis1650x_data["ADIS_SENS_BW_POS"] = 4;
  m_adis1650x_data["ADIS_LN_ACCL_COMP_POS"] = 7;

  m_adis1650x_data["ADIS_DR_POL"] = (1 << m_adis1650x_data["ADIS_DR_POL_POS"]);
  m_adis1650x_data["ADIS_SYNC_POL"] = (1 << m_adis1650x_data["ADIS_SYNC_POL_POS"]);
  m_adis1650x_data["ADIS_SENS_BW"] = (1 << m_adis1650x_data["ADIS_SENS_BW_POS"]);
  m_adis1650x_data["ADIS_LN_ACCL_COMP"] = (1 << m_adis1650x_data["ADIS_LN_ACCL_COMP_POS"]);

  m_adis1650x_data["ADIS_GLOB_CMD_ADDR"] = 0x68;
  m_adis1650x_data["ADIS_FACTORY_CALIBRATION_RESTORE"] = (1 << 1);
  m_adis1650x_data["ADIS_SENSOR_SELF_TEST"] = (1 << 2);
  m_adis1650x_data["ADIS_FLASH_MEMORY_UPDATE"] = (1 << 3);
  m_adis1650x_data["ADIS_FLASH_MEMORY_TEST"] = (1 << 4);
  m_adis1650x_data["ADIS_SOFTWARE_RESET_CMD"] = (1 << 7);

  m_adis1650x_data["ADIS_PT_OF_PERC_REG_ADDR"] = m_adis1650x_data["ADIS_MSC_CTRL_ADDR"];
  m_adis1650x_data["ADIS_PT_OF_PERC_ALGNMNT_POS"] = 6;
  m_adis1650x_data["ADIS_PT_OF_PERC_ALGNMNT"] = (1 << m_adis1650x_data["ADIS_PT_OF_PERC_ALGNMNT_POS"]);


  // adis1646x --------------------------------
  m_adis1646x_data["ADIS_HAS_DELTA_BURST"] = 0;

  m_adis1646x_data["ADIS_FLS_MEM_ENDURANCE"] = 10000;
  m_adis1646x_data["ADIS_MAX_SAMP_FREQ"] = 2100.0;

  m_adis1646x_data["ADIS_DIAG_STAT_ADDR"] = 0x02;
  m_adis1646x_data["ADIS_DATA_PATH_OVERRUN_POS"] = 1;
  m_adis1646x_data["ADIS_FLS_MEM_UPDATE_FAIL_POS"] = 2;
  m_adis1646x_data["ADIS_SPI_COMM_ERR_POS"] = 3;
  m_adis1646x_data["ADIS_STDBY_MODE_POS"] = 4;
  m_adis1646x_data["ADIS_SNSR_FAIL_POS"] = 5;
  m_adis1646x_data["ADIS_MEM_FAIL_POS"] = 6;
  m_adis1646x_data["ADIS_CLK_ERR_POS"] = 7;

  m_adis1646x_data["ADIS_DATA_PATH_OVERRUN"] =
                         (1 << m_adis1646x_data["ADIS_DATA_PATH_OVERRUN_POS"]);
  m_adis1646x_data["ADIS_FLS_MEM_UPDATE_FAIL"] =
                         (1 << m_adis1646x_data["ADIS_FLS_MEM_UPDATE_FAIL_POS"]);
  m_adis1646x_data["ADIS_SPI_COMM_ERR"] =
                         (1 << m_adis1646x_data["ADIS_SPI_COMM_ERR_POS"]);
  m_adis1646x_data["ADIS_STDBY_MODE"] =
                         (1 << m_adis1646x_data["ADIS_STDBY_MODE_POS"]);
  m_adis1646x_data["ADIS_SNSR_FAIL"] =
                         (1 << m_adis1646x_data["ADIS_SNSR_FAIL_POS"]);
  m_adis1646x_data["ADIS_MEM_FAIL"] =
                         (1 << m_adis1646x_data["ADIS_MEM_FAIL_POS"]);
  m_adis1646x_data["ADIS_CLK_ERR"] =
                         (1 << m_adis1646x_data["ADIS_CLK_ERR_POS"]);

  m_adis1646x_data["ADIS_RANG_MDL_ADDR"] = 0x5E;
  m_adis1646x_data["ADIS_GYRO_MEAS_RANG_POS"] = 2;

  m_adis1646x_data["ADIS_GYRO_MEAS_RANG"] = (3 << m_adis1646x_data["ADIS_GYRO_MEAS_RANG_POS"]);

  m_adis1646x_data["ADIS_MSC_CTRL_ADDR"] = 0x60;
  m_adis1646x_data["ADIS_DR_POL_POS"] = 0;
  m_adis1646x_data["ADIS_SYNC_POL_POS"] = 1;
  m_adis1646x_data["ADIS_LN_ACCL_COMP_POS"] = 7;

  m_adis1646x_data["ADIS_DR_POL"] = (1 << m_adis1646x_data["ADIS_DR_POL_POS"]);
  m_adis1646x_data["ADIS_SYNC_POL"] = (1 << m_adis1646x_data["ADIS_SYNC_POL_POS"]);

  m_adis1646x_data["ADIS_LN_ACCL_COMP"] = (1 << m_adis1646x_data["ADIS_LN_ACCL_COMP_POS"]);

  m_adis1646x_data["ADIS_NULL_CNFG_ADDR"] = 0x66;
  m_adis1646x_data["ADIS_TIME_BASE_CONTROL_POS"] = 0;
  m_adis1646x_data["ADIS_X_AXIS_GYRO_BIAS_CORR_EN_POS"] = 8;
  m_adis1646x_data["ADIS_Y_AXIS_GYRO_BIAS_CORR_EN_POS"] = 9;
  m_adis1646x_data["ADIS_Z_AXIS_GYRO_BIAS_CORR_EN_POS"] = 10;
  m_adis1646x_data["ADIS_X_AXIS_ACCEL_BIAS_CORR_EN_POS"] = 11;
  m_adis1646x_data["ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN_POS"] = 12;
  m_adis1646x_data["ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN_POS"] = 13;

  m_adis1646x_data["ADIS_TIME_BASE_CONTROL"] = 0xF;
  m_adis1646x_data["ADIS_X_AXIS_GYRO_BIAS_CORR_EN"] =
                         (1 << m_adis1646x_data["ADIS_X_AXIS_GYRO_BIAS_CORR_EN_POS"]);
  m_adis1646x_data["ADIS_Y_AXIS_GYRO_BIAS_CORR_EN"] =
                         (1 << m_adis1646x_data["ADIS_Y_AXIS_GYRO_BIAS_CORR_EN_POS"]);
  m_adis1646x_data["ADIS_Z_AXIS_GYRO_BIAS_CORR_EN"] =
                         (1 << m_adis1646x_data["ADIS_Z_AXIS_GYRO_BIAS_CORR_EN_POS"]);
  m_adis1646x_data["ADIS_X_AXIS_ACCEL_BIAS_CORR_EN"] =
                         (1 << m_adis1646x_data["ADIS_X_AXIS_ACCEL_BIAS_CORR_EN_POS"]);
  m_adis1646x_data["ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN"] =
                         (1 << m_adis1646x_data["ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN_POS"]);
  m_adis1646x_data["ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN"] =
                         (1 << m_adis1646x_data["ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN_POS"]);

  m_adis1646x_data["ADIS_GLOB_CMD_ADDR"] = 0x68;
  m_adis1646x_data["ADIS_BIAS_CORRECTION_UPDATE"] = (1 << 0);
  m_adis1646x_data["ADIS_FACTORY_CALIBRATION_RESTORE"] = (1 << 1);
  m_adis1646x_data["ADIS_SENSOR_SELF_TEST"] = (1 << 2);
  m_adis1646x_data["ADIS_FLASH_MEMORY_UPDATE"] = (1 << 3);
  m_adis1646x_data["ADIS_FLASH_MEMORY_TEST"] = (1 << 4);
  m_adis1646x_data["ADIS_SOFTWARE_RESET_CMD"] = (1 << 7);

  m_adis1646x_data["ADIS_PT_OF_PERC_REG_ADDR"] = m_adis1646x_data["ADIS_MSC_CTRL_ADDR"];
  m_adis1646x_data["ADIS_PT_OF_PERC_ALGNMNT_POS"] = 6;
  m_adis1646x_data["ADIS_PT_OF_PERC_ALGNMNT"] = (1 << m_adis1646x_data["ADIS_PT_OF_PERC_ALGNMNT_POS"]);


  // adis1647x --------------------------------
  m_adis1647x_data["ADIS_HAS_DELTA_BURST"] = 1;

  m_adis1647x_data["ADIS_FLS_MEM_ENDURANCE"] = 10000;
  m_adis1647x_data["ADIS_MAX_SAMP_FREQ"] = 2100.0;

  m_adis1647x_data["ADIS_DIAG_STAT_ADDR"] = 0x02;
  m_adis1647x_data["ADIS_DATA_PATH_OVERRUN_POS"] = 1;
  m_adis1647x_data["ADIS_FLS_MEM_UPDATE_FAIL_POS"] = 2;
  m_adis1647x_data["ADIS_SPI_COMM_ERR_POS"] = 3;
  m_adis1647x_data["ADIS_STDBY_MODE_POS"] = 4;
  m_adis1647x_data["ADIS_SNSR_FAIL_POS"] = 5;
  m_adis1647x_data["ADIS_MEM_FAIL_POS"] = 6;
  m_adis1647x_data["ADIS_CLK_ERR_POS"] = 7;

  m_adis1647x_data["ADIS_DATA_PATH_OVERRUN"] =
                         (1 << m_adis1647x_data["ADIS_DATA_PATH_OVERRUN_POS"]);
  m_adis1647x_data["ADIS_FLS_MEM_UPDATE_FAIL"] =
                         (1 << m_adis1647x_data["ADIS_FLS_MEM_UPDATE_FAIL_POS"]);
  m_adis1647x_data["ADIS_SPI_COMM_ERR"] =
                         (1 << m_adis1647x_data["ADIS_SPI_COMM_ERR_POS"]);
  m_adis1647x_data["ADIS_STDBY_MODE"] =
                         (1 << m_adis1647x_data["ADIS_STDBY_MODE_POS"]);
  m_adis1647x_data["ADIS_SNSR_FAIL"] =
                         (1 << m_adis1647x_data["ADIS_SNSR_FAIL_POS"]);
  m_adis1647x_data["ADIS_MEM_FAIL"] =
                         (1 << m_adis1647x_data["ADIS_MEM_FAIL_POS"]);
  m_adis1647x_data["ADIS_CLK_ERR"] =
                         (1 << m_adis1647x_data["ADIS_CLK_ERR_POS"]);

  m_adis1647x_data["ADIS_RANG_MDL_ADDR"] = 0x5E;
  m_adis1647x_data["ADIS_GYRO_MEAS_RANG_POS"] = 2;

  m_adis1647x_data["ADIS_GYRO_MEAS_RANG"] = (3 << m_adis1647x_data["ADIS_GYRO_MEAS_RANG_POS"]);

  m_adis1647x_data["ADIS_MSC_CTRL_ADDR"] = 0x60;
  m_adis1647x_data["ADIS_DR_POL_POS"] = 0;
  m_adis1647x_data["ADIS_SYNC_POL_POS"] = 1;
  m_adis1647x_data["ADIS_LN_ACCL_COMP_POS"] = 7;

  m_adis1647x_data["ADIS_DR_POL"] = (1 << m_adis1647x_data["ADIS_DR_POL_POS"]);
  m_adis1647x_data["ADIS_SYNC_POL"] = (1 << m_adis1647x_data["ADIS_SYNC_POL_POS"]);
  m_adis1647x_data["ADIS_LN_ACCL_COMP"] = (1 << m_adis1647x_data["ADIS_LN_ACCL_COMP_POS"]);

  m_adis1647x_data["ADIS_NULL_CNFG_ADDR"] = 0x66;
  m_adis1647x_data["ADIS_TIME_BASE_CONTROL_POS"] = 0;
  m_adis1647x_data["ADIS_X_AXIS_GYRO_BIAS_CORR_EN_POS"] = 8;
  m_adis1647x_data["ADIS_Y_AXIS_GYRO_BIAS_CORR_EN_POS"] = 9;
  m_adis1647x_data["ADIS_Z_AXIS_GYRO_BIAS_CORR_EN_POS"] = 10;
  m_adis1647x_data["ADIS_X_AXIS_ACCEL_BIAS_CORR_EN_POS"] = 11;
  m_adis1647x_data["ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN_POS"] = 12;
  m_adis1647x_data["ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN_POS"] = 13;

  m_adis1647x_data["ADIS_TIME_BASE_CONTROL"] = 0xF;
  m_adis1647x_data["ADIS_X_AXIS_GYRO_BIAS_CORR_EN"] =
                         (1 << m_adis1647x_data["ADIS_X_AXIS_GYRO_BIAS_CORR_EN_POS"]);
  m_adis1647x_data["ADIS_Y_AXIS_GYRO_BIAS_CORR_EN"] =
                         (1 << m_adis1647x_data["ADIS_Y_AXIS_GYRO_BIAS_CORR_EN_POS"]);
  m_adis1647x_data["ADIS_Z_AXIS_GYRO_BIAS_CORR_EN"] =
                         (1 << m_adis1647x_data["ADIS_Z_AXIS_GYRO_BIAS_CORR_EN_POS"]);
  m_adis1647x_data["ADIS_X_AXIS_ACCEL_BIAS_CORR_EN"] =
                         (1 << m_adis1647x_data["ADIS_X_AXIS_ACCEL_BIAS_CORR_EN_POS"]);
  m_adis1647x_data["ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN"] =
                         (1 << m_adis1647x_data["ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN_POS"]);
  m_adis1647x_data["ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN"] =
                         (1 << m_adis1647x_data["ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN_POS"]);

  m_adis1647x_data["ADIS_GLOB_CMD_ADDR"] = 0x68;
  m_adis1647x_data["ADIS_BIAS_CORRECTION_UPDATE"] = (1 << 0);
  m_adis1647x_data["ADIS_FACTORY_CALIBRATION_RESTORE"] = (1 << 1);
  m_adis1647x_data["ADIS_SENSOR_SELF_TEST"] = (1 << 2);
  m_adis1647x_data["ADIS_FLASH_MEMORY_UPDATE"] = (1 << 3);
  m_adis1647x_data["ADIS_FLASH_MEMORY_TEST"] = (1 << 4);
  m_adis1647x_data["ADIS_SOFTWARE_RESET_CMD"] = (1 << 7);

  m_adis1647x_data["ADIS_PT_OF_PERC_REG_ADDR"] = m_adis1647x_data["ADIS_MSC_CTRL_ADDR"];
  m_adis1647x_data["ADIS_PT_OF_PERC_ALGNMNT_POS"] = 6;
  m_adis1647x_data["ADIS_PT_OF_PERC_ALGNMNT"] = (1 << m_adis1647x_data["ADIS_PT_OF_PERC_ALGNMNT_POS"]);


  // adis1654x --------------------------------
  m_adis1654x_data["ADIS_HAS_DELTA_BURST"] = 1;

  m_adis1654x_data["ADIS_HAS_CALIB_SCALE"] = 1;

  m_adis1654x_data["ADIS_FLS_MEM_ENDURANCE"] = 100000;
  m_adis1654x_data["ADIS_MAX_SAMP_FREQ"] = 4250.0;

  m_adis1654x_data["ADIS_PAGE_ID_VAL"] = 0x80;

  // global commands
  m_adis1654x_data["ADIS_GLOB_CMD_PAGE_ID"] = 0x03;
  m_adis1654x_data["ADIS_GLOB_CMD_ADDR_WITHOUT_PAGE"] = 0x02;
  m_adis1654x_data["ADIS_GLOB_CMD_ADDR"] =
      (m_adis1654x_data["ADIS_PAGE_ID_VAL"] * m_adis1654x_data["ADIS_PT_OF_PERC_PAGE_ID"] + m_adis1654x_data["ADIS_GLOB_CMD_ADDR_WITHOUT_PAGE"]);

  m_adis1654x_data["ADIS_BIAS_CORRECTION_UPDATE_POS"] = 0;
  m_adis1654x_data["ADIS_SENSOR_SELF_TEST_POS"] = 1;
  m_adis1654x_data["ADIS_FLASH_MEMORY_UPDATE_POS"] = 3;
  m_adis1654x_data["ADIS_FACTORY_CALIBRATION_RESTORE_POS"] = 6;
  m_adis1654x_data["ADIS_SOFTWARE_RESET_CMD_POS"] = 7;

  m_adis1654x_data["ADIS_BIAS_CORRECTION_UPDATE"] =
                         (1 << m_adis1654x_data["ADIS_BIAS_CORRECTION_UPDATE_POS"]);
  m_adis1654x_data["ADIS_SENSOR_SELF_TEST"] =
                         (1 << m_adis1654x_data["ADIS_SENSOR_SELF_TEST_POS"]);
  m_adis1654x_data["ADIS_FLASH_MEMORY_UPDATE"] =
                         (1 << m_adis1654x_data["ADIS_FLASH_MEMORY_UPDATE_POS"]);
  m_adis1654x_data["ADIS_FACTORY_CALIBRATION_RESTORE"] =
                         (1 << m_adis1654x_data["ADIS_FACTORY_CALIBRATION_RESTORE_POS"]);
  m_adis1654x_data["ADIS_SOFTWARE_RESET_CMD"] =
                         (1 << m_adis1654x_data["ADIS_SOFTWARE_RESET_CMD_POS"]);

  // status and error flag indication
  m_adis1654x_data["ADIS_DIAG_STAT_PAGE_ID"] = 0x00;
  m_adis1654x_data["ADIS_DIAG_STAT_ADDR_WITHOUT_PAGE"] = 0x08;
  m_adis1654x_data["ADIS_DIAG_STAT_ADDR"] =
                         (m_adis1654x_data["ADIS_PAGE_ID_VAL"] * m_adis1654x_data["ADIS_DIAG_STAT_PAGE_ID"] + m_adis1654x_data["ADIS_DIAG_STAT_ADDR_WITHOUT_PAGE"]);
  m_adis1654x_data["ADIS_MEM_FAIL_POS"] = 1;
  m_adis1654x_data["ADIS_CRC_ERROR_POS"] = 2;
  m_adis1654x_data["ADIS_SPI_COMM_ERR_POS"] = 3;
  m_adis1654x_data["ADIS_SNSR_FAIL_POS"] = 5;
  m_adis1654x_data["ADIS_FLS_MEM_UPDATE_FAIL_POS"] = 6;
  m_adis1654x_data["ADIS_DATA_PATH_OVERRUN_POS"] = 7;
  m_adis1654x_data["ADIS_CLK_ERR_POS"] = 8;
  m_adis1654x_data["ADIS_WDG_TIMER_FLAG_POS"] = 15;

  m_adis1654x_data["ADIS_MEM_FAIL"] =
                         (1 << m_adis1654x_data["ADIS_MEM_FAIL_POS"]);
  m_adis1654x_data["ADIS_CRC_ERROR"] =
                         (1 << m_adis1654x_data["ADIS_CRC_ERROR_POS"]);
  m_adis1654x_data["ADIS_SPI_COMM_ERR"] =
                         (1 << m_adis1654x_data["ADIS_SPI_COMM_ERR_POS"]);
  m_adis1654x_data["ADIS_SNSR_FAIL"] =
                         (1 << m_adis1654x_data["ADIS_SNSR_FAIL_POS"]);
  m_adis1654x_data["ADIS_FLS_MEM_UPDATE_FAIL"] =
                         (1 << m_adis1654x_data["ADIS_FLS_MEM_UPDATE_FAIL_POS"]);
  m_adis1654x_data["ADIS_DATA_PATH_OVERRUN"] =
                         (1 << m_adis1654x_data["ADIS_DATA_PATH_OVERRUN_POS"]);
  m_adis1654x_data["ADIS_CLK_ERR"] =
                         (1 << m_adis1654x_data["ADIS_CLK_ERR_POS"]);
  m_adis1654x_data["ADIS_WDG_TIMER_FLAG"] =
                         (1 << m_adis1654x_data["ADIS_WDG_TIMER_FLAG_POS"]);

  // self test error flags
  m_adis1654x_data["ADIS_DIAG_STS_PAGE_ID"] = 0x00;
  m_adis1654x_data["ADIS_DIAG_STS_REG_WITHOUT_PAGE"] = 0x0A;
  m_adis1654x_data["ADIS_DIAG_STS_REG"] =
                         (m_adis1654x_data["ADIS_PAGE_ID_VAL"] * m_adis1654x_data["ADIS_DIAG_STS_PAGE_ID"] + m_adis1654x_data["ADIS_DIAG_STS_REG_WITHOUT_PAGE"]);

  m_adis1654x_data["ADIS_GYRO_ACCEL_FAIL_REG"] = m_adis1654x_data["ADIS_DIAG_STS_REG"];
  m_adis1654x_data["ADIS_GYRO_X_FAIL_POS"] = 0;
  m_adis1654x_data["ADIS_GYRO_Y_FAIL_POS"] = 1;
  m_adis1654x_data["ADIS_GYRO_Z_FAIL_POS"] = 3;
  m_adis1654x_data["ADIS_ACCEL_X_FAIL_POS"] = 4;
  m_adis1654x_data["ADIS_ACCEL_Y_FAIL_POS"] = 5;
  m_adis1654x_data["ADIS_ACCEL_Z_FAIL_POS"] = 6;

  m_adis1654x_data["ADIS_GYRO_X_FAIL"] = (1 << m_adis1654x_data["ADIS_GYRO_X_FAIL_POS"]);
  m_adis1654x_data["ADIS_GYRO_Y_FAIL"] = (1 << m_adis1654x_data["ADIS_GYRO_Y_FAIL_POS"]);
  m_adis1654x_data["ADIS_GYRO_Z_FAIL"] = (1 << m_adis1654x_data["ADIS_GYRO_Z_FAIL_POS"]);
  m_adis1654x_data["ADIS_ACCEL_X_FAIL"] = (1 << m_adis1654x_data["ADIS_ACCEL_X_FAIL_POS"]);
  m_adis1654x_data["ADIS_ACCEL_Y_FAIL"] = (1 << m_adis1654x_data["ADIS_ACCEL_Y_FAIL_POS"]);
  m_adis1654x_data["ADIS_ACCEL_Z_FAIL"] = (1 << m_adis1654x_data["ADIS_ACCEL_Z_FAIL_POS"]);

  // measurement range identifier
  m_adis1654x_data["ADIS_RANG_MDL_PAGE_ID"] = 0x03;
  m_adis1654x_data["ADIS_RANG_MDL_ADDR_WITHOUT_PAGE"] = 0x12;
  m_adis1654x_data["ADIS_RANG_MDL_ADDR"] =
                         (m_adis1654x_data["ADIS_PAGE_ID_VAL"] * m_adis1654x_data["ADIS_RANG_MDL_PAGE_ID"] + m_adis1654x_data["ADIS_RANG_MDL_ADDR_WITHOUT_PAGE"]);
  m_adis1654x_data["ADIS_GYRO_MEAS_RANG_POS"] = 2;
  m_adis1654x_data["ADIS_GYRO_MEAS_RANG"] = (3 << m_adis1654x_data["ADIS_GYRO_MEAS_RANG_POS"]);

  // point of percussion
  m_adis1654x_data["ADIS_PT_OF_PERC_PAGE_ID"] = 0x03;
  m_adis1654x_data["ADIS_PT_OF_PERC_REG_ADDR_WITHOUT_PAGE"] = 0x0A;
  m_adis1654x_data["ADIS_PT_OF_PERC_REG_ADDR"] =
                         (m_adis1654x_data["ADIS_PAGE_ID_VAL"] * m_adis1654x_data["ADIS_PT_OF_PERC_PAGE_ID"] + m_adis1654x_data["ADIS_PT_OF_PERC_REG_ADDR_WITHOUT_PAGE"]);
  m_adis1654x_data["ADIS_PT_OF_PERC_ALGNMNT_POS"] = 6;
  m_adis1654x_data["ADIS_PT_OF_PERC_ALGNMNT"] = (1 << m_adis1654x_data["ADIS_PT_OF_PERC_ALGNMNT_POS"]);

  // continuous bias estimation
  m_adis1654x_data["ADIS_NULL_CNFG_PAGE_ID"] = 0x03;
  m_adis1654x_data["ADIS_NULL_CNFG_ADDR_WITHOUT_PAGE"] =  0x0E;
  m_adis1654x_data["ADIS_NULL_CNFG_ADDR"] =
                         (m_adis1654x_data["ADIS_PAGE_ID_VAL"] * m_adis1654x_data["ADIS_NULL_CNFG_PAGE_ID"] + m_adis1654x_data["ADIS_NULL_CNFG_ADDR_WITHOUT_PAGE"]);

  m_adis1654x_data["ADIS_TIME_BASE_CONTROL_POS"] = 0;
  m_adis1654x_data["ADIS_X_AXIS_GYRO_BIAS_CORR_EN_POS"] = 8;
  m_adis1654x_data["ADIS_Y_AXIS_GYRO_BIAS_CORR_EN_POS"] = 9;
  m_adis1654x_data["ADIS_Z_AXIS_GYRO_BIAS_CORR_EN_POS"] = 10;
  m_adis1654x_data["ADIS_X_AXIS_ACCEL_BIAS_CORR_EN_POS"] = 11;
  m_adis1654x_data["ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN_POS"] = 12;
  m_adis1654x_data["ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN_POS"] = 13;

  m_adis1654x_data["ADIS_TIME_BASE_CONTROL"] = 0xF;
  m_adis1654x_data["ADIS_X_AXIS_GYRO_BIAS_CORR_EN"] = (1 << m_adis1654x_data["ADIS_X_AXIS_GYRO_BIAS_CORR_EN_POS"]);
  m_adis1654x_data["ADIS_Y_AXIS_GYRO_BIAS_CORR_EN"] = (1 << m_adis1654x_data["ADIS_Y_AXIS_GYRO_BIAS_CORR_EN_POS"]);
  m_adis1654x_data["ADIS_Z_AXIS_GYRO_BIAS_CORR_EN"] = (1 << m_adis1654x_data["ADIS_Z_AXIS_GYRO_BIAS_CORR_EN_POS"]);
  m_adis1654x_data["ADIS_X_AXIS_ACCEL_BIAS_CORR_EN"] = (1 << m_adis1654x_data["ADIS_X_AXIS_ACCEL_BIAS_CORR_EN_POS"]);
  m_adis1654x_data["ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN"] = (1 << m_adis1654x_data["ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN_POS"]);
  m_adis1654x_data["ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN"] = (1 << m_adis1654x_data["ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN_POS"]);

  // adis1655x --------------------------------
  m_adis1655x_data["ADIS_HAS_DELTA_BURST"] = 1;

  m_adis1655x_data["ADIS_HAS_CALIB_SCALE"] = 1;

  m_adis1655x_data["ADIS_FLS_MEM_ENDURANCE"] = 100000;
  m_adis1655x_data["ADIS_MAX_SAMP_FREQ"] = 4250.0;

  m_adis1655x_data["ADIS_PAGE_ID_VAL"] = 0x80;

  // global commands
  m_adis1655x_data["ADIS_GLOB_CMD_PAGE_ID"] = 0x00;
  m_adis1655x_data["ADIS_GLOB_CMD_ADDR_WITHOUT_PAGE"] = 0x50;
  m_adis1655x_data["ADIS_GLOB_CMD_ADDR"] =
      (m_adis1655x_data["ADIS_PAGE_ID_VAL"] * m_adis1655x_data["ADIS_PT_OF_PERC_PAGE_ID"] + m_adis1655x_data["ADIS_GLOB_CMD_ADDR_WITHOUT_PAGE"]);

  m_adis1655x_data["ADIS_SENSOR_SELF_TEST_POS"] = 1;
  m_adis1655x_data["ADIS_FLASH_MEMORY_UPDATE_POS"] = 3;
  m_adis1655x_data["ADIS_FACTORY_CALIBRATION_RESTORE_POS"] = 2;
  m_adis1655x_data["ADIS_SOFTWARE_RESET_CMD_POS"] = 5;

  m_adis1655x_data["ADIS_SENSOR_SELF_TEST"] =
                         (1 << m_adis1655x_data["ADIS_SENSOR_SELF_TEST_POS"]);
  m_adis1655x_data["ADIS_FLASH_MEMORY_UPDATE"] =
                         (1 << m_adis1655x_data["ADIS_FLASH_MEMORY_UPDATE_POS"]);
  m_adis1655x_data["ADIS_FACTORY_CALIBRATION_RESTORE"] =
                         (1 << m_adis1655x_data["ADIS_FACTORY_CALIBRATION_RESTORE_POS"]);
  m_adis1655x_data["ADIS_SOFTWARE_RESET_CMD"] =
                         (1 << m_adis1655x_data["ADIS_SOFTWARE_RESET_CMD_POS"]);

  // status and error flag indication
  m_adis1655x_data["ADIS_DIAG_STAT_PAGE_ID"] = 0x00;
  m_adis1655x_data["ADIS_DIAG_STAT_ADDR_WITHOUT_PAGE"] = 0x0E;
  m_adis1655x_data["ADIS_DIAG_STAT_ADDR"] =
      (m_adis1655x_data["ADIS_PAGE_ID_VAL"] * m_adis1655x_data["ADIS_DIAG_STAT_PAGE_ID"] + m_adis1655x_data["ADIS_DIAG_STAT_ADDR_WITHOUT_PAGE"]);

  m_adis1655x_data["ADIS_MEM_FAIL_POS"] = 0;
  m_adis1655x_data["ADIS_CRC_ERROR_POS"] = 1;
  m_adis1655x_data["ADIS_FLS_MEM_UPDATE_FAIL_POS"] = 2;
  m_adis1655x_data["ADIS_SNSR_FAIL_POS"] = 4;
  m_adis1655x_data["ADIS_SPI_COMM_ERR_POS"] = 6;
  m_adis1655x_data["ADIS_DATA_PATH_OVERRUN_POS"] = 7;
  m_adis1655x_data["ADIS_CLK_ERR_POS"] = 10;
  m_adis1655x_data["ADIS_WDG_TIMER_FLAG_POS"] = 15;

  m_adis1655x_data["ADIS_MEM_FAIL"] =
                         (1 << m_adis1655x_data["ADIS_MEM_FAIL_POS"]);
  m_adis1655x_data["ADIS_CRC_ERROR"] =
                         (1 << m_adis1655x_data["ADIS_CRC_ERROR_POS"]);
  m_adis1655x_data["ADIS_FLS_MEM_UPDATE_FAIL"] =
                         (1 << m_adis1655x_data["ADIS_FLS_MEM_UPDATE_FAIL_POS"]);
  m_adis1655x_data["ADIS_SNSR_FAIL"] =
                         (1 << m_adis1655x_data["ADIS_SNSR_FAIL_POS"]);
  m_adis1655x_data["ADIS_SPI_COMM_ERR"] =
                         (1 << m_adis1655x_data["ADIS_SPI_COMM_ERR_POS"]);
  m_adis1655x_data["ADIS_DATA_PATH_OVERRUN"] =
                         (1 << m_adis1655x_data["ADIS_DATA_PATH_OVERRUN_POS"]);
  m_adis1655x_data["ADIS_CLK_ERR"] =
                         (1 << m_adis1655x_data["ADIS_CLK_ERR_POS"]);
  m_adis1655x_data["ADIS_WDG_TIMER_FLAG"] =
                         (1 << m_adis1655x_data["ADIS_WDG_TIMER_FLAG_POS"]);

  // self test error flags
  m_adis1655x_data["ADIS_DIAG_STS_PAGE_ID"] = 0x00;
  m_adis1655x_data["ADIS_DIAG_STS_REG_WITHOUT_PAGE"] = 0x0A;
  m_adis1655x_data["ADIS_DIAG_STS_REG"] =
                         (m_adis1655x_data["ADIS_PAGE_ID_VAL"] * m_adis1655x_data["ADIS_DIAG_STS_PAGE_ID"] + m_adis1655x_data["ADIS_DIAG_STS_REG_WITHOUT_PAGE"]);

  m_adis1655x_data["ADIS_GYRO_ACCEL_FAIL_REG"] = m_adis1655x_data["ADIS_DIAG_STS_REG"];
  m_adis1655x_data["ADIS_GYRO_X_FAIL_POS"] = 0;
  m_adis1655x_data["ADIS_GYRO_Y_FAIL_POS"] = 2;
  m_adis1655x_data["ADIS_GYRO_Z_FAIL_POS"] = 4;
  m_adis1655x_data["ADIS_ACCEL_X_FAIL_POS"] = 6;
  m_adis1655x_data["ADIS_ACCEL_Y_FAIL_POS"] = 8;
  m_adis1655x_data["ADIS_ACCEL_Z_FAIL_POS"] = 10;

  m_adis1655x_data["ADIS_GYRO_X_FAIL"] = (3 << m_adis1655x_data["ADIS_GYRO_X_FAIL_POS"]);
  m_adis1655x_data["ADIS_GYRO_Y_FAIL"] = (3 << m_adis1655x_data["ADIS_GYRO_Y_FAIL_POS"]);
  m_adis1655x_data["ADIS_GYRO_Z_FAIL"] = (3 << m_adis1655x_data["ADIS_GYRO_Z_FAIL_POS"]);
  m_adis1655x_data["ADIS_ACCEL_X_FAIL"] = (3 << m_adis1655x_data["ADIS_ACCEL_X_FAIL_POS"]);
  m_adis1655x_data["ADIS_ACCEL_Y_FAIL"] = (3 << m_adis1655x_data["ADIS_ACCEL_Y_FAIL_POS"]);
  m_adis1655x_data["ADIS_ACCEL_Z_FAIL"] = (3 << m_adis1655x_data["ADIS_ACCEL_Z_FAIL_POS"]);

  // measurement range identifier
  m_adis1655x_data["ADIS_RANG_MDL_PAGE_ID"] = 0x00;
  m_adis1655x_data["ADIS_RANG_MDL_ADDR_WITHOUT_PAGE"] = 0x10;
  m_adis1655x_data["ADIS_RANG_MDL_ADDR"] =
                         (m_adis1655x_data["ADIS_PAGE_ID_VAL"] * m_adis1655x_data["ADIS_RANG_MDL_PAGE_ID"] + m_adis1655x_data["ADIS_RANG_MDL_ADDR_WITHOUT_PAGE"]);

  // point of percussion
  m_adis1655x_data["ADIS_PT_OF_PERC_PAGE_ID"] = 0x00;
  m_adis1655x_data["ADIS_PT_OF_PERC_REG_ADDR_WITHOUT_PAGE"] = 0x52;
  m_adis1655x_data["ADIS_PT_OF_PERC_REG_ADDR"] =
                         (m_adis1655x_data["ADIS_PAGE_ID_VAL"] * m_adis1655x_data["ADIS_PT_OF_PERC_PAGE_ID"] + m_adis1655x_data["ADIS_PT_OF_PERC_REG_ADDR_WITHOUT_PAGE"]);
  m_adis1655x_data["ADIS_PT_OF_PERC_ALGNMNT_POS"] = 4;
  m_adis1655x_data["ADIS_PT_OF_PERC_ALGNMNT"] = (1 << m_adis1655x_data["ADIS_PT_OF_PERC_ALGNMNT_POS"]);


  // adis1657x --------------------------------
  m_adis1657x_data["ADIS_HAS_DELTA_BURST"] = 1;

  m_adis1657x_data["ADIS_FLS_MEM_ENDURANCE"] = 100000;
  m_adis1657x_data["ADIS_MAX_SAMP_FREQ"] = 4100.0;

  m_adis1657x_data["ADIS_DIAG_STAT_ADDR"] = 0x02;
  m_adis1657x_data["ADIS_SNSR_INIT_FAIL_POS"] = 0;
  m_adis1657x_data["ADIS_DATA_PATH_OVERRUN_POS"] = 1;
  m_adis1657x_data["ADIS_FLS_MEM_UPDATE_FAIL_POS"] = 2;
  m_adis1657x_data["ADIS_SPI_COMM_ERR_POS"] = 3;
  m_adis1657x_data["ADIS_STDBY_MODE_POS"] = 4;
  m_adis1657x_data["ADIS_SNSR_FAIL_POS"] = 5;
  m_adis1657x_data["ADIS_MEM_FAIL_POS"] = 6;
  m_adis1657x_data["ADIS_CLK_ERR_POS"] = 7;

  m_adis1657x_data["ADIS_DATA_PATH_OVERRUN"] =
                         (1 << m_adis1657x_data["ADIS_DATA_PATH_OVERRUN_POS"]);
  m_adis1657x_data["ADIS_FLS_MEM_UPDATE_FAIL"] =
                         (1 << m_adis1657x_data["ADIS_FLS_MEM_UPDATE_FAIL_POS"]);
  m_adis1657x_data["ADIS_SPI_COMM_ERR"] =
                         (1 << m_adis1657x_data["ADIS_SPI_COMM_ERR_POS"]);
  m_adis1657x_data["ADIS_STDBY_MODE"] =
                         (1 << m_adis1657x_data["ADIS_STDBY_MODE_POS"]);
  m_adis1657x_data["ADIS_SNSR_FAIL"] =
                         (1 << m_adis1657x_data["ADIS_SNSR_FAIL_POS"]);
  m_adis1657x_data["ADIS_MEM_FAIL"] =
                         (1 << m_adis1657x_data["ADIS_MEM_FAIL_POS"]);
  m_adis1657x_data["ADIS_ADUC_MCU_FAULT"] =
                         (1 << m_adis1657x_data["ADIS_ADUC_MCU_FAULT_POS"]);

  m_adis1657x_data["ADIS_GYRO_ACCEL_FAIL_REG"] = m_adis1657x_data["ADIS_DIAG_STAT_ADDR"];
  m_adis1657x_data["ADIS_GYRO_X_FAIL_POS"] = 8;
  m_adis1657x_data["ADIS_GYRO_Y_FAIL_POS"] = 9;
  m_adis1657x_data["ADIS_GYRO_Z_FAIL_POS"] = 10;
  m_adis1657x_data["ADIS_ACCEL_X_FAIL_POS"] = 11;
  m_adis1657x_data["ADIS_ACCEL_Y_FAIL_POS"] = 12;
  m_adis1657x_data["ADIS_ACCEL_Z_FAIL_POS"] = 13;
  m_adis1657x_data["ADIS_ADUC_MCU_FAULT_POS"] = 15;

  m_adis1657x_data["ADIS_CLK_ERR"] = (1 << m_adis1657x_data["ADIS_CLK_ERR_POS"]);
  m_adis1657x_data["ADIS_GYRO_X_FAIL"] = (1 << m_adis1657x_data["ADIS_GYRO_X_FAIL_POS"]);
  m_adis1657x_data["ADIS_GYRO_Y_FAIL"] = (1 << m_adis1657x_data["ADIS_GYRO_Y_FAIL_POS"]);
  m_adis1657x_data["ADIS_GYRO_Z_FAIL"] = (1 << m_adis1657x_data["ADIS_GYRO_Z_FAIL_POS"]);
  m_adis1657x_data["ADIS_ACCEL_X_FAIL"] = (1 << m_adis1657x_data["ADIS_ACCEL_X_FAIL_POS"]);
  m_adis1657x_data["ADIS_ACCEL_Y_FAIL"] = (1 << m_adis1657x_data["ADIS_ACCEL_Y_FAIL_POS"]);
  m_adis1657x_data["ADIS_ACCEL_Z_FAIL"] = (1 << m_adis1657x_data["ADIS_ACCEL_Z_FAIL_POS"]);

  m_adis1657x_data["ADIS_RANG_MDL_ADDR"] = 0x5E;
  m_adis1657x_data["ADIS_GYRO_MEAS_RANG_POS"] = 2;
  m_adis1657x_data["ADIS_GYRO_MEAS_RANG"] =
                         (3 << m_adis1657x_data["ADIS_GYRO_MEAS_RANG_POS"]);

  m_adis1657x_data["ADIS_MSC_CTRL_ADDR"] = 0x60;
  m_adis1657x_data["ADIS_DR_POL_POS"] = 0;
  m_adis1657x_data["ADIS_SYNC_POL_POS"] = 1;
  m_adis1657x_data["ADIS_LN_ACCL_COMP_POS"] = 7;
  m_adis1657x_data["ADIS_SENS_BW_POS"] = 12;

  m_adis1657x_data["ADIS_DR_POL"] = (1 << m_adis1657x_data["ADIS_DR_POL_POS"]);
  m_adis1657x_data["ADIS_SYNC_POL"] = (1 << m_adis1657x_data["ADIS_SYNC_POL_POS"]);
  m_adis1657x_data["ADIS_LN_ACCL_COMP"] = (1 << m_adis1657x_data["ADIS_LN_ACCL_COMP_POS"]);
  m_adis1657x_data["ADIS_SENS_BW"] = (1 << m_adis1657x_data["ADIS_SENS_BW_POS"]);

  m_adis1657x_data["ADIS_NULL_CNFG_ADDR"] = 0x66;
  m_adis1657x_data["ADIS_TIME_BASE_CONTROL_POS"] = 0;
  m_adis1657x_data["ADIS_X_AXIS_GYRO_BIAS_CORR_EN_POS"] = 8;
  m_adis1657x_data["ADIS_Y_AXIS_GYRO_BIAS_CORR_EN_POS"] = 9;
  m_adis1657x_data["ADIS_Z_AXIS_GYRO_BIAS_CORR_EN_POS"] = 10;
  m_adis1657x_data["ADIS_X_AXIS_ACCEL_BIAS_CORR_EN_POS"] = 11;
  m_adis1657x_data["ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN_POS"] = 12;
  m_adis1657x_data["ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN_POS"] = 13;

  m_adis1657x_data["ADIS_TIME_BASE_CONTROL"] = 0xF;
  m_adis1657x_data["ADIS_X_AXIS_GYRO_BIAS_CORR_EN"] = (1 << m_adis1657x_data["ADIS_X_AXIS_GYRO_BIAS_CORR_EN_POS"]);
  m_adis1657x_data["ADIS_Y_AXIS_GYRO_BIAS_CORR_EN"] = (1 << m_adis1657x_data["ADIS_Y_AXIS_GYRO_BIAS_CORR_EN_POS"]);
  m_adis1657x_data["ADIS_Z_AXIS_GYRO_BIAS_CORR_EN"] = (1 << m_adis1657x_data["ADIS_Z_AXIS_GYRO_BIAS_CORR_EN_POS"]);
  m_adis1657x_data["ADIS_X_AXIS_ACCEL_BIAS_CORR_EN"] = (1 << m_adis1657x_data["ADIS_X_AXIS_ACCEL_BIAS_CORR_EN_POS"]);
  m_adis1657x_data["ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN"] = (1 << m_adis1657x_data["ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN_POS"]);
  m_adis1657x_data["ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN"] = (1 << m_adis1657x_data["ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN_POS"]);

  m_adis1657x_data["ADIS_GLOB_CMD_ADDR"] = 0x68;
  m_adis1657x_data["ADIS_BIAS_CORRECTION_UPDATE"] = (1 << 0);
  m_adis1657x_data["ADIS_FACTORY_CALIBRATION_RESTORE"] = (1 << 1);
  m_adis1657x_data["ADIS_SENSOR_SELF_TEST"] = (1 << 2);
  m_adis1657x_data["ADIS_FLASH_MEMORY_UPDATE"] = (1 << 3);
  m_adis1657x_data["ADIS_FLASH_MEMORY_TEST"] = (1 << 4);
  m_adis1657x_data["ADIS_SOFTWARE_RESET_CMD"] = (1 << 7);

  m_adis1657x_data["ADIS_PT_OF_PERC_REG_ADDR"] =  m_adis1657x_data["ADIS_MSC_CTRL_ADDR"];
  m_adis1657x_data["ADIS_PT_OF_PERC_ALGNMNT_POS"] = 6;
  m_adis1657x_data["ADIS_PT_OF_PERC_ALGNMNT"] =  (1 << m_adis1657x_data["ADIS_PT_OF_PERC_ALGNMNT_POS"]);

  m_devicesMap["adis1646x"] = m_adis1646x_data;
  m_devicesMap["adis1647x"] = m_adis1647x_data;
  m_devicesMap["adis1650x"] = m_adis1650x_data;
  m_devicesMap["adis1654x"] = m_adis1654x_data;
  m_devicesMap["adis1655x"] = m_adis1655x_data;
  m_devicesMap["adis1657x"] = m_adis1657x_data;


}

AdisData::~AdisData() {}

void AdisData::setDeviceName(std::string devname)
{
  m_deviceName = devname;
}

int32_t AdisData::getDeviceValue(std::string key)
{
  int32_t val = m_devicesMap[m_deviceName][key];
  return val;
}

bool AdisData::checkDeviceName(std::string devname)
{
  return (m_deviceName == devname);
}
