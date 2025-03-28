#ifndef ADIS1657X_DATA_ACCESS_H
#define ADIS1657X_DATA_ACCESS_H

#define ADIS_HAS_DELTA_BURST

#define ADIS_FLS_MEM_ENDURANCE 100000
#define ADIS_MAX_SAMP_FREQ 4100.0

#define ADIS_DIAG_STAT_ADDR 0x02
#define ADIS_SNSR_INIT_FAIL_POS 0
#define ADIS_DATA_PATH_OVERRUN_POS 1
#define ADIS_FLS_MEM_UPDATE_FAIL_POS 2
#define ADIS_SPI_COMM_ERR_POS 3
#define ADIS_STDBY_MODE_POS 4
#define ADIS_SNSR_FAIL_POS 5
#define ADIS_MEM_FAIL_POS 6
#define ADIS_CLK_ERR_POS 7
#define ADIS_DATA_PATH_OVERRUN (1 << ADIS_DATA_PATH_OVERRUN_POS)
#define ADIS_FLS_MEM_UPDATE_FAIL (1 << ADIS_FLS_MEM_UPDATE_FAIL_POS)
#define ADIS_SPI_COMM_ERR (1 << ADIS_SPI_COMM_ERR_POS)
#define ADIS_STDBY_MODE (1 << ADIS_STDBY_MODE_POS)
#define ADIS_SNSR_FAIL (1 << ADIS_SNSR_FAIL_POS)
#define ADIS_MEM_FAIL (1 << ADIS_MEM_FAIL_POS)
#define ADIS_ADUC_MCU_FAULT (1 << ADIS_ADUC_MCU_FAULT_POS)

#define ADIS_GYRO_ACCEL_FAIL_REG ADIS_DIAG_STAT_ADDR
#define ADIS_GYRO_X_FAIL_POS 8
#define ADIS_GYRO_Y_FAIL_POS 9
#define ADIS_GYRO_Z_FAIL_POS 10
#define ADIS_ACCEL_X_FAIL_POS 11
#define ADIS_ACCEL_Y_FAIL_POS 12
#define ADIS_ACCEL_Z_FAIL_POS 13
#define ADIS_ADUC_MCU_FAULT_POS 15
#define ADIS_CLK_ERR (1 << ADIS_CLK_ERR_POS)
#define ADIS_GYRO_X_FAIL (1 << ADIS_GYRO_X_FAIL_POS)
#define ADIS_GYRO_Y_FAIL (1 << ADIS_GYRO_Y_FAIL_POS)
#define ADIS_GYRO_Z_FAIL (1 << ADIS_GYRO_Z_FAIL_POS)
#define ADIS_ACCEL_X_FAIL (1 << ADIS_ACCEL_X_FAIL_POS)
#define ADIS_ACCEL_Y_FAIL (1 << ADIS_ACCEL_Y_FAIL_POS)
#define ADIS_ACCEL_Z_FAIL (1 << ADIS_ACCEL_Z_FAIL_POS)

#define ADIS_RANG_MDL_ADDR 0x5E
#define ADIS_GYRO_MEAS_RANG_POS 2
#define ADIS_GYRO_MEAS_RANG (3 << ADIS_GYRO_MEAS_RANG_POS)

#define ADIS_MSC_CTRL_ADDR 0x60
#define ADIS_DR_POL_POS 0
#define ADIS_SYNC_POL_POS 1
#define ADIS_LN_ACCL_COMP_POS 7
#define ADIS_SENS_BW_POS 12

#define ADIS_DR_POL (1 << ADIS_DR_POL_POS)
#define ADIS_SYNC_POL (1 << ADIS_SYNC_POL_POS)
#define ADIS_LN_ACCL_COMP (1 << ADIS_LN_ACCL_COMP_POS)
#define ADIS_SENS_BW (1 << ADIS_SENS_BW_POS)

#define ADIS_NULL_CNFG_ADDR 0x66
#define ADIS_TIME_BASE_CONTROL_POS 0
#define ADIS_X_AXIS_GYRO_BIAS_CORR_EN_POS 8
#define ADIS_Y_AXIS_GYRO_BIAS_CORR_EN_POS 9
#define ADIS_Z_AXIS_GYRO_BIAS_CORR_EN_POS 10
#define ADIS_X_AXIS_ACCEL_BIAS_CORR_EN_POS 11
#define ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN_POS 12
#define ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN_POS 13

#define ADIS_TIME_BASE_CONTROL 0xF
#define ADIS_X_AXIS_GYRO_BIAS_CORR_EN (1 << ADIS_X_AXIS_GYRO_BIAS_CORR_EN_POS)
#define ADIS_Y_AXIS_GYRO_BIAS_CORR_EN (1 << ADIS_Y_AXIS_GYRO_BIAS_CORR_EN_POS)
#define ADIS_Z_AXIS_GYRO_BIAS_CORR_EN (1 << ADIS_Z_AXIS_GYRO_BIAS_CORR_EN_POS)
#define ADIS_X_AXIS_ACCEL_BIAS_CORR_EN (1 << ADIS_X_AXIS_ACCEL_BIAS_CORR_EN_POS)
#define ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN (1 << ADIS_Y_AXIS_ACCEL_BIAS_CORR_EN_POS)
#define ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN (1 << ADIS_Z_AXIS_ACCEL_BIAS_CORR_EN_POS)

#define ADIS_GLOB_CMD_ADDR 0x68
#define ADIS_BIAS_CORRECTION_UPDATE (1 << 0)
#define ADIS_FACTORY_CALIBRATION_RESTORE (1 << 1)
#define ADIS_SENSOR_SELF_TEST (1 << 2)
#define ADIS_FLASH_MEMORY_UPDATE (1 << 3)
#define ADIS_FLASH_MEMORY_TEST (1 << 4)
#define ADIS_SOFTWARE_RESET_CMD (1 << 7)

// Point of percussion
#define ADIS_PT_OF_PERC_REG_ADDR ADIS_MSC_CTRL_ADDR
#define ADIS_PT_OF_PERC_ALGNMNT_POS 6
#define ADIS_PT_OF_PERC_ALGNMNT (1 << ADIS_PT_OF_PERC_ALGNMNT_POS)

#endif
