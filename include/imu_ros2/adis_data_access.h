#ifndef ADIS_DATA_ACCESS_H
#define ADIS_DATA_ACCESS_H

#define DELTAVEL_DELTAANG_BUFFERED_DATA 0
#define ACCEL_GYRO_BUFFERED_DATA 1
#define IMU_STD_MSG_DATA 2
#define FULL_MEASURED_DATA 3

#define ADIS_DELTANG_X_LOW_REG 0x24
#define ADIS_DELTANG_X_OUT_REG 0x26
#define ADIS_DELTANG_Y_LOW_REG 0x28
#define ADIS_DELTANG_Y_OUT_REG 0x2A
#define ADIS_DELTANG_Z_LOW_REG 0x2C
#define ADIS_DELTANG_Z_OUT_REG 0x2E
#define ADIS_DELTVEL_X_LOW_REG 0x30
#define ADIS_DELTVEL_X_OUT_REG 0x32
#define ADIS_DELTVEL_Y_LOW_REG 0x34
#define ADIS_DELTVEL_Y_OUT_REG 0x36
#define ADIS_DELTVEL_Z_LOW_REG 0x38
#define ADIS_DELTVEL_Z_OUT_REG 0x3A

enum adis_device_id
{
  ADIS16465_1,
  ADIS16465_2,
  ADIS16465_3,
  ADIS16467_1,
  ADIS16467_2,
  ADIS16467_3,
  ADIS16470,
  ADIS16475_1,
  ADIS16475_2,
  ADIS16475_3,
  ADIS16477_1,
  ADIS16477_2,
  ADIS16477_3,
  ADIS16500,
  ADIS16501,
  ADIS16505_1,
  ADIS16505_2,
  ADIS16505_3,
  ADIS16507_1,
  ADIS16507_2,
  ADIS16507_3,
  ADIS16545_1,
  ADIS16545_2,
  ADIS16545_3,
  ADIS16547_1,
  ADIS16547_2,
  ADIS16547_3,
  ADIS16550,
  ADIS16575_2,
  ADIS16575_3,
  ADIS16576_2,
  ADIS16576_3,
  ADIS16577_2,
  ADIS16577_3,
};

#if defined(ADIS1646X)
#include "adis1646x/adis1646x_data_access.h"
#elif defined(ADIS1647X)
#include "adis1647x/adis1647x_data_access.h"
#elif defined(ADIS1650X)
#include "adis1650x/adis1650x_data_access.h"
#elif defined(ADIS1654X)
#include "adis1654x/adis1654x_data_access.h"
#elif defined(ADIS1655X)
#include "adis1655x/adis1655x_data_access.h"
#elif defined(ADIS1657X)
#include "adis1657x/adis1657x_data_access.h"
#else
#error "No device defined."
#endif

#endif
