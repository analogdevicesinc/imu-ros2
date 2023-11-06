#ifndef ADIS_DATA_ACCESS_H
#define ADIS_DATA_ACCESS_H

#define DELTAVEL_DELTAANG_BUFFERED_DATA 0
#define ACCEL_GYRO_BUFFERED_DATA 1
#define IMU_STD_MSG_DATA 2
#define FULL_MEASURED_DATA 3

#if defined(ADIS1646X)
#include "adis1646x/adis1646x_data_access.h"
#elif defined(ADIS1647X)
#include "adis1647x/adis1647x_data_access.h"
#elif defined(ADIS1650X)
#include "adis1650x/adis1650x_data_access.h"
#elif defined(ADIS1657X)
#include "adis1657x/adis1657x_data_access.h"
#else
#error "No device defined."
#endif

#endif
