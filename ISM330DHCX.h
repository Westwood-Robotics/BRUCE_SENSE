/*
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "January 31, 2024"
__project__   = "BRUCE"
__version__   = "0.0.4"
__status__    = "Product"
*/

#ifndef ISM330DHCX_H
#define ISM330DHCX_H

#include "Arduino.h"

// I2C Address
#define ISM330DHCX_ADDR 0x6A

// Chip ID
#define ISM330DHCX_ID 0x6B

// Register Address
#define REG_INT1_CTRL  0x0D
#define REG_INT2_CTRL  0x0E
#define REG_WHO_AM_I   0x0F
#define REG_CTRL1_XL   0x10
#define REG_CTRL2_G    0x11
#define REG_CTRL3_C    0x12
#define REG_CTRL4_C    0x13
#define REG_CTRL6_C    0x15
#define REG_CTRL8_XL   0x17
#define REG_CTRL10_C   0x19
#define REG_OUTX_L_G   0x22
#define REG_TIMESTAMP0 0x40

// Constant
#define DEGREE_TO_RADIAN     (0.0174533f)
#define GRAVITY_ACCELERATION (9.80665f)

// Accelerometer Data Range
typedef enum {
  ISM330DHCX_ACCEL_RANGE_2_G,  // milli_g actually
  ISM330DHCX_ACCEL_RANGE_4_G,
  ISM330DHCX_ACCEL_RANGE_8_G,
  ISM330DHCX_ACCEL_RANGE_16_G
} ism330dhcx_accel_range;

// Gyroscope Data Range
typedef enum {
  ISM330DHCX_GYRO_RANGE_125_DPS,  //milli-degree/s actually
  ISM330DHCX_GYRO_RANGE_250_DPS,
  ISM330DHCX_GYRO_RANGE_500_DPS,
  ISM330DHCX_GYRO_RANGE_1000_DPS,
  ISM330DHCX_GYRO_RANGE_2000_DPS,
  ISM330DHCX_GYRO_RANGE_4000_DPS
} ism330dhcx_gyro_range;

#endif
