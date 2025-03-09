/*
 * bno055.h
 *
 *  Created on: Feb 13, 2025
 *      Author: sagar
 */

/* BNO055.h */
#ifndef BNO055_H
#define BNO055_H

#include "main.h"

// BNO055 I2C address
#define BNO055_I2C_ADDR 0x28<<1

// Register addresses
#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_ACCEL_DATA_X_LSB 0x08
#define BNO055_GYRO_DATA_X_LSB 0x14
#define BNO055_MAG_DATA_X_LSB 0x0E
#define BNO055_EULER_H_LSB 0x1A
#define BNO055_QUATERNION_DATA_W_LSB 0x20
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_PWR_MODE_ADDR 0x3E
#define BNO055_SYS_TRIGGER_ADDR 0x3F

// Operation modes
#define OPERATION_MODE_CONFIG 0x00
#define OPERATION_MODE_NDOF 0x0C

typedef struct {
    // Accelerometer data (m/s^2)
    float accel_x;
    float accel_y;
    float accel_z;

    // Gyroscope data (degrees/s)
    float gyro_x;
    float gyro_y;
    float gyro_z;

    // Euler angles
    float euler_h;    // Heading
    float euler_r;    // Roll
    float euler_p;    // Pitch

    // Quaternion
    float quat_w;     // Quaternion w
    float quat_x;     // Quaternion x
    float quat_y;     // Quaternion y
    float quat_z;     // Quaternion z
} BNO055_Data_t;

// Function prototypes
HAL_StatusTypeDef BNO055_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BNO055_GetEulerAngles(I2C_HandleTypeDef *hi2c, BNO055_Data_t *data);
HAL_StatusTypeDef BNO055_GetQuaternion(I2C_HandleTypeDef *hi2c, BNO055_Data_t *data);
HAL_StatusTypeDef BNO055_GetAccelerometer(I2C_HandleTypeDef *hi2c, BNO055_Data_t *data);
HAL_StatusTypeDef BNO055_GetGyroscope(I2C_HandleTypeDef *hi2c, BNO055_Data_t *data);
HAL_StatusTypeDef BNO055_GetAllData(I2C_HandleTypeDef *hi2c, BNO055_Data_t *data);

#endif
