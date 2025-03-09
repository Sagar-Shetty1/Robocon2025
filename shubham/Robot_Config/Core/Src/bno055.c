/*
 * bno055.c
 *
 *  Created on: Feb 13, 2025
 *      Author: sagar
 */


/* BNO055.c */
#include "BNO055.h"

static HAL_StatusTypeDef BNO055_Write8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return HAL_I2C_Master_Transmit(hi2c, BNO055_I2C_ADDR, buf, 2, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef BNO055_Read(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(hi2c, BNO055_I2C_ADDR, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    return HAL_I2C_Master_Receive(hi2c, BNO055_I2C_ADDR, data, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BNO055_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t chip_id;

    // Read chip ID
    status = BNO055_Read(hi2c, BNO055_CHIP_ID_ADDR, &chip_id, 1);
    if (status != HAL_OK) return status;

    // Verify chip ID
    if (chip_id != 0xA0) return HAL_ERROR;

    // Reset device
    status = BNO055_Write8(hi2c, BNO055_SYS_TRIGGER_ADDR, 0x20);
    if (status != HAL_OK) return status;

    // Wait for reset
    HAL_Delay(650);

    // Set power mode to normal
    status = BNO055_Write8(hi2c, BNO055_PWR_MODE_ADDR, 0x00);
    if (status != HAL_OK) return status;

    // Set operation mode to NDOF (fusion mode)
    status = BNO055_Write8(hi2c, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    if (status != HAL_OK) return status;

    // Wait for sensor fusion to stabilize
    HAL_Delay(100);

    return HAL_OK;
}

HAL_StatusTypeDef BNO055_GetAccelerometer(I2C_HandleTypeDef *hi2c, BNO055_Data_t *data)
{
    uint8_t buf[6];
    HAL_StatusTypeDef status;

    status = BNO055_Read(hi2c, BNO055_ACCEL_DATA_X_LSB, buf, 6);
    if (status != HAL_OK) return status;

    // Convert to m/s^2 (BNO055 accelerometer LSB = 1/100 m/s^2)
    data->accel_x = (int16_t)((buf[1] << 8) | buf[0]) / 100.0f;
    data->accel_y = (int16_t)((buf[3] << 8) | buf[2]) / 100.0f;
    data->accel_z = (int16_t)((buf[5] << 8) | buf[4]) / 100.0f;

    return HAL_OK;
}

HAL_StatusTypeDef BNO055_GetGyroscope(I2C_HandleTypeDef *hi2c, BNO055_Data_t *data)
{
    uint8_t buf[6];
    HAL_StatusTypeDef status;

    status = BNO055_Read(hi2c, BNO055_GYRO_DATA_X_LSB, buf, 6);
    if (status != HAL_OK) return status;

    // Convert to degrees/s (BNO055 gyroscope LSB = 1/16 degree/s)
    data->gyro_x = (int16_t)((buf[1] << 8) | buf[0]) / 16.0f;
    data->gyro_y = (int16_t)((buf[3] << 8) | buf[2]) / 16.0f;
    data->gyro_z = (int16_t)((buf[5] << 8) | buf[4]) / 16.0f;

    return HAL_OK;
}

HAL_StatusTypeDef BNO055_GetEulerAngles(I2C_HandleTypeDef *hi2c, BNO055_Data_t *data)
{
    uint8_t buf[6];
    HAL_StatusTypeDef status;

    status = BNO055_Read(hi2c, BNO055_EULER_H_LSB, buf, 6);
    if (status != HAL_OK) return status;

    // Convert to degrees (BNO055 euler angles are in degrees * 16)
    data->euler_h = (int16_t)((buf[1] << 8) | buf[0]) / 16.0f;
    data->euler_r = (int16_t)((buf[3] << 8) | buf[2]) / 16.0f;
    data->euler_p = (int16_t)((buf[5] << 8) | buf[4]) / 16.0f;

    return HAL_OK;
}

HAL_StatusTypeDef BNO055_GetQuaternion(I2C_HandleTypeDef *hi2c, BNO055_Data_t *data)
{
    uint8_t buf[8];
    HAL_StatusTypeDef status;

    status = BNO055_Read(hi2c, BNO055_QUATERNION_DATA_W_LSB, buf, 8);
    if (status != HAL_OK) return status;

    // Convert to float (BNO055 quaternion values are scaled by 2^14)
    data->quat_w = (int16_t)((buf[1] << 8) | buf[0]) / 16384.0f;
    data->quat_x = (int16_t)((buf[3] << 8) | buf[2]) / 16384.0f;
    data->quat_y = (int16_t)((buf[5] << 8) | buf[4]) / 16384.0f;
    data->quat_z = (int16_t)((buf[7] << 8) | buf[6]) / 16384.0f;

    return HAL_OK;
}

HAL_StatusTypeDef BNO055_GetAllData(I2C_HandleTypeDef *hi2c, BNO055_Data_t *data)
{
    HAL_StatusTypeDef status;

    status = BNO055_GetAccelerometer(hi2c, data);
    if (status != HAL_OK) return status;

    status = BNO055_GetGyroscope(hi2c, data);
    if (status != HAL_OK) return status;

    status = BNO055_GetEulerAngles(hi2c, data);
    if (status != HAL_OK) return status;

    status = BNO055_GetQuaternion(hi2c, data);
    if (status != HAL_OK) return status;

    return HAL_OK;
}
