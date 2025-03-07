/*
 * sensor_fusion.h
 *
 *  Created on: Feb 21, 2025
 *      Author: sagar
 */

#ifndef INC_SENSOR_FUSION_H_
#define INC_SENSOR_FUSION_H_

#include "main.h"
#include <math.h>

// Structure to hold robot position
typedef struct {
    float x;          // X position in meters
    float y;          // Y position in meters
    float theta;      // Heading in radians
    float vx;         // X velocity in m/s
    float vy;         // Y velocity in m/s
    float omega;      // Angular velocity in rad/s
} RobotState;

// Structure to hold encoder counts
typedef struct {
    int32_t encoder1;
    int32_t encoder2;
    int32_t encoder3;
} EncoderCounts;

// Structure to hold IMU data
typedef struct {
    float heading;     // Euler angle heading in radians
    float gyroZ;      // Angular velocity around Z axis in rad/s
    float accelX;     // Linear acceleration in X axis in m/s²
    float accelY;     // Linear acceleration in Y axis in m/s²
} IMUData;

// Robot physical parameters
#define WHEEL_RADIUS          0.05f   // Wheel radius in meters
#define ROBOT_RADIUS         0.15f   // Distance from center to wheel in meters
#define ENCODER_CPR         1024     // Encoder counts per revolution
#define GEAR_RATIO          20.0f    // Motor gear ratio
#define UPDATE_RATE         100.0f   // Update rate in Hz
#define DT                  (1.0f/UPDATE_RATE)

// Function prototypes
void SensorFusion_Init(void);
void SensorFusion_Reset(void);
void SensorFusion_Update(void);
RobotState SensorFusion_GetState(void);
void SensorFusion_SetState(RobotState newState);
HAL_StatusTypeDef BNO055_Init(void);
IMUData BNO055_GetData(void);


#endif /* INC_SENSOR_FUSION_H_ */
