/*
 * sensor_fusion.c
 *
 *  Created on: Feb 21, 2025
 *      Author: sagar
 */

#include "sensor_fusion.h"
#include "bno055.h"

// Static variables
static RobotState currentState = {0};
static EncoderCounts lastCounts = {0};

//Yaw
float yaw = 0;

//test variables
float test_x = 0;
float test_y = 0;

// Transformation matrix for 3-wheel omni configuration
static const float K_MATRIX[3][3] = {
    {-0.866f,  0.5f,   ROBOT_RADIUS}, // Wheel 1 (120 degrees)
    { 0.866f,  0.5f,   ROBOT_RADIUS}, // Wheel 2 (240 degrees)
    { 0.0f,   -1.0f,   ROBOT_RADIUS}  // Wheel 3 (0 degrees)
};

// Kalman filter parameters
static float Q_pos = 0.01f;    // Process noise for position
static float Q_vel = 0.1f;     // Process noise for velocity
static float R_encoder = 0.1f; // Encoder measurement noise
static float R_imu = 0.05f;    // IMU measurement noise

void SensorFusion_Init(void) {
    if(BNO055_Init() != HAL_OK) {
        Error_Handler();
    }
    SensorFusion_Reset();
}

void SensorFusion_Reset(void) {
    currentState.x = 0.0f;
    currentState.y = 0.0f;
    currentState.theta = 0.0f;
    currentState.vx = 0.0f;
    currentState.vy = 0.0f;
    currentState.omega = 0.0f;
    lastCounts.encoder1 = 0;
    lastCounts.encoder2 = 0;
    lastCounts.encoder3 = 0;
}

void SensorFusion_Update(int32_t e1, int32_t e2, int32_t e3) {
    // Get sensor data

	BNO055_Data_t imuData = BNO055_GetQuaternion();

	yaw = (atan2(2.0 * (imuData.quat_w * imuData.quat_z + imuData.quat_x * imuData.quat_y), 1.0 - 2.0 * (imuData.quat_y * imuData.quat_y + imuData.quat_z * imuData.quat_z))) * (180.0 / PI);
	currentState.theta = yaw;
    // Calculate encoder deltas
    float deltaCounts[3] = {
        (float)(e1 - lastCounts.encoder1),
        (float)(e2 - lastCounts.encoder2),
        (float)(e3 - lastCounts.encoder3)
	};



	float testVel[3];
	for(int i = 0; i < 3; i++) {
		testVel[i] = (deltaCounts[i]* COUNTS_TO_RAD * WHEEL_RADIUS) / DT;
	}
	float wheelVel[3];
	for(int i = 0; i < 3; i++) {
		wheelVel[i] = deltaCounts[i] / DT;
	}

	// Calculate robot velocities from encoders
	float vel_x_enc = 0.0f;
	float vel_y_enc = 0.0f;
	float omega_enc = 0.0f;

	for(int i = 0; i < 3; i++) {
		vel_x_enc += K_MATRIX[i][0] * wheelVel[i] / 3.0f;
		vel_y_enc += K_MATRIX[i][1] * wheelVel[i] / 3.0f;
		omega_enc += K_MATRIX[i][2] * wheelVel[i] / (3.0f * ROBOT_RADIUS);
	}

	float tvel_x_enc = 0.0f;
	float tvel_y_enc = 0.0f;
	float tomega_enc = 0.0f;

	for(int i = 0; i < 3; i++) {
		tvel_x_enc += K_MATRIX[i][0] * wheelVel[i] / 3.0f;
		tvel_y_enc += K_MATRIX[i][1] * wheelVel[i] / 3.0f;
		tomega_enc += K_MATRIX[i][2] * wheelVel[i] / (3.0f * ROBOT_RADIUS);
	}

	// Fuse encoder and IMU data using complementary filter
	float alpha = 0.85f; // Weight for encoder data

	// Update velocities
	currentState.vx = vel_x_enc;
	currentState.vy = vel_y_enc;

	currentState.omega = omega_enc;




	// Update heading using IMU (more accurate than encoder integration)


	// Update position
	float cosTheta = cosf(currentState.theta);
	float sinTheta = sinf(currentState.theta);

	// Transform velocities to global frame and integrate
	currentState.x += (currentState.vx * cosTheta - currentState.vy * sinTheta) * DT;
	currentState.y += (currentState.vx * sinTheta + currentState.vy * cosTheta) * DT;

	test_x += (tvel_x_enc * cosTheta - tvel_y_enc * sinTheta) * DT;
	test_y += (tvel_x_enc * sinTheta + tvel_y_enc * cosTheta) * DT;

//	// Normalize theta to -π to π
//	while(currentState.theta > M_PI) currentState.theta -= 2.0f * M_PI;
//	while(currentState.theta < -M_PI) currentState.theta += 2.0f * M_PI;

	// Save current counts for next update
	lastCounts.encoder1 = e1;
	lastCounts.encoder2 = e2;
	lastCounts.encoder3 = e3;
}

RobotState SensorFusion_GetState(void) {
    return currentState;
}

void SensorFusion_SetState(RobotState newState) {
    currentState = newState;
}












