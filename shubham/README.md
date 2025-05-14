# Robocon 2025 - Robot Code

This repository contains the codebase for the Robocon 2025 robot, designed to operate using an STM32 Nucleo F446RE microcontroller. The robot is equipped with advanced functionalities and is controlled via a PS5 controller. Below is an overview of the features and components implemented in this project.

## Features

### 1. **Three-Wheel Kinematics**
- Implements equations for three-wheel omnidirectional movement.
- Provides precise control over robot motion using encoder feedback.

### 2. **Encoders**
- Tracks wheel rotations to calculate speed and position.
- Ensures accurate movement and navigation.

### 3. **IMU (Inertial Measurement Unit)**
- Provides orientation and angular velocity data.
- Enhances stability and control during operation.

### 4. **Stepper Motor Control**
- Controls stepper motors for precise linear or rotational movements.
- Used for specific mechanisms requiring high accuracy.

### 5. **Pneumatics**
- Operates pneumatic actuators for gripping or other mechanical actions.
- Controlled via GPIO pins on the STM32.

### 6. **E-Bike Motor Integration**
- Drives high-power e-bike motors for propulsion.
- Supports variable speed control for smooth operation.

### 7. **PS5 Controller Integration**
- The robot is controlled wirelessly using a PS5 controller.
- Inputs are processed and sent to the STM32 via an ESP32 module.

### 8. **I2C Communication**
- PS5 controller data is transmitted from the ESP32 to the STM32 using I2C protocol.
- Ensures reliable and efficient communication between the devices.

## Getting Started

### Prerequisites
- STM32 Nucleo F446RE development board.
- ESP32 module for wireless communication.
- PS5 controller.
- Required sensors and actuators (encoders, IMU, stepper motors, pneumatic components, e-bike motor).

### Setup
1. Clone this repository to your local machine.
2. Connect the hardware components as per the circuit diagram (to be added).
3. Flash the STM32 with the provided firmware.
4. Pair the PS5 controller with the ESP32 module.

### Usage
- Power on the robot and ensure all connections are secure.
- Use the PS5 controller to control the robot's movement and functionalities.
- Monitor feedback from encoders and IMU for debugging and performance tuning.

## Contributing
Contributions are welcome! Please fork the repository and submit a pull request with your changes.

## Acknowledgments
- Robocon team members for their dedication and hard work.
- Open-source libraries and tools used in this project.
