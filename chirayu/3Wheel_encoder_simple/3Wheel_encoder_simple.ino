#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Tracking variables
float totalDistanceX = 0;
float totalDistanceY = 0;

// Velocity tracking
float velocityX = 0;
float velocityY = 0;

// Acceleration tracking
float accelX = 0;
float accelY = 0;
unsigned long lastUpdateTime = 0;

// Calibration offset
imu::Vector<3> linearAccelOffset;
// Global position and velocity variables
double dx = 0.0;  // Position in mm
double dy = 0.0;  // Position in mm
double vx = 0.0;  // Velocity in mm/s
double vy = 0.0;  // Velocity in mm/s

// Encoder pins
const int ENCODER1_A = 3;
const int ENCODER1_B = 4;
const int ENCODER2_A = 18;
const int ENCODER2_B = 7;
const int ENCODER3_A = 2;
const int ENCODER3_B = 24;

// Robot physical parameters
const float WHEEL_DIAMETER_MM = 150.0;  // Wheel diameter in mm
const float WHEEL_RADIUS_MM = WHEEL_DIAMETER_MM / 2.0;
const int ENCODER_RESOLUTION = 200;  // Ticks per revolution
const float MM_PER_TICK = (PI * WHEEL_DIAMETER_MM) / ENCODER_RESOLUTION;
const float ROBOT_RADIUS_MM = 310.0;  // Distance from center to wheels in mm

// Timing variables
unsigned long lastTime = 0;
int lastTickF = 0;
int lastTickR = 0;
int lastTickL = 0;

// Encoder tracking
volatile long encoder1Ticks = 0;
volatile long encoder2Ticks = 0;
volatile long encoder3Ticks = 0;
volatile bool encoder1Direction = true;
volatile bool encoder2Direction = true;
volatile bool encoder3Direction = true;

// Function to calculate wheel velocity in mm/s
float calculateWheelVelocity(long currentTicks, long lastTicks, float timeChange) {
    float deltaTicks = currentTicks - lastTicks;
    return (deltaTicks * MM_PER_TICK) / timeChange;
}

void computeRobotMotion() {
    unsigned long currentTime = millis();
    float timeChange = (currentTime - lastTime) / 1000.0;  // Convert to seconds
    
    // Only update if enough time has passed (avoid division by very small numbers)
    if (timeChange >= 0.01) {  // 10ms minimum interval
        // Calculate wheel velocities in mm/s
        float Va = calculateWheelVelocity(encoder1Ticks, lastTickF, timeChange);
        float Vb = calculateWheelVelocity(encoder2Ticks, lastTickR, timeChange);
        float Vc = calculateWheelVelocity(encoder3Ticks, lastTickL, timeChange);

        // Calculate robot velocities using inverse kinematics
        // For a three-wheel configuration at 0°, 120°, and 240°
        vx = Va - 0.866*Vb - 0.5*Vc;  // mm/s
        vy =  0.0*Va - 0.866*Vb + 0.866*Vc;  // mm/s
        // vy=0.5*Vb+Vc*0.866;
        // vx=Va+Vb*0.866+Vc*0.5;

        // Update position by integrating velocity
        dx += vx * timeChange;  // mm
        dy += vy * timeChange;  // mm

        // Store current values for next iteration
        lastTickF = encoder1Ticks;
        lastTickR = encoder2Ticks;
        lastTickL = encoder3Ticks;
        lastTime = currentTime;

        // Print results

        Serial.print("Wheel Velocities (mm/s) - A: ");
        Serial.print(Va, 2);
        Serial.print(" B: ");
        Serial.print(Vb, 2);
        Serial.print(" C: ");
        Serial.println(Vc, 2);
        Serial.println();
    }
}

void setup() {
    Serial.begin(115200);  // Higher baud rate for better data transmission
    
    // Configure encoder pins
    pinMode(ENCODER1_A, INPUT_PULLUP);
    pinMode(ENCODER1_B, INPUT_PULLUP);
    pinMode(ENCODER2_A, INPUT_PULLUP);
    pinMode(ENCODER2_B, INPUT_PULLUP);
    pinMode(ENCODER3_A, INPUT_PULLUP);
    pinMode(ENCODER3_B, INPUT_PULLUP);
    
    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder1Interrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder2Interrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER3_A), encoder3Interrupt, RISING);

  // Perform initial calibration
  delay(1000);
  // Initialize time
  lastUpdateTime = micros();
  lastTime = millis()/1000.0;
}

void loop() {
    computeRobotMotion();
    
        Serial.println("Robot Status:");
        Serial.print("Position (mm) - X: ");
        Serial.print(dx, 2);
        Serial.print(" Y: ");
        Serial.println(dy, 2);
        
        Serial.print("Velocity (mm/s) - X: ");
        Serial.print(vx, 2);
        Serial.print(" Y: ");
        Serial.println(vy, 2);
        Serial.print("Encoder Ticks - 1: ");
    Serial.print(encoder1Ticks);
    Serial.print(" 2: ");
    Serial.print(encoder2Ticks);
    Serial.print(" 3: ");
    Serial.println(encoder3Ticks);
        
    delay(100);  // Small delay for stability
}

// Interrupt handlers for encoders
void encoder1Interrupt() {
    encoder1Direction = (digitalRead(ENCODER1_A) == digitalRead(ENCODER1_B));
    encoder1Ticks += encoder1Direction ? 1 : -1;
}

void encoder2Interrupt() {
    encoder2Direction = (digitalRead(ENCODER2_A) == digitalRead(ENCODER2_B));
    encoder2Ticks += encoder2Direction ? 1 : -1;
}

void encoder3Interrupt() {
    encoder3Direction = (digitalRead(ENCODER3_A) == digitalRead(ENCODER3_B));
    encoder3Ticks += encoder3Direction ? 1 : -1;
}