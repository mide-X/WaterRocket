/* 
By Olufemi Balogun, Testimony Aina and Jehovah-Nissi Daniel

For tuning, 

Start with the Proportional Gain (Kp):
  * Set the Integral Gain (Ki) and Derivative Gain (Kd) to zero.
  * Increase Kp until the system starts to oscillate or exhibit instability.
  * Note the Kp value where instability begins.

Adjust the Integral Gain (Ki):
  * Increase Ki until any steady-state error in the system is minimized or eliminated.
  * Be cautious not to set Ki too high, as it can lead to overshooting and instability.

Fine-tune the Derivative Gain (Kd):
  * Increase Kd slightly to improve the system's response time and reduce overshooting.
  * If the system becomes too sensitive or exhibits high-frequency oscillations, decrease Kd.
  * Adjust Kd to achieve a good balance between response time and stability.

Repeat the steps:

Iterate through steps 1 to 3, making small adjustments to the gains.
Observe the system's behavior and assess its response to different inputs.
Fine-tune the gains until you achieve the desired performance characteristics.
Test the system:

Evaluate the system's response to various disturbances and setpoint changes.
Verify if the system maintains stable and accurate thrust vectoring control.
Make further adjustments as necessary based on the test results.

 */


#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

// MPU6050 object
MPU6050 mpu;

// Servo objects
Servo servo1;
Servo servo2;

// MPU6050 calibration values
int16_t accelX_offset, accelY_offset, accelZ_offset;
int16_t gyroX_offset, gyroY_offset, gyroZ_offset;

// Servo motor pins
const int servo1Pin = 3;
const int servo2Pin = 5;

// Servo motor angles for thrust vectoring control
const int minAngle = -45;  // Minimum servo angle
const int maxAngle = 45;   // Maximum servo angle

// PID parameters
const float Kp = 2.0;   // Proportional gain
const float Ki = 0.2;   // Integral gain
const float Kd = 1.0;   // Derivative gain

// Setpoint (desired orientation)
float setpoint = 0.0;

// PID variables
float previousError = 0.0;
float integral = 0.0;

// Water rocket parameters
const float launchAngle = 60.0;  // Launch angle (degrees)

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();

  // Calibrate MPU6050
  calibrateMPU6050();

  // Attach servo motors to pins
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);

  // Move servos to initial position
  servo1.write(90);
  servo2.write(90);
}

void loop() {
  // Read MPU6050 data
  int16_t accelX, accelY, accelZ;
  int16_t gyroX, gyroY, gyroZ;

  mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);

  // Convert raw sensor data to meaningful values
  float accelX_g = (accelX - accelX_offset) / 16384.0;
  float accelY_g = (accelY - accelY_offset) / 16384.0;
  float accelZ_g = (accelZ - accelZ_offset) / 16384.0;

  // Calculate current orientation
  float currentOrientation = atan2(accelY_g, accelZ_g) * 180.0 / PI;

  // Calculate error
  float error = setpoint - currentOrientation;

  // Update integral term
  integral += error;

  // Apply PID control
  float output = Kp * error + Ki * integral + Kd * (error - previousError);

  // Constrain output within limits
  output = constrain(output, minAngle, maxAngle);

  // Set servo angles for thrust vectoring control
  servo1.write(90 + output);
  servo2.write(90 - output);

  // Print sensor data and servo angles
  Serial.print("Orientation: ");
  Serial.print(currentOrientation);
  Serial.print("\tServo angles: ");
  Serial.print(90 + output);
  Serial.print("\t");
  Serial.println(90 - output);

  // Update previous error
  previousError = error;

  delay(100);  // Delay between updates
}

void calibrateMPU6050() {
  // Initialize offsets to zero
  accelX_offset = 0;
  accelY_offset = 0;
  accelZ_offset = 0;
  gyroX_offset = 0;
  gyroY_offset = 0;
  gyroZ_offset = 0;

  const int numSamples = 1000;  // Number of samples for calibration

  // Accumulate samples
  for (int i = 0; i < numSamples; i++) {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;

    mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);

    accelX_offset += accelX;
    accelY_offset += accelY;
    accelZ_offset += accelZ;
    gyroX_offset += gyroX;
    gyroY_offset += gyroY;
    gyroZ_offset += gyroZ;

    delay(10);  // Delay between samples
  }

  // Calculate average offsets
  accelX_offset /= numSamples;
  accelY_offset /= numSamples;
  accelZ_offset /= numSamples;
  gyroX_offset /= numSamples;
  gyroY_offset /= numSamples;
  gyroZ_offset /= numSamples;
}
