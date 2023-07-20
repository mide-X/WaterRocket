#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

// MPU6050 object
MPU6050 mpu;

// Servo objects
Servo servo1;      // Thrust vectoring servo
Servo servo2;      // Thrust vectoring servo
Servo parachuteServo;  // Parachute deployment servo

// Servo motor pins
const int servo1Pin = 3;
const int servo2Pin = 5;
const int parachuteServoPin = 6;  // Change to the actual pin for the parachute servo

// Servo motor angles for thrust vectoring control
const int minAngle = 0;   // Minimum servo angle
const int maxAngle = 180; // Maximum servo angle

// Setpoint (desired orientation)
float setpoint = 0.0;

// MPU6050 calibration values
int16_t accelX_offset, accelY_offset, accelZ_offset;
int16_t gyroX_offset, gyroY_offset, gyroZ_offset;

void setup() {
  // Initialize serial communication
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();

  // Calibrate MPU6050
  calibrateMPU6050();

  // Attach servo motors to pins
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  parachuteServo.attach(parachuteServoPin);

  // Move servos to initial position
  servo1.write(90);
  servo2.write(90);
  parachuteServo.write(0);  // Assuming the parachute servo needs to be at 0 degrees initially

  // LED indication to tell us the system is ready to go!
  digitalWrite(LED_BUILTIN, HIGH);
  delay(4000);
  digitalWrite(LED_BUILTIN, LOW);
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

  // Map current orientation to servo angles
  int angle1 = map(currentOrientation, -90, 90, minAngle, maxAngle);
  int angle2 = map(currentOrientation, -90, 90, minAngle, maxAngle);

  // Set servo angles for thrust vectoring control
  servo1.write(90 + angle1);
  servo2.write(90 + angle2);

  // Print sensor data and servo angles
  Serial.print("Orientation: ");
  Serial.print(currentOrientation);
  Serial.print("\tServo angles: ");
  Serial.print(90 + angle1);
  Serial.print("\t");
  Serial.println(90 + angle2);

  // Check for freefall condition and deploy parachute if detected
  if (abs(accelZ_g) < 0.1) { // Adjust the threshold (0.1) as needed
    deployParachute();
  }

  delay(100);  // Delay between updates
}

void calibrateMPU6050() {
  // Calibration code remains the same as before
  // ...
}

void deployParachute() {
  // Assuming the parachute servo needs to be at 180 degrees for deployment
  parachuteServo.write(180);
  delay(1000);  // Wait for the parachute to deploy
  parachuteServo.write(0); // Reset the parachute servo position
}
