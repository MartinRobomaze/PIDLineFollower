/**
  This is the main source code for the MazeCar line following robot.
  This entire project is licensed under GNU GPL v3.0.
  Creators: Martinlinux1, Kiriwer
  Version: 0.1a
*/

#include "MazeCar.h"
#include "IOHandle.h"
#include "Motors.h"
#include "LightSensors.h"

// Motor and light sensor pins.
int motorsPins[4] = {5, 6, 9, 10};
int lightSensorsPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

int numberLightSensors = 8;

// Kp, Kd constants - you hzve to experiment with them to have good results.
int Kp = 1;
int Kd = 1;
// Last error.
int lastError = 0;

// Calibration values for light sensors - zou have to set them depending on the light conditions
int blackSensorValue = 0;
int whiteSensorValue = 0;

// Base speed for motors.
int baseSpeed = 150;

// Function declaration.
void calculatePID(int error, int Kp, int Kd, int *speedA, int *speedB);
void readLightSensors(int *sensorsValueArr);
int getError(int *sensorsReadValue);

void setup() {
  // Start serial communication.
  Serial.begin(9600);

  // Setup motors.
  setupMotors(motorPins);
  // Setup light sensors.
  setupSensors(lightSensorsPins);
}

void loop() {
  // Light sensors readings array.
  int lightSensorsReading[numberLightSensors];

  // Read light sensors.
  readLightSensors(lightSensorsReading);

  // Get error based on the light sensors reading.
  int error = getError(lightSensorsReading);

  int speedA = 0;
  int speedB = 0;

  // Calculate PID value based on error and Kp and Kd constants.
  calculatePID(error, Kp, Kd, &speedA, &speedB);

  // Movve motors with speeds returned by PID algorhitm.
  moveTank(speedA, speedB);
}

void calculatePID(int error, int Kp, int Kd, int *speedA, int *speedB) {
  // Proportional.
  int P = error;
  // Derivative.
  int D = error - lastError;
  // PD value.
  int PID = P * Kp + D * Kd;

  // Calculate speedA and speedB values based on PD value.
  *speedA = baseSpeed - PID;
  *speedB = baseSpeed + PID;
}

void readLightSensors(int *sensorsValueArr) {
  // Sensor readings array.
  int sensorValues[numberLightSensors];

  for (int i = 0; i < numberLightSensors; i++) {
    // If light sensor is on black, write 1.
    if (readLightSensor(i) >= blackSensorValue) {
      sensorValues[i] = 1;
    }

    // If light sensor is whitw, write 0.
    else if (readLightSensor(i) <= whiteSensorValue) {
      sensorValues[i] = 0;
    }
  }

  sensorsValueArr = sensorValues;
}

int getError(int *sensorsReadValue) {
  // Set error to -4.
  int error = -4;

  for (int i = 0; i < numberLightSensors; i++) {
    // If sensor is black, increment error.
    if (sensorsReadValue[i] == 1) {
      error++;
    }
  }

  // Return error.
  return error;
}
