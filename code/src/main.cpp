/**
  This is the main source code for the MazeCar line following robot.
  This entire project is licensed under GNU GPL v3.0.
  Creators: Martinlinux1, Kiriwer
  Version: 0.1a
*/

#include "MazeCar.h"
#include "Arduino.h"
#include "IOHandle.h"
#include "Motors.h"
#include "LightSensors.h"


int motorsPins[4] = {3, 5, 6, 9};
int lightSensorsPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

int Kp = 1;
int Kd = 1;
int lastError = 0;

int blackSensorValue = 0;
int whiteSensorValue = 0;

int baseSpeed = 150;

void calculatePID(int error, int Kp, int Kd, int *speedA, int *speedB);
void readLightSensors(int *sensorsValueArr);
int getError(int *sensorsReadValue);

void setup() {
  Serial.begin(9600);

  setupMotors(motorPins);
  setupSensors(lightSensorsPins);
}

void loop() {
  int lightSensorsReading[8];

  readLightSensors(lightSensorsReading);

  int error = getError(lightSensorsReading);

  int speedA = 0;
  int speedB = 0;

  calculatePID(error, Kp, Kd, &speedA, &speedB);

  moveTank(speedA, speedB);
}

void calculatePID(int error, int Kp, int Kd, int *speedA, int *speedB) {
  int P = error;
  int D = error - lastError;
  int PID = P * Kp + D * Kd;

  *speedA = baseSpeed - PID;
  *speedB = baseSpeed + PID;
}

void readLightSensors(int *sensorsValueArr) {
  int sensorValues[8];
  for (int i = 0; i < 8; i++) {
    if (readLightSensor(i) <= blackSensorValue) {
      sensorValues[i] = 1;
    }

    else if (readLightSensor(i) >= whiteSensorValue) {
      sensorValues[i] = 0;
    }
  }

  sensorsValueArr = sensorValues;
}

int getError(int *sensorsReadValue) {
  int error = -4;

  for (int i = 0; i < 8; i++) {
    if (sensorsReadValue[i] == 1) {
      error++;
    }
  }

  return error;
}
