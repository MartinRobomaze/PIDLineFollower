#include "MazeCar.h"
#include "Arduino.h"
#include "IOHandle.h"
#include "Motors.h"
#include "LightSensors.h"

#define CAL_NUM_READ 4096

int motorsPins[4] = {3, 5, 6, 9};
int lightSensorsPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

int Kp = 1;
int Kd = 1;
int lastError = 0;

int minSensorValue = 0;
int maxSensorValue = 0;

int baseSpeed = 150;

void calibrate(int *minValue, int *maxValue);
void calculatePID(int error, int Kp, int Kd, int *speedA, int *speedB);
void readLightSensors(int *sensorsValueArr);
int getError(int *sensorsReadValue);

void setup() {
  Serial.begin(9600);

  setupMotors(motorPins);
  setupSensors(lightSensorsPins);

  calibrate(&minSensorValue, &maxSensorValue);
}

void loop() {

}

void calibrate(int *minValue, int *maxValue) {
  int sensorMinValue = 1023;
  int sensorMaxValue = 0;

  for (int i = 0; i < CAL_NUM_READ; i++) {
    for (int j = 0; j < 8; j++) {
      int sensorValue = readLightSensor(j);

      if (sensorValue < sensorMinValue) {
        sensorMinValue = sensorValue;
      }

      else if (sensorValue > sensorMaxValue) {
        sensorMaxValue = sensorValue;
      }
    }

    delay(10);
  }
}

void calculatePID(int error, int Kp, int Kd, int *speedA, int *speedB) {
  int P = error;
  int D = error - lastError;
  int PID = P * Kp + D * Kd;

  *speedA = baseSpeed - PID;
  *speedB = baseSpeed + PID;
}

void readLightSensors(int *sensorsValueArr) {
  for (int i = 0; i < 8; i++) {

  }
}
