#include "MazeCar.h"
#include "Arduino.h"
#include "IOHandle.h"
#include "Motors.h"
#include "LightSensors.h"

#define CAL_NUM_READ 4096

int motorPins[4] = {3, 5, 6, 9};
int lightSensorsPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

int Kp = 1;
int Kd = 1;
int lastError = 0;

int minSensorValue = 0;
int maxSensorValue = 0;

int baseSpeed = 150;



void calibrate(int *minValue, int *maxValue);
void calculatePID(int error, int Kp, int Kd, int *speedA, int *speedB);

void setup() {
  Serial.begin(9600);

  calibrate(&minSensorValue, &maxSensorValue);
}

void loop() {

}

void calibrate(int *minValue, int *maxValue) {
  int sensorMinValue = 1023;
  int sensorMaxValue = 0;

  for (int i = 0; i < CAL_NUM_READ; i++) {
    for (int j = 0; j < arrLen(lightSensorsPins); j++) {
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

  speedA = baseSpeed - PID;
  speedB = baseSpeed + PID;
}
