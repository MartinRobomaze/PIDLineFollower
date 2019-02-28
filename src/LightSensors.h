#include "MazeCar.h"
#include "Arduino.h"

int *sensorPins;

void setupSensors(int *pins) {
  sensorPins = pins;
}

int readLightSensor(int sensor) {
  return analogRead(sensor);
}
