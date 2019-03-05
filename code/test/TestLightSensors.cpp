#include "MazeCar.h"
#include "LightSensors.h"

int lightSensorsPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

void setup() {
  setupSensors(lightSensorsPins);
  Serial.begin(9600);
}

void loop() {
  for (size_t i = 0; i < 8; i++) {
    Serial.print(readLightSensor(i));
    Serial.print("\t");
    delay(10);
  }
  Serial.print("\n");
}
