#include "MazeCar.h"
#include "Motors.h"

int motorsPins[4] = {5, 10, 9, 6};

void setup() {
  Serial.begin(9600);
  setupMotors(motorsPins);
}

void loop() {
  moveTank(150, 120);
  // for (size_t i = 0; i < 255; i++) {
  //   forward(i);
  //   delay(2);
  // }
  //
  // for (size_t i = 255; i < 0; i++) {
  //   forward(i);
  //   delay(2);
  // }
}
