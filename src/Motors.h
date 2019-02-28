#include "MazeCar.h"
#include "Arduino.h"

int *motorPins;

void setupMotors(int pins[4]) {
  motorPins = pins;
  for (int i = 0; i < 4; i++) {
    PinMode(pins[i], OUTPUT);
  }
}

void forward(int speed) {
  analogWrite(motorPins[0], speed);
  analogWrite(motorPins[1], 0);
  analogWrite(motorPins[2], speed);
  analogWrite(motorPins[3], 0);
}

void backward(int speed) {
  analogWrite(motorPins[0], 0);
  analogWrite(motorPins[1], speed);
  analogWrite(motorPins[2], 0);
  analogWrite(motorPins[3], speed);
}

void moveTank(int speedA, int speedB) {
  if (speedA < 0 && speedB < 0) {
    analogWrite(motorPins[0], 0);
    analogWrite(motorPins[1], -speedA);
    analogWrite(motorPins[2], 0);
    analogWrite(motorPins[3], speedB);
  }

  else if (speedA < 0) {
    analogWrite(motorPins[0], 0);
    analogWrite(motorPins[1], -speedA);
    analogWrite(motorPins[2], speedB);
    analogWrite(motorPins[3], 0);
  }

  else if (speedB < 0) {
    analogWrite(motorPins[0], speedA);
    analogWrite(motorPins[1], 0);
    analogWrite(motorPins[2], 0);
    analogWrite(motorPins[3], -speedB);
  }

  else {
    analogWrite(motorPins[0], speedA);
    analogWrite(motorPins[1], 0);
    analogWrite(motorPins[2], speedB);
    analogWrite(motorPins[3], 0);
  }
}
