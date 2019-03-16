/**
  This is the main source code for the MazeCar line following robot.
  This entire project is licensed under GNU GPL v3.0.
  Creators: Martinlinux1, Kiriwer
  Version: 0.1a
*/

#include "MazeCar.h"
#include "Motors.h"
#include "LightSensors.h"

// Motor and light sensor pins.
int motorsPins[4] = {5, 10, 9, 6};
int lightSensorsPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

int numberLightSensors = 8;

int BTRx = 12;
int BTTx = 11;

int buttonPin = 2;

SoftwareSerial BT(BTRx, BTTx);

// Kp, Kd constants - you have to experiment with them to have good results.
float Kp = 1;
float Ki = 1;
float Kd = 1;
// Last error.
int lastError = 0;

// Calibration values for light sensors - you have to set them depending on the light conditions
int blackSensorValue = 500;
int whiteSensorValue = 100;

// Base speed for motors.
int baseSpeed = 220;

bool running = false;
int previous = 1;
long time = 0;
int debounce = 200;

// Function declaration.
void calculatePID(int error, int Kp, int Kd, int *speedA, int *speedB);
int readLightSensorDigital(int sensor);
int getError(int *sensorsReadValue);
void getBluetoothData(float *kp, float *ki, float *kd);
void interrupt();
void save();

void setup() {
  // Set interrupt button pin as INPUT_PULLUP.
  pinMode(buttonPin, INPUT_PULLUP);

  // Start serial communication.
  Serial.begin(9600);

  // Start bluetooth communication.
  BT.begin(9600);

  // Setup motors.
  setupMotors(motorsPins);
  // Setup light sensors.
  setupSensors(lightSensorsPins);

  Kp = EEPROM.read(0);
  Ki = EEPROM.read(1);
  Kd = EEPROM.read(2);
  Serial.println(Kp);
  Serial.println(Ki);
  Serial.println(Kd);
}

void loop() {
  // If running = true.
  if (running) {
    // Light sensors readings array.
    int lightSensorsReading[numberLightSensors];

    // Read light sensors.
    for (int i = 0; i < numberLightSensors; i++) {
      lightSensorsReading[i] = readLightSensorDigital(i);
    }

    // Get error based on the light sensors reading.
    int error = getError(lightSensorsReading);
    // Serial.println(error);

    int speedA = 0;
    int speedB = 0;

    // Calculate PID value based on error and Kp and Kd constants.
    calculatePID(error, Kp, Kd, &speedA, &speedB);

    // Move motors with speeds returned by PID algorhitm.
    moveTank(speedA, speedB);

    // If data from bluetooth are available.
    if (BT.available()) {
      // Get Kp and Kd from the app.
      getBluetoothData(&Kp, &Ki, &Kd);

      // Debug messages.
      Serial.print(Kp);
      Serial.print("\t");
      Serial.print(Ki);
      Serial.print("\t");
      Serial.print(Kd);
      Serial.print("\n");
    }
  }

  else {
    forward(0);
  }

  if (digitalRead(buttonPin) == LOW && previous == HIGH && millis() - time > debounce) {
    if (running == HIGH)
      running = LOW;
    else
      running = HIGH;

    time = millis();
  }

  previous = digitalRead(buttonPin);
}

void calculatePID(int error, int Kp, int Kd, int *speedA, int *speedB) {
  // Proportional.
  int P = error;
  int I = I + error;
  // Derivative.
  int D = error - lastError;
  // PD value.
  int PID = P * Kp + I * Ki + D * Kd;

  // Calculate speedA and speedB values based on PD value.
  *speedA = baseSpeed - PID > 255 ? 255 : baseSpeed - PID;
  *speedB = baseSpeed + PID > 255 ? 255 : baseSpeed + PID;
}

int readLightSensorDigital(int sensor) {
  // Sensor readings array.
  int sensorValue;

    // If light sensor is on black, write 1.
  if (readLightSensor(sensor) >= blackSensorValue) {
    sensorValue = 1;
  }

  // If light sensor is whitw, write 0.
  else if (readLightSensor(sensor) <= whiteSensorValue) {
    sensorValue = 0;
  }
  return sensorValue;
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

void getBluetoothData(float *kp, float *ki, float *kd) {
  int  data[10];
  int i = 0;
  while (BT.available()) {
      if (BT.read() == 41) {
        break;
      }
      data[i] = BT.read();
      delay(1);
      i++;
    }

  if (data[0] == 's') {
    save();
    *kp = Kp;
    *ki = Ki;
    *kd = Kd;
  }

  else {
    *kp = data[0];
    *ki = data[1];
    *kd = data[2];
  }
}

void save() {
  EEPROM.write(0, Kp);
  EEPROM.write(1, Ki);
  EEPROM.write(2, Kd);
  Serial.println("data saved");
  Serial.println(EEPROM.read(0));
  Serial.println(EEPROM.read(1));
  Serial.println(EEPROM.read(2));
}
