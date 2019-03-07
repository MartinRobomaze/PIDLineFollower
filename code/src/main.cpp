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
float Kd = 1;
// Last error.
int lastError = 0;

// Calibration values for light sensors - zou have to set them depending on the light conditions
int blackSensorValue = 500;
int whiteSensorValue = 100;

// Base speed for motors.
int baseSpeed = 150;
bool running = false;

int previous = 1;
long time = 0;
int debounce = 200;

// Function declaration.
void calculatePID(int error, int Kp, int Kd, int *speedA, int *speedB);
int readLightSensorDigital(int sensor);
int getError(int *sensorsReadValue);
void getBluetoothData(float *Kp, float *Kd);
void interrupt();
void save();

void setup() {
  Kp = EEPROM.read(0);
  Kd = EEPROM.read(1);
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
      getBluetoothData(&Kp, &Kd);

      // Debug messages.
      Serial.print(Kp);
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
  // Derivative.
  int D = error - lastError;
  // PD value.
  int PID = P * Kp + D * Kd;

  // Calculate speedA and speedB values based on PD value.
  *speedA = baseSpeed - PID;
  *speedB = baseSpeed + PID;
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

void getBluetoothData(float *Kp, float *Kd) {
  String data = "";
  while (BT.available()) {
    char ReadChar = (char)BT.read();

    if (ReadChar == ')') {
      break;
    } else {
      data += ReadChar;
    }
  }
  int index0 = data.indexOf(':');
  int index1 = data.indexOf(':', index0 + 1);
  int index2 = data.indexOf(':', index1 + 1);
  int kp = data.substring(0, index0).toInt();
  int ki = data.substring(index0 + 1, index1).toInt();
  int kd = data.substring(index1 + 1, index2).toInt();
  *Kp = kp;
  *Kd = kd;
}

void save() {
  EEPROM.write(0, Kp);
  EEPROM.write(1, Kd);
}
