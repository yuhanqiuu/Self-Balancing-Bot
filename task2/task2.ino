//Task 2 – Computing angles with accelerometer readings


/*
  Arduino BMI270 - Simple Accelerometer

  This example reads the acceleration values from the BMI270
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense Rev2

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include "Arduino_BMI270_BMM150.h"
#include <math.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");
}

void loop() {
  float x, y, z;
  float theta;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    theta = atan(x/y)*180/M_PI;

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    // Serial.println(z);
      
    Serial.println(theta);

  }
  delay(500);  // Reduce excessive polling

}
