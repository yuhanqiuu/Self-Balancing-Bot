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

char userInput;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  ;

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Serial.print("Accelerometer sample rate = ");
  // Serial.print(IMU.accelerationSampleRate());
  // Serial.println(" Hz");
  // Serial.println();
  // Serial.println("Acceleration in G's");
  // Serial.println("X\tY\tZ");
}

void loop() {
  float x, y, z;
  float theta;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    theta = atan(y/z))*180/M_PI; // might need to change the axis later

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(z);
    Serial.print('\t');
    Serial.println(theta);
      

    // Check for serial input
    if (Serial.available() > 0)
    {
      userInput = Serial.read();  // Read user input

      if (userInput == 'g')  // If Python requests data
      {
        Serial.println(theta);
      }
    }

  }
  //delay(500);  // Reduce excessive polling

}
