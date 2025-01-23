/*
  Arduino BMI270_BMM150 - Simple Gyroscope

  This example reads the gyroscope values from the BMI270_BMM150
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

float x, y, z;
float theta_gn = 0; // set the inital angle to 0. (initial condition)
float theta_an;
float theta_n = 0;

float k = 0.1; // weighting factor
char userInput;
// unsigned long prevTime = 0;

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ;
    Serial.println("Started");

    if (!IMU.begin())
    {
        Serial.println("Failed to initialize IMU!");
        while (1)
            ;
    }

    //prevTime = millis(); // Initialize timing
}

void loop()
{   
    // unsigned long currentTime = millis();
    // float dt = (currentTime - prevTime) / 1000.0; // Convert to seconds
    // prevTime = currentTime;

    if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    theta_an = atan(y/z)*180/M_PI; // might need to change the axis later
    }
    
    if (IMU.gyroscopeAvailable())
    {
    IMU.readGyroscope(x, y, z);
    theta_gn += x*(1/ IMU.gyroscopeSampleRate());
    }

    theta_n = k *(theta_n+theta_gn) + (1-k) * theta_an;

    //theta_nPrv = theta_n; // update the previous angle

    Serial.print(theta_an);
    Serial.print('\t');
    Serial.print(theta_gn);
    Serial.print('\t');
    Serial.println(theta_n);

    // Check for serial input
    if (Serial.available() > 0)
    {
      userInput = Serial.read();  // Read user input

      if (userInput == 'g')  // If Python requests data
      {
        Serial.print(theta_an);
        Serial.print(",");
        Serial.print(theta_gn);
        Serial.print(",");
        Serial.println(theta_n);
      }
    }

    delay(50);

}