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
float theta = 0; // set the inital angle to 0. (initial condition)
//unsigned long time; // might need to use the time to calculate the angle later.
char userInput;

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
    Serial.print("Gyroscope sample rate = ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Gyroscope in degrees/second");
}

void loop()
{  
    if (IMU.gyroscopeAvailable())
    {
    IMU.readGyroscope(x, y, z);
    theta = theta + x*(1/IMU.gyroscopeSampleRate());
    
    // Serial.print(x);
    // Serial.print('\t');
    // Serial.print(y);
    // Serial.print('\t');
    // Serial.print(z);    
    // Serial.print('\t');
    // Serial.println(theta);

    
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
}