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

float x, y, z;

int plusThreshold = 30, minusThreshold = -30;

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
    float theta;
    float theta0 = 0; // reference angle

    if (IMU.gyroscopeAvailable())
    {
    IMU.readGyroscope(x, y, z);
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
    
    theta = theta0 + z*IMU.gyroscopeSampleRate();

       
    }
}