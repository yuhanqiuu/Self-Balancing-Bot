/*
  HS300x - Read Sensors

  This example reads data from the on-board HS300x sensor of the
  Nano 33 BLE Sense Rev2 and prints the temperature and humidity sensor
  values to the Serial Monitor once a second.

  The circuit:
  - Arduino Nano 33 BLE Sense Rev2

  This example code is in the public domain.
*/

#include <Arduino_HS300x.h>

float old_temp = 0;

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;

  if (!HS300x.begin())
  {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1)
      ;
  }
}

void loop()
{
  // read all the sensor values
  float temperature = HS300x.readTemperature();
  float humidity = HS300x.readHumidity();
  if (abs(old_temp - temperature) >= 0.5 || abs(old_hum - humidity) >= 1)
  {
    // print each of the sensor values
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" °C");

    if(Serial.available()> 0){ 
    
      userInput = Serial.read();               // read user input
        
        if(userInput == 'g'){                  // if we get expected value 

            Serial.println(temperature);            
        } // if user input 'g' 
    } // Serial.available   
    
    // wait 1 second to print again
    delay(1000);
  }
}