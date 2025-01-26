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
char userInput;

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
  // Read sensor values
  float temperature = HS300x.readTemperature();
  float humidity = HS300x.readHumidity();

  // Update only when there is a significant change OR at least once per loop
  if (abs(old_temp - temperature) >= 0.5 )
  {
    // print each of the sensor values

  // Serial.print("Temperature = ");
  // Serial.print(temperature);
  // Serial.println(" °C");

   // Check for serial input
  if (Serial.available() > 0)
  {
    userInput = Serial.read();  // Read user input

    if (userInput == 'g')  // If Python requests data
    {
      Serial.println(temperature); // Send temperature to Python
    }
  }

  }

  // // Check for serial input
  // if (Serial.available() > 0)
  // {
  //   userInput = Serial.read();  // Read user input

  //   if (userInput == 'g')  // If Python requests data
  //   {
  //     Serial.println(temperature); // Send temperature to Python
  //   }
  // }

  delay(500);  // Reduce excessive polling
}