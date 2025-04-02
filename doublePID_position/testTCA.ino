#include <Wire.h>
#include <Arduino.h>
#include "TCA9548A.h"
#include <AS5600.h>

TCA9548A I2CMux;
AS5600 encoder;

unsigned long lastReadTime = 0;
const unsigned long readInterval = 5; // milliseconds

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    I2CMux.begin(Wire);
    I2CMux.closeAll();

    // Prime angle & time state for both encoders
    I2CMux.openChannel(0);
    delay(2);
    encoder.readAngle(); // initialize internal angle state
    I2CMux.closeChannel(0);

    I2CMux.openChannel(1);
    delay(2);
    encoder.readAngle();
    I2CMux.closeChannel(1);
}

void loop()
{
    if (millis() - lastReadTime >= readInterval)
    {
        lastReadTime = millis();

        float speed0 = 0;
        float speed1 = 0;

        // --- Read encoder 0 ---
        I2CMux.openChannel(0);
        delayMicroseconds(200);                                       // I2C settle
        encoder.readAngle();                                          // Update internal angle
        speed0 = encoder.getAngularSpeed(AS5600_MODE_DEGREES, false); // Use delta
        I2CMux.closeChannel(0);

        // --- Read encoder 1 ---
        I2CMux.openChannel(1);
        delayMicroseconds(200);
        encoder.readAngle();
        speed1 = encoder.getAngularSpeed(AS5600_MODE_DEGREES, false);
        I2CMux.closeChannel(1);

        // --- Print results ---
        Serial.print("Encoder 0 Speed: ");
        Serial.print(speed0, 2);
        Serial.print(" deg/s | Encoder 1 Speed: ");
        Serial.println(speed1, 2);
    }
}
