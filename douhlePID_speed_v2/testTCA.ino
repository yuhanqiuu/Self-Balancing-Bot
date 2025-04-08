#include <Wire.h>
#include <Arduino.h>
#include "TCA9548A.h"
#include <AS5600.h>
#include "movement.h"

// Multiplexer and AS5600 objects
TCA9548A I2CMux;
AS5600 encoderLeft;
AS5600 encoderRight;

// Filtering parameters
float filteredRPMLeft = 0;
float filteredRPMRight = 0;
const float alpha = 0.2;          // Smoothing factor
const float noiseThreshold = 1.0; // RPM threshold for noise suppression

// Previous angle and timestamp per encoder
float prevAngleLeft = 0;
float prevAngleRight = 0;
unsigned long prevTimeLeft = 0;
unsigned long prevTimeRight = 0;

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    I2CMux.begin(Wire);
    I2CMux.closeAll();

    // Prime encoders
    I2CMux.openChannel(0);
    delay(10);
    prevAngleLeft = encoderLeft.readAngle();
    prevTimeLeft = micros();
    I2CMux.closeChannel(0);
    I2CMux.openChannel(1);
    prevAngleRight = encoderRight.readAngle();
    prevTimeRight = micros();
    I2CMux.closeChannel(1);

    Serial.println("AS5600 RPM monitoring via TCA9548A started.");
}

void loop()
{
    forward(100, 100); // Command motors (movement.h)

    float rpmLeft = 0;
    float rpmRight = 0;

    // --- Left Encoder (Channel 0) ---
    I2CMux.openChannel(0);
    enLeft = encoderLeft.readAngle();
    I2CMux.closeChannel(0);

    // --- Right Encoder (Channel 1) ---
    I2CMux.openChannel(1);
    enRight = -encoderRight.readAngle();
    I2CMux.closeChannel(1);
    if (abs(rpmLeft) < noiseThreshold)
        filteredRPMLeft = 0;
    if (abs(rpmRight) < noiseThreshold)
        filteredRPMRight = 0;

    // --- Print results ---
    Serial.print("Left: ");
    Serial.print(rpmLeft, 2);
    Serial.print("Right: ");
    Serial.println(rpmRight, 2);
}


