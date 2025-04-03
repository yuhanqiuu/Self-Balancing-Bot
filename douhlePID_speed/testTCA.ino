// #include <Wire.h>
// #include <Arduino.h>
// #include "TCA9548A.h"
// #include <AS5600.h>
// #include "movement.h"

// // Multiplexer and AS5600 objects
// TCA9548A I2CMux;
// AS5600 encoderLeft;
// AS5600 encoderRight;

// // Filtering parameters
// float filteredRPMLeft = 0;
// float filteredRPMRight = 0;
// const float alpha = 0.2;          // Smoothing factor
// const float noiseThreshold = 1.0; // RPM threshold for noise suppression

// // Previous angle and timestamp per encoder
// float prevAngleLeft = 0;
// float prevAngleRight = 0;
// unsigned long prevTimeLeft = 0;
// unsigned long prevTimeRight = 0;

// void setup()
// {
//     Serial.begin(115200);
//     Wire.begin();
//     I2CMux.begin(Wire);
//     I2CMux.closeAll();

//     // Prime encoders
//     I2CMux.openChannel(0);
//     delay(10);
//     prevAngleLeft = encoderLeft.readAngle() * (360.0 / 4096.0);
//     prevTimeLeft = micros();
//     I2CMux.closeChannel(0);
//     I2CMux.openChannel(1);
//     delay(10);
//     prevAngleRight = encoderRight.readAngle() * (360.0 / 4096.0);
//     prevTimeRight = micros();
//     I2CMux.closeChannel(1);

//     Serial.println("AS5600 RPM monitoring via TCA9548A started.");
// }

// void loop()
// {
//     forward(100, 100); // Command motors (movement.h)

//     float rpmLeft = 0;
//     float rpmRight = 0;

//     // --- Left Encoder (Channel 0) ---
//     I2CMux.openChannel(0);
//     delayMicroseconds(25);
//     rpmLeft = readRPM(prevAngleLeft, prevTimeLeft, encoderLeft);
//     I2CMux.closeChannel(0);

//     delay(1); // brief pause between switching

//     // --- Right Encoder (Channel 1) ---
//     I2CMux.openChannel(1);
//     delayMicroseconds(25);
//     rpmRight = -readRPM(prevAngleRight, prevTimeRight, encoderRight); // Negate if needed for direction
//     I2CMux.closeChannel(1);

//     // --- Filter ---
//     filteredRPMLeft = alpha * rpmLeft + (1 - alpha) * filteredRPMLeft;
//     filteredRPMRight = alpha * rpmRight + (1 - alpha) * filteredRPMRight;

//     if (abs(filteredRPMLeft) < noiseThreshold)
//         filteredRPMLeft = 0;
//     if (abs(filteredRPMRight) < noiseThreshold)
//         filteredRPMRight = 0;

//     // --- Print results ---
//     Serial.print("Raw RPM L: ");
//     Serial.print(rpmLeft, 2);
//     Serial.print(" | Filtered RPM L: ");
//     Serial.print(filteredRPMLeft, 2);
//     Serial.print(" || Raw RPM R: ");
//     Serial.print(rpmRight, 2);
//     Serial.print(" | Filtered RPM R: ");
//     Serial.println(filteredRPMRight, 2);

//     delay(20); // Limit refresh rate
// }


