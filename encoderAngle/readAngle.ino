#include <Wire.h>  //This is for I2C

// I2C pins:
// STM32: SDA: PB7 SCL: PB6
// Arduino: SDA: A4 SCL: A5

//---------------------------------------------------------------------------
// Magnetic sensor things
int magnetStatus = 0;

int lowbyte;
word highbyte;
int rawAngle;
float degAngle;

int quadrantNumber, previousquadrantNumber;
float numberofTurns = 0;
float correctedAngle = 0;
float startAngle = 0;
float totalAngle = 0;
float previoustotalAngle = 0;

float OLEDTimer = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);  // Give serial time to initialize
  Serial.println("Booting...");
  Wire.begin();
  Wire.setClock(800000L);

  // checkMagnetPresence();
  ReadRawAngle();
  startAngle = degAngle;
  OLEDTimer = millis();  // reused as a print timer
}

void loop() {
  ReadRawAngle();
  correctAngle();
  checkQuadrant();
  refreshDisplay();
}

void ReadRawAngle() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0D);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  while (Wire.available() == 0);
  lowbyte = Wire.read();

  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  while (Wire.available() == 0);
  highbyte = Wire.read();

  highbyte = highbyte << 8;
  rawAngle = highbyte | lowbyte;
  degAngle = rawAngle * 0.087890625;
}

void correctAngle() {
  correctedAngle = degAngle - startAngle;
  if (correctedAngle < 0) {
    correctedAngle += 360;
  }
}

void checkQuadrant() {
  if (correctedAngle >= 0 && correctedAngle <= 90) quadrantNumber = 1;
  if (correctedAngle > 90 && correctedAngle <= 180) quadrantNumber = 2;
  if (correctedAngle > 180 && correctedAngle <= 270) quadrantNumber = 3;
  if (correctedAngle > 270 && correctedAngle < 360) quadrantNumber = 4;

  if (quadrantNumber != previousquadrantNumber) {
    if (quadrantNumber == 1 && previousquadrantNumber == 4) numberofTurns++;
    if (quadrantNumber == 4 && previousquadrantNumber == 1) numberofTurns--;
    previousquadrantNumber = quadrantNumber;
  }

  totalAngle = (numberofTurns * 360) + correctedAngle;
}

void refreshDisplay() {
  if (millis() - OLEDTimer > 100) {
    if (totalAngle != previoustotalAngle) {
      Serial.print("Total angle: ");
      Serial.println(totalAngle, 2);
      Serial.print("\t");
      Serial.print("Correct angle: ");
      Serial.println(correctedAngle, 2);
      Serial.print("\n");
      OLEDTimer = millis();
      previoustotalAngle = totalAngle;
    }
  }
}

void checkMagnetPresence() {
  while ((magnetStatus & 32) != 32) {
    magnetStatus = 0;
    Wire.beginTransmission(0x36);
    Wire.write(0x0B);
    Wire.endTransmission();
    Wire.requestFrom(0x36, 1);
    while (Wire.available() == 0);
    magnetStatus = Wire.read();
  }
}
