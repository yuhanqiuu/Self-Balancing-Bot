#define AIN1 2
#define AIN2 3
#define BIN2 4
#define BIN1 5

#define MAX_ANGLE 90

#define MAX_PWM 255

//--------------------------------------
// Angle Calculate Variables
#include "Arduino_BMI270_BMM150.h"
#include <math.h>

float x, y, z;
float theta_gn = 0; // set the inital angle to 0. (initial condition)
float theta_an;
float theta_n = 0;

float k = 0.15; // weighting factor
char userInput;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  // Angle Calculation Module
  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(x, y, z);
    theta_an = atan(y / z) * 180 / M_PI; // might need to change the axis later
  }

  if (IMU.gyroscopeAvailable())
  {
    IMU.readGyroscope(x, y, z);
    theta_gn += x * (1 / IMU.gyroscopeSampleRate());
  }

  theta_n = k * (theta_n + theta_gn) + (1 - k) * theta_an;

  // Map angle (0° = 0% speed, MAX_ANGLE° = 100% speed)
  int pwm = theta_n * 255 / 90;
  // map(constrain(theta_n, 0, MAX_ANGLE), 0, MAX_ANGLE, 0, MAX_PWM);

  //--------------------------------------
}
