#define LEFT1 2 
#define LEFT2 3
#define RIGHT2 4
#define RIGHT1 5

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
}


float rpm_to_pwm_left(float rpm) {
  return 7.58 * exp(7.89E-3 * rpm);  // return pwm
}

float rpm_to_pwm_right(float rpm) {
  return 8.33 * exp(7.55E-3 * rpm);  // return pwm
}

// Forward fast decay
void forward(int pwm1, int pwm2) {
    analogWrite(AIN1, pwm1);   
    digitalWrite(AIN2, LOW);   
    analogWrite(BIN1, pwm2); 
    digitalWrite(BIN2, LOW); 
}

// Reverse fast decay
void backward(int pwm1, int pwm2) {  
    digitalWrite(AIN1, LOW);   
    analogWrite(AIN2, pwm1);   
    digitalWrite(BIN1, LOW); 
    analogWrite(BIN2, pwm2);  
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
  Serial.println(theta_n);

  // Map angle (0° = 0% speed, MAX_ANGLE° = 100% speed)
  float rpm_left = abs(theta_n) * 420 / 90;
  float rpm_right = abs(theta_n) * 437 / 90;

  if (theta_n > 0) {
    forward(rpm_to_pwm_left(rpm_left), rpm_to_pwm_right(rpm_right));
  } else if (theta_n < 0) {
    backward(rpm_to_pwm_left(rpm_left), rpm_to_pwm_right(rpm_right));
  } else {
    forward(0, 0);
  }

  //--------------------------------------
}
