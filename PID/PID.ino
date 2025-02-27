#define LEFT1 2 
#define LEFT2 3
#define RIGHT2 4
#define RIGHT1 5

#define MAX_ANGLE 90

#define MAX_PWM 255
//-------------------------------------------------------------------------
// Libraries
#include "Arduino_BMI270_BMM150.h"
#include <math.h>
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
// Global Variables
char userInput;
float old_theta_n = 0;
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
// PID Controller Function
float pid(float target, float current) {
    
}
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
// Angle calculation function
float getAngle(float old_angle) {
  float k = 0.15;  // weighting factor
  float x, y, z;
  float theta_gn = 0;  // set the inital angle to 0. (initial condition)
  float theta_n = 0;
  float theta_an;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    theta_an = atan(y / z) * 180 / M_PI;
    }

    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(x, y, z);
        theta_gn += x * (1 / IMU.gyroscopeSampleRate());
    }

    theta_n = k * (old_angle + theta_gn) + (1 - k) * theta_an;
    
    return theta_n;
}
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
// rpm to pwm functions
float rpm_to_pwm_left(float rpm) {
    return 7.58 * exp(7.89E-3 * rpm);  // return pwm
}
  
float rpm_to_pwm_right(float rpm) {
    return 8.33 * exp(7.55E-3 * rpm);  // return pwm
}
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
// Movement functions
void forward(int pwm1, int pwm2){
    analogWrite(LEFT1, pwm1);   
    digitalWrite(LEFT2, LOW);   
    analogWrite(RIGHT1, pwm2); 
    digitalWrite(RIGHT2, LOW); 
}

void backward(int pwm1, int pwm2) {  
    digitalWrite(LEFT1, LOW);   
    analogWrite(LEFT2, pwm1);   
    digitalWrite(RIGHT1, LOW); 
    analogWrite(RIGHT2, pwm2);  
}
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
void setup() {
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Started");

    if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}
//-------------------------------------------------------------------------

void loop(){
    float theta_n = getAngle(old_theta_n);

    old_theta_n = theta_n;

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
}