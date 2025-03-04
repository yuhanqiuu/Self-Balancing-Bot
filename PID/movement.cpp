// pwm.cpp
#include "movement.h"
#include <math.h>
#include "Arduino_BMI270_BMM150.h"

//-------------------------------------------------------------------------

// Convert RPM to PWM for left and right motors
float rpm_to_pwm_left(float rpm) {
    return 7.58 * exp(7.89E-3 * rpm); // return pwm
}

float rpm_to_pwm_right(float rpm) {
    return 8.33 * exp(7.55E-3 * rpm); // return pwm
}

//-------------------------------------------------------------------------

// Initialize motor pins
void setupMotors() {
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
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

// Left forward, right backward
void lfw_rbw(int pwm1, int pwm2) { 
    analogWrite(AIN1, pwm1);  
    digitalWrite(AIN2, LOW);
    analogWrite(BIN2, pwm2);
    digitalWrite(BIN1, LOW);
}

// Right forward, left backward
void rfw_lbw(int pwm1, int pwm2) { 
    analogWrite(AIN2, pwm1); 
    digitalWrite(AIN1, LOW);
    analogWrite(BIN2, pwm2);
    digitalWrite(BIN1, LOW);
}

//-------------------------------------------------------------------------

void goForward() {
    forward(rpm_to_pwm_left(420), rpm_to_pwm_right(437));
}

void goBackward() {
    backward(rpm_to_pwm_left(420), rpm_to_pwm_right(437));
}

void goRight() {
    lfw_rbw(rpm_to_pwm_left(420), rpm_to_pwm_right(437));
}

void goLeft() {
    rfw_lbw(rpm_to_pwm_left(420), rpm_to_pwm_right(437));
}

//-------------------------------------------------------------------------

double getAngle(double old_theta_n)
{
    float k = 0.15; // weighting factor
    float x, y, z;
    float theta_an, theta_gn = 0;
    float theta_n;
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

    theta_n = k * ((float)old_theta_n + theta_gn) + (1 - k) * theta_an;
    return (double)theta_n;
}

//-------------------------------------------------------------------------
