// pwm.cpp
#include "movement.h"
#include <math.h>
#include "Arduino_BMI270_BMM150.h"

unsigned long ct = 0;

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

//right foward, left backward
void rfw_lbw(int pwm1, int pwm2){ 
    analogWrite(AIN2, pwm1); 
    digitalWrite(AIN1, LOW);
  
    analogWrite(BIN1, pwm2);
    digitalWrite(BIN2, LOW);
  }

  // slow decay
void backward_slow(int pwm1, int pwm2){
    analogWrite(AIN2, pwm1);   
    digitalWrite(AIN1, HIGH); 
  
    analogWrite(BIN2, pwm2); 
    digitalWrite(BIN1, HIGH); 
  }
  
  void forward_slow(int pwm1, int pwm2) {  
      digitalWrite(AIN2, HIGH);   
      analogWrite(AIN1, pwm1);
  
      digitalWrite(BIN2, HIGH); 
      analogWrite(BIN1, pwm2);  
  }
  
  //left forward, right backward
  // A left, B right
  void lfw_rbw_slow(int pwm1, int pwm2){ 
  
    // Right motor forward
    analogWrite(AIN2, pwm1);  
    digitalWrite(AIN1, HIGH);
    
    analogWrite(BIN1, pwm2);
    digitalWrite(BIN2, HIGH);
  
  }
  
  //right foward, left backward
  void rfw_lbw_slow(int pwm1, int pwm2){ 
    analogWrite(AIN1, pwm1); 
    digitalWrite(AIN2, HIGH);
  
    analogWrite(BIN2, pwm2);
    digitalWrite(BIN1, HIGH);
  }
  
//-------------------------------------------------------------------------
// full rpm. 
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

float getAngle(float theta_n)
{
    float k = 0.95; // weighting factor
    float x, y, z;
    float theta_an, theta_gn = 0;

    float dt = (float) (micros() - ct) / 1000000;  // gets time for ∆t
    ct = micros();  // sets new current time

    if (IMU.gyroscopeAvailable()) {
        // reads gyroscope value
        IMU.readGyroscope(x, y, z);
        
        // computes theta using integration
        theta_gn = (theta_n + x * dt);
        
    }

    if (IMU.accelerationAvailable()) {
        // reads acceleration
        IMU.readAcceleration(x, y, z);
        
        // computes theta values
        theta_an = -degrees(atan(y / z)); 
    }

    theta_n = k * theta_gn + (1 - k) * theta_an;
    return theta_n;
}

//-------------------------------------------------------------------------

