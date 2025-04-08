// pwm.cpp
#include "movement.h"
#include <math.h>
#include "Arduino_BMI270_BMM150.h"

// float theta_n = 0;
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
    pinMode(RIGHT1, OUTPUT);
    pinMode(RIGHT2, OUTPUT);
    pinMode(LEFT1, OUTPUT);
    pinMode(LEFT2, OUTPUT);
}

// Forward fast decay
void forward(int pwm1, int pwm2) {
    analogWrite(LEFT1, pwm1);   
    digitalWrite(LEFT2, LOW);  
     
    analogWrite(RIGHT1, pwm2); 
    digitalWrite(RIGHT2, LOW); 
}

// Reverse fast decay
void backward(int pwm1, int pwm2) {  
    digitalWrite(LEFT1, LOW);   
    analogWrite(LEFT2, pwm1);   
    digitalWrite(RIGHT1, LOW); 
    analogWrite(RIGHT2, pwm2);  
}

// Left forward, right backward
void lfw_rbw(int pwm1, int pwm2) { 
    analogWrite(LEFT1, pwm1);  
    digitalWrite(LEFT2, LOW);
    analogWrite(RIGHT2, pwm2);
    digitalWrite(RIGHT1, LOW);
}

//right foward, left backward
void rfw_lbw(int pwm1, int pwm2){ 
    analogWrite(LEFT2, pwm1); 
    digitalWrite(LEFT1, LOW);
  
    analogWrite(RIGHT1, pwm2);
    digitalWrite(RIGHT2, LOW);
  }

  // slow decay
void backward_slow(int pwm1, int pwm2){
    pwm1 = map(abs(pwm1),0,255,250,0);   
    pwm2 = map(abs(pwm2),0,255,250,0); 
    analogWrite(LEFT2, pwm1);   
    digitalWrite(LEFT1, HIGH); 
  
    analogWrite(RIGHT2, pwm2); 
    digitalWrite(RIGHT1, HIGH); 
  }
  
  void forward_slow(int pwm1, int pwm2) {  
    pwm1 = map(abs(pwm1),0,255,250,0);   
    pwm2 = map(abs(pwm2),0,255,250,0); 
      digitalWrite(LEFT2, HIGH);   
      analogWrite(LEFT1, pwm1);
  
      digitalWrite(RIGHT2, HIGH); 
      analogWrite(RIGHT1, pwm2);  
  }

  void forward_slow_v2(int pwm1, int pwm2) {  
    pwm1 = map(abs(pwm1),0,255,250,0);   
    pwm2 = map(abs(pwm2),0,255,250,0); 
      digitalWrite(LEFT1, HIGH);   
      analogWrite(LEFT2, pwm1);
  
      digitalWrite(RIGHT1, HIGH); 
      analogWrite(RIGHT2, pwm2);  
  }


  void backward_slow_v2(int pwm1, int pwm2) {

    pwm1 = map(abs(pwm1),0,255,250,0);   
    pwm2 = map(abs(pwm2),0,255,250,0); 
    analogWrite(LEFT2, pwm1);   
    digitalWrite(LEFT1, HIGH); 

    analogWrite(RIGHT1, pwm2); 
    digitalWrite(RIGHT2, HIGH); 
  }
  
  //left forward, right backward
  // A left, B right
  void lfw_rbw_slow(int pwm1, int pwm2){ 
  
    // Right motor forward
    analogWrite(LEFT2, pwm1);  
    digitalWrite(LEFT1, HIGH);
    
    analogWrite(RIGHT1, pwm2);
    digitalWrite(RIGHT2, HIGH);
  
   
  }
  
  //right foward, left backward
  void rfw_lbw_slow(int pwm1, int pwm2){ 
    analogWrite(LEFT1, pwm1); 
    digitalWrite(LEFT2, HIGH);
  
    analogWrite(RIGHT2, pwm2);
    digitalWrite(RIGHT1, HIGH);
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

void driveMotors(int leftPWM, int rightPWM)
{
  
  // Case 1: both forward
  if (leftPWM > 0 && rightPWM > 0)
  {
    forward_slow(leftPWM, rightPWM);
  }
  // Case 2: both backward
  else if (leftPWM < 0 && rightPWM < 0)
  {
    backward_slow(leftPWM, rightPWM);
  }
  // // Case 3: turning in place (left forward, right backward)
  // else if (leftPWM > 0 && rightPWM < 0)
  // {
  //   lfw_rbw(abs(leftPWM), abs(rightPWM));
  // }
  // // Case 4: turning in place (right forward, left backward)
  // else if (leftPWM < 0 && rightPWM > 0)
  // {
  //   rfw_lbw(abs(leftPWM), abs(rightPWM));
  // }
  // else {
  //   forward(0, 0); // or brake if you want
  //   return;
  // }

}
