#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>

#define LEFT1 2  //  AIN1, BIN1 yellow wire
#define LEFT2 3  //  AIN2, BIN2 yellow wire
#define RIGHT2 4 //  AIN2, BIN2 green wire
#define RIGHT1 5 //  AIN1, BIN1 green wire

// Function declarations
float rpm_to_pwm_left(float rpm);
float rpm_to_pwm_right(float rpm);

void setupMotors();
void forward(int pwm1, int pwm2);
void backward(int pwm1, int pwm2);
void lfw_rbw(int pwm1, int pwm2);
void rfw_lbw(int pwm1, int pwm2);

void forward_slow(int pwm1, int pwm2);
void backward_slow(int pwm1, int pwm2);
void lfw_rbw_slow(int pwm1, int pwm2);
void rfw_lbw_slow(int pwm1, int pwm2);


void goForward();
void goBackward();
void goLeft();
void goRight();

float getAngle(float old_theta_n);

#endif
