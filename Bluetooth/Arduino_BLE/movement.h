#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>

// Define motor control pins
#define AIN1 2
#define AIN2 3
#define BIN2 4
#define BIN1 5

// Function declarations
float rpm_to_pwm_left(float rpm);
float rpm_to_pwm_right(float rpm);

void setupMotors();
void forward(int pwm1, int pwm2);
void backward(int pwm1, int pwm2);
void lfw_rbw(int pwm1, int pwm2);
void rfw_lbw(int pwm1, int pwm2);

#endif
