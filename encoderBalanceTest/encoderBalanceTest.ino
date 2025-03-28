#include <Wire.h>
#include "AS5600.h"
#include "movement.h"
#include <math.h>

String input;
int task = 0;

// PID Tuning Parameters
float Kp_speed = 100;  
float Ki_speed = 0.0;  
float Kd_speed = 0.0; 

// PID Variables

float integral_speed = 0;
float proportional_speed = 0;
float derivative_speed = 0;
float previous_speed_error = 0;

float dt = 0;
unsigned long currentTime = 0;
unsigned long previousTime = 0;

// Encoder Setup
AS5600 encoder;
float rpm = 0;
float previousAngle = 0;
unsigned long lastTime = 0;

// Desired Setpoint
float setpoint_speed = 0;  // We want the wheel to stay stationary

void setup() {
    //Serial.begin(115200);
    Wire.begin();

    if (!encoder.begin()) {
        Serial.println("Failed to initialize AS5600 encoder!");
        while (1);
    }

    lastTime = micros();
}

float speed_PID(float setpoint_speed, float current_speed) {
  float output = 0;
  dt = (float) (micros() - currentTime) / 1000000.0;  // gets time for ∆t
  currentTime = micros();  // sets new current time

  float speed_error = setpoint_speed - current_speed;
  integral_speed += speed_error * dt;
  derivative_speed = (speed_error - previous_speed_error) / dt;
  previous_speed_error = speed_error;
  
  return constrain(Kp_speed * speed_error + Ki_speed * integral_speed + Kd_speed * derivative_speed, -230, 230);
}

void keyboard_test(void)
{

  if (Serial.available() > 0)
  {
    input = Serial.readString();
    input.trim();
    input.toLowerCase();
  }

  if (input == "kp")
  {
    task = 1;
  }
  else if (input == "ki")
  {
    task = 2;
  }
  else if (input == "kd")
  {
    task = 3;
  }
  else if (input == "reset")
  {
    integral_speed = 0;
  }

  switch (task)
  {
  case 1:
    if (input == "0" || input.toFloat() > 0)
      Kp_speed = input.toFloat();
    break;
  case 2:
    if (input == "0" || input.toFloat() > 0)
      Ki_speed = input.toFloat();
    break;
  case 3:
    if (input == "0" || input.toFloat() > 0)
      Kd_speed = input.toFloat();
    break;
  }
}



void loop() { 
    // Read encoder angle
    keyboard_test();
    float currentAngle = encoder.readAngle()* (360.0 / 4096.0); ;
    
    // Calculate RPM
    unsigned long currentTime = micros();
    float dt = (float)(currentTime - previousTime) / 1000000.0;  // Convert to seconds
    previousTime = currentTime;

    float angleChange = currentAngle - previousAngle;
    if (angleChange > 180) angleChange -= 360;  // Handle wrap-around
    if (angleChange < -180) angleChange += 360;

    rpm = (angleChange / 360.0) * 60.0 / dt;  // Convert to RPM
    previousAngle = currentAngle;
    lastTime = currentTime;

    // PID Control
    float output = speed_PID(setpoint_speed, rpm); // targe value = 0, current value = rpm, pid output

    // Set Motor Speed and Direction
    int pwmValue = abs(output);

      if(output > 0){
        forward_slow(pwmValue, pwmValue);
      } 
      else if (output < 0){
        backward_slow(pwmValue, pwmValue);
      }
      else forward(0, 0);

    // Debugging
    // Serial.print(dt,5);
    // Serial.print("\t");
    // Serial.print(proportional_speed);
    // Serial.print("\t");
    // Serial.print(integral_speed);
    // Serial.print("\t");
    // Serial.print(derivative_speed);
    // Serial.print("\t");
    Serial.print(rpm);
    Serial.print("\t");
    Serial.print(angleChange);
    Serial.print("\t");
    Serial.print(output);
    Serial.print("\t");
    Serial.print(Kp_speed);
    Serial.print("\t");
    Serial.print(Ki_speed);
    Serial.print("\t");
    Serial.println(Kd_speed);

    // delay(10); // Small delay for stability
}