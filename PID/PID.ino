#define LEFT1 2 
#define LEFT2 3
#define RIGHT2 4
#define RIGHT1 5

//-------------------------------------------------------------------------

// Libraries
#include "Arduino_BMI270_BMM150.h"
#include <math.h>
#include <PID_v1.h>
#include "movement.h"

//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
// Global Variables
char userInput;
float old_theta_n = 0;
String input;
int task = 0;

float Kp = 13;          // (P)roportional Tuning Parameter 12-14?
float Ki = 60;          // (I)ntegral Tuning Parameter        
float Kd = 5;          // (D)erivative Tuning Parameter   1049
float K_mast = 1.0;
    
// PID Variables
float previousError = 0;
float integral = 0;
float proportional = 0;
float derivative = 0;
unsigned long lastTime = 0;
float dt = 0;
float currentTime = 0;


float theta_n = 0;     // current angle inputs???
float pidOutput = 0;   // PID output
float setpoint = 0;
//-------------------------------------------------------------------------
float maxPID = 1000; 

// for gyroscope
float x, y, z;

// Variables for Time Keeper function:
#define LOOP_TIME 10          // Time in ms (10ms = 100Hz)
unsigned long timerValue = 0;

// PID myPID(&theta_n, &pidOutput, &Setpoint, Kp, Ki, Kd, DIRECT); // initialize PID controller

float PID(float setpoint, float currentValue){
  float output = 0;
  //   currentTime = micros();
  //   dt = (currentTime - lastTime) / 1000000.0;  // Time difference in seconds
  // lastTime = currentTime;
  dt = (float) (micros() - currentTime) / 1000000;  // gets time for ∆t
  currentTime = micros();  // sets new current time

  float error = setpoint - currentValue;

  proportional = Kp * error;

  integral += Ki * dt * (error + previousError) / 2.0;
  integral = constrain(integral, -100, 100);  // Example limit


  // Derivative term (rate of change of error)
  derivative = (error - previousError) / dt;
  if (theta_n - old_theta_n > 0.1 || theta_n - old_theta_n < -0.1) 
        derivative = -Kd * (theta_n - old_theta_n)/dt; // computes the derivative error
    else{ 
        derivative = 0; // filters out noise
    }
    
    output = K_mast * ( proportional + integral + derivative );    // computes sum of error
    // output = constrain(output, -1000, 1000);  // limits output of PID to limits of PWM
    
    previousError = error; // update the previous error

    if(output>500){
      output = 500;
    } else if(output <-500){
      output = -500;
    }

  return output;
} 

void keyboard_test (void) {
  if (Serial.available() > 0) {
      input = Serial.readString();
      input.trim();
      input.toLowerCase();
  }

  if (input == "kp") {
      task = 1;
  }
  else if (input == "ki") {
      task = 2;
  }
  else if (input == "kd") {
      task = 3;
  }
  else if (input == "reset") {
    integral = 0;
  }


  switch (task) {
      case 1:
          if(input == "0" || input.toFloat() > 0) Kp = input.toFloat();
      break;
      case 2:
          if(input == "0" || input.toFloat() > 0) Ki = input.toFloat();
      break;
      case 3:
          if(input == "0" || input.toFloat() > 0) Kd = input.toFloat();
      break;
  }
}

void setup() {

    // IMU and serial setup
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Started");

    if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1);
    }
 
  Serial.setTimeout(10);
  // Motor setup should be done in movement.h
}
//-------------------------------------------------------------------------


void loop(){

  // if (millis() - timerValue > LOOP_TIME) {
	// 	timerValue = millis();

    keyboard_test();

    theta_n = getAngle(old_theta_n); // angles 
    old_theta_n = theta_n;
    int leftpwm;
    int rightpwm;

    
    // Run the PID controller
      float result = PID(0, theta_n); // targe value = 0, current value = theta_n, pid output
      float motorOutput = constrain(map(abs(result),0,500,35,255),35,255); // pwm output
      // result = constrain(result, -500, 500);
      // float motorOutput = map(result, -500, 500, 0, 255);

      leftpwm = (int) abs(motorOutput*0.9);
      rightpwm =abs(motorOutput);

    if(result > 0){
      forward_slow(leftpwm, rightpwm);

    } else if (result < 0){
      backward_slow(leftpwm,rightpwm);

    }
      else forward_slow(255, 255);


      Serial.print(theta_n);
      Serial.print("\t");
      Serial.print(leftpwm);
      Serial.print("\t");
      Serial.print(rightpwm);
      Serial.print("\t");
      // Serial.print(proportional);
      // Serial.print("\t");
      // Serial.print(derivative);
      // Serial.print("\t");
      Serial.print(integral);
      Serial.print("\t");
      // Serial.print(dt,5);
      // Serial.print("\t");
      Serial.print(Kp);
      Serial.print("\t");
      Serial.print(Ki);
      Serial.print("\t");
      Serial.print(Kd);
      Serial.print("\t");
      Serial.print(result);
      Serial.print("\t");
      Serial.println(motorOutput);

  // }
}