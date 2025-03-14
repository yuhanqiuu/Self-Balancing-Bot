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

float Kp = 7;          // (P)roportional Tuning Parameter 12-14?
float Ki = 0;          // (I)ntegral Tuning Parameter        
float Kd = 1;          // (D)erivative Tuning Parameter   1049
    
float iTerm = 0;       // Used to accumulate error (integral)
float lastTime = 0;    // Records the time the function was last called
float last_error = 0;    // The last sensor value
float dTerm = 0;    
float dT = 0;   


float theta_n = 0;     // current angle inputs???
float pidOutput = 0;   // PID output
float target = 0;
//-------------------------------------------------------------------------
float maxPID = 255; 

// for gyroscope
float x, y, z;

// Variables for Time Keeper function:
#define LOOP_TIME 10          // Time in ms (10ms = 100Hz)
unsigned long timerValue = 0;

// PID myPID(&theta_n, &pidOutput, &Setpoint, Kp, Ki, Kd, DIRECT); // initialize PID controller

int PID(float target, float current){
  float thisTime = millis();
	dT = (thisTime - lastTime)/1000;
	lastTime = thisTime;

  float error = target - current;

	iTerm += error * dT; 

  dTerm = (error  - last_error) / dT;
  last_error = error;
  if(dTerm < 0.5 && dTerm > -0.5){
    dTerm = 0;
  }

	// Multiply each term by its constant, and add it all up
	float result = (error * Kp) + (iTerm * Ki) + (dTerm * Kd);

  // using gyroscope data directly for Kd term
  // float result = (error * Kp) + (iTerm * Ki) + (x * Kd);

  
	// Limit PID value to maximum values
	if (result > maxPID) result = maxPID;
	else if (result < -maxPID) result = -maxPID;

	return result;
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
  // else if (input == "reset") {
  //   // integral = 0;
  // }


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
 
  timerValue = millis();
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

    // if (IMU.gyroscopeAvailable())
    // {
    //     IMU.readGyroscope(x, y, z);
    // }
    
    // Run the PID controller
      float motorOutput = PID(0, theta_n); // targe value = 0, current value = theta_n

      Serial.print(theta_n);
      Serial.print("\t");
      Serial.print(dTerm);
      Serial.print("\t");
      Serial.print(dT,5);
      Serial.print("\t");
      Serial.print(Kp);
      Serial.print("\t");
      Serial.print(Ki);
      Serial.print("\t");
      Serial.print(Kd);
      Serial.print("\t");
      Serial.println(motorOutput);

    if(motorOutput > 0){
      backward_slow(abs(motorOutput), abs(motorOutput));
      //foward(abs(motorOutput), abs(motorOutput));


      //forward(rpm_to_pwm_left(abs(motorOutput)/255*420), rpm_to_pwm_right(abs(motorOutput)/255*437));
    } else if (motorOutput < 0){
      // backward(abs(motorOutput), abs(motorOutput));
      forward_slow(abs(motorOutput), abs(motorOutput));
      //backward(rpm_to_pwm_left(abs(motorOutput)/255*420), rpm_to_pwm_right(abs(motorOutput)/255*437));

    }
      else forward(0, 0);
  // }
}