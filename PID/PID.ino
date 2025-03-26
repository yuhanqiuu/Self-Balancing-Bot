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

float Kp = 13;          // (P)roportional Tuning Parameter prob around 17
float Ki = 0.1;          // (I)ntegral Tuning Parameter 10?       
float Kd = 1.3;          // (D)erivative Tuning Parameter   around 1 
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
#define LOOP_TIME 5          // Time in ms (10ms = 100Hz)
unsigned long timerValue = 0;

// PID myPID(&theta_n, &pidOutput, &Setpoint, Kp, Ki, Kd, DIRECT); // initialize PID controller

float PID(float setpoint, float currentValue){
  float output = 0;
  //   currentTime = micros();
  //   dt = (currentTime - lastTime) / 1000000.0;  // Time difference in seconds
  // lastTime = currentTime;
  dt = (float) (micros() - currentTime) / 1000000.0;  // gets time for ∆t
  currentTime = micros();  // sets new current time

  float error = setpoint - currentValue;

  if (dt <= 0) dt = 1e-3; // Prevent division by zero

  proportional = Kp * error;

  integral += dt * error;
  integral = constrain(integral, -30, 30);  // Example limit


  // Derivative term (rate of change of error)
  derivative = (error - previousError) / dt;
  if (theta_n - old_theta_n > 0.1 || theta_n - old_theta_n < -0.1) 
        derivative = -Kd * (theta_n - old_theta_n)/dt; // computes the derivative error
    else{ 
        derivative = 0; // filters out noise
    }
    
    // output = K_mast * (proportional + Ki * integral + Kd * derivative );    // computes sum of error
    // output = constrain(output, -1000, 1000);  // limits output of PID to limits of PWM
    output = constrain(proportional + Ki * integral + derivative, -230.00, 230.0);
    
    previousError = error; // update the previous error

    // if(output>255){
    //   output = 255;
    // } else if(output <-255){
    //   output = -255;
    // }

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

  // if (micros() - timerValue > LOOP_TIME) {
	timerValue = micros();

    keyboard_test();

    theta_n = getAngle(old_theta_n); // angles 
    old_theta_n = theta_n;
    int leftpwm;
    int rightpwm;

    
    // Run the PID controller
      float result = PID(0, theta_n); // targe value = 0, current value = theta_n, pid output
      // float motorOutput = constrain(map(abs(result),0,1000,50,255),50,255); // pwm output
      // result = constrain(result, -500, 500);
      // float motorOutput = map(result, -1000, 1000, -255, 255);

      
      leftpwm = (int) abs(result)*1.7;
      rightpwm = (int) abs(result);

      //given left pwm = 7.58 * exp(7.89E-3 * rpm), we can calculate rpm from pwm
      float leftrpm = log(leftpwm/7.58)/7.89E-3; 
      float rightpwm1 = 8.33 * exp(7.55E-3 * leftrpm);
    if(result < 5){
      forward_slow(rightpwm, leftpwm);
      // forward(leftpwm,rightpwm);

    } else if (result > -5){
      backward_slow(rightpwm, leftpwm);
      // backward(leftpwm,rightpwm);

    }
      else forward(0, 0);


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
      Serial.println(result);
      // Serial.print("\t");
      // Serial.println(motorOutput);

  //  }
}