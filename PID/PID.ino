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
#include <PID_v1.h>
#include "movement.h"

//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
// Global Variables
char userInput;
double old_theta_n = 0;

double Kp = 4.25;          // (P)roportional Tuning Parameter 
double Ki = 0;          // (I)ntegral Tuning Parameter        
double Kd = 0;          // (D)erivative Tuning Parameter   
    
double iTerm = 0;       // Used to accumulate error (integral)
double lastTime = 0;    // Records the time the function was last called
double oldValue = 0;    // The last sensor value

double theta_n = 0;     // current angle inputs???
double pidOutput = 0;   // PID output
double Setpoint = 0;
//-------------------------------------------------------------------------
double maxPID = 255; 

// PID myPID(&theta_n, &pidOutput, &Setpoint, Kp, Ki, Kd, DIRECT); // initialize PID controller

int PID(double target, double current){
  int output;
  double kp, ki, kd, error, last_error, iTerm;
  float thisTime = millis();
	float dT = thisTime - lastTime;
	lastTime = thisTime;

  error = target - current;

	iTerm += error * dT; 

  double dTerm = (oldValue - current) / dT;
  oldValue = current;

	// Multiply each term by its constant, and add it all up
	double result = (error * Kp) + (iTerm * Ki) + (dTerm * Kd);

	// Limit PID value to maximum values
	if (result > maxPID) result = maxPID;
	else if (result < -maxPID) result = -maxPID;

	return result
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

  // PID setup

  target = 0;
 
  last_time = millis();

  // Motor setup should be done in movement.h
}
//-------------------------------------------------------------------------


void loop(){
    theta_n = getAngle(old_theta_n); // angles 
    old_theta_n = theta_n;
    
    // Run the PID controller
      double motorOutput = pid(targetValue, currentValue);

    if(motorOutput > 0){
        forward(abs(motorOutput), abs(motorOutput));
    } else if (motorOutput < 0){
      backward(abs(motorOutput), abs(motorOutput));
    }
      else {forward(0, 0);}
    
}