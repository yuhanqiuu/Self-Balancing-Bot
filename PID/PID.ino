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
#include <movement.h>
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
// Global Variables
char userInput;
float old_theta_n = 0;

float Kp = 0;          // (P)roportional Tuning Parameter
float Ki = 0;          // (I)ntegral Tuning Parameter        
float Kd = 0;          // (D)erivative Tuning Parameter   
    
float iTerm = 0;       // Used to accumulate error (integral)
float lastTime = 0;    // Records the time the function was last called
float oldValue = 0;    // The last sensor value

float theta_n = 0;     // current angle inputs???
float pidOutput = 0;   // PID output
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------

// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // initialize PID controller
PID myPID(&theta_n, &pidOutput, &Setpoint, Kp, Ki, Kd, DIRECT); // initialize PID controller

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
  Input = 0; 
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);

  // Motor setup should be done in movement.h
}
//-------------------------------------------------------------------------


void loop(){
    theta_n = getAngle(old_theta_n); // angles 
    old_theta_n = theta_n;
    

    // Map angle (0° = 0% speed, MAX_ANGLE° = 100% speed)
    float rpm_left = abs(theta_n) * 420 / 90;
    float rpm_right = abs(theta_n) * 437 / 90;

    myPID.Compute(); // 
    Serial.print(theta_n);
    Serial.print("\t");
    Serial.println(pidOutput);
    // if (theta_n > 0) {
    // forward(rpm_to_pwm_left(rpm_left), rpm_to_pwm_right(rpm_right));
    // } else if (theta_n < 0) {
    // backward(rpm_to_pwm_left(rpm_left), rpm_to_pwm_right(rpm_right));
    // } else {
    // forward(0, 0);
    // }

}