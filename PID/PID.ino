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

double Kp = 5;          // (P)roportional Tuning Parameter 
double Ki = 0;          // (I)ntegral Tuning Parameter        
double Kd = 0;          // (D)erivative Tuning Parameter   
    
double iTerm = 0;       // Used to accumulate error (integral)
double lastTime = 0;    // Records the time the function was last called
double oldValue = 0;    // The last sensor value

double theta_n = 0;     // current angle inputs???
double pidOutput = 0;   // PID output
double Setpoint = 0;
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

  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);

  // Motor setup should be done in movement.h
}
//-------------------------------------------------------------------------


void loop(){
    theta_n = getAngle(old_theta_n); // angles 
    old_theta_n = theta_n;
    

    // Map angle (0° = 0% speed, MAX_ANGLE° = 100% speed)
    double rpm_left = abs(theta_n) * 420 / 90;
    double rpm_right = abs(theta_n) * 437 / 90;
 

    myPID.Compute(); // 
  

    // if leaning forward, move forward
    if (pidOutput> 0){
      // leaning forward, go forward

      // FOR SOME REASON ITS BACKWARDS FOR THE SLOW DECAY?? IDK WHYYY, MIGHT NEED TO DEBUG?
      backward_slow(abs(pidOutput)/255*420, abs(pidOutput)/255*437);

      // ***************TEST OUT THE FOLLOWING LINE AS WELL***************************************************
      // backward_slow(rpm_to_pwm_left((pidOutput)/255*420), rpm_to_pwm_right((pidOutput)/255*437));


    } else if (pidOutput < 0){
      // leaning backward, go backward
      forward_slow(abs(pidOutput)/255*420, abs(pidOutput)/255*437);
      // forward_slow(rpm_to_pwm_left((pidOutput)/255*420), rpm_to_pwm_right((pidOutput)/255*437));

    }else{
      forward_slow(0, 0);
    }

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