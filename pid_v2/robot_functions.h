#ifndef ROBOT_FUNCTIONS_H
#define ROBOT_FUNCTIONS_H

#include "Arduino_BMI270_BMM150.h"
#include "Arduino.h"

/////////////////////////////// PWM Config ///////////////////////////////

// Define Arduino pins (corresponding to nRF52840 GPIOs)
#define PWM_PIN_D2  11  // P1.11 (Arduino D2) | Left Motor Black
#define PWM_PIN_D3  12  // P1.12 (Arduino D3) | Left Motor Red
#define PWM_PIN_D4  15  // P1.15 (Arduino D4) | Right Motor Black
#define PWM_PIN_D5  13  // P1.13 (Arduino D5) | Right Motor Red

// PWM values
extern uint16_t pwm_values[]; 

//////////////////////////////////////////////////////////////////////////////////

/////////////////////////////// Global Variables //////////////////////////////////

#define k .9

// PID controller class
class pid_controller{
    public: 
        float K_mast;   // master gain
        float Kp;   // porportional gain
        float Ki;   // integral gain
        float Kd;   // derivative gain

        float setpoint; // desired tilt angle
        float error;        // current error value
        float prev_error;   // previous error value

        // for derivative low-pass filt time constant if we use
        float tau;

        float proportional; // value of proportional error with gain
        float integral; // accumulated error of integral
        float derivative;   // value of derivative error w ith gain
        float control_sig;  // value of control sigal | sum of all errors

        unsigned long ct; // variable to store current/previous time in microseconds
        float dt; // variable for delta time in seconds

        pid_controller(void);
        void update(void);
};

// IMU class
class complementary_filter {
    public:
        float x;    // temporary value for x-axis
        float y;    // temporary value for y-axis
        float z;    // temporary value for z-axis
        float theta_a;  // value of accelerometer angle
        float theta;    // value of complementary filter angle
        float theta_prev; // value of previous angle

        unsigned long ct; // variable to store current/previous time in microseconds
        float dt; // variable for delta time in seconds

        complementary_filter(void);
        void update(void);
};


extern pid_controller pid;
extern complementary_filter comp_fil;
extern int task;
extern String input;
extern volatile int flag;

//////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////// Timer ISR(s)  ////////////////////////////////////////

extern "C" {
    void TIMER4_IRQHandler_v(void);
}

/////////////////////////////// Funciton Prototypes  /////////////////////////////////////

void setup_Timer4(void);
void setup_PWM0(void);
void keyboard_test(void);
void update_pwm(void);

#endif