#include "robot_functions.h"

/////////////////////////////// Global Variables //////////////////////////////////

// PWM values (initialized to 0%)
uint16_t pwm_values[] = {0 | (1 << 15), 200 | (1 << 15), 0 | (1 << 15), 0 | (1 << 15)}; 

pid_controller pid; // creates instance of class pid_controller named pid
complementary_filter comp_fil;   // creates instance of class complementary_filter named comp_fil

int task = 0;   // for serial testing
String input;   // for serial testing
volatile int flag = 0;  // for timer testing

////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////// Timer ISR(s) ////////////////////////////////////


extern "C" {
    /** 
     * Interrupt Service Routine (ISR) for Timer 4
     */ 
    void TIMER4_IRQHandler_v(void) {
        if (NRF_TIMER4->EVENTS_COMPARE[0]) { // Checks if event occured
            NRF_TIMER4->EVENTS_COMPARE[0] = 0;  // Clear the event
            flag++;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////

////////////////////////////////// Functions //////////////////////////////////////

/**
 * Timer4 setup function
 * 
 * Sets to Timer mode (Counter) at 1 MHz frequency
 * Compare event(s) at: 1 second
 */
void setup_Timer4(void) {
    // Enables TIMER4
    NRF_TIMER4->TASKS_STOP = 1;  // Stop timer (just in case)
    NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer;  // Set to Timer mode
    NRF_TIMER4->PRESCALER = 4;  // 16 MHz / 2^4 = 1 MHz timer frequency
    NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_32Bit;  // 32-bit mode
    NRF_TIMER4->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos; // Shortcut to clear compare register
    
    // Sets Compare Register(s) (Up to 6 compare evenets)
    // Compare event for 1 second
    NRF_TIMER4->CC[0] = 1000000;
    
    // Enable Interrupt on Compare Event(s)
    NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
    NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE1_Disabled << TIMER_INTENSET_COMPARE0_Pos;
    NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE2_Disabled << TIMER_INTENSET_COMPARE0_Pos;
    NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE3_Disabled << TIMER_INTENSET_COMPARE0_Pos;
    NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE4_Disabled << TIMER_INTENSET_COMPARE0_Pos;
    NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE5_Disabled << TIMER_INTENSET_COMPARE0_Pos;
    NVIC_EnableIRQ(TIMER4_IRQn);  // Enable interrupt in NVIC
    
    // Start the Timer
    NRF_TIMER4->TASKS_CLEAR = 1;  // Reset counter
    NRF_TIMER4->TASKS_START = 0;  // Start timer
}


/**
 * PWM0 setup function
 * 
 * Sets to Timer mode (Counter) at 1 MHz frequncy
 * Countertop set at 1000 clock cycles for 1kHz
 */
void setup_PWM0 (void) {
    // Enable the PWM hardware module
    NRF_PWM0->ENABLE = 1;
    
    // Set PWM mode to UP mode (counts up and resets)
    NRF_PWM0->MODE = PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos;
    
    // Set the prescaler (PWM frequency base) - 16MHz / 16 = 1MHz timer
    NRF_PWM0->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_16;
    
    // Set the maximum counter top value (for a frequency of 1kHz)
    NRF_PWM0->COUNTERTOP = 1000;  // 1 MHz / 1000 = 1 kHz PWM Frequency
    
    // Configure the sequence (4 independent PWM duty cycles)
    NRF_PWM0->SEQ[0].PTR = (uint32_t)pwm_values;  // Point to duty cycle array
    NRF_PWM0->SEQ[0].CNT = 4;  // 4 PWM outputs
    NRF_PWM0->SEQ[0].REFRESH = 0;
    NRF_PWM0->SEQ[0].ENDDELAY = 0;
    
    // Configure PWM output pins
    NRF_PWM0->PSEL.OUT[0] = (1 << 5) | PWM_PIN_D2;  // Select P1.11 (D2)
    NRF_PWM0->PSEL.OUT[1] = (1 << 5) | PWM_PIN_D3;  // Select P1.12 (D3)
    NRF_PWM0->PSEL.OUT[2] = (1 << 5) | PWM_PIN_D4;  // Select P1.15 (D4)
    NRF_PWM0->PSEL.OUT[3] = (1 << 5) | PWM_PIN_D5;  // Select P1.13 (D5)
    
    // Configure PWM decoder (individual duty cycles)
    NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) | (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    
    // Enable the loop and start the sequence
    NRF_PWM0->LOOP = PWM_LOOP_CNT_Disabled;
    NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

// Constructor definition
pid_controller::pid_controller() {
    K_mast = 1.0f;   // master gain
    Kp = 37f;   // porportional gain
    Ki = 280.0f;   // integral gain
    Kd = 0.15f;  // derivative gain

    setpoint = 0.0f; // desired tilt angle
    error = 0.0f;        // current error value
    prev_error = 0.0f;   // previous error value

    // for derivative low-pass filt time constant if we use
    tau = 0.0f;

    proportional = 0.0f; // value of proportional error with gain
    integral = 0.0f; // accumulated error of integral
    derivative = 0.0f;   // value of derivative error w ith gain
    control_sig = 0.0f;  // value of control sigal | sum of all errors

    ct = 0; // variable to store current/previous time in microseconds
    dt = 0.0f; // variable for delta time in seconds
}

/**
 * Function to update PID controller
 * 
 * Takes the sum of all error multiplied with respective gain. Integral is constrained to (-10, and 10) to limit windup.
 */
void pid_controller::update(void) {
    dt = (float) (micros() - ct) / 1000000;  // gets time for ∆t
    ct = micros();  // sets new current time
    proportional = Kp * error; // computes the proportional error

    integral += Ki * dt * (error + prev_error) / 2.0;   // computes the integral error
    integral = constrain(integral, -250, 250);    // contrains the error to prevent integral windup

    if (comp_fil.theta - comp_fil.theta_prev > 0.1 || comp_fil.theta - comp_fil.theta_prev < -0.1) 
        derivative = -Kd * (comp_fil.theta - comp_fil.theta_prev)/dt; // computes the derivative error
    else 
        derivative = 0; // filters out noise

    control_sig = K_mast * ( proportional + integral + derivative );    // computes sum of error
    control_sig = constrain(control_sig, -1000, 1000);  // limits output of PID to limits of PWM
    
    prev_error = error; // update the previous error
}

// Constructor definition
complementary_filter::complementary_filter() {
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    theta_a = 0.0f;
    theta = 0.0f;
    theta_prev = 0.0f;
    ct = 0;
    dt = 0.0f;
}

/**
 * Function to use update IMU using complementary filter
 */
void complementary_filter::update(void) {
    dt = (float) (micros() - ct) / 1000000;  // gets time for ∆t
    ct = micros();  // sets new current time
    theta_prev = theta;

    if (IMU.gyroscopeAvailable()) {
        // reads gyroscope value
        IMU.readGyroscope(x, y, z);
        
        // computes theta using integration
        theta = (theta + x * dt);
    }

    if (IMU.accelerationAvailable()) {
        // reads acceleration
        IMU.readAcceleration(x, y, z);
        
        // computes theta values
        theta_a = -degrees(atan(y / z));
    }

    theta = k * theta + (1 - k) * theta_a;
}

/**
 * Function to update K gain values through serial
 * 
 * Enter kp/ki/kd
 * Enter gain value
 */
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
      pid.integral = 0;
    }


    switch (task) {
        case 1:
            if(input == "0" || input.toFloat() > 0) pid.Kp = input.toFloat();
        break;
        case 2:
            if(input == "0" || input.toFloat() > 0) pid.Ki = input.toFloat();
        break;
        case 3:
            if(input == "0" || input.toFloat() > 0) pid.Kd = input.toFloat();
        break;
    }
}

/**
 * Function to update PWM for motors
 * 
 * Slow decay mode to increase torque
 * 100% dutcy cycle is stop 0% duty cycle is full speed
 */
void update_pwm(void) {
    if (pid.control_sig < 0) {
        if(pid.control_sig != 0) {
            pwm_values[1] = (1 << 15) | map(abs(pid.control_sig), 0, 1000, 950, 0);
            pwm_values[3] = pwm_values[1]-50;
        }
        else {
            pwm_values[1] = (1 << 15) | 1000;
            pwm_values[3] = (1 << 15) | 1000;
        }

        pwm_values[0] = (1 << 15) | 1000;
        pwm_values[2] = (1 << 15) | 1000;
    } 
    else {
        if(pid.control_sig != 0) {
            pwm_values[0] = (1 << 15) | (map(abs(pid.control_sig), 0, 1000, 950, 0));
            pwm_values[2] = pwm_values[0]-50;
        }
        else {
            pwm_values[0] = (1 << 15) | 1000;
            pwm_values[2] = (1 << 15) | 1000;
        }
        
        pwm_values[1] = (1 << 15) | 1000;
        pwm_values[3] = (1 << 15) | 1000;
    }

    // Wait for the PWM sequence to finish before updating
    while (NRF_PWM0->EVENTS_SEQEND[0] == 0);
    NRF_PWM0->EVENTS_SEQEND[0] = 0;  // Clear event flag

    // Restart the PWM sequence with updated values
    NRF_PWM0->TASKS_SEQSTART[0] = 1;
}