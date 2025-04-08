#include "robot_functions.h"

void setup() {
    // initialize serial
    Serial.begin(9600);
    while(!Serial);
    Serial.setTimeout(10);

    // case if serial does not initialize
    if(!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while(1);
    }

    
    setup_Timer4(); // initialize timer4
    
    setup_PWM0();   // initialize PWM0
   
}

void loop() {
    comp_fil.update();

    float actual_angle = comp_fil.theta;   // measured angle from the sensor with filter
    if (actual_angle < 1 && actual_angle > -1) actual_angle = 0;    // low-pass filter for angle
    pid.error = pid.setpoint - actual_angle;    // sets new error
    pid.update();

    update_pwm();   // updates motor pwm values
    keyboard_test();    // gets new K gain values from keyboard

    // Serial.print(pwm_values[1]);
    // Serial.print("\t");
    // Serial.print(pwm_values[0]);
    // Serial.print("\t");
    // Serial.print(pwm_values[3]);
    // Serial.print("\t");
    // Serial.print(pwm_values[2]);

    Serial.print("\t");
    Serial.print(pid.Kp);
    Serial.print("\t");
    Serial.print(pid.Ki);
    Serial.print("\t");
    Serial.print(pid.Kd);
    
    // Serial.print("\t");
    // Serial.print(pid.proportional);
    // Serial.print("\t");
    // Serial.print(pid.integral);
    // Serial.print("\t");
    // Serial.print(pid.derivative);

    // Serial.print("\t");
    // Serial.print(comp_fil.dt,5);
    Serial.print("\t");
    Serial.print(pid.dt,5);
    // Serial.print("\t");
    // Serial.println(comp_fil.theta);
    Serial.print("\t");
    Serial.println(pid.error);
    // Serial.print("\t");
    // Serial.println(pid.control_sig);
    // Serial.println(flag);
}
