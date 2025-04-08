#include "Arduino_BMI270_BMM150.h"
#include <math.h>
#include <PID_v1.h>
#include <AS5600.h>
#include <ArduinoBLE.h>
#include "movement.h"
#include <Wire.h>
#include "TCA9548A.h"


// #include "../utils/movement.h"

#define BUFFER_SIZE 20

//-------------------------------------------------------------------------

// Keyboard test
char userInput;
String input;
int task = 0;

//-------------------------------------------------------------------------

// Angle PID parameters
float Kp = 21;  // (P)roportional Tuning Parameter 12-14? 21
float Ki = 0;   // (I)ntegral Tuning Parameter        0
float Kd = 0.5; // (D)erivative Tuning Parameter   0.5
float K_mast = 0.6;

float previousError = 0;
float integral = 0;
float proportional = 0;
float derivative = 0;
unsigned long lastTime = 0;
float dt = 0;
unsigned long currentTime = 0;

float old_theta_n = 0;
float theta_n = 0;

float setpoint = 0; // Global setpoint for PID controller

//-------------------------------------------------------------------------

// Speed Control Parameters
// Multiplexer and AS5600 objects
TCA9548A I2CMux;
AS5600 encoderLeft;
AS5600 encoderRight;

// Filtering parameters
float filteredRPMLeft = 0;
float filteredRPMRight = 0;
const float alpha = 0.2;          // Smoothing factor
const float noiseThreshold = 3.0; // RPM threshold for noise suppression

// Previous angle and timestamp per encoder
float prevAngleLeft = 0;
float prevAngleRight = 0;
unsigned long prevTimeLeft = 0;
unsigned long prevTimeRight = 0;

float setpoint_speed = 0; // Desired cumulative ticks

float Kp_v = 8;          // Start small, tune upward    8
float Ki_v = Kp_v / 200.0; // Prevent slow drift

float PWM_a = 0;
float PWM_vl = 0;
float PWM_vr = 0;
float PWM_t = 0;
float totalPWM = 0;

float Encoder_Integral = 0.0;

int leftpwm = 0;
int rightpwm = 0;

int stop_flag = 0;
int turning = 0;

int turning_left_w = 0;
int turning_right_w = 0;
int turning_offest = 0;
//-------------------------------------------------------------------------

// Define a custom BLE service and characteristic
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

//-------------------------------------------------------------------------

float PID_speed(float filteredRPM, float setpoint_speed);
float readRPM(float &previousAngle, unsigned long &previousTime, AS5600 &encoder);
void keyboard_test(void);
float PID(float setpoint, float currentValue);

//-------------------------------------------------------------------------

void setup()
{
    // Serial.begin(9600);
    // while (!Serial);
    // Serial.setTimeout(10);

    // Initialize IMU for self-balancing
    if (!IMU.begin())
    {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    // ------------------------- TCA Setup -------------------------
    Wire.begin();
    I2CMux.begin(Wire);
    I2CMux.closeAll();

    // ------------------------- Prime encoders -------------------------
    I2CMux.openChannel(0);
    delay(10);

    prevAngleLeft = encoderLeft.readAngle() * (360.0 / 4096.0);
    prevTimeLeft = micros();
    I2CMux.closeChannel(0);
    I2CMux.openChannel(1);

    prevAngleRight = encoderRight.readAngle() * (360.0 / 4096.0);
    prevTimeRight = micros();
    I2CMux.closeChannel(1);

    Serial.println("AS5600 RPM monitoring via TCA9548A started.");

    // ------------------------- Bluetooth Setup -------------------------

    pinMode(LED_BUILTIN, OUTPUT);
    if (!BLE.begin())
    {
        Serial.println("Starting BLE failed!");
        while (1)
            ;
    }

    BLE.setLocalName("BLE-DEVICE-A12");
    BLE.setDeviceName("BLE-DEVICE-A12");
    customService.addCharacteristic(customCharacteristic);
    BLE.addService(customService);
    customCharacteristic.writeValue("Waiting for data");
    BLE.advertise();
    Serial.println("Bluetooth® device active, waiting for connections...");
}

//-------------------------------------------------------------------------

void loop()
{
    BLEDevice central = BLE.central();
    
    float current_rpm;
    float rpmLeft = 0;
    float rpmRight = 0;
    

    if (central)
    {
        // Check central connection
        Serial.print("Connected to central: ");
        Serial.println(central.address());
        digitalWrite(LED_BUILTIN, HIGH);
        // Keep running while connected
        while (central.connected())
        {
            keyboard_test();

            I2CMux.openChannel(0);
            rpmLeft = readRPM(prevAngleLeft, prevTimeLeft, encoderLeft);
            I2CMux.closeChannel(0);

            // --- Right Encoder (Channel 1) ---
            I2CMux.openChannel(1);
            rpmRight = -readRPM(prevAngleRight, prevTimeRight, encoderRight); // Negate if needed for direction
            I2CMux.closeChannel(1);

            // --- Filter ---
            filteredRPMLeft = alpha * rpmLeft + (1 - alpha) * filteredRPMLeft;
            filteredRPMRight = alpha * rpmRight + (1 - alpha) * filteredRPMRight;

            if (abs(filteredRPMLeft) < noiseThreshold)
                filteredRPMLeft = 0;
            if (abs(filteredRPMRight) < noiseThreshold)
                filteredRPMRight = 0;
            
            Ki_v = Kp_v / 200.0;

            PWM_vl = PID_speed(filteredRPMLeft, setpoint_speed);

            // ------------------------- Bluetooth Controller -------------------------
            if (customCharacteristic.written())
            {
                int length = customCharacteristic.valueLength();
                const unsigned char *receivedData = customCharacteristic.value();
                char receivedString[length + 1];

                memcpy(receivedString, receivedData, length);
                receivedString[length] = '\0';

                if (strcmp(receivedString, "W") == 0)
                {
                    Serial.println("W");
                    setpoint = 0.9; // setpoint for PID for forward, 1 degree
                    stop_flag = 0;
                    turning = 0;
                    setpoint_speed = 20;
                    turning_left_w = 0;
                    turning_right_w = 0;
                    turning_offest = 0;
                }
                else if (strcmp(receivedString, "S") == 0)
                {
                    Serial.println("S");
                    setpoint = -0.9; // setpoint for PID for backward, 1 degree
                    stop_flag = 0;
                    turning = 0;
                    setpoint_speed = -20;
                    turning_left_w = 0;
                    turning_right_w = 0;
                    turning_offest = 0;
                }
                else if (strcmp(receivedString, "A") == 0) // turning left 
                { // left
                    Serial.println("A");
                    turning_left_w = 0;
                    turning_right_w = 10;
                    stop_flag = 0;
                    turning_offest = -10;
                }
                else if (strcmp(receivedString, "D") == 0) // turning right
                { // right
                    Serial.println("D");
                    turning_left_w = 40;
                    turning_right_w = 1;
                    stop_flag = 0;

                    turning_offest = 10;
                }
                else if (strcmp(receivedString, "0") == 0)
                { 
                    Serial.println("stop");
                    stop_flag = 1;
                    turning_left_w = 0;
                    turning_right_w = 0;
                    turning_offest = 0;
                }
                
            }

            if (stop_flag){
                setpoint = 0.0;
                setpoint_speed = 0.0;
                turning_left_w = 0;
                turning_right_w = 0;
                turning_offest = 0;
            } 

            theta_n = getAngle(old_theta_n); // angles
            PWM_a = PID(setpoint, theta_n);
            old_theta_n = theta_n;

            leftpwm = PWM_vl + PWM_a;
            rightpwm = PWM_vl + PWM_a;

            totalPWM = PWM_vl + PWM_a;
            int total_leftpwm = constrain(leftpwm + turning_offest, -255, 255); 
            int total_rightpwm = constrain(rightpwm - turning_offest, -255, 255);

            // if (leftpwm > 0 && rightpwm > 0)
            // {
            //     if (turning_right_w) {
            //         forward_slow(total_leftpwm, total_rightpwm);
            //     }
            //     else if (turning_left_w) {
            //         backward_slow(total_leftpwm, total_rightpwm);
            //     }
            //     else {
            //         forward_slow(total_leftpwm, total_rightpwm);
            //     }
            // }
            // else if (leftpwm < 0 && rightpwm < 0)
            // {
            //     backward_slow(leftpwm, rightpwm);
            // }
            // else
            //     forward(0, 0);

            if (totalPWM>0)
            {
                forward_slow(total_leftpwm, total_rightpwm);
            }
            else if (totalPWM<0)
            {
                backward_slow(leftpwm, rightpwm);
            }
            else
                forward(0, 0);


            // driveMotors(total_leftpwm, total_rightpwm);
            
            Serial.print(total_leftpwm);
            Serial.print("\t");
            Serial.print(total_rightpwm);
            Serial.print("\t");
            Serial.print(leftpwm);
            Serial.print("\t");
            Serial.print(rightpwm);
            Serial.print("\t");
            Serial.print(turning_left_w);
            Serial.print("\t");
            Serial.print(turning_right_w);
            Serial.print("\t");
            Serial.println(totalPWM);
        } // Central Closed
    }
    else
    {
        keyboard_test();

        // ------------------------- Angle PID Controller -------------------------

        theta_n = getAngle(old_theta_n); // angles
        PWM_a = PID(setpoint, theta_n);
        old_theta_n = theta_n;

        // ------------------------- Speed Controller -----------------------------
        // --- Left Encoder (Channel 0) ---
        I2CMux.openChannel(0);
        rpmLeft = readRPM(prevAngleLeft, prevTimeLeft, encoderLeft);
        I2CMux.closeChannel(0);

        // --- Right Encoder (Channel 1) ---
        I2CMux.openChannel(1);
        rpmRight = -readRPM(prevAngleRight, prevTimeRight, encoderRight); // Negate if needed for direction
        I2CMux.closeChannel(1);

        // --- Filter ---
        filteredRPMLeft = alpha * rpmLeft + (1 - alpha) * filteredRPMLeft;
        filteredRPMRight = alpha * rpmRight + (1 - alpha) * filteredRPMRight;

        if (abs(filteredRPMLeft) < noiseThreshold)
            filteredRPMLeft = 0;
        if (abs(filteredRPMRight) < noiseThreshold)
            filteredRPMRight = 0;
        
        Ki_v = Kp_v / 200.0;

        PWM_vl = PID_speed(filteredRPMLeft, setpoint_speed);
        PWM_vr = PID_speed(filteredRPMRight, setpoint_speed);

        // ------------------------- PWM Assignment-----------------------------

        leftpwm = PWM_vl + PWM_a;
        rightpwm = PWM_vl + PWM_a;

        totalPWM = PWM_vl + PWM_a;
        int total_leftpwm = constrain((leftpwm + turning_left_w), -255, 255); 
        int total_rightpwm = constrain(rightpwm + turning_right_w, -255, 255);
        // if (totalPWM > 0)
        // {
        //     forward_slow(rightpwm, leftpwm); // originally rightpwm, leftpwm
        // }
        // else if (totalPWM < 0)
        // {
        //     backward_slow(rightpwm, leftpwm);
        // }
        // else
        //     forward(0, 0);
        driveMotors(total_leftpwm, total_rightpwm);
    }
}


//-------------------------------------------------------------------------
// Speed PID controller
float PID_speed(float filteredRPM, float targetSpeed)
{
    float pwm_speed;
    float Encoder_Least, Encoder;
    Encoder_Integral;

    Encoder_Least = filteredRPM - targetSpeed; // movement is target speed = 0

    Encoder *= 0.6;
    Encoder += Encoder_Least * 0.2;

    Encoder_Integral += Encoder;

    // Integral windup guard
    if (Encoder_Integral > 700)
        Encoder_Integral = 700;
    if (Encoder_Integral < -700)
        Encoder_Integral = -700;

    if (theta_n > 35 || theta_n < -35 )
        Encoder_Integral = 0;
    
    // PI controller
    pwm_speed = Kp_v * Encoder + Ki_v * Encoder_Integral;

    return pwm_speed;
}

//-------------------------------------------------------------------------
// Angle PID controller
float PID(float setpoint, float currentValue)
{
    float output = 0;
    dt = (float)(micros() - currentTime) / 1000000.0; // gets time for ∆t
    if (dt <= 0 || dt > 0.2)
        dt = 0.01;          // clamp for safety
    currentTime = micros(); // sets new current time

    float error = setpoint - currentValue;

    proportional = Kp * error;

    integral += Ki * dt * error;
    integral = constrain(integral, -30, 30); // Example limit

    // Derivative term (rate of change of error)
    // derivative = (error - previousError) / dt;
    if (theta_n - old_theta_n > 0.1 || theta_n - old_theta_n < -0.1)
        derivative = -Kd * (theta_n - old_theta_n) / dt; // computes the derivative error
    else
    {
        derivative = 0; // filters out noise
    }

    output = constrain(K_mast * (proportional + integral + derivative), -255.0, 255.0); // computes sum of error
    // output = constrain(output, -1000, 1000);  // limits output of PID to limits of PWM

    previousError = error; // update the previous error

    return output;
}

//-------------------------------------------------------------------------
// Keyboard Input PID values
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
        Encoder_Integral = 0;
        integral = 0;
    }
    else if (input == "s")
    {
        task = 4;
    }
    else if (input == "kpv")
    {
        task = 5;
    }
    else if (input == "kiv")
    {
        task = 6;
    }
    else if (input == "km")
    {
        task = 7;
    }
    else if (input == "tl")
    {
        task = 8;
    }
    else if (input == "tr")
    {
        task = 9;
    }

    switch (task)
    {
    case 1:
        if (input == "0" || input.toFloat() > 0)
            Kp = input.toFloat();
        break;
    case 2:
        if (input == "0" || input.toFloat() > 0)
            Ki = input.toFloat();
        break;
    case 3:
        if (input == "0" || input.toFloat() > 0)
            Kd = input.toFloat();
        break;
    case 4:
        if (input == "0" || input.toFloat() != 0)
            setpoint = input.toFloat();
        break;
    case 5:
        if (input == "0" || input.toFloat() > 0)
            Kp_v = input.toFloat();
        break;
    case 6:
        if (input == "0" || input.toFloat() > 0)
            Ki_v = input.toFloat();
        break;
    case 7:
        if (input == "0" || input.toFloat() > 0)
            K_mast = input.toFloat();
    break;
    }
}

//-------------------------------------------------------------------------
// Reads RPM from an AS5600 encoder via angle delta and time delta
float readRPM(float &previousAngle, unsigned long &previousTime, AS5600 &encoder)
{
    float currentAngle = encoder.readAngle() * (360.0 / 4096.0);
    unsigned long currentTime = micros();
    float dt = (float)(currentTime - previousTime) / 1e6; // seconds

    float delta = currentAngle - previousAngle;
    if (delta > 180)
        delta -= 360;
    if (delta < -180)
        delta += 360;

    float rpm = (delta / 360.0) * 60.0 / dt;

    previousAngle = currentAngle;
    previousTime = currentTime;

    return rpm;
}