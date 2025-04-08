#include "Arduino_BMI270_BMM150.h"
#include <math.h>
#include <PID_v1.h>
#include <AS5600.h>
#include <ArduinoBLE.h>
#include "movement.h"
#include <Wire.h>

#define BUFFER_SIZE 20

//-------------------------------------------------------------------------

// Keyboard test
char userInput;
String input;
int task = 0;

//-------------------------------------------------------------------------

// Angle PID parameters
float Kp = 18;  // (P)roportional Tuning Parameter 12-14? 18
float Ki = 8;   // (I)ntegral Tuning Parameter        8
float Kd = 0.6; // (D)erivative Tuning Parameter   0.6
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

float setpoint = 0.0; // Global setpoint for PID controller

//-------------------------------------------------------------------------

// Speed Control Parameters
AS5600 encoder;
int32_t targetSpeed = 0;  // Desired cumulative ticks
int32_t currentSpeed = 0; // From AS5600
int32_t speedError = 0;
float speedIntegral = 0;

float Kp_v = 0.1;          // Start small, tune upward
float Ki_v = Ki_v / 200.0; // Prevent slow drift

float maxTilt = 5;
float PWM_v = 0;
float mech_zero = 0;
float output_v = 0;

//-------------------------------------------------------------------------

// Define a custom BLE service and characteristic
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

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
    derivative = (error - previousError) / dt;
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

// Speed PID controller
float PID_speed(int32_t encoder_left, int32_t encoder_right)
{
    float pwm_speed;
    float Encoder_Least, Encoder;
    float Encoder_Integral;

    Encoder_Least = (encoder_left + encoder_right) - targetSpeed; // movement is target speed = 0

    Encoder *= 0.6;
    Encoder += Encoder_Least * 0.2;

    Encoder_Integral += Encoder;

    // Integral windup guard
    if (Encoder_Integral > 8000)
        Encoder_Integral = 8000;
    if (Encoder_Integral < -6000)
        Encoder_Integral = -6000;

    // PI controller
    pwm_speed = Kp_v * Encoder + Kp_v * Encoder_Integral;

    if (maxTilt == 5)
    {
        Encoder_Integral = 0;
    }

    return pwm_speed;
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
        setpoint = 0;
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
            K_mast = input.toFloat();
        break;
    case 5:
        if (input == "0" || input.toFloat() > 0)
            Kp_v = input.toFloat();
        break;
    case 6:
        if (input == "0" || input.toFloat() > 0)
            Ki_v = input.toFloat();
        break;
    }
}

//-------------------------------------------------------------------------

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ;
    Serial.setTimeout(10);

    // Initialize IMU for self-balancing
    if (!IMU.begin())
    {
        Serial.println("Failed to initialize IMU!");
        while (1)
            ;
    }

    // ------------------------- AS5600 Setup -------------------------

    Wire.begin(); // Initialize I2C
    if (!encoder.begin())
    {
        Serial.println("Failed to initialize AS5600 encoder!");
        while (1)
            ;
    } // Error message of AS5600

    // -------------------------------------------------------------------

    // LED to indicate connection status
    pinMode(LED_BUILTIN, OUTPUT);
    if (!BLE.begin())
    {
        Serial.println("Starting BLE failed!");
        while (1)
            ;
    }

    // ------------------------- Bluetooth Setup -------------------------

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
    int leftpwm;
    int rightpwm;
    float current_rpm;
    float result;

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
            // ------------------------- Speed Controller -----------------------------

            currentSpeed = encoder.getAngularSpeed(AS5600_MODE_DEGREES, true); // deg/sec
            speedError = targetSpeed - currentSpeed;
            speedIntegral += speedError * dt;                      // dt in seconds
            speedIntegral = constrain(speedIntegral, -1000, 1000); // anti-windup
            output_v = constrain(Kp_v * speedError + Ki_v * speedIntegral, -maxTilt, maxTilt);

            // ------------------------- Angle PID Controller -------------------------
            old_theta_n = theta_n;
            theta_n = getAngle(old_theta_n); // angles
            result = PID(output_v + mech_zero, theta_n);

            leftpwm = map(abs(result) * 1.2, 0, 255, 0, 255);
            rightpwm = map(abs(result), 0, 255, 0, 255);

            if (result < 0)
            {
                forward_slow(rightpwm, leftpwm);
            }
            else if (result > 0)
            {
                backward_slow(rightpwm, leftpwm);
            }
            else
                forward(0, 0);

            Serial.print(output_v);
            Serial.print("\t");
            Serial.print(speedError);
            Serial.print("\t");
            Serial.print(speedIntegral);

            Serial.print(theta_n);
            Serial.print("\t");
            // Serial.print(integral);
            // Serial.print("\t");
            // Serial.print(leftpwm);
            // Serial.print("\t");
            // Serial.print(rightpwm);
            // Serial.print("\t");
            Serial.print(Kp);
            Serial.print("\t");
            Serial.print(Ki);
            Serial.print("\t");
            Serial.print(Kd);
            Serial.print("\t");
            Serial.print(Kp_v);
            Serial.print("\t");
            Serial.print(Ki_v);
            Serial.print("\t");
            Serial.println(result); // ends the line

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
                    setpoint = 1; // setpoint for PID for forward, 1 degree
                }
                else if (strcmp(receivedString, "S") == 0)
                {
                    Serial.println("S");
                    setpoint = -1; // setpoint for PID for backward, 1 degree
                }
                else if (strcmp(receivedString, "A") == 0)
                { // left
                    Serial.println("A");
                    rfw_lbw(leftpwm, rightpwm);
                }
                else if (strcmp(receivedString, "D") == 0)
                { // right
                    Serial.println("D");
                    lfw_rbw(leftpwm, rightpwm);
                }
            }

        } // Central Closed
    }
    else
    {
        keyboard_test();
        // ------------------------- Speed Controller -----------------------------

        currentSpeed = encoder.getAngularSpeed(AS5600_MODE_DEGREES, true); // deg/sec
        speedError = targetSpeed - currentSpeed;
        speedIntegral += speedError * dt;                      // dt in seconds
        speedIntegral = constrain(speedIntegral, -1000, 1000); // anti-windup
        output_v = constrain(Kp_v * speedError + Ki_v * speedIntegral, -maxTilt, maxTilt);

        // ------------------------- Angle PID Controller -------------------------
        old_theta_n = theta_n;
        theta_n = getAngle(old_theta_n); // angles
        result = PID(output_v + mech_zero, theta_n);

        leftpwm = map(abs(result) * 1.2, 0, 255, 0, 255);
        rightpwm = map(abs(result), 0, 255, 0, 255);

        if (result < 0)
        {
            forward_slow(rightpwm, leftpwm);
        }
        else if (result > 0)
        {
            backward_slow(rightpwm, leftpwm);
        }
        else
            forward(0, 0);

        Serial.print(output_v);
        Serial.print("\t");
        Serial.print(speedError);
        Serial.print("\t");
        Serial.print(speedIntegral);
        Serial.print("\t");
        Serial.print(theta_n);
        Serial.print("\t");
        // Serial.print(integral);
        // Serial.print("\t");
        // Serial.print(leftpwm);
        // Serial.print("\t");
        // Serial.print(rightpwm);
        // Serial.print("\t");
        Serial.print(Kp);
        Serial.print("\t");
        Serial.print(Ki);
        Serial.print("\t");
        Serial.print(Kd);
        Serial.print("\t");
        Serial.print(Kp_v);
        Serial.print("\t");
        Serial.print(Ki_v);
        Serial.print("\t");
        Serial.println(result); // ends the line
    }
}