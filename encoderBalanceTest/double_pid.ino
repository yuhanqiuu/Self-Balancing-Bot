#include "Arduino_BMI270_BMM150.h"
#include <math.h>
#include <PID_v1.h>
#include "movement.h"
#include "AS5600.h"
#include <ArduinoBLE.h>

#define BUFFER_SIZE 20

//-------------------------------------------------------------------------

// Keyboard test
char userInput;
String input;
int task = 0;

//-------------------------------------------------------------------------

// Angle PID parameters
float Kp = 19;  // (P)roportional Tuning Parameter 12-14? 18
float Ki = 7.5; // (I)ntegral Tuning Parameter        8
float Kd = 0.6; // (D)erivative Tuning Parameter   0.6
float K_mast = 1.0;

float previousError = 0;
float integral = 0;
float proportional = 0;
float derivative = 0;
unsigned long lastTime = 0;
float dt = 0;
unsigned long currentTime = 0;

float old_theta_n = 0;
float theta_n = 0;

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
    currentTime = micros();                           // sets new current time

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
        integral = 0;
    }
    else if (input == "s")
    {
        task = 4;
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
    }
}

//-------------------------------------------------------------------------

void setup()
{
    Serial.begin(9600);
    while (!Serial);
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
    if (!as5600.begin())
    {
        Serial.println("Failed to initialize AS5600 encoder!");
        while (1);
    } // Error message of AS5600

    as5600.begin(4);                        //  set direction pin.
    as5600.setDirection(AS5600_CLOCK_WISE); //  default, just be explicit.

    Serial.println(as5600.getAddress()); // Check Initialization
    int b = as5600.isConnected();
    if (b) Serial.print("Connect: ");
    else Serial.println(b);
    // -------------------------------------------------------------------

    // LED to indicate connection status
    pinMode(LED_BUILTIN, OUTPUT);
    if (!BLE.begin())
    {
        Serial.println("Starting BLE failed!");
        while (1);
    }

    // ------------------------- Bluetooth Setup -------------------------
    BLE.setLocalName("BLE-DEVICE-A12");
    BLE.setDeviceName("BLE-DEVICE-A12");
    customService.addCharacteristic(customCharacteristic);
    BLE.addService(customService);
    customCharacteristic.writeValue("Waiting for data");
    BLE.advertise();
    Serial.println("Bluetooth® device active, waiting for connections...");
    // -------------------------------------------------------------------
}

//-------------------------------------------------------------------------