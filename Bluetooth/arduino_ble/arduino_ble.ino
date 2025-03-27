#include "Arduino_BMI270_BMM150.h"
#include <math.h>
#include <PID_v1.h>
#include "movement.h"
#include "AS5600.h"
#include <ArduinoBLE.h>

#define BUFFER_SIZE 20

//-------------------------------------------------------------------------

// Global Variables
char userInput;
float old_theta_n = 0;
String input;
int task = 0;

float Kp = 18;          // (P)roportional Tuning Parameter 12-14? 18
float Ki = 0;          // (I)ntegral Tuning Parameter        8
float Kd = 0.6;          // (D)erivative Tuning Parameter   0.6
float K_mast = 1.0;
    
// PID Variables
float previousError = 0;
float integral = 0;
float proportional = 0;
float derivative = 0;
unsigned long lastTime = 0;
float dt = 0;
unsigned long currentTime = 0;


float theta_n = 0;     // current angle inputs???
float pidOutput = 0;   // PID output
float setpoint = 0;

// AS5600
AS5600 as5600;   //  use default Wire

//-------------------------------------------------------------------------

// Define a custom BLE service and characteristic
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

//-------------------------------------------------------------------------
// PID contoller

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

  output = constrain(K_mast * (proportional + integral + derivative), -230.0, 230.0); // computes sum of error
  // output = constrain(output, -1000, 1000);  // limits output of PID to limits of PWM

  previousError = error; // update the previous error

  return output;
}
  
//-------------------------------------------------------------------------
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
  }
}

//-------------------------------------------------------------------------

void setup() {
  // Serial.begin(9600);
  // while (!Serial);

  // Initialize IMU for self-balancing
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  //setting up encoder
  // while(!Serial);
  // Serial.begin(115200);
  Wire.begin();

  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.

  Serial.println(as5600.getAddress());

  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  // Initialize the built-in LED to indicate connection status
  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set the device name and local name
  BLE.setLocalName("BLE-DEVICE");
  BLE.setDeviceName("BLE-DEVICE");

  // Add the characteristic to the service
  customService.addCharacteristic(customCharacteristic);

  // Add the service
  BLE.addService(customService);

  // Set an initial value for the characteristic
  customCharacteristic.writeValue("Waiting for data");

  // Start advertising the service
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

//-------------------------------------------------------------------------

void loop() {
  // Wait for a BLE central to connect
  BLEDevice central = BLE.central();
  int leftpwm;
  int rightpwm;
  float current_rpm;
  float result;

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate connection

    // Keep running while connected
    while (central.connected()) {
      keyboard_test();

      old_theta_n = theta_n;
      theta_n = getAngle(old_theta_n); // angles 
      current_rpm = as5600.getAngularSpeed(AS5600_MODE_RPM);
      
      // Run the PID controller
      result = PID(setpoint, theta_n); // targe value = 0, current value = theta_n, pid output

        
      leftpwm = (int) abs(result)*1.3;
      rightpwm = (int) abs(result);

      if(result < 5){
        forward_slow(rightpwm, leftpwm);
      } 
      else if (result > -5){
        backward_slow(rightpwm, leftpwm);
      }
      else forward(0, 0);

      Serial.print(theta_n);
      Serial.print("\t");
      Serial.print(leftpwm);
      Serial.print("\t");
      Serial.print(rightpwm);
      Serial.print("\t");
      Serial.print(Kp);
      Serial.print("\t");
      Serial.print(Ki);
      Serial.print("\t");
      Serial.print(Kd);
      Serial.print("\t");
      Serial.print(result);
      Serial.print("\t");
      Serial.println(current_rpm);
      
        // Check if the characteristic was written
      if (customCharacteristic.written()) {
        // Get the length of the received data
        int length = customCharacteristic.valueLength();

        // Read the received data
        const unsigned char* receivedData = customCharacteristic.value();

        // Create a properly terminated string
        char receivedString[length + 1]; // +1 for null terminator
        memcpy(receivedString, receivedData, length);
        receivedString[length] = '\0'; // Null-terminate the string

        if (strcmp(receivedString, "W") == 0) {
          Serial.println("W");
          setpoint = 1; // setpoint for PID for forward, 1 degree
        } 
        else if (strcmp(receivedString, "S") == 0) {
          Serial.println("S");
          setpoint = -1; // setpoint for PID for backward, 1 degree
        } 
        else if (strcmp(receivedString, "A") == 0) { // left
          Serial.println("A");
          rfw_lbw(leftpwm, rightpwm);
        } 
        else if (strcmp(receivedString, "D") == 0) { // right
          Serial.println("D");
          lfw_rbw(leftpwm, rightpwm);
        }

      }
    }

    digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
    Serial.println("Disconnected from central.");
  }

  keyboard_test();
  old_theta_n = theta_n; 
  theta_n = getAngle(old_theta_n); // angles 
    
  current_rpm = as5600.getAngularSpeed(AS5600_MODE_RPM);
    
  // Run the PID controller
  result = PID(setpoint, theta_n); // targe value = 0, current value = theta_n, pid output

  leftpwm = (int) abs(result)*1.3;
  rightpwm = (int) abs(result);
      
    if(result < 5){
      forward_slow(rightpwm, leftpwm);
    } else if (result > -5){
      backward_slow(rightpwm, leftpwm);
    }
      else forward(0, 0);

      Serial.print(theta_n);
      Serial.print("\t");
      Serial.print(leftpwm);
      Serial.print("\t");
      Serial.print(rightpwm);
      Serial.print("\t");
      Serial.print(Kp);
      Serial.print("\t");
      Serial.print(Ki);
      Serial.print("\t");
      Serial.print(Kd);
      Serial.print("\t");
      Serial.println(result);

}