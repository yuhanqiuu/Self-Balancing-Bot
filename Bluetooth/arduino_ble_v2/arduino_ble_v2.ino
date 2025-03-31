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

float Kp = 20;          // (P)roportional Tuning Parameter 12-14? 33
float Ki = 0;          // (I)ntegral Tuning Parameter        78
float Kd = 0.63;          // (D)erivative Tuning Parameter   0.77
float K_mast = 1.0;

float consKp = 20;          
float consKi = 0;          
float consKd = 0.63;         
// PID Variables
float previousError = 0;
float integral = 0;
float proportional = 0;
float derivative = 0;
unsigned long lastTime = 0;
float dt = 0;
unsigned long currentTime = 0;

float k_correction = 0.05;  // Tunable scaling factor
float correction = 0.0;

float theta_n = 0;     // current angle inputs???
float setpoint = -1.75;
float error_value = 0.0;

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

  error_value = setpoint - currentValue;

  proportional = Kp * error_value;

  integral += Ki * dt * error_value;
  integral = constrain(integral, -30, 30); // Example limit

  // Derivative term (rate of change of error_value)
  if (theta_n - old_theta_n > 0.1 || theta_n - old_theta_n < -0.1)
    derivative = -Kd * (theta_n - old_theta_n) / dt; // computes the derivative error_value
  else
  {
    derivative = 0; // filters out noise
  }

  output = constrain(K_mast * (proportional + integral + derivative), -255.0, 255.0); // computes sum of error_value
  // output = constrain(output, -1000, 1000);  // limits output of PID to limits of PWM

  previousError = error_value; // update the previous error_value

  return output;
}

//-------------------------------------------------------------------------
// PID contoller with 2 k values
float PID2(float setpoint, float currentValue)
{
  float output = 0;

  currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0; 
  lastTime = currentTime;

  error_value = setpoint - currentValue;

  if (abs(error_value) < 5 ) {
    proportional = consKp * error_value;

    integral += consKi * dt * error_value;
    // integral = constrain(integral, -255, 255); // hard constrainter
    if (abs(integral) > 255) integral *= 0.9;  // Soft constraint

    // Derivative term (rate of change of error_value)
      derivative = (error_value - previousError) / dt;
    if (theta_n - old_theta_n > 0.1 || theta_n - old_theta_n < -0.1)
      derivative = -consKd * (theta_n - old_theta_n) / dt; // computes the derivative error_value
    else {
      derivative = 0; // filters out noise
    }
  }

  else {
  

  proportional = Kp * error_value;

  integral += Ki * dt * error_value;
  // integral = constrain(integral, -255, 255); // Example limit
  if (abs(integral) > 255) integral *= 0.9;  // Soft constraint

  // Derivative term (rate of change of error_value)
  derivative = (error_value - previousError) / dt;
  if (theta_n - old_theta_n > 0.1 || theta_n - old_theta_n < -0.1)
    derivative = -Kd * (theta_n - old_theta_n) / dt; // computes the derivative error_value
  else
  {
    derivative = 0; // filters out noise
  }
}

  output = constrain(K_mast * (proportional + integral + derivative), -255.0, 255.0); // computes sum of error_value
  // output = constrain(output, -1000, 1000);  // limits output of PID to limits of PWM

  previousError = error_value; // update the previous error_value

  return output;
}


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
  else if (input == "s")// setpoint
  {
    task = 4;
  }
  else if (input == "ckp")
  {
    task = 5;
  }
  else if (input == "cki")
  {
    task = 6;
  }
  else if (input == "ckd")
  {
    task = 7;
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
    if (input == "0"||input.toFloat() != 0)
      setpoint = input.toFloat();
    break;
  case 5:
    if (input == "0"||input.toFloat() != 0)
      consKp = input.toFloat();
    break;
  case 6:
    if (input == "0"||input.toFloat() != 0)
      consKi = input.toFloat();
    break;
  case 7:
    if (input == "0"||input.toFloat() != 0)
      consKd = input.toFloat();
    break;
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.setTimeout(10);

  // Initialize IMU for self-balancing
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }


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
  float gx, gy, gz;

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate connection

    // Keep running while connected
    while (central.connected()) {
      keyboard_test();

      old_theta_n = theta_n;
      theta_n = getAngle(old_theta_n); // angles       
      // Run the PID controller
      result = PID2(setpoint, theta_n); // targe value = 0, current value = theta_n, pid output

      Serial.print(theta_n);
      Serial.print("\t");
      Serial.print(setpoint);
      Serial.print("\t");
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
      Serial.println(k_correction);
      // Serial.print("\t");
      // Serial.print(result);
      // Serial.print("\t");
      // Serial.println(current_rpm);
      
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
          setpoint = 2-1.75; // setpoint for PID for forward, 2 degree
          leftpwm = abs(result)*1.2;
          rightpwm = abs(result);
        } 
        else if (strcmp(receivedString, "S") == 0) {
          Serial.println("S");
          setpoint = -2-1.75; // setpoint for PID for backward, 2 degree
          leftpwm = abs(result)*1.2;
          rightpwm = abs(result);
        } 
        else if (strcmp(receivedString, "A") == 0) { // left
          Serial.println("A");
          
          // do we run two motors in different direction or scale them for turning?
          // rfw_lbw(leftpwm, rightpwm); 
          leftpwm = result * 0.5 * 1.2;
          rightpwm = result;
        } 
        else if (strcmp(receivedString, "D") == 0) { // right
          Serial.println("D");
          // lfw_rbw(leftpwm, rightpwm);
          leftpwm = result * 1.2;
          rightpwm = result * 0.5;
        }

      // leftpwm = map(abs(result)*1.2,0,255,0,255);
      // rightpwm = map(abs(result),0,255,0,255);

      if(result < 0){
        forward_slow(rightpwm, leftpwm);
      } 
      else if (result > 0){
        backward_slow(rightpwm, leftpwm);
      }
      else forward_slow(255, 255);
      }
    }

    digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
    Serial.println("Disconnected from central.");


  }

  keyboard_test();
  old_theta_n = theta_n; 
  theta_n = getAngle(old_theta_n); // angles 
    
  result = PID(setpoint, theta_n);
  leftpwm = abs(result);
  rightpwm = abs(result);

  // // Apply correction
  // IMU.readGyroscope(gx, gy, gz);
  // correction = k_correction * gz;
  // leftpwm  = constrain(leftpwm - correction, 0, 255);
  // rightpwm = constrain(rightpwm + correction, 0, 255);
  float scale_factor = 1 - (abs(theta_n) / 10); // MAX_theta_n = 10

  if (result < 3) {
    forward_slow(rightpwm, leftpwm);
  } else if (result > -3) {
    rightpwm = 0.95*rightpwm;
    leftpwm = 0.95*leftpwm;
    backward_slow(rightpwm, leftpwm);
  } else {
    forward(0, 0);
  }

      Serial.print(theta_n);
      Serial.print("\t");
      // Serial.print(setpoint);
      // Serial.print("\t");
      Serial.print(scale_factor);
      Serial.print("\t");
      Serial.print(integral);
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
      Serial.print(consKp);
      Serial.print("\t");
      Serial.print(consKi);
      Serial.print("\t");
      Serial.print(consKd);
      Serial.print("\t");
      Serial.println(result);
      // Serial.print("\t");
      // Serial.println(current_rpm);

}


