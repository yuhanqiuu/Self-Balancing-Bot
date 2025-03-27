#include <ArduinoBLE.h>
#include "movement.h"
#include "Arduino_BMI270_BMM150.h"

#define BUFFER_SIZE 20


// Global Variables
char userInput;
float old_theta_n = 0;
String input;
int task = 0;

float Kp = 15;          // (P)roportional Tuning Parameter 12-14? 18
float Ki = 8;          // (I)ntegral Tuning Parameter        8
float Kd = 5;          // (D)erivative Tuning Parameter   2
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


// Define a custom BLE service and characteristic
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

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
    integral = 0;
  }


  switch (task) {
      case 1:
          if(input == "0" || input.toFloat() > 0) Kp = input.toFloat();
      break;
      case 2:
          if(input == "0" || input.toFloat() > 0) Ki = input.toFloat();
      break;
      case 3:
          if(input == "0" || input.toFloat() > 0) Kd = input.toFloat();
      break;
  }
}

// PID contoller 
float PID(float setpoint, float currentValue){
  float output = 0;
  //   currentTime = micros();
  //   dt = (currentTime - lastTime) / 1000000.0;  // Time difference in seconds
  // lastTime = currentTime;
  dt = (float) (micros() - currentTime) / 1000000.0;  // gets time for ∆t
  currentTime = micros();  // sets new current time

  float error = setpoint - currentValue;

  proportional = Kp * error;

  integral += dt * error;
  integral = constrain(integral, -30, 30);  // Example limit


  // Derivative term (rate of change of error)
  derivative = (error - previousError) / dt;
  if (theta_n - old_theta_n > 0.1 || theta_n - old_theta_n < -0.1) 
        derivative = -Kd * (theta_n - old_theta_n)/dt; // computes the derivative error
    else{ 
        derivative = 0; // filters out noise
    }
    
    output = constrain(K_mast * ( proportional + integral + derivative ), -230.0, 230.0);    // computes sum of error
    // output = constrain(output, -1000, 1000);  // limits output of PID to limits of PWM
    
    previousError = error; // update the previous error

    // if(output>255){
    //   output = 255;
    // } else if(output <-255){
    //   output = -255;
    // }

  return output;
}


void setup() {
  Serial.begin(9600);
  while (!Serial);

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

void loop() {
  // Wait for a BLE central to connect
  BLEDevice central = BLE.central();
  int leftpwm;
  int rightpwm;


  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate connection

    // Keep running while connected
    while (central.connected()) {
    keyboard_test();


    theta_n = getAngle(old_theta_n); // angles 
    old_theta_n = theta_n;
   // float currentAngleMotor = as5600.readAngle() * (360.0 / 4096.0);  // Convert raw to degrees

    
    // Run the PID controller
      float result = PID(0, theta_n); // targe value = 0, current value = theta_n, pid output
      // float motorOutput = constrain(map(abs(result),0,1000,50,255),50,255); // pwm output
      // result = constrain(result, -500, 500);
      // float motorOutput = map(result, -1000, 1000, -255, 255);

      
    leftpwm = (int) abs(result)*1.3;
    rightpwm = (int) abs(result);

      //given left pwm = 7.58 * exp(7.89E-3 * rpm), we can calculate rpm from pwm
      // leftrpm = log(leftpwm/7.58)/7.89E-3; 
      // rightpwm1 = 8.33 * exp(7.55E-3 * leftrpm);
    if(result < 5){
      forward_slow(rightpwm, leftpwm);
      // forward(leftpwm,rightpwm);

    } else if (result > -5){
      backward_slow(rightpwm, leftpwm);
      // backward(leftpwm,rightpwm);

    }
      else forward(0, 0);

      Serial.print(theta_n);
      Serial.print("\t");
      Serial.print(leftpwm);
      Serial.print("\t");
      Serial.print(rightpwm);
      Serial.print("\t");
      // Serial.print(proportional);
      // Serial.print("\t");
      // Serial.print(derivative);
      // Serial.print("\t");
      Serial.print(integral);
      Serial.print("\t");
      // Serial.print(dt,5);
      // Serial.print("\t");
      Serial.print(Kp);
      Serial.print("\t");
      Serial.print(Ki);
      Serial.print("\t");
      Serial.print(Kd);
      Serial.print("\t");
      Serial.println(result);
     



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

    theta_n = getAngle(old_theta_n); // angles 
    old_theta_n = theta_n;
    
   // float currentAngleMotor = as5600.readAngle() * (360.0 / 4096.0);  // Convert raw to degrees

    
    // Run the PID controller
      float result = PID(0, theta_n); // targe value = 0, current value = theta_n, pid output
      // float motorOutput = constrain(map(abs(result),0,1000,50,255),50,255); // pwm output
      // result = constrain(result, -500, 500);
      // float motorOutput = map(result, -1000, 1000, -255, 255);

      
      leftpwm = (int) abs(result)*1.3;
      rightpwm = (int) abs(result);

      //given left pwm = 7.58 * exp(7.89E-3 * rpm), we can calculate rpm from pwm
      float leftrpm = log(leftpwm/7.58)/7.89E-3; 
      float rightpwm1 = 8.33 * exp(7.55E-3 * leftrpm);
    if(result < 5){
      forward_slow(rightpwm, leftpwm);
      // forward(leftpwm,rightpwm);

    } else if (result > -5){
      backward_slow(rightpwm, leftpwm);
      // backward(leftpwm,rightpwm);

    }
      else forward(0, 0);

      Serial.print(theta_n);
      Serial.print("\t");
      Serial.print(leftpwm);
      Serial.print("\t");
      Serial.print(rightpwm);
      Serial.print("\t");
      // Serial.print(proportional);
      // Serial.print("\t");
      // Serial.print(derivative);
      // Serial.print("\t");
      Serial.print(integral);
      Serial.print("\t");
      // Serial.print(dt,5);
      // Serial.print("\t");
      Serial.print(Kp);
      Serial.print("\t");
      Serial.print(Ki);
      Serial.print("\t");
      Serial.print(Kd);
      Serial.print("\t");
      Serial.println(result);

}