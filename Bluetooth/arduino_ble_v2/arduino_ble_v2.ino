#include "Arduino_BMI270_BMM150.h"
#include <math.h>
#include <PID_v1.h>
// #include "movement.h"
#include "AS5600.h"
#include <ArduinoBLE.h>

#define BUFFER_SIZE 20

#define AIN1 2
#define AIN2 3
#define BIN2 4
#define BIN1 5

//-------------------------------------------------------------------------

// Global Variables
char userInput;
float old_theta_n = 0;
String input;
int task = 0;

float Kp = 18.0;          // (P)roportional Tuning Parameter 12-14? 33
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

float gyroBiasX = 0;
//-------------------------------------------------------------------------

// Define a custom BLE service and characteristic
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

// function prototypes
void calibrateGyro(); 
float getAngle(float old_theta_n);
float PID(float setpoint, float input);
float PID2(float setpoint, float input);
void keyboard_test(void);


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
      result = PID(setpoint, theta_n); // targe value = 0, current value = theta_n, pid output
      
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
          leftpwm = result * 1.2 - 20; // turning costants
          rightpwm = result + 20;
        } 
        else if (strcmp(receivedString, "D") == 0) { // right
          Serial.println("D");
          // lfw_rbw(leftpwm, rightpwm);
          leftpwm = result * 1.2 + 20;
          rightpwm = result * 0.5 - 20;
        }

      // leftpwm = map(abs(result)*1.2,0,255,0,255);
      // rightpwm = map(abs(result),0,255,0,255);
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

  if (result < 0.5) {
    // fowrad slow decay

    rightpwm = map(rightpwm,0,255,200,0);   
    leftpwm = map(leftpwm,0,255,200,0);  

    digitalWrite(AIN1, HIGH); 
    analogWrite(AIN2, rightpwm); 

    analogWrite(BIN2, leftpwm); 
    digitalWrite(BIN1, HIGH); 

    // forward_slow(rightpwm, leftpwm);
  } else if (result > -0.5) {

    rightpwm = map(rightpwm,0,255,200,0);   
    leftpwm = map(leftpwm,0,255,200,0);  
    digitalWrite(AIN2, HIGH);   
    analogWrite(AIN1, rightpwm);

    digitalWrite(BIN2, HIGH); 
    analogWrite(BIN1, leftpwm); 

    // backward_slow(rightpwm, leftpwm);
  } else {
    // stop

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
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
    Serial.print(Ki);kp

    Serial.print("\t");
    Serial.println(Kd);
}



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

  // Use different gains based on error threshold
  float Kp_use = (abs(error_value) < 5) ? consKp : Kp;
  float Ki_use = (abs(error_value) < 5) ? consKi : Ki;
  float Kd_use = (abs(error_value) < 5) ? consKd : Kd;

  proportional = Kp_use * error_value;
  integral += Ki_use * dt * error_value;

  if (abs(integral) > 255) integral *= 0.9;  // Soft constraint

  // Compute derivative, with filtering
  if (abs(theta_n - old_theta_n) > 0.1) {
      derivative = -Kd_use * (theta_n - old_theta_n) / dt;
  } else {
      derivative *= 0.8;  // Low-pass filtering instead of setting to zero
  }

  output = constrain(K_mast * (proportional + integral + derivative), -255.0, 255.0);
  previousError = error_value;

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

void calibrateGyro() {
  int numSamples = 500;
  float sumX = 0, sumY = 0, sumZ = 0;
  float gx, gy, gz;

  for (int i = 0; i < numSamples; i++) {
      IMU.readGyroscope(gx,gy,gz); // Collect gyroscope data
      sumX += gx;  
      delay(2); // Small delay to prevent excessive sampling
  }

  gyroBiasX = sumX / numSamples;
}

float getAngle(float theta_n)
{
    float k = 0.95; // weighting factor
    float theta_an, theta_gn = 0;
    float gx, gy, gz, ax, ay, az;

    float dt = (float) (micros() - currentTime) / 1000000;  // gets time for ∆t
    currentTime = micros();  // sets new current time

    if (IMU.gyroscopeAvailable()) {
        // reads gyroscope value
        IMU.readGyroscope(gx, gy, gz);
        gx -= gyroBiasX; // Subtract bias factor
        
        // computes theta using integration
        theta_gn = (theta_n + gx * dt);
        
    }

    if (IMU.accelerationAvailable()) {
        // reads acceleration
        IMU.readAcceleration(ax, ay, az);
        
        // computes theta values
        theta_an = -degrees(atan(ay / az)); 
    }

    theta_n = k * theta_gn + (1 - k) * theta_an;
    return theta_n;
}