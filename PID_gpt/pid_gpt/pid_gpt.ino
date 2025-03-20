#include <math.h>
#include "Arduino_BMI270_BMM150.h"
#include <Arduino.h>


#define AIN1 2
#define AIN2 3
#define BIN2 4
#define BIN1 5

// PID Constants

float Kp = 15.0;
float Ki = 0.0;
float Kd = 0.0;
float K_mast = 1.0;

// PID Variables
float previousError = 0;
float integral = 0;
unsigned long lastTime = 0;
float dt = 0;
float currentTime = 0;

// Output limits
float maxOutput = 255;
float minOutput = 35;

//keyboard input
char userInput;
float old_theta_n = 0;
String input;
int task = 0;

float theta_n;


// Forward fast decay
void forward(int pwm1, int pwm2) {
    analogWrite(AIN1, pwm1);   
    digitalWrite(AIN2, LOW);   
    analogWrite(BIN1, pwm2); 
    digitalWrite(BIN2, LOW); 
}

// Reverse fast decay
void backward(int pwm1, int pwm2) {  
    digitalWrite(AIN1, LOW);   
    analogWrite(AIN2, pwm1);   
    digitalWrite(BIN1, LOW); 
    analogWrite(BIN2, pwm2);  
}

void forward_slow(int pwm1, int pwm2) {  
      digitalWrite(AIN2, HIGH);   
      analogWrite(AIN1, pwm1);
  
      digitalWrite(BIN2, HIGH); 
      analogWrite(BIN1, pwm2);  
  }

void backward_slow(int pwm1, int pwm2){
    analogWrite(AIN2, pwm1);   
    digitalWrite(AIN1, HIGH); 
  
    analogWrite(BIN2, pwm2); 
    digitalWrite(BIN1, HIGH); 
  }

float getAngle(float old_theta_n)
{
    float k = 0.; // weighting factor
    float x, y, z;
    float theta_an, theta_gn = 0;
    float theta_n;
    if (IMU.accelerationAvailable())
    {
        IMU.readAcceleration(x, y, z);
        theta_an = atan(y / z) * 180 / M_PI; // might need to change the axis later
    }

    if (IMU.gyroscopeAvailable())
    {
        IMU.readGyroscope(x, y, z);
        theta_gn += x * (1 / IMU.gyroscopeSampleRate());
    }

    theta_n = k * (old_theta_n + theta_gn) + (1 - k) * theta_an;
    return theta_n;
}


// Compute the PID output
int computePID(float setpoint, float currentValue) {
  float output = 0;
  //   currentTime = micros();
  //   dt = (currentTime - lastTime) / 1000000.0;  // Time difference in seconds
  // lastTime = currentTime;
  dt = (float) (micros() - currentTime) / 1000000;  // gets time for ∆t
  currentTime = micros();  // sets new current time

  float error = setpoint - currentValue;

  float proportional = Kp * error;

  integral += Ki * dt * (error + previousError) / 2.0;
  integral = constrain(integral, -100, 100);  // Example limit


  // Derivative term (rate of change of error)
  float derivative = (error - previousError) / dt;
  if (theta_n - old_theta_n > 0.1 || theta_n - old_theta_n < -0.1) 
        derivative = -Kd * (theta_n - old_theta_n)/dt; // computes the derivative error
    else{ 
        derivative = 0; // filters out noise
    }
    
    output = K_mast * ( proportional + integral + derivative );    // computes sum of error
    output = constrain(output, -1000, 1000);  // limits output of PID to limits of PWM
    
    previousError = error; // update the previous error

  return (int) output;
}

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

void setup() {
    // IMU and serial setup
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Started");

    if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1);
    }
 
  Serial.setTimeout(10);
}

void loop() {
  float setpoint = 0;  // Target value (e.g., target angle or position)

  theta_n = getAngle(old_theta_n); // angles 
  old_theta_n = theta_n;

  keyboard_test();

  // Get PID output
  float output = (computePID(setpoint, theta_n));
  float motorOutput = constrain(map(abs(output), 0, 1000, 50, 255), 50, 255);

  if (output < 0) {
        forward_slow(motorOutput, motorOutput);
    } else if (output > 0) {
        backward_slow((motorOutput), (motorOutput));
    } else {
        // If exactly zero, apply full forward control
        forward_slow(255, 255);
    }

  // Use the output value (e.g., control motors, set speed, etc.)
  
  Serial.print(theta_n);
  Serial.print("\t");
  Serial.print(Kp);
  Serial.print("\t");
  Serial.print(Ki);
  Serial.print("\t");
  Serial.print(Kd);
  Serial.print("\t");
  Serial.println(output);
      

  delay(100);  // Add a small delay to simulate real-time control loop
}
