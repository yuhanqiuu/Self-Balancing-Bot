// #define LEFT1 2
// #define LEFT2 3
// #define RIGHT2 4
// #define RIGHT1 5

#define LEFT1 4  //  LEFT1, RIGHT1 yellow wire
#define LEFT2 5  //  LEFT2, RIGHT2 yellow wire
#define RIGHT1 2 //  LEFT1, RIGHT1 green wire
#define RIGHT2 3 //  LEFT2, RIGHT2 green wire

#include <math.h>

// pwm=7.58e^(7.89E-3*rpm) with screw left wheel
// max rpm = 420
// ()(449,255)
// 
// pwm=8.33e^(7.55E-3*rpm) without screw
// max rpm = 437
//--------------------------------------

float rpm_to_pwm_left(float rpm){
  return 7.58 * exp(7.89E-3 * rpm); // return pwm
}

float rpm_to_pwm_right(float rpm){
  return 8.33 * exp(7.55E-3 * rpm); // return pwm
}


void setup() {
  // put your setup code here, to run once:
    pinMode(LEFT1, OUTPUT);
    pinMode(LEFT2, OUTPUT);
    pinMode(RIGHT1, OUTPUT);
    pinMode(RIGHT2, OUTPUT);

}

// foward fast decay

void forward(int pwm1, int pwm2){
  analogWrite(LEFT1, pwm1);   
  digitalWrite(LEFT2, LOW);   
  analogWrite(RIGHT1, pwm2); 
  digitalWrite(RIGHT2, LOW); 
}


// reverse fast decay
void backward(int pwm1, int pwm2) {  
    digitalWrite(LEFT1, LOW);   
    analogWrite(LEFT2, pwm1);   
    digitalWrite(RIGHT1, LOW); 
    analogWrite(RIGHT2, pwm2);  
}

//left forward, right backward
// A left, B right
void lfw_rbw(int pwm1, int pwm2){ 

  // Right motor forward
  analogWrite(LEFT1, pwm1);  
  digitalWrite(LEFT2, LOW);
  
  analogWrite(RIGHT2, pwm2);
  digitalWrite(RIGHT1, LOW);

 
}

//right foward, left backward
void rfw_lbw(int pwm1, int pwm2){ 
  analogWrite(LEFT2, pwm1); 
  digitalWrite(LEFT1, LOW);

  analogWrite(RIGHT1, pwm2);
  digitalWrite(RIGHT2, LOW);
}

  // slow decay
void backward_slow(int pwm1, int pwm2){
    pwm1 = map(abs(pwm1),0,255,250,0);   
    pwm2 = map(abs(pwm2),0,255,250,0); 
    analogWrite(LEFT2, pwm1);   
    digitalWrite(LEFT1, HIGH); 
  
    analogWrite(RIGHT2, pwm2); 
    digitalWrite(RIGHT1, HIGH); 
  }
  
  void forward_slow(int pwm1, int pwm2) {  
    pwm1 = map(abs(pwm1),0,255,250,0);   
    pwm2 = map(abs(pwm2),0,255,250,0); 
      digitalWrite(LEFT2, HIGH);   
      analogWrite(LEFT1, pwm1);
  
      digitalWrite(RIGHT2, HIGH); 
      analogWrite(RIGHT1, pwm2);  
  }


void loop() {

  int leftpwm = 200; // Example PWM value for left motor
  int rightpwm = 20; // Example PWM value for right motor

    forward_slow(leftpwm, rightpwm); 

    delay(10);


}
