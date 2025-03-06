#define AIN1 2
#define AIN2 3
#define BIN2 4
#define BIN1 5

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
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

}

// foward fast decay

void forward(int pwm1, int pwm2){
  analogWrite(AIN1, pwm1);   
  digitalWrite(AIN2, LOW);   
  analogWrite(BIN1, pwm2); 
  digitalWrite(BIN2, LOW); 
}

// reverse fast decay
void backward(int pwm1, int pwm2) {  
    digitalWrite(AIN1, LOW);   
    analogWrite(AIN2, pwm1);   
    digitalWrite(BIN1, LOW); 
    analogWrite(BIN2, pwm2);  
}

//left forward, right backward
// A left, B right
void lfw_rbw(int pwm1, int pwm2){ 

  // Right motor forward
  analogWrite(AIN1, pwm1);  
  digitalWrite(AIN2, LOW);
  
  analogWrite(BIN2, pwm2);
  digitalWrite(BIN1, LOW);

 
}

//right foward, left backward
void rfw_lbw(int pwm1, int pwm2){ 
  analogWrite(AIN2, pwm1); 
  digitalWrite(AIN1, LOW);

  analogWrite(BIN1, pwm2);
  digitalWrite(BIN2, LOW);
}

void loop() {

  // //---------------------------------------------------------------

  // rfw_lbw(rpm_to_pwm_left(420), rpm_to_pwm_right(437));
  // delay(3000); // wait for 5 sec


  rfw_lbw(rpm_to_pwm_left(420), rpm_to_pwm_right(437));
  delay(3000); // wait for 5 sec

  // forward(rpm_to_pwm_left(420 * 3 / 4), rpm_to_pwm_right(437 * 3 / 4));
  // delay(3000); 

  // forward(rpm_to_pwm_left(420 / 2), rpm_to_pwm_right(437 / 2));
  // delay(3000); 

  // forward(rpm_to_pwm_left(420 / 4), rpm_to_pwm_right(437 / 4));
  // delay(3000);

  // //---------------------------------------------------------------

  // backward(rpm_to_pwm_left(420 * 3 / 4), rpm_to_pwm_right(437 * 3 / 4));
  // delay(3000); 

  // backward(rpm_to_pwm_left(420 / 4), rpm_to_pwm_right(437 / 4));
  // delay(3000);

  //---------------------------------------------------------------
  
  // lfw_rbw(rpm_to_pwm_left(420*3/4), rpm_to_pwm_right(437*3/4));
  // delay(3000);

  // lfw_rbw(rpm_to_pwm_left(420 / 4), rpm_to_pwm_right(437 / 4));
  // delay(3000);

  //---------------------------------------------------------------

  // rfw_lbw(rpm_to_pwm_left(420*3/4), rpm_to_pwm_right(437*3/4));
  // delay(3000);
  
  // rfw_lbw(rpm_to_pwm_left(420/4), rpm_to_pwm_right(437/4));
  // delay(3000);

  //delay(10);


}
