#define AIN1 2
#define AIN2 3
#define BIN2 4
#define BIN1 5



void setup() {
  // put your setup code here, to run once:
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

}

// foward fast decay

void forward(int pwm){
  analogWrite(AIN1, pwm);   
  digitalWrite(AIN2, LOW);   
  analogWrite(BIN1, pwm); 
  digitalWrite(BIN2, LOW); 

}


// reverse fast decay
void backward(int pwm) {  
    digitalWrite(AIN1, LOW);   
    analogWrite(AIN2, pwm);   
    digitalWrite(BIN1, LOW); 
    analogWrite(BIN2, pwm);  
}

// 25%, 75% of maximum rpm in opposite directions of each other

//left forward, right backward
// A left, B right
void lfw_rbw(int pwm){ 
  analogWrite(AIN1, pwm); 
  digitalWrite(AIN2, LOW);
  analogWrite(BIN2, pwm);
  digitalWrite(BIN1, LOW);
}

//right foward, left backward
void rfw_lbw(int pwm){ 
  analogWrite(AIN2, pwm); 
  digitalWrite(AIN1, LOW);
  analogWrite(BIN1, pwm);
  digitalWrite(BIN2, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Forward direction for both motors
  // PWM speed (0-255)

  // 25% duty cycle; pwm = 64
  // 50% duty cycle; pwm = 127
  // 75% duty cycle; pwm = 191
  // 100% duty cycle; pwm = 255

  // forward(127);
  // delay(3000);


  // backward(127);
  // delay(3000);

  // lfw_rbw(191);
  // delay(3000);

  rfw_lbw(255);
  delay(3000);

  //delay(10);


}
