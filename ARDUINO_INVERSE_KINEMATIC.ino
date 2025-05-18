#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial mySerial (2,3);

#define SERVOMIN 112
#define SERVOMAX 500

#define SERVODEGMIN 0
#define SERVODEGMAX 180

#define stepPin 9
#define dirPin 8

#define SERVO_FREQ 50

float D, d1, d, zOffset, R, r;
float a1 = 9, a2 = 7.2, a3 = 10.4, a4 = 10.2, a5 = 14;
float step = 0.01;
float currentStep = 0, targetStep;
float theta2, theta3, theta4, theta5;
float phi1, phi2, phi3, phi4;
float angle1, angle2, angle3, angle4;

int currentAngle[4];
int targetAngle[4];
int interval[] = {10, 8, 3, 1};

int PWM8 = map(30, SERVODEGMIN, SERVODEGMAX, SERVOMIN, SERVOMAX);
int PWM9 = map(60, SERVODEGMIN, SERVODEGMAX, SERVOMIN, SERVOMAX);

void setup() {
  Serial.begin(9600);
  mySerial.begin(115200);
  lcd.init();
  delay(500);
  lcd.backlight();
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  currentAngle[0] = 306;
  currentAngle[1] = 372;
  currentAngle[2] = 346;
  currentAngle[3] = 196;

  pwm.setPWM(0, 0, currentAngle[0]);
  pwm.setPWM(1, 0, currentAngle[1]);
  pwm.setPWM(2, 0, currentAngle[2]);
  pwm.setPWM(3, 0, currentAngle[3]);

  Serial.println("CURRENT");
  Serial.println(currentAngle[0]);
  Serial.println(currentAngle[1]);
  Serial.println(currentAngle[2]);
  Serial.println(currentAngle[3]);
  Serial.println();

  delay(5000);
  Serial.println("Arm Robot Ready");
}

void loop() {
  if (mySerial.available()) {
    char data = mySerial.read();
    if(data == 'A'){
      lcd.setCursor(6,0);
      lcd.print("BOX A");
      Serial.println("STARTING BOX A");
      pickup();
      InverseKinematic_Move(-24, 20, 20, 20);
      simultaneously();
      delay(1000);
      InverseKinematic_Move(-24, 20, 14, 20);
      simultaneously();
      delay(1000);
      InverseKinematic_Move(-24, 20, 12, 24);
      simultaneously();
      delay(1000);
      pwm.setPWM(4, 0, PWM8);
      delay(1000);
      InverseKinematic_Move(-24, 20, 14, 20);
      simultaneously();
      delay(1000);
      InverseKinematic_Move(-24, 20, 20, 20);
      simultaneously();
      delay(1000);
      normal();
      delay(1000);
      Serial.println("DONE BOX A");
      lcd.clear();
    }
    else if(data == 'B'){
      lcd.setCursor(6,0);
      lcd.print("BOX B");
      Serial.println("STARTING BOX B");
      pickup();
      InverseKinematic_Move(-24, 28, 20, 20);
      simultaneously();
      delay(1000);
      InverseKinematic_Move(-24, 28, 12, 20);
      simultaneously();
      delay(1000);
      InverseKinematic_Move(-24, 28, 10, 24);
      simultaneously();
      delay(1000);
      pwm.setPWM(4, 0, PWM8);
      delay(1000);
      InverseKinematic_Move(-24, 28, 12, 20);
      simultaneously();
      delay(1000);
      InverseKinematic_Move(-24, 28, 20, 20);
      simultaneously();
      delay(1000);
      normal();
      delay(1000);
      Serial.println("DONE BOX B");
      lcd.clear();
    }
    else if(data == 'C'){
      lcd.setCursor(6,0);
      lcd.print("BOX C");
      Serial.println("STARTING BOX C");
      pickup();
      InverseKinematic_Move(20, 25, 20, 20);
      simultaneously();
      delay(1000);
      InverseKinematic_Move(20, 25, 14, 20);
      simultaneously();
      delay(1000);
      InverseKinematic_Move(20, 25, 12, 24);
      simultaneously();
      delay(1000);
      pwm.setPWM(4, 0, PWM8);
      delay(1000);
      InverseKinematic_Move(20, 25, 14, 20);
      simultaneously();
      delay(1000);
      InverseKinematic_Move(20, 25, 20, 20);
      simultaneously();
      delay(1000);
      normal();
      delay(1000);
      Serial.println("DONE BOX C");
      lcd.clear();
    }
    else if(data == 'D'){
      lcd.setCursor(6,0);
      lcd.print("BOX D");
      Serial.println("STARTING BOX D");
      pickup();
      InverseKinematic_Move(20, 34, 20, 20);
      simultaneously();
      delay(1000);
      InverseKinematic_Move(20, 34, 12, 20);
      simultaneously();
      delay(1000);
      InverseKinematic_Move(20, 34, 10, 24);
      simultaneously();
      delay(1000);
      pwm.setPWM(4, 0, PWM8);
      delay(1000);
      InverseKinematic_Move(20, 34, 12, 20);
      simultaneously();
      delay(1000);
      InverseKinematic_Move(20, 34, 20, 20);
      simultaneously();
      delay(1000);
      normal();
      delay(1000);
      Serial.println("DONE BOX D");
      lcd.clear();
    }
    else {
      Serial.println(data);
      Serial.println("NO SERIAL DATA RECEIVED");
      delay(3000);
    }
  }
}

void InverseKinematic_Move(int x, int y, int z, int r){
  if (x == 0){
    d1 = y - a1;
    D = r;
    theta2 = 90;
  }
  else if (x > 0){
    d1 = y - a1;
    theta2 = 0;
    D = abs(x);
  }
  else if (x < 0){
    d1 = y - a1;
    theta2 = 180;
    D = abs(x);
  }
  d = D - a5;
  zOffset = z - a2;
  R = sqrt(pow(zOffset,2) + pow(d,2));
  
  phi1 = atan(zOffset/d) * (180/M_PI);
  phi2 = acos((pow(R,2) + pow(a3,2) - pow(a4,2)) / (2*R*a3)) * (180/M_PI);
  theta3 = phi1 + phi2;
  theta4 = acos((pow(a3,2) + pow(a4,2) - pow(R,2)) / (2*a3*a4)) * (180/M_PI);

  phi3 = 180 - theta3 - theta4;
  
  theta5 = 90 + phi3;

  targetStep = d1 / step;
  targetAngle[0] = (map(theta2, SERVODEGMIN, SERVODEGMAX, SERVOMIN, SERVOMAX));
  targetAngle[1] = (map(theta3, SERVODEGMIN, SERVODEGMAX, SERVOMIN, SERVOMAX));
  targetAngle[2] = (map(theta4, SERVODEGMIN, SERVODEGMAX, SERVOMIN, SERVOMAX));
  targetAngle[3] = (map(theta5, SERVODEGMIN, SERVODEGMAX, SERVOMIN, SERVOMAX));
}

void slowServo(int channel){
  if(targetAngle[channel] > currentAngle[channel]){
    pwm.setPWM(channel, 0, currentAngle[channel]++);
    delay(interval[channel]);
  }
  else if(targetAngle[channel] < currentAngle[channel]){
    pwm.setPWM(channel, 0, currentAngle[channel]--);
    delay(interval[channel]);
  }
}

void moveStepper(){
  if(currentStep < targetStep){
    digitalWrite(dirPin,LOW);
    for(float x = 0; x < (targetStep - currentStep); x++) {
      digitalWrite(stepPin,HIGH); 
      delayMicroseconds(500);
      digitalWrite(stepPin,LOW);
      delayMicroseconds(500); 
    }
    currentStep = targetStep;
  }
  else if(currentStep > targetStep){
    digitalWrite(dirPin,HIGH);
    for(float x = 0; x < (currentStep - targetStep); x++) {
      digitalWrite(stepPin,HIGH); 
      delayMicroseconds(500);
      digitalWrite(stepPin,LOW);
      delayMicroseconds(500);
    }
    currentStep = targetStep;
  }
}

void simultaneously(){
  moveStepper();
  while (1) {
    slowServo(3);
    slowServo(0);
    slowServo(1);
    slowServo(2);
    if(currentAngle[0] == targetAngle[0] && currentAngle[1] == targetAngle[1] && currentAngle[2] == targetAngle[2] && currentAngle[3] == targetAngle[3]) {
      break;
    }
  }
}

void pickup(){
  pwm.setPWM(4, 0, PWM8);
  InverseKinematic_Move(0, 45, 20, 20);
  simultaneously();
  delay(1000);
  InverseKinematic_Move(0, 45, 12, 20);
  simultaneously();
  delay(1000);
  InverseKinematic_Move(0, 45, 10, 24);
  simultaneously();
  delay(1000);
  pwm.setPWM(4, 0, PWM9);
  delay(500);
  InverseKinematic_Move(0, 45, 24, 15);
  simultaneously();
  delay(1000);
}

void putdown(){

}

void normal(){
  InverseKinematic_Move(0, 10, 24, 15);
  simultaneously();
}