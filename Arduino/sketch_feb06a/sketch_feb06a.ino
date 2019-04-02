#include <Servo.h>

#define RIGHT_PROPELLER_PIN 11
#define LEFT_PROPELLER_PIN 10
#define CH3_PIN 50
#define CH2_PIN 48
#define CH1_PIN 46
#define LEFT_THRUSTER 8
#define RIGHT_THRUSTER 5
#define IN1 3
#define IN2 4
#define IN3 6
#define IN4 7
#define JOYSTICK_MAX 1900
#define JOYSTICK_MIN 1000
#define JOYSTICK_MID 1450

int ch3;
int ch2;
int ch1;

char readValue[32];

Servo rightPropeller;
Servo leftPropeller;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  rightPropeller.attach(RIGHT_PROPELLER_PIN);
  leftPropeller.attach(LEFT_PROPELLER_PIN);
  pinMode(RIGHT_THRUSTER, OUTPUT);
  pinMode(LEFT_THRUSTER, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  leftPropeller.writeMicroseconds(2000);
  rightPropeller.writeMicroseconds(2000);
  delay(1000);
  leftPropeller.writeMicroseconds(0);
  leftPropeller.writeMicroseconds(0);
  runThruster(1, 1023);
  delay(1000);
  leftPropeller.writeMicroseconds(2000);
  rightPropeller.writeMicroseconds(2000);
}

void runThruster(int mode, int power){
  if(mode == 0){
    Serial.println(0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  else if(mode == 1){
    Serial.println(1);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(RIGHT_THRUSTER, abs(power));
    analogWrite(LEFT_THRUSTER, abs(power));
    Serial.println(abs(power));
  }
  else if(mode == 2){
    Serial.println(2);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(RIGHT_THRUSTER, abs(power));
    analogWrite(LEFT_THRUSTER, abs(power));
    Serial.println(abs(power));
  }
}
