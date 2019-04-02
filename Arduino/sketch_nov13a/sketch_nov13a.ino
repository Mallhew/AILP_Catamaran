#include <Servo.h>

#define RIGHT_PROPELLER_PIN 11
#define LEFT_PROPELLER_PIN 10
#define CH3_PIN 3
#define CH2_PIN 6
#define CH1_PIN 9
#define LEFT_THRUSTER 4
#define RIGHT_THRUSTER 5
#define IN1 7
#define IN2 8
#define JOYSTICK_MAX 1900
#define JOYSTICK_MIN 1000
#define JOYSTICK_MID 1450

int ch3;
int ch2;
int ch1;

Servo rightPropeller;
Servo leftPropeller;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  rightPropeller.attach(RIGHT_PROPELLER_PIN);
  leftPropeller.attach(LEFT_PROPELLER_PIN);
  pinMode(RIGHT_THRUSTER, OUTPUT);
  pinMode(LEFT_THRUSTER, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  ch3 = constrain(pulseIn(CH3_PIN, HIGH), JOYSTICK_MIN, JOYSTICK_MAX);
  ch2 = constrain(pulseIn(CH2_PIN, HIGH), JOYSTICK_MIN, JOYSTICK_MAX);
  ch1 = pulseIn(CH1_PIN, HIGH);
  leftPropeller.writeMicroseconds(ch3);
  rightPropeller.writeMicroseconds(ch2);
  if(ch1 < 1300 && ch1 > 500){
    runThruster(1, map(ch1, 1000, 1900, -255, 255));
  } else if(ch1 > 1700){
    runThruster(2, map(ch1, 1000, 1900, -255, 255));
  } else {
    runThruster(0, 0);
  }
  Serial.print("Signal: ");
  Serial.println(ch1);
  
}

void runThruster(int mode, int power){
  if(mode == 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    Serial.println("Sending: nothing");
  }
  else if(mode == 1){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(RIGHT_THRUSTER, abs(power));
    Serial.print("Sending: ");
    Serial.print(power);
    Serial.print(", ");
    Serial.println(mode);
  }
  else if(mode == 2){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(RIGHT_THRUSTER, abs(power));
    Serial.print("Sending: ");
    Serial.print(power);
    Serial.print(", ");
    Serial.println(mode);
  }
}
