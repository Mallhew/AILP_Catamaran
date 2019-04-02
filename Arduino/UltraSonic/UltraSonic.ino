#include<NewPing.h>
int PING_PIN=11;
int MAX_DISTANCE=200;
NewPing sonar(PING_PIN,PING_PIN,MAX_DISTANCE);

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
delay(50);
unsigned int uS = sonar.ping();
int distance = uS/US_ROUNDTRIP_CM;
Serial.print("Ping");
Serial.print(distance);
Serial.println("cm");
}
