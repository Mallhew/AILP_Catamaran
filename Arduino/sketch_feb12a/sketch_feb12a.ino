char arrayc[64];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(100000);
  Serial1.begin(100000);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial1.readBytes(arrayc, 64);
  Serial.println(arrayc);
}
