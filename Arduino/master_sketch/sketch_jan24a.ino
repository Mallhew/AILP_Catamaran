#include <Servo.h>
#include <SBUS.h>
#include <NMEAGPS.h>
#include <Streamers.h>
#include <GPSport.h>
#include <EEPROM.h>
#include <Wire.h>

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
#define Addr 0x1E

int ch5;
int ch4;
int ch3;
int ch2;
int ch1;

float lat;
float lon;
float rLat[16];
float rLon[16];
bool buoyDeployed[16];

bool gpsLocGot = false;

char readValue[32];

static NMEAGPS  gps;
static gps_fix  fix;

String mode = "auto";

Servo rightPropeller;
Servo leftPropeller;
SBUS sbus(Serial2);

float compassHeading(){
    int x, y, z;

  // Initiate communications with compass
  Wire.beginTransmission(Addr);
  Wire.write(byte(0x03));       // Send request to X MSB register
  Wire.endTransmission();

  Wire.requestFrom(Addr, 6);    // Request 6 bytes; 2 bytes per axis
  if(Wire.available() <=6) {    // If 6 bytes available
    x = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();
  }
  // If compass module lies flat on the ground with no tilt,
  // just x and y are needed for calculation
  float heading=atan2(x, y)/0.0174532925;
  if(heading < 0) heading+=360;
  heading=360-heading; // N=0/360, E=90, S=180, W=270
  return heading;
}

double lonToX(double lon){
  return 256 * (0.5 + lon / 360);
}

double latToY(double lat){
  double siny = sin(lat * PI / 180);

  // Truncating to 0.9999 effectively limits latitude to 89.189. This is
  // about a third of a tile past the edge of the world tile.
  siny = min(max(siny, -0.9999), 0.9999);

  return 256 * (0.5 - log((1 + siny) / (1 - siny)) / (4 * PI));
}

double getTargetHeading(double cLon, double cLat, double tLon, double tLat){
  cLon = lonToX(cLon);
  cLat = latToY(cLat);
  tLon = lonToX(tLon);
  tLat = latToY(tLat);
  double northernX = cLon;
  double northernY = latToY(85.05);
  double vectOneX = 0;
  double vectOneY = northernY - cLat;
  double vectTwoX = tLon - cLon;
  double vectTwoY = tLat - cLat;

  if(tLon - cLon > 0){
    return (acos((vectOneX * vectTwoX + vectOneY * vectTwoY) / (sqrt(pow(vectOneX, 2) + pow(vectOneY, 2)) * sqrt(pow(vectTwoX, 2) + pow(vectTwoY, 2)))))*360/2/PI;
  } else {
    return (2*PI - acos((vectOneX * vectTwoX + vectOneY * vectTwoY) / (sqrt(pow(vectOneX, 2) + pow(vectOneY, 2)) * sqrt(pow(vectTwoX, 2) + pow(vectTwoY, 2)))))*360/2/PI;
  }
}

bool isTurnCCW(float hdg, double newHdg) { // should a new heading turn left ie. CCW?
   const double diff = newHdg - hdg;        // CCW = counter-clockwise ie. left
   return diff > 0 ? diff > 180 : diff >= -180;
}

void gpsSetup() {
  DEBUG_PORT.begin(9600);
  while (!DEBUG_PORT)
    ;

  DEBUG_PORT.print( F("NMEA.INO: started\n") );
  DEBUG_PORT.print( F("  fix object size = ") );
  DEBUG_PORT.println( sizeof(gps.fix()) );
  DEBUG_PORT.print( F("  gps object size = ") );
  DEBUG_PORT.println( sizeof(gps) );
  DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );

#ifndef NMEAGPS_RECOGNIZE_ALL
#error You must define NMEAGPS_RECOGNIZE_ALL in NMEAGPS_cfg.h!
#endif

#ifdef NMEAGPS_INTERRUPT_PROCESSING
#error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

#if !defined( NMEAGPS_PARSE_GGA ) & !defined( NMEAGPS_PARSE_GLL ) & \
      !defined( NMEAGPS_PARSE_GSA ) & !defined( NMEAGPS_PARSE_GSV ) & \
      !defined( NMEAGPS_PARSE_RMC ) & !defined( NMEAGPS_PARSE_VTG ) & \
      !defined( NMEAGPS_PARSE_ZDA ) & !defined( NMEAGPS_PARSE_GST )

  DEBUG_PORT.println( F("\nWARNING: No NMEA sentences are enabled: no fix data will be displayed.") );

#else
  if (gps.merging == NMEAGPS::NO_MERGING) {
    DEBUG_PORT.print  ( F("\nWARNING: displaying data from ") );
    DEBUG_PORT.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
    DEBUG_PORT.print  ( F(" sentences ONLY, and only if ") );
    DEBUG_PORT.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
    DEBUG_PORT.println( F(" is enabled.\n"
                          "  Other sentences may be parsed, but their data will not be displayed.") );
  }
#endif

  DEBUG_PORT.print  ( F("\nGPS quiet time is assumed to begin after a ") );
  DEBUG_PORT.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
  DEBUG_PORT.println( F(" sentence is received.\n"
                        "  You should confirm this with NMEAorder.ino\n") );

  trace_header( DEBUG_PORT );
  DEBUG_PORT.flush();

  gpsPort.begin( 38400 );
}

static void GPSloop()
{
  if(gps.available( gpsPort )){
    fix = gps.read();
    lon = (double) fix.longitudeL() / (double) 10000000;
    lat = (double) fix.latitudeL() / (double) 10000000;
    gpsLocGot = fix.valid.location;
    trace_all( DEBUG_PORT, gps, fix );
    Serial.print(lon);
    Serial.print(", ");
    Serial.println(lat);
    Serial.print(getTargetHeading(lon, lat, rLon[0], rLat[0]));
    Serial.print(", ");
    Serial.println(compassHeading());
  }

} // GPSloop

void setup() {
  // put your setup code here, to run once:
  sbus.begin();
  gpsSetup();
  Wire.begin();
  
  // Set operating mode to continuous
  Wire.beginTransmission(Addr); 
  Wire.write(byte(0x02));
  Wire.write(byte(0x00));
  Wire.endTransmission();
  Serial.println("here");
  
  rightPropeller.attach(RIGHT_PROPELLER_PIN);
  leftPropeller.attach(LEFT_PROPELLER_PIN);
  
  pinMode(RIGHT_THRUSTER, OUTPUT);
  pinMode(LEFT_THRUSTER, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  rLat[0] = 21.284667;
  rLon[0] = -157.823889;
}

ISR(TIMER2_COMPA_vect)
{
  sbus.process();
}

void loop() {
  // put your main code here, to run repeatedly:
  GPSloop();
  ch5 = sbus.getNormalizedChannel(5);
  ch4 = map(sbus.getNormalizedChannel(4), -83, 83, 1000, 2000); // lstick x-axis
  ch3 = map(sbus.getNormalizedChannel(3), -83, 83, 1000, 2000); // rstick y-axis
  ch2 = map(sbus.getNormalizedChannel(2), -83, 83, 1000, 2000); // rstick x-axis
  ch1 = map(sbus.getNormalizedChannel(1), -83, 83, 1000, 2000); // lstick y-axis

  if(sbus.getNormalizedChannel(5) > 0){
    mode = "auto";
  } else if (sbus.getNormalizedChannel(5) < 0) {
    mode = "auto";
  } else {
    mode = "dead";
  }
  Serial.println(mode);
  if (mode == "manual") {
    Serial.print(ch4);
    Serial.print(", ");
    Serial.print(ch3);
    Serial.print(", ");
    Serial.print(ch2);
    Serial.print(", ");
    Serial.print(ch1);
    Serial.print(", ");
    Serial.println(compassHeading());
    
    if (ch1 > 1650 || (ch1 < 1300 && ch1 > 900)) {
      writeLeftMotor(constrain(ch1, 1000, 2000));
    }
    else {
      writeLeftMotor(1500);
      Serial.println("this");
    }
    if (ch3 > 1650 || (ch3 < 1300 && ch3 > 900)) {
      writeRightMotor(constrain(ch3, 1000, 2000));
      Serial.println("not");
    }
    else {
      writeRightMotor(1500);
      Serial.println("this");
    }
    if (ch4 < 1300 && ch4 > 900) {
      runThruster(1, constrain(map(ch4, 1300, 1000, 0, 1023), 0, 1023));
    } else if (ch4 > 1700) {
      runThruster(2, constrain(map(ch4, 1700, 2000, 0, 1023), 0, 1023));
    } else {
      runThruster(0, 0);
    }
  }
  else if (mode == "auto") {
    if(!buoyDeployed[0]);
    Serial.println(abs(compassHeading() - getTargetHeading(lon, lat, rLon[0], rLat[0])));
    if(abs(compassHeading() - getTargetHeading(lon, lat, rLon[0], rLat[0])) > 5 && gpsLocGot && (abs(lon - rLon[0]) > .001 || abs(lat - rLat[0]) > .001))
    {
      if(isTurnCCW(compassHeading(), getTargetHeading(lon, lat, rLon[0], rLat[0])))
      {
        writeRightMotor(1700); 
        writeLeftMotor(1300); 
        Serial.println("Turn CCW");
      }
      else
      {
        writeRightMotor(1300); 
        writeLeftMotor(1700); 
        Serial.println("Turn CW");
      }
    }
    else if((abs(lon - rLon[0]) > .0001 || abs(lat - rLat[0]) > .0001) && gpsLocGot){
      writeLeftMotor(1700);
      writeRightMotor(1700);
      Serial.println("Go forth");
    }
    else if(!gpsLocGot){
      Serial.println("ERROR: Could not get GPS, can it see the sun?");
      Serial.println(gpsLocGot);
      writeLeftMotor(1500);
      writeRightMotor(1500);
    }
    else{
      writeLeftMotor(1500);
      writeRightMotor(1500);
      Serial.println("There");
      mode == "manual";
    }
  }
  else if (mode == "dead"){
      writeLeftMotor(1500);
      writeRightMotor(1500);
  }
}

void runThruster(int mode, int power) {
  if (mode == 0) {
    Serial.println(0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  else if (mode == 1) {
    Serial.println(1);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(RIGHT_THRUSTER, abs(power));
    analogWrite(LEFT_THRUSTER, abs(power));
    Serial.println(abs(power));
  }
  else if (mode == 2) {
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

void writeLeftMotor(int power){
  leftPropeller.writeMicroseconds(map(power, 1000, 2000, 2000, 1000));
}
void writeRightMotor(int power){
  rightPropeller.writeMicroseconds(map(power, 1000, 2000, 1000, 2000));
}
