
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
#define HARD_OFFSET -160

// Following defines are constant coordinates in a range of 
#define HOME_LAT latToY(21.283158)
#define HOME_LON lonToX(-157.824616)

#define ALA_WAI_TOP_LAT1 latToY(21.285056)
#define ALA_WAI_TOP_LON1 lonToX(-157.820440)
#define ALA_WAI_TOP_LAT2 latToY(21.283057)
#define ALA_WAI_TOP_LON2 lonToX(-157.822666)

#define ALA_WAI_CURB_LAT1 latToY(21.283277)
#define ALA_WAI_CURB_LON1 lonToX(-157.826319)
#define ALA_WAI_CURB_LAT2 latToY(21.282987)
#define ALA_WAI_CURB_LON2 lonToX(-157.822108)

#define ALA_WAI_RIGHT_LAT1 latToY(21.275039)
#define ALA_WAI_RIGHT_LON1 lonToX(-157.817447)
#define ALA_WAI_RIGHT_LAT2 latToY(21.282967)
#define ALA_WAI_RIGHT_LON2 lonToX(-157.827076)

#define ALA_WAI_LEFT_LAT1 latToY(21.283027)
#define ALA_WAI_LEFT_LON1 lonToX(-157.826137)
#define ALA_WAI_LEFT_LAT2 latToY(21.288355)
#define ALA_WAI_LEFT_LON2 lonToX(-157.832998)

#define CURB_END_LAT latToY(21.283257)
#define CURB_END_LON lonToX(-157.822467)

#define CENTRAL_HUB_LAT latToY(21.283027)
#define CENTRAL_HUB_LON lonToX(-157.826732)


int ACC_Data0, ACC_Data1, ACC_Data2, ACC_Data3, ACC_Data4, ACC_Data5;
 
int MAG_Data0, MAG_Data1, MAG_Data2, MAG_Data3, MAG_Data4, MAG_Data5;
 
int Ax, Ay, Az, Mx, My, Mz;
 
float Xm_print, Ym_print, Zm_print;
float Xm_off, Ym_off, Zm_off, Xm_cal, Ym_cal, Zm_cal, Norm_m;
 
float Xa_print, Ya_print, Za_print;
float Xa_off, Ya_off, Za_off, Xa_cal, Ya_cal, Za_cal, Norm_a;
 
const float alpha = 0.15;
float fXa = 0;
float fYa = 0;
float fZa = 0;
float fXm = 0;
float fYm = 0;
float fZm = 0;
float pitch, pitch_print, roll, roll_print, Heading;
float fXm_comp, fYm_comp;


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

String mode = "manual";

Servo rightPropeller;
Servo leftPropeller;
SBUS sbus(Serial2);


float minX = 0;
float maxX = 0;
float minY = 0;
float maxY = 0;
float offX = 0;
float offY = 0;
bool calibratedCompass = false;
float xTarget = 0;
float yTarget = 0;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


float compassHeading(){
    Wire.beginTransmission(0x19);
  Wire.write(0x28);
  Wire.endTransmission();
  Wire.requestFrom(0x19, (byte)1);
  ACC_Data0 = Wire.read();
 
  Wire.beginTransmission(0x19);
  Wire.write(0x29);
  Wire.endTransmission();
  Wire.requestFrom(0x19, (byte)1);
  ACC_Data1 = Wire.read();
 
  Wire.beginTransmission(0x19);
  Wire.write(0x2A);
  Wire.endTransmission();
  Wire.requestFrom(0x19, (byte)1);
  ACC_Data2 = Wire.read();
 
  Wire.beginTransmission(0x19);
  Wire.write(0x2B);
  Wire.endTransmission();
  Wire.requestFrom(0x19, (byte)1);
  ACC_Data3 = Wire.read();
 
  Wire.beginTransmission(0x19);
  Wire.write(0x2C);
  Wire.endTransmission();
  Wire.requestFrom(0x19, (byte)1); 
  ACC_Data4 = Wire.read();
 
  Wire.beginTransmission(0x19);
  Wire.write(0x2D);
  Wire.endTransmission();
  Wire.requestFrom(0x19, (byte)1);
  ACC_Data5 = Wire.read();
 
  Ax = (int16_t)(ACC_Data1 << 8) | ACC_Data0;
  Ay = (int16_t)(ACC_Data3 << 8) | ACC_Data2;
  Az = (int16_t)(ACC_Data5 << 8) | ACC_Data4;
 
  Xa_off = Ax/16.0 + 14.510699; // add/subtract bias calculated by Magneto 1.2. Search the web and you will
  Ya_off = Ay/16.0 - 17.648453; // find this Windows application.  It works very well to find calibrations
  Za_off = Az/16.0 -  6.134981;
  Xa_cal =  1.006480*Xa_off - 0.012172*Ya_off + 0.002273*Za_off; // apply scale factors calculated by Magneto1.2
  Ya_cal = -0.012172*Xa_off + 0.963586*Ya_off - 0.006436*Za_off;
  Za_cal =  0.002273*Xa_off - 0.006436*Ya_off + 0.965482*Za_off;
  Norm_a = sqrt(Xa_cal * Xa_cal + Ya_cal * Ya_cal + Za_cal * Za_cal); //original code did not appear to normalize, and this seems to help
  Xa_cal = Xa_cal / Norm_a;
  Ya_cal = Ya_cal / Norm_a;
  Za_cal = Za_cal / Norm_a;
 
  Ya_cal = -1.0 * Ya_cal;  // This sign inversion is needed because the chip has +Z up, while algorithms assume +Z down
  Za_cal = -1.0 * Za_cal;  // This sign inversion is needed for the same reason and to preserve right hand rotation system
 
// Low-Pass filter accelerometer
  fXa = Xa_cal * alpha + (fXa * (1.0 - alpha));
  fYa = Ya_cal * alpha + (fYa * (1.0 - alpha));
  fZa = Za_cal * alpha + (fZa * (1.0 - alpha));
 
 
  Wire.beginTransmission(0x1E);
  Wire.write(0x03);
  Wire.endTransmission();
 
  Wire.requestFrom(0x1E, (byte)6);
  MAG_Data0 = Wire.read();
  MAG_Data1 = Wire.read();
  MAG_Data2 = Wire.read();
  MAG_Data3 = Wire.read();
  MAG_Data4 = Wire.read();
  MAG_Data5 = Wire.read();
 
  Mx = (int16_t)(MAG_Data0 << 8) | MAG_Data1;
  Mz = (int16_t)(MAG_Data2 << 8) | MAG_Data3;
  My = (int16_t)(MAG_Data4 << 8) | MAG_Data5;
 
  Xm_off = Mx*(100000.0/1100.0) -   617.106577; // Gain X [LSB/Gauss] for selected sensor input field range (1.3 in these case)
  Ym_off = My*(100000.0/1100.0) -  3724.617984; // Gain Y [LSB/Gauss] for selected sensor input field range
  Zm_off = Mz*(100000.0/980.0 ) - 16432.772031;  // Gain Z [LSB/Gauss] for selected sensor input field range
  Xm_cal =  0.982945*Xm_off + 0.012083*Ym_off + 0.014055*Zm_off; // same calibration program used for mag as accel.
  Ym_cal =  0.012083*Xm_off + 0.964757*Ym_off - 0.001436*Zm_off;
  Zm_cal =  0.014055*Xm_off - 0.001436*Ym_off + 0.952889*Zm_off;
  Norm_m = sqrt(Xm_cal * Xm_cal + Ym_cal * Ym_cal + Zm_cal * Zm_cal); // original code did not appear to normalize  This seems to help
  Xm_cal = Xm_cal / Norm_m;
  Ym_cal = Ym_cal / Norm_m;
  Zm_cal = Zm_cal / Norm_m;
 
  Ym_cal = -1.0 * Ym_cal;  // This sign inversion is needed because the chip has +Z up, while algorithms assume +Z down
  Zm_cal = -1.0 * Zm_cal;  // This sign inversion is needed for the same reason and to preserve right hand rotation system
 
// Low-Pass filter magnetometer
  fXm = Xm_cal * alpha + (fXm * (1.0 - alpha));
  fYm = Ym_cal * alpha + (fYm * (1.0 - alpha));
  fZm = Zm_cal * alpha + (fZm * (1.0 - alpha));
 
 
// Pitch and roll
  pitch = asin(fXa);
  roll = -asin(fYa);
  pitch_print = pitch*180.0/M_PI;
  roll_print = roll*180.0/M_PI;
 
// Tilt compensated magnetic sensor measurements
  fXm_comp = fXm*cos(pitch) + fZm*sin(pitch);
  fYm_comp = fYm*cos(roll) - fZm*sin(roll);
 
  Heading = (atan2(-fYm_comp,fXm_comp)*180.0)/M_PI - 160;
 
  if (Heading < 0)
  Heading += 360;
  return Heading;
 
}

void compassSetup(){
  Wire.begin();
 
  Wire.beginTransmission(0x19); // Set accel
  Wire.write(0x20);             // CTRL_REG1_A register
  Wire.write(0x47);             // 50 Hz, normal power, all 3 axis enabled
  Wire.endTransmission();
 
  Wire.beginTransmission(0x19); // Set accel
  Wire.write(0x23);             // CTRL_REG4_A register
  Wire.write(0x08);             // continuous update, littleendian, 2g, high resolution, 4-wire spi
  Wire.endTransmission();
 
  Wire.beginTransmission(0x1E); // set Mag
  Wire.write(0x00);             // CRA_REG_M register
  Wire.write(0x14);             // 30 Hz min data output rate
  Wire.endTransmission();
 
  Wire.beginTransmission(0x1E); // Set Mag
  Wire.write(0x02);             // MR_REG_M
  Wire.write(0x00);             // continuous conversion mode
  Wire.endTransmission();
}

double lonToX(double lon){
  return 256 * (0.5 + lon / 360);
}

double xToLon(double x){
  return (x/256 - .5) * 360;
}

double latToY(double lat){
  double siny = sin(lat * PI / 180);

  // Truncating to 0.9999 effectively limits latitude to 89.189. This is
  // about a third of a tile past the edge of the world tile.
  siny = min(max(siny, -0.9999), 0.9999);

  return 256 * (0.5 - log((1 + siny) / (1 - siny)) / (4 * PI));
}

double yToLat(double y){
  return asin((pow(EULER, ((y/256 - .5)*-4*PI)) - 1)/(pow(EULER, ((y/256 - .5)*-4*PI)) + 1)) * 180 / PI;
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
  DEBUG_PORT.begin(115200);
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

void markIntent(float xcoord, float ycoord){
  if(sqrt(pow((CURB_END_LAT-latToY(lat)), 2)+pow((CURB_END_LON-lonToX(lon)), 2)) < .0001414){
    if((ycoord < max(ALA_WAI_CURB_LAT1, ALA_WAI_CURB_LAT2) && ycoord > min(ALA_WAI_CURB_LAT1, ALA_WAI_CURB_LAT2) && xcoord < max(ALA_WAI_CURB_LON1, ALA_WAI_CURB_LON2) && xcoord > min(ALA_WAI_CURB_LON1, ALA_WAI_CURB_LON2)) || (ycoord < max(ALA_WAI_TOP_LAT1, ALA_WAI_TOP_LAT2) && ycoord > min(ALA_WAI_TOP_LAT1, ALA_WAI_TOP_LAT2) && xcoord < max(ALA_WAI_TOP_LON1, ALA_WAI_TOP_LON2) && xcoord > min(ALA_WAI_TOP_LON1, ALA_WAI_TOP_LON2))){
      xTarget = xcoord;
      yTarget = ycoord;
    }
    else{
      xTarget = CENTRAL_HUB_LAT;
      yTarget = CENTRAL_HUB_LON; 
    }
  }
  else if(sqrt(pow((CENTRAL_HUB_LAT-latToY(lat)), 2)+pow((CENTRAL_HUB_LON-lonToX(lon)),2)) < .0001414){
    if(!(ycoord < max(ALA_WAI_TOP_LAT1, ALA_WAI_TOP_LAT2) && ycoord > min(ALA_WAI_TOP_LAT1, ALA_WAI_TOP_LAT2) && xcoord < max(ALA_WAI_TOP_LON1, ALA_WAI_TOP_LON2) && xcoord > min(ALA_WAI_TOP_LON1, ALA_WAI_TOP_LON2))){
      xTarget = xcoord; 
      yTarget = ycoord;
    }
    else{
      xTarget = CURB_END_LAT;
      yTarget = CURB_END_LON; 
    }
  }
  else if(latToY(lat) < max(ALA_WAI_TOP_LAT1, ALA_WAI_TOP_LAT2) && latToY(lat) > min(ALA_WAI_TOP_LAT1, ALA_WAI_TOP_LAT2) && lonToX(lon) < max(ALA_WAI_TOP_LON1, ALA_WAI_TOP_LON2) && lonToX(lon) > min(ALA_WAI_TOP_LON1, ALA_WAI_TOP_LON2)){
    if(ycoord < max(ALA_WAI_TOP_LAT1, ALA_WAI_TOP_LAT2) && ycoord > min(ALA_WAI_TOP_LAT1, ALA_WAI_TOP_LAT2) && xcoord < max(ALA_WAI_TOP_LON1, ALA_WAI_TOP_LON2) && xcoord > min(ALA_WAI_TOP_LON1, ALA_WAI_TOP_LON2)){
      xTarget = xcoord; 
      yTarget = ycoord;
    }
    else{
      xTarget = CURB_END_LAT;
      yTarget = CURB_END_LON; 
    }
  }
  else if(latToY(lat) < max(ALA_WAI_CURB_LAT1, ALA_WAI_CURB_LAT2) && latToY(lat) > min(ALA_WAI_CURB_LAT1, ALA_WAI_CURB_LAT2) && lonToX(lon) < max(ALA_WAI_CURB_LON1, ALA_WAI_CURB_LON2) && lonToX(lon) > min(ALA_WAI_CURB_LON1, ALA_WAI_CURB_LON2)){
    if(ycoord < max(ALA_WAI_CURB_LAT1, ALA_WAI_CURB_LAT2) && ycoord > min(ALA_WAI_CURB_LAT1, ALA_WAI_CURB_LAT2) && xcoord < max(ALA_WAI_CURB_LON1, ALA_WAI_CURB_LON2) && xcoord > min(ALA_WAI_CURB_LON1, ALA_WAI_CURB_LON2)){
      xTarget = xcoord; 
      yTarget = ycoord;
    }
    else if(ycoord < max(ALA_WAI_TOP_LAT1, ALA_WAI_TOP_LAT2) && ycoord > min(ALA_WAI_TOP_LAT1, ALA_WAI_TOP_LAT2) && xcoord < max(ALA_WAI_TOP_LON1, ALA_WAI_TOP_LON2) && xcoord > min(ALA_WAI_TOP_LON1, ALA_WAI_TOP_LON2)){
      xTarget = CURB_END_LAT;
      yTarget = CURB_END_LON; 
    }
    else{
      xTarget = CENTRAL_HUB_LAT;
      yTarget = CENTRAL_HUB_LON;
    }
  }
  else if(latToY(lat) < max(ALA_WAI_LEFT_LAT1, ALA_WAI_LEFT_LAT2) && latToY(lat) > min(ALA_WAI_LEFT_LAT1, ALA_WAI_LEFT_LAT2) && lonToX(lon) < max(ALA_WAI_LEFT_LON1, ALA_WAI_LEFT_LON2) && lonToX(lon) > min(ALA_WAI_LEFT_LON1, ALA_WAI_LEFT_LON2)){
    if(ycoord < max(ALA_WAI_LEFT_LAT1, ALA_WAI_LEFT_LAT2) && ycoord > min(ALA_WAI_LEFT_LAT1, ALA_WAI_LEFT_LAT2) && xcoord < max(ALA_WAI_LEFT_LON1, ALA_WAI_LEFT_LON2) && xcoord > min(ALA_WAI_LEFT_LON1, ALA_WAI_LEFT_LON2)){
      xTarget = xcoord; 
      yTarget = ycoord;
    }
    else{
      xTarget = CENTRAL_HUB_LAT;
      yTarget = CENTRAL_HUB_LON;
    }
  }
  else if(latToY(lat) < max(ALA_WAI_RIGHT_LAT1, ALA_WAI_RIGHT_LAT2) && latToY(lat) > min(ALA_WAI_RIGHT_LAT1, ALA_WAI_RIGHT_LAT2) && lonToX(lon) < max(ALA_WAI_RIGHT_LON1, ALA_WAI_RIGHT_LON2) && lonToX(lon) > min(ALA_WAI_RIGHT_LON1, ALA_WAI_RIGHT_LON2)){
    if(ycoord < max(ALA_WAI_RIGHT_LAT1, ALA_WAI_RIGHT_LAT2) && ycoord > min(ALA_WAI_RIGHT_LAT1, ALA_WAI_RIGHT_LAT2) && xcoord < max(ALA_WAI_RIGHT_LON1, ALA_WAI_RIGHT_LON2) && xcoord > min(ALA_WAI_RIGHT_LON1, ALA_WAI_RIGHT_LON2)){
      xTarget = xcoord; 
      yTarget = ycoord;
    }
    else{
      xTarget = CENTRAL_HUB_LAT;
      yTarget = CENTRAL_HUB_LON;
    }
  }
  else{
    xTarget = xcoord;
    yTarget = ycoord;
  }
  rLat[0] = yToLat(yTarget);
  rLon[0] = xToLon(xTarget);
}

static void GPSloop()
{
  if(gps.available( gpsPort )){
    fix = gps.read();
    lon = (double) fix.longitudeL() / (double) 10000000;
    lat = (double) fix.latitudeL() / (double) 10000000;
    gpsLocGot = fix.valid.location;
    trace_all( DEBUG_PORT, gps, fix );
    Serial.print("GPS: ");
    Serial.print(lon);
    Serial.print(", ");
    Serial.println(lat);
  }
    Serial.print("COMPASS: ");
    Serial.print(getTargetHeading(lon, lat, rLon[0], rLat[0]));
    Serial.print(", ");
    Serial.println(compassHeading());

} // GPSloop


void setup() {
  // put your setup code here, to run once:
  sbus.begin();
  gpsSetup();
  compassSetup();
  
  rightPropeller.attach(RIGHT_PROPELLER_PIN);
  leftPropeller.attach(LEFT_PROPELLER_PIN);
  
  pinMode(RIGHT_THRUSTER, OUTPUT);
  pinMode(LEFT_THRUSTER, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  rLat[16] = HOME_LAT;
  rLon[16] = HOME_LON;
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
    mode = "manual";
  } else {
    mode = "dead";
  }
  Serial.println(mode);
  if (mode == "manual") {
    Serial.print(ch5);
    Serial.print(", ");
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
    
    //if(!buoyDeployed[0]);
    markIntent(rLon[0], rLat[0]);
    Serial.println(abs(compassHeading() - getTargetHeading(lon, lat, rLon[0], rLat[0])) > 5);
    Serial.print("COMPASS:");Serial.println(compassHeading());
    if(abs(compassHeading() - getTargetHeading(lon, lat, rLon[0], rLat[0])) > 5 && gpsLocGot && (abs(lon - rLon[0]) > .0001 || abs(lat - rLat[0]) > .0001))
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
  leftPropeller.writeMicroseconds(map(power, 1000, 2000, 1000, 2000));
}
void writeRightMotor(int power){
  rightPropeller.writeMicroseconds(map(power, 1000, 2000, 1000, 2000));
}
