/*
Flight Data Recorder Code Here!
*/

#include <Wire.h>
#include <I2C_Anything.h>
#include <SPI.h>
//#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_GPS.h>

SoftwareSerial mySerial(8, 7);


//~~~~~~~~~~~~~~~~~~~Sensors~~~~~~~~~~~~~~~~~~~~~~~~~
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_GPS GPS(&mySerial);

#define GPSECHO false
/* alignmant
 * X is Forward
 * Z is Up
 * Y is wing span wise
 */

//~~~~~~~~~~~~~~Constant Variables~~~~~~~~~~~~~~~~~~
//File dataFile; // define file to write
long t = 0;
int dt = 0;
float pitch = 0;
float roll = 0;
float allDat[10]; //format: [0Time, 1Pitch, 2Roll, 3Heading, 4PressAlt, 5GPSLong, 6GPSLat,
                  // 7GPSAlt, 8GPSSpeed, 9AutoPilot]
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
long time_GPS = 0;
const byte SLAVE_ADDRESS = 42;

//~~~~~~~~~~~~~~Adjustable Variables~~~~~~~~~~~~~~~~~~
int sampleTime = 50;       // minimum time between samples in ms
float P_CompCoeff = 0.975;  // Pitch complimentary coefficient
float R_CompCoeff = 0.975;  // Roll complimentary coefficient

//~~~~~~~~~~~~~~~~~~~~~~~Setup~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
  Wire.begin();
  Serial.begin(115200); // begin serial communication for debugging
  if (! accel.begin()) {    // start accelerometer
    Serial.println("Couldn't start accelerometer");
    delay(100);
  }
  if (! gyro.begin()) { // start gyroscope
    Serial.println("Couldn't start gyroscope");
    delay(100);
  }
//  if (!SD.begin(10)) { // start SD card
//    Serial.println("Sd initialization failed!");
//    while (1);
//  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.println("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    for (int i = 2; i < 8; i++){  // set pins D2-D7 as input pins for receiver
    pinMode(i, INPUT);
  }
  GPS.begin(9600);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);  // Request updates on antenna status, comment out to keep quiet
  useInterrupt(true);
  delay(1000);
  
  Serial.println("Initialization done.");
  gyro.enableAutoRange(true);
  accel.enableAutoRange(true);
}


//~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~
void updateSensorData(){ // get sensor data over I2C
    sensors_event_t event;
    accel.getEvent(&event); // get acceleration data
    float AX = event.acceleration.x;
    float AY = event.acceleration.y;
    float AZ = event.acceleration.z;
    
    gyro.getEvent(&event);  // get gyro data
    float RX = event.gyro.x;
    float RY = event.gyro.y;
    float RZ = event.gyro.z;

    //~~Pitch Complimentary Filter~~
    pitch += RX * 57.29578 * dt * 0.001;
    pitch = 0.975 * pitch + 0.025 * (atan2(AX, sqrt((AY*AY)+(AZ*AZ)))*57.29578);  //57.295 = 180/pi

    //~~Roll Complimentary Filter~~
    roll += RY * 57.29578 * dt * 0.001;
    roll = 0.975 * roll + 0.025 * (atan2(AY, sqrt((AX*AX)+(AZ*AZ)))*57.29578);  //57.295 = 180/pi
    
    allDat[1] = pitch;
    allDat[2] = roll;
    allDat[5] = dt;

//    bmp.getEvent(&event);
//    allDat[4] = bmp.pressureToAltitude(1015, event.pressure);
//    GPSS();
}
/*
void sdWrite(){   // Write data in allDat to SD card
  dataFile = SD.open("datatest.csv", FILE_WRITE);
  if (dataFile) {
    for(int i = 0; i < 15; i++){
      dataFile.print(allDat[i]); dataFile.print(", ");
    }
    dataFile.println();
  }
  else{
    Serial.println("Error opening file");
  }
  dataFile.close();
}
*/   
void updateReceiverData(){   // get pulse lengths from receiver in ms
  if (pulseIn(3, HIGH) > 1500){
    allDat[9] = 1;
  }
  else{
    allDat[9] = 0;
  }
  
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

void GPSS(){
    if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
  }

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
    time_GPS = (long) GPS.hour * 3600 + (long) GPS.minute * 60 + (long) GPS.seconds;
//    Serial.println(time_GPS);
    if (GPS.fix) {
//      Serial.print("Location: ");
//      Serial.print(GPS.latitudeDegrees, 5);
//      Serial.print(", "); 
//      Serial.print(GPS.longitudeDegrees, 5);
//      Serial.println();
//      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
//      Serial.print("Angle: "); Serial.println(GPS.angle);
//      Serial.print("Altitude: "); Serial.println(GPS.altitude);
//      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
        allDat[0] = time_GPS;
        allDat[5] = GPS.latitudeDegrees;
        allDat[6] = GPS.longitudeDegrees;
        allDat[7] = GPS.speed;
  }
}


void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

//~~~~~~~~~~~~~~~~I2C Communicatcion~~~~~~~~~~~~~~~~
void sendData(){
  Wire.beginTransmission (SLAVE_ADDRESS);
  I2C_writeAnything (allDat);
  Wire.endTransmission ();
}

//~~~~~~~~~~~~~~~~~~~~~Main Loop~~~~~~~~~~~~~~~~~~~~
void loop(){
  if (millis() >= (t + sampleTime)){
    allDat[0] = millis();   // write current time to allDat[0]
    dt = millis() - t;      // time since previous read
    t = millis();           //  reset previous read time to current time


    
    updateSensorData();
    sendData();
    updateReceiverData();
//    sdWrite();

  for (int i=0; i < 10; i++){
    Serial.print(allDat[i]);
    Serial.print(", ");
    }
    Serial.println();
  }
}
