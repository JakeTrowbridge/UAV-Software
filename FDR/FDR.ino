/* alignmant
 * X is Forward
 * Z is Up
 * Y is wing span wise
 */

#include <Wire.h>
#include <I2C_Anything.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>
#include <TinyGPS++.h>
#include "SdFat.h"

//~~~~~~~~~~~~~~~~~~~Sensors~~~~~~~~~~~~~~~~~~~~~~~~~
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
TinyGPSPlus gps;
SdFat sd;
SdFile file;

//~~~~~~~~~~~~~~Constant Variables~~~~~~~~~~~~~~~~~~
//File dataFile; // define file to write
long t = 0;
int dt = 0;
float AX;
float AY;
float AZ;
float RX;
float RY;
float RZ;

float pitch = 0;
float roll = 0;
float allDat[10]; //format: [0Time, 1Pitch, 2Roll, 3Heading, 4PressAlt, 5AutoPilot, 6GPSLat, 7GPSLong,
                  // 8GPSAlt, 9GPSSpeed, ]
                  
long time_GPS = 0;
const byte SLAVE_ADDRESS = 42;
static const int RXPin = 7, TXPin = 8;
static const uint32_t GPSBaud = 9600;
SoftwareSerial ss(RXPin, TXPin);
const uint8_t chipSelect = SS;
#define FILE_BASE_NAME "FDR"
#define error(msg) sd.errorHalt(F(msg))

//~~~~~~~~~~~~~~Adjustable Variables~~~~~~~~~~~~~~~~~~
int sampleTime = 200;       // minimum time between samples in ms
//float P_CompCoeff = 0.975;  // Pitch complimentary coefficient
//float R_CompCoeff = 0.975;  // Roll complimentary coefficient

//~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~
void updateSensorData(){ // get sensor data over I2C
    sensors_event_t event;
    accel.getEvent(&event); // get acceleration data
    AX = event.acceleration.x;
    AY = event.acceleration.y;
    AZ = event.acceleration.z;
    
    gyro.getEvent(&event);  // get gyro data
    RX = event.gyro.x;
    RY = event.gyro.y;
    RZ = event.gyro.z;

    mag.getEvent(&event);   // get magnotometer data
    float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / 3.141592;
  
    // Normalize to 0-360
    if (heading < 0){
      heading = 360 + heading;
    }
    allDat[3] = allDat[3] * 0.025 + heading * 0.975;
    
    //~~Pitch Complimentary Filter~~
    pitch += RX * 57.29578 * dt * 0.001;
    pitch = 0.975 * pitch + 0.025 * (atan2(AX, sqrt((AY*AY)+(AZ*AZ)))*57.29578);  //57.295 = 180/pi

    //~~Roll Complimentary Filter~~
    roll += RY * 57.29578 * dt * 0.001;
    roll = 0.975 * roll + 0.025 * (atan2(AY, sqrt((AX*AX)+(AZ*AZ)))*57.29578);  //57.295 = 180/pi
    
    allDat[1] = pitch;
    allDat[2] = roll;

    bmp.getEvent(&event);
    allDat[4] = bmp.pressureToAltitude(1015, event.pressure);

    GPS();

    autopilot();
}

void sendData(){
  Wire.beginTransmission (SLAVE_ADDRESS);
  I2C_writeAnything (allDat);
  Wire.endTransmission ();
}

void autopilot(){ 
  if(pulseIn((3), HIGH) > 1500){
    allDat[5] = 1;
  }
  else{
    allDat[5] = 0;
  }
}

void GPS(){
  while (ss.available() > 0){
    if (gps.encode(ss.read())){
      allDat[6] = gps.location.lat();
      allDat[7] = gps.location.lng();
      allDat[8] = gps.altitude.meters();
      allDat[9] = gps.speed.kmph();
    }
  }
}

void writeHeader() {
  file.print(F("millis"));
  file.print(F(",Pitch"));
  file.print(F(",Roll"));
  file.print(F(",Heading"));
  file.print(F(",Altitude"));
  file.print(F(",Autopilot"));
  file.print(F(",GPS Lat"));
  file.print(F(",GPS Lon"));
  file.print(F(",GPS Alt"));
  file.print(F(",GPS Speed"));  
  file.println();
}

//~~~~~~~~~~~~~~~~~~~~~~~Setup~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
  Wire.begin();
  Serial.begin(115200); // begin serial communication for debugging
  ss.begin(GPSBaud);
  
  if (! accel.begin()) {    // start accelerometer
    Serial.println(F("Couldn't start accelerometer"));
    delay(100);
  }

  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Couldn't start magnetometer"));
    while(1);
  }

  if (! gyro.begin()) { // start gyroscope
    Serial.println(F("Couldn't start gyroscope"));
    delay(100);
  }

  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.println(F("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }

  delay(500);
  Serial.println(F("Initialization done."));
  gyro.enableAutoRange(true);
  accel.enableAutoRange(true);
  mag.enableAutoRange(true);

  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";

    file.println();
    // Initialize at the highest speed supported by the board that is
  // not over 50 MHz. Try a lower speed if SPI errors occur.
  if (!sd.begin(chipSelect, SD_SCK_MHZ(50))) {
    sd.initErrorHalt();
  }

  // Find an unused file name.
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    }
    else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    }
    else {
      error("Can't create file name");
    }
  }
  if (!file.open(fileName, O_WRONLY | O_CREAT | O_EXCL)) {
    error("file.open");
  }

  Serial.print(F("Logging to: "));
  Serial.println(fileName);

  // Write data header
  writeHeader();
}


//~~~~~~~~~~~~~~~~~~~~~Main Loop~~~~~~~~~~~~~~~~~~~~
void loop(){
  if (millis() >= (t + sampleTime)){
    allDat[0] = millis();   // write current time to allDat[0]
    dt = millis() - t;      // time since previous read
    t = millis();           //  reset previous read time to current time

    updateSensorData();

    sendData();
    

  for (int i=0; i < 10; i++){
    file.print(allDat[i], 6);
    file.write(", ");
    }
    file.println();
  }
  file.sync();
}
