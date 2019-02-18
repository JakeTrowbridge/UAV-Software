/*
Flight Data Recorder Code Here!
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>

//sensors
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

/* alignmant
 * X is Forward
 * Z is Up
 * Y is wing span wise
 */

//~~~~~~~~~~~~~~Constant Variables~~~~~~~~~~~~~~~~~~
File dataFile; // define file to write
long t = 0;
int dt = 0;
float pitch = 0;
float roll = 0;
float yaw = 0;
float allDat[15]; //format: [0Time, 1Pitch, 2Roll, 3Heading, 4PressAlt, 5GPSLong, 6GPSLat,
                  // 7GPSAlt, 8GPSSpeed, 9Ch1, 10Ch2, 11Ch3, 12Ch4, 13Ch5, 14Ch6]

//~~~~~~~~~~~~~~Adjustable Variables~~~~~~~~~~~~~~~~~~
int sampleTime = 50;       // minimum time between samples in ms
float P_CompCoeff = 0.975;  // Pitch complimentary coefficient
float R_CompCoeff = 0.975;  // Roll complimentary coefficient

//~~~~~~~~~~~~~~~~~~~~~~~Setup~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
  Serial.begin(9600); // begin serial communication for debugging
  if (! accel.begin()) {    // start accelerometer
    Serial.println("Couldn't start accelerometer");
    delay(100);
  }
  if (! gyro.begin()) { // start gyroscope
    Serial.println("Couldn't start gyroscope");
    delay(100);
  }
  if (!SD.begin(10)) { // start SD card
    Serial.println("Sd initialization failed!");
    while (1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.println("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    for (int i = 2; i < 8; i++){  // set pins D2-D7 as input pins for receiver
    pinMode(i, INPUT);
  }
  Serial.println("Initialization done.");
  gyro.enableAutoRange(true);
  accel.enableAutoRange(true);
}

//~~~~~~~~~~~~~~~~~~~~~Main Loop~~~~~~~~~~~~~~~~~~~~
void loop(){
  if (millis() >= (t + sampleTime)){
    allDat[0] = millis();   // write current time to allDat[0]
    dt = millis() - t;      // time since previous read
    t = millis();           //  reset previous read time to current time
    
    updateSensorData();
    updateReceiverData();
    sdWrite();

//  Serial.print(allDat[0]); //Serial write for testing
//  Serial.print(", ");
//  Serial.print(allDat[1]);
//  Serial.print(", ");
//  Serial.print(allDat[2]);
//  Serial.print(", ");
//  Serial.println(allDat[4]);
  }  
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
  
    pitch = P_ComplementaryFilter(AX,AY,AZ,RX,RY,RZ); // calculate pitch
    roll  = R_ComplementaryFilter(AX,AY,AZ,RX,RY,RZ); // calculate roll
    
    allDat[1] = pitch;
    allDat[2] = roll;

    bmp.getEvent(&event);
    allDat[4] = bmp.pressureToAltitude(1015, event.pressure);
}


float P_ComplementaryFilter(float ax,float ay,float az,float rx,float ry,float rz) {    // pitch complimentary filter
  long squaresum=(long)ay*ay+(long)az*az;
  pitch+=((-rx/32.8f)*(dt/1000000.0f));
  float pitchAcc =atan(ax/sqrt(squaresum))*RAD_TO_DEG;
  pitch =P_CompCoeff*pitch + (1.0f-P_CompCoeff)*pitchAcc;
  return(pitch);
}

float R_ComplementaryFilter(float ax,float ay,float az,float rx,float ry,float rz) {    // roll complimentary filter
  long squaresum=(long)ax*ax+(long)az*az;
  roll+=((-ry/32.8f)*(dt/1000000.0f));
  float rollAcc =atan(ay/sqrt(squaresum))*RAD_TO_DEG;
  roll =R_CompCoeff*roll + (1.0f-R_CompCoeff)*rollAcc;
  return(roll);
}

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

    
void updateReceiverData(){   // get pulse lengths from receiver in ms
  for (int x = 2; x < 8; x++){
    allDat[(x + 7)] = pulseIn((x), HIGH); //(x + 7) to assign to correct allDat entry
  }
}

//~~~~~~~~~~~~~~~~I2C Communicatcion~~~~~~~~~~~~~~~~
