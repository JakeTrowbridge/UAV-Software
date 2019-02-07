#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_L3GD20_U.h>
#include <I2C_Anything.h>

//sensors
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_LIS3DH accel = Adafruit_LIS3DH();

const byte MY_ADDRESS = 42;

/* alignmant
 * X is Forward
 * Z is Up
 * Y is wing span wise
 */
File dataFile;

long t = 0;
long dt = 0;
float pitch = 0;
float roll = 0;

// for comunicating with the autopilot arduino

volatile boolean haveData = false;
volatile float data[6];

void setup() {
  
  Serial.begin(9600);
  if (! accel.begin()) {
    Serial.println("Couldn't start accelerrometer");
    delay(100);
  }
  if (! gyro.begin()) {
    Serial.println("Couldn't start gyroscope");
    delay(100);
  }

  /*if (!SD.begin(10)) {
    Serial.println("Sd initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  */
  gyro.enableAutoRange(true);
  accel.enableAutoRange(true);
  
  Wire.begin (MY_ADDRESS);
  Wire.onReceive (receiveEvent);
  
}

void loop() {

  if (millis()>t+dt){
  dt = millis()-t;    
  t = millis();
    sensors_event_t event;
    accel.getEvent(&event);
    float AX = event.acceleration.x;
    float AY = event.acceleration.y;
    float AZ = event.acceleration.z;

    gyro.getEvent(&event);
    float RX = event.gyro.x;
    float RY = event.gyro.y;
    float RZ = event.gyro.z;


    pitch = P_ComplementaryFilter(AX,AY,AZ,RX,RY,RZ);
    roll  = R_ComplementaryFilter(AX,AY,AZ,RX,RY,RZ);

    float att_data[]={pitch,roll};


//  dataFile = SD.open("datatest.csv", FILE_WRITE); 
    //if (dataFile) {
      for(int i=0;i<2;i++){
        Serial.print(att_data[i]); Serial.print(", ");
        //dataFile.print(att_data[i]); dataFile.print(", ");
      } //dataFile.println();
      Serial.println();
//    } else { Serial.println("error opening file");
    }  
  //  dataFile.close();
// recive data from A.P.
int input_data[6];
  if (haveData){
     for(int i=0;i<6;i++){
      input_data[i] = data[i];
     }
    haveData = false;
    }  // end if haveData

  
  }


float P_CompCoeff = 0.975;
float R_CompCoeff = 0.975;

float P_ComplementaryFilter(float ax,float ay,float az,float rx,float ry,float rz) {
 long squaresum=(long)ay*ay+(long)az*az;
 pitch+=((-ry/32.8f)*(dt/1000000.0f));
 float pitchAcc =atan(ax/sqrt(squaresum))*RAD_TO_DEG;
 pitch =P_CompCoeff*pitch + (1.0f-P_CompCoeff)*pitchAcc;
 return(pitch);
} 
float R_ComplementaryFilter(float ax,float ay,float az,float rx,float ry,float rz) {
 long squaresum=(long)ax*ax+(long)az*az;
 roll+=((-rx/32.8f)*(dt/1000000.0f));
 float rollAcc =atan(ay/sqrt(squaresum))*RAD_TO_DEG;
 roll =R_CompCoeff*roll + (1.0f-R_CompCoeff)*rollAcc;
 return(roll);
}
void receiveEvent (int howMany){
 if (howMany >= (sizeof data)){
   I2C_readAnything (data); 
   haveData = true;     
   }}
