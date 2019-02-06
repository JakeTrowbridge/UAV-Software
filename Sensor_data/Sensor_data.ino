#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_L3GD20_U.h>

//sensors
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_LIS3DH accel = Adafruit_LIS3DH();

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
float yaw = 0;
    
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

    float raw_data[]={millis(),AX,AY,AZ,RX,RY,RZ};


    pitch = P_ComplementaryFilter(AX,AY,AZ,RX,RY,RZ);
    roll  = R_ComplementaryFilter(AX,AY,AZ,RX,RY,RZ);
    yaw   = Y_ComplementaryFilter(AX,AY,AZ,RX,RY,RZ);
    float data[]={pitch,roll,yaw-90};


//  dataFile = SD.open("datatest.csv", FILE_WRITE); 
    //if (dataFile) {
      for(int i=0;i<3;i++){
        Serial.print(data[i]); Serial.print(", ");
        //dataFile.print(data[i]); dataFile.print(", ");
      } //dataFile.println();
      Serial.println();
//    } else { Serial.println("error opening file");
    }  
  //  dataFile.close();
  }
//}

float P_CompCoeff = 0.975;
float R_CompCoeff = 0.975;
float Y_CompCoeff = 0.975;

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
float Y_ComplementaryFilter(float ax,float ay,float az,float rx,float ry,float rz) {
 long squaresum=(long)ax*ax+(long)ay*ay;
 yaw+=((-rz/32.8f)*(dt/1000000.0f));
 float yawAcc =atan(az/sqrt(squaresum))*RAD_TO_DEG;
 yaw =R_CompCoeff*yaw + (1.0f-Y_CompCoeff)*yawAcc;
 return(yaw);
} 
