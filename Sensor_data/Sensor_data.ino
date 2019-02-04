#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_L3GD20_U.h>

//sensors
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_LIS3DH accel = Adafruit_LIS3DH();

File dataFile;

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

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  
  gyro.enableAutoRange(true);
  accel.enableAutoRange(true);
  
  int counter = 0
}

void loop() {
  if (isdigit(counter/10)){
    int t = millis();
    sensors_event_t event;
    accel.getEvent(&event);
    float AX = event.acceleration.x;
    float AY = event.acceleration.y;
    float AZ = event.acceleration.z;

    gyro.getEvent(&event);
    float RX = event.gyro.x;
    float RY = event.gyro.y;
    float RZ = event.gyro.z;

    float data[]={t,AX,AY,AZ,RX,RY,RZ};
  
  dataFile = SD.open("datatest.csv", FILE_WRITE); 
    if (dataFile) {
      for(int i=0;i<7;i++){
        Serial.print(data[i]); Serial.print(", ");
        dataFile.print(data[i]); dataFile.print(", ");
      } dataFile.println(); Serial.println();
    } else { Serial.println("error opening file");
    }
    dataFile.close();
  }counter++;
}
