#include <I2C_Anything.h>
#include <Servo.h>
#include <Wire.h>


const byte MY_ADDRESS = 42;

volatile boolean haveData = false;

//~~~~~~~~~~~~~~~~~~~~Variable~~~~~~~~~~~~~~~~~~~~~
float Elv_target = 0;
float Elv_current = 0;
float Elv_error = 0;
float prevElv_error = 0;
float Elv_integral = 0;
float Elv_derivative = 0;

float Elv_kp = 10;
float Elv_ki = 0;
float Elv_kd = 0;

long t = 0;
int dt = 0;

float allDat[15]; //format: [0Time, 1Pitch, 2Roll, 3Heading, 4PressAlt, 5GPSLong, 6GPSLat,
                  // 7GPSAlt, 8GPSSpeed, 9Ch1, 10Ch2, 11Ch3, 12Ch4, 13Ch5, 14Ch6]

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void receiveEvent(int howMany)// called by interrupt service routine when incoming data arrives
 {
 if (howMany >= (sizeof allDat))
   {
   I2C_readAnything (allDat); 
   haveData = true;     
   }  // end if have enough data
 
} // end of receiveEvent

//~~~~~~~~~~~~~~~~~~~~~ServoSetup~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Servo Elv_Servo;
Servo Alr_Servo;

//~~~~~~~~~~~~~~~~~~~~~~~~~~Setup~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup(){
  Wire.begin (MY_ADDRESS);
  Serial.begin (9600);
  Wire.onReceive (receiveEvent);
  Elv_Servo.attach(12);
  Alr_Servo.attach(11);
}  // end of setup

void loop(){
//~~~~~~~~~~~~~~~~~~~~~~~~~I2C comunication~~~~~~~~~~~~~~~~~~~  
  if (haveData)
    {
     for(int i=0;i<15;i++){
      Serial.print(allDat[i]); Serial.print(", ");
     }
      Serial.println();
    haveData = false;
    }  // end if haveData

//~~~~~~~~~~~~~~~~~~~~~PID~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  dt = millis() - t;
  if (dt > 50){
    t = millis();
    Elv_target = allDat[1];
    
    Elv_error = Elv_target - Elv_current;   //P

    Elv_integral = Elv_integral + (Elv_error * dt);
    if (Elv_error < 0.1){
      Elv_integral = 0;
    }
    if (abs(Elv_error) > 40){
      Elv_integral = 0;
    }

    Elv_derivative = (Elv_error - prevElv_error)/dt;
    prevElv_error = Elv_error;
    Elv_current = Elv_current + (Elv_kp * Elv_error + Elv_ki * Elv_integral + Elv_kd * Elv_derivative)/dt;
    Serial.print(Elv_target);
    Serial.print(", ");
    Serial.println(Elv_current);
    Elv_Servo.write(map(Elv_current, -90, 90, 180, 0));
}
}  // end of loop



