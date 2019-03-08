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

float allDat[10]; //format: [0Time, 1Pitch, 2Roll, 3Heading, 4PressAlt, 5GPSLong, 6GPSLat,
                  // 7GPSAlt, 8GPSSpeed, 9AutoPilot]

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void receiveEvent(int howMany)// called by interrupt service routine when incoming data arrives
 {
 Serial.println(howMany);
 if (howMany >= (32))
   {
   I2C_readAnything (allDat); 
   haveData = true;     
   }  // end if have enough data
 
} // end of receiveEvent

//~~~~~~~~~~~~~~~~~~~~~ServoSetup~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~Setup~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup(){
  Wire.begin (MY_ADDRESS);
  Serial.begin (9600);
  Wire.onReceive (receiveEvent);
}  // end of setup

void loop(){
//~~~~~~~~~~~~~~~~~~~~~~~~~I2C comunication~~~~~~~~~~~~~~~~~~~  
  if (haveData)
    {
     for(int i=0;i<10;i++){
      Serial.print(allDat[i]); Serial.print(", ");
     }
      Serial.println();
    haveData = false;
    }  // end if haveData
}  // end of loop
