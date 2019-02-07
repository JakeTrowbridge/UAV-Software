#include <Wire.h>
#include <I2C_Anything.h>

const byte MY_ADDRESS = 42;

void setup() 
{
  Wire.begin (MY_ADDRESS);
  Serial.begin (115200);
  Wire.onReceive (receiveEvent);
}  // end of setup

volatile boolean haveData = false;
volatile float data[5];

void loop() 
{


  if (haveData)
    {
     for(int i=0;i<5;i++){
      Serial.print(data[i]); Serial.print(", ");
     }
      Serial.println();
    haveData = false;
    }  // end if haveData

}  // end of loop

// called by interrupt service routine when incoming data arrives
void receiveEvent (int howMany)
 {
 if (howMany >= (sizeof data))
   {
   I2C_readAnything (data); 
   haveData = true;     
   }  // end if have enough data
 
 }  // end of receiveEvent
 
