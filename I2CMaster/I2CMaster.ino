#include <Wire.h>
#include <I2C_Anything.h>

const byte SLAVE_ADDRESS = 42;

void setup() 
{
  Wire.begin ();
}  // end of setup

void loop() 
{

 float data[] = {42.6, 360, 1, 2, 3};
    Wire.beginTransmission (SLAVE_ADDRESS);
    I2C_writeAnything (data);
    Wire.endTransmission ();
    delay (200);

}  // end of loop
