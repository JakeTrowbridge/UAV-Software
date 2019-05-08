#include <I2C_Anything.h>
#include <Servo.h>
#include <Wire.h>

#define select 13
#define elevIn 4
#define ailIn 3

const byte MY_ADDRESS = 42;

volatile boolean haveData = false;


//~~~~~~~~~~~~~~~~~~~~Variable~~~~~~~~~~~~~~~~~~~~~
float Roll_target = 0;
float Roll_current = 0;
float Roll_error = 0;
float Roll_prevError = 0;
float Roll_integral = 0;
float Roll_derivative = 0;
float Ail_pos = 0;

long t = 0;
int dt = 0;

float allDat[9]; //format: [0Time, 1Roll, 2Pitch, 3Heading, 4PressAlt, 5AutoPilot, 6GPSLong, 7GPSLat,8GPSSpeed]

//~~~~~~~~~~~~~~~~Changable Variables~~~~~~~~~~~~
float Roll_kp = 0.9;
float Roll_ki = 0.01;
float Roll_kd = 0;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void receiveEvent(int howMany){// called by interrupt service routine when incoming data arrives
  if (howMany >= (32))
    {
    I2C_readAnything (allDat); 
    haveData = true;     
  }  // end if have enough data
} // end of receiveEvent

//~~~~~~~~~~~~~~~~~~~~~ServoSetup~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  Servo Ail_servo;

//~~~~~~~~~~~~~~~~~~~~~~~~~~Setup~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup(){
  Wire.begin (MY_ADDRESS);
  Serial.begin (115200);
  Wire.onReceive (receiveEvent);
  Ail_servo.attach(ailIn);
}

void loop(){
//~~~~~~~~~~~~~~~~~~~~~~~~~I2C comunication~~~~~~~~~~~~~~~~~~  
  if (haveData){
    for(int i=1;i<6;i++){
//      Serial.print(allDat[i]);
//      Serial.print(", ");
   }
//   Serial.println();
   haveData = false;
  }
////~~~~~~~~~~~~~~~~~~~~~~~~Autopilot~~~~~~~~~~~~~~~~~~~~~~~~~
  if(allDat[5]==0){
    digitalWrite(select, LOW);
  }
  else if(allDat[5]==1){
    digitalWrite(select, HIGH);
    dt = millis() - t;
    if (dt > 50){
      t = millis();

      Roll_current = Roll_current * 0.3 + allDat[1] * 0.7;
      Roll_error = Roll_target - Roll_current;
  
      Roll_integral = Roll_integral + (Roll_error * dt);
      if (abs(Roll_error) < 0.1){
        Roll_integral = 0;
      }
      if (abs(Roll_integral) > 40){
        Roll_integral = 0;
      }
      Roll_derivative = (Roll_error - Roll_prevError)/dt;
      Roll_prevError = Roll_error;
      Ail_pos =(Roll_kp * Roll_error + Roll_ki * Roll_integral + Roll_kd * Roll_derivative);
      if (Ail_pos > 15){
        Ail_pos = 15;
      }
      if (Ail_pos < -15){
        Ail_pos = -15;
      }
      Serial.println(Ail_pos);
      Ail_servo.write(map(Ail_pos, -15, 15, 60, 140));
    }
  }
} // end of loop
