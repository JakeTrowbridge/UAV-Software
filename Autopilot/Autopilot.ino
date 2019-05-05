#include <I2C_Anything.h>
#include <Servo.h>
#include <Wire.h>

#define select 13
#define elevIn 4
#define ailIn 3

const byte MY_ADDRESS = 42;

volatile boolean haveData = false;




//~~~~~~~~~~~~~~~~~~~~Variable~~~~~~~~~~~~~~~~~~~~~
float Alt_target = 0;
float Alt_current = 0;
float Alt_error = 0;
float Alt_prevError = 0;
float Alt_integral = 0;
float Alt_derivative = 0;
float Elv_pos = 0;

float Roll_target = 0;
float Roll_current = 0;
float Roll_error = 0;
float Roll_prevError = 0;
float Roll_integral = 0;
float Roll_derivative = 0;
float Ail_pos = 0;

long t = 0;
int dt = 0;

float allDat[9]; //format: [0Time, 1Pitch, 2Roll, 3Heading, 4PressAlt, 5AutoPilot, 6GPSLong, 7GPSLat,8GPSSpeed]

//~~~~~~~~~~~~~~~~Changable Variables~~~~~~~~~~~~
float Alt_kp = 1;
float Alt_ki = 0;
float Alt_kd = 0;

float Roll_kp = 1;
float Roll_ki = 0;
float Roll_kd = 0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void receiveEvent(int howMany)// called by interrupt service routine when incoming data arrives
 {
 if (howMany >= (32))
   {
   I2C_readAnything (allDat); 
   haveData = true;     
   }  // end if have enough data
 
} // end of receiveEvent

//~~~~~~~~~~~~~~~~~~~~~ServoSetup~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Servo Elv_servo;
  Servo Ail_servo;

//~~~~~~~~~~~~~~~~~~~~~~~~~~Setup~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup(){
  Wire.begin (MY_ADDRESS);
  Serial.begin (115200);
  Wire.onReceive (receiveEvent);

  Elv_servo.attach(elevIn);
  Ail_servo.attach(ailIn);
}

void loop(){
//~~~~~~~~~~~~~~~~~~~~~~~~~I2C comunication~~~~~~~~~~~~~~~~~~  
  if (haveData)    {
     for(int i=1;i<6;i++){
      Serial.print(allDat[i]); Serial.print(", ");
     }
      Serial.println();
    haveData = false;
    }
////~~~~~~~~~~~~~~~~~~~~~~~~Autopilot~~~~~~~~~~~~~~~~~~~~~~~~~
  if(allDat[5]==0){
    Alt_target = 0;
    digitalWrite(select, LOW);
  }
  else if(allDat[5]==1){
    if (Alt_target == 0){
      Alt_target = allDat[4];
    }
    digitalWrite(select, HIGH);
    dt = millis() - t;
    if (dt > 50){
      t = millis();
////~~~~~~~~~~~~~~~~~~~~~~~~Altitude Stabilization~~~~~~~~~~~~~~~~      
//      Alt_current = Alt_current * 0.3 + allDat[4] * 0.7;
//      Alt_error = Alt_target - Alt_current;
//
//      Alt_integral = Alt_integral + (Alt_error * dt);
//      if (Alt_error < 0.1){
//        Alt_integral = 0;
//      }
//      if (abs(Alt_error) > 40){
//        Alt_integral = 0;
//      }
//  
//      Alt_derivative = (Alt_error - Alt_prevError)/dt;
//      Alt_prevError = Alt_error;
//      Elv_pos =(Alt_kp * Alt_error + Alt_ki * Alt_integral + Alt_kd * Alt_derivative)/dt;
//      Serial.print(allDat[4]);
//      Serial.print(", ");
//      Serial.print(Alt_current);
//      Serial.print(", ");
//      Serial.println(Elv_pos);
//      Elv_servo.write(map(Elv_pos, -90, 90, 150, 30));

// //~~~~~~~~~~~~~~~~~~~~~~~Roll Stabilization~~~~~~~~~~~~~~~~~~~~~
      Roll_current = Roll_current * 0.3 + allDat[2] * 0.7;
      Roll_error = Roll_target - Roll_current;
  
      Roll_integral = Roll_integral + (Roll_error * dt);
      if (Roll_error < 0.1){
        Roll_integral = 0;
      }
      if (abs(Roll_error) > 40){
        Roll_integral = 0;
      }
      Roll_derivative = (Roll_error - Roll_prevError)/dt;
      Roll_prevError = Roll_error;
      Ail_pos =(Roll_kp * Roll_error + Roll_ki * Roll_integral + Roll_kd * Roll_derivative)/dt;
//      Serial.print(allDat[2]);
//      Serial.print(", ");
//      Serial.print(Roll_current);
//      Serial.print(", ");
//      Serial.println(Ail_pos);
      Ail_servo.write(map(Ail_pos, -90, 90, 150, 30));
    }
  }
} // end of loop
