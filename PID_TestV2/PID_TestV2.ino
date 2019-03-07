#include <Servo.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>
#include <Wire.h>

LSM9DS1 imu;
Servo myservo;

#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

float apitch = 0;
float accelx = 0;
float accely = 0;
float accelz = 0;

float target = 0;
float current = 0;
float error = 0;
float prevError = 0;
float integral = 0;
float derivative = 0;
float pos = 0;

float kp = 30;
float ki = 0;
float kd = 0;

int t = 0;
int dt = 0;


void setup() {
  Serial.begin(115200);
  myservo.attach(9);
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
  imu.calibrate();
}

void loop() {
  dt = millis() - t;
  if (dt > 50){
    t = millis();
    target = 0;
    current = current * 0.3 + getPitch() * 0.7;
    error = target - current;

    integral = integral + (error * dt);
    if (error < 0.1){
      integral = 0;
    }
    if (abs(error) > 40){
      integral = 0;
    }

    derivative = (error - prevError)/dt;
    prevError = error;
    pos = pos + (kp * error + ki * integral + kd * derivative)/dt;
    Serial.print(current);
    Serial.print(", ");
    Serial.println(error);
    myservo.write(map(pos, -90, 90, 180, 0));
  }
}

float getPitch(){
  if (imu.accelAvailable()){
    imu.readAccel();
  }
  accelx = imu.calcAccel(imu.ax);
  accely = imu.calcAccel(imu.ay);
  accelz = imu.calcAccel(imu.az);
  apitch = atan2(accely, sqrt(accelx * accelx + accelz * accelz)) * 57.29578;
  return(apitch);
}
