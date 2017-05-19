#include <Kalman.h>
#include<Servo.h>
#include<Wire.h>
#include<I2Cdev.h>
#include<MPU6050.h>
#include<LiquidCrystal.h>                                          
#include <stdlib.h>
#include <AutoDIY.h>
MPU6050 CBgoc;
Kalman kalmanX;
AutoDIY robot;
//IMU 6050====================================================
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;
float accXangle;
float gyroXangel;
float kalAngelX;
unsigned long timer;
uint8_t i2cData[14];
float CurrentAngle;

// LCD====================================================
LiquidCrystal lcd(1, 0, A0, A1, A2, A3);   
int speed;


// PID====================================================
const float Kp = 14.1; //14
const float Ki = 2.1; //2 xuất hiện tượng ì, tăng Ki. Ki đủ lớn, vọt lố lại xuất hiện, giảm Ki hệ đi vào ổn định.
const float Kd = 10.1; //10 //cần tăng Kd từ từ sao cho giảm độ vọt lố
float pTerm, iTerm, dTerm, integrated_error, last_error, error;
const float K = 1.9*1.2; //1.9*1.12
#define   GUARD_GAIN   10.0
#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t))

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress,&data,1,sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  return Wire.endTransmission(sendStop); // Returns 0 on success
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t* data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  if(Wire.endTransmission(false)) // Don't release the bus
    return 1; // Error in communication
  Wire.requestFrom(IMUAddress, nbytes,(uint8_t)true); // Send a repeated start and then release the bus after reading
  for(uint8_t i = 0; i < nbytes; i++) {
    if(Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while(((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if(Wire.available())
        data[i] = Wire.read();
      else
        return 2; // Error in communication
    }
  }
  return 0; // Success
}


void setup() 
{
Serial.begin(9600);
lcd.begin(16,2);
  pinMode(11,INPUT_PULLUP);
  pinMode(12,INPUT_PULLUP);
  pinMode(9,INPUT_PULLUP);
  lcd.print("ROBOTIC-STARTER"); delay(1000); lcd.clear();
  lcd.setCursor(0,0); lcd.print("AUTODIY"); 
  robot.init_pin();
Wire.begin();

i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz 
i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling 
i2cData[2] = 0x00;
i2cData[3] = 0x00;
while(i2cWrite(0x19,i2cData,4,false)); 
while(i2cWrite(0x6B,0x01,true));
while(i2cRead(0x75,i2cData,1));
if(i2cData[0] != 0x68) { // Read "WHO_AM_I" register
Serial.print(F("Error reading sensor"));
while(1);
  }
delay(100); 

//Kalman====================================================
while(i2cRead(0x3B,i2cData,6));
accX = ((i2cData[0] << 8) | i2cData[1]);
accY = ((i2cData[2] << 8) | i2cData[3]);
accZ = ((i2cData[4] << 8) | i2cData[5]);
accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
kalmanX.setAngle(accXangle); 
gyroXangel = accXangle; 
timer = micros();
 }

void loop()
{
//Serial.println(accX);
 // Serial.println(accY);
 // Serial.println(accZ);
//Serial.print(accXangle);
//Serial.print("  ");
Serial.println(CurrentAngle);
runEvery(25)
  {
dof();
if(CurrentAngle <=180 && CurrentAngle >=179)
  stop();
else{
  if(CurrentAngle < 200 && CurrentAngle > 150){
    Pid();
    Motors();
  }
  else
    stop();
}
} 
}
void Motors()
{
 // if(speed>160)
 //   speed = speed/3;
 // if(speed<-160)
  //  speed = speed/3;
//Serial.print("Speed  ");
//Serial.println(speed);
if(speed > 0)
  {
    robot.control_motor(0,0,speed);
    robot.control_motor(1,0,speed);
   }
else
   {
speed = map(speed,0,-255,0,255);
    robot.control_motor(0,1,speed);
    robot.control_motor(1,1,speed);
   }
}
void stop()
{
robot.control_motor(0,0,0);
robot.control_motor(1,0,0);
}
void Pid()
{
error = 178 - CurrentAngle;  // 179 = level
pTerm = Kp * error;
  integrated_error += error;
iTerm = Ki*constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
dTerm = Kd*(error - last_error);
  last_error = error;
speed = constrain(K*(pTerm + iTerm + dTerm), -255, 255);
}
void dof()
{
while(i2cRead(0x3B,i2cData,14));
accX = ((i2cData[0] << 8) | i2cData[1]);
accY = ((i2cData[2] << 8) | i2cData[3]);
accZ = ((i2cData[4] << 8) | i2cData[5]);
tempRaw = ((i2cData[6] << 8) | i2cData[7]);  
gyroX = ((i2cData[8] << 8) | i2cData[9]);
gyroY = ((i2cData[10] << 8) | i2cData[11]);
gyroZ = ((i2cData[12] << 8) | i2cData[13]);
accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
double gyroXrate = (double)gyroX/131.0;
   CurrentAngle = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000);
timer = micros();
}
