/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2014 by Alexander Grau
  Copyright (c) 2013-2014 by Sven Gennat
  
  Private-use only! (you need to ask for a commercial-use)
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  Private-use only! (you need to ask for a commercial-use)
*/

#include "imu.h"
#include <Arduino.h>
#include <Wire.h>
#include "drivers.h"
#include "i2c.h"
#include "config.h"
#include "flashmem.h"
#include "buzzer.h"

// -------------I2C addresses ------------------------
#define ADXL345B (0x53)          // ADXL345B acceleration sensor (GY-80 PCB)
#define HMC5883L (0x1E)          // COMPASSTOUSE = 0; HMC5883L compass sensor (GY-80 PCB)
#define L3G4200D (0xD2 >> 1)     // L3G4200D gyro sensor (GY-80 PCB)
#define MMC5883MA (0x30)         // Sensor I2C address

#define ADDR 600
#define MAGIC 6

//*****************************************************
// MMC5883MA Register map
//*****************************************************
#define XOUT_LSB    (0x00)
#define XOUT_MSB    (0x01)
#define YOUT_LSB    (0x02)
#define YOUT_MSB    (0x03)
#define ZOUT_LSB    (0x04)
#define ZOUT_MSB    (0x05)
#define TEMPERATURE (0x06)
#define STATUS      (0x07)
#define INT_CTRL0   (0x08)
#define INT_CTRL1   (0x09)
#define INT_CTRL2   (0x0A)
#define X_THRESHOLD (0x0B)
#define Y_THRESHOLD (0x0C)
#define Z_THRESHOLD (0x0D)
#define PROD_ID1    (0x2F)

#define MMC5883MA_OFFSET      32768
#define MMC5883MA_SENSITIVITY   4096
//*****************************************************
// MMC5883MA Register map end
//*****************************************************



struct {
  uint8_t xl;
  uint8_t xh;
  uint8_t yl;
  uint8_t yh;
  uint8_t zl;
  uint8_t zh;
} gyroFifo[32];



IMU::IMU(){
  hardwareInitialized = false;
  calibrationAvail = false;
  state = IMU_RUN;
  callCounter = 0;  
  errorCounter = 0;  
  gyroOfs.x=gyroOfs.y=gyroOfs.z=0;  
  gyroNoise = 0;      
  gyroCounter = 0; 
  useGyroCalibration = false;
  lastGyroTime = millis();
  MMC5883MAHeading = 0;
  accelCounter = 0;
  calibAccAxisCounter = 0;
  useAccCalibration = true; 
  accPitch = 0;
  accRoll = 0;
  ypr.yaw=ypr.pitch=ypr.roll = 0;
  
  accMin.x=accMin.y=accMin.z = 0;
  accMax.x=accMax.y=accMax.z = 0;  
  accOfs.x=accOfs.y=accOfs.z = 0;
  accScale.x=accScale.y=accScale.z = 2;  
  com.x=com.y=com.z=0;  
  
  comTemperature = 0;
  comScale.x=comScale.y=comScale.z=2;  
  comOfs.x=comOfs.y=comOfs.z=0;    
  useComCalibration = true;
}

// rescale to -PI..+PI
float IMU::scalePI(float v)
{
  float d = v;
  while (d < 0) d+=2*PI;
  while (d >= 2*PI) d-=2*PI;
  if (d >= PI) return (-2*PI+d); 
  else if (d < -PI) return (2*PI+d);
  else return d;  
}

// rescale to -180..+180
float IMU::scale180(float v)
{
  float d = v;
  while (d < 0) d+=2*180;
  while (d >= 2*180) d-=2*180;
  if (d >= 180) return (-2*180+d); 
  else if (d < -180) return (2*180+d);
  else return d;  
}


// computes minimum distance between x radiant (current-value) and w radiant (set-value)
float IMU::distancePI(float x, float w)
{
  // cases:   
  // w=330 degree, x=350 degree => -20 degree
  // w=350 degree, x=10  degree => -20 degree
  // w=10  degree, x=350 degree =>  20 degree
  // w=0   degree, x=190 degree => 170 degree
  // w=190 degree, x=0   degree => -170 degree 
  float d = scalePI(w - x);
  if (d < -PI) d = d + 2*PI;
  else if (d > PI) d = d - 2*PI;  
  return d;
}

float IMU::distance180(float x, float w)
{
  float d = scale180(w - x);
  if (d < -180) d = d + 2*180;
  else if (d > 180) d = d - 2*180;  
  return d;
}


// weight fusion (w=0..1) of two radiant values (a,b)
float IMU::fusionPI(float w, float a, float b)
{ 
  float c;
  if ((b >= PI/2) && (a <= -PI/2)){
    c = w * a + (1.0-w) * (b-2*PI);
  } else if ((b <= -PI/2) && (a >= PI/2)){
    c = w * (a-2*PI) + (1.0-w) * b;
  } else c = w * a + (1.0-w) * b;
  return scalePI(c);
}

void IMU::loadSaveCalib(boolean readflag){
  int addr = ADDR;
  short magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic
  eereadwrite(readflag, addr, accOfs);
  eereadwrite(readflag, addr, accScale);    
  eereadwrite(readflag, addr, comOfs);
  eereadwrite(readflag, addr, comScale);      
}

void IMU::loadCalib(){
  short magic = 0;
  int addr = ADDR;
  eeread(addr, magic);
  if (magic != MAGIC) {
    Console.println(F("IMU error: no calib data"));
    return;  
  }
  calibrationAvail = true;
  Console.println(F("IMU: found calib data"));
  loadSaveCalib(true);
}

void IMU::saveCalib(){
  loadSaveCalib(false);
}

void IMU::deleteCalib(){
  int addr = ADDR;
  eewrite(addr, (short)0); // magic  
  accOfs.x=accOfs.y=accOfs.z=0;
  accScale.x=accScale.y=accScale.z=2;  
  comOfs.x=comOfs.y=comOfs.z=0;
  comScale.x=comScale.y=comScale.z=2;  
  Console.println("IMU calibration deleted");  
}

void IMU::printPt(point_float_t p){
  Console.print(p.x);
  Console.print(",");    
  Console.print(p.y);  
  Console.print(",");
  Console.println(p.z);  
}

void IMU::printCalib(){
  Console.println(F("--------"));
  Console.print(F("accOfs="));
  printPt(accOfs);
  Console.print(F("accScale="));
  printPt(accScale);
  Console.print(F("comOfs="));
  printPt(comOfs);
  Console.print(F("comScale="));
  printPt(comScale);  
  Console.println(F("--------"));
}



// calculate gyro offsets
void IMU::calibGyro(){
  Console.println(F("---calibGyro---"));    
  useGyroCalibration = false;
  gyroOfs.x = gyroOfs.y = gyroOfs.z = 0;
  point_float_t ofs;
  while(true){    
    float zmin =  99999;
    float zmax = -99999;  
    gyroNoise = 0;
    ofs.x = ofs.y = ofs.z = 0;      
    for (int i=0; i < 50; i++){
      delay(10);
      readL3G4200D(true);      
      zmin = min(zmin, gyro.z);
      zmax = max(zmax, gyro.z);
      ofs.x += ((float)gyro.x)/ 50.0;
      ofs.y += ((float)gyro.y)/ 50.0;
      ofs.z += ((float)gyro.z)/ 50.0;          
      gyroNoise += sq(gyro.z-gyroOfs.z) /50.0;   // noise is computed with last offset calculation
    }
    Console.print(F("gyro calib min="));
    Console.print(zmin);
    Console.print(F("\tmax="));    
    Console.print(zmax);
    Console.print(F("\tofs="));        
    Console.print(ofs.z);    
    Console.print(F("\tnoise="));                
    Console.println(gyroNoise);  
    if (gyroNoise < 20) break; // optimum found    
    gyroOfs = ofs; // new offset found
  }  
  useGyroCalibration = true;
  Console.print(F("counter="));
  Console.println(gyroCounter);  
  Console.print(F("ofs="));
  printPt(gyroOfs);  
  Console.println(F("------------"));  
}      

// ADXL345B acceleration sensor driver
void  IMU::initADXL345B(){
  I2CwriteTo(ADXL345B, 0x2D, 0);
  I2CwriteTo(ADXL345B, 0x2D, 16);
  I2CwriteTo(ADXL345B, 0x2D, 8);         
}

void IMU::readADXL345B(){  
  uint8_t buf[6];
  if (I2CreadFrom(ADXL345B, 0x32, 6, (uint8_t*)buf) != 6){
    errorCounter++;
    return;
  }
  // Convert the accelerometer value to G's. 
  // With 10 bits measuring over a +/-4g range we can find how to convert by using the equation:
  // Gs = Measurement Value * (G-range/(2^10)) or Gs = Measurement Value * (8/1024)
  // ( *0.0078 )
  float x=(int16_t) (((uint16_t)buf[1]) << 8 | buf[0]); 
  float y=(int16_t) (((uint16_t)buf[3]) << 8 | buf[2]); 
  float z=(int16_t) (((uint16_t)buf[5]) << 8 | buf[4]); 
  //Console.println(z);
  if (useAccCalibration){
    x -= accOfs.x;
    y -= accOfs.y;
    z -= accOfs.z;
    x /= accScale.x*0.5;    
    y /= accScale.y*0.5;    
    z /= accScale.z*0.5;
    acc.x = x;
    //Console.println(z);
    acc.y = y;
    acc.z = z;
  } else {
    acc.x = x;
    acc.y = y;
    acc.z = z;
  }
  /*float accXreal = x + sin(gyroYpr.pitch);
  accXmax = max(accXmax, accXreal );    
  accXmin = min(accXmin, accXreal );        */
  //float amag = sqrt(Xa*Xa + Ya*Ya + Za*Za);
  //Xa /= amag;
  //Ya /= amag;
  //Za /= amag;  
  accelCounter++;
}

// L3G4200D gyro sensor driver
boolean IMU::initL3G4200D(){
  Console.println(F("initL3G4200D"));
  uint8_t buf[6];    
  int retry = 0;
  while (true){
    I2CreadFrom(L3G4200D, 0x0F, 1, (uint8_t*)buf);
    if (buf[0] != 0xD3) {        
      Console.println(F("gyro read error"));
      retry++;
      if (retry > 2){
        errorCounter++;
        return false;
      }
      delay(1000);            
    } else break;
  }
  // Normal power mode, all axes enabled, 100 Hz
  I2CwriteTo(L3G4200D, 0x20, 0b00001100);    
  // 2000 dps (degree per second)
  I2CwriteTo(L3G4200D, 0x23, 0b00100000);      
  I2CreadFrom(L3G4200D, 0x23, 1, (uint8_t*)buf);
  if (buf[0] != 0b00100000){
      Console.println(F("gyro write error")); 
      while(true);
  }  
  // fifo mode 
 // I2CwriteTo(L3G4200D, 0x24, 0b01000000);        
 // I2CwriteTo(L3G4200D, 0x2e, 0b01000000);          
  delay(250);
  calibGyro();    
  return true;
}

void IMU::readL3G4200D(boolean useTa){  
  now = micros();
  float Ta = 1;
  //if (useTa) {
    Ta = ((now - lastGyroTime) / 1000000.0);    
    //float Ta = ((float)(millis() - lastGyroTime)) / 1000.0; 			    
    lastGyroTime = now;
    if (Ta > 0.5) Ta = 0;   // should only happen for the very first call
    //lastGyroTime = millis();    
  //}  
  uint8_t fifoSrcReg = 0;  
  I2CreadFrom(L3G4200D, 0x2F, sizeof(fifoSrcReg), &fifoSrcReg);         // read the FIFO_SRC_REG
   // FIFO_SRC_REG
   // 7: Watermark status. (0: FIFO filling is lower than WTM level; 1: FIFO filling is equal or higher than WTM level)
   // 6: Overrun bit status. (0: FIFO is not completely filled; 1:FIFO is completely filled)
   // 5: FIFO empty bit. (0: FIFO not empty; 1: FIFO empty)
   // 4..0: FIFO stored data level
   //Console.print("FIFO_SRC_REG: "); Console.println(fifoSrcReg, HEX);
  uint8_t countOfData = (fifoSrcReg & 0x1F) + 1;   
  //  if (bitRead(fifoSrcReg, 6)==1) Console.println(F("IMU error: FIFO overrun"));

  memset(gyroFifo, 0, sizeof(gyroFifo[0])*32);
  I2CreadFrom(L3G4200D, 0xA8, sizeof(gyroFifo[0])*countOfData, (uint8_t *)gyroFifo);         // the first bit of the register address specifies we want automatic address increment
  //I2CreadFrom(L3G4200D, 0x28, sizeof(gyroFifo[0])*countOfData, (uint8_t *)gyroFifo);         // the first bit of the register address specifies we want automatic address increment

  gyro.x = gyro.y = gyro.z = 0;
  //Console.print("fifo:");
  //Console.println(countOfData);
  if (!useGyroCalibration) countOfData = 1;
  for (uint8_t i=0; i<countOfData; i++){
      gyro.x += (int16_t) (((uint16_t)gyroFifo[i].xh) << 8 | gyroFifo[i].xl);
      gyro.y += (int16_t) (((uint16_t)gyroFifo[i].yh) << 8 | gyroFifo[i].yl);
      gyro.z += (int16_t) (((uint16_t)gyroFifo[i].zh) << 8 | gyroFifo[i].zl);
      if (useGyroCalibration){
        gyro.x -= gyroOfs.x;
        gyro.y -= gyroOfs.y;
        gyro.z -= gyroOfs.z;               
      }
  }
  if (useGyroCalibration){
    gyro.x *= 0.07 * PI/180.0;  // convert to radiant per second
    gyro.y *= 0.07 * PI/180.0; 
    gyro.z *= 0.07 * PI/180.0;      
  }
  gyroCounter++;
}


// HMC5883L compass sensor driver
void  IMU::initHMC5883L(){
  I2CwriteTo(HMC5883L, 0x00, 0x70);  // 8 samples averaged, 75Hz frequency, no artificial bias.       
  //I2CwriteTo(HMC5883L, 0x01, 0xA0);      // gain
  I2CwriteTo(HMC5883L, 0x01, 0x20);   // gain
  I2CwriteTo(HMC5883L, 0x02, 00);    // mode         
}

void IMU::readHMC5883L(){    
  uint8_t buf[6];  
  if (I2CreadFrom(HMC5883L, 0x03, 6, (uint8_t*)buf) != 6){
    errorCounter++;
    return;
  }
  // scale +1.3Gauss..-1.3Gauss  (*0.00092)  
  float x = (int16_t) (((uint16_t)buf[0]) << 8 | buf[1]);
  float y = (int16_t) (((uint16_t)buf[4]) << 8 | buf[5]);
  float z = (int16_t) (((uint16_t)buf[2]) << 8 | buf[3]);  
  if (useComCalibration){
    x -= comOfs.x;
    y -= comOfs.y;
    z -= comOfs.z;
    x /= comScale.x*0.5;    
    y /= comScale.y*0.5;    
    z /= comScale.z*0.5;
    com.x = x;
    //Console.println(z);
    com.y = y;
    com.z = z;
  } else {
    com.x = x;
    com.y = y;
    com.z = z;
  }  
}

float xBridgeOffset, yBridgeOffset, zBridgeOffset;
uint8_t bridgeOffsetDone = 0;

void  IMU::initMMC5883MA(){

  delay(50);  // allow the chip to online for sure

/* continuous reading test  
  I2CwriteTo(MMC5883MA, INT_CTRL1, 0x03);  // set cmd resolution to 16bit, 10ms*3, 100HZ
  delay(1);

  uint8_t buf[6]; 
  uint8_t buf2[2];  

  if (bridgeOffsetDone < 1) {   // Not in use at the moment
    ///////////////////////////////
    // Bridge offset calculation //
    ///////////////////////////////
  
    // set 
    I2CwriteTo(MMC5883MA, INT_CTRL0, 0x08);  
    delay(1);
  
    //measurement 1
    I2CwriteTo(MMC5883MA, INT_CTRL0, 0x01);
    delay(1);

    byte mask = 0x01;
  
    I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)buf2);
    while ( (buf2[0] & mask) != 1 ) {  
      I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)buf2);
    }

    I2CreadFrom(MMC5883MA, 0x00, 6, (uint8_t*)buf);
    
    float x = (uint16_t)(buf[1]<< 8 | buf[0]);
    float y = (uint16_t)(buf[3]<< 8 | buf[2]);
    float z = (uint16_t)(buf[5]<< 8 | buf[4]);
    
    x = ((float)x - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
    y = ((float)y - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
    z = ((float)z - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
  
    // reset
    I2CwriteTo(MMC5883MA, INT_CTRL0, 0x10);
    delay(1);

    //measurement 2
    I2CwriteTo(MMC5883MA, INT_CTRL0, 0x01);
    delay(1);

    I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)buf2);
    while ( (buf2[0] & mask) != 1 ) {  
      I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)buf2);
    }

    I2CreadFrom(MMC5883MA, 0x00, 6, (uint8_t*)buf);

    float x2 = (uint16_t)(buf[1]<< 8 | buf[0]);
    float y2 = (uint16_t)(buf[3]<< 8 | buf[2]);
    float z2 = (uint16_t)(buf[5]<< 8 | buf[4]);
    
    x2 = ((float)x2 - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
    y2 = ((float)y2 - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
    z2 = ((float)z2 - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;

    xBridgeOffset = (x2+x)/2;
    yBridgeOffset = (y2+y)/2;
    zBridgeOffset = (z2+y)/2;

    bridgeOffsetDone = 1;
  }


  Console.print("Offsets x,y,z:");
  Console.print(xBridgeOffset);
  Console.print(",");
  Console.print(yBridgeOffset);
  Console.print(",");
  Console.println(zBridgeOffset);

  I2CwriteTo(MMC5883MA, INT_CTRL1, 0x00);  // set cmd resolution to none
  delay(1);


  I2CwriteTo(MMC5883MA, INT_CTRL2, 0x43);  // start continuous measurement at 14Hz with INT_Meas_Done_En interrupt
  delay(1);
*/

 // I2CwriteTo(MMC5883MA, INT_CTRL0, 0x01);
 // delay(1);

  // Read serial number 
  uint8_t serialNumber[1];  
  I2CreadFrom(MMC5883MA, 0x2F, 1, (uint8_t*)serialNumber);
  Serial.print("Compass serialNumber: ");
  Serial.print(serialNumber[0]);

  // 0x00 set cmd resolution to 16bit,  10 ms*3, 100HZ
  // 0x01 set cmd resolution to 16bit,   5 ms*3, 200HZ
  // 0x02 set cmd resolution to 16bit, 2.5 ms*3, 400HZ
  // 0x03 set cmd resolution to 16bit, 1.6 ms*3, 600HZ
  I2CwriteTo(MMC5883MA, INT_CTRL1, 0x03);  
  delay(1);


}

#include "math.h"
int lastReadTime = 0;
uint8_t compassReadWait = 100; //ms


void IMU::readMMC5883MA(){    
/* continuous test
  uint8_t buf[2];  
  uint8_t buf2[2];  

  byte mask = 0x01;
  I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)buf2);
  if ( (buf2[0] & mask) == 1 ) {  

    I2CreadFrom(MMC5883MA, 0x00, 6, (uint8_t*)buf);
    
    float x = (uint16_t)(buf[1]<< 8 | buf[0]);
    float y = (uint16_t)(buf[3]<< 8 | buf[2]);
    float z = (uint16_t)(buf[5]<< 8 | buf[4]);
    
    x = ((float)x - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY - xBridgeOffset;
    y = ((float)y - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY - yBridgeOffset;
    z = ((float)z - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY - zBridgeOffset;

    if (useComCalibration){
        x -= comOfs.x;
        y -= comOfs.y;
        z -= comOfs.z;
        x /= comScale.x*0.5;    
        y /= comScale.y*0.5;    
        z /= comScale.z*0.5;
        com.x = x;
        com.y = y;
        com.z = z;
      } else {
       com.x = x;
       com.y = y;
       com.z = z;
      }

  }


  I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)buf2);
  if ( (buf2[0] & mask) == mask ) {  
    I2CreadFrom(MMC5883MA, 0x06, 1, (uint8_t*)buf);
    comTemperature = (uint8_t)(((float)buf[0]-71,2)/0,69);
  }
*/

    uint8_t buf[2];  
    uint8_t buf2[1];  
    byte mask = 0xFF;

    mask = 0x02;        
    I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)buf2);
    if ( (buf2[0] & mask) == mask ) {  
      I2CreadFrom(MMC5883MA, 0x06, 1, (uint8_t*)buf);
      //comTemperature = (uint8_t)(((float)buf[0]-71,2)/0,69);
      comTemperature = (uint8_t)(((float)buf[0]-71,2)/0,69);
    }

    // reset 
    I2CwriteTo(MMC5883MA, INT_CTRL0, 0x10);  
    delay(1);
  
    //measurement 1
    I2CwriteTo(MMC5883MA, INT_CTRL0, 0x01);
    delay(1);

    mask = 0x01;
  
    I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)buf2);
    while ( (buf2[0] & mask) != mask ) {  
      I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)buf2);
    }

    I2CreadFrom(MMC5883MA, 0x00, 6, (uint8_t*)buf);
    
    float x = (uint16_t)(buf[1]<< 8 | buf[0]);
    float y = (uint16_t)(buf[3]<< 8 | buf[2]);
    float z = (uint16_t)(buf[5]<< 8 | buf[4]);
    
    x = ((float)x - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
    y = ((float)y - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
    z = ((float)z - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
  
    // set
    I2CwriteTo(MMC5883MA, INT_CTRL0, 0x08);
    delay(1);

    //measurement 2
    I2CwriteTo(MMC5883MA, INT_CTRL0, 0x01);
    delay(1);

    I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)buf2);
    while ( (buf2[0] & mask) != 1 ) {  
      I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)buf2);
    }

    I2CreadFrom(MMC5883MA, 0x00, 6, (uint8_t*)buf);

    float x2 = (uint16_t)(buf[1]<< 8 | buf[0]);
    float y2 = (uint16_t)(buf[3]<< 8 | buf[2]);
    float z2 = (uint16_t)(buf[5]<< 8 | buf[4]);
    
    x2 = ((float)x2 - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
    y2 = ((float)y2 - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
    z2 = ((float)z2 - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;

    x = (x2 - x) / 2;
    y = (y2 - y) / 2;
    z = (z2 - z) / 2;

    if (useComCalibration){
        x -= comOfs.x;
        y -= comOfs.y;
        z -= comOfs.z;
        x /= comScale.x*0.5;    
        y /= comScale.y*0.5;    
        z /= comScale.z*0.5;
        com.x = x;
        com.y = y;
        com.z = z;
      } else {
       com.x = x;
       com.y = y;
       com.z = z;
      }

/*
  if (lastReadTime+compassReadWait < millis() || (state == IMU_CAL_COM)){
    lastReadTime = millis();

//    I2CwriteTo(MMC5883MA, INT_CTRL0, 0x02);
//    delay(20);
//    uint8_t tempBuf[1];
//    I2CreadFrom(MMC5883MA, 0x06, 1, (uint8_t*)tempBuf);
//    Console.print("temp: ");
//    Console.println(tempBuf[0]*0,71-75);

    uint8_t statusreq[1];
    I2CreadFrom(MMC5883MA, STATUS, 1,(uint8_t*)statusreq);
    
//    Console.print("STATASREQ: ");
//    Console.print((statusreq[0] & 1));
//    Console.print(" - ");
//    Console.println(statusreq[0]);
  
    if( (statusreq[0] & 1) == 1) {
      uint8_t buf[6];  
      
      if ( I2CreadFrom(MMC5883MA, 0x00, 6, (uint8_t*)buf) != 6){
        errorCounter++;
        return;
      }
    
//      Serial.print((float)buf[0]);
//      Serial.print(" : ");
//      Serial.print((float)buf[1]);
//      Serial.print(" : ");
//      Serial.print((float)buf[2]);
//      Serial.print(" : ");
//      Serial.print((float)buf[3]);
//      Serial.print(" : ");
//      Serial.print((float)buf[4]);
//      Serial.print(" : ");
//      Serial.println((float)buf[5]);


      float x = (uint16_t)(buf[1]<< 8 | buf[0]);
      float y = (uint16_t)(buf[3]<< 8 | buf[2]);
      float z = (uint16_t)(buf[5]<< 8 | buf[4]);
    
      x = ((float)x - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY - xBridgeOffset;
      y = ((float)y - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY - yBridgeOffset;
      z = ((float)z - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY - zBridgeOffset;



//      x = (float)(((uint16_t)buf[1]) << 8 | buf[0]);
//      x = ((float)x - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY + xBridgeOffset; //unit Gauss
     // x = ConvertTwosComplementByteToInteger(x);
      //x = x - 0x07FF;
//      y = (float)(((uint16_t)buf[3]) << 8 | buf[2]);
//      y = ((float)y - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY + yBridgeOffset; //unit Gauss

   //   y = ConvertTwosComplementByteToInteger(y);
      //y = y - 0x07FF;
//      z = (float)(((uint16_t)buf[5]) << 8 | buf[4]);
//      z = ((float)z - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY + zBridgeOffset; //unit Gauss

   //   z = ConvertTwosComplementByteToInteger(z);
      //z = z - 0x07FF;
    
       
//      float magFieldd = atan2(y,x);
//      float magFieldi = atan(sqrt(pow(x,2) + pow(y,2)) / z);
//      float magFieldf = sqrt(pow(x,2) + pow(y,2) + pow(z,2));

//      Serial.print("magFieldd: ");
//      Serial.print(magFieldd);
//      Serial.print(" - magFieldi: ");
//      Serial.print(magFieldi);
//      Serial.print(" - magFieldf: ");
//      Serial.println(magFieldf);
//
      //Print out values of each axis
//      Serial.print("x: ");
//      Serial.print(x);
//      Serial.print("  y: ");
//      Serial.print(y);
//      Serial.print("  z: ");
//      Serial.println(z);

    if (useComCalibration){
        x -= comOfs.x;
        y -= comOfs.y;
        z -= comOfs.z;
        x /= comScale.x*0.5;    
        y /= comScale.y*0.5;    
        z /= comScale.z*0.5;
        com.x = x;
        com.y = y;
        com.z = z;
      } else {
       com.x = x;
       com.y = y;
       com.z = z;
      }

    }

    I2CwriteTo(MMC5883MA, INT_CTRL0, 0x01);
  
  }
*/
}


float IMU::sermin(float oldvalue, float newvalue){
  if (newvalue < oldvalue) {
    Console.print(".");
    digitalWrite(pinLED, true);
  }
  return min(oldvalue, newvalue);
}

float IMU::sermax(float oldvalue, float newvalue){
  if (newvalue > oldvalue) {
    Console.print(".");
    digitalWrite(pinLED, true);
  }
  return max(oldvalue, newvalue);
}

void IMU::calibComStartStop(){  
  while (Console.available()) Console.read();  
  if (state == IMU_CAL_COM){
    // stop 
    Console.println(F("com calib completed"));    
    calibrationAvail = true;
    float xrange = comMax.x - comMin.x;
    float yrange = comMax.y - comMin.y;
    float zrange = comMax.z - comMin.z;
    comOfs.x = xrange/2 + comMin.x;
    comOfs.y = yrange/2 + comMin.y;
    comOfs.z = zrange/2 + comMin.z;
    comScale.x = xrange;
    comScale.y = yrange;  
    comScale.z = zrange;
    saveCalib();  
    printCalib();
    useComCalibration = true; 
    state = IMU_RUN;    
    // completed sound
    Buzzer.tone(600);
    delay(200); 
    Buzzer.tone(880);
    delay(200); 
    Buzzer.tone(1320);              
    delay(200); 
    Buzzer.noTone();    
    delay(500);
  } else {
    // start
    Console.println(F("com calib..."));
    Console.println(F("rotate sensor 360 degree around all three axis"));
    foundNewMinMax = false;  
    useComCalibration = false;
    state = IMU_CAL_COM;  
    comMin.x = comMin.y = comMin.z = 9999999;
    comMax.x = comMax.y = comMax.z = -9999999;
  }
}

boolean IMU::newMinMaxFound(){
  boolean res = foundNewMinMax;
  foundNewMinMax = false;
  return res;
}  
  
void IMU::calibComUpdate(){
  comLast = com;
  delay(20);
  //if (COMPASSTOUSE == 0) readHMC5883L();
  //if (COMPASSTOUSE == 1) 
  readMMC5883MA();
  boolean newfound = false;
  if ( (abs(com.x-comLast.x)<10) &&  (abs(com.y-comLast.y)<10) &&  (abs(com.z-comLast.z)<10) ){
    if (com.x < comMin.x) { 
      comMin.x = com.x;
      newfound = true;      
    }
    if (com.y < comMin.y) { 
      comMin.y = com.y;
      newfound = true;      
    }
    if (com.z < comMin.z) { 
      comMin.z = com.z;
      newfound = true;      
    }
    if (com.x > comMax.x) { 
      comMax.x = com.x;
      newfound = true;      
    }
    if (com.y > comMax.y) { 
      comMax.y = com.y;
      newfound = true;      
    }
    if (com.z > comMax.z) { 
      comMax.z = com.z;
      newfound = true;      
    }    
    if (newfound) {      
      foundNewMinMax = true;
      Buzzer.tone(440);
      Console.print("x:");
      Console.print(comMin.x);
      Console.print(",");
      Console.print(comMax.x);
      Console.print("\t  y:");
      Console.print(comMin.y);
      Console.print(",");
      Console.print(comMax.y);
      Console.print("\t  z:");
      Console.print(comMin.z);
      Console.print(",");
      Console.print(comMax.z);    
      Console.println("\t");
    } else Buzzer.noTone();   
  }    
}

// calculate acceleration sensor offsets
boolean IMU::calibAccNextAxis(){  
  boolean complete = false;
  Buzzer.tone(440);
  while (Console.available()) Console.read();  
  useAccCalibration = false;  
  if (calibAccAxisCounter >= 6) calibAccAxisCounter = 0;
  if (calibAccAxisCounter == 0){
    // restart
    Console.println(F("acc calib restart..."));
    accMin.x = accMin.y = accMin.z = 99999;
    accMax.x = accMax.y = accMax.z = -99999;    
  }
  point_float_t pt = {0,0,0};
  for (int i=0; i < 100; i++){        
    readADXL345B();            
    pt.x += acc.x / 100.0;
    pt.y += acc.y / 100.0;
    pt.z += acc.z / 100.0;                  
    Console.print(acc.x);
    Console.print(",");
    Console.print(acc.y);
    Console.print(",");
    Console.println(acc.z);
    delay(1);
  }
  accMin.x = min(accMin.x, pt.x);
  accMax.x = max(accMax.x, pt.x);         
  accMin.y = min(accMin.y, pt.y);
  accMax.y = max(accMax.y, pt.y);         
  accMin.z = min(accMin.z, pt.z);
  accMax.z = max(accMax.z, pt.z);           
  calibAccAxisCounter++;        
  useAccCalibration = true;  
  Console.print("side ");
  Console.print(calibAccAxisCounter);
  Console.println(" of 6 completed");    
  if (calibAccAxisCounter == 6){    
    // all axis complete 
    float xrange = accMax.x - accMin.x;
    float yrange = accMax.y - accMin.y;
    float zrange = accMax.z - accMin.z;
    accOfs.x = xrange/2 + accMin.x;
    accOfs.y = yrange/2 + accMin.y;
    accOfs.z = zrange/2 + accMin.z;
    accScale.x = xrange;
    accScale.y = yrange;  
    accScale.z = zrange;    
    printCalib();
    saveCalib();    
    Console.println("acc calibration completed");    
    complete = true;
    // completed sound
    Buzzer.tone(600);
    delay(200); 
    Buzzer.tone(880);
    delay(200); 
    Buzzer.tone(1320);              
    delay(200); 
  };
  Buzzer.noTone();
  delay(500);
  return complete;
}      

// first-order complementary filter
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Complementary2(float newAngle, float newRate,int looptime, float angle) {
  float k=10;
  float dtc2=float(looptime)/1000.0;
  float x1 = (newAngle -   angle)*k*k;
  float y1 = dtc2*x1 + y1;
  float x2 = y1 + (newAngle -   angle)*2*k + newRate;
  angle = dtc2*x2 + angle;
  return angle;
}

// second-order complementary filter
// a=tau / (tau + loop time)
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Complementary(float newAngle, float newRate,int looptime, float angle) {
  float tau=0.075;
  float a=0.0;
  float dtC = float(looptime)/1000.0;
  a=tau/(tau+dtC);
  angle= a* (angle + newRate * dtC) + (1-a) * (newAngle);
  return angle;
}

// Kalman filter                                      
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Kalman(float newAngle, float newRate,int looptime, float x_angle)
{
  float Q_angle  =  0.01; //0.001
  float Q_gyro   =  0.0003;  //0.003
  float R_angle  =  0.01;  //0.03

  float x_bias = 0;
  float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
  float  y, S;
  float K_0, K_1;

  float dt = float(looptime)/1000;
  x_angle += dt * (newRate - x_bias);
  P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
  P_01 +=  - dt * P_11;
  P_10 +=  - dt * P_11;
  P_11 +=  + Q_gyro * dt;

  y = newAngle - x_angle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;

  x_angle +=  K_0 * y;
  x_bias  +=  K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;

  return x_angle;
}

// scale setangle, so that both PI angles have the same sign    
float scalePIangles(float setAngle, float currAngle){
  if ((setAngle >= PI/2) && (currAngle <= -PI/2)) return (setAngle-2*PI);
    else if ((setAngle <= -PI/2) && (currAngle >= PI/2)) return (setAngle+2*PI);
    else return setAngle;
}

void IMU::update(){
  read();  
  now = millis();
  int looptime = (now - lastAHRSTime);
  lastAHRSTime = now;
/*
  Serial.print("Gyro x,y: ");
  Serial.print(gyro.x);
  Serial.print(" , ");
  Serial.print(gyro.y);


  Serial.print("  Acc x,y,z: ");
  Serial.print(acc.x);
  Serial.print(" , ");
  Serial.print(acc.y);
  Serial.print(" , ");
  Serial.print(acc.z);


  Serial.print("  com x,y,z: ");
  Serial.print(com.x);
  Serial.print(" , ");
  Serial.print(com.y);
  Serial.print(" , ");
  Serial.println(com.z);
*/

  if (state == IMU_RUN){
    // ------ roll, pitch --------------  
    float forceMagnitudeApprox = abs(acc.x) + abs(acc.y) + abs(acc.z);    
    //if (forceMagnitudeApprox < 1.2) {
      //Console.println(forceMagnitudeApprox);      
      accPitch   = atan2(-acc.x , sqrt(sq(acc.y) + sq(acc.z)));         
      accRoll    = atan2(acc.y , acc.z);       
      accPitch = scalePIangles(accPitch, ypr.pitch);
      accRoll  = scalePIangles(accRoll, ypr.roll);
      // complementary filter            
      ypr.pitch = Kalman(accPitch, gyro.x, looptime, ypr.pitch);  
      ypr.roll  = Kalman(accRoll,  gyro.y, looptime, ypr.roll);            
    /*} else {
      //Console.print("too much acceleration ");
      //Console.println(forceMagnitudeApprox);
      ypr.pitch = ypr.pitch + gyro.y * ((float)(looptime))/1000.0;
      ypr.roll  = ypr.roll  + gyro.x * ((float)(looptime))/1000.0;
    }*/
    ypr.pitch=scalePI(ypr.pitch);
    ypr.roll=scalePI(ypr.roll);
    // ------ yaw --------------
    // tilt-compensated yaw HMC5883L
    //comTilt.x =  com.x  * cos(ypr.pitch) + com.z * sin(ypr.pitch);
    //comTilt.y =  com.x  * sin(ypr.roll)         * sin(ypr.pitch) + com.y * cos(ypr.roll) - com.z * sin(ypr.roll) * cos(ypr.pitch);
    //comTilt.z = -com.x  * cos(ypr.roll)         * sin(ypr.pitch) + com.y * sin(ypr.roll) + com.z * cos(ypr.roll) * cos(ypr.pitch);     
    //comYaw = scalePI( atan2(comTilt.y, comTilt.x)  );  

    // tilt-compensated yaw MMC5883MA
    comTilt.x =  com.x  * cos(ypr.pitch) - com.x * sin(ypr.pitch);
    comTilt.y =  com.x  * cos(ypr.roll) * sin(ypr.pitch) + com.y * cos(ypr.roll) + com.z * sin(ypr.roll) * cos(ypr.pitch);
    comTilt.z = -com.x  * cos(ypr.roll)         * sin(ypr.pitch) + com.y * sin(ypr.roll) + com.z * cos(ypr.roll) * cos(ypr.pitch);     
    comYaw = scalePI( atan2(comTilt.y, comTilt.x) );  

/*
    if (comTilt.x > 0) comYaw = scalePI( atan(-(comTilt.y/comTilt.x))) ;  
    if (comTilt.x < 0 && comTilt.y >= 0 ) comYaw = scalePI( atan(-(comTilt.y/comTilt.x))+M_PI);  
    if (comTilt.x < 0 && comTilt.y  < 0 ) comYaw = scalePI( atan(-(comTilt.y/comTilt.x))-M_PI);  
    if (comTilt.x == 0 && comTilt.y > 0 ) comYaw = scalePI(+M_PI/2);
    if (comTilt.x == 0 && comTilt.y < 0 ) comYaw = scalePI(-M_PI/2);
    if (comTilt.x == 0 && comTilt.y == 0 ) comYaw = 0;
*/

    comYaw = scalePIangles(comYaw, ypr.yaw);
    //comYaw = atan2(com.y, com.x);  // assume pitch, roll are 0
    // complementary filter
    ypr.yaw = Complementary2(comYaw, -gyro.z, looptime, ypr.yaw);
    ypr.yaw = scalePI(ypr.yaw);
  } 
  else if (state == IMU_CAL_COM) {
    calibComUpdate();
  }
}  

boolean IMU::init(){    
  loadCalib();
  printCalib();    
  if (!initL3G4200D()) return false;
  initADXL345B();
  //if (COMPASSTOUSE == 0) initHMC5883L();
  //if (COMPASSTOUSE == 1) 
  initMMC5883MA();

  now = 0;  
  hardwareInitialized = true;
  return true;
}

int IMU::getCallCounter(){
  int res = callCounter;
  callCounter = 0;
  return res;
}

int IMU::getErrorCounter(){
  int res = errorCounter;
  errorCounter = 0;
  return res;
}

void IMU::read(){  
  if (!hardwareInitialized) {
    errorCounter++;
    return;
  }
  callCounter++;    
  readL3G4200D(true);
  readADXL345B();
  //if (COMPASSTOUSE == 0) readHMC5883L();
  //if (COMPASSTOUSE == 1) 
  readMMC5883MA();
  //calcComCal();
}


