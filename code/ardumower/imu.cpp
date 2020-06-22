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
#include "MPU9250.h"

#define ADDR 600
#define MAGIC 6

IMU::IMU(){
  hardwareInitialized = false;
  calibrationAvail = false;
  state = IMU_RUN;
  callCounter = 0;  
  errorCounter = 0;
  
  lastGyroTime = millis();
  
  ypr.yaw=ypr.pitch=ypr.roll = 0;
  
  com.x=com.y=com.z=0;  
  
  comScale.x=comScale.y=comScale.z=2;  
  comOfs.x=comOfs.y=comOfs.z=0;    
  useComCalibration = true;
  
  MPU9250 mpu;

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

  mpu.setMagBias(0,comOfs.x);
  mpu.setMagBias(1,comOfs.y);
  mpu.setMagBias(2,comOfs.z);
  mpu.setMagScale(0,comScale.x);
  mpu.setMagScale(1,comScale.y);
  mpu.setMagScale(2,comScale.z);
/*
    Serial.println("COM BIAS AND SCALE---------------------------------------");
    Serial.println(mpu.getMagBias(0));
    Serial.println(mpu.getMagBias(1));
    Serial.println(mpu.getMagBias(2));
    Serial.println(mpu.getMagScale(0));
    Serial.println(mpu.getMagScale(1));
    Serial.println(mpu.getMagScale(2));
    Serial.println("COM BIAS AND SCALE---------------------------------------");
*/
}

void IMU::saveCalib(){
/*
  #ifdef IMUMODEL
    Serial.println("COM BIAS AND SCALE---------------------------------------");
    Serial.println(mpu.getMagBias(0));
    Serial.println(mpu.getMagBias(1));
    Serial.println(mpu.getMagBias(2));
    Serial.println(mpu.getMagScale(0));
    Serial.println(mpu.getMagScale(1));
    Serial.println(mpu.getMagScale(2));
    Serial.println("COM BIAS AND SCALE---------------------------------------");
*/
    comOfs.x = mpu.getMagBias(0);
    comOfs.y = mpu.getMagBias(1);
    comOfs.z = mpu.getMagBias(2);
    comScale.x = mpu.getMagScale(0);
    comScale.y = mpu.getMagScale(1);
    comScale.z = mpu.getMagScale(2);

//  #endif    

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

void IMU::calibComStartStop(){  
  while (Console.available()) Console.read();  

  Console.println(F("com calib..."));
  Console.println(F("Rotate sensor 360 degrees around all three axis"));

  mpu.calibrateMag();
  saveCalib();  
  // completed sound
  Buzzer.tone(600);
  delay(200); 
  Buzzer.tone(880);
  delay(200); 
  Buzzer.tone(1320);              
  delay(200); 
  Buzzer.noTone();    
  delay(500);
  useComCalibration = true;
  state = IMU_RUN;    
}

void IMU::calibComUpdate(){
}

boolean IMU::calibAccNextAxis(){  
}

void IMU::update(){
  now = millis();
  int looptime = (now - lastAHRSTime);
  lastAHRSTime = now;

  if (state == IMU_RUN){
    read(); // not reading imu data while calibrating compass
      ypr.pitch = mpu.getPitch()*PI/180.0;
      ypr.roll  = mpu.getRoll()*PI/180.0;
      ypr.yaw   = mpu.getYaw()*PI/180.0;
      robot.caseTemperature = mpu.getTemperature();
  } else if (state == IMU_CAL_COM) {
    calibComUpdate();
  }
}  

boolean IMU::init(){
  loadCalib();
  
  mpu.setup();

  delay(500);

  mpu.calibrateAccelGyro();
  mpu.printCalibration();

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

  mpu.update();
/*
  Serial.print("roll:");
  Serial.print(mpu.getRoll());
  Serial.print(" pitch:");
  Serial.print(mpu.getPitch());
  Serial.print(" yaw:");
  Serial.println(mpu.getYaw());
*/
  gyro.x = mpu.getGyro(0)*PI/180.0;
  gyro.y = mpu.getGyro(1)*PI/180.0;
  gyro.z = mpu.getGyro(2)*PI/180.0;

  acc.x = mpu.getAcc(0)*PI/180.0;
  acc.y = mpu.getAcc(1)*PI/180.0;
  acc.z = mpu.getAcc(2)*PI/180.0;

  com.x = mpu.getMag(0)*PI/180.0;
  com.y = mpu.getMag(1)*PI/180.0;
  com.z = mpu.getMag(2)*PI/180.0;
}