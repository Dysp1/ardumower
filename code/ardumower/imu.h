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

/* pitch/roll and heading estimation (IMU sensor fusion)  
   requires: GY-80 module (L3G4200D, ADXL345B, HMC5883L) 
   
How to use it (example):     
  1. initialize IMU:                 IMU imu;  imu.init(); 
  2. read IMU (yaw/pitch/roll:       Serial.println( imu.ypr.yaw );
*/


#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

#include "MPU9250.h"

// IMU state
enum { IMU_RUN, IMU_CAL_COM };


struct point_int_t {
  int16_t x;
  int16_t y;
  int16_t z;
};
typedef struct point_int_t point_int_t;

struct point_long_t {
  long x;
  long y;
  long z;
};
typedef struct point_long_t point_long_t;

struct point_float_t {
  float x;
  float y;
  float z;
};
typedef struct point_float_t point_float_t;

struct ypr_t {
  float yaw;
  float pitch;
  float roll;
};
typedef struct ypr_t ypr_t;

class IMU
{
public:
  IMU();    
  boolean init();   
  void update();  
  int getCallCounter();
  int getErrorCounter();
  void deleteCalib();  
  float degreesToRads (float degrees);
  float radsToDegrees (float rads);


  int callCounter;
  int errorCounter;
  boolean hardwareInitialized;  
  byte state;
  unsigned long lastAHRSTime;
  unsigned long now;  
  ypr_t ypr;  // gyro yaw,pitch,roll    
  // --------- gyro state -----------------------------
  point_float_t gyro;   // gyro sensor data (degree)    
  point_float_t gyroOfs; // gyro calibration data
  unsigned long lastGyroTime;
  // --------- acceleration state ---------------------
  point_float_t acc;  // acceleration sensor data
  point_float_t accOfs;
  point_float_t accScale;
  // calibrate acceleration sensor  
  boolean calibAccNextAxis();  
  boolean calibrationAvail;
  // --------- compass state --------------------------  
  point_float_t com; // compass sensor data (raw)
  point_float_t comOfs;
  point_float_t comScale;
  uint8_t comTemperature;

  MPU9250 mpu;
  
  float comYaw;         // compass heading (radiant, raw)
  boolean useComCalibration;
  // calibrate compass sensor  
  void calibComStartStop();  
  void calibComUpdate();    
  boolean newMinMaxFound();
private:  
  void read();
  float smoothedYawRads(float newYaw);
  void loadSaveCalib(boolean readflag);  
  void loadCalib();  
  // print IMU values
  void printPt(point_float_t p);
  void printCalib();
  void saveCalib();

  boolean foundNewMinMax;
};




#endif

