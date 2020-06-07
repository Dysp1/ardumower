/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2019 by Marko Riihimaki
 
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
  
  You are free to use the code in this class for commercial or personal use at your will.
*/

#ifndef mmc5883ma_h
#define mmc5883ma_h

#include "Arduino.h"

//*****************************************************
// MMC5883MA Register map
//*****************************************************
#define MMC5883MA (0x30)         // Sensor I2C address

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

#define MMC5883MA_OFFSET       32768
#define MMC5883MA_SENSITIVITY   4096
//*****************************************************
// MMC5883MA Register map end
//*****************************************************


class mmc5883ma
{
  
  private:
    float _bridgeOffsetX, _bridgeOffsetY, _bridgeOffsetZ;
    void calculateBridgeOffsets();
    float _magx;
    float _magy;
    float _magz;
    float _comOfsx;
    float _comOfsy;
    float _comOfsz;
    float _comScalex;
    float _comScaley;
    float _comScalez;
    uint8_t _readingMode;
    int initManualMode();
    int initContinuousMode();
    bool readMagsManualMode(bool);    
    bool readMagsContinuousMode(bool);    

  public:
    int init(uint8_t);
    int init();
    bool readMags(bool);
    uint8_t readTemperature();

    void calibrate();

    float x();
    float y();
    float z();

    void setCalibOffsetX(float);
    void setCalibOffsetY(float);
    void setCalibOffsetZ(float);
    float getCalibOffsetX();
    float getCalibOffsetY();
    float getCalibOffsetZ();

    void setCalibScaleX(float);
    void setCalibScaleY(float);
    void setCalibScaleZ(float);
    float getCalibScaleX();
    float getCalibScaleY();
    float getCalibScaleZ();
};

#endif

