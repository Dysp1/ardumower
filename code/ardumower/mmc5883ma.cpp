#include "Arduino.h"
#include "mmc5883ma.h"
#include <Wire.h>
#include "i2c.h"
#include "buzzer.h"



float mmc5883ma::x(){
  return _magx;
}    

float mmc5883ma::y(){
  return _magy;
}    

float mmc5883ma::z(){
  return _magz;
}    

void mmc5883ma::setCalibOffsetX(float myComOfsx){
  _comOfsx = myComOfsx;
}

void mmc5883ma::setCalibOffsetY(float myComOfsy){
  _comOfsy = myComOfsy;
}

void mmc5883ma::setCalibOffsetZ(float myComOfsz){
  _comOfsz = myComOfsz;
}

float mmc5883ma::getCalibOffsetX(){
  return _comOfsx;
}

float mmc5883ma::getCalibOffsetY(){
  return _comOfsy;
}

float mmc5883ma::getCalibOffsetZ(){
  return _comOfsz;
}

void mmc5883ma::setCalibScaleX(float mycomScalex){
  _comScalex = mycomScalex;
}

void mmc5883ma::setCalibScaleY(float mycomScaley){
  _comScaley = mycomScaley;
}

void mmc5883ma::setCalibScaleZ(float mycomScalez){
  _comScalez = mycomScalez;
}

float mmc5883ma::getCalibScaleX(){
  return _comScalex;
}

float mmc5883ma::getCalibScaleY(){
  return _comScaley;
}

float mmc5883ma::getCalibScaleZ(){
  return _comScalez;
}


void  mmc5883ma::calculateBridgeOffsets(){
  uint8_t buf[6]; 
  uint8_t buf2[2];  

  ///////////////////////////////
  // Bridge offset calculation //
  ///////////////////////////////

  // RESET 
  I2CwriteTo(MMC5883MA, INT_CTRL0, 0x10);  
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
  
  float x1 = (uint16_t)(buf[1]<< 8 | buf[0]);
  float y1 = (uint16_t)(buf[3]<< 8 | buf[2]);
  float z1 = (uint16_t)(buf[5]<< 8 | buf[4]);
  
  x1 = ((float)x1 - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
  y1 = ((float)y1 - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
  z1 = ((float)z1 - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
  
  delay(50);
  // SET
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
  
  _bridgeOffsetX = (x2+x1)/2;
  _bridgeOffsetY = (y2+y1)/2;
  _bridgeOffsetZ = (z2+y1)/2;

//  Serial.print("MMC5883MA Bridge Offsets x,y,z:");
//  Serial.print(_bridgeOffsetX);
//  Serial.print(",");
//  Serial.print(_bridgeOffsetY);
//  Serial.print(",");
//  Serial.println(_bridgeOffsetZ);
}

void mmc5883ma::calibrate() 
{
  uint16_t ii = 0, sample_count = 0;
  float mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  float mag_max[3] = {-132767, -132767, -132767}, mag_min[3] = {132767, 132767, 132767}, mag_temp[3] = {0, 0, 0};
  float dest1[3];
  float dest2[3];
  
  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);
  
  // shoot for ~30 seconds of mag data
  sample_count = 400;  // at 14 Hz ODR, new mag data is available every 71,4 ms
  
  for(ii = 0; ii < sample_count; ii++) {
    readMags(false);  // Read the mag data
    mag_temp[0] = _magx;
    mag_temp[1] = _magy;
    mag_temp[2] = _magz;   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
      Serial.print(mag_temp[jj]);
      Serial.print(" ");
    }
    Serial.println();
    delay(75);  // at 14 Hz ODR, new mag data is available every 71,4 ms
    Buzzer.tone(440);
  }
    
  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
  
  dest1[0] = (float) mag_bias[0];//*0.25;  // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1];//*0.25;   
  dest1[2] = (float) mag_bias[2];//*0.25;   //the full scale range of ±8 gauss (G), with 0.25mG per LSB resolution 
                                            //for 16 bits operation mode and 0.4mG total RMS noise level,
                                            //enabling heading accuracy of 1°in electronic compass applications.
    
  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts
  
  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;
  
  dest2[0] = avg_rad/((float)mag_scale[0]);
  dest2[1] = avg_rad/((float)mag_scale[1]);
  dest2[2] = avg_rad/((float)mag_scale[2]);
  
  Serial.print(dest1[0]);
  Serial.print(" , ");
  Serial.print(dest1[1]);
  Serial.print(" , ");
  Serial.print(dest1[2]);
  Serial.print(" , ");
  Serial.print(dest2[0]);
  Serial.print(" , ");
  Serial.print(dest2[1]);
  Serial.print(" , ");
  Serial.println(dest2[2]);

  _comOfsx = dest1[0];
  _comOfsy = dest1[1];
  _comOfsz = dest1[2];
  _comScalex = dest2[0];
  _comScaley = dest2[1];
  _comScalez = dest2[2];

  // completed sound
  Buzzer.tone(600);
  delay(200); 
  Buzzer.tone(880);
  delay(200); 
  Buzzer.tone(1320);              
  delay(200); 
  Buzzer.noTone();

  Serial.println("Mag Calibration done!");
 }


// Manual reading mode = 1
// Continuous mode = 2
int mmc5883ma::init(uint8_t mode){
  if (mode == 2) {
    _readingMode = 2;
    return initContinuousMode();
  } else {
    _readingMode = 1;
    return initManualMode();
  }
}

// Manual reading mode = 1
int mmc5883ma::init(){
  _readingMode = 1;
  return initManualMode();
}

int mmc5883ma::initManualMode(){
 
  delay(5);  // allow the chip to online for sure

  // Read serial number 
  uint8_t serialNumber[2];  
  I2CreadFrom(MMC5883MA, 0x2F, 1, (uint8_t*)serialNumber);

  if (serialNumber[0] != 12){
    Serial.print("MMC5883MA manual mode initialization failed. Expected productID 12, got: ");
    Serial.println(serialNumber[0]);
    return 0;
  }

  // 0x00 set cmd resolution to 16bit,  10 ms*3, 100HZ
  // 0x01 set cmd resolution to 16bit,   5 ms*3, 200HZ
  // 0x02 set cmd resolution to 16bit, 2.5 ms*3, 400HZ
  // 0x03 set cmd resolution to 16bit, 1.6 ms*3, 600HZ
  I2CwriteTo(MMC5883MA, INT_CTRL1, 0x01);  
  delay(1);

  Serial.print("MMC5883MA manual mode initialisation successful. Compass serialNumber: ");
  Serial.println(serialNumber[0]);

  return 1;
}

int mmc5883ma::initContinuousMode(){

  // continuous measurement mode
  delay(5);  // allow the chip to online for sure

  I2CwriteTo(MMC5883MA, INT_CTRL1, 0x80);  // software reset
  delay(20);


  uint8_t serialNumber[1];  
  I2CreadFrom(MMC5883MA, 0x2F, 1, (uint8_t*)serialNumber);

  // could not read the correct serial number. Intiliasation failed
  if (serialNumber[0] != 12){
    Serial.print("MMC5883MA continuous mode initialization failed. Expected productID 12, got: ");
    Serial.println(serialNumber[0]);
    return false;
  }

  calculateBridgeOffsets(); // performs reset - set

  I2CwriteTo(MMC5883MA, INT_CTRL1, 0x00);  
  delay(1);

  I2CwriteTo(MMC5883MA, INT_CTRL2, 0x41);  // start continuous measurement at 14Hz with INT_Meas_Done_En interrupt
  delay(1);

  Serial.print("MMC5883MA continuous mode initialisation successful. Compass serialNumber: ");
  Serial.println(serialNumber[0]);

  return true;
}



uint8_t mmc5883ma::readTemperature(){    

  uint8_t temperatureBuffer[1];
  uint8_t temperatureStatusBuffer[1];
  byte temperatureMask = 0x02;

  I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)temperatureStatusBuffer);
  if ( (temperatureStatusBuffer[0] & temperatureMask) == temperatureMask ) {  
    I2CreadFrom(MMC5883MA, TEMPERATURE, 1, (uint8_t*)temperatureBuffer);
  }

  return (uint8_t)round(temperatureBuffer[0]*0.69-71.2);
}



bool mmc5883ma::readMags(bool _useComCalibration){    
  if (_readingMode == 2) {
    if (readMagsContinuousMode(_useComCalibration) == false) {
      init(2);
      return false;
    }
  } else {
    if (readMagsManualMode(_useComCalibration) == false ) {
      init();  
      return false;
    }
  }
  return true;
}


bool mmc5883ma::readMagsContinuousMode(bool _useComCalibration){    

  uint8_t magnetometerBuffer[7];  
  uint8_t magnetometerStatusBuffer[2];  
  byte magnetometerMask = 0x01;

  if (I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)magnetometerStatusBuffer) == 1) {
    if ( (magnetometerStatusBuffer[0] & magnetometerMask) == magnetometerMask ) {  
  
        if( I2CreadFrom(MMC5883MA, 0x00, 6, (uint8_t*)magnetometerBuffer) == 6) {
        
        _magx = (uint16_t)(magnetometerBuffer[1]<< 8 | magnetometerBuffer[0]);
        _magy = (uint16_t)(magnetometerBuffer[3]<< 8 | magnetometerBuffer[2]);
        _magz = (uint16_t)(magnetometerBuffer[5]<< 8 | magnetometerBuffer[4]);
    
        _magx = ((float)_magx - MMC5883MA_OFFSET) / MMC5883MA_SENSITIVITY - _bridgeOffsetX;
        _magy = ((float)_magy - MMC5883MA_OFFSET) / MMC5883MA_SENSITIVITY - _bridgeOffsetY;
        _magz = ((float)_magz - MMC5883MA_OFFSET) / MMC5883MA_SENSITIVITY - _bridgeOffsetZ;
    
        if (_useComCalibration){
          _magx -= _comOfsx;
          _magy -= _comOfsy;
          _magz -= _comOfsz;
          if (_comScalex != 0) _magx /= _comScalex*0.5;    
          if (_comScaley != 0) _magy /= _comScaley*0.5;    
          if (_comScalez != 0) _magz /= _comScalez*0.5;
        }
      } else {
        Serial.println("MMC5883MA Could not read 6 bytes from mag x,y,z registers.");
        return false;
      }
    } else {
      Serial.println("MMC5883MA Could not read 1 byte from status register.");
      return false;
    }
  } else {
    Serial.println("MMC5883MA Status buffer 0");
    return false;
  }
  return true;
}

bool mmc5883ma::readMagsManualMode(bool _useComCalibration){    
    float magnetoX1;
    float magnetoY1;
    float magnetoZ1;
    float magnetoX2;
    float magnetoY2;
    float magnetoZ2;

    // reset 
    I2CwriteTo(MMC5883MA, INT_CTRL0, 0x08);  
    delay(1);
  
    //measurement 1
    I2CwriteTo(MMC5883MA, INT_CTRL0, 0x01);
    delay(2);

    uint8_t measure1Buffer[6];  
    uint8_t measure1bufferStatus[6];  
    byte measure1Mask = 0x01;

    float startTime = millis();
    measure1bufferStatus[0] = 0;

    while ( (measure1bufferStatus[0] & measure1Mask) != measure1Mask || millis() > startTime+50) {  
      if (I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)measure1bufferStatus) != 1) {
        return false;
      } 
    }

    if ((measure1bufferStatus[0] & measure1Mask) != measure1Mask) return false;

    if (I2CreadFrom(MMC5883MA, 0x00, 6, (uint8_t*)measure1Buffer) == 6) {
      magnetoX1 = (uint16_t)(measure1Buffer[1]<< 8 | measure1Buffer[0]);
      magnetoY1 = (uint16_t)(measure1Buffer[3]<< 8 | measure1Buffer[2]);
      magnetoZ1 = (uint16_t)(measure1Buffer[5]<< 8 | measure1Buffer[4]);
    } else {
      return false;
    }

    magnetoX1 = -1 * ((float)magnetoX1 - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
    magnetoY1 = -1 * ((float)magnetoY1 - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
    magnetoZ1 = -1 * ((float)magnetoZ1 - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
  
    // set
    I2CwriteTo(MMC5883MA, INT_CTRL0, 0x10);
    delay(1);

    //measurement 2
    I2CwriteTo(MMC5883MA, INT_CTRL0, 0x01);
    delay(2);

    uint8_t measure2Buffer[6];  
    uint8_t measure2bufferStatus[6];  
    byte measure2Mask = 0x01;

    startTime = millis();
    measure2bufferStatus[0] = 0;

    while ( (measure2bufferStatus[0] & measure2Mask) != measure2Mask || millis() > startTime+50 ) {  
      if (I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)measure2bufferStatus) != 1) {
        return false;
      }
    }

    if ((measure2bufferStatus[0] & measure2Mask) != measure2Mask) return false;

    if (I2CreadFrom(MMC5883MA, 0x00, 6, (uint8_t*)measure2Buffer) == 6) {
      magnetoX2 = (uint16_t)(measure2Buffer[1]<< 8 | measure2Buffer[0]);
      magnetoY2 = (uint16_t)(measure2Buffer[3]<< 8 | measure2Buffer[2]);
      magnetoZ2 = (uint16_t)(measure2Buffer[5]<< 8 | measure2Buffer[4]);
    } else {
      return false;
    }

    magnetoX2 = -1 * ((float)magnetoX2 - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
    magnetoY2 = -1 * ((float)magnetoY2 - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;
    magnetoZ2 = -1 * ((float)magnetoZ2 - MMC5883MA_OFFSET)/MMC5883MA_SENSITIVITY;

    float magnetoFinalX = (magnetoX2 - magnetoX1) / 2;
    float magnetoFinalY = (magnetoY2 - magnetoY1) / 2;
    float magnetoFinalZ = (magnetoZ2 - magnetoZ1) / 2;

    if (_useComCalibration){
      magnetoFinalX -= _comOfsx;
      magnetoFinalY -= _comOfsy;
      magnetoFinalZ -= _comOfsz;
      magnetoFinalX /= _comScalex*0.5;    
      magnetoFinalY /= _comScaley*0.5;    
      magnetoFinalZ /= _comScalez*0.5;
      _magx = magnetoFinalX;
      _magy = magnetoFinalY;
      _magz = magnetoFinalZ;
    } else {
      _magx = magnetoFinalX;
      _magy = magnetoFinalY;
      _magz = magnetoFinalZ;
    }

    return true;
}
