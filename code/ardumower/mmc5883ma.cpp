#include "Arduino.h"
#include "mmc5883ma.h"
#include <Wire.h>
#include "i2c.h"
#include "buzzer.h"



float mmc5883ma::x(){
  return _magX;
}    

float mmc5883ma::y(){
  return _magY;
}    

float mmc5883ma::z(){
  return _magZ;
}    

void mmc5883ma::setMaxX(float myMaxX){
  _maxX = myMaxX;
}

void mmc5883ma::setMaxY(float myMaxY){
  _maxY = myMaxY;
}

void mmc5883ma::setMaxZ(float myMaxZ){
  _maxZ = myMaxZ;
}

float mmc5883ma::getMaxX(){
  return _maxX;
}

float mmc5883ma::getMaxY(){
  return _maxY;
}

float mmc5883ma::getMaxZ(){
  return _maxZ;
}

void mmc5883ma::setMinX(float myMinX){
  _minX = myMinX;
}

void mmc5883ma::setMinY(float myMinY){
  _minY = myMinY;
}

void mmc5883ma::setMinZ(float myMinZ){
  _minZ = myMinZ;
}

float mmc5883ma::getMinX(){
  return _minX;
}

float mmc5883ma::getMinY(){
  return _minY;
}

float mmc5883ma::getMinZ(){
  return _minZ;
}

void mmc5883ma::setAccX(float myAccX){
  _accX = myAccX;
}

void mmc5883ma::setAccY(float myAccY){
  _accY = myAccY;
}

void mmc5883ma::setAccZ(float myAccZ){
  _accZ = myAccZ;
}

void mmc5883ma::setDeclination(float myDeclination) {
  _declination = myDeclination;
}

float mmc5883ma::getHeading(){
  
  //Normalize accelerometer raw values.
  float accXnorm = _accX/sqrt(_accX * _accX + _accY * _accY + _accZ * _accZ);
  float accYnorm = _accY/sqrt(_accX * _accX + _accY * _accY + _accZ * _accZ);

    //Calculate pitch and roll
  float pitch = asin(accXnorm);
  float roll = -asin(accYnorm/cos(pitch));

    //Calculate the new tilt compensated values
  float magXcomp = _magX*cos(pitch)+_magZ*sin(pitch);
  float magYcomp = _magX*sin(roll)*sin(pitch)+_magY*cos(roll)-_magZ*sin(roll)*cos(pitch); // LSM9DS1

  //Calculate heading
  float heading = 180*atan2(magYcomp,magXcomp)/M_PI;

  //Convert heading to 0 - 360
  if(heading < 0)
    heading += 360;

  if (_declination != 0) heading += _declination;

  if(heading > 360)
    heading -= 360;

  return heading*M_PI/180;
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

  _maxX = -128000; 
  _maxY = -128000;
  _maxZ = -128000;

  _minX = 128000;
  _minY = 128000;
  _minZ = 128000;

  float endTimeAfterNoMoreFound = 20000;   // end calibration if no new max/min found in x ms
  float informFrequencyOfCalibStop = 5000; // Inform about ending calibration in x ms
  float stoppingSteps = informFrequencyOfCalibStop; // first reminder after after no new found

  Serial.println("MMC5883MA Compass calibration:");
  Serial.println("Please move the compass slowly 360 degrees around all three axes.");
  Serial.println("Calibration will end automaticly after 120 seconds");
  Serial.print("or after maximum of ");
  Serial.print((endTimeAfterNoMoreFound/1000));
  Serial.println(" seconds of no new points found (beebs)");
  Serial.println("------------------------------------");
  Serial.println("Calibration will start in 5 seconds.");
  
  // Little countdown until starting calibration
  for (uint8_t i=4; i>0; i--){
    delay(1000);
    Serial.println(i);
  }

  Serial.println("Start moving the Compass!");

  float calibStartTime = millis();
  float lastNewFoundTime = millis();

  while (millis() < (calibStartTime + 120000)) { // end calibration after maximum of x ms
    delay(200);
    Buzzer.noTone();

    readMags(false);
    
    bool newFound = false;

    if (_magX > _maxX) { _maxX = _magX; newFound = true; }
    if (_magY > _maxY) { _maxY = _magY; newFound = true; }
    if (_magZ > _maxZ) { _maxZ = _magZ; newFound = true; }

    if (_magX < _minX) { _minX = _magX; newFound = true; }
    if (_magY < _minY) { _minY = _magY; newFound = true; }
    if (_magZ < _minZ) { _minZ = _magZ; newFound = true; }

    if (newFound == true) {
      lastNewFoundTime = millis();
      stoppingSteps = informFrequencyOfCalibStop;
      Buzzer.tone(440); //beeb if new min/max found
      Serial.print("X max/min: ");
      Serial.print(_maxX);
      Serial.print(",");
      Serial.print(_minX);
      Serial.print(" - Y max/min: ");
      Serial.print(_maxY);
      Serial.print(",");
      Serial.print(_minY);
      Serial.print(" - Z max/min: ");
      Serial.print(_maxZ);
      Serial.print(",");
      Serial.println(_minZ);
    } else {
      // informing between 5 seconds that the calibration is about to end
      if ((millis() - lastNewFoundTime) > stoppingSteps) {
        Buzzer.tone(800);
        if ( stoppingSteps > 500) {
          Serial.print("Auto stopping calibration if no new points found in ");
          Serial.print(round((endTimeAfterNoMoreFound - stoppingSteps) / 1000));
          Serial.println(" seconds");
        }
        stoppingSteps += informFrequencyOfCalibStop;
      }
    }

    if (millis() > (lastNewFoundTime + endTimeAfterNoMoreFound)) {
          break;
    }
  }
  
  Serial.println("-----------------------------");
  Serial.println("Calibration values:");
  Serial.print("X max/min: ");
  Serial.print(_maxX);
  Serial.print(",");
  Serial.println(_minX);
  Serial.print("Y max/min: ");
  Serial.print(_maxY);
  Serial.print(",");
  Serial.println(_minY);
  Serial.print("Z max/min: ");
  Serial.print(_maxZ);
  Serial.print(",");
  Serial.println(_minZ);
  
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
  
  I2CwriteTo(MMC5883MA, INT_CTRL1, 0x80);  // software reset 5ms at least.
  delay(10);

  calculateBridgeOffsets();
  delay(10);

  // Read serial number 
  uint8_t serialNumber[2];  
  I2CreadFrom(MMC5883MA, 0x2F, 1, (uint8_t*)serialNumber);

  if (serialNumber[0] != 12){
    Serial.print("MMC5883MA manual mode initialization failed. Expected productID 12, got: ");
    Serial.println(serialNumber[0]);
    return false;
  } 

  // Initialize manual mode reading
  //
  // 0x00 set cmd resolution to 16bit,  10 ms*3, 100HZ
  // 0x01 set cmd resolution to 16bit,   5 ms*3, 200HZ
  // 0x02 set cmd resolution to 16bit, 2.5 ms*3, 400HZ
  // 0x03 set cmd resolution to 16bit, 1.6 ms*3, 600HZ
  //
  I2CwriteTo(MMC5883MA, INT_CTRL1, 0x01);  
  delay(1);

  Serial.print("MMC5883MA manual mode initialisation successful. Compass serialNumber: ");
  Serial.println(serialNumber[0]);

  return true;
}

int mmc5883ma::initContinuousMode(){
/*
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
*/
  return true;
}



float mmc5883ma::getTemperature(){    
  // failsafe for not measuring temperature too often 
  if(_nextTimeMeasureTemperature > millis()) {
    return _caseTemperature;
  } else {
    _nextTimeMeasureTemperature = millis() + _temperatureMeasureInterval;
  }

  uint8_t temperatureBuffer[1];
  uint8_t temperatureStatusBuffer[1];
  byte temperatureMask = 0x02;
  float startTime = millis();

  // We must not change control register 0 when automatic mode enabled
  if (_readingMode != 2) { 
    //if we are in manual mode, we will need set control register 0 bit 2 to begin temperature measurement
    I2CwriteTo(MMC5883MA, INT_CTRL0, 0x02);
    delay(1);
  }
  
  // we will read the STATUS register and monitor bit 2. When bit 2 is set to 1, measurement is ready to read
  I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)temperatureStatusBuffer);
  while ( (temperatureStatusBuffer[0] & temperatureMask) != temperatureMask && (millis() < (startTime+50))) {  
    delay(2);
    I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)temperatureStatusBuffer);
  }

  //We will read 1 byte from the TEMPERATURE register 0x06
  if ((temperatureStatusBuffer[0] & temperatureMask) == temperatureMask) {
    if (I2CreadFrom(MMC5883MA, TEMPERATURE, 1, (uint8_t*)temperatureBuffer) == 1) {
      _caseTemperature = ((float)temperatureBuffer[0]*0.69-71.2); //each step is about 0,69 degrees C and steps start from about 71.2 degrees C
      return _caseTemperature;
    }  
  } 

  return -999;  // returning invalid temperature if measurement failed for any reason.
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

  uint8_t measureBuffer[6];  
  uint8_t magnetometerStatusBuffer[1];  
  byte magnetometerMask = 0x01;

  if (I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)magnetometerStatusBuffer) == 1) {

    if ( (magnetometerStatusBuffer[0] & magnetometerMask) == magnetometerMask ) {  
  
      if (I2CreadFrom(MMC5883MA, 0x00, 6, (uint8_t*)measureBuffer) == 6) 
      {
        float magnetoX1 = (uint16_t)(measureBuffer[1]<< 8 | measureBuffer[0]);
        float magnetoY1 = (uint16_t)(measureBuffer[3]<< 8 | measureBuffer[2]);
        float magnetoZ1 = (uint16_t)(measureBuffer[5]<< 8 | measureBuffer[4]);
      
        if (abs(magnetoX1 - MMC5883MA_OFFSET) < 10000     // I Find MMC5883MA to be very unreliable at times
         && abs(magnetoY1 - MMC5883MA_OFFSET) < 10000     // the scale should be in -2048 to 2048 range.
         && abs(magnetoZ1 - MMC5883MA_OFFSET) < 10000) {  // That is why we will skip all clearly (faulty) readings. Especially important when calibrating!
  
          magnetoX1 = ((float)magnetoX1 - MMC5883MA_OFFSET); // /MMC5883MA_SENSITIVITY;
          magnetoY1 = ((float)magnetoY1 - MMC5883MA_OFFSET); // /MMC5883MA_SENSITIVITY;
          magnetoZ1 = ((float)magnetoZ1 - MMC5883MA_OFFSET); // /MMC5883MA_SENSITIVITY;
  
          if (_useComCalibration){
  
            //hard iron
            magnetoX1 -= ((_minX + _maxX)/2) / (_maxX - _minX) * 2 - 1;
            magnetoY1 -= ((_minY + _maxY)/2) / (_maxY - _minY) * 2 - 1;
            magnetoZ1 -= ((_minZ + _maxZ)/2) / (_maxZ - _minZ) * 2 - 1;
  
            //soft iron
            _magX = (magnetoX1 - _minX) / (_maxX - _minX) * 2 - 1;
            _magY = (magnetoY1 - _minY) / (_maxY - _minY) * 2 - 1;
            _magZ = (magnetoZ1 - _minZ) / (_maxZ - _minZ) * 2 - 1;
  
          } else {
            _magX = magnetoX1;
            _magY = magnetoY1;
            _magZ = magnetoZ1;
          }
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

  I2CwriteTo(MMC5883MA, INT_CTRL0, 0x01);
  delay(2);
  
  uint8_t measure1Buffer[6];  
  uint8_t measure1bufferStatus[6];  
  byte measureMask = 0x01;
  float startTime = millis();
  measure1bufferStatus[0] = 0;

  while ( ((measure1bufferStatus[0] & measureMask) != measureMask) && (millis() < (startTime+50))) {  
    delay(2);
    I2CreadFrom(MMC5883MA, STATUS, 1, (uint8_t*)measure1bufferStatus);
  }
  
  if ((measure1bufferStatus[0] & measureMask) == measureMask) 
  {
    if (I2CreadFrom(MMC5883MA, 0x00, 6, (uint8_t*)measure1Buffer) == 6) 
    {
      float magnetoX1 = (uint16_t)(measure1Buffer[1]<< 8 | measure1Buffer[0]);
      float magnetoY1 = (uint16_t)(measure1Buffer[3]<< 8 | measure1Buffer[2]);
      float magnetoZ1 = (uint16_t)(measure1Buffer[5]<< 8 | measure1Buffer[4]);
    
      if (abs(magnetoX1 - MMC5883MA_OFFSET) < 10000     // I Find MMC5883MA to be very unreliable at times
       && abs(magnetoY1 - MMC5883MA_OFFSET) < 10000     // the scale should be in -2048 to 2048 range.
       && abs(magnetoZ1 - MMC5883MA_OFFSET) < 10000) {  // That is why we will skip all clearly (faulty) readings. Especially important when calibrating!

        magnetoX1 = ((float)magnetoX1 - MMC5883MA_OFFSET); // /MMC5883MA_SENSITIVITY;
        magnetoY1 = ((float)magnetoY1 - MMC5883MA_OFFSET); // /MMC5883MA_SENSITIVITY;
        magnetoZ1 = ((float)magnetoZ1 - MMC5883MA_OFFSET); // /MMC5883MA_SENSITIVITY;

        if (_useComCalibration){

          //hard iron
          magnetoX1 -= ((_minX + _maxX)/2) / (_maxX - _minX) * 2 - 1;
          magnetoY1 -= ((_minY + _maxY)/2) / (_maxY - _minY) * 2 - 1;
          magnetoZ1 -= ((_minZ + _maxZ)/2) / (_maxZ - _minZ) * 2 - 1;

          //soft iron
          _magX = (magnetoX1 - _minX) / (_maxX - _minX) * 2 - 1;
          _magY = (magnetoY1 - _minY) / (_maxY - _minY) * 2 - 1;
          _magZ = (magnetoZ1 - _minZ) / (_maxZ - _minZ) * 2 - 1;

        } else {
          _magX = magnetoX1;
          _magY = magnetoY1;
          _magZ = magnetoZ1;
        }
      }
    } else return false;
  } else return false;
  
/*
  Serial.print("x,y,z: ");
  Serial.print(_magX);
  Serial.print(" , ");
  Serial.print(_magY);
  Serial.print(" , ");
  Serial.println(_magZ);
*/

  return true;
}
