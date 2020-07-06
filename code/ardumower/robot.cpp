/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2015 by Alexander Grau
  Copyright (c) 2013-2015 by Sven Gennat
  Copyright (c) 2014 by Maxime Carpentieri    
  Copyright (c) 2014-2015 by Stefan Manteuffel
  Copyright (c) 2015 by Uwe Zimprich
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

#include "robot.h"
#include "config.h"
#include "i2c.h"
#include "flashmem.h"

#define MAGIC 52


#define ADDR_USER_SETTINGS 0
#define ADDR_ERR_COUNTERS 400
#define ADDR_ROBOT_STATS 800

const char* stateNames[] ={"OFF ", "ROS", "RC  ", "FORW", "ROLL", "REV ", "CIRC", "ERR ", "PFND", "PTRK", "PROL", "PREV", "STAT", "CHARG", "STCHK",
  "STREV", "STROL", "STFOR", "MANU", "ROLW", "POUTFOR", "POUTREV", "POUTROLL", "TILT", "BUMPREV", "BUMPFORW", "GPSPERIMROLL", "GPSHOMEROLL", "GPSHOMING"};

const char* sensorNames[] ={"SEN_PERIM_LEFT", "SEN_PERIM_RIGHT", "SEN_PERIM_LEFT_EXTRA", "SEN_PERIM_RIGHT_EXTRA", "SEN_LAWN_FRONT", "SEN_LAWN_BACK", 
	"SEN_BAT_VOLTAGE", "SEN_CHG_CURRENT", "SEN_CHG_VOLTAGE", "SEN_MOTOR_LEFT", "SEN_MOTOR_RIGHT", "SEN_MOTOR_MOW", "SEN_BUMPER_LEFT", "SEN_BUMPER_RIGHT", 
	"SEN_DROP_LEFT", "SEN_DROP_RIGHT", "SEN_SONAR_CENTER", "SEN_SONAR_LEFT", "SEN_SONAR_RIGHT", "SEN_BUTTON", "SEN_IMU", "SEN_MOTOR_MOW_RPM", "SEN_RTC",
  "SEN_RAIN", "SEN_TILT", "SEN_FREE_WHEEL", "SEN_GPS_SPEED", "SEN_IMU_ACC", "SEN_MOW_POWER", "SEN_GPSPERIMETER"};

const char* errorNames[] ={"ERR_MOTOR_LEFT", "ERR_MOTOR_RIGHT", "ERR_MOTOR_MOW", "ERR_MOW_SENSE", "ERR_IMU_COMM", "ERR_IMU_TILT", "ERR_RTC_COMM",
  "ERR_RTC_DATA", "ERR_PERIMETER_TIMEOUT", "ERR_TRACKING", "ERR_ODOMETRY_LEFT", "ERR_ODOMETRY_RIGHT", "ERR_BATTERY", "ERR_CHARGER", "ERR_GPS_COMM",
  "ERR_GPS_DATA", "ERR_ADC_CALIB", "ERR_IMU_CALIB", "ERR_EEPROM_DATA", "ERR_STUCK", "ERR_CPU_SPEED", "ERR_ENUM_COUNT"};  

const char* mowPatternNames[] = {"RAND", "LANE", "BIDIR"};

const char* consoleModeNames[] ={"sen_counters", "sen_values", "perimeter", "off"}; 


// --- split robot class ----
#include "battery.h"
#include "consoleui.h"
#include "ros.h"
#include "rmcs.h" // Use Robot Mower Communication Standard 
#include "motor.h"
#include "buzzer.h"
#include "modelrc.h"
#include "settings.h"
#include "timer.h"
// -----------------------------


// Spannungsteiler Gesamtspannung ermitteln (Reihenschaltung R1-R2, U2 bekannt, U_GES zu ermitteln)
float Robot::voltageDividerUges(float R1, float R2, float U2){
	return (U2/R2 * (R1+R2));  // Uges 
}

// ADC-value to voltage
float Robot::ADC2voltage(float ADCvalue){
  return (ADCvalue /1023.0 * IOREF);   // ADCman works @ 10 bit
}  

extern "C" char* sbrk(int incr);

int Robot::freeMemory() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}

Robot::Robot(){
  name = "Generic";
  developerActive = false;
  rc.setRobot(this);

  for (int initCounter = 0; initCounter < 30; initCounter++) {
    stateLog[initCounter] = -1;
    errorLog[initCounter] = -1;
    sensorLog[initCounter] = -1;
  }

  batteryNextChargeAfterFull = 0;
  lastSensorTriggeredTime =0;
	stateLast = stateCurr = stateNext = STATE_OFF; 
  stateTime = 0;
  idleTimeSec = 0;
  rosTimeout = 0;
  statsMowTimeTotalStart = false;            
  mowPatternCurr = MOW_RANDOM;
  
  odometryLeft = odometryRight = 0;	
  odometryLeftLastState = odometryLeftLastState2 = odometryRightLastState = odometryRightLastState2 = LOW;
  odometryTheta = odometryX = odometryY = 0;

  gpsMap gpsMapPerimeter;
    
  motorRightRpmCurr = motorLeftRpmCurr = 0;
  lastMotorRpmTime = 0;
  lastSetMotorSpeedTime = 0;
  motorLeftSpeedRpmSet =  motorRightSpeedRpmSet = 0; 
  motorLeftPWMCurr = motorRightPWMCurr = 0;
  motorRightSenseADC = motorLeftSenseADC = 0;
  motorLeftSenseCurrent = motorRightSenseCurrent = 0;     
  motorLeftSense = motorRightSense = 0;
  motorLeftSenseCounter = motorRightSenseCounter = 0;  
  motorZeroSettleTime = 1000;  
  motorLeftZeroTimeout = 0;
  motorRightZeroTimeout = 0;  
  rotateLeft = true;
  mowIncreaseCircleRadiusTime = 0;
  currentCirclingStep = 0;

  remoteSteer = remoteSpeed = remoteMow = remoteSwitch = 0;  
  remoteSteerLastTime = remoteSpeedLastTime =remoteMowLastTime =remoteSwitchLastTime = 0;
  remoteSteerLastState = remoteSpeedLastState = remoteMowLastState = remoteSwitchLastState = LOW;

  
  motorMowRpmCounter = 0;
  motorMowRpmLastState = LOW;
  motorMowEnable = false;
  motorMowForceOff = false;
  motorMowSpeedPWMSet = motorSpeedMaxRpm;
  motorMowPWMCurr = 0;
  motorMowSenseADC = 0;
  motorMowSenseCurrent  = 0;
  motorMowSense = 0;
  motorMowSenseCounter = 0;
  motorMowSenseErrorCounter = 0;
  motorMowRpmCurr = 0;
  lastMowSpeedPWM = 0;
  lastSetMotorMowSpeedTime = 0;
  nextTimeCheckCurrent = 0;
  lastTimeMotorMowStuck = 0;
  lastErrType = 0;
  
  bumperLeftCounter = bumperRightCounter = 0;
  bumperLeft = bumperRight = false;          
   
   dropLeftCounter = dropRightCounter = 0;                                                                                              // Dropsensor - Absturzsensor
   dropLeft = dropRight = false;                                                                                                        // Dropsensor - Absturzsensor
  
  gpsLat = gpsLon = gpsX = gpsY = 0;
  robotIsStuckCounter = 0;
  gpsAchievedSpeed = 0;
  gpsSpeedUpdatedTime = 0;
  gpsDeclination = 0;
  gpsPerimeterRollState = 0;
  gpsPerimeterRollSubStateStartTime = 0;
  gpsHomingLastStep = 0;

  imuDriveHeading = 0;
  imuRollHeading = 0;
  imuRollDir = LEFT;  
  
  perimeterMag = 1;
  perimeterMagMedian.add(perimeterMag);
  perimeterInside = true;
  perimeterCounter = 0;  
  perimeterLastTransitionTime = 0;
  perimeterTriggerTime = 0;
  
  lawnSensorCounter = 0;
  lawnSensor = false;
  lawnSensorFront = lawnSensorFrontOld = lawnSensorBack = lawnSensorBackOld = 0;  
  
  rain = false;
  rainCounter = 0;

  freeWheelIsMoving = false;

  sonarLeftUse = sonarRightUse = sonarCenterUse = false;
  sonarDistCenter = sonarDistRight = sonarDistLeft = 0;
  sonarDistCounter = 0;
  tempSonarDistCounter = 0;
  sonarObstacleTimeout = 0;  

  batADC = 0;
  batVoltage = 0;
  batRefFactor = 0;
  batCapacity = 0;
  lastTimeBatCapacity = 0;
  chgVoltage = 0;
  chgCurrent = 0;
    
  memset(errorCounterMax, 0, sizeof errorCounterMax);
  memset(errorCounter, 0, sizeof errorCounterMax);
    
  loopsPerSec = 0;
  loopsPerSecMax = 0;
	loopsPerSecLowCounter = 0;
  loopsTa = 5.0;
  loopsPerSecCounter = 0;
  buttonCounter = 0;
  ledState = 0;

  consoleMode = CONSOLE_SENSOR_COUNTERS; 
  nextTimeButtonCheck = 0;
  nextTimeInfo = 0;
  nextTimeROS = 0;
  nextTimeMotorSense = 0;
  nextTimeIMU = 0;
  nextTimeCheckTilt = 0;
  nextTimeOdometry = 0;
  nextTimeOdometryInfo = 0;
  nextTimeBumper = 0;
  nextTimeDrop = 0;                                                                                                                    // Dropsensor - Absturzsensor
  nextTimeSonar = 0;
  nextTimeFreeWheel = 0;
  nextTimeBattery = 0;
  nextTimeCheckBattery = millis() + 10000;
  nextTimePerimeter = 0;
  nextTimeLawnSensor = 0;
  nextTimeLawnSensorCheck = 0;
  nextTimePrintErrors = 0;
  nextTimeTimer = millis() + 60000;
  nextTimeRTC = 0;
  nextTimeGPS = 0;
  nextTimeCheckIfStuck = 0;
  nextTimePfodLoop = 0;
  nextTimeRain = 0;
  lastMotorMowRpmTime = millis();
  nextTimeButton = 0;
  nextTimeErrorCounterReset = 0;    
  nextTimeErrorBeep = 0;
  nextTimeMotorControl = 0;  
  nextTimeMotorImuControl = 0;
  nextTimeCheckIfImuMaxed = 0;
  nextTimeMotorPerimeterControl = 0;
  nextTimeMotorMowControl = 0;
  nextTimeRotationChange = 0;
  nextTimeTemperatureCheck = 0;
  caseTemperature = -999;
  
  nextTimeRobotStats = 0;
  statsMowTimeMinutesTripCounter = 0;
  statsBatteryChargingCounter = 0;
  
  nextTimeRMCSInfo			= 0;  
  rmcsInfoLastSendState = 0;
  rmcsInfoLastSendMotorCurrent = 0;
  rmcsInfoLastSendSonar = 0;
  rmcsInfoLastSendBumper = 0;
  rmcsInfoLastSendOdometry = 0;
  rmcsInfoLastSendPeri = 0;
}

const char *Robot::mowPatternName(){
  return mowPatternNames[mowPatternCurr];
}

void Robot::setSensorTriggered(char type){
  lastSensorTriggered = type;
  lastSensorTriggeredTime = millis();

  for (int shiftArrayCounter = 28; shiftArrayCounter >= 0; shiftArrayCounter--) {
    sensorLog[shiftArrayCounter+1] = sensorLog[shiftArrayCounter];
  }
  sensorLog[0] = lastSensorTriggered;


  if (!rmcsUse){
  Console.println( sensorNames[lastSensorTriggered] );
  }else{
    rmcsSendSensorTriggered(type);
  }
}

const char *Robot::lastSensorTriggeredName(){
  String s = "N/A";
  if (lastSensorTriggeredTime != 0) {
    s = sensorNames[lastSensorTriggered];
    s += " (";
    s += String( (millis()-lastSensorTriggeredTime) /1000);
    s += " s ago)";
  }
  return s.c_str();
}

void Robot::resetIdleTime(){
  if (idleTimeSec == BATTERY_SW_OFF){ // battery switched off?
    Console.println(F("BATTERY switching ON again"));
    setActuator(ACT_BATTERY_SW, 1);  // switch on battery again (if connected via USB)
  }
  idleTimeSec = 0;
}

void Robot::beep(int numberOfBeeps, boolean shortbeep = false){
  for (int i=0; i < numberOfBeeps; i++){
    setActuator(ACT_BUZZER, 4200); 
    if (shortbeep) delay(50);
      else delay(500);
    setActuator(ACT_BUZZER, 0); 
    if (shortbeep) delay(250);
      else delay(500);
  }
}

// set user-defined switches
void Robot::setUserSwitches(){
  setActuator(ACT_USER_SW1, userSwitch1);   
  setActuator(ACT_USER_SW2, userSwitch2);     
  setActuator(ACT_USER_SW3, userSwitch3);     
}

void Robot::setup()  {     
  
  while(Console.available() > 0) {
    char t = Console.read();
  }

  setDefaultTime();
  setMotorPWM(0, 0, false);
  loadSaveErrorCounters(true);
  loadUserSettings();

  //imu.compass.setDeclination(gpsDeclination);
  gpsPerimeter.init(longGrassTempAreaSize, perimeterUse);

  if (!statsOverride) loadSaveRobotStats(true);
  else loadSaveRobotStats(false);
  setUserSwitches();	
	if (!ADCMan.calibrationDataAvail()) {
    ADCMan.calibrate();
  }
  
  if (!buttonUse){
    // robot has no ON/OFF button => start immediately
    setNextState(STATE_FORWARD,0);
  }  
    
  stateStartTime = millis();  
  beep(1);  
  Console.println(F("-------------------START-------------------"));  
  Console.print(F("Ardumower "));
  Console.print(VER);
  Console.print(F("  "));
    #if defined (PCB_1_2)
		 Console.print(F("PCB 1.2"));
  #elif defined (PCB_1_3)
     Console.print(F("PCB 1.3"));  
	#endif
	#ifdef __AVR__
		Console.print(F("  Arduino Mega"));
	#else
		Console.print(F("  Arduino Due"));
  #endif
	Console.print(F("  IOREF="));	
	Console.println(IOREF);

  Console.print(F("Robot: "));
  Console.println(name);  
      
  Console.println(F("press..."));
  Console.println(F("  d main menu"));    
  Console.println(F("  v change console mode (sensor counters, values, perimeter etc.)"));    
  Console.println(F("  1 start automatic mowing"));    
  Console.println(F("  0 stop"));      
  Console.println(F("  h drive home"));        
  Console.println(F("  3 activate model R/C mode"));     
  Console.println(F("  m toggle mow motor"));        
  Console.println(F("  p track perimeter"));    
  Console.println(F("  l simulate left bumper"));    
  Console.println(F("  r simulate right bumper"));          
  Console.println();        
  Console.print(F("current console mode: "));
  Console.println(consoleModeNames[consoleMode]);
  Console.println(F("-------------------------------------------"));  
} 



void Robot::checkButton(){
  if ( (!buttonUse) || (millis() < nextTimeButtonCheck) ) return;
    
  nextTimeButtonCheck = millis() + 50;      
  boolean buttonPressed = (readSensor(SEN_BUTTON) == LOW);    
  if ( ((!buttonPressed) && (buttonCounter > 0)) || ((buttonPressed) && (millis() >= nextTimeButton)) ) {                
    nextTimeButton = millis() + 1000;
    if (buttonPressed){
      Console.println(F("buttonPressed"));
      // ON/OFF button pressed                                                
      beep(1);
      buttonCounter++;
			setSensorTriggered(SEN_BUTTON);
     if (!rmcsUse){
      resetIdleTime();
     }
    } 
    else { 
      // ON/OFF button released          
      if  ( ((stateCurr != STATE_OFF) || (stateCurr == STATE_ERROR)) && (stateCurr != STATE_STATION) ) {
        setNextState(STATE_OFF, 0);
      } else if (buttonCounter == 2){
        motorMowEnable = true;
        mowPatternCurr = MOW_BIDIR;
        setNextState(STATE_FORWARD, 0);                
      } else if (buttonCounter == 3){      
        // start remote control mode
        setNextState(STATE_REMOTE, 0);
      } else if (buttonCounter == 4){
        // start normal without perimeter
        //motorMowEnable = false;        
        perimeterUse = false;
        setNextState(STATE_FORWARD,0);           
      } else if (buttonCounter == 7){
        // start normal with mowing in lanes
        motorMowEnable = true;
        //motorMowModulate = true;
        mowPatternCurr = MOW_LANES;
        setNextState(STATE_FORWARD, 0);                
      } else if (buttonCounter == 6){
        // track perimeter
        setNextState(STATE_PERI_TRACK, 0);        
      } else if (buttonCounter == 5){
        // drive home
        if (gpsHomingInUse && imuUse) setNextState(STATE_GPS_HOMING_FIRST_TURN, 0);
        else setNextState(STATE_PERI_FIND, 0); 
      } else if (buttonCounter == 1){
        /*if ((perimeterUse) && (!perimeter.isInside())){
          Console.println("start inside perimeter!");
          addErrorCounter(ERR_PERIMETER_TIMEOUT);
          setNextState(STATE_ERROR, 0);                          
        } else {*/
          // start normal with mowing        
          motorMowEnable = true;
          //motorMowModulate = true;                     
          mowPatternCurr = MOW_RANDOM;   
          setNextState(STATE_FORWARD, 0);                
        //}
      } 
      
      buttonCounter = 0;                 
    }       
  }
}

void Robot::readSensors(){
//NOTE: this function should only read in sensors into variables - it should NOT change any state!

  if (millis() >= nextTimeMotorSense){    
    nextTimeMotorSense = millis() +  50;
    double accel = 0.05;
    motorRightSenseADC = readSensor(SEN_MOTOR_RIGHT);
    motorLeftSenseADC = readSensor(SEN_MOTOR_LEFT);
    motorMowSenseADC = readSensor(SEN_MOTOR_MOW);
    
    motorRightSenseCurrent = motorRightSenseCurrent * (1.0-accel) + ((double)motorRightSenseADC) * motorSenseRightScale * accel;
    motorLeftSenseCurrent = motorLeftSenseCurrent * (1.0-accel) + ((double)motorLeftSenseADC) * motorSenseLeftScale * accel;
    // NOTE for motor mower current : we double motor current as two drivers are connected in parallel
    motorMowSenseCurrent = motorMowSenseCurrent * (1.0-accel) + ((double)motorMowSenseADC) * motorMowSenseScale * accel * 2; 
   
    if (batVoltage > 8){
      motorRightSense = motorRightSenseCurrent * batVoltage /1000;   // conversion to power in Watt
      motorLeftSense  = motorLeftSenseCurrent  * batVoltage /1000;
      motorMowSense   = motorMowSenseCurrent   * batVoltage /1000;
    }
    else{
      motorRightSense = motorRightSenseCurrent * batFull /1000;   // conversion to power in Watt in absence of battery voltage measurement
      motorLeftSense  = motorLeftSenseCurrent  * batFull /1000;
      motorMowSense   = motorMowSenseCurrent   * batFull /1000;
    }
  
    if ((millis() - lastMotorMowRpmTime) >= 500){                  
      motorMowRpmCurr = readSensor(SEN_MOTOR_MOW_RPM);    
      if ((motorMowRpmCurr == 0) && (motorMowRpmCounter != 0)){
        // rpm may be updated via interrupt
        motorMowRpmCurr = (int) ((((double)motorMowRpmCounter) / ((double)(millis() - lastMotorMowRpmTime))) * 60000.0);
        motorMowRpmCounter = 0;        
      }       
      lastMotorMowRpmTime = millis();     
      if (!ADCMan.calibrationDataAvail()) {
        //Console.println(F("Error: missing ADC calibration data"));
        //addErrorCounter(ERR_ADC_CALIB);
        //setNextState(STATE_ERROR, 0);
      }
    }
  }  


  if ((perimeterUse) && (millis() >= nextTimePerimeter)){    
    if (stateCurr == STATE_PERI_TRACK) nextTimePerimeter = millis() +  30;   
    else nextTimePerimeter = millis() +  50;   // 50
    perimeterMag = readSensor(SEN_PERIM_LEFT);
    if (stateCurr == STATE_PERI_FIND)perimeterMagMedian.add(abs(perimeterMag));
    if ((perimeter.isInside(0) != perimeterInside)){      
      perimeterCounter++;
			setSensorTriggered(SEN_PERIM_LEFT);
      perimeterLastTransitionTime = millis();
      perimeterInside = perimeter.isInside(0);
    }    
    static boolean LEDstate = false;
    if (perimeterInside && !LEDstate) {
      setActuator(ACT_LED, HIGH);
      LEDstate = true;
	}
    if (!perimeterInside && LEDstate) {
      setActuator(ACT_LED, LOW);
      LEDstate = false;
	  }
    if ((!perimeterInside) && (perimeterTriggerTime == 0)){
      // set perimeter trigger time      
      if (millis() > stateStartTime + 2000){ // far away from perimeter?
        perimeterTriggerTime = millis() + perimeterTriggerTimeout;  
      } else {  
        perimeterTriggerTime = millis();
      }
    }
    if (perimeter.signalTimedOut(0) && stateCurr != STATE_ERROR)  {      
      if ( (stateCurr != STATE_OFF) && (stateCurr != STATE_MANUAL) && (stateCurr != STATE_STATION) 
      	&& (stateCurr != STATE_STATION_CHARGING) && (stateCurr != STATE_STATION_CHECK) 
      	&& (stateCurr != STATE_STATION_REV) && (stateCurr != STATE_STATION_ROLL) 
      	&& (stateCurr != STATE_STATION_FORW) && (stateCurr != STATE_REMOTE) && (stateCurr != STATE_PERI_OUT_FORW)
        && (stateCurr != STATE_PERI_OUT_REV) && (stateCurr != STATE_PERI_OUT_ROLL) && (stateCurr != STATE_PERI_TRACK)) {
        Console.println("Error: perimeter too far away");
        addErrorCounter(ERR_PERIMETER_TIMEOUT);
        setNextState(STATE_ERROR,0);
      }
    }
  }


  if ((lawnSensorUse) && (millis() >= nextTimeLawnSensor)){    
    nextTimeLawnSensor = millis() + 100;               
    double accel = 0.03;
    lawnSensorFront = (1.0-accel) * lawnSensorFront + accel * ((double)readSensor(SEN_LAWN_FRONT));
    lawnSensorBack  = (1.0-accel) * lawnSensorBack  + accel * ((double)readSensor(SEN_LAWN_BACK));        
  }
 if ((lawnSensorUse) && (millis() >= nextTimeLawnSensorCheck)){          
    nextTimeLawnSensorCheck = millis() + 2000;               
    double deltaFront = lawnSensorFront/lawnSensorFrontOld * 100.0;    
    double deltaBack = lawnSensorBack/lawnSensorBackOld * 100.0;        
    if ((deltaFront <= 95) || (deltaBack <= 95)){
      Console.print(F("LAWN "));
      Console.print(deltaFront);
      Console.print(",");
      Console.println(deltaBack);
      lawnSensorCounter++;
			setSensorTriggered(SEN_LAWN_FRONT);
      lawnSensor=true;
    }
    lawnSensorFrontOld = lawnSensorFront;
    lawnSensorBackOld  = lawnSensorBack;
  }


 if ((sonarUse) && (millis() >= nextTimeSonar)){
    static char senSonarTurn = SEN_SONAR_CENTER;    
    nextTimeSonar = millis() + 200;
    
    switch(senSonarTurn) {
      case SEN_SONAR_RIGHT:
        if (sonarRightUse) sonarDistRight = readSensor(SEN_SONAR_RIGHT);
        senSonarTurn = SEN_SONAR_LEFT;
        break;
      case SEN_SONAR_LEFT:
        if (sonarLeftUse) sonarDistLeft = readSensor(SEN_SONAR_LEFT);
        senSonarTurn = SEN_SONAR_CENTER;
        break;
      case SEN_SONAR_CENTER:
        if (sonarCenterUse) sonarDistCenter = readSensor(SEN_SONAR_CENTER);
        senSonarTurn = SEN_SONAR_RIGHT;
        break;
      default:
        senSonarTurn = SEN_SONAR_CENTER;
        break;
    }   
/*
    if (sonarRightUse) sonarDistRight = readSensor(SEN_SONAR_RIGHT);    
    if (sonarLeftUse) sonarDistLeft = readSensor(SEN_SONAR_LEFT);    
    if (sonarCenterUse) sonarDistCenter = readSensor(SEN_SONAR_CENTER); 
*/         
  }

  if ((freeWheelUse) && (millis() >= nextTimeFreeWheel)){    
    nextTimeFreeWheel = millis() + 100;               
    freeWheelIsMoving = (readSensor(SEN_FREE_WHEEL) == 0);
    if (!freeWheelIsMoving) setSensorTriggered(SEN_FREE_WHEEL);
  }

  if ((bumperUse) && (millis() >= nextTimeBumper)){    
    nextTimeBumper = millis() + 100;               
    tilt = (readSensor(SEN_TILT) == 0);
        
    if (readSensor(SEN_BUMPER_LEFT) == 0) {
      bumperLeftCounter++;
			setSensorTriggered(SEN_BUMPER_LEFT);
      bumperLeft=true;
    }

    if (readSensor(SEN_BUMPER_RIGHT) == 0) {
      bumperRightCounter++;
			setSensorTriggered(SEN_BUMPER_RIGHT);
      bumperRight=true;
    } 
  }


  if ((dropUse) && (millis() >= nextTimeDrop)){                                                                          // Dropsensor - Absturzsensor
    nextTimeDrop = millis() + 100;                                                                                          // Dropsensor - Absturzsensor
    if (readSensor(SEN_DROP_LEFT) == dropcontact) {                                                                         // Dropsensor - Absturzsensor
      dropLeftCounter++;                                                                                                    // Dropsensor - Absturzsensor
			setSensorTriggered(SEN_DROP_LEFT);	
			dropLeft=true;                                                                                                        // Dropsensor - Absturzsensor
    }                                                                                                                       // Dropsensor - Absturzsensor
 
    if (readSensor(SEN_DROP_RIGHT) == dropcontact) {                                                                          // Dropsensor - Absturzsensor
      dropRightCounter++;                                                                                                   // Dropsensor - Absturzsensor
      setSensorTriggered(SEN_DROP_RIGHT);
			dropRight=true;                                                                                                       // Dropsensor - Absturzsensor
    } 
  }    
  
  if ((timerUse) && (millis() >= nextTimeRTC)) {
    nextTimeRTC = millis() + 60000;    
    readSensor(SEN_RTC);       // read RTC             
    Console.print(F("RTC datetime received: "));
    Console.print(date2str(datetime.date));  
		Console.print(F("  "));  
		Console.println(time2str(datetime.time));  
  }

/*  // this seems to be handled now a days by the IMU class directly from main loop
  if ((imuUse) && (millis() >= nextTimeIMU)) {
    // IMU
    readSensor(SEN_IMU);
    nextTimeIMU = millis() + 200;   // 5 hz    
    if (imu.getErrorCounter()>0) {
      addErrorCounter(ERR_IMU_COMM);
      Console.println(F("IMU comm error"));    
    }    
    if (!imu.calibrationAvail) {
      //MR Console.println(F("Error: missing IMU calibration data"));
      addErrorCounter(ERR_IMU_CALIB);
      setNextState(STATE_ERROR, 0);
    }
  }
*/

  if (millis() >= nextTimeBattery){
    // read battery
    nextTimeBattery = millis() + 100;       
    if ((abs(chgCurrent) > 0.04) && (chgVoltage > 5)){
      // charging
      batCapacity += (chgCurrent / 36.0);
    }
    // convert to double  
    batADC = readSensor(SEN_BAT_VOLTAGE);
		int currentADC = readSensor(SEN_CHG_CURRENT);
		int chgADC = readSensor(SEN_CHG_VOLTAGE);    
		//Console.println(currentADC);
    double batvolt = ((double)batADC) * batFactor / 10;  // / 10 due to arduremote bug, can be removed after fixing    
    double chgvolt = ((double)chgADC) * batChgFactor / 10;  // / 10 due to arduremote bug, can be removed after fixing    
		double curramp = ((double)currentADC) * chgFactor / 10;  // / 10 due to arduremote bug, can be removed after fixing		

    #if defined (PCB_1_3)         // Prüfe ob das V1.3 Board verwendet wird - und wenn ja **UZ**
    batvolt = batvolt + DiodeD9;  // dann rechnet zur Batteriespannung den Spannungsabfall der Diode D9 hinzu. (Spannungsabfall an der Diode D9 auf den 1.3 Board (Die Spannungsanzeige ist zu niedrig verursacht durch die Diode D9) **UZ**
    #endif                        // **UZ**
    
    // low-pass filter
    double accel = 0.01;
		//double accel = 1.0;
    if (abs(batVoltage-batvolt)>5)   batVoltage = batvolt; else batVoltage = (1.0-accel) * batVoltage + accel * batvolt;
    if (abs(chgVoltage-chgvolt)>5)   chgVoltage = chgvolt; else chgVoltage = (1.0-accel) * chgVoltage + accel * chgvolt;
		if (abs(chgCurrent-curramp)>0.5) chgCurrent = curramp; else chgCurrent = (1.0-accel) * chgCurrent + accel * curramp;       
  } 

  if ((rainUse) && (millis() >= nextTimeRain)) {
    // read rain sensor
    nextTimeRain = millis() + 5000;
    rain = (readSensor(SEN_RAIN) != 0);  
    if (rain) {
		  rainCounter++;	
		  setSensorTriggered(SEN_RAIN);
    }
  }  
}


void Robot::setDefaults(){
  motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;    
  motorMowEnable = false;                
}



void Robot::receiveGPSTime(){
  if (gpsUse) {
    unsigned long chars = 0;
    unsigned short good_sentences = 0;
    unsigned short failed_cs = 0;
    gps.stats(&chars, &good_sentences, &failed_cs);    
    if (good_sentences == 0) {
      // no GPS sentences received so far
      Console.println(F("GPS communication error!"));      
      addErrorCounter(ERR_GPS_COMM);
      // next line commented out as GPS communication may not be available if GPS signal is poor
      //setNextState(STATE_ERROR, 0);
    }
    Console.print(F("GPS sentences: "));    
    Console.println(good_sentences);    
    Console.print(F("GPS satellites in view: "));          
    Console.println(gps.satellites());          
    if (gps.satellites() == 255) {
      // no GPS satellites received so far
      addErrorCounter(ERR_GPS_DATA);          
    }
    int year;
    byte month, day, hour, minute, second, hundredths;
    unsigned long age; 
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    if (age != GPS::GPS_INVALID_AGE)
    {
      Console.print(F("GPS date received: "));
      Console.println(date2str(datetime.date));  
      datetime.date.dayOfWeek = getDayOfWeek(month, day, year, 1);      
      datetime.date.day = day;
      datetime.date.month = month;
      datetime.date.year = year;
      datetime.time.hour = hour;
      datetime.time.minute = minute;
      if (timerUse){
        // set RTC using GPS data
        Console.print(F("RTC date set: "));
        Console.println(date2str(datetime.date));  
        setActuator(ACT_RTC, 0);            
      }
    }      
  }
}


void Robot::reverseOrBidir(byte aRollDir) {
  if (mowPatternCurr == MOW_BIDIR) {
    if (stateCurr == STATE_FORWARD) {
      setNextState(STATE_REVERSE, RIGHT);
    } else if (stateCurr == STATE_REVERSE) {
      setNextState(STATE_FORWARD, LEFT);
    }
  } else {
    if (stateCurr == STATE_FORWARD || stateCurr == STATE_CIRCLE) {
      setNextState(STATE_REVERSE, aRollDir);
    } else {
      setNextState(STATE_FORWARD, aRollDir);
    }
  }
}

void Robot::reverseOrBidirBumper(byte aRollDir) {
  if (mowPatternCurr == MOW_BIDIR) {
    if (stateCurr == STATE_FORWARD) {
      setNextState(STATE_REVERSE, RIGHT);
    } else if (stateCurr == STATE_REVERSE) {
      setNextState(STATE_FORWARD, LEFT);
    }
  } else {
    if (stateCurr == STATE_FORWARD) {
      setNextState(STATE_BUMPER_REVERSE, aRollDir);
    } else if (stateCurr == STATE_REVERSE) {
      setNextState(STATE_BUMPER_FORWARD, aRollDir);
    }
  }
}

// check motor current
void Robot::checkCurrent(){
  if (millis() < nextTimeCheckCurrent) return;
  nextTimeCheckCurrent = millis() + 100;

  //bb add test MotorCurrent in manual mode and stop immediatly If >Powermax
  if (stateCurr == STATE_MANUAL)
  {
    if (motorLeftSense >= motorPowerMax)
    {
       motorLeftSenseCounter++;
			 setSensorTriggered(SEN_MOTOR_LEFT);
       setMotorPWM( 0, 0, false );
       addErrorCounter(ERR_MOTOR_LEFT);
       setNextState(STATE_ERROR, 0);
       Console.println("Error: Motor Left current");
    }
    if (motorRightSense >= motorPowerMax)
    {
       motorRightSenseCounter++;
			 setSensorTriggered(SEN_MOTOR_RIGHT);
			 setMotorPWM( 0, 0, false );
       addErrorCounter(ERR_MOTOR_RIGHT);
       setNextState(STATE_ERROR, 0);
       Console.println("Error: Motor Right current");
    }
  }


  if ( motorMowCircleTriggerPower > 3
      && motorMowSense >= motorMowCircleTriggerPower
      && stateCurr == STATE_FORWARD
      && millis() >= stateStartTime + 5000
      && perimeter.isInside(0)
      && abs(perimeterMag) < 1000
      && mowPatternCurr == MOW_RANDOM) {  // if motor power goes above motorMowCircleTriggerPower assume that we hit longer grass and start moving around it
        setSensorTriggered(SEN_MOW_POWER);

        if(longGrassTempAreaSize > 5) {
          gpsPerimeter.setTemporaryArea(gpsLat, gpsLon);
          if((rand() % 2) == 0) {
            rollDir = LEFT;
          } else {
            rollDir = RIGHT;
          }
        }
        setNextState(STATE_CIRCLE, rollDir);
  }


  if (motorMowSense >= motorMowPowerMax){
    motorMowSenseCounter++;
		setSensorTriggered(SEN_MOTOR_MOW);
  }
  else{ 
      errorCounterMax[ERR_MOW_SENSE] = 0;
      motorMowSenseCounter = 0;
      if ( (lastTimeMotorMowStuck != 0) && (millis() >= lastTimeMotorMowStuck + 10000) ) { // wait 30 seconds before switching on again
        errorCounter[ERR_MOW_SENSE] = 0;
        motorMowEnable = true;
				lastTimeMotorMowStuck = 0;
      }
  }


  if (motorMowSenseCounter >= 30){ //ignore motorMowPower for 3 seconds
      motorMowEnable = false;
      Console.println("Error: Motor mow current");
      addErrorCounter(ERR_MOW_SENSE);
      lastTimeMotorMowStuck = millis();
     // if (rollDir == RIGHT) reverseOrBidir(LEFT); // toggle roll dir
     //else reverseOrBidir(RIGHT); 
  }       

    
  if (motorLeftSense >=motorPowerMax){  
    // left wheel motor overpowered    
    if (  ((stateCurr == STATE_FORWARD) 
        || (stateCurr == STATE_PERI_FIND)  
        || (stateCurr == STATE_CIRCLE)  
        || (stateCurr == STATE_PERI_TRACK)) 
      && (millis() > stateStartTime + motorPowerIgnoreTime)){    				  
      //beep(1);
      motorLeftSenseCounter++;
			setSensorTriggered(SEN_MOTOR_LEFT);
      setMotorPWM( 0, 0, false );  
      reverseOrBidir(RIGHT);
    } else if    ((stateCurr == STATE_REVERSE) && (millis() > stateStartTime + motorPowerIgnoreTime)){
      motorLeftSenseCounter++;
			setSensorTriggered(SEN_MOTOR_LEFT);
      setMotorPWM( 0, 0, false );  
      //   reverseOrBidir(RIGHT);
      setNextState(STATE_ROLL,RIGHT);				          
    } else if ((stateCurr == STATE_ROLL) && (millis() > stateStartTime + motorPowerIgnoreTime)){
      motorLeftSenseCounter++;
			setSensorTriggered(SEN_MOTOR_LEFT);
      setMotorPWM( 0, 0, false );  
      setNextState(STATE_FORWARD, 0);
    }    
  }
  else if (motorRightSense >= motorPowerMax){  
     // right wheel motor overpowered
     if ( ((stateCurr == STATE_FORWARD) 
          || (stateCurr == STATE_CIRCLE)
          || (stateCurr == STATE_PERI_FIND) 
          || (stateCurr == STATE_PERI_TRACK)) 
          && (millis() > stateStartTime + motorPowerIgnoreTime)){    				  
       //beep(1);
       motorRightSenseCounter++;
			 setSensorTriggered(SEN_MOTOR_RIGHT);
       setMotorPWM( 0, 0, false );  
       reverseOrBidir(RIGHT);
     } else if ((stateCurr == STATE_REVERSE) && (millis() > stateStartTime + motorPowerIgnoreTime)){
       motorRightSenseCounter++;
				setSensorTriggered(SEN_MOTOR_RIGHT);
       setMotorPWM( 0, 0, false );  
       setNextState(STATE_ROLL,LEFT);				          
     } else if ((stateCurr == STATE_ROLL) && (millis() > stateStartTime + motorPowerIgnoreTime)){
       motorRightSenseCounter++;
			 setSensorTriggered(SEN_MOTOR_RIGHT);
       setMotorPWM( 0, 0, false );  
       setNextState(STATE_FORWARD, 0);
    }
  }
}  

// check bumpers
void Robot::checkBumpers(){
  if (!bumperUse) return;
  if ((mowPatternCurr == MOW_BIDIR) && (millis() < stateStartTime + 4000)) return;
  
  if ((bumperLeft || bumperRight)) {    
      if (bumperLeft) {
        reverseOrBidirBumper(RIGHT);          
      } else {
        reverseOrBidirBumper(LEFT);
      }    
  }  
}

// check free wheel
void Robot::checkFreeWheel(){
  if (!freeWheelUse) return;
  if (millis() < stateStartTime + 4000) return;

  if (!freeWheelIsMoving) {    
      if (random(2) == 0){
        reverseOrBidirBumper(RIGHT);          
      } else {
        reverseOrBidirBumper(LEFT);
      }    
  }  
}

// check drop                                                                                                                       // Dropsensor - Absturzsensor
void Robot::checkDrop(){                                                                                                            // Dropsensor - Absturzsensor
  if (!dropUse) return;
  if ((mowPatternCurr == MOW_BIDIR) && (millis() < stateStartTime + 4000)) return;                                                  // Dropsensor - Absturzsensor

  if ((dropLeft || dropRight)) {                                                                                                    // Dropsensor - Absturzsensor  
      if (dropLeft) {                                                                                                               // Dropsensor - Absturzsensor
        reverseOrBidir(RIGHT);                                                                                                      // Dropsensor - Absturzsensor     
      } else {                                                                                                                      // Dropsensor - Absturzsensor
        reverseOrBidir(LEFT);                                                                                                       // Dropsensor - Absturzsensor
      }                                                                                                                             // Dropsensor - Absturzsensor
  }                                                                                                                                 // Dropsensor - Absturzsensor
}                                                                                                                                   // Dropsensor - Absturzsensor

// check bumpers while tracking perimeter
void Robot::checkBumpersPerimeter(){
  if (!bumperUse) return;
  if ((bumperLeft || bumperRight)) {    
    if ((bumperLeft) || (stateCurr == STATE_PERI_TRACK)) {
      setNextState(STATE_PERI_REV, RIGHT);          
    } else {
      setNextState(STATE_PERI_REV, LEFT);
    }
  }
}

// check perimeter as a boundary
void Robot::checkPerimeterBoundary(){

  if (gpsPerimeterUse) {
    if (stateCurr == STATE_FORWARD || stateCurr == STATE_CIRCLE) {
      if (gpsPerimeter.insidePerimeter(gpsLat, gpsLon) == 0) {
        setSensorTriggered(SEN_GPSPERIMETER);
        setNextState(STATE_GPSPERIMETER_CHANGE_DIR, rollDir);
      }
    }
  }  

  if(longGrassTempAreaSize > 5) {
    if (stateCurr == STATE_FORWARD || stateCurr == STATE_CIRCLE) {
      if (gpsPerimeter.getLongGrassTempAreaInUse() == 1 && (gpsPerimeter.insideLongGrassTempArea(gpsLat, gpsLon) == 0)) {
        setSensorTriggered(SEN_GPSLONGGRASS);
        setNextState(STATE_GPS_LONG_GRASS_CHANGE_DIR, rollDir);
      }
    }
  }

  if (!perimeterUse) return;
  if (millis() >= nextTimeRotationChange){
      nextTimeRotationChange = millis() + 60000;
      rotateLeft = !rotateLeft;
  }

  if (mowPatternCurr == MOW_BIDIR){
    if ((millis() < stateStartTime + 3000)) return;    
    if (!perimeterInside) {
      if ((rand() % 2) == 0){      
        reverseOrBidir(LEFT);
      } else {
        reverseOrBidir(RIGHT);
      }     
    }
  } else {  
    if (stateCurr == STATE_FORWARD || stateCurr == STATE_CIRCLE) {
      if (perimeterTriggerTime != 0) {
        if (millis() >= perimeterTriggerTime){        
          perimeterTriggerTime = 0;
          //if ((rand() % 2) == 0){  
          if ((rand() % 2) == 0){      
            reverseOrBidir(LEFT);
          } else {
            reverseOrBidir(RIGHT);
          }     
        }
      }
    } 
    else if ((stateCurr == STATE_ROLL)) {
      if (perimeterTriggerTime != 0) {
        if (millis() >= perimeterTriggerTime){ 
          perimeterTriggerTime = 0;
          setMotorPWM( 0, 0, false );
          //if ((rand() % 2) == 0){
          if ((rand() % 2) == 0){      
            reverseOrBidir(LEFT);
          } else {
            reverseOrBidir(RIGHT);
          }     
        }
      }
    }
  }
}

// check perimeter while finding it
void Robot::checkPerimeterFind(){
  if (!perimeterUse) return;
  if (stateCurr == STATE_PERI_FIND){
    if (perimeterInside) {
      // inside
      if (motorLeftSpeedRpmSet != motorRightSpeedRpmSet){      
        // we just made an 'outside=>inside' rotation, now track
        setNextState(STATE_PERI_TRACK, 0);    
      }
    } else {
      // we are outside, now roll to get inside
      motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.5;
      motorLeftSpeedRpmSet  = motorSpeedMaxRpm / 1.5;
    }
  }
}

 
// check lawn 
void Robot::checkLawn(){
  if (!lawnSensorUse) return;
  if ( (lawnSensor) && (millis() > stateStartTime + 3000) ) {      
    if (rollDir == RIGHT) reverseOrBidir(LEFT); // toggle roll dir
       else reverseOrBidir(RIGHT);        
  } else lawnSensor = false;
}

void Robot::checkRain(){
  if (!rainUse) return;
  if (rain){
    Console.println(F("RAIN"));
    if (perimeterUse) {
      if (gpsHomingInUse && imuUse) setNextState(STATE_GPS_HOMING_FIRST_TURN, 0);
      else setNextState(STATE_PERI_FIND, 0); 
    } else {
      setNextState(STATE_OFF, 0);
    }
  }
}

// check sonar
void Robot::checkSonar(){
  if(!sonarUse) return;
  if (millis() < nextTimeCheckSonar) return;
  nextTimeCheckSonar = millis() + 200;
  if ((mowPatternCurr == MOW_BIDIR) && (millis() < stateStartTime + 4000)) return;
  if (sonarDistCenter < 11 || sonarDistCenter > 100) sonarDistCenter = NO_ECHO; // Objekt ist zu nah am Sensor Wert ist unbrauchbar
  if (sonarDistRight < 11 || sonarDistRight > 100) sonarDistRight = NO_ECHO; // Object is too close to the sensor. Sensor value is useless
  if (sonarDistLeft < 11 || sonarDistLeft  > 100) sonarDistLeft = NO_ECHO; // Filters spiks under the possible detection limit
  // slow down motor wheel speed near obstacles   
  if ( (stateCurr == STATE_FORWARD) //|| stateCurr == STATE_CIRCLE)
          || (  (mowPatternCurr == MOW_BIDIR) && ((stateCurr == STATE_FORWARD) || (stateCurr == STATE_REVERSE))  )  
     ){
        if (sonarObstacleTimeout == 0) {
          if (     ((NO_ECHO != sonarDistCenter) && (sonarDistCenter < sonarSlowBelow)) 
               ||  ((NO_ECHO != sonarDistRight) && (sonarDistRight < sonarSlowBelow)) 
               ||  ((NO_ECHO != sonarDistLeft) && (sonarDistLeft < sonarSlowBelow))  ) {    
              tempSonarDistCounter++;
            if (tempSonarDistCounter >= 5){
             // Console.println("sonar slow down");
              motorLeftSpeedRpmSet /= 1.5;
              motorRightSpeedRpmSet /= 1.5;
              sonarObstacleTimeout = millis() + 3000;
            }
          } else tempSonarDistCounter = 0;
        } else if ((sonarObstacleTimeout != 0) && (millis() > sonarObstacleTimeout)) {
          //Console.println("no sonar");
          sonarObstacleTimeout = 0;
          tempSonarDistCounter = 0;
          motorLeftSpeedRpmSet *= 1.5;
          motorRightSpeedRpmSet *= 1.5;
        }
   }  
  
	 if (sonarTriggerBelow != 0  // Sonars may mess up reversing and rolls. sonars are in front so only use them for forward motion
    && stateCurr != STATE_ROLL
    && stateCurr != STATE_REVERSE
    && stateCurr != STATE_PERI_ROLL
    && stateCurr != STATE_PERI_REV
    && stateCurr != STATE_STATION_REV
    && stateCurr != STATE_STATION_ROLL
    && stateCurr != STATE_ROLL_WAIT
    && stateCurr != STATE_PERI_OUT_REV
    && stateCurr != STATE_PERI_OUT_ROLL){
		if ((sonarDistCenter != NO_ECHO) && (sonarDistCenter < sonarTriggerBelow)) {
			sonarDistCounter++;   
			setSensorTriggered(SEN_SONAR_CENTER);
			if (rollDir == RIGHT) reverseOrBidir(LEFT); // toggle roll dir
				else reverseOrBidir(RIGHT);    
		}
		if ((sonarDistRight != NO_ECHO) && (sonarDistRight < sonarTriggerBelow)){
			sonarDistCounter++;
			setSensorTriggered(SEN_SONAR_RIGHT);
			reverseOrBidir(LEFT);
		}
		if ((sonarDistLeft != NO_ECHO) && (sonarDistLeft < sonarTriggerBelow)){
			sonarDistCounter++; 
			setSensorTriggered(SEN_SONAR_LEFT);
			reverseOrBidir(RIGHT);
		}
	}
}


// check BumperDuino tilt, IMU tilt
void Robot::checkTilt(){
  if (millis() < nextTimeCheckTilt) return;
  nextTimeCheckTilt = millis() + 200; // 5Hz same as nextTimeImu
  
  if (tiltUse){
    if ((tilt) && (stateCurr != STATE_TILT_STOP) && (stateCurr != STATE_OFF) && (stateCurr != STATE_STATION_CHARGING)){
      Console.println(F("BumperDuino tilt"));      
			setSensorTriggered(SEN_TILT);
      setNextState(STATE_TILT_STOP,0);
    }
  }
  
  if(!imuUse) return;    
  int pitchAngle = (imu.ypr.pitch/PI*180.0);
  int rollAngle  = (imu.ypr.roll/PI*180.0);
  if ( (stateCurr != STATE_OFF) && (stateCurr != STATE_ERROR) && (stateCurr != STATE_STATION) ){
    if ( (abs(pitchAngle) > 60) || (abs(rollAngle) > 60) ){
      Console.println(F("Error: IMU tilt"));
      addErrorCounter(ERR_IMU_TILT);
			setSensorTriggered(SEN_TILT);
      setNextState(STATE_ERROR,0);
    }
  }
  if (stateCurr == STATE_ERROR){
    //if ( (abs(pitchAngle) < 40) && (abs(rollAngle) < 40) ) setNextState(STATE_FORWARD,0);
  }
}
//markor work in progress
void Robot::checkIfImuAccelerationMaxed(){
  if (millis() >= nextTimeCheckIfImuMaxed){
    nextTimeCheckIfImuMaxed = millis() + 100;
      if ( (imu.acc.x >= 44.30 || imu.acc.y >= 44.3 || imu.acc.z >= 44.2)
           && stateCurr != STATE_REVERSE
           && (stateCurr != STATE_OFF) 
           && (stateCurr != STATE_MANUAL) 
           && (stateCurr != STATE_STATION) 
           && (stateCurr != STATE_STATION_CHARGING) 
           && (stateCurr != STATE_STATION_CHECK) 
           && (stateCurr != STATE_STATION_REV) 
           && (stateCurr != STATE_STATION_ROLL) 
           && (stateCurr != STATE_REMOTE) 
           && (stateCurr != STATE_ERROR)
         ){
        setNextState(STATE_REVERSE,0);
        setSensorTriggered(SEN_IMU_ACC);
      } else if ( (imu.acc.x >= 44.30 || imu.acc.y >= 44.30 || imu.acc.z >= 44.2)
                  && stateCurr == STATE_REVERSE ){
        setSensorTriggered(SEN_IMU_ACC);
        reverseOrBidir(LEFT);
      }
  }
}

// check if mower is stuck ToDo: take HDOP into consideration if gpsSpeed is reliable
void Robot::checkIfStuck(){
  if (millis() >= nextTimeCheckIfStuck) {
    nextTimeCheckIfStuck = millis() + 500;
    if ((gpsUse) && (gps.hdop() < 500) && (gps.satellites() > 3) && (gps.satellites() < 250) ) {

      float gpsSpeed = gps.f_speed_kmph();
      if (gpsSpeed > stuckIfGpsSpeedBelow) {
        gpsAchievedSpeed = gpsSpeed;
        gpsSpeedUpdatedTime = millis();
      } else if (millis() >= gpsSpeedUpdatedTime + 20000) gpsAchievedSpeed = 0;  // do not use GPS speed if not updated in last 20 seconds
      
      if ((stateCurr != STATE_MANUAL) 
          && (stateCurr != STATE_REMOTE) 
          && (gpsSpeed <= stuckIfGpsSpeedBelow)   // checks if mower is stuck and counts up
          && (gpsAchievedSpeed > stuckIfGpsSpeedBelow) // gpsSpeed not reliable. Only add error if the gpsSpeed has been above the stuck limit.
          && (abs(motorLeftRpmCurr) > 1 || abs(motorRightRpmCurr) > 1) 
          && (millis() >= stateStartTime + gpsSpeedIgnoreTime) ){
        robotIsStuckCounter++;
      } else {                        // if mower gets unstuck it resets errorCounterMax to zero and reenabling motorMow
        robotIsStuckCounter = 0;    // resets temporary counter to zero
        if ( (errorCounter[ERR_STUCK] == 0) && (stateCurr != STATE_OFF) && (stateCurr != STATE_MANUAL) && (stateCurr != STATE_STATION) 
          && (stateCurr != STATE_STATION_CHARGING) && (stateCurr != STATE_STATION_CHECK) 
          && (stateCurr != STATE_STATION_REV) && (stateCurr != STATE_STATION_ROLL) 
          && (stateCurr != STATE_REMOTE) && (stateCurr != STATE_ERROR)) {
          motorMowEnable = true;
          errorCounterMax[ERR_STUCK] = 0;
        }
        return;
      }

      if (robotIsStuckCounter >= 5){    
        //motorMowEnable = false;
        if (errorCounterMax[ERR_STUCK] >= 3){   // robot is definately stuck and unable to move
          Console.println(F("Error: Mower is stuck"));
          addErrorCounter(ERR_STUCK);
          setSensorTriggered(SEN_GPS_SPEED);
          setNextState(STATE_ERROR,0);    //mower is switched into ERROR
          //robotIsStuckCounter = 0;
        }
        else if (errorCounter[ERR_STUCK] < 3) {   // mower tries 3 times to get unstuck
          if (stateCurr == STATE_FORWARD || stateCurr == STATE_CIRCLE){
           // motorMowEnable = false;
					  addErrorCounter(ERR_STUCK);             
            setSensorTriggered(SEN_GPS_SPEED);
					  setMotorPWM( 0, 0, false );  
					  reverseOrBidir(RIGHT);
				  }
				  else if (stateCurr == STATE_ROLL){
					 // motorMowEnable = false;
					  addErrorCounter(ERR_STUCK);             
	          setSensorTriggered(SEN_GPS_SPEED);
  				  setMotorPWM( 0, 0, false );  
					  setNextState (STATE_FORWARD,0);
				  }
        }
      }
      
    }
  }
}


void Robot::processGPSData()
{
  long nlat, nlon;
  unsigned long age;
  gps.get_position(&nlat, &nlon, &age);
  if (nlat == GPS::GPS_INVALID_F_ANGLE ) return;
  if (gpsLon == 0){
    gpsLon = nlon;  // this is xy (0,0)
    gpsLat = nlat;
    return;
  } else {
    gpsLon = nlon; // storing lon and lat for others to use
    gpsLat = nlat;
  }
  gpsX = (float)gps.distance_between(nlat,  gpsLon,  gpsLat, gpsLon);
  gpsY = (float)gps.distance_between(gpsLat, nlon,   gpsLat, gpsLon);
}

void Robot::checkTimeout(){
  if (stateTime > motorForwTimeMax){ 
    // timeout 
    motorMowSenseErrorCounter = 0;
    if (rollDir == RIGHT) setNextState(STATE_REVERSE, LEFT); // toggle roll dir
      else setNextState(STATE_REVERSE, RIGHT);
  }
}


const char* Robot::stateName(byte nameForThisState){
  if (nameForThisState < 0) return "-";
  else return stateNames[nameForThisState];
}

const char* Robot::errorName(byte nameForThisError){
  if (nameForThisError < 0) return "-";
  else return errorNames[nameForThisError];
}

const char* Robot::sensorName(byte nameForThisSensor){
  if (nameForThisSensor < 0) return "-";
  else return sensorNames[nameForThisSensor];
}

// set state machine new state
// http://wiki.ardumower.de/images/f/ff/Ardumower_states.png
// called *ONCE* to set to a *NEW* state
void Robot::setNextState(byte stateNew, byte dir){
  unsigned long stateTime = millis() - stateStartTime;
  
  if (stateNew == stateCurr) return;
  
  // state correction  
	if ((stateNew == STATE_ERROR) && (stateCurr == STATE_STATION_CHARGING)) {
		stateNew = STATE_STATION_CHARGING; // do not enter ERROR state when charging
	}
  
  if ((stateCurr == STATE_PERI_FIND) || (stateCurr == STATE_PERI_TRACK)) {
    if (stateNew == STATE_ROLL) stateNew = STATE_PERI_ROLL;
    if (stateNew == STATE_REVERSE) stateNew = STATE_PERI_REV;    
  }

  if (stateNew == STATE_FORWARD) {    
    if ((stateCurr == STATE_STATION_REV) ||(stateCurr == STATE_STATION_ROLL) || (stateCurr == STATE_STATION_CHECK) ) return;  
    if ((stateCurr == STATE_STATION) || (stateCurr == STATE_STATION_CHARGING)) {
      stateNew = STATE_STATION_CHECK;         
    } 
  }

  // evaluate new state
  stateNext = stateNew;
  rollDir = dir;
  if (stateNew == STATE_STATION_REV){
    motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm;                    
    stateEndTime = millis() + stationRevTime + motorZeroSettleTime;                     
		setActuator(ACT_CHGRELAY, 0);         
  } else if (stateNew == STATE_STATION_ROLL) {
      motorLeftSpeedRpmSet = motorSpeedMaxRpm;
      motorRightSpeedRpmSet = -motorSpeedMaxRpm;						      
      stateEndTime = millis() + stationRollTime + motorZeroSettleTime;                     
    } else if (stateNew == STATE_STATION_FORW) {
        motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm;      
        motorMowEnable = true;    
        stateEndTime = millis() + stationForwTime + motorZeroSettleTime;                     
      } else if (stateNew == STATE_STATION_CHECK) {
          motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm/2; 
          stateEndTime = millis() + stationCheckTime + motorZeroSettleTime; 
		      setActuator(ACT_CHGRELAY, 0);         
          motorMowEnable = false;
        } else if (stateNew == STATE_PERI_ROLL) {    
            stateEndTime = millis() + perimeterTrackRollTime + motorZeroSettleTime;                     
            if (dir == RIGHT){
	            motorLeftSpeedRpmSet = motorSpeedMaxRpm/2;
	            motorRightSpeedRpmSet = -motorLeftSpeedRpmSet;						
            } else {
              motorRightSpeedRpmSet = motorSpeedMaxRpm/2;
              motorLeftSpeedRpmSet = -motorRightSpeedRpmSet;	
            }
          } 
  
  
  if (stateNew == STATE_PERI_REV) {
    motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm/2;                    
    stateEndTime = millis() + perimeterTrackRevTime + motorZeroSettleTime;                     
  } else if (stateNew == STATE_PERI_OUT_FORW) {
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm;      
      stateEndTime = millis() + perimeterOutRevTime + motorZeroSettleTime + 1000;   
    } else if (stateNew == STATE_PERI_OUT_REV) {
        motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm/1.25;                    
        stateEndTime = millis() + perimeterOutRevTime + motorZeroSettleTime; 
      } else if (stateNew == STATE_PERI_OUT_ROLL) {
    	    //Ehl
          //imuDriveHeading = scalePI(imuDriveHeading + PI); // toggle heading 180 degree (IMU)
	        if (imuRollDir == LEFT) {
		        imuDriveHeading = scalePI(imuDriveHeading - random((PI / 2), PI )); // random toggle heading between 90 degree and 180 degrees (IMU)
		        imuRollHeading = scalePI(imuDriveHeading);
		        imuRollDir = rollDir;
	        } else	{
		        imuDriveHeading = scalePI(imuDriveHeading + random((PI / 2), PI )); // random toggle heading between 90 degree and 180 degrees (IMU)
		        imuRollHeading = scalePI(imuDriveHeading);
		        imuRollDir = rollDir;
	        }
      
          stateEndTime = millis() + random(perimeterOutRollTimeMin,perimeterOutRollTimeMax) + motorZeroSettleTime;
	
          if (dir == RIGHT) {
		        motorLeftSpeedRpmSet = motorSpeedMaxRpm/1.25;
		        motorRightSpeedRpmSet = -motorLeftSpeedRpmSet;           
	        } else {
            motorRightSpeedRpmSet = motorSpeedMaxRpm/1.25;
		        motorLeftSpeedRpmSet = -motorRightSpeedRpmSet; 
	        }
        } else if (stateNew == STATE_FORWARD) {      
            motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm;  
            statsMowTimeTotalStart = true;            
		        setActuator(ACT_CHGRELAY, 0);         
          } else if (stateNew == STATE_CIRCLE) {  
              if (rollDir == LEFT) {
                motorLeftSpeedRpmSet = motorSpeedMaxRpm/1.25;
                motorRightSpeedRpmSet = 0;
              } else {
                motorLeftSpeedRpmSet = 0;
                motorRightSpeedRpmSet = motorSpeedMaxRpm/1.25;
              }   
              mowIncreaseCircleRadiusTime = 0;
              currentCirclingStep = 0;
              statsMowTimeTotalStart = true;            
              setActuator(ACT_CHGRELAY, 0);         
            } else if (stateNew == STATE_GPSPERIMETER_CHANGE_DIR) {
                stateEndTime = millis() + 15000;
                gpsPerimeterRollState = 0;
                gpsPerimeterRollSubStateStartTime = millis();
                gpsPerimeterRollNewHeading = gpsPerimeter.getNewHeadingFromPerimeterDegrees(gpsLat, gpsLon);
                motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0; 
              } else if (stateNew == STATE_GPS_LONG_GRASS_CHANGE_DIR) {
                  stateEndTime = millis() + 15000;
                  gpsPerimeterRollState = 0;
                  gpsPerimeterRollSubStateStartTime = millis();
                  gpsPerimeterRollNewHeading = gpsPerimeter.getNewHeadingLongGrassAreaDegrees(gpsLat, gpsLon);
                  motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0; 
                  } else if (stateNew == STATE_REVERSE) {
                    motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm/1.25;                    
                    stateEndTime = millis() + motorReverseTime + motorZeroSettleTime;
                  } else if (stateNew == STATE_BUMPER_REVERSE) {
                      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm / 1.25;
                      stateEndTime = millis() + motorReverseTime + motorZeroSettleTime;
                    } else if (stateNew == STATE_BUMPER_FORWARD) {
                        motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.25;
                        stateEndTime = millis() + motorReverseTime + motorZeroSettleTime;
                      } else if (stateNew == STATE_ROLL) {                  
                          imuDriveHeading = scalePI(imuDriveHeading + PI); // toggle heading 180 degree (IMU)
                          if (imuRollDir == LEFT){
                            imuRollHeading = scalePI(imuDriveHeading - PI/20);        
                            imuRollDir = RIGHT;
                          } else {
                            imuRollHeading = scalePI(imuDriveHeading + PI/20);        
                            imuRollDir = LEFT;
                          }      
  
                          stateEndTime = millis() + random(motorRollTimeMin,motorRollTimeMax) + motorZeroSettleTime;
              
                          if (dir == RIGHT){
	                         motorLeftSpeedRpmSet = motorSpeedMaxRpm/1.25;
	                         motorRightSpeedRpmSet = -motorLeftSpeedRpmSet;						
                          } else {
                            motorRightSpeedRpmSet = motorSpeedMaxRpm/1.25;
                            motorLeftSpeedRpmSet = -motorRightSpeedRpmSet;	
                          }      
}  
  
  if (stateCurr == STATE_STATION_CHARGING) {
		// always switch off charging relay if leaving state STATE_STATION_CHARGING
		setActuator(ACT_CHGRELAY, 0); 
	}
  
  if (stateNew == STATE_REMOTE){
    motorMowEnable = true;
    //motorMowModulate = false;              
  } 
  
  if (stateNew == STATE_STATION){
    setMotorPWM(0,0,false);
    setActuator(ACT_CHGRELAY, 0); 
    setDefaults(); 
    statsMowTimeTotalStart = false;  // stop stats mowTime counter
    loadSaveRobotStats(false);        //save robot stats
       
  }
  
  if (stateNew == STATE_STATION_CHARGING){
    setActuator(ACT_CHGRELAY, 1); 
    setDefaults();        
  }
  
  if (stateNew == STATE_OFF){
    setActuator(ACT_CHGRELAY, 0);
    setDefaults();   
    statsMowTimeTotalStart = false; // stop stats mowTime counter
    loadSaveRobotStats(false);      //save robot stats
  } 
  
  if (stateNew == STATE_TILT_STOP){
     motorMowEnable = false;    
     motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0; 
  }
  
  if (stateNew == STATE_ERROR){
    motorMowEnable = false;    
    motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0; 
    setActuator(ACT_CHGRELAY, 0);
    statsMowTimeTotalStart = false;  
    //loadSaveRobotStats(false);   
  }
  
  if (stateNew == STATE_PERI_FIND){
      motorLeftSpeedRpmSet = motorRightSpeedRpmSet = motorSpeedMaxRpm / 1.5;    
    //motorMowEnable = false;     // FIXME: should be an option?
  }
  
  if (stateNew == STATE_GPS_HOMING_FIRST_TURN){
    if (gpsHomingInUse && imuUse) {
      imuDriveHeading = gpsPerimeter.getHeadingToClosestHomingPointDegrees(gpsLat, gpsLon)*PI/180;
      imuRollHeading = imuDriveHeading;
      imuRollDir = gpsPerimeter.getShortestWayToTurnDegrees((imu.ypr.yaw/PI*180.0) , (imuDriveHeading/PI*180.0));
      
      if (dir == RIGHT){
        motorLeftSpeedRpmSet = motorSpeedMaxRpm/1.25;
        motorRightSpeedRpmSet = -motorLeftSpeedRpmSet;           
      } else {
        motorRightSpeedRpmSet = motorSpeedMaxRpm/1.25;
        motorLeftSpeedRpmSet = -motorRightSpeedRpmSet;  
      }      
  
      stateEndTime = millis() + random(motorRollTimeMin,motorRollTimeMax) + motorZeroSettleTime;
    }
  }

  if (stateNew == STATE_GPS_HOMING_FOLLOW_POINTS) {
      motorLeftSpeedRpmSet = motorSpeedMaxRpm/1.25;
      motorRightSpeedRpmSet = motorSpeedMaxRpm/1.25;           

      imuDriveHeading = gpsPerimeter.getHeadingToClosestHomingPointDegrees(gpsLat, gpsLon)*PI/180;
      imuRollHeading = imuDriveHeading;
      imuRollDir = gpsPerimeter.getShortestWayToTurnDegrees((imu.ypr.yaw/PI*180.0) , (imuDriveHeading/PI*180.0));
  }
  
  if (stateNew == STATE_PERI_TRACK){        
    //motorMowEnable = false;     // FIXME: should be an option?
    perimeterMagMaxValue = perimeterMagMedian.getHighest();
    setActuator(ACT_CHGRELAY, 0);
    perimeterPID.reset();
    //beep(6);
  }   
  
  if (stateNew != STATE_REMOTE){
    motorMowSpeedPWMSet = motorMowSpeedMaxPwm;
  }      

  sonarObstacleTimeout = 0;
  // state has changed    
  stateStartTime = millis();
  stateLast = stateCurr;
  stateCurr = stateNext;    
  perimeterTriggerTime=0;

  for (int shiftArrayCounter = 28; shiftArrayCounter >= 0; shiftArrayCounter--) {
    stateLog[shiftArrayCounter+1] = stateLog[shiftArrayCounter];
  }
  stateLog[0] = stateCurr;

  if (rmcsUse == false) {  
     printInfo(Console);          
  }
  else{
    rmcsPrintInfo(Console);
  }

}// -------------------------- ENDE void Robot::setNextState(byte stateNew, byte dir)

/* markor debug
double Robot::debugTimer(double currentMax, double lastTime, int stage) {
 double deltaTime = millis() - lastTime;
 currentMax = max(currentMax, deltaTime);
 return currentMax;
}
*/

void Robot::loop()  {
  stateTime = millis() - stateStartTime;
  int steer;
  ADCMan.run();

  if (stateCurr != STATE_ROS) readSerial();   

  if (!rmcsUse){
     if (rc.readSerial()) resetIdleTime();
  } else {
    rc.readSerial();
  }
  
  readSensors(); 
  checkBattery(); 
  checkIfStuck();
  checkRobotStats();
  calcOdometry();
  checkOdometryFaults();    
  checkButton(); 
  motorMowControl(); 
  checkTilt(); 

  if (imuUse) { 
    if (millis() > nextTimeIMU) {
      nextTimeIMU = millis() + 100;
      imu.update();  
      //checkIfImuAccelerationMaxed();
    }
  }

  if (gpsUse) { 
    if (millis() > nextTimeGPS) {
      nextTimeGPS = millis() + 100;
      gps.feed();
      processGPSData();
    }
  }

  if (millis() >= nextTimePfodLoop){
    nextTimePfodLoop = millis() + 200;
    rc.run();        
  }
   
  if (rmcsUse == true and millis() >= nextTimeRMCSInfo ) { 
	   nextTimeRMCSInfo = millis() + 100;
     rmcsPrintInfo(Console);
  }

  if (millis() >= nextTimeInfo) {        
    nextTimeInfo = millis() + 1000; 
  
    //gpsPerimeter.doUnitTest();
    //gpsPerimeter.getNewHeadingFromPerimeterDegrees(gpsLat, gpsLon);
    //gpsPerimeter.insidePerimeter(random(6247269,6247280), random(2544013,2544030));
    
    if (rmcsUse == false) { 
      printInfo(Console); 
      printErrors();
    }
    if (stateCurr != STATE_ROS && rmcsUse == false) {
      printInfo(Console);    
      printErrors();
    }    
    ledState = ~ledState;    
    /*if (ledState) setActuator(ACT_LED, HIGH);
      else setActuator(ACT_LED, LOW);        */
    //checkErrorCounter();  
    if (stateCurr == STATE_REMOTE) printRemote();    

    loopsPerSec = loopsPerSecCounter;					
		if (stateCurr != STATE_ERROR){		
			if (loopsPerSec < 10) { // loopsPerSec too low
				if (loopsPerSecLowCounter < 255) loopsPerSecLowCounter++;
			} else if (loopsPerSecLowCounter > 0) loopsPerSecLowCounter--; // loopsPerSec OK
      
      if (loopsPerSecLowCounter > 10) { // too long I2C cables can be a reason for this
			  Console.println(F("Error: loopsPerSec too low (check I2C cables)"));
			  addErrorCounter(ERR_CPU_SPEED);
			  setNextState(STATE_ERROR,0);    //mower is switched into ERROR
      }
		} else loopsPerSecLowCounter = 0; // reset counter to zero
    
    if (loopsPerSec > 0) loopsTa = 1000.0 / ((double)loopsPerSec);    
    loopsPerSecMax = loopsPerSecCounter;
    loopsPerSecCounter = 0;    
  }   
     
   // state machine - things to do *PERMANENTLY* for current state
   // robot state machine
   // http://wiki.ardumower.de/images/f/ff/Ardumower_states.png
  switch (stateCurr) {
    case STATE_TILT_STOP:
      // tilt      
      if (millis() >= nextTimeErrorBeep){
        nextTimeErrorBeep = millis() + 1000;
        beep(1, true);
      }	
		  if (millis() >= stateStartTime + 10000)  setNextState(STATE_OFF, 0);
			if (!tilt) setNextState(stateLast, 0);
      break;
    case STATE_ERROR:
      // fatal-error
      if (millis() >= nextTimeErrorBeep){
        nextTimeErrorBeep = millis() + 5000;
        beep(1, true);
      }			      

      if (stateStartTime + 10000 
       && errorCounterMax[lastErrType] <= 5) 
      {
        if (lastErrType == ERR_ADC_CALIB);
        if (lastErrType == ERR_CHARGER);
        if (lastErrType == ERR_BATTERY);
        
        if (lastErrType == ERR_MOTOR_LEFT
         || lastErrType == ERR_MOTOR_RIGHT) 
        {
          if (stateLast == STATE_REVERSE) {
            setNextState(STATE_FORWARD,0);
          } else {
            setNextState(STATE_REVERSE,0);            
          }
        }
        
        if (lastErrType == ERR_MOTOR_MOW);
        if (lastErrType == ERR_MOW_SENSE);
        
        if (lastErrType == ERR_ODOMETRY_LEFT
         || lastErrType == ERR_ODOMETRY_RIGHT)
        {
          if (stateLast == STATE_REVERSE) {
            setNextState(STATE_FORWARD,0);
          } else {
            setNextState(STATE_REVERSE,0);            
          }
        }
        if (lastErrType == ERR_TRACKING);
        if (lastErrType == ERR_IMU_COMM);
        if (lastErrType == ERR_IMU_CALIB);
        if (lastErrType == ERR_IMU_TILT);
        if (lastErrType == ERR_RTC_COMM);
        if (lastErrType == ERR_RTC_DATA);
        if (lastErrType == ERR_GPS_COMM);
        if (lastErrType == ERR_GPS_DATA);
        if (lastErrType == ERR_STUCK);
        if (lastErrType == ERR_EEPROM_DATA);
        
        if (lastErrType == ERR_CPU_SPEED) {
          if (stateLast == STATE_REVERSE) {
            setNextState(STATE_FORWARD,0);
          } else {
            setNextState(STATE_REVERSE,0);            
          }
        }
      
        if (lastErrType == ERR_PERIMETER_TIMEOUT) {
          if (perimeter.isInside(0)) {
            if(rotateLeft){  
              setNextState(STATE_PERI_OUT_REV, LEFT);
            } else {
              setNextState(STATE_PERI_OUT_REV, RIGHT);
            }
          }
        }

      }
      


      break;
    case STATE_OFF:
      // robot is turned off      
      //checkTimer();   // deactivated due to safety issues. when mower is off it should stay off. timer is only active when mower is n STATE_STATION.
      if (batMonitor && (millis()-stateStartTime>2000)){
        if (chgVoltage > 5.0) {
          beep(2, true);      
          setNextState(STATE_STATION, 0);
        }
      }
      imuDriveHeading = imu.ypr.yaw;
      break;
    case STATE_ROS:
      // Linux ROS mode
      rosSerial();   
      break;
    case STATE_REMOTE:
      // remote control mode (RC)
      //if (remoteSwitch > 50) setNextState(STATE_FORWARD, 0);
      steer = ((double)motorSpeedMaxRpm/2) * (((double)remoteSteer)/100.0);
      if (remoteSpeed < 0) steer *= -1;
      motorLeftSpeedRpmSet  = ((double)motorSpeedMaxRpm) * (((double)remoteSpeed)/100.0) - steer; 
      motorRightSpeedRpmSet = ((double)motorSpeedMaxRpm) * (((double)remoteSpeed)/100.0) + steer; 
      motorLeftSpeedRpmSet = max(-motorSpeedMaxRpm, min(motorSpeedMaxRpm, motorLeftSpeedRpmSet));
      motorRightSpeedRpmSet = max(-motorSpeedMaxRpm, min(motorSpeedMaxRpm, motorRightSpeedRpmSet));
      motorMowSpeedPWMSet = ((double)motorMowSpeedMaxPwm) * (((double)remoteMow)/100.0);      
      break;
    case STATE_MANUAL:
      //bb add  
      checkCurrent();
      checkBumpers();
      checkDrop();
      //bb add end
      break;
    case STATE_FORWARD:
      // driving forward            
      if (mowPatternCurr == MOW_BIDIR){
        double ratio = motorBiDirSpeedRatio1;
        if (stateTime > 4000) ratio = motorBiDirSpeedRatio2;
        if (rollDir == RIGHT) motorRightSpeedRpmSet = ((double)motorLeftSpeedRpmSet) * ratio;
          else motorLeftSpeedRpmSet = ((double)motorRightSpeedRpmSet) * ratio;                            
      }             
      checkErrorCounter();    
      checkTimer();
      checkRain();
      checkCurrent();
      checkFreeWheel();            
      checkBumpers();      
      checkDrop();                                                                                                                            // Dropsensor - Absturzsensor
      checkSonar();             
      checkPerimeterBoundary(); 
      checkLawn();      
      checkTimeout();  

      break;
    case STATE_ROLL:
      checkCurrent();            
      checkFreeWheel();
      checkBumpers();
      checkDrop();                                                                                                                            // Dropsensor - Absturzsensor
      //checkSonar();             
      checkPerimeterBoundary(); 
      checkLawn();
      // making a roll (left/right)            
      if (mowPatternCurr == MOW_LANES){
        if (abs(distancePI(imu.ypr.yaw, imuRollHeading)) < PI/36) setNextState(STATE_FORWARD,0);				        
      } else {
        if (millis() >= stateEndTime) {
          setNextState(STATE_FORWARD,0);				          
        }        
      }
      break;
    case STATE_GPSPERIMETER_CHANGE_DIR:
      checkErrorCounter();    
      checkTimer();
      checkRain();
      checkCurrent();
      checkFreeWheel();            
      checkBumpers();      
      checkDrop();                                                                                                                            // Dropsensor - Absturzsensor
      checkPerimeterBoundary(); 
      checkLawn();      
      checkTimeout();    

      // fail-safe if didn't complete our turn sequence
      if (millis() > stateEndTime) setNextState(STATE_FORWARD,rollDir);

      if (gpsPerimeterRollState == 0) {
        if (millis() > (gpsPerimeterRollSubStateStartTime + 500)) { //+ motorZeroSettleTime
          gpsPerimeterRollState = 1;
          gpsPerimeterRollSubStateStartTime = millis();
        }
        break ;
      } 

      if (gpsPerimeterRollState == 1) {
        motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm;                    
        gpsPerimeterRollState = 2;
        gpsPerimeterRollSubStateStartTime = millis();
        break;
      } 

      if (gpsPerimeterRollState == 2) {      
        if (millis() >= gpsPerimeterRollSubStateStartTime + 1500){
          motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
          gpsPerimeterRollState = 3;
          gpsPerimeterRollSubStateStartTime = millis();
        }
        break;
      }

      if (gpsPerimeterRollState == 3) {
        if (millis() > (gpsPerimeterRollSubStateStartTime + 500)) { //+ motorZeroSettleTime
          gpsPerimeterRollState = 4;
          gpsPerimeterRollSubStateStartTime = millis();
        }
        break ;
      } 

      if (gpsPerimeterRollState == 4) {      
 
        float currentHeading = imu.ypr.yaw/PI*180.0;
        float first = abs(gpsPerimeterRollNewHeading - currentHeading);
        float second = abs(gpsPerimeterRollNewHeading - currentHeading + 360);
        float third = abs(gpsPerimeterRollNewHeading - currentHeading - 360);

        rollDir = RIGHT;
        if (first < second && first < third && (gpsPerimeterRollNewHeading - currentHeading) > 0) rollDir = LEFT;
        else if (second < first && second < third && (gpsPerimeterRollNewHeading - currentHeading + 360) > 0) rollDir = LEFT;
        else if (third < first && third < second && (gpsPerimeterRollNewHeading- currentHeading - 360) > 0) rollDir = LEFT;

        if (rollDir == LEFT) {
          motorLeftSpeedRpmSet  =  motorSpeedMaxRpm * 0.5;
          motorRightSpeedRpmSet = -motorSpeedMaxRpm * 0.5;                    
        } else {
          motorLeftSpeedRpmSet  = -motorSpeedMaxRpm * 0.5;
          motorRightSpeedRpmSet =  motorSpeedMaxRpm * 0.5;                    
        }

        gpsPerimeterRollState = 5;
        gpsPerimeterRollSubStateStartTime = millis();
 
        break; 
      }

      if (gpsPerimeterRollState == 5) {  
        if (millis() > gpsPerimeterRollSubStateStartTime + 102) {
          if (rollDir == LEFT) {
            motorLeftSpeedRpmSet  =  motorSpeedMaxRpm * 0.5;
            motorRightSpeedRpmSet = -motorSpeedMaxRpm * 0.5;                    
          } else {
            motorLeftSpeedRpmSet  = -motorSpeedMaxRpm * 0.5;
            motorRightSpeedRpmSet =  motorSpeedMaxRpm * 0.5;                    
          }
  
          float currentHeading = imu.ypr.yaw/PI*180.0;
  
          float first = abs(gpsPerimeterRollNewHeading - currentHeading);
          float second = abs(gpsPerimeterRollNewHeading - currentHeading + 360);
          float third = abs(gpsPerimeterRollNewHeading - currentHeading - 360);
  
          float minimumAngle = min(first, second);
          minimumAngle = min(minimumAngle, third);
  
          if (abs(minimumAngle) <= 10) {
            gpsPerimeterRollState = 6;
            gpsPerimeterRollSubStateStartTime = millis();
          } 
        }
        break;
      }

      if (gpsPerimeterRollState == 6) {  
        motorLeftSpeedRpmSet = 0;
        motorRightSpeedRpmSet = 0;                    

        if (millis() >= gpsPerimeterRollSubStateStartTime + 300){
          gpsPerimeterRollState = 7;
          gpsPerimeterRollSubStateStartTime = millis();
        }
        break;
      }

      if (gpsPerimeterRollState == 7) {      
//        motorLeftSpeedRpmSet = motorSpeedMaxRpm/1.25;
//        motorRightSpeedRpmSet = motorSpeedMaxRpm/1.25;
        gpsPerimeterRollState = 8;
//        gpsPerimeterRollSubStateStartTime = millis();
//        break;
      }

      if (gpsPerimeterRollState == 8) {      
       
        if (gpsPerimeter.insidePerimeter(gpsLat, gpsLon) != 0) {
          gpsPerimeterRollState = 9;
          gpsPerimeterRollSubStateStartTime = millis();
        } else {
          if (millis() >= gpsPerimeterRollSubStateStartTime + 100) {
          
            float currentHeading = imu.ypr.yaw/PI*180.0;
            float first = abs(gpsPerimeterRollNewHeading - currentHeading);
            float second = abs(gpsPerimeterRollNewHeading - currentHeading + 360);
            float third = abs(gpsPerimeterRollNewHeading - currentHeading - 360);

            rollDir = RIGHT;
            if (first < second && first < third && (gpsPerimeterRollNewHeading - currentHeading) > 0) rollDir = LEFT;
            else if (second < first && second < third && (gpsPerimeterRollNewHeading - currentHeading + 360) > 0) rollDir = LEFT;
            else if (third < first && third < second && (gpsPerimeterRollNewHeading- currentHeading - 360) > 0) rollDir = LEFT;

            float minimumAngle = min(first, second);
            minimumAngle = min(minimumAngle, third);
  
            if (abs(minimumAngle) >= 10) {
              if (rollDir == LEFT) {
                motorLeftSpeedRpmSet  = motorSpeedMaxRpm/1.35;
                motorRightSpeedRpmSet = motorSpeedMaxRpm/1.25;
              } else {
                motorLeftSpeedRpmSet  = motorSpeedMaxRpm/1.25;
                motorRightSpeedRpmSet = motorSpeedMaxRpm/1.35;
              }
            } else {
              motorLeftSpeedRpmSet  = motorSpeedMaxRpm/1.25;
              motorRightSpeedRpmSet = motorSpeedMaxRpm/1.25;
            }

//            gpsPerimeterRollState = 9;
            gpsPerimeterRollSubStateStartTime = millis();
          }
        }
        break;
      }

      if (gpsPerimeterRollState == 9) {  
//        motorLeftSpeedRpmSet = 0;
//        motorRightSpeedRpmSet = 0;                    
        gpsPerimeterRollState = 10;
        gpsPerimeterRollSubStateStartTime = millis();
        break;
      }

      if (gpsPerimeterRollState == 10) {      
//        if (millis() >= gpsPerimeterRollSubStateStartTime + 300){
          gpsPerimeterRollState = 11;
          gpsPerimeterRollSubStateStartTime = millis();
//        }
        break;
      }

      if (gpsPerimeterRollState == 11) {  
        gpsPerimeterRollState = 0;
        setNextState(STATE_FORWARD, rollDir);
      }

      break;

    case STATE_GPS_LONG_GRASS_CHANGE_DIR:
//        checkErrorCounter();    
//      checkTimer();
//      checkRain();
//      checkCurrent();
//      checkFreeWheel();            
//      checkBumpers();      
//      checkDrop();                                                                                                                            // Dropsensor - Absturzsensor
      //checkPerimeterBoundary(); 
//      checkLawn();      
//      checkTimeout();    

      // fail-safe if didn't complete our turn sequence
      if (millis() > stateEndTime) setNextState(STATE_FORWARD,rollDir);

      if (gpsPerimeterRollState == 0) {
        if (millis() > (gpsPerimeterRollSubStateStartTime + 500)) { //+ motorZeroSettleTime
          gpsPerimeterRollState = 1;
          gpsPerimeterRollSubStateStartTime = millis();
        }
        break ;
      } 

      if (gpsPerimeterRollState == 1) {
        motorLeftSpeedRpmSet = motorRightSpeedRpmSet = -motorSpeedMaxRpm;                    
        gpsPerimeterRollState = 2;
        gpsPerimeterRollSubStateStartTime = millis();
        break;
      } 

      if (gpsPerimeterRollState == 2) {      
        if (millis() >= gpsPerimeterRollSubStateStartTime + 500){
          motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
          gpsPerimeterRollState = 3;
          gpsPerimeterRollSubStateStartTime = millis();
        }
        break;
      }

      if (gpsPerimeterRollState == 3) {
        if (millis() > (gpsPerimeterRollSubStateStartTime + 500)) { //+ motorZeroSettleTime
          gpsPerimeterRollState = 4;
          gpsPerimeterRollSubStateStartTime = millis();
        }
        break ;
      } 

      if (gpsPerimeterRollState == 4) {      
 
        float currentHeading = imu.ypr.yaw/PI*180.0;

        float first = abs(gpsPerimeterRollNewHeading - currentHeading);
        float second = abs(gpsPerimeterRollNewHeading - currentHeading + 360);
        float third = abs(gpsPerimeterRollNewHeading - currentHeading - 360);

        rollDir = RIGHT;
        if (first < second && first < third && (gpsPerimeterRollNewHeading - currentHeading) > 0) rollDir = LEFT;
        else if (second < first && second < third && (gpsPerimeterRollNewHeading - currentHeading + 360) > 0) rollDir = LEFT;
        else if (third < first && third < second && (gpsPerimeterRollNewHeading- currentHeading - 360) > 0) rollDir = LEFT;

        if (rollDir == LEFT) {
          motorLeftSpeedRpmSet  =  motorSpeedMaxRpm * 0.5;
          motorRightSpeedRpmSet = -motorSpeedMaxRpm * 0.5;                    
        } else {
          motorLeftSpeedRpmSet  = -motorSpeedMaxRpm * 0.5;
          motorRightSpeedRpmSet =  motorSpeedMaxRpm * 0.5;                    
        }

        gpsPerimeterRollState = 5;
        gpsPerimeterRollSubStateStartTime = millis();
 
        break; 
      }

      if (gpsPerimeterRollState == 5) {  
        if (millis() > gpsPerimeterRollSubStateStartTime + 102) {
          if (rollDir == LEFT) {
            motorLeftSpeedRpmSet  =  motorSpeedMaxRpm * 0.5;
            motorRightSpeedRpmSet = -motorSpeedMaxRpm * 0.5;                    
          } else {
            motorLeftSpeedRpmSet  = -motorSpeedMaxRpm * 0.5;
            motorRightSpeedRpmSet =  motorSpeedMaxRpm * 0.5;                    
          }
  
          float currentHeading = imu.ypr.yaw/PI*180.0;
  
          float first = abs(gpsPerimeterRollNewHeading - currentHeading);
          float second = abs(gpsPerimeterRollNewHeading - currentHeading + 360);
          float third = abs(gpsPerimeterRollNewHeading - currentHeading - 360);
  
          float minimumAngle = min(first, second);
          minimumAngle = min(minimumAngle, third);
  
          if (abs(minimumAngle) <= 20) {
            gpsPerimeterRollState = 6;
            gpsPerimeterRollSubStateStartTime = millis();
          } 
        }
        break;
      }

      if (gpsPerimeterRollState == 6) {  
        motorLeftSpeedRpmSet = 0;
        motorRightSpeedRpmSet = 0;                    

        if (millis() >= gpsPerimeterRollSubStateStartTime + 300){
          gpsPerimeterRollState = 7;
          gpsPerimeterRollSubStateStartTime = millis();
        }
        break;
      }

      if (gpsPerimeterRollState == 7) {      
//        motorLeftSpeedRpmSet = motorSpeedMaxRpm/1.25;
//        motorRightSpeedRpmSet = motorSpeedMaxRpm/1.25;
        gpsPerimeterRollState = 8;
//        gpsPerimeterRollSubStateStartTime = millis();
//        break;
      }

      if (gpsPerimeterRollState == 8) {      
   //     checkSonar();             
        motorLeftSpeedRpmSet = motorSpeedMaxRpm/1.25;
        motorRightSpeedRpmSet = motorSpeedMaxRpm/1.25;
        if (millis() >= gpsPerimeterRollSubStateStartTime + 2000){
          gpsPerimeterRollState = 9;
          gpsPerimeterRollSubStateStartTime = millis();
        }
        break;
      }

      if (gpsPerimeterRollState == 9) {  
//        motorLeftSpeedRpmSet = 0;
//        motorRightSpeedRpmSet = 0;                    
        gpsPerimeterRollState = 10;
        gpsPerimeterRollSubStateStartTime = millis();
        break;
      }

      if (gpsPerimeterRollState == 10) {      
//        if (millis() >= gpsPerimeterRollSubStateStartTime + 300){
          gpsPerimeterRollState = 11;
          gpsPerimeterRollSubStateStartTime = millis();
//        }
        break;
      }

      if (gpsPerimeterRollState == 11) {  
        gpsPerimeterRollState = 0;
        setNextState(STATE_FORWARD, rollDir);
      }

      break;


    case STATE_ROLL_WAIT:
      // making a roll (left/right)            
      //if (abs(distancePI(imuYaw, imuRollHeading)) < PI/36) setNextState(STATE_OFF,0);				
      break;
    case STATE_CIRCLE:
      // driving circles
      
      checkErrorCounter();    
      checkTimer();
      checkRain();
      checkCurrent();
      checkFreeWheel();            
      checkBumpers();      
      checkDrop();                                                                                                                            // Dropsensor - Absturzsensor
      checkSonar();             
      checkPerimeterBoundary(); 
      checkLawn();      
      checkTimeout();     
      
      if (millis() >= mowIncreaseCircleRadiusTime) {
        float cmsPerSecond = odometryTicksPerRevolution*(motorSpeedMaxRpm/1.25)/60/odometryTicksPerCm;

        float innerWheelCircleDiameter = ( (motorMowCircleRadiusWidenCM * currentCirclingStep) / 2 ) * 2 * PI;  //currentCirclingStep * (odometryWheelBaseCm / 2) * 2 * PI;
        float outerWheelCircleDiameter = ( (odometryWheelBaseCm + motorMowCircleRadiusWidenCM * currentCirclingStep) / 2) * 2 * PI; //(currentCirclingStep + 1) * (odometryWheelBaseCm / 2) * 2 * PI;

        float secondsToCompleteOuterWheelCircle = outerWheelCircleDiameter/cmsPerSecond;

        if (rollDir == LEFT){      
          motorLeftSpeedRpmSet = motorSpeedMaxRpm/1.25;
          motorRightSpeedRpmSet = (int)(innerWheelCircleDiameter/secondsToCompleteOuterWheelCircle/cmsPerSecond*motorSpeedMaxRpm/1.25);
        } else {
          motorRightSpeedRpmSet = motorSpeedMaxRpm/1.25;
          motorLeftSpeedRpmSet = (int)(innerWheelCircleDiameter/secondsToCompleteOuterWheelCircle/cmsPerSecond*motorSpeedMaxRpm/1.25);
        }
        
        mowIncreaseCircleRadiusTime = millis() + secondsToCompleteOuterWheelCircle*1000;
        currentCirclingStep = currentCirclingStep + 1;
      }

      if ((rollDir == LEFT && motorRightSpeedRpmSet >= motorLeftSpeedRpmSet) || (rollDir == RIGHT && motorLeftSpeedRpmSet >= motorRightSpeedRpmSet)) {
        motorLeftSpeedRpmSet = 0;
        motorRightSpeedRpmSet = 0;
        mowIncreaseCircleRadiusTime = 0;
        currentCirclingStep = 0;
        if (rollDir == RIGHT) setNextState(STATE_FORWARD, LEFT); // toggle roll dir
        else setNextState(STATE_FORWARD, RIGHT);
      }

      
      break;
    case STATE_REVERSE:
      // driving reverse
        checkErrorCounter();    
        checkTimer();
        checkCurrent();            
        checkFreeWheel();
        checkBumpers();
        checkDrop();                                                                                                                            // Dropsensor - Absturzsensor
        //checkSonar();             
        checkPerimeterBoundary();      
        checkLawn();    
        
      if (mowPatternCurr == MOW_BIDIR){
        double ratio = motorBiDirSpeedRatio1;
        if (stateTime > 4000) ratio = motorBiDirSpeedRatio2;
        if (rollDir == RIGHT) motorRightSpeedRpmSet = ((double)motorLeftSpeedRpmSet) * ratio;
          else motorLeftSpeedRpmSet = ((double)motorRightSpeedRpmSet) * ratio;                                
        if (stateTime > motorForwTimeMax){ 
          // timeout 
          if (rollDir == RIGHT) setNextState(STATE_FORWARD, LEFT); // toggle roll dir
            else setNextState(STATE_FORWARD, RIGHT);
        }        
      } else {        
        if (millis() >= stateEndTime) {
          setNextState(STATE_ROLL, rollDir);				                         
        }
      }
      break;
    case STATE_BUMPER_FORWARD:
      // driving forward
      if (mowPatternCurr == MOW_BIDIR) {
        double ratio = motorBiDirSpeedRatio1;
        if (stateTime > 4000) ratio = motorBiDirSpeedRatio2;
        if (rollDir == RIGHT) motorRightSpeedRpmSet = ((double)motorLeftSpeedRpmSet) * ratio;
        else motorLeftSpeedRpmSet = ((double)motorRightSpeedRpmSet) * ratio;
      } else {
        if (millis() >= stateEndTime) {
          setNextState(STATE_ROLL, rollDir);
        }
      }
      checkErrorCounter();
      checkTimer();
      checkRain();
      checkCurrent();
      checkFreeWheel();
      checkBumpers();
      checkDrop();                                                                                                                            // Dropsensor - Absturzsensor
      checkSonar();
      checkPerimeterBoundary();
      checkLawn();
      checkTimeout();
      break;
    case STATE_BUMPER_REVERSE:
      // driving reverse
      checkErrorCounter();
      checkTimer();
      checkCurrent();
      checkFreeWheel();
      checkBumpers();
      checkDrop();                                                                                                                            // Dropsensor - Absturzsensor
      //checkSonar();
      checkPerimeterBoundary();
      checkLawn();

      if (mowPatternCurr == MOW_BIDIR) {
        double ratio = motorBiDirSpeedRatio1;
        if (stateTime > 4000) ratio = motorBiDirSpeedRatio2;
        if (rollDir == RIGHT) motorRightSpeedRpmSet = ((double)motorLeftSpeedRpmSet) * ratio;
        else motorLeftSpeedRpmSet = ((double)motorRightSpeedRpmSet) * ratio;
        if (stateTime > motorForwTimeMax) {
          // timeout
          if (rollDir == RIGHT) setNextState(STATE_FORWARD, LEFT); // toggle roll dir
          else setNextState(STATE_FORWARD, RIGHT);
        }
      } else {
        if (millis() >= stateEndTime) {
          setNextState(STATE_ROLL, rollDir);
        }
      }
      break;
    case STATE_PERI_ROLL:
      // perimeter tracking roll
      if (millis() >= stateEndTime) setNextState(STATE_PERI_FIND,0);				
      break;
    case STATE_PERI_REV:
      // perimeter tracking reverse
      if (millis() >= stateEndTime) setNextState(STATE_PERI_ROLL, RIGHT); //rollDir);				               
      
      break;
    case STATE_PERI_FIND:
      // find perimeter
      if (motorLeftSpeedRpmSet == motorRightSpeedRpmSet){ // do not check during 'outside=>inside' rotation              
        checkCurrent();
        checkBumpersPerimeter();
        checkSonar();                           
      }  
      checkPerimeterFind();      
      checkTimeout();                    
      break;
    case STATE_PERI_TRACK:
      // track perimeter
      checkCurrent();                  
      checkBumpersPerimeter();
      //checkSonar();                   
      if (batMonitor){
        if (chgVoltage > 5.0){ 
          setNextState(STATE_STATION, 0);
        }
      }
      break;
    case STATE_STATION:
      // waiting until auto-start by user or timer triggered
      if (batMonitor){
        if (chgVoltage > 5.0) {
          if (batVoltage < startChargingIfBelow 
//             && (millis()-stateStartTime>2000) 
             && (millis() >= batteryNextChargeAfterFull)) { // only try to start charging again if it's over 30mins from last time battery full
            batteryNextChargeAfterFull = 0; 
            setNextState(STATE_STATION_CHARGING,0);
          } else checkTimer();  
        } else setNextState(STATE_OFF,0);
      }  else checkTimer();
      break;     
    case STATE_STATION_CHARGING:
      // waiting until charging completed    
      if (batMonitor){
        if ((chgCurrent < batFullCurrent) && (millis()-stateStartTime>2000)) { 
          batteryNextChargeAfterFull = millis () + 1800000 ;
          setNextState(STATE_STATION,0); 
        } else if (millis()-stateStartTime > chargingTimeout) {            
						Console.println(F("Battery chargingTimeout"));
						addErrorCounter(ERR_BATTERY);
            setNextState(STATE_ERROR, 0);
          }
      } 
      break;  
    case STATE_PERI_OUT_FORW:  
      checkPerimeterBoundary();                 
      //if (millis() >= stateEndTime) setNextState(STATE_PERI_OUT_ROLL, rollDir);  
      if (perimeterInside || (millis() >= stateEndTime)) setNextState(STATE_PERI_OUT_ROLL, rollDir); 
      break;
    case STATE_PERI_OUT_REV: 
      checkPerimeterBoundary();      
     // if (millis() >= stateEndTime) setNextState(STATE_PERI_OUT_ROLL, rollDir);   
      if (perimeterInside || (millis() >= stateEndTime)) setNextState (STATE_PERI_OUT_ROLL, rollDir); 
      break;
    case STATE_PERI_OUT_ROLL: 
      if (millis() >= stateEndTime) setNextState(STATE_FORWARD,0);                
      break;

    case STATE_STATION_CHECK:
      // check for charging voltage disappearing before leaving charging station
      if (millis() >= stateEndTime-500){
        //setActuator(ACT_CHGRELAY, 1);  // do we need this?
				if (millis() >= stateEndTime){
					if (chgVoltage > 5) {
						addErrorCounter(ERR_CHARGER);
						setNextState(STATE_ERROR, 0);
					} else setNextState(STATE_STATION_REV, 0);
				}
      }      
      break;
    case STATE_STATION_REV:
      // charging: drive reverse 
      if (millis() >= stateEndTime) setNextState(STATE_STATION_ROLL, 0);				             
      break;
    case STATE_STATION_ROLL:
      // charging: roll 
      if (millis() >= stateEndTime) setNextState(STATE_STATION_FORW,0);				
      break;
    case STATE_STATION_FORW:
      // forward (charge station)    
      if (millis() >= stateEndTime) setNextState(STATE_FORWARD,0);				        
      break;

    case STATE_GPS_HOMING_FIRST_TURN:
      if (abs(distancePI(imu.ypr.yaw, imuRollHeading)) < PI/36) setNextState(STATE_GPS_HOMING_FOLLOW_POINTS,0);                
      if (millis() >= stateEndTime) setNextState(STATE_GPS_HOMING_FIRST_TURN,0);                  
      
      break;        

    case STATE_GPS_HOMING_FOLLOW_POINTS:
      imuDriveHeading = gpsPerimeter.getHeadingToNextHomingPointDegrees(gpsLat, gpsLon)*PI/180;
      imuRollHeading = imuDriveHeading;
      imuRollDir = gpsPerimeter.getShortestWayToTurnDegrees((imu.ypr.yaw/PI*180.0) , (imuDriveHeading/PI*180.0));
      if (gpsPerimeter.lastPointBeforeStation()) setNextState(STATE_PERI_FIND,0);
      break;
  } // end switch  
      

  // next line deactivated (issue with RC failsafe)
  //if ((useRemoteRC) && (remoteSwitch < -50)) setNextState(STATE_REMOTE, 0);

  
  // decide which motor control to use
  if ( ((mowPatternCurr == MOW_LANES) && (stateCurr == STATE_ROLL)) 
       || stateCurr == STATE_ROLL_WAIT 
       || stateCurr == STATE_GPS_HOMING_FIRST_TURN) motorControlImuRoll();
    
    else if (stateCurr == STATE_PERI_TRACK) motorControlPerimeter();

      else if (stateCurr == STATE_GPS_HOMING_FOLLOW_POINTS) motorControlImuDir();
      
        else if (  (stateCurr == STATE_FORWARD) 
                    && (imuUse) 
                    && (imuCorrectDir || (mowPatternCurr == MOW_LANES))) motorControlImuDir();                            
      
          else motorControl();  
  
  if (stateCurr != STATE_REMOTE) motorMowSpeedPWMSet = motorMowSpeedMaxPwm;
      
  bumperRight = false;
  bumperLeft = false;     
  
  dropRight = false;                                                                                                                             // Dropsensor - Absturzsensor
  dropLeft = false;                                                                                                                              // Dropsensor - Absturzsensor
                             
  loopsPerSecCounter++;  
}
