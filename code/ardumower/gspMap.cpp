
// Copyright 2000 softSurfer, 2012 Dan Sunday
// This code may be freely used and modified for any purpose
// providing that this copyright notice is included with it.
// SoftSurfer makes no warranty for this code, and cannot be held
// liable for any real or imagined damage resulting from its use.
// Users of this code must verify correctness for their application.

#include "Arduino.h"
#include "gpsMap.h"
#include "flashmem.h"


// a Point is defined by its coordinates {int x, y;}
//===================================================================
//typedef struct {int x, y;} Point;
//typedef struct {int x, y;} Point;

// isLeft(): tests if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2  on the line
//            <0 for P2  right of the line
//    See: Algorithm 1 "Area of Triangles and Polygons"
//inline int isLeft(Point P0, Point P1, Point P2) __attribute__((always_inline));

float gpsMap::isLeft( Point P0, Point P1, Point P2 )
{
    return ( (P1.x - P0.x) * (P2.y - P0.y)
            - (P2.x -  P0.x) * (P1.y - P0.y) );
}
//===================================================================

// wn_PnPoly(): winding number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  wn = the winding number (=0 only when P is outside)
/*
int gpsMap::insidePerimeter(float x, float y)
{
    int    wn = 0;    // the  winding number counter
    Point _currentLocation = {x,y};

    // loop through all edges of the polygon
    for (int i=0; i<_numberOfMainAreaPoints; i++) {   // edge from area[i] to  area[i+1]
        if (_mainAreaPointList[i].y <= _currentLocation.y) {          // start y <= _currentLocation.y
            if (_mainAreaPointList[i+1].y  > _currentLocation.y)      // an upward crossing
                 if (isLeft( _mainAreaPointList[i], _mainAreaPointList[i+1], _currentLocation) > 0)  // P left of  edge
                     ++wn;            // have  a valid up intersect
        }
        else {                        // start y > _currentLocation.y (no test needed)
            if (_mainAreaPointList[i+1].y  <= _currentLocation.y)     // a downward crossing
                 if (isLeft( _mainAreaPointList[i], _mainAreaPointList[i+1], _currentLocation) < 0)  // P right of  edge
                     --wn;            // have  a valid down intersect
        }
    }
    return wn;
}
*/

int gpsMap::insidePerimeter(float x, float y)
{
  int    wn = 0;    // the  winding number counter
  int    wnTempArea = 0;    // the  winding number counter
  Point _currentLocation = {x,y};

  if (_numberOfExclusionAreas > 0) {
    for (int j=0; j < _numberOfExclusionAreas; j++) {
      // loop through all edges of the polygon
      for (int i=0; i<_exclusionAreas[j].numPoints; i++) {   // edge from area[i] to  area[i+1]
          if (_exclusionAreas[j].point[i].y <= _currentLocation.y) {          // start y <= _currentLocation.y
              if (_exclusionAreas[j].point[i+1].y  > _currentLocation.y)      // an upward crossing
                   if (isLeft( _exclusionAreas[j].point[i], _exclusionAreas[j].point[i+1], _currentLocation) > 0)  // P left of  edge
                       ++wn;            // have  a valid up intersect
          }
          else {                        // start y > _currentLocation.y (no test needed)
              if (_exclusionAreas[j].point[i+1].y  <= _currentLocation.y)     // a downward crossing
                   if (isLeft( _exclusionAreas[j].point[i], _exclusionAreas[j].point[i+1], _currentLocation) < 0)  // P right of  edge
                       --wn;            // have  a valid down intersect
          }
      }
      if (wn != 0) return 0; // we are inside of at least one exclusion area, no need to check others 
                             // return 0 = because we are outside of perimeter when we are inside one of the exclusion areas
    }
  }

  if (_numberOfMainAreas > 0) {
    for (int j=0; j < _numberOfMainAreas; j++) {
      // loop through all edges of the polygon
      for (int i=0; i<_mainAreas[j].numPoints; i++) {   // edge from area[i] to  area[i+1]
          if (_mainAreas[j].point[i].y <= _currentLocation.y) {          // start y <= _currentLocation.y
              if (_mainAreas[j].point[i+1].y  > _currentLocation.y)      // an upward crossing
                   if (isLeft( _mainAreas[j].point[i], _mainAreas[j].point[i+1], _currentLocation) > 0)  // P left of  edge
                       ++wn;            // have  a valid up intersect
          }
          else {                        // start y > _currentLocation.y (no test needed)
              if (_mainAreas[j].point[i+1].y  <= _currentLocation.y)     // a downward crossing
                   if (isLeft( _mainAreas[j].point[i], _mainAreas[j].point[i+1], _currentLocation) < 0)  // P right of  edge
                       --wn;            // have  a valid down intersect
          }
      }
      if (wn != 0) break; //return wn; // we are inside of one main area, no need to check others
    }
  }

  //checking if we are inside the temporary area
  if ( _longGrassTempAreaInUse > 0) {
    if (millis() > _tempAreaStartTime + _tempAreaTimeIfNoLongGrassFound) {
      _longGrassTempAreaInUse = 0;
    } else {

      for (int i=0; i<5; i++) {   // edge from area[i] to  area[i+1]
          if (_longGrassTempArea[i].y <= _currentLocation.y) {          // start y <= _currentLocation.y
              if (_longGrassTempArea[i].y  > _currentLocation.y)      // an upward crossing
                   if (isLeft( _longGrassTempArea[i], _longGrassTempArea[i+1], _currentLocation) > 0)  // P left of  edge
                       ++wnTempArea;            // have  a valid up intersect
          }
          else {                        // start y > _currentLocation.y (no test needed)
              if (_longGrassTempArea[i+1].y  <= _currentLocation.y)     // a downward crossing
                   if (isLeft( _longGrassTempArea[i], _longGrassTempArea[i+1], _currentLocation) < 0)  // P right of  edge
                       --wnTempArea;            // have  a valid down intersect
          }
      }
    }
  }

  if (_longGrassTempAreaInUse > 0) {
    if(wn !=0 && wnTempArea !=0) return wn;
    else return 0; 
  }
  else return wn;
}


float gpsMap::distanceToClosestWall(float x, float y)
{
/*    float distanceToClosestWall = 100000;
     Point _currentLocation = {x,y};

    for (int i=0; i<_numberOfMainAreaPoints; i++) {   // edge from V[i] to  V[i+1]
        float distance = abs( ((_mainAreaPointList[i+1].y - _mainAreaPointList[i].y)*_currentLocation.x 
                             - (_mainAreaPointList[i+1].x - _mainAreaPointList[i].x)*_currentLocation.y 
                             +  _mainAreaPointList[i+1].x*_mainAreaPointList[i].y 
                             -  _mainAreaPointList[i+1].y*_mainAreaPointList[i].x )) 
                         / sqrt(pow((_mainAreaPointList[i+1].y-_mainAreaPointList[i].y),2) 
                              + pow((_mainAreaPointList[i+1].x-_mainAreaPointList[i].x),2) );
        Serial.print(i);
        Serial.print(":: ");
        Serial.print(_currentLocation.x);
        Serial.print(",");
        Serial.print(_currentLocation.y);
        Serial.print(" - ");
        Serial.print(_mainAreaPointList[i].x);
        Serial.print(",");
        Serial.print(_mainAreaPointList[i].y);
        Serial.print(" - ");
        Serial.print(_mainAreaPointList[i+1].x);
        Serial.print(",");
        Serial.print(_mainAreaPointList[i+1].y);
        Serial.print(" - ");
        
        distanceToClosestWall = min(distanceToClosestWall, distance);
        Serial.println(distanceToClosestWall);
    }

    return distanceToClosestWall;
*/
}

/*
uint8_t gpsMap::addMainAreaPointDELETEME( float x, float y) {
    if (_numberOfMainAreaPoints >= MAXMAINAREAPOINTS-1) {
        return 1;
    } else {
        _mainAreaPointList[_numberOfMainAreaPoints] = {x,y};
        _mainAreaPointList[_numberOfMainAreaPoints+1] = _mainAreaPointList[0]; // last point of area must be equal to first
        _numberOfMainAreaPoints++;
        loadSaveMapData(false);
    }
    return 0;
}
*/
uint8_t gpsMap::setTemporaryArea( float x, float y) {
    _tempAreaStartTime = millis(); // We will reset the timer everytime we find long grass in the temp area.
    if (_longGrassTempAreaInUse > 0) {  // we are already working on temp area, do not change temp area coordinates
      return 1;  
    } else {
      _longGrassTempArea[0] = {x - _tempAreaSize, y - _tempAreaSize};
      _longGrassTempArea[1] = {x - _tempAreaSize, y + _tempAreaSize};
      _longGrassTempArea[2] = {x + _tempAreaSize, y + _tempAreaSize};
      _longGrassTempArea[3] = {x + _tempAreaSize, y - _tempAreaSize};
      _longGrassTempArea[4] = _longGrassTempArea[0];
      _longGrassTempAreaInUse = 1;
    }
}

int gpsMap::addMainAreaPoint(int areaNumber, float lat, float lon) {
    if (_mainAreas[areaNumber].numPoints >= MAXMAINAREAPOINTS-1) {
        return 1;
    } else {
        _mainAreas[areaNumber].point[_mainAreas[areaNumber].numPoints] = {lat , lon};
        _mainAreas[areaNumber].point[_mainAreas[areaNumber].numPoints + 1] = _mainAreas[areaNumber].point[0]; // last point of area must be equal to first
        _mainAreas[areaNumber].numPoints++;
    }
    loadSaveMapData(false);
    return 0;
}

int gpsMap::addExclusionAreaPoint(int areaNumber, float lat, float lon) {
    if (_exclusionAreas[areaNumber].numPoints >= MAXEXCLUSIONAREAPOINTS-1) {
        return 1;
    } else {
        _exclusionAreas[areaNumber].point[_exclusionAreas[areaNumber].numPoints] = {lat , lon};
        _exclusionAreas[areaNumber].point[_exclusionAreas[areaNumber].numPoints + 1] = _exclusionAreas[areaNumber].point[0]; // last point of area must be equal to first
        _exclusionAreas[areaNumber].numPoints++;
    }
    loadSaveMapData(false);
    return 0;
}

uint8_t gpsMap::removeMainAreaPoint(int pointNro) {
/*
    if (_numberOfMainAreaPoints <= 1) {
        _numberOfMainAreaPoints = 0;
        return 1;
    } else {
        int i=0;
        while (pointNro-1+i < _numberOfMainAreaPoints+1) {
            _mainAreaPointList[pointNro-1+i] = _mainAreaPointList[pointNro+i]; // last point of area must be equal to first
            i++;
        }
        _numberOfMainAreaPoints--;

    }
*/
    return 0;
}

float gpsMap::getMainAreaPointX(int areaNumber, int pointNumber) {
  return _mainAreas[areaNumber].point[pointNumber].x;
}

float gpsMap::getMainAreaPointY(int areaNumber, int pointNumber) {
  return _mainAreas[areaNumber].point[pointNumber].y;
}

int gpsMap::getNumberOfMainAreaPoints(int areaNumber) {
  return _mainAreas[areaNumber].numPoints;
}


float gpsMap::getExclusionAreaPointX(int areaNumber, int pointNumber) {
  return _exclusionAreas[areaNumber].point[pointNumber].x;
}

float gpsMap::getExclusionAreaPointY(int areaNumber, int pointNumber) {
  return _exclusionAreas[areaNumber].point[pointNumber].y;
}

int gpsMap::getNumberOfExclusionAreaPoints(int areaNumber) {
  return _exclusionAreas[areaNumber].numPoints;
}

void gpsMap::deleteMainAreaPoints(int areaNumber) {
  _mainAreas[areaNumber].numPoints = 0; 
  loadSaveMapData(false);
}

void gpsMap::deleteExclusionAreaPoints(int areaNumber) {
  _exclusionAreas[areaNumber].numPoints = 0;
  loadSaveMapData(false);
}

void gpsMap::init() {
  loadSaveMapData(true);
}


void gpsMap::loadSaveMapData(boolean readflag){
  if (readflag) Serial.println(F("loadSavegpsMapData:: read"));
  else Serial.println(F("loadSavegpsMapData: write"));

  int addr = ADDR_GPSMAP_DATA;
  short magic = 0;
  if (!readflag) magic = MAGIC;  
  eereadwrite(readflag, addr, magic); // magic
  if ((readflag) && (magic != MAGIC)) {
    Serial.println(F("EEPROM ERROR DATA: NO gpsMap DATA FOUND"));    
    return;
  }

  eereadwrite(readflag, addr, _numberOfMainAreas);  

  for (int i=0; i <= _numberOfMainAreas; i++) {
    eereadwrite(readflag, addr, _mainAreas[i].numPoints);  
    int j=0;
    if (_mainAreas[i].numPoints > 0) {
      for (j; j <= _mainAreas[i].numPoints; j++) {
        eereadwrite(readflag, addr, _mainAreas[i].point[j].x);      
        eereadwrite(readflag, addr, _mainAreas[i].point[j].y);      
      }
      if (readflag) {
        _mainAreas[i].point[j+1].x = _mainAreas[i].point[0].x;
        _mainAreas[i].point[j+1].y = _mainAreas[i].point[0].y;
      }
    }
  }

  eereadwrite(readflag, addr, _numberOfExclusionAreas);  

  for (int i=0; i <= _numberOfExclusionAreas; i++) {
    eereadwrite(readflag, addr, _exclusionAreas[i].numPoints);  
    int j=0;
    if (_exclusionAreas[i].numPoints > 0) {
      for (j; j <= _exclusionAreas[i].numPoints; j++) {
        eereadwrite(readflag, addr, _exclusionAreas[i].point[j].x);      
        eereadwrite(readflag, addr, _exclusionAreas[i].point[j].y);      
      }
      if (readflag) {
        _exclusionAreas[i].point[j+1].x = _exclusionAreas[i].point[0].x;
        _exclusionAreas[i].point[j+1].y = _exclusionAreas[i].point[0].y;
      }
    }
  }


  Serial.print(F("loadSaveMapData addrstop="));
  Serial.println(addr);
  
}


void gpsMap::doUnitTest() {
  addMainAreaPoint(0,0,0);
  addMainAreaPoint(0,0,100);
  addMainAreaPoint(0,100,100);
  addMainAreaPoint(0,100,0);
  addMainAreaPoint(0,0,0);

  addExclusionAreaPoint(0,20,20);
  addExclusionAreaPoint(0,20,40);
  addExclusionAreaPoint(0,40,40);
  addExclusionAreaPoint(0,40,20);
  addExclusionAreaPoint(0,20,20);

  _longGrassTempAreaInUse = 0;

  doTest(1,50,50,false); // inside of main area / not in exclusion areas

  doTest(2,30,30,true);  // inside of exclusion area

  doTest(3,0,101, true); // outside of main areas

  setTemporaryArea(70, 70);

  doTest(4,71,71, false); // inside of temporary area
  
  doTest(5,88,88, true);  // outside of temporary area
}

void gpsMap::doTest(uint8_t testNum, float lat, float lon, bool expectZero) {
  Serial.println(" ");
  Serial.print("Test ");
  Serial.print(testNum);
  Serial.println(" --------------------------------");
  int temppi = insidePerimeter(lat,lon);

  Serial.print("Test ");
  Serial.print(testNum);
  Serial.print(": ");
  if (expectZero == true) {
    if (temppi == 0 ) {
      Serial.print("- Success - waited 0 got ");
      Serial.println(temppi);
    } else {
      Serial.print("- Failed - waited 0 got ");
      Serial.println(temppi);
    }
  } else {
    if (temppi != 0 ) {
      Serial.print("- Success - waited non zero got ");
      Serial.println(temppi);
    } else {
      Serial.print("- Failed - waited non zero got ");
      Serial.println(temppi);
    }
  }
  Serial.println(" ");
}




/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2019 by Marko Riihimaki
  
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

/*
#include "gpsMap.h"
//#include <Wire.h>
//#include "drivers.h"
//#include "i2c.h"
#include "config.h"
#include "flashmem.h"
//#include "buzzer.h"

//#include <map>






// gpsMapData is used to map the yard which we are cutting
//
// Each point in the map is referenced by GPS latitude and longitude.
// 5th decimal point is used to reach about 1 - 1,1 meters between map points
//
// gpsMapData integer is used to save 6 values for each point in map:
//  1 = is within perimeter
//  2 = is edge node of perimeter
//  4 = connected to adjacent point to north
//  8 = connected to adjacent point to east
// 16 = connected to adjacent point to south
// 32 = connected to adjacent point to west
//std::map<std::pair<float,float>, int> gpsMapData;

struct innerData {
  int32_t longitude;
  uint8_t pointData;
};
 
struct outerData {
  int32_t latitude;
  innerData innerData222[3];
};
 
//outerData gpsMapData[3]; 

//gpsMapData[0].latitude = 0;
//gpsMapData[0].innerData.longitude = 0;

gpsMap::gpsMap() 
{
}

//
// public methods
//
void gpsMap::init(){
  loadSaveMapData(true);
  boolean gpsMapDataChanged = false;
  float lastPointLongitude = 0.0;
  float lastPointLatitude = 0.0;
  unsigned long nextgpsMapSaveTime = 0;
}


void gpsMap::printMap() {
  for(auto it = gpsMapData.begin(); it != gpsMapData.end(); ++it)
  {
    Console.print(it->first.first);
    Console.print(",");
    Console.print(it->first.second);
    Console.print(",");

    if ( bitOfPointIsOn(it->first.first,it->first.second,INSIDE_PERIMETER   ) ) Console.print("I");
    if ( bitOfPointIsOn(it->first.first,it->first.second,PERIMETER_EDGE_NODE) ) Console.print("P");
    if ( bitOfPointIsOn(it->first.first,it->first.second,CONNECTED_NORTH    ) ) Console.print("N");
    if ( bitOfPointIsOn(it->first.first,it->first.second,CONNECTED_EAST     ) ) Console.print("E");
    if ( bitOfPointIsOn(it->first.first,it->first.second,CONNECTED_SOUTH    ) ) Console.print("S");
    if ( bitOfPointIsOn(it->first.first,it->first.second,CONNECTED_WEST     ) ) Console.print("W");

    Console.println("");
  }
}

boolean gpsMap::bitOfPointIsOn(float latitude, float longitude, int bitToCompare) {
//  return (boolean)((gpsMapData[std::make_pair(latitude,longitude)] >> bitToCompare) & 1);
}

void gpsMap::setBitOnOfPoint(float latitude, float longitude, int bitToSet) {
  // If the point is not yet in the map, add it to map
  if (gpsMapData[std::make_pair(latitude,longitude)]) {
    gpsMapData[std::make_pair(latitude,longitude)] = 0;
    gpsMapDataChanged = true;
  }

  // if the bit that we are trying to set is not yet 1, set it to 1
  if ( !bitOfPointIsOn(latitude,longitude,bitToSet) ){
    gpsMapData[std::make_pair(latitude,longitude)] |= 1UL << bitToSet;
    gpsMapDataChanged = true;
  }

  // If the map has changed during the last xxx ms, save it to memory
  if (gpsMapDataChanged && millis() > nextgpsMapSaveTime) {
    nextgpsMapSaveTime = millis() + 10000;
    loadSaveMapData(false);
  }
}

void gpsMap::setPerimeterEdgePoint(float latitude, float longitude){
  setBitOnOfPoint(latitude,longitude,PERIMETER_EDGE_NODE);
}

void gpsMap::checkPoint(float latitude, float longitude){

  setBitOnOfPoint(latitude,longitude,INSIDE_PERIMETER);

  if ( lastPointLongitude - longitude == 0.00001 && lastPointLatitude - latitude == 0 ) {
    setBitOnOfPoint(latitude,longitude,CONNECTED_WEST);
    setBitOnOfPoint(lastPointLatitude,lastPointLongitude,CONNECTED_EAST);
  }
  
  if ( lastPointLongitude - longitude == -0.00001 && lastPointLatitude - latitude == 0 ) {
    setBitOnOfPoint(latitude,longitude,CONNECTED_EAST);
    setBitOnOfPoint(lastPointLatitude,lastPointLongitude,CONNECTED_WEST);
  }
  
  if ( lastPointLongitude - longitude == 0 && lastPointLatitude - latitude == 0.00001 ) {
    setBitOnOfPoint(latitude,longitude,CONNECTED_NORTH);
    setBitOnOfPoint(lastPointLatitude,lastPointLongitude,CONNECTED_SOUTH);
  }

  if ( lastPointLongitude - longitude == 0 && lastPointLatitude - latitude == -0.00001 ) {
    setBitOnOfPoint(latitude,longitude,CONNECTED_SOUTH);
    setBitOnOfPoint(lastPointLatitude,lastPointLongitude,CONNECTED_NORTH);
  }

}

void gpsMap::loadSaveMapData(boolean readflag){
  if (readflag) Console.println(F("loadSavegpsMapData:: read"));
    else Console.println(F("loadSavegpsMapData: write"));
  int addr = ADDR_gpsMap_DATA;
  short magic = 0;
  if (!readflag) magic = MAGIC;  
  eereadwrite(readflag, addr, magic); // magic
  if ((readflag) && (magic != MAGIC)) {
    Console.println(F("EEPROM ERROR DATA: NO gpsMap DATA FOUND"));    
    return;
  }

  eereadwrite(readflag, addr, chargingLatitude);  
  eereadwrite(readflag, addr, chargingLongitude);  
  eereadwrite(readflag, addr, gpsMapData);  
  Console.print(F("loadSaveMapData addrstop="));
  Console.println(addr);
  
}

*/