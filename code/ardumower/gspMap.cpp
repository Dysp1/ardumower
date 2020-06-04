
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

int gpsMap::isLeft( Point P0, Point P1, Point P2 )
{
    return ( (P1.x - P0.x) * (P2.y - P0.y)
            - (P2.x -  P0.x) * (P1.y - P0.y) );
}
//===================================================================

// wn_PnPoly(): winding number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  wn = the winding number (=0 only when P is outside)
int gpsMap::wn_PnPoly(float x, float y)
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

float gpsMap::distanceToClosestWall(float x, float y)
{
    float distanceToClosestWall = 100000;
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
}

uint8_t gpsMap::addMainAreaPoint( float x, float y) {
    if (_numberOfMainAreaPoints >= MAXMAINAREAPOINTS-1) {
        return 1;
    } else {
        _mainAreaPointList[_numberOfMainAreaPoints] = {x,y};
        _mainAreaPointList[_numberOfMainAreaPoints+1] = _mainAreaPointList[0]; // last point of area must be equal to first
        _numberOfMainAreaPoints++;
    }
    return 0;
}

uint8_t gpsMap::removeMainAreaPoint(int pointNro) {
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
    return 0;
}

float gpsMap::getMainAreaPointX(int pointNro) {
  return _mainAreaPointList[pointNro].x;
}

float gpsMap::getMainAreaPointY(int pointNro) {
  return _mainAreaPointList[pointNro].y;
}

uint8_t gpsMap::getNumberOfMainAreaPoints() {
  return _numberOfMainAreaPoints;
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

  int i=0;
  while (i < MAXMAINAREAPOINTS) {
    eereadwrite(readflag, addr, _mainAreaPointList[i].x);  
    eereadwrite(readflag, addr, _mainAreaPointList[i].y);  
    i++;
  }

  Serial.print(F("loadSaveMapData addrstop="));
  Serial.println(addr);
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