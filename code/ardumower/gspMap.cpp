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

#include "gpsMap.h"
//#include <Wire.h>
//#include "drivers.h"
//#include "i2c.h"
#include "config.h"
#include "flashmem.h"
//#include "buzzer.h"

//#include <map>



#define MAGIC 1
#define ADDR_GPSMAP_DATA 900



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

GPSMAP::GPSMAP() 
{
}

//
// public methods
//
void GPSMAP::init(){
  loadSaveMapData(true);
  boolean gpsMapDataChanged = false;
  float lastPointLongitude = 0.0;
  float lastPointLatitude = 0.0;
  unsigned long nextGpsMapSaveTime = 0;
}


void GPSMAP::printMap() {
/*  for(auto it = gpsMapData.begin(); it != gpsMapData.end(); ++it)
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
  }*/
}

boolean GPSMAP::bitOfPointIsOn(float latitude, float longitude, int bitToCompare) {
//  return (boolean)((gpsMapData[std::make_pair(latitude,longitude)] >> bitToCompare) & 1);
}

void GPSMAP::setBitOnOfPoint(float latitude, float longitude, int bitToSet) {
/*
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
  if (gpsMapDataChanged && millis() > nextGpsMapSaveTime) {
    nextGpsMapSaveTime = millis() + 10000;
    loadSaveMapData(false);
  }
}

void GPSMAP::setPerimeterEdgePoint(float latitude, float longitude){
  setBitOnOfPoint(latitude,longitude,PERIMETER_EDGE_NODE);
}

void GPSMAP::checkPoint(float latitude, float longitude){

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
*/
}

void GPSMAP::loadSaveMapData(boolean readflag){
/*  if (readflag) Console.println(F("loadSaveGPSMapData:: read"));
    else Console.println(F("loadSaveGPSMapData: write"));
  int addr = ADDR_GPSMAP_DATA;
  short magic = 0;
  if (!readflag) magic = MAGIC;  
  eereadwrite(readflag, addr, magic); // magic
  if ((readflag) && (magic != MAGIC)) {
    Console.println(F("EEPROM ERROR DATA: NO GPSMAP DATA FOUND"));    
    return;
  }

  eereadwrite(readflag, addr, chargingLatitude);  
  eereadwrite(readflag, addr, chargingLongitude);  
  eereadwrite(readflag, addr, gpsMapData);  
  Console.print(F("loadSaveMapData addrstop="));
  Console.println(addr);
  */
}

