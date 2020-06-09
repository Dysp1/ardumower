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
  
  You are free to use the code in this classs for commercial or personal use at your will.
*/



#ifndef gpsMap_h
#define gpsMap_h

#include "Arduino.h"

#define MAXMAINAREAPOINTS 31
#define MAGIC 1
#define ADDR_GPSMAP_DATA 900

typedef struct {int x, y;} Point;

class gpsMap
{
    private:
        int isLeft( Point P0, Point P1, Point P2 );
        Point _mainAreaPointList[MAXMAINAREAPOINTS] = {};
        uint8_t _numberOfMainAreaPoints = 0;
        void loadSaveMapData(boolean readflag);
    public:
        int insidePerimeter(float x, float y);
        float distanceToClosestWall(float x, float y);
        uint8_t addMainAreaPoint( float x, float y);
        uint8_t removeMainAreaPoint( int pointNro);
        float getMainAreaPointX(int pointNro);
        float getMainAreaPointY(int pointNro);
        uint8_t getNumberOfMainAreaPoints();

};

#endif


/*

#ifndef gpsMap_H
#define gpsMap_H

#include "Arduino.h"

enum {INSIDE_PERIMETER, PERIMETER_EDGE_NODE, CONNECTED_NORTH, CONNECTED_EAST, CONNECTED_SOUTH, CONNECTED_WEST};

class gpsMap
{
public:
  gpsMap();
  void init();
  void checkPoint(float longitude, float latitude) ;
  void setPerimeterEdgePoint(float longitude, float latitude) ;
  void printMap() ; 

private:
  void loadSaveMapData(boolean) ; 
  float chargingLatitude ;
  float chargingLongitude ;
  float lastPointLatitude ;
  float lastPointLongitude ;
  int angleToChargingStation ;
  unsigned long nextgpsMapSaveTime ;
  boolean gpsMapDataChanged ; 
  void setBitOnOfPoint(float longitude, float latitude, int bitToSet) ;
  boolean bitOfPointIsOn(float longitude, float latitude, int bitToCompare) ;
};


#endif
*/