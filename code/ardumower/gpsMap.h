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

#define MAXMAINAREAS 4
#define MAXEXCLUSIONAREAS 10

#define MAXMAINAREAPOINTS 31 // 30 + 1 extra. last point must be = first point
#define MAXEXCLUSIONAREAPOINTS 11 // 10 + 1 extra. last point must be = first point

#define MAGIC 1
#define ADDR_GPSMAP_DATA 2049

typedef struct {float x, y;} Point;

typedef struct  {
                  int numPoints = 0; 
                  Point point[MAXMAINAREAPOINTS] = {};
                } mainArea;

typedef struct  {
                  int numPoints = 0; 
                  Point point[MAXEXCLUSIONAREAPOINTS] = {};
                } exclusionArea;

class gpsMap
{
    private:
        mainArea _mainAreas[MAXMAINAREAS];
        exclusionArea _exclusionAreas[MAXEXCLUSIONAREAS];

        int _numberOfMainAreas = 1; // we must have at least one main area
        int _numberOfExclusionAreas = 2;

//        Point _mainAreaPointList[MAXMAINAREAPOINTS] = {};
//        uint8_t _numberOfMainAreaPoints = 0;
        

        Point _longGrassTempArea[5] = {};
        Point _longGrassTempAreaMiddlePoint;
        uint8_t _longGrassTempAreaInUse = 0;
        float _tempAreaStartTime;
        float _tempAreaSize = 0.0001;
        float _wiredPerimeterInUse = 0;
        float _tempAreaTimeIfNoLongGrassFound = 300000;

        bool _unitTesting = false;

        float isLeft( Point P0, Point P1, Point P2 );
        void doTest(uint8_t testNum, float lat, float lon, bool expectZero);
        void loadSaveMapData(boolean readflag);

    public:
        void doUnitTest(); 
        void init(float size, float wiredInUse);

        int getLongGrassTempAreaInUse();
        void setLongGrassTempAreaSize(float size);
        float getNewHeadingFromPerimeterDegrees( float lat, float lon);
        void wiredPerimeterInUse(float inUse);

        int addMainAreaPoint(int areaNum, float lat, float lon);
        int getNumberOfMainAreaPoints(int areaNumber);
        float getMainAreaPointX(int areaNumber, int pointNumber);
        float getMainAreaPointY(int areaNumber, int pointNumber);
        void deleteMainAreaPoints(int areaNumber);

        int addExclusionAreaPoint(int areaNum, float lat, float lon);
        int getNumberOfExclusionAreaPoints(int areaNumber);
        float getExclusionAreaPointX(int areaNumber, int pointNumber);
        float getExclusionAreaPointY(int areaNumber, int pointNumber);
        void deleteExclusionAreaPoints(int areaNumber);

        int insidePerimeter(float x, float y);
        float distanceToClosestWall(float x, float y);
        uint8_t removeMainAreaPoint( int pointNro);
        uint8_t setTemporaryArea( float x, float y);
        void disableTemporaryArea();
        //int removeExclusionAreaPoint( int pointNro);
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