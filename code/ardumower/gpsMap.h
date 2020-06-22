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
#define MAXHOMINGPOINTS 30

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

typedef struct  {
                  int numPoints = 0; 
                  Point point[MAXHOMINGPOINTS] = {};
                } homingPointList;

class gpsMap
{
    private:
        mainArea _mainAreas[MAXMAINAREAS];
        exclusionArea _exclusionAreas[MAXEXCLUSIONAREAS];
        homingPointList _homingPointList[1];

        float _numberOfMainAreas = 1; // we must have at least one main area
        float _numberOfExclusionAreas = 2;
        float _numberOfHomingPointLists = 1;
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
        int insideLongGrassTempArea(float x, float y);
        void setLongGrassTempAreaSize(float size);
        float distanceFromTempAreaMiddle(float lat, float lon);
        float getNewHeadingLongGrassAreaDegrees( float lat, float lon);
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

        int addHomingPoint(int areaNum, float lat, float lon);
        int getNumberOfHomingPoints (int areaNumber);
        float getHomingPointX (int areaNumber, int pointNumber);
        float getHomingPointY (int areaNumber, int pointNumber);
        void deleteHomingPoints(int areaNumber);
        float getNewHeadingFromPerimeterDegrees( float lat, float lon);

        int insidePerimeter(float x, float y);
        float distanceToClosestWall(float x, float y);
        uint8_t removeMainAreaPoint( int pointNro);
        uint8_t setTemporaryArea( float x, float y);
        void disableTemporaryArea();
        //int removeExclusionAreaPoint( int pointNro);
};

#endif

