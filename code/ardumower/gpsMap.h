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

#define MAXPOINTS 51 // 50 + 1 extra. last point must be = first point

#define MAXMAINAREAPOINTS 51 // 30 + 1 extra. last point must be = first point
#define MAXEXCLUSIONAREAPOINTS 11 // 10 + 1 extra. last point must be = first point
#define MAXHOMINGPOINTS 30

#define MAGIC 1
#define ADDR_GPSMAP_DATA 2049

typedef struct {long x, y;} Point;

typedef struct  {
                  int numPoints = 0; 
                  Point point[MAXMAINAREAPOINTS] = {};
                } pointList;

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
        //mainArea _mainAreas[MAXMAINAREAS];
        //exclusionArea _exclusionAreas[MAXEXCLUSIONAREAS];
        //homingPointList _homingPointList[2];

        pointList _mainAreas[MAXMAINAREAS];
        pointList _exclusionAreas[MAXEXCLUSIONAREAS];
        pointList _homingPointList[2];

        float _numberOfMainAreas = 1; // we must have at least one main area
        float _numberOfExclusionAreas = 2;
        float _numberOfHomingPointLists = 1;
//        Point _mainAreaPointList[MAXMAINAREAPOINTS] = {};
//        uint8_t _numberOfMainAreaPoints = 0;
        
        Point _longGrassTempArea[5] = {};
        Point _longGrassTempAreaMiddlePoint;
        uint8_t _longGrassTempAreaInUse = 0;
        float _tempAreaStartTime;
        float _tempAreaSize = 10;
        float _wiredPerimeterInUse = 0;
        float _tempAreaTimeIfNoLongGrassFound = 300000;

        bool _unitTesting = true; //false;

        inline long isLeft( Point P0, Point P1, Point P2 );
        void doTest(uint8_t testNum, long lat, long lon, bool expectZero);
        void loadSaveMapData(boolean readflag);
        pointList getRightArray(String areaType, int areaNumber);
        void putBackToRightArray(String areaType, int areaNumber, pointList arrayToModify);

    public:
        void doUnitTest(); 
        void init(float size, float wiredInUse);

        int addPoint(String areaType, int areaNumber, long lat, long lon);
        void addPointInMiddle(String areaType, int areaNumber, int positionToAdd, long lat, long lon);
        void deleteAllPoints(String areaType, int areaNumber);
        void deletePointFromMiddle(String areaType, int areaNumber, int positionToDelete);

        int getLongGrassTempAreaInUse();
        int insideLongGrassTempArea(long x, long y);
        void setLongGrassTempAreaSize(float size);
        float distanceFromTempAreaMiddle(long lat, long lon);
        float getNewHeadingLongGrassAreaDegrees( long lat, long lon);
        void wiredPerimeterInUse(float inUse);

        int addMainAreaPoint(int areaNum, long lat, long lon);
        int getNumberOfMainAreaPoints(int areaNumber);
        int getMaxNumberOfMainAreaPoints();
        long getMainAreaPointX(int areaNumber, int pointNumber);
        long getMainAreaPointY(int areaNumber, int pointNumber);
        void deleteMainAreaPoints(int areaNumber);
        void addMainAreaPointInMiddle(int areaNumber, int positionToAdd, long lat, long lon);
        void deleteMainAreaPointFromMiddle(int areaNumber, int positionToAdd, long lat, long lon);

        int addExclusionAreaPoint(int areaNum, long lat, long lon);
        int getNumberOfExclusionAreaPoints(int areaNumber);
        int getMaxNumberOfExclusionAreaPoints();
        long getExclusionAreaPointX(int areaNumber, int pointNumber);
        long getExclusionAreaPointY(int areaNumber, int pointNumber);
        void deleteExclusionAreaPoints(int areaNumber);
        void deleteExclusionAreaPointFromMiddle(int areaNumber, int positionToAdd, long lat, long lon);


        int addHomingPoint(int areaNum, long lat, long lon);
        int getNumberOfHomingPoints (int areaNumber);
        int getMaxNumberOfHomingPoints();
        long getHomingPointX (int areaNumber, int pointNumber);
        long getHomingPointY (int areaNumber, int pointNumber);
        void deleteHomingPoints(int areaNumber);
        float getNewHeadingFromPerimeterDegrees( long lat, long lon);


        int insidePerimeter(long x, long y);
        float distanceToClosestWall(long x, long y);
        uint8_t removeMainAreaPoint( int pointNro);
        uint8_t setTemporaryArea( long x, long y);
        void disableTemporaryArea();
        //int removeExclusionAreaPoint( int pointNro);
};

#endif

