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

#define MAGIC 1
#define ADDR_GPSMAP_DATA 2049

typedef struct {long x, y;} Point;

typedef struct  {
                  int numPoints = 0; 
                  Point point[MAXPOINTS] = {};
                } pointList;

class gpsMap
{
    private:

        // TURN THIS true IF you wish to test without saving all the gps perimeter data to eeprom
        bool _unitTesting = false; 


        pointList _mainAreas[MAXMAINAREAS];
        pointList _exclusionAreas[MAXEXCLUSIONAREAS];
        pointList _safePointList[2];
        pointList _homingPointList[2];

        float _numberOfMainAreas = 1; // we must have at least one main area
        float _numberOfExclusionAreas = 2;
        float _numberOfSafePointLists = 1;
        float _numberOfHomingPointLists = 1;
        
        Point _longGrassTempArea[5] = {};
        Point _longGrassTempAreaMiddlePoint;
        uint8_t _longGrassTempAreaInUse = 0;
        float _tempAreaStartTime;
        float _tempAreaSize = 10;
        float _wiredPerimeterInUse = 0;
        float _tempAreaTimeIfNoLongGrassFound = 300000;
        int currentHomingPoint = 999;

        inline long isLeft( Point P0, Point P1, Point P2 );
        void doTest(uint8_t testNum, long lat, long lon, bool expectZero);
        void loadSaveMapData(boolean readflag);
        pointList getRightArray(String areaType, int areaNumber);
        void putBackToRightArray(String areaType, int areaNumber, pointList arrayToModify);
        float getNewHeadingToClosestPoint(String areaType, int areaNumber, long lat, long lon);
        // roll types
        enum { LEFT, RIGHT };

    public:
        void doUnitTest(); 
        void init(float size, float wiredInUse);

        int  addPoint(String areaType, int areaNumber, long lat, long lon);
        void addPointInMiddle(String areaType, int areaNumber, int positionToAdd, long lat, long lon);
        void deleteAllPoints(String areaType, int areaNumber);
        void deletePointFromMiddle(String areaType, int areaNumber, int positionToDelete);
        int  getNumberOfPoints(String areaType,int areaNumber);
        int  getMaxNumberOfPoints(String areaType, int areaNumber);
        long getPointX(String areaType, int areaNumber, int pointNumber);
        long getPointY(String areaType, int areaNumber, int pointNumber);

        float getHeadingToClosestHomingPointDegrees(long lat, long lon);
        float getHeadingToNextHomingPointDegrees(long lat, long lon);
        int lastPointBeforeStation();
        float getDistanceBetweenPoints (long lat1, long lon1, long lat2, long lon2);
        float getHeadingBetweenPointsDegrees (long lat1, long lon1, long lat2, long lon2);

        bool getShortestWayToTurnDegrees( float currentHeading, float gpsPerimeterRollNewHeading);
        float getNewHeadingFromPerimeterDegrees( long lat, long lon);
        int insidePerimeter(long x, long y);
        float distanceToClosestWall(long x, long y);

        uint8_t setTemporaryArea( long x, long y);
        void disableTemporaryArea();
        int getLongGrassTempAreaInUse();
        int insideLongGrassTempArea(long x, long y);
        void setLongGrassTempAreaSize(float size);
        float distanceFromTempAreaMiddle(long lat, long lon);
        float getNewHeadingLongGrassAreaDegrees( long lat, long lon);
        void wiredPerimeterInUse(float inUse);
};

#endif

