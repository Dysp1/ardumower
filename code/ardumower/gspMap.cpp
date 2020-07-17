
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

inline long gpsMap::isLeft( Point P0, Point P1, Point P2 )
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

int gpsMap::insidePerimeter(long x, long y)
{
  int    wn = 0;    // the  winding number counter
  Point _currentLocation = {x,y};

  if (_numberOfExclusionAreas > 0) {
    for (int j=0; j < _numberOfExclusionAreas; j++) {

      if (_exclusionAreas[j].numPoints < 3 ) break; //less than 3 points can't make an area.

      // loop through all edges of the polygon
      for (int i=0; i < _exclusionAreas[j].numPoints; i++) {   // edge from area[i] to  area[i+1]
          if (_exclusionAreas[j].point[i].y <= _currentLocation.y) {          // start y <= _currentLocation.y
              if (_exclusionAreas[j].point[i+1].y  > _currentLocation.y)      // an upward crossing
                   if (isLeft( _exclusionAreas[j].point[i], _exclusionAreas[j].point[i+1], _currentLocation) > 0) { // P left of  edge
                      ++wn;            // have  a valid up intersect
//                      Serial.println("EA++");
                    }
          }
          else {                        // start y > _currentLocation.y (no test needed)
              if (_exclusionAreas[j].point[i+1].y  <= _currentLocation.y)     // a downward crossing
                   if (isLeft( _exclusionAreas[j].point[i], _exclusionAreas[j].point[i+1], _currentLocation) < 0) { // P right of  edge
                      --wn;            // have  a valid down intersect
//                      Serial.println("EA--");
                    }

          }
      }
      if (wn != 0) return 0; // we are inside of at least one exclusion area, no need to check others 
                             // return 0 = because we are outside of perimeter when we are inside one of the exclusion areas
    }
  }

  wn = 0;
  if (_numberOfMainAreas > 0) {
    for (int j=0; j < _numberOfMainAreas; j++) {

      if (_mainAreas[j].numPoints < 3 ) break; //less than 3 points can't make an area.

      // loop through all edges of the polygon
      for (int i=0; i < _mainAreas[j].numPoints; i++) {   // edge from area[i] to  area[i+1]

/*Serial.print("HERE2: i: ");
Serial.print(_mainAreas[j].point[i].x);
delay(2);
Serial.print(" , ");
Serial.print(_mainAreas[j].point[i].y);
delay(2);
Serial.print(" - i+1: ");
Serial.print(_mainAreas[j].point[i+1].x);
delay(2);
Serial.print(" , ");
Serial.print(_mainAreas[j].point[i+1].y);
delay(2);
Serial.print(" - curr:");
Serial.print(_currentLocation.x);
delay(2);
Serial.print(" , ");
Serial.println(_currentLocation.y);
delay(2);*/
        if (_mainAreas[j].point[i].y <= _currentLocation.y) {          // start y <= _currentLocation.y
          if (_mainAreas[j].point[i+1].y  > _currentLocation.y) {      // an upward crossing
            if (isLeft( _mainAreas[j].point[i], _mainAreas[j].point[i+1], _currentLocation) > 0) { // P left of  edge
              ++wn;            // have  a valid up intersect
  //           Serial.println("ma++");
            }
          }
        }
        else {                        // start y > _currentLocation.y (no test needed)
          if (_mainAreas[j].point[i+1].y  <= _currentLocation.y) {    // a downward crossing
            if (isLeft( _mainAreas[j].point[i], _mainAreas[j].point[i+1], _currentLocation) < 0) { // P right of  edge
              --wn;            // have  a valid down intersect
  //            Serial.println("ma--");
            }
          }
        }
      }
//      Serial.print("WN:");
//      Serial.println(wn);

      if (wn != 0) return wn; // we are inside of one main area, no need to check others
    }
  }

  if (wn == 0 && _wiredPerimeterInUse) return 11;

  return wn;
}

// 0  0 1
// 1  1 2
// 2  2 3
// 3  3 4
// 4  4 5

int gpsMap::insideLongGrassTempArea(long x, long y)
{
  Point _currentLocation = {x,y};
  int    wnTempArea = 0;    // the  winding number counter
  //checking if we are inside the temporary area
  if ( _longGrassTempAreaInUse > 0) {
    if (millis() > _tempAreaStartTime + _tempAreaTimeIfNoLongGrassFound) {
      _longGrassTempAreaInUse = 0;
    } else {

      for (int i=0; i <= 4; i++) {   
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
    return wnTempArea;
  }
}

float gpsMap::distanceToClosestWall(long x, long y)
{
  float distanceToClosestWall = 100000;
  Point _currentLocation = {x,y};
 
  pointList arrayToModify = getRightArray("MA", 0);

  for (int i=0; i < arrayToModify.numPoints; i++) {   // edge from V[i] to  V[i+1]
      float distance = abs( ((arrayToModify.point[i+1].y - arrayToModify.point[i].y)*_currentLocation.x 
                           - (arrayToModify.point[i+1].x - arrayToModify.point[i].x)*_currentLocation.y 
                           +  arrayToModify.point[i+1].x*arrayToModify.point[i].y 
                           -  arrayToModify.point[i+1].y*arrayToModify.point[i].x )) 
                       / sqrt(pow((arrayToModify.point[i+1].y-arrayToModify.point[i].y),2) 
                            + pow((arrayToModify.point[i+1].x-arrayToModify.point[i].x),2) );
    Serial.print(i);
    Serial.print(":: ");
    Serial.print(_currentLocation.x);
    Serial.print(",");
    Serial.print(_currentLocation.y);
    Serial.print(" - ");
    Serial.print(arrayToModify.point[i].x);
    Serial.print(",");
    Serial.print(arrayToModify.point[i].y);
    Serial.print(" - ");
    Serial.print(arrayToModify.point[i+1].x);
    Serial.print(",");
    Serial.print(arrayToModify.point[i+1].y);
    Serial.print(" - ");
      
    distanceToClosestWall = min(distanceToClosestWall, distance);
    Serial.println(distanceToClosestWall);
  }
  return distanceToClosestWall;

}

float gpsMap::distanceFromTempAreaMiddle(long lat, long lon)
{
  return sqrt( pow((lat - _longGrassTempAreaMiddlePoint.x),2) + pow((lon - _longGrassTempAreaMiddlePoint.y),2) );
}


float gpsMap::getNewHeadingLongGrassAreaDegrees( long lat, long lon) {
  if (_longGrassTempAreaInUse > 0) {
    float degrees = atan2( (_longGrassTempAreaMiddlePoint.y - lon ), (_longGrassTempAreaMiddlePoint.x - lat) )/PI*180.0;
    if (degrees < 0) degrees += 360; 
    return degrees;
  }
}

float gpsMap::getDegreesToTurn( float currentHeading, float gpsPerimeterRollNewHeading) {
  float first = abs(gpsPerimeterRollNewHeading - currentHeading);
  float second = abs(gpsPerimeterRollNewHeading - currentHeading + 360);
  float third = abs(gpsPerimeterRollNewHeading - currentHeading - 360);

  if (first < second && first < third && (gpsPerimeterRollNewHeading - currentHeading) > 0) return first;
    else if (second < first && second < third && (gpsPerimeterRollNewHeading - currentHeading + 360) > 0) return second;
      else if (third < first && third < second && (gpsPerimeterRollNewHeading- currentHeading - 360) > 0) return third;
}

bool gpsMap::getShortestWayToTurnDegrees( float currentHeading, float gpsPerimeterRollNewHeading) {

  bool rollDir;
  float first = abs(gpsPerimeterRollNewHeading - currentHeading);
  float second = abs(gpsPerimeterRollNewHeading - currentHeading + 360);
  float third = abs(gpsPerimeterRollNewHeading - currentHeading - 360);

  rollDir = RIGHT;
  if (first < second && first < third && (gpsPerimeterRollNewHeading - currentHeading) > 0) rollDir = LEFT;
    else if (second < first && second < third && (gpsPerimeterRollNewHeading - currentHeading + 360) > 0) rollDir = LEFT;
      else if (third < first && third < second && (gpsPerimeterRollNewHeading- currentHeading - 360) > 0) rollDir = LEFT;

  return rollDir;
}

float gpsMap::getNewHeadingToClosestPoint(String areaType, int areaNumber, long lat, long lon) {

  pointList arrayToModify = getRightArray(areaType, areaNumber);

  int closestPoint = 0;
  float lastShortestDistance = 88888.0;
  float shortestDistance = 99999.0;
  int i = 0;
  for (i=0; i < arrayToModify.numPoints; i++) {
    shortestDistance = min(shortestDistance, (sqrt( pow((lat - arrayToModify.point[i].x),2) + pow((lon - arrayToModify.point[i].y),2) )));

    if (shortestDistance < lastShortestDistance) {
      lastShortestDistance = shortestDistance;
      closestPoint = i;
      if (areaType == "HP") currentHomingPoint = i;
    }

  }

  float degrees = atan2( (arrayToModify.point[closestPoint].y - lon ), (arrayToModify.point[closestPoint].x - lat) )/PI*180.0;
  if (degrees < 0) degrees += 360; 

  return degrees;
}

float gpsMap::getHeadingToNextHomingPointDegrees(long lat, long lon) {
  float distanceToCurrent = getDistanceBetweenPoints(lat, lon, _homingPointList[0].point[currentHomingPoint].x, _homingPointList[0].point[currentHomingPoint].y);
  float distanceToNext =  getDistanceBetweenPoints(lat, lon, _homingPointList[0].point[currentHomingPoint-1].x, _homingPointList[0].point[currentHomingPoint-1].y);
  float distanceCurrentToNext =  getDistanceBetweenPoints(_homingPointList[0].point[currentHomingPoint].x, _homingPointList[0].point[currentHomingPoint].y,_homingPointList[0].point[currentHomingPoint-1].x, _homingPointList[0].point[currentHomingPoint-1].y);

  Serial.print("distanceToCurrent:");
  Serial.println(distanceToCurrent);
  Serial.print("distanceToNext:");
  Serial.println(distanceToNext);
  Serial.print("distanceCurrentToNext:");
  Serial.println(distanceCurrentToNext);
  Serial.print("currentHomingPoint:");
  Serial.println(currentHomingPoint);


  if ((distanceToCurrent <= 8 || distanceToNext < distanceCurrentToNext) && currentHomingPoint > 0) {
    float nextPointHeading = getHeadingBetweenPointsDegrees(lat, lon, _homingPointList[0].point[currentHomingPoint-1].x, _homingPointList[0].point[currentHomingPoint-1].y);
    currentHomingPoint--;
  Serial.print("nextPointHeading, new point: ");
  Serial.println(round(nextPointHeading));

    return round(nextPointHeading);
  } else {
    float currentPointHeading = getHeadingBetweenPointsDegrees(lat, lon, _homingPointList[0].point[currentHomingPoint].x, _homingPointList[0].point[currentHomingPoint].y);
  Serial.print("nextPointHeading, no change: ");
  Serial.println(round(currentPointHeading));

    return round(currentPointHeading);
  }

}

int gpsMap::lastPointBeforeStation() {
  if (currentHomingPoint == 0) return 1;
  else return 0;
}

float gpsMap::getDistanceBetweenPoints (long lat1, long lon1, long lat2, long lon2) {
  return (sqrt( pow((lat1 - lat2),2) + pow((lon1 - lon2),2) ));
}

float gpsMap::getHeadingBetweenPointsDegrees (long lat1, long lon1, long lat2, long lon2) {
  float degrees = atan2( (lon1 - lon2 ), (lat1 - lat2) )/PI*180.0;
  if (degrees < 0) degrees += 360;
  return round(degrees); 
}

float gpsMap::getHeadingToClosestHomingPointDegrees(long lat, long lon) {
  return getNewHeadingToClosestPoint("HP", 0, lat, lon);
}

float gpsMap::getNewHeadingFromPerimeterDegrees(long lat, long lon) {
/*
  int closestPoint = 0;
  float lastShortestDistance = 88888.0;
  float shortestDistance = 99999.0;
  int i = 0;
  for (i=0; i < _safePointList[0].numPoints; i++) {
    shortestDistance = min(shortestDistance, (sqrt( pow((lat - _safePointList[0].point[i].x),2) + pow((lon - _safePointList[0].point[i].y),2) )));

    if (shortestDistance < lastShortestDistance) {
      lastShortestDistance = shortestDistance;
      closestPoint = i;
    }

  }

  float degrees = atan2( (_safePointList[0].point[closestPoint].y - lon ), (_safePointList[0].point[closestPoint].x - lat) )/PI*180.0;
  if (degrees < 0) degrees += 360; 
*/
  
  return getNewHeadingToClosestPoint("SP", 0, lat, lon);
}

uint8_t gpsMap::setTemporaryArea( long x, long y) {
    _tempAreaStartTime = millis(); // We will reset the timer everytime we find long grass in the temp area.
    if (_longGrassTempAreaInUse > 0) {  // we are already working on temp area, do not change temp area coordinates
      return 1;  
    } else {
      _longGrassTempAreaMiddlePoint = {x,y};
      _longGrassTempArea[0] = {x - _tempAreaSize, y - _tempAreaSize};
      _longGrassTempArea[1] = {x - _tempAreaSize, y + _tempAreaSize};
      _longGrassTempArea[2] = {x + _tempAreaSize, y + _tempAreaSize};
      _longGrassTempArea[3] = {x + _tempAreaSize, y - _tempAreaSize};
      _longGrassTempArea[4] = _longGrassTempArea[0];
      _longGrassTempAreaInUse = 1;
    /*
      Serial.print(_longGrassTempArea[0].x,6);
      Serial.print(",");
      Serial.println(_longGrassTempArea[0].y,6);
      Serial.print(_longGrassTempArea[1].x,6);
      Serial.print(",");
      Serial.println(_longGrassTempArea[1].y,6);
      Serial.print(_longGrassTempArea[2].x,6);
      Serial.print(",");
      Serial.println(_longGrassTempArea[2].y,6);
      Serial.print(_longGrassTempArea[3].x,6);
      Serial.print(",");
      Serial.println(_longGrassTempArea[3].y,6);
      Serial.print(_longGrassTempArea[4].x,6);
      Serial.print(",");
      Serial.println(_longGrassTempArea[4].y,6);
    */
    }
}

void gpsMap::wiredPerimeterInUse(float inUse) {
  _wiredPerimeterInUse = inUse;
}

void gpsMap::disableTemporaryArea() {
  _longGrassTempAreaInUse = 0;
}

pointList gpsMap::getRightArray(String areaType, int areaNumber) {
  if (areaType == "MA") return _mainAreas[areaNumber];
    else if (areaType == "EA") return _exclusionAreas[areaNumber];
      else if (areaType == "SP") return _safePointList[areaNumber];
        else if (areaType == "HP") return _homingPointList[areaNumber];
}

void gpsMap::putBackToRightArray(String areaType, int areaNumber, pointList arrayToModify) {
  if (areaType == "MA") _mainAreas[areaNumber] = arrayToModify;
    else if (areaType == "EA") _exclusionAreas[areaNumber] = arrayToModify;
      else if (areaType == "SP") _safePointList[areaNumber] = arrayToModify;
        else if (areaType == "HP") _homingPointList[areaNumber] = arrayToModify;
}

void gpsMap::addPointInMiddle(String areaType, int areaNumber, int positionToAdd, long lat, long lon) {

    pointList arrayToModify = getRightArray(areaType, areaNumber);

    if (arrayToModify.numPoints > 0 && arrayToModify.numPoints < MAXPOINTS-1) {
      int i = 0;
      for (i = arrayToModify.numPoints; i > positionToAdd; i--) {
        arrayToModify.point[i+1] = arrayToModify.point[i];
      }
//      arrayToModify.point[positionToAdd+1].x = random(1,200); //lat;
//      arrayToModify.point[positionToAdd+1].y = random(1,200); //lon;
      arrayToModify.point[positionToAdd+1].x = lat;
      arrayToModify.point[positionToAdd+1].y = lon;
      arrayToModify.numPoints++;
    }

    if (positionToAdd == 0 && (areaType == "MA" || areaType == "EA")) arrayToModify.point[arrayToModify.numPoints] = arrayToModify.point[0];  // last point of exclusion and main areas must be equal to the first point

    putBackToRightArray(areaType, areaNumber, arrayToModify);

    if (!_unitTesting) loadSaveMapData(false);
}


int gpsMap::addPoint(String areaType, int areaNumber, long lat, long lon) {
    
    pointList arrayToModify = getRightArray(areaType, areaNumber);

    if (arrayToModify.numPoints >= MAXPOINTS-1) {
        return 0;
    } else {
        arrayToModify.point[arrayToModify.numPoints] = {lat , lon};
        if (areaType == "MA" || areaType == "EA") arrayToModify.point[arrayToModify.numPoints + 1] = arrayToModify.point[0];  // last point of exclusion and main areas must be equal to the first point
        arrayToModify.numPoints++;
    }

    putBackToRightArray(areaType, areaNumber, arrayToModify);

    if (!_unitTesting) loadSaveMapData(false);
    return 1;
}

void gpsMap::deleteAllPoints(String areaType, int areaNumber) {
  
  pointList arrayToModify = getRightArray(areaType, areaNumber);
  arrayToModify.numPoints = 0; 
  putBackToRightArray(areaType, areaNumber, arrayToModify);

  if (!_unitTesting) loadSaveMapData(false);
}

void gpsMap::deletePointFromMiddle(String areaType, int areaNumber, int positionToDelete) {
  pointList arrayToModify = getRightArray(areaType, areaNumber);

  if (arrayToModify.numPoints > 0) {
    int i = 0;
    for (i = positionToDelete; i <= arrayToModify.numPoints; i++) {
      arrayToModify.point[i] = arrayToModify.point[i+1];
    }
    arrayToModify.numPoints--;
    if (positionToDelete == 0 && (areaType == "MA" || areaType == "EA")) arrayToModify.point[arrayToModify.numPoints] = arrayToModify.point[0];  // last point of exclusion and main areas must be equal to the first point
  }
  putBackToRightArray(areaType, areaNumber, arrayToModify);

  if (!_unitTesting) loadSaveMapData(false);
}


int gpsMap::getNumberOfPoints(String areaType,int areaNumber) {
  pointList arrayToModify = getRightArray(areaType, areaNumber);
  
  if (arrayToModify.numPoints > 0 && arrayToModify.numPoints <= MAXPOINTS) return arrayToModify.numPoints;
  else return 0;  
}

long gpsMap::getPointX(String areaType, int areaNumber, int pointNumber) {
  pointList arrayToModify = getRightArray(areaType, areaNumber);
  return arrayToModify.point[pointNumber].x;
}

long gpsMap::getPointY(String areaType, int areaNumber, int pointNumber) {
  pointList arrayToModify = getRightArray(areaType, areaNumber);
  return arrayToModify.point[pointNumber].y;
}

int gpsMap::getMaxNumberOfPoints(String areaType, int areaNumber) {
  return MAXPOINTS-1;  
}

int gpsMap::getLongGrassTempAreaInUse() {
  return _longGrassTempAreaInUse;
}

void gpsMap::setLongGrassTempAreaSize(float size) {
  _tempAreaSize = size; 
}

void gpsMap::init(float size, float wiredInUse) {
  _tempAreaSize = size; 
  _wiredPerimeterInUse = wiredInUse;
  loadSaveMapData(true);
}

void gpsMap::loadSaveMapData(boolean readflag){
  if (readflag) Serial.println(F("loadSavegpsMapData:: read"));
  else Serial.println(F("loadSavegpsMapData: write"));
  
  if (readflag) {
    short magic = 30;
    int addr = ADDR_GPSMAP_DATA;
    eeread(addr, magic);
    if (magic != 30) {
      Serial.println(F("No GPS Perimeter data found."));
      return;  
    }
  }

  int addr = ADDR_GPSMAP_DATA;
  short magic = 30;
  eereadwrite(readflag, addr, magic); // magic


  eereadwrite(readflag, addr, _numberOfMainAreas);  

  for (int i=0; i <= _numberOfMainAreas; i++) {
    eereadwrite(readflag, addr, _mainAreas[i].numPoints);  
    int j=0;
    if (_mainAreas[i].numPoints > 0 && _mainAreas[i].numPoints <= MAXPOINTS) {
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
    if (_exclusionAreas[i].numPoints > 0 && _exclusionAreas[i].numPoints <= MAXPOINTS) {
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

  eereadwrite(readflag, addr, _numberOfSafePointLists);  

  for (int i=0; i <= _numberOfSafePointLists; i++) {
    eereadwrite(readflag, addr, _safePointList[i].numPoints);  
    int j=0;
    if (_safePointList[i].numPoints > 0 && _safePointList[i].numPoints <= MAXPOINTS) {
      for (j; j < _safePointList[i].numPoints; j++) {
        eereadwrite(readflag, addr, _safePointList[i].point[j].x);      
        eereadwrite(readflag, addr, _safePointList[i].point[j].y);      
      }
    }
  }

  Serial.println("-----------------");
  eereadwrite(readflag, addr, _numberOfHomingPointLists);  
  Serial.print(_numberOfHomingPointLists,0);
  Serial.println(" homing point lists loaded:");

  for (int i=0; i <= _numberOfHomingPointLists; i++) {
    eereadwrite(readflag, addr, _homingPointList[i].numPoints);  

    int j=0;
    if (_homingPointList[i].numPoints > 0 && _homingPointList[i].numPoints <= MAXPOINTS) {
      Serial.print(_homingPointList[i].numPoints);
      Serial.println(" homing points loaded:");
      for (j; j < _homingPointList[i].numPoints; j++) {
        eereadwrite(readflag, addr, _homingPointList[i].point[j].x);      
        eereadwrite(readflag, addr, _homingPointList[i].point[j].y);  
        Serial.print(_homingPointList[i].point[j].x );
        Serial.print (",");
        Serial.println(_homingPointList[i].point[j].y);  
      }
    }
  }
  Serial.println("-----------------");

  Serial.print(F("loadSaveMapData addrstop="));
  Serial.println(addr);
  
}


void gpsMap::doUnitTest() {
  _unitTesting = true;
  addPoint("MA",0,10.0000,10.0000);
  addPoint("MA",0,10.0000,10.0010);
  addPoint("MA",0,10.0010,10.0010);
  addPoint("MA",0,10.0010,10.0000);
  addPoint("MA",0,10.0000,10.0000);

  addPoint("EA",0,10.0002,10.0002);
  addPoint("EA",0,10.0002,10.0004);
  addPoint("EA",0,10.0004,10.0004);
  addPoint("EA",0,10.0004,10.0002);
  addPoint("EA",0,10.0002,10.0002);

  _longGrassTempAreaInUse = 0;

  doTest(1,100005,100005,false); // inside of main area / not in exclusion areas

  doTest(2,100003,100003,true);  // inside of exclusion area

  doTest(3,100000,100011, true); // outside of main areas

  setTemporaryArea(100007, 100007);

  doTest(4,1000071,1000071, false); // inside of temporary area

  doTest(5,1000095,1000095, true);  // outside of temporary area

  Serial.print("Test 6 - Heading from South to temp area middle (waiting 0) got:");
  Serial.println(getNewHeadingFromPerimeterDegrees(10.00065,10.00070));

  Serial.print("Test 7 - Heading from West to temp area middle (waiting 90) got:");
  Serial.println(getNewHeadingFromPerimeterDegrees(10.00070,10.00065));

  Serial.print("Test 8 - Heading from North to temp area middle (waiting 180) got:");
  Serial.println(getNewHeadingFromPerimeterDegrees(10.00075,10.00070));
  
  Serial.print("Test 9 - Heading from East to temp area middle (waiting 270) got:");
  Serial.println(getNewHeadingFromPerimeterDegrees(10.00070,10.00075));

  _unitTesting = false;
}

void gpsMap::doTest(uint8_t testNum, long lat, long lon, bool expectZero) {
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
