
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
  Point _currentLocation = {x,y};

  if (_numberOfExclusionAreas > 0) {
    for (int j=0; j < _numberOfExclusionAreas; j++) {

      if (_exclusionAreas[j].numPoints < 3 ) break; //less than 3 points can't make an area.

      // loop through all edges of the polygon
      for (int i=0; i < _exclusionAreas[j].numPoints; i++) {   // edge from area[i] to  area[i+1]
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

  wn = 0;
  if (_numberOfMainAreas > 0) {
    for (int j=0; j < _numberOfMainAreas; j++) {

      if (_mainAreas[j].numPoints < 3 ) break; //less than 3 points can't make an area.

      // loop through all edges of the polygon
      for (int i=0; i < _mainAreas[j].numPoints; i++) {   // edge from area[i] to  area[i+1]

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
      if (wn != 0) return wn; // we are inside of one main area, no need to check others
    }
  }

  if (wn == 0 && _wiredPerimeterInUse) return 11;

  return wn;
}

//  if(_longGrassTempAreaInUse > 0 && (wn !=0 || _wiredPerimeterInUse) && wnTempArea !=0) return 6;
/*Serial.println(wn);
Serial.println(wnTempArea);
Serial.println(_wiredPerimeterInUse);
Serial.println(_longGrassTempAreaInUse);

   {
    if(_longGrassTempAreaInUse > 0) return wnTempArea;
  }

  if (wn != 0) {
    if(_longGrassTempAreaInUse > 0) return wnTempArea;
  }

  if (wn == 0 && _mainAreas[0].numPoints < 3 && _wiredPerimeterInUse) return 6;
*/


// 0  0 1
// 1  1 2
// 2  2 3
// 3  3 4
// 4  4 5


int gpsMap::insideLongGrassTempArea(float x, float y)
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

float gpsMap::distanceFromTempAreaMiddle(float lat, float lon)
{
  return sqrt( pow((lat - _longGrassTempAreaMiddlePoint.x),2) + pow((lon - _longGrassTempAreaMiddlePoint.y),2) );
}


float gpsMap::getNewHeadingLongGrassAreaDegrees( float lat, float lon) {
  if (_longGrassTempAreaInUse > 0) {
    float degrees = atan2( (_longGrassTempAreaMiddlePoint.y - lon ), (_longGrassTempAreaMiddlePoint.x - lat) )/PI*180.0;
    if (degrees < 0) degrees += 360; 
    return degrees;
  }
}


float gpsMap::getNewHeadingFromPerimeterDegrees( float lat, float lon) {
  int closestPoint = 0;
  float lastShortestDistance = 88888.0;
  float shortestDistance = 99999.0;
  int i = 0;
  for (i=0; i < _homingPointList[0].numPoints; i++) {
    shortestDistance = min(shortestDistance, (sqrt( pow((lat - _homingPointList[0].point[i].x),2) + pow((lon - _homingPointList[0].point[i].y),2) )));
/*    Serial.print("distance:");
    Serial.println(shortestDistance,8);

    Serial.print(" lastshortest:");
    Serial.println(lastShortestDistance,8);

    Serial.print(" pointnro:");    
    Serial.print(i); 
    Serial.print("distance:");    
    Serial.print(sqrt( pow((lat - _homingPointList[0].point[i].x),2) + pow((lon - _homingPointList[0].point[i].y),2) ),8);
    Serial.print(" currlat:");    
    Serial.print(lat,6);
    Serial.print(" currlon:");    
    Serial.print(lon,6);
    Serial.print(" pointlat:");    
    Serial.print(_homingPointList[0].point[i].x,6);
    Serial.print(" pointlon:");    
    Serial.println(_homingPointList[0].point[i].y,6);
  */  
    if (shortestDistance < lastShortestDistance) {
      lastShortestDistance = shortestDistance;
      closestPoint = i;
    /*  Serial.print("   HERE  ");
      Serial.print(i);
      Serial.print("   HERE  ");
      Serial.print(closestPoint);
      Serial.print("   HERE  ");
   */
    }

  }
/*  Serial.print("closest point: ");
  Serial.print(closestPoint);
  Serial.print(" - ");
  Serial.print(_homingPointList[0].point[closestPoint].x,6);
  Serial.print(",");
  Serial.print(_homingPointList[0].point[closestPoint].y,6);
  Serial.print(" - dist: ");
*/
  float degrees = atan2( (_homingPointList[0].point[closestPoint].y - lon ), (_homingPointList[0].point[closestPoint].x - lat) )/PI*180.0;
  if (degrees < 0) degrees += 360; 

  //Serial.println(degrees);
  return degrees;
}

uint8_t gpsMap::setTemporaryArea( float x, float y) {
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

int gpsMap::addMainAreaPoint(int areaNumber, float lat, float lon) {
    if (_mainAreas[areaNumber].numPoints >= MAXMAINAREAPOINTS-1) {
        return 1;
    } else {
        _mainAreas[areaNumber].point[_mainAreas[areaNumber].numPoints] = {lat , lon};
        _mainAreas[areaNumber].point[_mainAreas[areaNumber].numPoints + 1] = _mainAreas[areaNumber].point[0]; // last point of area must be equal to first
        _mainAreas[areaNumber].numPoints++;
    }
    if (!_unitTesting) loadSaveMapData(false);
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
    if (!_unitTesting) loadSaveMapData(false);
    return 0;
}

int gpsMap::addHomingPoint(int areaNumber, float lat, float lon) {
    if (_homingPointList[areaNumber].numPoints >= MAXHOMINGPOINTS-1) {
        return 1;
    } else {
        _homingPointList[areaNumber].point[_homingPointList[0].numPoints] = {lat , lon};
        _homingPointList[areaNumber].numPoints++;
    }
    if (!_unitTesting) loadSaveMapData(false);
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
  if (_mainAreas[areaNumber].numPoints > 0 && _mainAreas[areaNumber].numPoints <= MAXMAINAREAPOINTS) return _mainAreas[areaNumber].numPoints;
  else return 0;  
}

int gpsMap::getLongGrassTempAreaInUse() {
  return _longGrassTempAreaInUse;
}

void gpsMap::setLongGrassTempAreaSize(float size) {
  _tempAreaSize = (float)((float)size/100000); 
}

float gpsMap::getExclusionAreaPointX(int areaNumber, int pointNumber) {
  return _exclusionAreas[areaNumber].point[pointNumber].x;
}

float gpsMap::getExclusionAreaPointY(int areaNumber, int pointNumber) {
  return _exclusionAreas[areaNumber].point[pointNumber].y;
}

int gpsMap::getNumberOfExclusionAreaPoints(int areaNumber) {
  if (_exclusionAreas[areaNumber].numPoints > 0 && _exclusionAreas[areaNumber].numPoints <= MAXEXCLUSIONAREAPOINTS) return _exclusionAreas[areaNumber].numPoints;
  else return 0;  
}

float gpsMap::getHomingPointX(int areaNumber, int pointNumber) {
  return _homingPointList[areaNumber].point[pointNumber].x;
}

float gpsMap::getHomingPointY(int areaNumber, int pointNumber) {
  return _homingPointList[areaNumber].point[pointNumber].y;
}

int gpsMap::getNumberOfHomingPoints(int areaNumber) {
  if (_homingPointList[areaNumber].numPoints > 0 && _homingPointList[areaNumber].numPoints <= MAXHOMINGPOINTS) return _homingPointList[areaNumber].numPoints;
  else return 0;  
}

void gpsMap::deleteMainAreaPoints(int areaNumber) {
  _mainAreas[areaNumber].numPoints = 0; 
  if (!_unitTesting) loadSaveMapData(false);
}

void gpsMap::deleteExclusionAreaPoints(int areaNumber) {
  _exclusionAreas[areaNumber].numPoints = 0;
  if (!_unitTesting) loadSaveMapData(false);
}

void gpsMap::deleteHomingPoints(int areaNumber) {
  _homingPointList[areaNumber].numPoints = 0;
  if (!_unitTesting) loadSaveMapData(false);
}

void gpsMap::init(float size, float wiredInUse) {
  _tempAreaSize = (float)((float)size/100000); 
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
    if (_mainAreas[i].numPoints > 0 && _mainAreas[i].numPoints <= MAXMAINAREAPOINTS) {
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
    if (_exclusionAreas[i].numPoints > 0 && _exclusionAreas[i].numPoints <= MAXEXCLUSIONAREAPOINTS) {
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

  eereadwrite(readflag, addr, _numberOfHomingPointLists);  

  for (int i=0; i <= _numberOfHomingPointLists; i++) {
    eereadwrite(readflag, addr, _homingPointList[i].numPoints);  
    int j=0;
    if (_homingPointList[i].numPoints > 0 && _homingPointList[i].numPoints <= MAXHOMINGPOINTS) {
      for (j; j <= _homingPointList[i].numPoints; j++) {
        eereadwrite(readflag, addr, _homingPointList[i].point[j].x);      
        eereadwrite(readflag, addr, _homingPointList[i].point[j].y);      
      }
    }
  }

  Serial.print(F("loadSaveMapData addrstop="));
  Serial.println(addr);
  
}


void gpsMap::doUnitTest() {
  _unitTesting = true;
  addMainAreaPoint(0,10.0000,10.0000);
  addMainAreaPoint(0,10.0000,10.0010);
  addMainAreaPoint(0,10.0010,10.0010);
  addMainAreaPoint(0,10.0010,10.0000);
  addMainAreaPoint(0,10.0000,10.0000);

  addExclusionAreaPoint(0,10.0002,10.0002);
  addExclusionAreaPoint(0,10.0002,10.0004);
  addExclusionAreaPoint(0,10.0004,10.0004);
  addExclusionAreaPoint(0,10.0004,10.0002);
  addExclusionAreaPoint(0,10.0002,10.0002);

  _longGrassTempAreaInUse = 0;

  doTest(1,10.0005,10.0005,false); // inside of main area / not in exclusion areas

  doTest(2,10.0003,10.0003,true);  // inside of exclusion area

  doTest(3,10.0000,10.0011, true); // outside of main areas

  setTemporaryArea(10.0007, 10.0007);

  doTest(4,10.00071,10.00071, false); // inside of temporary area

  doTest(5,10.00095,10.00095, true);  // outside of temporary area

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
