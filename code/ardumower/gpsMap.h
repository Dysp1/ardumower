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

/*
#ifndef GPSMap_h
#define GPSMap_h

#include "Arduino.h"

typedef struct {int x, y;} Point;

class GPSMap
{
    private:
        int isLeft( Point P0, Point P1, Point P2 );
        Point _currentLocation = {0,0};
        Point _mainAreaPointList[31] = {};
        int _numberOfMainAreaPoints = 0;
    public:
        int wn_PnPoly();
        float distanceToClosestWall();
        void setCurrentLocation( float x, float y);
        uint8_t addMainAreaPoint( float x, float y);
        uint8_t removeMainAreaPoint( int pointNro);
};

#endif
*/



#ifndef GPSMAP_H
#define GPSMAP_H

#include "Arduino.h"

enum {INSIDE_PERIMETER, PERIMETER_EDGE_NODE, CONNECTED_NORTH, CONNECTED_EAST, CONNECTED_SOUTH, CONNECTED_WEST};

class GPSMAP
{
public:
  GPSMAP();
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
  unsigned long nextGpsMapSaveTime ;
  boolean gpsMapDataChanged ; 
  void setBitOnOfPoint(float longitude, float latitude, int bitToSet) ;
  boolean bitOfPointIsOn(float longitude, float latitude, int bitToCompare) ;
};


#endif
