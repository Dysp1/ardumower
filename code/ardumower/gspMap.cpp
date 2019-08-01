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
#include "vector"

#define MAGIC 1
#define ADDR_GPSMAP_DATA 900

GPSMAP::GPSMAP() 
{
  gpsMapData[][][] ;
}

//
// public methods
//

void GPSMAP::init(){
  loadSaveMapData(true);
}

void GPSMAP::loadSaveMapData(boolean readflag){
  if (readflag) Console.println(F("loadSaveGPSMapData:: read"));
    else Console.println(F("loadSaveGPSMapData: write"));
  int addr = ADDR_GPSMAP_DATA;
  short magic = 0;
  if (!readflag) magic = MAGIC;  
  eereadwrite(readflag, addr, magic); // magic
  if ((readflag) && (magic != MAGIC)) {
    Console.println(F("EEPROM ERROR DATA: NO GPSMAP DATA FOUND"));    
    return;
  }
  eereadwrite(readflag, addr, gpsMapData);  
  Console.print(F("loadSaveMapData addrstop="));
  Console.println(addr);
}

