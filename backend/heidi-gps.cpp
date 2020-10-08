/*
 * heidi-gps.cpp
 *
 *  Created on: 07.07.2020
 *      Author: frank
 */

#include "heidi-defines.h"
#ifdef GSM_MODULE
#include <Arduino.h>
#include <sys/time.h>
#include "heidi-sys.h"
#include "heidi-gps.h"
#include "heidi-error.h"
#include "heidi-debug.h"
#include "heidi-measures.h"


// The TinyGPS++ object
HardwareSerial SerialGPS(GPS_UART_NO);
TinyGPSPlus gps;


int GPSGetPosition(t_SendData* DataSet, int averages, int timeoutms){
  int      measures = 0;
  double   a_lng    = 0.0;
  double   a_lat    = 0.0;
  double   a_alt    = 0.0;
  int      gps_sat  = 0;
  uint32_t startMs  = millis();

  setError(DataSet, COULD_NOT_FETCH_GPS);
  setError(DataSet, WRONG_GPS_VALUES);

  _D(DebugPrintln("GPS: Acquire position", DEBUG_LEVEL_1));
  while((measures < averages) && ((millis() - startMs) < timeoutms)){
    while ((SerialGPS.available() == 0) && ((millis() - startMs) < timeoutms)) { delay(10); }
    while ((SerialGPS.available() > 0) && (measures < averages)) {
      gps.encode(SerialGPS.read());
      if (gps.location.isUpdated() && gps.location.isValid() && gps.altitude.isUpdated()){
        measures++;
        if (gps_sat < gps.satellites.value()) { gps_sat = gps.satellites.value(); }
        _D(_PrintDataGPS());
        a_lng += gps.location.lng();
        a_lat += gps.location.lat();
        a_alt += gps.altitude.meters();
      }
    }
  }
  if(measures > 0){
    a_lng = a_lng / measures;
    a_lat = a_lat / measures;
    a_alt = a_alt / measures;
	  rmError(DataSet, COULD_NOT_FETCH_GPS);
  }
  DataSet->latitude = (int32_t)(a_lat * 1000000);
  DataSet->longitude = (int32_t)(a_lng * 1000000);
  DataSet->altitude = (uint16_t)a_alt;
  DataSet->secGPS = (int8_t)((millis() - startMs)/1000);
  if (!((measures > 0) && (DataSet->latitude == 0) && (DataSet->longitude == 0))){ //0.0, 0.0 must be wrong (or a fish)
    rmError(DataSet, WRONG_GPS_VALUES);
  }
  DataSet->satellites = gps_sat;
  _D(DebugPrintln("GPS done: " + String(DataSet->secGPS), DEBUG_LEVEL_2));
  return measures;
}

bool SetSysToGPSTime()
{
  int startMs = millis();
  uint16_t year = 0;
  uint8_t  mon  = 0;
  uint8_t  day  = 0;
  uint32_t timeoutms = WAIT_FOR_GPS_TIME;
  setError(COULD_NOT_FETCH_GPS_TIME);
  while(((year <= 2000) || (mon == 0) || (day == 0)) && ((millis() - startMs) < timeoutms)){
    while ((SerialGPS.available() == 0) && ((millis() - startMs) < timeoutms)) {
      delay(10);
    }
    while (SerialGPS.available() > 0) {
     gps.encode(SerialGPS.read());
     if (gps.date.isUpdated() && gps.date.isValid() && gps.time.isUpdated() && gps.time.isValid()){
        year = gps.date.year();
        mon  = gps.date.month();
        day  = gps.date.day();
        if ((year > 2000) && (mon > 0) && (day > 0)){
          bool result = SetSysToGPS();
          _D(DebugPrint("Set sys-time to GPS time Date= ", DEBUG_LEVEL_3));
          _D(DebugPrint(String(year) + "-" + String(mon) + "-" + String(day), DEBUG_LEVEL_3));
          _D(DebugPrint(" Time= ", DEBUG_LEVEL_3));
          _D(DebugPrintln(String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()), DEBUG_LEVEL_3));
		      return result;
        }
	    }
    }
  }
  return false;
}
bool SetSysToGPS(){
  tm time;
  int startms = millis();
  time.tm_year = gps.date.year()-1900;
  time.tm_mon  = gps.date.month()-1;
  time.tm_mday = gps.date.day();
  time.tm_hour = gps.time.hour();
  time.tm_min  = gps.time.minute();
  time.tm_sec  = gps.time.second();
  if (gps.time.centisecond() > 50) { time.tm_sec++; }
  time_t t = mktime(&time);
  struct timeval now = { .tv_sec = t };
  if (settimeofday(&now, NULL) == 0){
	  int stampms = gps.time.second() + gps.time.minute() * 60 + gps.time.hour() * 3600;
	  stampms *= 1000;
	  stampms += gps.time.centisecond() * 10;
	  SetBootTimeFromMs(stampms - startms);
	  setError(COULD_NOT_FETCH_GPS_TIME);
	  return true;
  }
  return false;
}

void openGPS(){
  MeasuresOn();
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX, false);
  _D(DebugPrintln("GPS on", DEBUG_LEVEL_2));
}
void closeGPS(){
  SerialGPS.end();
  MeasuresOff();
  _D(DebugPrintln("GPS off", DEBUG_LEVEL_2));
}

#if DEBUG_LEVEL >= DEBUG_LEVEL_1
void _PrintDataGPS(){
  _D(DebugPrint("Latitude= ", DEBUG_LEVEL_3));
  _D(DebugPrint(gps.location.lat(), 6, DEBUG_LEVEL_3));
  _D(DebugPrint(" Longitude= ", DEBUG_LEVEL_3));
  _D(DebugPrint(gps.location.lng(), 6, DEBUG_LEVEL_3));
  _D(DebugPrint(" Altitude= ", DEBUG_LEVEL_3));
  _D(DebugPrint(gps.altitude.meters(), 6, DEBUG_LEVEL_3));
  _D(DebugPrint(" Date= ", DEBUG_LEVEL_3));
  _D(DebugPrint(String(gps.date.year()) + "-" + String(gps.date.month()) + "-" + String(gps.date.day()), DEBUG_LEVEL_3));
  _D(DebugPrint(" Time= ", DEBUG_LEVEL_3));
  _D(DebugPrint(String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()), DEBUG_LEVEL_3));
  _D(DebugPrint(" Age= ", DEBUG_LEVEL_3));
  _D(DebugPrint(gps.location.age(), DEBUG_LEVEL_3));
  _D(DebugPrint("ms", DEBUG_LEVEL_3));
  _D(DebugPrint(" Sat= ", DEBUG_LEVEL_3));
  _D(DebugPrintln(gps.satellites.value(), DEBUG_LEVEL_3));
}
#endif

#endif //GSM_MODULE
