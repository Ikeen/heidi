/*
 * heidi-gps.cpp
 *
 *  Created on: 07.07.2020
 *      Author: frank
 */

#include "heidi-defines.h"
bool GPSenabled = false;
#ifdef GPS_MODULE
#include <Arduino.h>
#include <sys/time.h>
#include "heidi-sys.h"
#include "heidi-gps.h"
#include "heidi-error.h"
#include "heidi-fence.h"
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
  int      a_hdop   = 0;
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
        a_hdop += gps.hdop.value();
        _D(DebugPrint("GPS: Lat :" + String(gps.location.lat(),6) + ", Lon: " + String(gps.location.lng(), 6), DEBUG_LEVEL_1));
        _D(DebugPrintln("GPS: Sats:" + String(gps.satellites.value()) + ", HDOP: " + String(gps.hdop.value() / 10), DEBUG_LEVEL_1));
      }
    }
  }
  if(measures > 0){
    a_lng /= measures;
    a_lat /= measures;
    a_alt /= measures;
    a_hdop /= measures;
    a_hdop /= 10;
	  rmError(DataSet, COULD_NOT_FETCH_GPS);
  }
  DataSet->latitude = GeoToInt(a_lat);
  DataSet->longitude = GeoToInt(a_lng);
  DataSet->altitude = (int16_t)a_alt;
  //DataSet->secGPS = (int8_t)((millis() - startMs)/1000);
  DataSet->GPShdop  = (uint8_t)a_hdop;
  if (!((measures > 0) && (DataSet->latitude == 0) && (DataSet->longitude == 0))){ //0.0, 0.0 must be wrong (or a fish)
    rmError(DataSet, WRONG_GPS_VALUES);
  }
  DataSet->metersOut = 0;
  if(!(getError(DataSet, COULD_NOT_FETCH_GPS) || getError(DataSet, WRONG_GPS_VALUES))){
    t_PointData position;
    position.latitude = a_lat;
    position.longitude = a_lng;
    if(!pointIn(&position)){
      int distance = meterDistFromFence(&position);
      if(distance > 65535){
        DataSet->metersOut = 65535;
      } else {
        DataSet->metersOut = (uint16_t)distance;
      }
      _D(DebugPrintln("GPS: position " + String(DataSet->metersOut) + " meters out", DEBUG_LEVEL_1));
    } _D(else {_D(DebugPrintln("GPS: position in", DEBUG_LEVEL_1));})
  }
  DataSet->satellites = gps_sat;
  _D(DebugPrintln("GPS done: " + String((millis() - startMs)/1000), DEBUG_LEVEL_2));
  return measures;
}

bool SetSysToGPSTime()
{
  uint16_t year = 0;
  uint8_t  mon  = 0;
  uint8_t  day  = 0;
  uint32_t timeoutms = WAIT_FOR_GPS_TIME;
  int startMs = millis();
  setError(COULD_NOT_FETCH_GPS_TIME);
  //_DD(DebugPrint("GPS raw data: ", DEBUG_LEVEL_3));
  while(((year <= 2000) || (mon == 0) || (day == 0)) && ((millis() - startMs) < timeoutms)){
    while ((SerialGPS.available() == 0) && ((millis() - startMs) < timeoutms)) {
      delay(10);
    }
    while ((SerialGPS.available() > 0)  && ((millis() - startMs) < timeoutms)){
     int y = SerialGPS.read();
     //_DD(DebugPrint(String((char)y), DEBUG_LEVEL_3));
     gps.encode(y);
     //gps.encode(SerialGPS.read());
     if (gps.date.isUpdated() && gps.date.isValid() && gps.time.isUpdated() && gps.time.isValid()){
        //_DD(DebugPrintln("", DEBUG_LEVEL_3));
        year = gps.date.year();
        mon  = gps.date.month();
        day  = gps.date.day();
        if ((year > 2000) && (mon > 0) && (day > 0)){
          bool result = SetSysToGPS();
          _DD(DebugPrint("Set sys-time to GPS time Date= ", DEBUG_LEVEL_3));
          _DD(DebugPrint(String(year) + "-" + String(mon) + "-" + String(day), DEBUG_LEVEL_3));
          _DD(DebugPrint(" Time= ", DEBUG_LEVEL_3));
          _DD(DebugPrintln(String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()), DEBUG_LEVEL_3));
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

bool openGPS(){
  if (GSMenabled){
    _D(DebugPrintln("GSM still enabled - cannot open GPS", DEBUG_LEVEL_1); delay(50);)
    return false;
  }
  if (!openMeasures()){
    _D(DebugPrintln("enable GPS power failed", DEBUG_LEVEL_1); delay(50);)
    return false;
  }
  if (!GPSenabled){
    GPSenabled = true;
    SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RXD, GPS_TXD, false);

    _D(DebugPrintln("GPS open", DEBUG_LEVEL_2));
  } _D(else { DebugPrintln("GPS already open", DEBUG_LEVEL_2); delay(50); })
  return true;
}
void closeGPS(){
  if (GPSenabled){
    SerialGPS.end();
    closeMeasures();
    _D(DebugPrintln("GPS closed", DEBUG_LEVEL_2));
  } _D(else { DebugPrintln("GPS already closed", DEBUG_LEVEL_2); })
  GPSenabled = false;
}

void testGPS(){
  int startMs = millis();
  while ((SerialGPS.available() == 0) && ((millis() - startMs) < 60000)) { delay(10); }
  while ((SerialGPS.available() > 0)) {
    gps.encode(SerialGPS.read());
    if (   gps.location.isUpdated() && gps.location.isValid() && gps.altitude.isValid()
        && gps.satellites.isValid() && gps.hdop.isValid()){
      _D(DebugPrint("Latitude= ", DEBUG_LEVEL_1));
      _D(DebugPrint(gps.location.lat(), 6, DEBUG_LEVEL_1));
      _D(DebugPrint(" Longitude= ", DEBUG_LEVEL_1));
      _D(DebugPrint(gps.location.lng(), 6, DEBUG_LEVEL_1));
      _D(DebugPrint(" Altitude= ", DEBUG_LEVEL_1));
      _D(DebugPrintln(gps.altitude.meters(), 6, DEBUG_LEVEL_1));
      _D(DebugPrint("Satellites= ", DEBUG_LEVEL_1));
      _D(DebugPrint(gps.satellites.value(), DEBUG_LEVEL_1));
      _D(DebugPrint(" HDOP= ", DEBUG_LEVEL_1));
      _D(DebugPrintln(gps.hdop.value()/10, DEBUG_LEVEL_1));
      break;
    }
  }
  if(!SerialGPS.available()){
    _D(DebugPrintln("Could not fetch GPS", DEBUG_LEVEL_1));
  }
}

#if DEBUG_LEVEL >= DEBUG_LEVEL_1
void _PrintDataGPS(){
  _DD(DebugPrint("Latitude= ", DEBUG_LEVEL_3));
  _DD(DebugPrint(gps.location.lat(), 6, DEBUG_LEVEL_3));
  _DD(DebugPrint(" Longitude= ", DEBUG_LEVEL_3));
  _DD(DebugPrint(gps.location.lng(), 6, DEBUG_LEVEL_3));
  _DD(DebugPrint(" Altitude= ", DEBUG_LEVEL_3));
  _DD(DebugPrint(gps.altitude.meters(), 6, DEBUG_LEVEL_3));
  _DD(DebugPrint(" Date= ", DEBUG_LEVEL_3));
  _DD(DebugPrint(String(gps.date.year()) + "-" + String(gps.date.month()) + "-" + String(gps.date.day()), DEBUG_LEVEL_3));
  _DD(DebugPrint(" Time= ", DEBUG_LEVEL_3));
  _DD(DebugPrint(String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()), DEBUG_LEVEL_3));
  _DD(DebugPrint(" Age= ", DEBUG_LEVEL_3));
  _DD(DebugPrint(gps.location.age(), DEBUG_LEVEL_3));
  _DD(DebugPrint("ms", DEBUG_LEVEL_3));
  _DD(DebugPrint(" Sat= ", DEBUG_LEVEL_3));
  _DD(DebugPrintln(gps.satellites.value(), DEBUG_LEVEL_3));
}
#endif

#endif //GPS_MODULE
