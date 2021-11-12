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

/*
 * UBX commands are the following,

Cold start : B5 62 06 04 04 00 FF B9 02 00 C8 8F
Warm start : B5 62 06 04 04 00 01 00 02 00 11 6C
Hot start  : B5 62 06 04 04 00 00 00 02 00 10 68
 */

// The TinyGPS++ object
HardwareSerial SerialGPS(GPS_UART_NO);
TinyGPSPlus gps;
//on
uint8_t gpsColdStart[]  = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0xB9, 0x02, 0x00, 0xC8, 0x8F};
uint8_t gpsWarmStart[]  = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x01, 0x00, 0x02, 0x00, 0x11, 0x6C};
uint8_t gpsHotStart[]   = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x68};

//Backup
uint8_t gpsToBakup[]   = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B}; //off
uint8_t gpsFromBakup[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37}; //on

//Sleep
uint8_t gpsToSleep[]   = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x08, 0x00, 0x16, 0x74}; //off
uint8_t gpsFromSleep[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x09, 0x00, 0x17, 0x76}; //on

int GPSGetPosition(t_SendData* DataSet, int averages, int timeOut){
  int      measures = 0;
  double   a_lng    = 0.0;
  double   a_lat    = 0.0;
  double   a_alt    = 0.0;
  int      a_hdop   = 0;
  int      gps_sat  = 0;
  bool     gotTime  = false;
  #ifdef HEIDI_CONFIG_TEST
  int      cnt      = 0;
  int      i        = 0;
  #endif
  setError(E_COULD_NOT_FETCH_GPS);
  setError(E_WRONG_GPS_VALUES);

  _D(DebugPrintln("GPS: Acquire position", DEBUG_LEVEL_1));
  while((measures < averages) && ((millis() < timeOut) || (timeOut == 0))){
    #ifdef DEBUG_SERIAL_GPS
    int i = 0;
    while ((SerialGPS.available() == 0) && ((millis() < timeOut) || (timeOut == 0))) { delay(1000); _DD(Serial.print('.'));
      i++; if (i >= 60) { i = 0;  _DD(DebugPrintln("", DEBUG_LEVEL_3)); }
    }
    _DD(DebugPrintln("", DEBUG_LEVEL_3));
    #else
    while ((SerialGPS.available() == 0) && ((millis() < timeOut) || (timeOut == 0))) {
      delay(10);
      #ifdef HEIDI_CONFIG_TEST
      _D(if (timeOut == 0) { cnt++; if (cnt == 1000) { cnt = 0; i++; Serial.print("."); }})
      #endif
    }
    #endif
    while ((SerialGPS.available() > 0) && (measures < averages) && ((millis() < timeOut) || (timeOut == 0))) {
      #ifdef DEBUG_SERIAL_GPS
      char r = SerialGPS.read();
      _DD(Serial.print(r));
      gps.encode(r);
      #else
      gps.encode(SerialGPS.read());
      #endif
      if (gps.location.isUpdated() && gps.location.isValid() && gps.altitude.isUpdated()){
        #ifdef DEBUG_SERIAL_GPS
        _DD(DebugPrintln("", DEBUG_LEVEL_3));
        #endif
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
      /*
       * already done on setup

      if (gps.date.isUpdated() && gps.date.isValid() && gps.time.isUpdated() && gps.time.isValid() && !gotTime){
        if ((gps.date.year() > 2000) && (gps.date.month() > 0) && (gps.date.day() > 0)){
          SetSysToGPS();
          gotTime = true;
        }
      }
      */
    }
  }
  if(measures > 0){
    a_lng /= measures;
    a_lat /= measures;
    a_alt /= measures;
    a_hdop /= measures;
    a_hdop /= 10;
	  rmError(E_COULD_NOT_FETCH_GPS);
  }
  DataSet->latitude = GeoToInt(a_lat);
  DataSet->longitude = GeoToInt(a_lng);
  DataSet->altitude = (int16_t)a_alt;
  //DataSet->secGPS = (int8_t)((millis() - startMs)/1000);
  DataSet->GPShdop  = (uint8_t)a_hdop;
  if (!((measures > 0) && (DataSet->latitude == 0) && (DataSet->longitude == 0))){ //0.0, 0.0 must be wrong (or a fish)
    rmError(E_WRONG_GPS_VALUES);
  }
  DataSet->metersOut = 0;
  if(!(getError(E_COULD_NOT_FETCH_GPS) || getError(E_WRONG_GPS_VALUES))){
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
  _D(DebugPrintln("GPS done: " + String((int)(millis() / 1000)), DEBUG_LEVEL_2));
  if (millis() >= ONE_MINUTE) { setError(E_GPS_LATE_LOCK); }
  return measures;
}

bool setSysTimeToGPSTime(int timeOut)
{
  uint16_t year = 0;
  uint8_t  mon  = 0;
  uint8_t  day  = 0;
  bool result   = false;
  #ifdef HEIDI_CONFIG_TEST
  uint16_t cnt  = 0;
  int      i    = 0;
  #endif
  setError(E_COULD_NOT_FETCH_GPS_TIME);
  //_DD(DebugPrint("GPS raw data: ", DEBUG_LEVEL_3));
  while(((year <= 2000) || (mon == 0) || (day == 0)) && ((millis() < timeOut) || (timeOut == 0))){
    #ifdef DEBUG_SERIAL_GPS
    int i = 0;
    while ((SerialGPS.available() == 0) && (millis() < timeOut)) { delay(1000); _DD(Serial.print('.'));
      i++; if (i >= 60) { i = 0;  _DD(DebugPrintln("", DEBUG_LEVEL_3)); }
    }
    _DD(DebugPrintln("", DEBUG_LEVEL_3));
    #else
    while ((SerialGPS.available() == 0) && ((millis() < timeOut) || (timeOut == 0))) {
      delay(10);
      #ifdef HEIDI_CONFIG_TEST
      _D(if (timeOut == 0) { cnt++; if (cnt == 1000) { cnt = 0; i++; Serial.print("."); }})
      #endif
    }
    #endif
    while ((SerialGPS.available() > 0)  && ((millis() < timeOut) || (timeOut == 0))){
      #ifdef DEBUG_SERIAL_GPS
      char r = SerialGPS.read();
      _DD(Serial.print(r));
      gps.encode(r);
      #else
      gps.encode(SerialGPS.read());
      #endif
      if (gps.date.isUpdated() && gps.date.isValid() && gps.time.isUpdated() && gps.time.isValid()){
        #ifdef DEBUG_SERIAL_GPS
        _DD(DebugPrintln("", DEBUG_LEVEL_3));
        #endif
        year = gps.date.year();
        mon  = gps.date.month();
        day  = gps.date.day();
        if ((year > 2000) && (mon > 0) && (day > 0)){
          if((result = SetSysToGPS()) == true) { rmError(E_COULD_NOT_FETCH_GPS_TIME);}
		      return result;
        }
	    }
    }
  }
  return result;
}

bool SetSysToGPS(){
  tm time;
  _DD(DebugPrint("Set system time to GPS time = ", DEBUG_LEVEL_3));
  _DD(DebugPrint(String(gps.date.year()) + "-" + LenTwo(String(gps.date.month())) + "-" + LenTwo(String(gps.date.day())), DEBUG_LEVEL_3));
  _DD(DebugPrint(" Time= ", DEBUG_LEVEL_3));
  _DD(DebugPrintln(LenTwo(String(gps.time.hour())) + ":" + LenTwo(String(gps.time.minute())) + ":" + LenTwo(String(gps.time.second())), DEBUG_LEVEL_3));
  time.tm_year = gps.date.year()-1900;
  time.tm_mon  = gps.date.month()-1;
  time.tm_mday = gps.date.day();
  time.tm_hour = gps.time.hour();
  time.tm_min  = gps.time.minute();
  time.tm_sec  = gps.time.second();
  if (gps.time.centisecond() > 50) { time.tm_sec++; }
  time_t t = mktime(&time);
  struct timeval now = { .tv_sec = t };
  return (settimeofday(&now, NULL) == 0);
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
    pinMode(GPS_RXD, INPUT_PULLDOWN);
    pinMode(GPS_TXD, OUTPUT);
    SerialGPS.flush();
    _D(DebugPrintln("GPS open", DEBUG_LEVEL_2));
  } _D(else { DebugPrintln("GPS already open", DEBUG_LEVEL_2); delay(50); })
  return true;
}
void closeGPS(){
  if (GPSenabled){
    SerialGPS.flush();
    SerialGPS.end();
    closeMeasures();
    _D(DebugPrintln("GPS closed", DEBUG_LEVEL_2));
  } _D(else { DebugPrintln("GPS already closed", DEBUG_LEVEL_2); })
  GPSenabled = false;
}

#if DEBUG_LEVEL >= DEBUG_LEVEL_1
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
