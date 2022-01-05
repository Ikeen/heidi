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

HardwareSerial SerialGPS(GPS_UART_NO);
#ifdef  USE_TINY_GPS_LIB
// The TinyGPS++ object
TinyGPSPlus gps;
#else
gpsData_t gpsData;
#endif

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
    GPSwaitForData(timeOut);
    while ((SerialGPS.available() > 0) && (measures < averages) && ((millis() < timeOut) || (timeOut == 0))) {
      GPSprocessData();
      #ifdef  USE_TINY_GPS_LIB
      if (gps.location.isUpdated()) { if (gps.location.isValid()) { if (gps.altitude.isUpdated()) {
      #else
      if (gpsData.valid) { if (gpsData.type == GPS_DT_NAV_PVT) { if ((gpsData.data->navPVT.fixType == GPS_FIX_TYPE_2D_FIX) || (gpsData.data->navPVT.fixType == GPS_FIX_TYPE_3D_FIX)) {
      #endif
        #ifdef DEBUG_SERIAL_GPS
        _DD(DebugPrintln("", DEBUG_LEVEL_3));
        #endif
        measures++;
        if (gps_sat < GPS_GET_SAT) { gps_sat = GPS_GET_SAT; }
        _DD(_PrintDataGPS();)
        a_lng += GPS_GET_LON;
        a_lat += GPS_GET_LAT;
        a_alt += GPS_GET_ALT;
        a_hdop += GPS_GET_DOP;
        _D(DebugPrint("GPS: Lat :" + String(GPS_GET_LAT,6) + ", Lon: " + String(GPS_GET_LON, 6), DEBUG_LEVEL_1));
        _D(DebugPrintln("GPS: Sats:" + String(GPS_GET_SAT) + ", DOP: " + String(GPS_GET_DOP) + ", ACC: " + String(GPS_GET_ACC), DEBUG_LEVEL_1));
        #ifndef  USE_TINY_GPS_LIB
        gpsData.valid = false;
        #endif
      }}}
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

bool setBootTimeFromGPSTime(tm* bootTime, int timeOut)
{
  tm GPStime;
  setError(E_COULD_NOT_FETCH_GPS_TIME);
  while(SerialGPS.available() > 0){ SerialGPS.read(); }
  GPStime.tm_year = 0;
  GPStime.tm_mon  = 0;
  GPStime.tm_mday = 0;
  //_DD(DebugPrint("GPS raw data: ", DEBUG_LEVEL_3));
  while(   ((GPStime.tm_year < 2000) || (GPStime.tm_mon <= 0) || (GPStime.tm_mday <= 0))
        && ((millis() < timeOut) || (timeOut == 0))){
    GPSwaitForData(timeOut);
    while ((SerialGPS.available() > 0)  && ((millis() < timeOut) || (timeOut == 0))){
      GPSprocessData();
      #ifdef  USE_TINY_GPS_LIB
      if (gps.date.isUpdated()) { if (gps.date.isValid()) { if (gps.time.isUpdated()) { if (gps.time.isValid()) {
      #else
      if (gpsData.valid) { if (gpsData.type == GPS_DT_NAV_PVT) { if (gpsData.data->navPVT.valid == GPS_DATE_TIME_VALID) {{
      #endif
        #ifdef DEBUG_SERIAL_GPS
        _DD(DebugPrintln("", DEBUG_LEVEL_3));
        #endif
        GPStime.tm_year  = GPS_GET_YR;
        GPStime.tm_mon   = GPS_GET_MO;
        GPStime.tm_mday  = GPS_GET_DY;
        if ((GPStime.tm_year > 2000) && (GPStime.tm_mon > 0) && (GPStime.tm_mday > 0)){
          GPStime.tm_mon--;
          GPStime.tm_hour = GPS_GET_HR;
          GPStime.tm_min  = GPS_GET_MI;
          GPStime.tm_sec  = GPS_GET_SE;
          mktime(&GPStime);
          setBootTimeFromCurrentTime(&GPStime, bootTime);
          rmError(E_COULD_NOT_FETCH_GPS_TIME);
		      return true;
        }
	    }}}}
    }
  }
  return false;
}

bool openGPS(){
  /*
   * https://youtu.be/ylxwOg2pXrc?list=RDCMUCTXOorupCLqqQifs2jbz7rQ
   *

                                           i2c  ser1 ser2 usb  spi
  $GPTXT B5 62 | 06 01 | 08 00 | F0 | 41 | 00   00   00   00   00  | 00 |
  $GPRMC B5 62 | 06 01 | 08 00 | F0 | 04 | 00   00   00   00   00  | 00 | 03 3F
  $GPVTG B5 62 | 06 01 | 08 00 | F0 | 05 | 00   00   00   00   00  | 00 | 04 46
  $GPGGA B5 62 | 06 01 | 08 00 | F0 | 00 | 00   00   00   00   00  | 00 | FF 23
  $GPGSA B5 62 | 06 01 | 08 00 | F0 | 02 | 00   00   00   00   00  | 00 | 01 31
  $GPGSV B5 62 | 06 01 | 08 00 | F0 | 03 | 00   00   00   00   00  | 00 | 02 38

 NAV_PVT B5 62   06 01   08 00   01   07   00   00   00   00   00    00   17 DC

  */
#ifndef  USE_TINY_GPS_LIB
  uint8_t NMEA_MSG_ON_OFF[] = {GPS_UBX_CLASS_CFG,
                               GPS_UBX_CFG_MSG,
                               0x08, //length low byte
                               0x00, //length high byte
                               GPS_CFG_MSG_NMEA,
                               GPS_CFG_MSG_GGA,
                               GPS_CFG_OFF, //i2c
                               GPS_CFG_OFF, //uart1
                               GPS_CFG_OFF, //uart2
                               GPS_CFG_OFF, //USB
                               GPS_CFG_OFF, //SPI
                               0x00};       //unused

  uint8_t UBX_SET_RATE[] =  {  GPS_UBX_CLASS_CFG,
                               GPS_UBX_CFG_RATE,
                               0x06, //length low byte
                               0x00, //length high byte
                               0xE8, //interval [ms] low byte   (1000)
                               0x03, //interval [ms] high byte  (1000)
                               0x01, //must be 1 low byte
                               0x00, //must be 1 high byte
                               0x01, //should be 1 low byte
                               0x00, //should be 1 high byte
                               };
  //  B5 62 06 08 06 00 F4 01 01 00 01 00 0B 77
#endif
  if (GSMenabled){
    _D(DebugPrintln("GSM still enabled - cannot open GPS", DEBUG_LEVEL_1); delay(50);)
    return false;
  }

  if (!enableMeasures()){
    _D(DebugPrintln("enable GPS power failed", DEBUG_LEVEL_1); delay(50);)
    return false;
  }
  if (!GPSenabled){
    GPSenabled = true;
    SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RXD, GPS_TXD, false);
    pinMode(GPS_RXD, INPUT_PULLDOWN);
    pinMode(GPS_TXD, OUTPUT);

    #ifdef DEBUG_SERIAL_GPS
    #ifndef I2C_SWITCH
    extern bool powerOnReset;
    if(powerOnReset){
      while(SerialGPS.available() > 0){ SerialGPS.read(); }
      //print status of module at boot up - so restart
      pinMode(MEASURES_ENABLE_PIN,OUTPUT);
      digitalWrite(MEASURES_ENABLE_PIN, MEASURES_OFF);
      delay(1000);
      digitalWrite(MEASURES_ENABLE_PIN, MEASURES_ON);
      for (int i=0; i<2000; i++) {
        delay(1);
        String line = "";
        while(SerialGPS.available() > 0){ line += char(SerialGPS.read()); }
        _DD(DebugPrint(line, DEBUG_LEVEL_3);)
       i++;
      }
      _DD(DebugPrintln("", DEBUG_LEVEL_3));
    }
    #endif
    #endif

#ifndef  USE_TINY_GPS_LIB
    //disable NMEA - messages on all channels
    NMEA_MSG_ON_OFF[NMEA_MSG_ON_OFF_TYPE] = GPS_CFG_MSG_GGA;
    GPSsendMessage(NMEA_MSG_ON_OFF, sizeof(NMEA_MSG_ON_OFF));
    NMEA_MSG_ON_OFF[NMEA_MSG_ON_OFF_TYPE] = GPS_CFG_MSG_GLL;
    GPSsendMessage(NMEA_MSG_ON_OFF, sizeof(NMEA_MSG_ON_OFF));
    NMEA_MSG_ON_OFF[NMEA_MSG_ON_OFF_TYPE] = GPS_CFG_MSG_GSA;
    GPSsendMessage(NMEA_MSG_ON_OFF, sizeof(NMEA_MSG_ON_OFF));
    NMEA_MSG_ON_OFF[NMEA_MSG_ON_OFF_TYPE] = GPS_CFG_MSG_GSV;
    GPSsendMessage(NMEA_MSG_ON_OFF, sizeof(NMEA_MSG_ON_OFF));
    NMEA_MSG_ON_OFF[NMEA_MSG_ON_OFF_TYPE] = GPS_CFG_MSG_VGT;
    GPSsendMessage(NMEA_MSG_ON_OFF, sizeof(NMEA_MSG_ON_OFF));
    NMEA_MSG_ON_OFF[NMEA_MSG_ON_OFF_TYPE] = GPS_CFG_MSG_RMC;
    GPSsendMessage(NMEA_MSG_ON_OFF, sizeof(NMEA_MSG_ON_OFF));
    NMEA_MSG_ON_OFF[NMEA_MSG_ON_OFF_TYPE] = GPS_CFG_MSG_TXT;
    GPSsendMessage(NMEA_MSG_ON_OFF, sizeof(NMEA_MSG_ON_OFF));

    //enable UBX NAV PVT - message on UART1
    NMEA_MSG_ON_OFF[NMEA_MSG_ON_OFF_GROUP] = GPS_CFG_MSG_UBX;
    NMEA_MSG_ON_OFF[GPS_CFG_SER_CH_UART1]  = GPS_CFG_ON;
    NMEA_MSG_ON_OFF[NMEA_MSG_ON_OFF_TYPE]  = GPS_UBX_NAV_PVT;
    GPSsendMessage(NMEA_MSG_ON_OFF, sizeof(NMEA_MSG_ON_OFF));
    //enable UBX NAV STATUS - message on UART1
    //NMEA_MSG_ON_OFF[NMEA_MSG_ON_OFF_TYPE]  = GPS_UBX_NAV_STATUS;
    //GPSsendMessage(NMEA_MSG_ON_OFF, sizeof(NMEA_MSG_ON_OFF));

    //set rate
    UBX_SET_RATE[4] = 0xF4; //500 low  byte
    UBX_SET_RATE[5] = 0x01; //500 high byte
    GPSsendMessage(UBX_SET_RATE, sizeof(UBX_SET_RATE));

    gpsData.valid = false;
    gpsData.type  = GPS_DT_NONE;
    gpsData.data  = NULL;

#endif
    while(SerialGPS.available() > 0){ SerialGPS.read(); }
    _D(DebugPrintln("GPS open", DEBUG_LEVEL_2));
  } _D(else { DebugPrintln("GPS already open", DEBUG_LEVEL_2); delay(50); })
  return true;
}
void closeGPS(){
  if (GPSenabled){
    while(SerialGPS.available() > 0){ SerialGPS.read(); }
    SerialGPS.end();
    disableMeasures();
    _D(DebugPrintln("GPS closed", DEBUG_LEVEL_2));
  } _D(else { DebugPrintln("GPS already closed", DEBUG_LEVEL_2); })
  GPSenabled = false;
}

bool GPSwaitForData(int timeOut){
  #ifdef DEBUG_SERIAL_GPS
  int  i  = 0;
  #else
  #ifdef HEIDI_CONFIG_TEST
  int  cnt  = 0;
  int  i  = 0;
  #endif
  #endif

  #ifdef DEBUG_SERIAL_GPS
  while ((SerialGPS.available() == 0) && ((millis() < timeOut) || (timeOut == 0))) { delay(200); _DD(Serial.print('.'));
    i++; if (i >= 60) { i = 0;  _DD(DebugPrintln("", DEBUG_LEVEL_3)); }
  }
  _DD(DebugPrintln("", DEBUG_LEVEL_3));
  #else
  while ((SerialGPS.available() == 0) && ((millis() < timeOut) || (timeOut == 0))) {
    delay(10);
    #ifdef HEIDI_CONFIG_TEST
    _D(if (timeOut == 0) { cnt++; if (cnt == 1000) { cnt = 0; i++; Serial.print("."); }})
    if (i >= 60) { i = 0;  Serial.println(""); }
    #endif
  }
  #endif
  return ((millis() < timeOut) || (timeOut == 0));
}

void GPSprocessData(void){
  #ifdef DEBUG_SERIAL_GPS
  static int i=0;
  #ifdef  USE_TINY_GPS_LIB
  char r = SerialGPS.read();
  _DD(Serial.print(r));
  gps.encode(r);
  #else
  uint8_t r = SerialGPS.read();
  _DD(Serial.print(LenTwo(String(r,HEX)) + " ");)
  _DD(i++; if(i >= 32){Serial.println (""); i = 0; })
  GPSdecodeData(r);
  #endif
  #else
  #ifdef  USE_TINY_GPS_LIB
  gps.encode(SerialGPS.read());
  #else
  GPSdecodeData(SerialGPS.read());
  #endif
  #endif

}
#ifndef  USE_TINY_GPS_LIB
void GPSdecodeData(uint8_t data) {
  static int fpos = 0;
  static uint8_t checksumSent[2];
  static UBXMessage_t ubxBuffer;
  static int payloadSize;
  static uint8_t* buffer = (uint8_t*)&ubxBuffer;

  if ( fpos < 2 ) {  if ( data == UBX_HEADER[fpos] ){  fpos++; } else { fpos = 0; } return; }
  //header found, load id & length
  if ( fpos < 5 ) { buffer[fpos-2] = data; fpos++; return; }
  if ( fpos == 5 ) { //check header information
    buffer[fpos-2] = data;
    payloadSize = ubxBuffer.head.len;
    if (payloadSize > 0) {
      fpos++;
      gpsData.valid = false;
      gpsData.type  = GPS_DT_NONE;
      if(ubxBuffer.head.cls == GPS_UBX_CLASS_NAV){
        if(ubxBuffer.head.id == GPS_UBX_NAV_PVT){ gpsData.type = GPS_DT_NAV_PVT; }
      }
    } else { fpos = 0;} //something strange
    return;
  }
  if ( fpos < (payloadSize + 6)){
    if (gpsData.type != GPS_DT_NONE){ //if data type is known then load
      buffer[fpos-2] = data;
    } //otherwise crush it
    fpos++;
    return;
  }
  if (fpos == (payloadSize + 6)) { checksumSent[0] = data; fpos++; return;}
  if (fpos == (payloadSize + 7)) {
    uint8_t checksumCalc[2];
    checksumSent[1] = data;
    fpos = 0;
    GPScalcChecksum(buffer, checksumCalc, payloadSize+4);
    if((checksumCalc[0]==checksumSent[0]) && (checksumCalc[1]==checksumSent[1])){
      gpsData.data  = &ubxBuffer;
      gpsData.valid = true;
      return;
    }
  }
  //we should never come here
  gpsData.valid = false;
  gpsData.type  = GPS_DT_NONE;
  fpos = 0;
  return;
}

void GPScalcChecksum(uint8_t* buffer, uint8_t* CK, int len) {
  memset(CK, 0, 2);
  for (int i = 0; i < len; i++) {
    CK[0] += buffer[i];
    CK[1] += CK[0];
  }
}

void GPSsendMessage(uint8_t* message, int len){
  uint8_t fullmessage[128];
  fullmessage[0] = UBX_HEADER[0];
  fullmessage[1] = UBX_HEADER[1];
  int l = len;
  for(int i=0; i<l; i++){ fullmessage[i+2] = message[i]; }
  GPScalcChecksum(message, &fullmessage[l+2] ,l);
  l += 4;  //plus sync bytes and checksum
  for(int i=0; i<l; i++){
    SerialGPS.write(fullmessage[i]);
    delay(2);
  }
  delay(50);
}
#endif

#if DEBUG_LEVEL >= DEBUG_LEVEL_1
void testGPS(){
  int startMs = millis();
  while ((SerialGPS.available() == 0) && ((millis() - startMs) < 60000)) { delay(10); }
  while((SerialGPS.available() > 0) && ((millis() - startMs) < 60000)){
    GPSprocessData();
    #ifdef  USE_TINY_GPS_LIB
    if (gps.location.isUpdated()) { if (gps.location.isValid()){ if (gps.altitude.isUpdated()) {
    #else
    if (gpsData.valid) { if (gpsData.type == GPS_DT_NAV_PVT) { if ((gpsData.data->navPVT.fixType == GPS_FIX_TYPE_2D_FIX) || (gpsData.data->navPVT.fixType == GPS_FIX_TYPE_3D_FIX)) {
    #endif
      _D(DebugPrint("Latitude= ", DEBUG_LEVEL_1));
      _D(DebugPrint(GPS_GET_LAT, 6, DEBUG_LEVEL_1));
      _D(DebugPrint(" Longitude= ", DEBUG_LEVEL_1));
      _D(DebugPrint(GPS_GET_LON, 6, DEBUG_LEVEL_1));
      _D(DebugPrint(" Altitude= ", DEBUG_LEVEL_1));
      _D(DebugPrintln(GPS_GET_ALT, 6, DEBUG_LEVEL_1));
      _D(DebugPrint("Satellites= ", DEBUG_LEVEL_1));
      _D(DebugPrint(GPS_GET_SAT, DEBUG_LEVEL_1));
      _D(DebugPrint(" HDOP= ", DEBUG_LEVEL_1));
      _D(DebugPrint((int)(GPS_GET_DOP/10), DEBUG_LEVEL_1));
      _D(DebugPrint(" ACC= ", DEBUG_LEVEL_1));
      _D(DebugPrintln(String(GPS_GET_ACC), DEBUG_LEVEL_1));
      break;
    }}}
  }
  if(!SerialGPS.available()){
    _D(DebugPrintln("Could not fetch GPS", DEBUG_LEVEL_1));
  }
}

void _PrintDataGPS(){
  _DD(DebugPrint("Latitude= ", DEBUG_LEVEL_3));
  _DD(DebugPrint(GPS_GET_LAT, 6, DEBUG_LEVEL_3));
  _DD(DebugPrint(" Longitude= ", DEBUG_LEVEL_3));
  _DD(DebugPrint(GPS_GET_LON, 6, DEBUG_LEVEL_3));
  _DD(DebugPrint(" Altitude= ", DEBUG_LEVEL_3));
  _DD(DebugPrint(GPS_GET_ALT, 6, DEBUG_LEVEL_3));
  _DD(DebugPrint(" Date= ", DEBUG_LEVEL_3));
  _DD(DebugPrint(String(GPS_GET_YR) + "-" + String(GPS_GET_MO) + "-" + String(GPS_GET_DY), DEBUG_LEVEL_3));
  _DD(DebugPrint(" Time= ", DEBUG_LEVEL_3));
  _DD(DebugPrint(String(GPS_GET_HR) + ":" + String(GPS_GET_MI) + ":" + String(GPS_GET_SE), DEBUG_LEVEL_3));
  _DD(DebugPrint(" Sat= ", DEBUG_LEVEL_3));
  _DD(DebugPrintln(GPS_GET_SAT, DEBUG_LEVEL_3));
}
#endif

#endif //GPS_MODULE
