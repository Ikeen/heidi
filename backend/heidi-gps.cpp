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

extern HardwareSerial heidiSerial1;
#define SerialGPS heidiSerial1
gpsData_t gpsData;
/*
 * https://youtu.be/ylxwOg2pXrc?list=RDCMUCTXOorupCLqqQifs2jbz7rQ
 */
#ifdef USE_CASIC_GPS
uint8_t GPS_MSG_UART_POLL[] =   { 0x00, 0x00, GPS_CASIC_CLASS_CFG, GPS_UBX_CFG_PRT, 0x00, 0x00, 0x06, 0x00};
#else
uint8_t GPS_MSG_UART_POLL[] =   { GPS_UBX_CLASS_CFG, GPS_UBX_CFG_PRT, 0x01, 0x00, 0x01 };
#endif

uint8_t CASIC_STATUS_POLL[] = { 0x04, 0x00, GPS_CASIC_CLASS_CFG, GPS_CASIC_CFG_MSG,
                                GPS_CASIC_CLASS_NAV, GPS_CASIC_NAV_STATUS, 0xff, 0xff };

uint8_t UBX_NAVX5_POLL[]=   { GPS_UBX_CLASS_CFG, GPS_UBX_CFG_NAVX5, 0x00, 0x00 };
uint8_t UBX_EPH_POLL[]    = { GPS_UBX_CLASS_AID, GPS_UBX_AID_EPH, 0x00, 0x00 };
uint8_t UBX_AOP_DT_POLL[] = { GPS_UBX_CLASS_AID, GPS_UBX_AID_AOP_DT, 0x00, 0x00 };
uint8_t UBX_AOP_ST_POLL[] = { GPS_UBX_CLASS_NAV, GPS_UBX_NAV_AOP_ST, 0x00, 0x00 };
uint8_t MSG_ON_OFF[]   = { GPS_UBX_CLASS_CFG, GPS_UBX_CFG_MSG,
                           0x08, 0x00, //pay load length 0x0008 (low, high)
                           GPS_CFG_MSG_NMEA, GPS_CFG_MSG_GGA,
                           GPS_CFG_OFF, //i2c
                           GPS_CFG_OFF, //uart1
                           GPS_CFG_OFF, //uart2
                           GPS_CFG_OFF, //USB
                           GPS_CFG_OFF, //SPI
                           0x00};       //unused

uint8_t UBX_SET_RATE[] = { GPS_UBX_CLASS_CFG, GPS_UBX_CFG_RATE,
                           0x06, 0x00,   //pay load length 0x0006 (low, high)
                           0xE8, 0x03,   //interval [ms] 1000 = 0x03E8 (low, high)
                           0x01, 0x00,   //must be 0x0001 (low, high)
                           0x01, 0x00 }; //should be 0x0001 (low, high)

/*
 * this function gets data sets until [sufficient_pDOP] is reached or [maxWorse] data sets had a less
 * accuracy then the current one (means: it seem to not getting better within a acceptable time)
 */
bool GPSGetPosition(t_SendData* DataSet, float sufficient_pDOP, int maxWorse, int timeOut){
  double acc_mn = -1.0;
  int curLoc  = 0;
  locationSet_t location[GPS_LOCATION_AVG_CNT];
  locationSet_t avrLocation = {0.0, 0.0, 0.0, 0.0, 0};
  int worse   = 0;
  int good    = 0;
  int altsum  = 0;
  int possum  = 0;
  uint32_t lastFixMS = 0;
  bool extendMeas = (heidiConfig->gpsStatus < GPS_GOT_3D_LOCK);

  setError(E_COULD_NOT_FETCH_GPS);
  setError(E_WRONG_GPS_VALUES);
  location[curLoc].pdop = REQUIRED_DOP_VALUE;
  for(int i=0; i < GPS_LOCATION_AVG_CNT; i++){
    location[i] = {0.0, 0.0, 0.0, REQUIRED_DOP_VALUE, 0};
  }
  _D(DebugPrintln("GPS: Acquire position", DEBUG_LEVEL_1));
  _DD(if(extendMeas){ DebugPrintln("GPS: extended measurement", DEBUG_LEVEL_1); })
  gpsData.valid = false;
  while((worse < maxWorse) && ((millis() < timeOut) || (timeOut == 0))){
    if (GPSprocessData(500))  {
      if (gpsData.valid) {
        if (GPS_IS_NAV_DATA) {
          if ((GPS_GOT_2D_FIX) || (GPS_GOT_3D_FIX)){
            if (GPS_GOT_2D_FIX) {
               heidiConfig->gpsStatus = GPS_GOT_2D_LOCK;
            }
            else if (GPS_GOT_3D_FIX){
              heidiConfig->gpsStatus = GPS_GOT_3D_LOCK;
            }
            if(GPS_GET_PDOP <= REQUIRED_DOP_VALUE) { good++; }
            if ((GPS_GET_PDOP <= (location[curLoc].pdop - 0.01))  || ((good < GPS_LOCATION_AVG_CNT) && (GPS_GET_PDOP <= REQUIRED_DOP_VALUE))){
              lastFixMS = millis();
              location[curLoc].gps_sat = GPS_GET_SAT;
              location[curLoc].lng   = GPS_GET_LON;
              location[curLoc].lat   = GPS_GET_LAT;
              if(GPS_GOT_2D_FIX){ location[curLoc].alt = 0.0; } else { location[curLoc].alt = GPS_GET_ALT; }
              location[curLoc].pdop  = GPS_GET_PDOP;
              _DD(DebugPrintln("GPS: [" + String(curLoc) + "] lat: " + String(location[curLoc].lat, 6) + ", lng: " + String(location[curLoc].lng, 6) +
                               ", sat: " + String(location[curLoc].gps_sat) +"  pDOP: " + String(location[curLoc].pdop, 4), DEBUG_LEVEL_3);)
              //find the next position to use
              for(int i=0; i < GPS_LOCATION_AVG_CNT; i++){ if(location[i].pdop > location[curLoc].pdop){curLoc = i;}}
            } else {
              if((!extendMeas) && (good >= GPS_LOCATION_AVG_CNT)){
                if(GPS_GET_PDOP < (location[curLoc].pdop + 2)){ //check for pDOP value leap
                  //no leap
                  worse++; _DD( DebugPrintln("pDOP: " + String(GPS_GET_PDOP, 1) + ", unUsedSets: " + String(worse),  DEBUG_LEVEL_3);)
                }
              } _DD( else { //else use full time to get ephemeris data
                 DebugPrintln("GPS: pDOP: " + String(GPS_GET_PDOP, 4) + ", good: " + String(good), DEBUG_LEVEL_3);
                })
            }
            if((location[curLoc].pdop <= sufficient_pDOP) && (good >= GPS_LOCATION_AVG_CNT) && (!extendMeas)) { RESET_GPS_DATA; break; }
          }
        }
        #ifndef USE_CASIC_GPS
        else if (gpsData.type == GPS_DT_NAV_STATUS) {
          if (gpsData.data->navStatus.gpsFix == 2) {
             heidiConfig->gpsStatus = GPS_GOT_2D_LOCK;
          }
          else if (gpsData.data->navStatus.gpsFix == 3){
            heidiConfig->gpsStatus = GPS_GOT_3D_LOCK;
          }
          /*
          _DD(
            DebugPrintln("GPS NAVstatus psmState: " + String(gpsData.data->navStatus.flags2 & 0x03)
                     + ", mapMatch: " + String((gpsData.data->navStatus.fixStat >> 6) & 0x03)
                     + ", flags: " + String(gpsData.data->navStatus.flags & 0x0f, HEX)
                     + ", gpsFix: " + String(gpsData.data->navStatus.gpsFix)
                        ,DEBUG_LEVEL_3);
          )
          */
        }
        #ifdef USE_AOP_STATUS
        else if (gpsData.type == GPS_DT_NAV_AOP_STATUS) {
          _DD(DebugPrintln("GPS AOPstatus: " + String(gpsData.data->navAOPstatus.status), DEBUG_LEVEL_3);)
        }
        #endif
        #endif //USE_CASIC_GPS
      }
      RESET_GPS_DATA;
    }
    #ifdef DEBUG_SERIAL_GPS
    else {_DD(DebugPrint(".", DEBUG_LEVEL_3);) }
    #endif
    if((lastFixMS > 0) && (!extendMeas)){
      if((millis() - lastFixMS) > GPS_MAX_SILENCE_MS){ RESET_GPS_DATA; break; }
    }
  }
  #ifdef DEBUG_SERIAL_GPS
  _DD(DebugPrintln("", DEBUG_LEVEL_3));
  #endif
  for(int i=0; i < GPS_LOCATION_AVG_CNT; i++){
    if(location[i].gps_sat > 0){
      avrLocation.lat  += location[i].lat;
      avrLocation.lng  += location[i].lng;
      avrLocation.pdop += location[i].pdop;
      avrLocation.gps_sat += location[i].gps_sat;
      possum++;
      if(location[i].alt > 0.0){
        avrLocation.alt  += location[i].alt;
        altsum++;
      }
    }
  }
  if(possum > 0){ avrLocation.lat  /= possum; } else { avrLocation.lat = 0.0; }
  if(possum > 0){ avrLocation.lng  /= possum; } else { avrLocation.lng = 0.0; }
  if(possum > 0){ avrLocation.pdop /= possum; } else { avrLocation.pdop = 0.0; }
  if(possum > 0){ avrLocation.gps_sat = round(avrLocation.gps_sat / possum); } else { avrLocation.gps_sat = 0; }
  if(altsum > 0){ avrLocation.alt  /= altsum; } else { avrLocation.alt = 0.0; }
  if(avrLocation.gps_sat > 0){
    _D(DebugPrintln("GPS: Lat: " + String(avrLocation.lat,6) + ", Lon: " + String(avrLocation.lng, 6)
                     + ", DOP: " + String(avrLocation.pdop,1), DEBUG_LEVEL_1);)
    rmError(E_COULD_NOT_FETCH_GPS);
    DataSet->latitude = GeoToInt(avrLocation.lat);
    DataSet->longitude = GeoToInt(avrLocation.lng);
    DataSet->altitude = (int16_t)avrLocation.alt;
    if(avrLocation.pdop > 25.5) { avrLocation.pdop = 25.5; }
    DataSet->GPSpDOP  = (uint8_t)round(avrLocation.pdop*10);
    DataSet->satellites = (uint8_t)avrLocation.gps_sat;
    if (!((DataSet->latitude == 0) && (DataSet->longitude == 0))){ //0.0, 0.0 must be wrong (or a fish)
      rmError(E_WRONG_GPS_VALUES);
    } else { return false; }
    DataSet->metersOut = 0;
    if(!(getError(E_COULD_NOT_FETCH_GPS) || getError(E_WRONG_GPS_VALUES) || invalidFence())){
      t_PointData position;
      position.latitude = avrLocation.lat;
      position.longitude = avrLocation.lng;
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
  }
  _D(DebugPrintln("GPS done: " + String((int)(millis() / 1000)) + " s", DEBUG_LEVEL_2));
  if (millis() >= ONE_MINUTE) { setError(E_GPS_LATE_LOCK); }
  return (avrLocation.gps_sat > 0);
}
bool setBootTimeFromGPSTime(tm* bootTime, int timeOut)
{
  tm GPStime;
  setError(E_COULD_NOT_FETCH_GPS_TIME);
  while(SerialGPS.available() > 0){ SerialGPS.read(); }
  GPStime.tm_year = 0;
  GPStime.tm_mon  = 0;
  GPStime.tm_mday = 0;
  while(((GPStime.tm_year < 2000) || (GPStime.tm_mon <= 0) || (GPStime.tm_mday <= 0))
        && ((millis() < timeOut) || (timeOut == 0))){
    #ifdef USE_CASIC_GPS
    if(GPSwaitForMessage(GPS_DT_CASIC_NAV_TIMEUTC, 1000) == GPS_WAIT_MSG_OK){
    #else
    if(GPSwaitForMessage(GPS_DT_NAV_PVT, 1000) == GPS_WAIT_MSG_OK){
    #endif
      if (GPS_TIM_VALID) {
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
          _DD(DebugPrintln("GPS time: " + String(GPStime.tm_year) + "-" + LenTwo(String(GPStime.tm_mon + 1)) + "-" + LenTwo(String(GPStime.tm_mday))
              + " " + LenTwo(String(GPStime.tm_hour)) +  ":"  + LenTwo(String(GPStime.tm_min)) + ":" + LenTwo(String(GPStime.tm_sec)), DEBUG_LEVEL_3);)
          GPStime.tm_year = 1900;
          return true;
        }
	    }
    }
    #ifdef DEBUG_SERIAL_GPS
    else {_DD(DebugPrint(".", DEBUG_LEVEL_3);) }
    #endif
  }
  #ifdef DEBUG_SERIAL_GPS
  _DD(DebugPrintln("", DEBUG_LEVEL_3));
  #endif
  return false;
}


bool openGPS(){
  if (GSMenabled){
    _D(DebugPrintln("GSM still enabled - cannot open GPS", DEBUG_LEVEL_1); pause(50);)
    return false;
  }
  if (!GPSenabled){
    if (!enableMeasures()){
      _D(DebugPrintln("enable GPS power failed", DEBUG_LEVEL_1); pause(50);)
      return false;
    }
    GPSenabled = true;
    if (!GPSsetupUART(GPS_HIGH_BAUD)) {return false; }
    #ifndef USE_CASIC_GPS
    if (!GPSenableAOP())  {return false; }
    #endif

    if (GPSEphermerisDataStatus() >= GPS_MIN_EPH_DATA){
      if (heidiConfig->gpsStatus < GPS_GOT_EHP_DATA) { heidiConfig->gpsStatus = GPS_GOT_EHP_DATA; }
    } else { heidiConfig->gpsStatus = GPS_LESS_EHP_DATA; }
    #ifdef SAVE_AOP_DATA
    if ((GPSuploadAOPdata() >= GPS_MIN_AOP_DATA) && (heidiConfig->gpsStatus < GPS_GOT_AOP_DATA)) {
      heidiConfig->gpsStatus = GPS_GOT_AOP_DATA;
    }
    #endif

    GPSsetupMessages();

#ifdef USE_CASIC_GPS
return true;
#endif

    _D(DebugPrintln("GPS: opened successfully", DEBUG_LEVEL_2));
  } _D(else { DebugPrintln("GPS: was already open", DEBUG_LEVEL_1); pause(50); })
  return true;
}
void closeGPS(){
  if (GPSenabled){
    #ifdef SAVE_AOP_DATA
    GPSsaveAOPdata();
    #endif
    disableMeasures();
    SerialGPS.flush();
    ////never use:  SerialGPS.end(); it may cause ESP32 to crash
    _D(DebugPrintln("GPS closed", DEBUG_LEVEL_2));
  } _D(else { DebugPrintln("GPS already closed", DEBUG_LEVEL_2); })
  GPSenabled = false;
}
#define CASIC_PROTO_MASK (GPS_CASIC_PROTO_IN_BIN | GPS_CASIC_PROTO_OUT_BIN)

/*
 * switch SerialGPS to new baud rate and switch off NMEA message
 *   - newBaudRate -> new baud rate
 *   - returns true if successful
 */
bool GPSsetupUART(int newBaudRate){
  int i = 0;
  uint32_t oldBaudRate = GPS_DEFAULT_BAUD;
  //we start with default
  setGPSserialBaudRate(GPS_DEFAULT_BAUD, true);
  #ifdef USE_CASIC_GPS
  //CASIC devices ignores setting of protoMask to switch off txt messages
  GPSrawWriteString("$PCAS03,0,0,0,0,0,0,0,0*02");
  #endif
  //check current configuration
  GPSsendMessage(GPS_MSG_UART_POLL, sizeof(GPS_MSG_UART_POLL));
  while(i < 3){ // tries
    //check if it's done:
    gpsWaitMsgRc_t waitRc = GPSwaitForMessage(GPS_DT_CFG_PRT, 1000);
    if(waitRc == GPS_WAIT_MSG_OK){
      #ifdef USE_CASIC_GPS
      if((gpsData.data->CAScfgPRT.protoMask == CASIC_PROTO_MASK) && (gpsData.data->CAScfgPRT.baudRate == newBaudRate)){
      #else
      if((gpsData.data->UARTconf.outProtoMask == GPS_UBX_PROTO_UBX) && (gpsData.data->UARTconf.baud == newBaudRate)){
      #endif
        _DD(DebugPrintln("GPS: setup UART successfully [" + String(i) + (i==1?" try]":" tries]"), DEBUG_LEVEL_3));
        return true;
      } else {
        /*
         * setting baud rate may lead into a deadlock if the message fails on the module and we switch
         * baud rate on host. 2 to 3 tries with alternating baud rate (new <-> old) should avoid that.
         * however high baud rates are not needed if we not intend to transfer huge amounts of data
         * e.g. EPHdata or AOPData (but currently we did)
         */
        #ifdef USE_CASIC_GPS
        if(i == 1){ oldBaudRate = gpsData.data->CAScfgPRT.baudRate; }
        gpsData.data->CAScfgPRT.baudRate = newBaudRate;
        gpsData.data->CAScfgPRT.protoMask = CASIC_PROTO_MASK;
        GPSsendMessage((uint8_t*)gpsData.data, sizeof(CCFG_PRT_t));
        waitRc = GPSwaitForMessage(GPS_DT_CFG_PRT, 1000, true);
        if (waitRc == GPS_WAIT_MSG_OK){ setGPSserialBaudRate(newBaudRate); };
        #else
        if(i == 1){ oldBaudRate = gpsData.data->UARTconf.baud; }
        gpsData.data->UARTconf.baud = newBaudRate;
        gpsData.data->UARTconf.outProtoMask = GPS_UBX_PROTO_UBX;
        GPSsendMessage((uint8_t*)gpsData.data, sizeof(UART_CONFIG_t)); //ublox will not acknowledge settings
        //NEO7M will (for whatever reason) not acknowledge this message
        setGPSserialBaudRate(newBaudRate);
        #endif
      }
    } else {
      //got no data - something went wrong. Now we try old and new baud rate in alternation
      _D(DebugPrintln("GPS: setup UART no data - alternate baud rate", DEBUG_LEVEL_1));
      if((i % 2) == 0){ setGPSserialBaudRate(oldBaudRate); }
      else { setGPSserialBaudRate(newBaudRate); }
    }
    GPSsendMessage(GPS_MSG_UART_POLL, sizeof(GPS_MSG_UART_POLL));
    i++;
  }
  _D(DebugPrintln("GPS: setup UART failed", DEBUG_LEVEL_1));
  return false;
}

bool GPSsetupMessages(void){
  #ifdef USE_CASIC_GPS
  CCFG_MSG_t msg;
  CCFG_RATE_t rmsg;
  #else
  CFG_MSG_t msg;
  msg.i2cRate = 0;
  msg.otherPorts = 0;
  CFG_RATE_t rmsg;
  rmsg.navRate = 1;
  rmsg.timeRef = 1;
  #endif
  uint8_t* pMsg = (uint8_t*)&msg;
  msg.cls = GPS_CLASS_CFG;
  msg.id  = GPS_CFG_MSG;
  msg.len = sizeof(msg)-GPS_MSG_HEAD_LEN;
  msg.clsID = GPS_CLASS_NAV;
  #ifdef USE_CASIC_GPS
  msg.msgID = GPS_CASIC_NAV_PV;
  #else
  msg.msgID = GPS_UBX_NAV_PVT;
  #endif
  msg.rate = 1;
  GPSsendMessage(pMsg, sizeof(msg));
  #ifdef USE_CASIC_GPS
  GPSwaitForMessage(GPS_DT_CFG_MSG, 200, true);
  #endif

  #ifdef USE_CASIC_GPS
  msg.msgID = GPS_CASIC_NAV_DOP;
  #else
  msg.msgID = GPS_UBX_NAV_STATUS;
  #endif
  msg.rate = 4;
  GPSsendMessage(pMsg, sizeof(msg));
  #ifdef USE_CASIC_GPS
  GPSwaitForMessage(GPS_DT_CFG_MSG, 200, true);
  #endif

  #ifdef USE_CASIC_GPS
  msg.msgID = GPS_CASIC_NAV_TIMEUTC;
  #else
  msg.msgID = GPS_UBX_NAV_AOP_ST;
  #endif
  msg.rate = 2;
  GPSsendMessage(pMsg, sizeof(msg));
  #ifdef USE_CASIC_GPS
  GPSwaitForMessage(GPS_DT_CFG_MSG, 200, true);
  #endif

  uint8_t* pRmsg = (uint8_t*)&rmsg;
  rmsg.cls = GPS_CLASS_CFG;
  #ifdef USE_CASIC_GPS
  rmsg.id  = GPS_CASIC_CFG_RATE;
  #else
  rmsg.id  = GPS_UBX_CFG_RATE;
  #endif
  rmsg.len = sizeof(rmsg)-GPS_MSG_HEAD_LEN;
  rmsg.period_ms = 500;
  GPSsendMessage(pRmsg, sizeof(rmsg));
  #ifdef USE_CASIC_GPS
  GPSwaitForMessage(GPS_DT_CFG_MSG, 200, true);
  #endif
  return true;

  //enable UBX NAV PVT - message on UART1
  //MSG_ON_OFF[MSG_ON_OFF_GROUP] = GPS_CFG_MSG_UBX;
  //MSG_ON_OFF[GPS_CFG_SER_CH_UART1]  = GPS_CFG_ON;
  //MSG_ON_OFF[MSG_ON_OFF_TYPE]  = GPS_UBX_NAV_PVT;
  //GPSsendMessage(MSG_ON_OFF, sizeof(MSG_ON_OFF));
  //enable UBX NAV STATUS - message on UART1
  //MSG_ON_OFF[MSG_ON_OFF_TYPE]  = GPS_UBX_NAV_STATUS;
  //GPSsendMessage(MSG_ON_OFF, sizeof(MSG_ON_OFF));
  //MSG_ON_OFF[MSG_ON_OFF_TYPE]  = GPS_UBX_NAV_AOP_ST;
  //GPSsendMessage(MSG_ON_OFF, sizeof(MSG_ON_OFF));
  //set GPS measure rate
  //*(uint16_t*)&UBX_SET_RATE[4] = 0x01F4; //500
  //GPSsendMessage(UBX_SET_RATE, sizeof(UBX_SET_RATE));
  //return true;
}

/*
 * waits for a GPS message and stores the result in gpsData struct (consider valid-flag)
 *   - type - which kind of message
 *   - timeout - timeout in ms
 *   - check_ack - CASIC cfg messages will be answered by a acknowledge message
 *                 which must not be ignored, defaults to false
 */
gpsWaitMsgRc_t GPSwaitForMessage(gpsDataType_t type, int timeout, bool just_check_ack){
  int t = millis();
  uint8_t buffer[sizeof(GPSMessage_t)];
  int     bufferlen;
  bool    waitForAck = just_check_ack;
  RESET_GPS_DATA;
  /*_DD(
      if (just_check_ack) { DebugPrintln("GPS: just wait for acknowledge for package " + String(type), DEBUG_LEVEL_3); }
      else { DebugPrintln("GPS: wait for package " + String(type), DEBUG_LEVEL_3); }
  )*/
  while(millis() - t < timeout){
    if (GPSprocessData(200)) {
      //_DD(DebugPrintln("GPS : got message " + String(gpsData.type) + ", len:" + String(gpsData.data->head.len), DEBUG_LEVEL_3);)
      if ((gpsData.type == type) && (!waitForAck)) {
        if(GPSmsgNeedAck(type)){
          uint8_t* data = (uint8_t*)gpsData.data;
          bufferlen = gpsData.data->head.len + GPS_MSG_HEAD_LEN;
          for(int i= 0; i<bufferlen; i++){ buffer[i] = data[i]; }
          RESET_GPS_DATA;
          waitForAck = true;
          //_DD(DebugPrintln("GPS: package " + String(type) + " need acknowledge", DEBUG_LEVEL_3);)
        } else { return GPS_WAIT_MSG_OK; }
      }
      else if ((waitForAck) && (gpsData.type == GPS_DT_ACK)) {
        gpsDataType_t ackType = GPSgetPackageType(gpsData.data->GPS_MSG_ACK.clsID, gpsData.data->GPS_MSG_ACK.msgID, GPS_MSG_IGNORE_LEN);
        if(ackType == type){
          //_DD(DebugPrintln("GPS: package " + String(type) + " acknowledged", DEBUG_LEVEL_3);)
          if (just_check_ack){
            gpsData.valid = false;
          } else {
            uint8_t* data = (uint8_t*)gpsData.data;
            for(int i= 0; i<bufferlen; i++){ data[i] = buffer[i]; }
            gpsData.type = type;
            gpsData.valid = true;
          }
          return GPS_WAIT_MSG_OK;
        }
      }
      else if (gpsData.type == GPS_DT_NACK) {
        gpsDataType_t ackType = GPSgetPackageType(gpsData.data->GPS_MSG_ACK.clsID, gpsData.data->GPS_MSG_ACK.msgID, GPS_MSG_IGNORE_LEN);
        if(ackType == type){
          _DD(DebugPrintln("GPS: package " + String(type) + " not acknowledged", DEBUG_LEVEL_3);)
          gpsData.valid = false;
          return GPS_WAIT_MSG_NACK;
        }
      }
      else { RESET_GPS_DATA; }
    }
  }
  gpsData.valid = false;
  //_DD(DebugPrintln("GPS: package " + String(type) + " timed out", DEBUG_LEVEL_3);)
  return GPS_WAIT_MSG_TIMEOUT;
}
/*
 * GPSprocessData: returns true if valid data is in buffer
 * keep in mind: GPS module will continuously send data. Processing it is a rolling process
 * that means, it could happen that GPSprocessData must be called twice until data is valid.
 * so it makes no sense to invalidate data here at the beginning.
 * Don't forget to invalidate gpsData before processing new, otherwise GPSprocessData may
 * return successful with old data in case no new data was sent within timeout period
 */
bool GPSprocessData(int timeOut){
  int t = millis();
  while ((millis()-t) < timeOut){
    if (SerialGPS.available() > 0){
      #ifdef DEBUG_SERIAL_GPS
      static int i=0;
      uint8_t r = SerialGPS.read();
      _DD(Serial.print(LenTwo(String(r,HEX)) + " ");)
      _DD(i++; if(i >= 32){Serial.println (""); i = 0; })
      /*
      _DD(Serial.print(char(r));)
      _DD(i++; if(i >= 46){Serial.println (""); i = 0; })
      */
      if (GPSdecodeData(r)) { _DD(DebugPrintln("", DEBUG_LEVEL_3); DebugPrintln("", DEBUG_LEVEL_3);) return true; }
      #else
       if (GPSdecodeData(SerialGPS.read())){ return true; }
      #endif
    } else { pause(50); }
  }
  return false;
}
/*
 * GPSdecodeData returns true if a data set was completely and successful decoded
 * keep in mind: GPS module will continuously send date and processing it is a rolling process
 */
bool GPSdecodeData(uint8_t data) {
  static int fpos = 0;
  static uint8_t checksumSent[GPS_MSG_CK_LEN];
  static GPSMessage_t msgBuffer;
  static int payloadSize;
  static uint8_t* buffer = (uint8_t*)&msgBuffer;

  //wait for sync bytes
  if ( fpos < GPS_MSG_SYNC_LEN ) {  if ( data == GPS_MSG_SYNC[fpos] ){  fpos++; } else { fpos = 0; } return false; }
  //header found, load id & length (now we write new data into structure, means: old data is invalid)
  gpsData.valid = false;
  //load header
  if ( fpos < (GPS_MSG_SYNC_LEN + GPS_MSG_HEAD_LEN - 1)) { buffer[fpos-GPS_MSG_SYNC_LEN] = data; fpos++; return false; }
  //got header
  if ( fpos == (GPS_MSG_SYNC_LEN + GPS_MSG_HEAD_LEN - 1)) { //check header information
    buffer[fpos-GPS_MSG_SYNC_LEN] = data;
    payloadSize = msgBuffer.head.len;
    //msg length = payload length + GPS_MSG_HEADER length should be <= buffer length
    if ((payloadSize + GPS_MSG_HEAD_LEN) <= sizeof(GPSMessage_t)){
      fpos++;
      gpsData.type = GPSgetPackageType(msgBuffer.head.cls, msgBuffer.head.id, msgBuffer.head.len);
      if(gpsData.type == GPS_DT_WRONG_LEN){ fpos = 0; }
    } else { fpos = 0;} //maybe unknown package with over length or transmission failure on length parameter
    return false;
  }
  //load payload
  if ( fpos < (payloadSize + GPS_MSG_SYNC_LEN + GPS_MSG_HEAD_LEN)){
    if (gpsData.type != GPS_DT_NONE){ //if data type is known then load
      buffer[fpos-2] = data;
    } //otherwise crush it
    fpos++;
    return false;
  }
  //load check sum
  if (fpos < (payloadSize + GPS_MSG_SYNC_LEN + GPS_MSG_HEAD_LEN + GPS_MSG_CK_LEN - 1)) {
    checksumSent[fpos-(payloadSize + GPS_MSG_SYNC_LEN + GPS_MSG_HEAD_LEN)] = data; fpos++; return false;
  }
  //got check sum
  if (fpos == (payloadSize + GPS_MSG_SYNC_LEN + GPS_MSG_HEAD_LEN + GPS_MSG_CK_LEN - 1)) {
    uint8_t checksumCalc[GPS_MSG_CK_LEN];
    checksumSent[GPS_MSG_CK_LEN - 1] = data;
    fpos = 0;
    GPScalcChecksum(buffer, checksumCalc);
    #ifdef USE_CASIC_GPS
    if(*(uint32_t*)checksumCalc == *(uint32_t*)checksumSent){
    #else
    if((checksumCalc[0]==checksumSent[0]) && (checksumCalc[1]==checksumSent[1])){
    #endif
      gpsData.data  = &msgBuffer;
      gpsData.valid = true;
      //if you intend to fetch cyclic transmitted data, this is the point to copy it into a dedicated place
      //otherwise it would be overwritten by the next packet
      return true;
    } //_DD( else {DebugPrintln("GPS: package wrong CK: " + String(*(uint32_t*)checksumCalc, HEX) + " / " + String(*(uint32_t*)checksumSent, HEX), DEBUG_LEVEL_3);})
  }
  //we should never come here
  gpsData.valid = false;
  gpsData.type  = GPS_DT_NONE;
  fpos = 0;
  return false;
}
/*
 * returns package type / checks length
 *   - cls - class id
 *   - id  - message id
 *   - len - received length
 *
 */
gpsDataType_t GPSgetPackageType(uint8_t cls, uint8_t id, uint16_t len){
  gpsDataType_t result = GPS_DT_NONE;
  int lenExp = 0;
  bool orLess = false;
  #ifdef USE_CASIC_GPS
  if(cls == GPS_CASIC_CLASS_NAV){
    if(id == GPS_CASIC_NAV_PV){
      lenExp = sizeof(CNAV_PV_t)-GPS_MSG_HEAD_LEN;
      result = GPS_DT_CASIC_NAV_PV;
    }
    if(id == GPS_CASIC_NAV_TIMEUTC){
      lenExp = sizeof(CNAV_TIMEUTC_t)-GPS_MSG_HEAD_LEN;
      result = GPS_DT_CASIC_NAV_TIMEUTC;
    }
    if(id == GPS_CASIC_NAV_STATUS){
      lenExp = sizeof(CNAV_STATUS_t)-GPS_MSG_HEAD_LEN;
      result = GPS_DT_CASIC_NAV_STATUS;
    }
    if(id == GPS_CASIC_NAV_DOP){
      lenExp = sizeof(CNAV_STATUS_t)-GPS_MSG_HEAD_LEN;
      result =GPS_DT_CASIC_NAV_DOP;
    }
  }
  if(cls == GPS_CASIC_CLASS_CFG){
    if(id == GPS_CASIC_CFG_PRT){
      lenExp = sizeof(CCFG_PRT_t)-GPS_MSG_HEAD_LEN;
      result = GPS_DT_CFG_PRT;
    }
    if(id == GPS_CASIC_CFG_POLLMSG){
      lenExp = sizeof(CCFG_POLLMSG_t)-GPS_MSG_HEAD_LEN;
      result = GPS_DT_CFG_POLLMSG;
    }
    if(id == GPS_CASIC_CFG_MSG){
      lenExp = sizeof(CCFG_MSG_t)-GPS_MSG_HEAD_LEN;
      result = GPS_DT_CFG_MSG;
    }
    if(id == GPS_CASIC_CFG_RATE){
      lenExp = sizeof(CCFG_RATE_t)-GPS_MSG_HEAD_LEN;
      result = GPS_DT_CFG_RATE;
    }
  }
  #else
  if(cls == GPS_UBX_CLASS_NAV){
    if(id == GPS_UBX_NAV_PVT){
      lenExp = sizeof(NAV_PVT_t)-GPS_MSG_HEAD_LEN;
      result = GPS_DT_NAV_PVT;
    }
    if(id == GPS_UBX_NAV_STATUS){
      lenExp = sizeof(NAV_STATUS_t)-GPS_MSG_HEAD_LEN;
      result = GPS_DT_NAV_STATUS;
    }
  }
  if(cls == GPS_UBX_CLASS_CFG){  // == GPS_CASIC_CLASS_CFG
    if(id == GPS_UBX_CFG_PRT){
      lenExp = sizeof(UART_CONFIG_t)-GPS_MSG_HEAD_LEN;
      result = GPS_DT_CFG_PRT;
    }
    if(id == GPS_UBX_CFG_NAVX5){
      lenExp = sizeof(NAVX5_CONFIG_t)-GPS_MSG_HEAD_LEN;
      result = GPS_DT_CFG_NAVX5;
    }
    if(id == GPS_UBX_CFG_RATE){
      lenExp = sizeof(CFG_RATE_t)-GPS_MSG_HEAD_LEN;
      result = GPS_DT_CFG_RATE;
    }

  }
  if(cls == GPS_UBX_CLASS_AID){
    if(id == GPS_UBX_AID_EPH){
      lenExp = sizeof(AID_EPH_DATA_t)-GPS_MSG_HEAD_LEN;
      orLess = true;
      result = GPS_DT_AID_EPH;
    }
    if(id == GPS_UBX_AID_AOP_DT){
      lenExp = sizeof(AID_AOP_DATA_t)-GPS_MSG_HEAD_LEN;
      orLess = true;
      result = GPS_DT_AID_AOP_DATA;
    }
    if(id == GPS_UBX_NAV_AOP_ST){
      lenExp = sizeof(NAV_AOPSTATUS_t)-GPS_MSG_HEAD_LEN;
      result = GPS_DT_NAV_AOP_STATUS;
    }
  }
  #endif
  if(cls == GPS_CLASS_ACK){
    lenExp = sizeof(GPS_MSG_ACK_T)-GPS_MSG_HEAD_LEN;
    if(id == GPS_ACK_NACK){
      result = GPS_DT_NACK;
    }
    if(id == GPS_ACK_ACK){
      result = GPS_DT_ACK;
    }
  }
  if((len || GPS_MSG_IGNORE_LEN) || (len != lenExp) || (orLess && (len < lenExp))|| (result == GPS_DT_NONE)) {
    return result;
  }
  _D(DebugPrintln("GPS : got message with wrong length" + String(id) + ", len:" + String(len) + ", should be:" + String(lenExp), DEBUG_LEVEL_1);)
  return GPS_DT_WRONG_LEN;
}

#ifdef USE_CASIC_GPS
/*
 * calculates the CASIC-variant of checksum
 *   - buffer - points to payload, including header with class, id and length
 *   - CK - points to buffer (32 bit) for storing the checksum
 */
void GPScalcChecksum(uint8_t* buffer, uint8_t* CK) {
  uint32_t* ckSum = (uint32_t*)CK;
  uint32_t* pload = (uint32_t*)buffer;
  MSG_HEAD_t* head = (MSG_HEAD_t*)buffer;
  int len = head->len;
  int l = len >> 2;
  *ckSum = (head->id << 24) + (head->cls << 16) + len;
  for (int i = 0; i < l; i++){
    *ckSum += pload[i+1];
  }
}
#else
/*
 * calculates the UBX-variant of checksum
 *   - buffer - points to payload, including header with class, id and length
 *   - CK - points to buffer (16 bit) for storing the checksum
 */
void GPScalcChecksum(uint8_t* buffer, uint8_t* CK) {
  memset(CK, 0, 2);
  int len = buffer[2] + (buffer[3] << 8) + 4;
  for (int i = 0; i < len; i++) {
    CK[0] += buffer[i];
    CK[1] += CK[0];
  }
}
#endif

void GPSsendMessage(uint8_t* message, int len){
  uint8_t fullmessage[(sizeof(GPSMessage_t) + 2 + GPS_MSG_CK_LEN)];
  if(fullmessage == NULL) { return; }
  fullmessage[0] = GPS_MSG_SYNC[0];
  fullmessage[1] = GPS_MSG_SYNC[1];
  int l = len;
  for(int i=0; i<l; i++){ fullmessage[i+2] = message[i]; }
  GPScalcChecksum(message, &fullmessage[l+2]);
  l += (2 + GPS_MSG_CK_LEN);  //plus header bytes and checksum
  #ifdef DEBUG_SERIAL_GPS
  _DD(DebugPrint("GPS send: ", DEBUG_LEVEL_3);
    for(int i=0; i<l; i++){ DebugPrint(LenTwo(String(fullmessage[i], HEX)) + " ", DEBUG_LEVEL_3); }
    DebugPrintln("", DEBUG_LEVEL_3);
  )
  #endif
  for(int i=0; i<l; i++){
    SerialGPS.write(fullmessage[i]);
    #ifndef USE_CASIC_GPS
    pause(1); //this is, for whatever reason, necessary. NEO7M will not understand messages sent without breaks
    #endif
  }
  #ifndef USE_CASIC_GPS
  pause(50); // setup time (see above)
  #endif
}
#ifdef USE_CASIC_GPS
void GPSrawWriteString(String message){
  #ifdef DEBUG_SERIAL_GPS
  _DD(DebugPrintln("GPS send: " + message, DEBUG_LEVEL_3);)
  #endif
  for(int i=0; i<message.length(); i++){
    SerialGPS.write(message.charAt(i));
  }
  SerialGPS.write(0x0d);
  SerialGPS.write(0x0a);
  pause(50); // setup time (see above)
}
#endif
#ifdef HEIDI_CONFIG_TEST
void GPSrawWriteMessage(uint8_t* message, int len){
  #ifdef DEBUG_SERIAL_GPS
  _DD(DebugPrint("GPS send: ", DEBUG_LEVEL_3);
    for(int i=0; i<len; i++){ DebugPrint(LenTwo(String(message[i], HEX)) + " ", DEBUG_LEVEL_3); }
    DebugPrintln("", DEBUG_LEVEL_3);
  )
  #endif
  for(int i=0; i<len; i++){
    SerialGPS.write(message[i]);
    pause(1); //this is, for whatever reason, necessary. NEO7M will not understand messages sent without breaks
  }
  pause(50); // setup time (see above)
}
#endif

void setGPSserialBaudRate(uint32_t rate, bool initialSetup){
  if (!openUart(GPS_UART_NO, rate)){_D(DebugPrintln("GPS: could not open UART", DEBUG_LEVEL_1);) return; }
  if (initialSetup){
    #ifdef DEBUG_SERIAL_GPS
    GPSprintBootData();
    #else
    pause(500); //let it print all boot stuff..
    #endif
  } else {
    pause(100); //wait for all the rubbish
  }
  SerialGPS.flush();
}

bool GPSmsgNeedAck(gpsDataType_t type){
  if ((type == GPS_DT_CFG_PRT) || (type == GPS_DT_CFG_NAVX5) || (type == GPS_DT_CFG_POLLMSG) || (type == GPS_DT_CFG_MSG) || (type == GPS_DT_CFG_RATE))
  { return true; }
  return false;
}


uint8_t GPSEphermerisDataStatus(void){
  int t = millis(), i = 0;
  uint8_t x = 0;
  _DD(DebugPrint("EPH: ", DEBUG_LEVEL_3);)
  #ifdef USE_CASIC_GPS
  GPSsendMessage(CASIC_STATUS_POLL, sizeof(CASIC_STATUS_POLL));
  if (GPSwaitForMessage(GPS_DT_CFG_MSG, 200, true) == GPS_WAIT_MSG_OK){
    if (GPSwaitForMessage(GPS_DT_CASIC_NAV_STATUS, 2000) == GPS_WAIT_MSG_OK){
      for(i=0; i<32; i++){
        uint8_t val = gpsData.data->CASnavStatus.ephGPS[i] & 0x03;
        if (val == 3) { x++; }
        _DD(DebugPrint(String(val)+ ", ", DEBUG_LEVEL_3);)
      }
    }
  }
  #else
  GPSsendMessage(UBX_EPH_POLL, sizeof(UBX_EPH_POLL));
  do {
    if(GPSwaitForMessage(GPS_DT_AID_EPH, 200) == GPS_WAIT_MSG_OK){
      _DD(DebugPrint(String(gpsData.data->AIDephData.len)+ ", ", DEBUG_LEVEL_3);)
      if(gpsData.data->AIDephData.len > 8){ x++; }
      i++;
    }
  }while((i<32) && (millis()-t < 2000));
  #endif
  _DD(DebugPrintln("[" + String(x) + "]", DEBUG_LEVEL_3);)
  return x;
}
#ifndef USE_CASIC_GPS
bool GPSenableAOP(void){
  int i = 0;
  GPSsendMessage(UBX_NAVX5_POLL, sizeof(UBX_NAVX5_POLL));
  while(i < 2){ // tries
    //check if it's done:
    if(GPSwaitForMessage(GPS_DT_CFG_NAVX5, 1000) == GPS_WAIT_MSG_OK){
      if(gpsData.data->NAVX5conf.aopCfg  == 0x01){
        _DD(DebugPrintln("GPS: auto assist mode enabled successfully [" + String(i) + (i==1?" try]":" tries]"), DEBUG_LEVEL_3);)
        return true;
      } else {
        gpsData.data->NAVX5conf.aopCfg  = 0x01;
        GPSsendMessage((uint8_t*)gpsData.data, sizeof(NAVX5_CONFIG_t));
      }
    }
    GPSsendMessage(UBX_NAVX5_POLL, sizeof(UBX_NAVX5_POLL));
    i++;
  }
  _D(DebugPrintln("GPS: enabling auto assist mode failed", DEBUG_LEVEL_1));
  return false;
}
#ifdef SAVE_AOP_DATA
uint8_t GPSAOPdataStatus(void){
  int t = millis(), i = 0;
  uint8_t x = 0;
  GPSsendMessage(UBX_AOP_DT_POLL, sizeof(UBX_AOP_DT_POLL));
  _DD(DebugPrint("AOP: ", DEBUG_LEVEL_3);)
  do {
    if(GPSwaitForMessage(GPS_DT_AID_AOP_DATA, 200) == GPS_WAIT_MSG_OK){
     _DD(DebugPrint(String(gpsData.data->AIDaopData.len)+ ", ", DEBUG_LEVEL_3);)
     if(gpsData.data->AIDaopData.len > 1){ x++; }
     i++;
    }
  }while((i<32) && (millis()-t < 5000));
  _DD(DebugPrintln(", [" + String(x) + "]", DEBUG_LEVEL_3);)
  return x;
}
void GPSsaveAOPdata(void){
  int t = millis(), k = 0;
  uint8_t x = 0;
  for (int x = 0; x < AOP_DATA_SETS; x++){ aopDataSet[x]->svId = 0; }
  GPSsendMessage(UBX_AOP_DT_POLL, sizeof(UBX_EPH_POLL));
  _DD(DebugPrint("AOP save data for svId: ", DEBUG_LEVEL_3);)
  do {
    if(GPSwaitForMessage(GPS_DT_AID_AOP_DATA, 300) == GPS_WAIT_MSG_OK){
      //_DD(DebugPrint("[" + String(gpsData.data->AIDaopData.svId) + "," + String(gpsData.data->AIDaopData.len) + "] ", DEBUG_LEVEL_3);)
      if((gpsData.data->AIDaopData.len >= 60) && (gpsData.data->AIDaopData.svId > 0)){
        aopDataSet[x]->svId = gpsData.data->AIDaopData.svId;
        for (int i=0; i<59; i++) { aopDataSet[x]->data[i] = gpsData.data->AIDaopData.data[i]; }
        _DD(DebugPrint(String(aopDataSet[x]->svId) + " ", DEBUG_LEVEL_3);)
        x++;
      }
      k++;
    }
  }while((k < AOP_DATA_SETS) && (millis()-t < 3000));
  _DD(DebugPrintln(", [" + String(x) + "]", DEBUG_LEVEL_3);)
}

int GPSuploadAOPdata(void){
  int t = millis(), k = 0;
  AID_AOP_DATA_t message;
  message.cls = GPS_UBX_CLASS_AID;
  message.id = GPS_UBX_AID_AOP_DT;
  message.len = AOP_DATA_SET_LEN;
  _DD(DebugPrint("AOP upload data for svId: ", DEBUG_LEVEL_3);)
  for (int x = 0; x < AOP_DATA_SETS; x++){
    if(aopDataSet[x]->svId == 0) { break; }
    k++;
    message.svId = aopDataSet[x]->svId;
    for (int i=0; i<59; i++) { message.data[i] = aopDataSet[x]->data[i]; }
    _DD(DebugPrint(String(aopDataSet[x]->svId ) + " ", DEBUG_LEVEL_3);)
    GPSsendMessage((uint8_t*)&message, (AOP_DATA_SET_LEN + 4));
  }
  _DD(DebugPrintln(", [" + String(k) + "]", DEBUG_LEVEL_3);)
  return k;
}
#endif
#endif
/*
 * Prints data sent by module during startup. maybe helpful to explore
 * hardware and firmware versions
 */
#if (DEBUG_LEVEL > 0 )
#ifdef DEBUG_SERIAL_GPS
void GPSprintBootData(void){
#ifndef I2C_SWITCH
extern bool powerOnReset;
if(powerOnReset){
  while(SerialGPS.available() > 0){ SerialGPS.read(); }
  //print status of module at boot up - so restart
  pinMode(MEASURES_ENABLE_PIN,OUTPUT);
  digitalWrite(MEASURES_ENABLE_PIN, MEASURES_OFF);
  pause(1000);
  digitalWrite(MEASURES_ENABLE_PIN, MEASURES_ON);
  for (int i=0; i<2000; i++) {
    pause(1);
    String line = "";
    while(SerialGPS.available() > 0){ line += char(SerialGPS.read()); }
    _DD(DebugPrint(line, DEBUG_LEVEL_3);)
   i++;
  }
  _DD(DebugPrintln("", DEBUG_LEVEL_3));
}
#endif //I2C_SWITCH
}
#endif //DEBUG_SERIAL_GPS
void testGPS(){
  int startMs = millis();
  while ((SerialGPS.available() == 0) && ((millis() - startMs) < 60000)) { pause(10); }
  if (GPSprocessData(2000)){
    if (gpsData.type == GPS_DT_NAV_PVT) { if ((GPS_GOT_2D_FIX) || (GPS_GOT_3D_FIX)) {
      _D(DebugPrint("Latitude= ", DEBUG_LEVEL_1));
      _D(DebugPrint(GPS_GET_LAT, 6, DEBUG_LEVEL_1));
      _D(DebugPrint(" Longitude= ", DEBUG_LEVEL_1));
      _D(DebugPrint(GPS_GET_LON, 6, DEBUG_LEVEL_1));
      _D(DebugPrint(" Altitude= ", DEBUG_LEVEL_1));
      _D(DebugPrintln(GPS_GET_ALT, 6, DEBUG_LEVEL_1));
      _D(DebugPrint("Satellites= ", DEBUG_LEVEL_1));
      _D(DebugPrint(GPS_GET_SAT, DEBUG_LEVEL_1));
      _D(DebugPrint(" HDOP= ", DEBUG_LEVEL_1));
      _D(DebugPrint((int)(GPS_GET_PDOP/10), DEBUG_LEVEL_1));
      #ifdef GPS_GET_ACC
      _D(DebugPrint(" ACC= ", DEBUG_LEVEL_1));
      _D(DebugPrint(String(GPS_GET_ACC), DEBUG_LEVEL_1));
      #endif
      _D(DebugPrintln("", DEBUG_LEVEL_1));
    }}
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
#endif //(DEBUG_LEVEL > 0 )

#endif //GPS_MODULE
