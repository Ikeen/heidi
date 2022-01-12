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

HardwareSerial SerialGPS(GPS_UART_NO);
gpsData_t gpsData;
/*
 * https://youtu.be/ylxwOg2pXrc?list=RDCMUCTXOorupCLqqQifs2jbz7rQ
 */
uint8_t UBX_UART_POLL[] =   { GPS_UBX_CLASS_CFG, GPS_UBX_CFG_PRT, 0x01, 0x00, 0x01 };
uint8_t UBX_NAVX5_POLL[]=   { GPS_UBX_CLASS_CFG, GPS_UBX_CFG_NAVX5, 0x00, 0x00 };
uint8_t UBX_EPH_POLL[]    = { GPS_UBX_CLASS_AID, GPS_UBX_AID_EPH, 0x00, 0x00 };
uint8_t UBX_AOP_DT_POLL[] = { GPS_UBX_CLASS_AID, GPS_UBX_AID_AOP_DT, 0x00, 0x00 };
uint8_t UBX_AOP_ST_POLL[] = { GPS_UBX_CLASS_AID, GPS_UBX_AID_AOP_ST, 0x00, 0x00 };
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
 * this function gets data sets until [requiredAccuracy] is reached or [maxWorse] data sets had a less
 * accuracy then the current one (means: it seem to not getting better within a acceptable time)
 */
bool GPSGetPosition(t_SendData* DataSet, int requiredAccuracy, int maxWorse, int timeOut){
  double acc_mn = -1.0;
  locationSet_t curLocation;
  int worse   = 0;
  bool extendMeas = (heidiConfig->gpsStatus >= GPS_GOT_3D_LOCK);

  setError(E_COULD_NOT_FETCH_GPS);
  setError(E_WRONG_GPS_VALUES);
  curLocation.gps_sat = 0;

  _D(DebugPrintln("GPS: Acquire position", DEBUG_LEVEL_1));
  gpsData.valid = false;
  while((worse < maxWorse) && ((millis() < timeOut) || (timeOut == 0))){
    if (GPSprocessData(500))  {
      if (gpsData.valid) {
        if (gpsData.type == GPS_DT_NAV_PVT) {
          if ((gpsData.data->navPVT.fixType == GPS_FIX_TYPE_2D_FIX) || (gpsData.data->navPVT.fixType == GPS_FIX_TYPE_3D_FIX)) {
            if ((GPS_GET_ACC < (curLocation.acc - 0.5))  || (curLocation.gps_sat == 0)){
              curLocation.gps_sat = GPS_GET_SAT;
              curLocation.lng   = GPS_GET_LON;
              curLocation.lat   = GPS_GET_LAT;
              if(gpsData.data->navPVT.fixType == GPS_FIX_TYPE_2D_FIX){ curLocation.alt = 0; } else { curLocation.alt = GPS_GET_ALT; }
              curLocation.acc   = GPS_GET_ACC;
              curLocation.hdop  = GPS_GET_DOP;
              //_DD(DebugPrintln("ACC: " + String(GPS_GET_ACC), DEBUG_LEVEL_3);)
            } else {
              if(!extendMeas){
                worse++; //_DD( DebugPrintln("ACC: " + String(GPS_GET_ACC, 1) + ", unUsedSets: " + String(worse),  DEBUG_LEVEL_3);)
              } //else use full time to get ephemeris data
            }
            if(curLocation.acc <= requiredAccuracy) { RESET_GPS_DATA; break; }
          } else if(gpsData.type == GPS_DT_AID_AOP_STATUS){ _DD(DebugPrintln(String(gpsData.data->AIDaopStatus.status)+ ", ", DEBUG_LEVEL_3);)}
        }

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
        else if (gpsData.type == GPS_DT_AID_AOP_STATUS) {
          _DD(DebugPrintln("GPS AOPstatus: " + String(gpsData.data->AIDaopStatus.status), DEBUG_LEVEL_3);)
        }
      }
      RESET_GPS_DATA;
    }
    #ifdef DEBUG_SERIAL_GPS
    else {_DD(DebugPrint(".", DEBUG_LEVEL_3);) }
    #endif
  }
  #ifdef DEBUG_SERIAL_GPS
  _DD(DebugPrintln("", DEBUG_LEVEL_3));
  #endif
  if(curLocation.gps_sat > 0){
    _D(DebugPrintln("GPS: Lat: " + String(curLocation.lat,6) + ", Lon: " + String(curLocation.lng, 6)
                     + ", ACC: " + String(curLocation.acc,1) + ", DOP: " + String(curLocation.hdop,1), DEBUG_LEVEL_1);)
    rmError(E_COULD_NOT_FETCH_GPS);
    DataSet->latitude = GeoToInt(curLocation.lat);
    DataSet->longitude = GeoToInt(curLocation.lng);
    DataSet->altitude = (int16_t)curLocation.alt;
    DataSet->GPShdop  = (uint8_t)round(curLocation.hdop*10);
    DataSet->satellites = (uint8_t)curLocation.gps_sat;
    if (!((DataSet->latitude == 0) && (DataSet->longitude == 0))){ //0.0, 0.0 must be wrong (or a fish)
      rmError(E_WRONG_GPS_VALUES);
    } else { return false; }
    DataSet->metersOut = 0;
    if(!(getError(E_COULD_NOT_FETCH_GPS) || getError(E_WRONG_GPS_VALUES) || invalidFence())){
      t_PointData position;
      position.latitude = curLocation.lat;
      position.longitude = curLocation.lng;
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
  return (curLocation.gps_sat > 0);
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
  RESET_GPS_DATA;
  while(   ((GPStime.tm_year < 2000) || (GPStime.tm_mon <= 0) || (GPStime.tm_mday <= 0))
        && ((millis() < timeOut) || (timeOut == 0))){
    if(GPSprocessData(500)){
      if (gpsData.valid) { if (gpsData.type == GPS_DT_NAV_PVT) { if (gpsData.data->navPVT.valid == GPS_DATE_TIME_VALID) {{
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
      RESET_GPS_DATA;
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
    GPSprintBootData();
    #else
    delay(500); //let it print all boot stuff..
    while(SerialGPS.available() > 1) {SerialGPS.read();} //..and crush it
    #endif

    if (!GPSsetupUART(57600)) {return false; }
    if (!GPSenableAOP())  {return false; }
    if (GPSEphermerisDataStatus() > 4){
      if (heidiConfig->gpsStatus < GPS_GOT_EHP_DATA) { heidiConfig->gpsStatus = GPS_GOT_EHP_DATA; }
    } else { heidiConfig->gpsStatus = GPS_LESS_EHP_DATA; }
    #ifdef SAVE_AOP_DATA
    if ((GPSuploadAOPdata() > 4) && (heidiConfig->gpsStatus < GPS_GOT_AOP_DATA)) { {
    #else
    if (heidiConfig->gpsStatus < GPS_GOT_AOP_DATA) { if (GPSAOPdataStatus() > 4) {
    #endif
      heidiConfig->gpsStatus = GPS_GOT_AOP_DATA;
    } }
    //enable UBX NAV PVT - message on UART1
    MSG_ON_OFF[MSG_ON_OFF_GROUP] = GPS_CFG_MSG_UBX;
    MSG_ON_OFF[GPS_CFG_SER_CH_UART1]  = GPS_CFG_ON;
    MSG_ON_OFF[MSG_ON_OFF_TYPE]  = GPS_UBX_NAV_PVT;
    GPSsendMessage(MSG_ON_OFF, sizeof(MSG_ON_OFF));
    //enable UBX NAV STATUS - message on UART1
    MSG_ON_OFF[MSG_ON_OFF_TYPE]  = GPS_UBX_NAV_STATUS;
    GPSsendMessage(MSG_ON_OFF, sizeof(MSG_ON_OFF));
    MSG_ON_OFF[MSG_ON_OFF_TYPE]  = GPS_UBX_AID_AOP_ST;
    GPSsendMessage(MSG_ON_OFF, sizeof(MSG_ON_OFF));
    //set GPS measure rate
    *(uint16_t*)&UBX_SET_RATE[4] = 0x01F4; //500
    GPSsendMessage(UBX_SET_RATE, sizeof(UBX_SET_RATE));

    gpsData.valid = false;
    gpsData.type  = GPS_DT_NONE;
    gpsData.data  = NULL;

    _D(DebugPrintln("GPS: opened successfully", DEBUG_LEVEL_2));
  } _D(else { DebugPrintln("GPS: was already open", DEBUG_LEVEL_1); delay(50); })
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

bool GPSsetupUART(int newBaudRate){
  //[switch SerialGPS to higher baud rate and] switch off NMEA message
  //it's tricky - to be on the safe side, clear all data
  while(SerialGPS.available() > 1) {SerialGPS.read();}
  int i = 0;
  int oldBaudRate = 9600;
  GPSsendMessage(UBX_UART_POLL, sizeof(UBX_UART_POLL));
  while(i < 3){ // tries
    //check if it's done:
    if(GPSwaitForMessage(GPS_DT_CFG_PRT, 1000)){
      if((gpsData.data->UARTconf.outProtoMask == 0x0001) && (gpsData.data->UARTconf.baud == newBaudRate)){
        _DD(DebugPrintln("GPS: setup UART successfully [" + String(i) + (i==1?" try]":" tries]"), DEBUG_LEVEL_3));
        return true;
      } else {
        if(i == 1){ oldBaudRate = gpsData.data->UARTconf.baud; }
        /*
         * setting baud rate may lead into a deadlock if the message fails on the module and we switch
         * baud rate on host. 2 to 3 tries with alternating baud rate (new <-> old) should avoid that.
         * however high baud rates are not needed if we not intend to transfer huge amounts of data
         * e.g. EPHdata or AOPData (but currently we did)
         */
        gpsData.data->UARTconf.baud = newBaudRate; //baud
        gpsData.data->UARTconf.outProtoMask = 0x0001; //mask all messages except UBX
        GPSsendMessage((uint8_t*)gpsData.data, sizeof(UART_CONFIG_t));
        setGPSserialBaudRate(newBaudRate);
      }
    } else {
      //got no data - something went wrong. Now we try old and new baud rate in alternation
      _D(DebugPrintln("GPS: setup UART no data - alternate baud rate", DEBUG_LEVEL_1));
      if((i % 2) == 0){ setGPSserialBaudRate(oldBaudRate); }
      else { setGPSserialBaudRate(newBaudRate); }
    }
    GPSsendMessage(UBX_UART_POLL, sizeof(UBX_UART_POLL));
    i++;
  }
  _D(DebugPrintln("GPS: setup UART failed", DEBUG_LEVEL_1));
  return false;
}

bool GPSenableAOP(void){
  int i = 0;
  GPSsendMessage(UBX_NAVX5_POLL, sizeof(UBX_NAVX5_POLL));
  while(i < 2){ // tries
    //check if it's done:
    if(GPSwaitForMessage(GPS_DT_CFG_NAVX5, 1000)){
      if(gpsData.data->NAVX5conf.aopCfg  == 0x01){
        _DD(DebugPrintln("GPS: auto assist mode enabled successfully [" + String(i) + (i==1?" try]":" tries]"), DEBUG_LEVEL_3));
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
      if (GPSdecodeData(r)) { _DD(DebugPrintln("", DEBUG_LEVEL_3); DebugPrintln("", DEBUG_LEVEL_3);) return true; }
      #else
       if (GPSdecodeData(SerialGPS.read())){ return true; }
      #endif
    } else { delay(1); }
  }
  return false;
}
/*
 * GPSdecodeData returns true if a data set was completely and successful decoded
 * keep in mind: GPS module will continuously send date and processing it is a rolling process
 */
bool GPSdecodeData(uint8_t data) {
  static int fpos = 0;
  static uint8_t checksumSent[2];
  static UBXMessage_t ubxBuffer;
  static int payloadSize;
  static uint8_t* buffer = (uint8_t*)&ubxBuffer;

  if ( fpos < 2 ) {  if ( data == UBX_HEADER[fpos] ){  fpos++; } else { fpos = 0; } return false; }
  //header found, load id & length (now we write new data into structure, means: old data is invalid)
  gpsData.valid = false;
  if ( fpos < 5 ) { buffer[fpos-2] = data; fpos++; return false; }
  if ( fpos == 5 ) { //check header information
    buffer[fpos-2] = data;
    payloadSize = ubxBuffer.head.len;
    if (payloadSize > 0) {
      fpos++;
      gpsData.type  = GPS_DT_NONE;
      if(ubxBuffer.head.cls == GPS_UBX_CLASS_NAV){
        if(ubxBuffer.head.id == GPS_UBX_NAV_PVT){ gpsData.type = GPS_DT_NAV_PVT; }
        if(ubxBuffer.head.id == GPS_UBX_NAV_STATUS){ gpsData.type = GPS_DT_NAV_STATUS; }
      }
      if(ubxBuffer.head.cls == GPS_UBX_CLASS_CFG){
        if(ubxBuffer.head.id == GPS_UBX_CFG_PRT){ gpsData.type = GPS_DT_CFG_PRT; }
        if(ubxBuffer.head.id == GPS_UBX_CFG_NAVX5){ gpsData.type = GPS_DT_CFG_NAVX5; }

      }
      if(ubxBuffer.head.cls == GPS_UBX_CLASS_AID){
        if(ubxBuffer.head.id == GPS_UBX_AID_EPH){ gpsData.type = GPS_DT_AID_EPH; }
        if(ubxBuffer.head.id == GPS_UBX_AID_AOP_DT){ gpsData.type = GPS_DT_AID_AOP_DATA; }
        if(ubxBuffer.head.id == GPS_UBX_AID_AOP_ST){ gpsData.type = GPS_DT_AID_AOP_STATUS; }
      }
    } else { fpos = 0;} //something strange
    return false;
  }
  if ( fpos < (payloadSize + 6)){
    if (gpsData.type != GPS_DT_NONE){ //if data type is known then load
      buffer[fpos-2] = data;
    } //otherwise crush it
    fpos++;
    return false;
  }
  if (fpos == (payloadSize + 6)) { checksumSent[0] = data; fpos++; return false;}
  if (fpos == (payloadSize + 7)) {
    uint8_t checksumCalc[2];
    checksumSent[1] = data;
    fpos = 0;
    GPScalcChecksum(buffer, checksumCalc, payloadSize+4);
    if((checksumCalc[0]==checksumSent[0]) && (checksumCalc[1]==checksumSent[1])){
      gpsData.data  = &ubxBuffer;
      gpsData.valid = true;
      //if you intend to fetch cyclic transmitted data, this is the point to copy it into a dedicated place
      //otherwise it would be overwritten by the next packet
      return true;
    }
  }
  //we should never come here
  gpsData.valid = false;
  gpsData.type  = GPS_DT_NONE;
  fpos = 0;
  return false;
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
  #ifdef DEBUG_SERIAL_GPS
  _DD(DebugPrint("GPS send: ", DEBUG_LEVEL_3);
    for(int i=0; i<l; i++){ DebugPrint(LenTwo(String(fullmessage[i], HEX)) + " ", DEBUG_LEVEL_3); }
    DebugPrintln("", DEBUG_LEVEL_3);
  )
  #endif
  for(int i=0; i<l; i++){
    SerialGPS.write(fullmessage[i]);
    delay(1); //this is, for whatever reason, necessary. NEO7M will not understand messages sent without breaks
  }
  delay(50); // setup time (see above)
}

bool GPSwaitForMessage(gpsDataType_t type, int timeout){
  int t = millis();
  RESET_GPS_DATA;
  while(millis() - t < timeout){
    if (GPSprocessData(200)) {
      if (gpsData.type == type) { return true; } else { RESET_GPS_DATA; }
    }
  }
  return false;
}
void setGPSserialBaudRate(int rate){
  SerialGPS.end();
  SerialGPS.begin(rate, SERIAL_8N1, GPS_RXD, GPS_TXD, false);
  pinMode(GPS_RXD, INPUT_PULLDOWN);
  pinMode(GPS_TXD, OUTPUT);
  delay(10); //wait for all the rubbish
  while(SerialGPS.available() > 1) {SerialGPS.read();}
}

uint8_t GPSEphermerisDataStatus(void){
  int t = millis(), i = 0;
  uint8_t x = 0;
  GPSsendMessage(UBX_EPH_POLL, sizeof(UBX_EPH_POLL));
  _DD(DebugPrint("EPH: ", DEBUG_LEVEL_3);)
  do {
    if(GPSwaitForMessage(GPS_DT_AID_EPH, 200)){
      _DD(DebugPrint(String(gpsData.data->AIDephData.len)+ ", ", DEBUG_LEVEL_3);)
      if(gpsData.data->AIDephData.len > 8){ x++; }
      i++;
    }
  }while((i<32) && (millis()-t < 2000));
  _DD(DebugPrintln(", [" + String(x) + "]", DEBUG_LEVEL_3);)
  return x;
}
#ifdef SAVE_AOP_DATA

void GPSsaveAOPdata(void){
  int t = millis(), k = 0;
  uint8_t x = 0;
  for (int x = 0; x < AOP_DATA_SETS; x++){ aopDataSet[x]->svId = 0; }
  GPSsendMessage(UBX_AOP_DT_POLL, sizeof(UBX_EPH_POLL));
  _DD(DebugPrint("AOP save data for svId: ", DEBUG_LEVEL_3);)
  do {
    if(GPSwaitForMessage(GPS_DT_AID_AOP_DATA, 300)){
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

uint8_t GPSAOPdataStatus(void){
  int t = millis(), i = 0;
  uint8_t x = 0;
  GPSsendMessage(UBX_AOP_DT_POLL, sizeof(UBX_EPH_POLL));
  _DD(DebugPrint("AOP: ", DEBUG_LEVEL_3);)
  do {
    if(GPSwaitForMessage(GPS_DT_AID_AOP_DATA, 200)){
     _DD(DebugPrint(String(gpsData.data->AIDaopData.len)+ ", ", DEBUG_LEVEL_3);)
     if(gpsData.data->AIDaopData.len > 1){ x++; }
     i++;
    }
  }while((i<32) && (millis()-t < 5000));
  _DD(DebugPrintln(", [" + String(x) + "]", DEBUG_LEVEL_3);)
  return x;
}
#ifdef HEIDI_CONFIG_TEST
/*
 * Prints data sent by module during startup. maybe helpful to explore
 * hardware and firmware versions
 */
void GPSprintBootData(void){
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
#endif //I2C_SWITCH
}

#endif //HEIDI_CONFIG_TEST

#if (DEBUG_LEVEL > 0 )
void testGPS(){
  int startMs = millis();
  while ((SerialGPS.available() == 0) && ((millis() - startMs) < 60000)) { delay(10); }
  if (GPSprocessData(2000)){
    if (gpsData.type == GPS_DT_NAV_PVT) { if ((gpsData.data->navPVT.fixType == GPS_FIX_TYPE_2D_FIX) || (gpsData.data->navPVT.fixType == GPS_FIX_TYPE_3D_FIX)) {
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
