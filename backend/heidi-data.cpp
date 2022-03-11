/*
 * heidi-data.cpp
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */
#include <rom/rtc.h>
#include <esp32/ulp.h>
#include <Arduino.h>
#include "heidi-data.h"
#include "heidi-acc.h"
#include "heidi-measures.h"
#include "heidi-fence.h"
#include "heidi-debug.h"
#include "heidi-sys.h"

t_ConfigData*  heidiConfig = NULL;
t_ConfigDataC* newClientConfig;
t_SendData*   availableDataSet[MAX_DATA_SETS];
t_FenceData*  FenceDataSet[FENCE_MAX_POS];
#ifdef SAVE_AOP_DATA
t_aopData*    aopDataSet[AOP_DATA_SETS];
#endif
bool newCycleSettings = false;
bool newFenceSettings = false;
bool newAccSettings   = false;
#ifdef USE_RTC_FAST_MEM
uint32_t* fastMemBuffer = NULL;
#endif
int allDataSets  =   0;


void initConfig(bool reset){
  heidiConfig = (t_ConfigData*)&(RTC_SLOW_MEM[RTC_DATA_SPACE_OFFSET]);
  if(reset) {
    _D(DebugPrintln("Power on reset: reset all RTC data", DEBUG_LEVEL_1);)
    heidiConfig->bootNumber      = START_FROM_RESET;
    heidiConfig->c.bootCycles      = DEFAULT_BOOT_CYCLES;
    heidiConfig->c.sleepMinutes    = DEFAULT_CYCLE_DURATION;
    heidiConfig->c.nightHourStart  = DEFALUT_NIGHT_HOUR_START_UTC;
    heidiConfig->c.nightHourEnd    = DEFALUT_NIGHT_HOUR_END_UTC;
    heidiConfig->c.nightBootCycles = (DEFAULT_BOOT_CYCLES * 2);
    heidiConfig->c.nightSleepMin   = DEFAULT_CYCLE_DURATION;
    heidiConfig->lastTimeDiffMs  = 0;
    heidiConfig->status          = (NEW_FENCE | RESET_INITS);
    heidiConfig->c.accThres1       = DEFALUT_ACCELERATION_THRESHOLD_1;
    heidiConfig->c.accAlertThres1  = DEFALUT_ACCEL_THRES_MAX_COUNT;
    heidiConfig->c.accThres2       = DEFALUT_ACCELERATION_THRESHOLD_2;
    heidiConfig->c.accAlertThres2  = DEFALUT_ACCEL_THRES_MAX_COUNT;
    heidiConfig->c.accNightFactor  = 100;  //equals to 1
    heidiConfig->gpsStatus       = 0;
    #ifdef HEIDI_GATEWAY
    heidiConfig->alertFailCount  = 0;
    heidiConfig->clients         = 0;
    heidiConfig->lastClientTrig  = 0;
    heidiConfig->clientsNeedConf = 0;
    heidiConfig->telNo[0][0]     = 0xBB; //empty
    heidiConfig->telNo[1][0]     = 0xBB; //empty
    #endif
    int t1 = xTaskGetTickCount();
    pause(1000);
    int t2 = xTaskGetTickCount();

  }
}

/*
 * calculate current client setup CRC
 */
uint16_t clientConfigCRC(t_ConfigDataC* data){
  _D(if(data == NULL){DebugPrintln("!!ERROR!! - null pinter clientConfigCRC", DEBUG_LEVEL_1);})
  uint8_t* buffer = (uint8_t*)data;
  return crc16D(buffer, sizeof(t_ConfigDataC));
}

void initRTCData(bool reset){
  #ifdef TEST_RTC
  if(reset){ fillRTCbounary(); }
  #endif
  initConfig(reset);
  uint8_t* curSet;
  //_D(DebugPrintln("Init data sets", DEBUG_LEVEL_1); pause(50));
  #ifdef USE_RTC_FAST_MEM
  RTCfastMemRead();
  curSet = (uint8_t*)fastMemBuffer;
  for (int i=0; i<FAST_MEM_DATA_SETS; i++){
    availableDataSet[allDataSets] = (t_SendData*)curSet;
    if(reset) { initDataSet(availableDataSet[allDataSets]); }
    curSet += DATA_SET_LEN;
    allDataSets++;
  }
  #endif
  curSet = (uint8_t*)&(RTC_SLOW_MEM[(RTC_DATA_SPACE_OFFSET + HEIDI_CONFIG_LENGTH_RTC)]);
  for (int i=0; i<FENCE_MAX_POS; i++){
    FenceDataSet[i] = (t_FenceData*)curSet;
    if(reset) {
      FenceDataSet[i]->latitude  = EMPTY_POINT_VALUE;
      FenceDataSet[i]->longitude = EMPTY_POINT_VALUE;
    }
    curSet += FENCE_SET_LEN;
  }
  #ifdef SAVE_AOP_DATA
  curSet = (uint8_t*)&(RTC_SLOW_MEM[(RTC_DATA_SPACE_OFFSET + HEIDI_CONFIG_LENGTH_RTC + FENCE_MEM_SPACE)]);
  for (int i=0; i<AOP_DATA_SETS; i++){
    aopDataSet[i] = (t_aopData*)curSet;
    if(reset) { aopDataSet[i]->svId = 0; }
    curSet += AOP_DATA_SET_LEN;
  }
  #endif
  curSet = (uint8_t*)&(RTC_SLOW_MEM[(RTC_DATA_SPACE_OFFSET + HEIDI_CONFIG_LENGTH_RTC + FENCE_MEM_SPACE + AOP_DATA_LEN)]);
  for (int i=0; i<SLOW_MEM_DATA_SETS; i++){
    availableDataSet[allDataSets] = (t_SendData*)curSet;
    if(reset) { initDataSet(availableDataSet[allDataSets]); }
    curSet += DATA_SET_LEN;
    allDataSets++;
  }
  _DD(DebugPrintln(String(allDataSets) + " data set available", DEBUG_LEVEL_3);)
  #ifdef USE_ULP
  //init ULP Variables
  if(reset){
    _D(DebugPrintln("Power on reset: reset all ULP vars", DEBUG_LEVEL_1);)
    RTC_SLOW_MEM[ACCEL_DATA_HEADER+ACCEL_DATA_CUR] = 0;
    RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_DIFF_VAL]   = 0;
    RTC_SLOW_MEM[ACCEL_X_VALUES + I2C_TRNS_RES] = 0;
    RTC_SLOW_MEM[ACCEL_X_VALUES + CUR_READ_RES] = 0;
    RTC_SLOW_MEM[ACCEL_X_VALUES + LST_READ_RES] = 0;
    RTC_SLOW_MEM[ACCEL_X_VALUES + CUR_DIFF_VAL] = 0;
    RTC_SLOW_MEM[ACCEL_Y_VALUES + I2C_TRNS_RES] = 0;
    RTC_SLOW_MEM[ACCEL_Y_VALUES + CUR_READ_RES] = 0;
    RTC_SLOW_MEM[ACCEL_Y_VALUES + LST_READ_RES] = 0;
    RTC_SLOW_MEM[ACCEL_Y_VALUES + CUR_DIFF_VAL] = 0;
    RTC_SLOW_MEM[ACCEL_Z_VALUES + I2C_TRNS_RES] = 0;
    RTC_SLOW_MEM[ACCEL_Z_VALUES + CUR_READ_RES] = 0;
    RTC_SLOW_MEM[ACCEL_Z_VALUES + LST_READ_RES] = 0;
    RTC_SLOW_MEM[ACCEL_Z_VALUES + CUR_DIFF_VAL] = 0;
    set_accel_meas_cnt_ULP(0);
    set_accel_avrerage_ULP(0);
    set_accel_excnt1_ULP(0);
    set_accel_exthr1_ULP(DEFALUT_ACCELERATION_THRESHOLD_1);
    set_accel_wake1_ULP(DEFALUT_ACCEL_THRES_MAX_COUNT);
    set_accel_excnt2_ULP(0);
    set_accel_exthr2_ULP(DEFALUT_ACCELERATION_THRESHOLD_2);
    set_accel_wake2_ULP(DEFALUT_ACCEL_THRES_MAX_COUNT);
    set_ULP_request(0);
    set_ULP_lock(0);
  }
  #endif
  _D(
    if(!reset){
      int c = 0;
      for(int i=0; i<allDataSets; i++){ if(!emptyDataSet(availableDataSet[i])) { c++; } }
      DebugPrintln("Used data sets: " + String(c), DEBUG_LEVEL_2);
    }
  )
}

void initDataSet(t_SendData* DataSet){
  DataSet->longitude   = 0;
  DataSet->latitude    = 0;
  DataSet->altitude    = 0;
  DataSet->date        = 0;
  DataSet->time        = 0;
  DataSet->battery     = 0;
  DataSet->GPSpDOP     = 0;
  DataSet->temperature = TEMPERATURE_NOT_SET; //-273,15 .. if this is the real temperature, the sheep is dead.. definitely ;-)
  DataSet->errCode     = 0;
  DataSet->satellites  = 0;
  DataSet->metersOut   = 0;
  DataSet->accThresCnt1   = 0;
  DataSet->accThresCnt2   = 0;
  DataSet->accThCnt1Spr   = 0;
  DataSet->accThCnt2Spr   = 0;

}

void copyDataSet(t_SendData* _from, t_SendData* _to){
  _to->longitude    = _from->longitude ;
  _to->latitude     = _from->latitude  ;
  _to->altitude     = _from->altitude  ;
  _to->date         = _from->date      ;
  _to->time         = _from->time      ;
  _to->battery      = _from->battery   ;
  _to->temperature  = _from->temperature;
  _to->satellites   = _from->satellites;
  _to->GPSpDOP      = _from->GPSpDOP   ;
  _to->errCode      = _from->errCode   ;
  _to->metersOut    = _from->metersOut ;
  _to->accThresCnt1 = _from->accThresCnt1;
  _to->accThresCnt2 = _from->accThresCnt2;
  _to->accThCnt1Spr = _from->accThCnt1Spr;
  _to->accThCnt2Spr = _from->accThCnt2Spr;
}
/*
 * generates a String for transmission of multible data sets
 * sets: array of pointers to data sets
 * first, last: boundaries
 */
String generateMulti64SendLine(t_SendData** sets, int first, int last){
  //php.ini: max_input_var = 1000 by default (1 data set = 1 var) -> max 100 datasets -> no problem
  //php.ini: post_max_size = 8M by default - SIM800 312k only! (1 data set = 64 byte) -> no problem
  //don't forget to increase timeouts on web server side
  int current = first;
  String result = "";
  int k = 0;
  while ((current <= last) && (current < allDataSets)){
    t_SendData* DataSet = sets[current];
    if (!emptyDataSet(DataSet)) {
      k++;
      if (k > 1) { result += "&"; }
      _DD(_PrintDataSet(DataSet, DEBUG_LEVEL_3));
      // why that complicated and not just memcopy?
      // push_data.phtml expects 8 bit count of values, 16 bit ID, 2x32bit coordinates
      // and than count-3 16 bit values - always 16 bit - but values in data structure aren't always 16 bit
      uint8_t hexbuffer[64];
      hexbuffer[0] = HEX_BUFFER_DATA_VALUES + 1; //count of data values + animal id
      hexbuffer[1] = herdeID();
      hexbuffer[2] = animalID();
      _copyInt32toBuffer(hexbuffer,HEX_BUFFER_OFFSET  +  0, DataSet->latitude); //1
      _copyInt32toBuffer(hexbuffer,HEX_BUFFER_OFFSET  +  4, DataSet->longitude); //2
      _copyInt16toBuffer(hexbuffer,HEX_BUFFER_OFFSET  +  8, DataSet->altitude); //3
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 10, DataSet->date); //4
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 12, DataSet->time); //5
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 14, DataSet->battery); //6
      _copyInt16toBuffer(hexbuffer,HEX_BUFFER_OFFSET  + 16, DataSet->temperature); //7
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 18, DataSet->errCode); //8
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 20, DataSet->satellites); //9
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 22, DataSet->GPSpDOP); //10
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 24, DataSet->accThresCnt1);  //11
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 26, DataSet->accThresCnt2);  //12
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 28, DataSet->metersOut);  //13
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 30, DataSet->accThCnt1Spr);  //14
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 32, DataSet->accThCnt2Spr);  //15 = HEX_BUFFER_VALUES
      _DD(
        String HexStr = "";
        for(int i=0; i<HEX_BUFFER_LEN; i++){
          String _hex = String(hexbuffer[i], HEX);
          if (_hex.length() < 2) {_hex = "0" + _hex;}
          HexStr = HexStr + _hex;
        }
        DebugPrintln("bin: " + HexStr, DEBUG_LEVEL_3);
      )
      result += "X" + LenTwo(String(k)) + "=" + b64Encode(hexbuffer, HEX_BUFFER_LEN);
    }
    current++;
  }
  return result;
}

bool setSettingsFromHTTPresponse(String response)
{
  String settingsData = getCSVvalue(response, 4);
  String settingsCRC = getCSVvalue(response, 5);
  uint16_t c_crc = crc16F(settingsData);
  clrState(NEW_SETTINGS);
  if ((settingsData.length() == 0) && (settingsCRC.length() == 0)) { return true; }
  if (settingsData.length() > 3){
    uint16_t t_crc = (uint16_t)hex2int(settingsCRC);
    if(t_crc == c_crc){
      if(newSettingsB64(settingsData, &heidiConfig->c)){
        _D(DebugPrintln("new settings set", DEBUG_LEVEL_1));
        return true;
      } _D( else { DebugPrintln("setting settings failed", DEBUG_LEVEL_1); } )
    } _D( else { DebugPrintln("settings CRC error", DEBUG_LEVEL_1); } )
  } _D( else { DebugPrintln("settings data error", DEBUG_LEVEL_1); } )
  return false;
}

bool newSettingsB64(String b64, t_ConfigDataC* dataBuffer){
  unsigned char buffer[256];
  int dataLen = b64Decode(b64, buffer);
  if (dataLen < 3) { return false; };
  if ( buffer[0] >= 1  ) { setNewSettingU8( buffer, 1, &dataBuffer->bootCycles, NEW_SETTINGS); }
  if ( buffer[0] >= 2  ) { setNewSettingU8( buffer, 3, &dataBuffer->nightBootCycles, NEW_SETTINGS); }
  if ( buffer[0] >= 3  ) { setNewSettingU8( buffer, 5, &dataBuffer->sleepMinutes, NEW_SETTINGS); }
  if ( buffer[0] >= 4  ) { setNewSettingU8( buffer, 7, &dataBuffer->nightSleepMin, NEW_SETTINGS); }
  if ( buffer[0] >= 5  ) { setNewSettingU8( buffer, 9, &dataBuffer->nightHourStart, NEW_SETTINGS); }
  if ( buffer[0] >= 6  ) { setNewSettingU8( buffer,11, &dataBuffer->nightHourEnd, NEW_SETTINGS); }
  if ( buffer[0] >= 7  ) { setNewSettingU16(buffer,13, &dataBuffer->distAlertThres, NEW_SETTINGS); }

  if ( buffer[0] >= 8  ) { setNewSettingU16(buffer,15, &dataBuffer->accThres1, NEW_ACC_DATA); }
  if ( buffer[0] >= 9  ) { setNewSettingU16(buffer,17, &dataBuffer->accAlertThres1, NEW_ACC_DATA); }
  if ( buffer[0] >= 10 ) { setNewSettingU16(buffer,19, &dataBuffer->accThres2, NEW_ACC_DATA); }
  if ( buffer[0] >= 11 ) { setNewSettingU16(buffer,21, &dataBuffer->accAlertThres2, NEW_ACC_DATA); }
  if ( buffer[0] >= 12 ) { setNewSettingU8( buffer,23, &dataBuffer->accNightFactor, NEW_ACC_DATA); }

  if (dataBuffer->sleepMinutes < MIN_CYCLE_DURATION) { dataBuffer->sleepMinutes = MIN_CYCLE_DURATION; }
  if (dataBuffer->nightSleepMin < MIN_CYCLE_DURATION) { dataBuffer->nightSleepMin = MIN_CYCLE_DURATION; }
  if (dataBuffer->sleepMinutes > MAX_CYCLE_DURATION) { dataBuffer->sleepMinutes = MAX_CYCLE_DURATION; }
  if (dataBuffer->nightSleepMin > MAX_CYCLE_DURATION) { dataBuffer->nightSleepMin = MAX_CYCLE_DURATION; }
  if (dataBuffer->bootCycles > MAX_BOOT_CYCLES) { dataBuffer->bootCycles = MAX_BOOT_CYCLES; }
  if (dataBuffer->nightBootCycles > MAX_BOOT_CYCLES) { dataBuffer->nightBootCycles = MAX_BOOT_CYCLES; }
  if (dataBuffer->bootCycles < 1) { dataBuffer->bootCycles = DEFAULT_BOOT_CYCLES; }
  if (dataBuffer->nightBootCycles < 1) { dataBuffer->nightBootCycles = DEFAULT_BOOT_CYCLES; }
  if ((dataBuffer->nightHourStart < 0) || (dataBuffer->nightHourStart > 23)) { dataBuffer->nightHourStart = 0; }
  // if we have a bad night end-Value we set start = end, means no night modus
  if ((dataBuffer->nightHourEnd < 0) || (dataBuffer->nightHourEnd > 23)) { dataBuffer->nightHourEnd = dataBuffer->nightHourStart; }
  // if no night modus - set them to zero
  if (dataBuffer->nightHourStart == dataBuffer->nightHourEnd) { dataBuffer->nightHourStart = 0; dataBuffer->nightHourEnd = 0; }
  return true;
}

void setNewSetting8(uint8_t *buffer, int pufferPos, int8_t *setting, uint32_t status){
  int8_t dummy = _copyBufferToUInt16(buffer, pufferPos);
  if(dummy != *setting){
    *setting = dummy;
    setState(status);
    _DD( DebugPrintln("set [" + String(pufferPos) + "] to " + String(*setting), DEBUG_LEVEL_3); )
  } _DD( else { DebugPrintln("setting " + String(pufferPos) +  "keeps the same", DEBUG_LEVEL_3); } )
}
void setNewSettingU8(uint8_t *buffer, int pufferPos, uint8_t *setting, uint32_t status){
  uint8_t dummy = _copyBufferToUInt16(buffer, pufferPos);
  if(dummy != *setting){
    *setting = dummy;
    setState(status);
    _DD( DebugPrintln("set [" + String(pufferPos) + "] to " + String(*setting), DEBUG_LEVEL_3); )
  } _DD( else { DebugPrintln("setting " + String(pufferPos) +  "keeps the same", DEBUG_LEVEL_3); } )
}void setNewSettingU16(uint8_t *buffer, int pufferPos, uint16_t *setting, uint32_t status){
  uint16_t dummy = _copyBufferToUInt16(buffer, pufferPos);
  if(dummy != *setting){
    *setting = dummy;
    setState(status);
    _DD( DebugPrintln("set [" + String(pufferPos) + "] to " + String(*setting), DEBUG_LEVEL_3); )
  } _DD( else { DebugPrintln("setting " + String(pufferPos) +  "keeps the same", DEBUG_LEVEL_3); } )
}
#ifdef HEIDI_GATEWAY
bool newTelNoB64(String b64){
  unsigned char buffer[256];
  int dataLen = b64Decode(b64, buffer);
  if (( buffer[0] == 0 ) || ( buffer[0] > 2 )) { return false; }
  for(int n=0; n<buffer[0]; n++){
    if (n >= TEL_NO_CNT){ break; }
    for(int i=0; i<TEL_NO_LEN; i++) { heidiConfig->telNo[n][i] = buffer[n*12+i+1]; }
  }
  return true;
}
String getTelNo(int which){
  static String TelNo;
  TelNo = "";
  if(which >= TEL_NO_CNT) { return TelNo; }
  for(int i=0; i<TEL_NO_LEN; i++){
    uint8_t b = heidiConfig->telNo[which][i];
    for(int j=0; j<2; j++){
      uint8_t n = b & 0xf;
      if (n >= 0x0B) { return TelNo; }
      else if (n == 0x0A) { TelNo = TelNo + '+'; }
      else if (n <= 0x09) { TelNo = TelNo + String(n); }
      b = b >> 4;
    }
  }
  return TelNo;
}
bool setTelNoFromHTTPresponse(String response)
{
  String settingsData = getCSVvalue(response, 6);
  String settingsCRC = getCSVvalue(response, 7);
  uint16_t c_crc = crc16F(settingsData);
  if ((settingsData.length() == 0) && (settingsCRC.length() == 0)) { return true; }
  if (settingsData.length() > 3){
    uint16_t t_crc = (uint16_t)hex2int(settingsCRC);
    if(t_crc == c_crc){
      if(newTelNoB64(settingsData)){
        _D(DebugPrintln("new tel. number set", DEBUG_LEVEL_1));
        _DD( for(int x= 0; x<TEL_NO_CNT; x++) {DebugPrintln("number " +String(x) + ": " + getTelNo(x), DEBUG_LEVEL_3);})
        return true;
      } _D( else { DebugPrintln("setting tel. number failed", DEBUG_LEVEL_1); } )
    } _D( else { DebugPrintln("tel. number CRC error", DEBUG_LEVEL_1); } )
  } _D( else { DebugPrintln("tel. number data error", DEBUG_LEVEL_1); } )
  return false;
}
#endif
/*
 * packUpDataSets removes free data sets between used ones and returns count of used data sets
 * Parameter: start position of packing
 */
int packUpDataSets(){
  int k = 0;
  for(int i=0; i<allDataSets; i++){
    if(!emptyDataSet(availableDataSet[i]) && (k < i) && emptyDataSet(availableDataSet[k])){
      copyDataSet(availableDataSet[i], availableDataSet[k]);
      initDataSet(availableDataSet[i]);
    }
    if(!emptyDataSet(availableDataSet[k])){ k++; }
  }
  _DD( DebugPrintln("Data sets used: " + String(k) + " / "  + String(allDataSets), DEBUG_LEVEL_3);)
  return k;
}
void freeFirstDataSet(void){
  if(emptyDataSet(availableDataSet[0])) { return; }
  for(int i=(allDataSets - 2); i >= 0; i--){
    if(!emptyDataSet(availableDataSet[i])){
      copyDataSet(availableDataSet[i], availableDataSet[i+1]);
      initDataSet(availableDataSet[i]);
    }
  }
}

bool emptyDataSet(t_SendData* DataSet){
  return (DataSet->date == 0);
}
void initDataSets(t_SendData** sets, int first, int last){
  int current = first;
  while (current <= last){
    t_SendData* set = sets[current];
    initDataSet(set);
    current++;
  }
}

void testData()
{
  _D(
     String testCSV1    = ";;3;45;";
     String testCSV2   = "1;2;3;4;5";

     _D(DebugPrintln("testCSV1: " + testCSV1, DEBUG_LEVEL_1));
     for(int i=1; i<7; i++){
       _D(DebugPrintln(String(i) + ": '" + getCSVvalue(testCSV1,i) + "'", DEBUG_LEVEL_1));
     }
     _D(DebugPrintln("testCSV2: " + testCSV2, DEBUG_LEVEL_1));
     for(int i=1; i<6; i++){
       _D(DebugPrintln(String(i) + ": '" + getCSVvalue(testCSV2,i) + "'", DEBUG_LEVEL_1));
     }
  )
}

void getRTCDataSpace(uint8_t** buffer){
	*buffer = (uint8_t*)&(RTC_SLOW_MEM[RTC_DATA_SPACE_OFFSET]);
}

bool getState(uint32_t which){
  _D(if(heidiConfig == NULL){DebugPrintln("!!ERROR!! - null pinter getState", DEBUG_LEVEL_1);})
  return((heidiConfig->status & which) != 0);
}
void setState(uint32_t which){
  _D(if(heidiConfig == NULL){DebugPrintln("!!ERROR!! - null pinter setState", DEBUG_LEVEL_1);})
  heidiConfig->status |= which;
}
void clrState(uint32_t which){
  _D(if(heidiConfig == NULL){DebugPrintln("!!ERROR!! - null pinter clrState", DEBUG_LEVEL_1);})
  heidiConfig->status &= ~which;
}

int32_t getCycleTimeMS(void){
  _D(if(heidiConfig == NULL){DebugPrintln("!!ERROR!! - null pinter getCycleTimeMS", DEBUG_LEVEL_1);})
  if(_night()){
    return heidiConfig->c.nightSleepMin * 60000;
  }
  return heidiConfig->c.sleepMinutes * 60000;
}

int32_t  GeoToInt(double geo)
{
  return (int32_t)rint(geo * 1000000);
}
double   IntToGeo(int32_t val)
{
  return (double)(val) / 1000000;
}


String DateString(tm* timestamp){
  return String(timestamp->tm_year) + "-" + LenTwo(String(timestamp->tm_mon + 1)) + "-" + LenTwo(String(timestamp->tm_mday));
}
String TimeString(tm* timestamp){
  return LenTwo(String(timestamp->tm_hour)) + ":" + LenTwo(String(timestamp->tm_min)) + ":"+ LenTwo(String(timestamp->tm_sec));
}
String DOSdateString(uint16_t _dosDate){
  return String(dosYear(_dosDate)) + "-" + LenTwo(String(dosMonth(_dosDate))) + "-" + LenTwo(String(dosDay(_dosDate)));
}
String DOStimeString(uint16_t _dosTime){
  return LenTwo(String(dosHour(_dosTime))) + ":" + LenTwo(String(dosMinute(_dosTime))) + ":" + LenTwo(String(dosSecond(_dosTime)));
}
String LenTwo(const String No){
  if (No.length() == 1) { return "0" + No; }
  return No;
}


//uint16_t date; // 0-4 Day of the month / 5-8 Month /  9-15 Year offset from 1980
uint16_t dosDate(const uint8_t year, const uint8_t month, const uint8_t day){
  uint16_t date_val = day & 0x1F;
  date_val += (month & 0x0F) << 5;
  date_val += (year-1980) << 9;
  return date_val;
}
uint16_t dosYear(const uint16_t date){
  return (date >> 9) + 1980;
}
uint8_t dosMonth(const uint16_t date){
  return (date >> 5) & 0x0F;
}
uint8_t dosDay(const uint16_t date){
  return date & 0x1F;
}
//uint16_t time; // 0-4 Second divided by 2 / 5-10 Minute / 11-15 Hour
uint16_t dosTime(const uint8_t hour, const uint8_t minute, const uint8_t second){
  uint16_t time_val = second >> 1;
  time_val += (minute & 0x3F) << 5;
  time_val += (hour & 0x1F) << 11;
  return time_val;
}
uint8_t dosHour(const uint16_t time){
  return (time >> 11);
}
uint8_t dosMinute(const uint16_t time){
  return (time >> 5) & 0x3F;;
}
uint8_t dosSecond(const uint16_t time){
  return (time & 0x1F) << 1;
}



#ifdef USE_RTC_FAST_MEM

TaskHandle_t task_fmem_read;
TaskHandle_t task_fmem_write;
#define FAST_MEM_TASK_HEAP_SIZE 1024
RTC_FAST_ATTR uint32_t _realFastMem[RTC_FAST_MEM_SIZE_32];
bool taskReady;
bool taskRunning;

/*
 * usually arduino code runs on CPU_1, but RTC_FAST_MEM is accessible only from CPU_0. Therefore we need
 * to create Tasks for reading and writing pinned on CPU_0. You may change several settings in
 * "./arduinocdt/packages/esp32/hardware/esp32/[version]/tools/sdk/sdkconfig" to get arduino code running
 * on CPU_0 - no clue what's all needed for that
 */

#ifdef USE_RTC_FAST_MEM
bool RTCfastMemRead(void){
  fastMemBuffer = (uint32_t*)malloc(RTC_MAX_FAST_DATA_SPACE);
  if(fastMemBuffer == NULL){
    _D(DebugPrintln("unable to allocate fast mem buffer. ", DEBUG_LEVEL_1); pause(50);)
    return false;
  }
  BaseType_t rc = xTaskCreatePinnedToCore(fastMemReadTask, "fastMemReadTask", FAST_MEM_TASK_HEAP_SIZE, NULL, 1, &task_fmem_read, 0);
  if (rc != pdPASS){
    _D(DebugPrintln("unable to create fast mem read Task. " + String(rc), DEBUG_LEVEL_1); pause(50);)
    return false;
  }
  while (task_fmem_read != NULL){ vTaskDelay(100); }
  //_D( PrintRTCFastMemBufferBoundaries(); )
  return true;
}
bool RTCfastMemWrite(void){
  BaseType_t rc = xTaskCreatePinnedToCore(fastMemWriteTask, "fastMemWriteTask", FAST_MEM_TASK_HEAP_SIZE, NULL, 1, &task_fmem_write, 0);
  if (rc != pdPASS){
    _D(DebugPrintln("unable to create fast mem read Task. " + String(rc), DEBUG_LEVEL_1); pause(50);)
    return false;
  }
  while (task_fmem_write != NULL){ vTaskDelay(100); }
  free(fastMemBuffer);
  fastMemBuffer = NULL;
  return true;
}

void fastMemReadTask(void *pvParameters) {
  if(fastMemBuffer != NULL){
    for(int i=0; i<RTC_FAST_MEM_SIZE_32; i++){ fastMemBuffer[i] = _realFastMem[i];  }
  }
  task_fmem_read = NULL;
  vTaskDelete(NULL);
}

void fastMemWriteTask(void *pvParameters) {
  if(fastMemBuffer != NULL){
    for(int i=0; i<RTC_FAST_MEM_SIZE_32; i++){ _realFastMem[i] = fastMemBuffer[i]; }
  }
  task_fmem_write = NULL;
  vTaskDelete(NULL);
}
/*
_D(
void PrintRTCFastMemBufferBoundaries(void){
  int a = 0;
  for(int i=0; i<32; i++){
    if ((a++) == 8) {DebugPrintln(hexString8(fastMemBuffer[i]), DEBUG_LEVEL_1); a = 0; }
    else { DebugPrint(hexString8(fastMemBuffer[i]) + ", ", DEBUG_LEVEL_1); }
  }
  pause(100);
  a = 0;
  for(int i=(RTC_FAST_MEM_SIZE_32-32); i<RTC_FAST_MEM_SIZE_32; i++){
    if ((a++) == 8) { DebugPrintln(hexString8(fastMemBuffer[i]), DEBUG_LEVEL_1); a = 0; }
    else { DebugPrint(hexString8(fastMemBuffer[i]) + ", ", DEBUG_LEVEL_1); }
  }
  pause(100);
}
)
*/
#endif
#ifdef TEST_RTC

#define RTC_TEST_PATTERN 0x55AA55AA
#define RTC_TEST_MAX_MEM 2048

void testRTC(t_SendData* currentDataSet, tm* bootTime){
  currentDataSet->date = dosDate(bootTime->tm_year, bootTime->tm_mon + 1, bootTime->tm_mday);
  currentDataSet->time = dosTime(bootTime->tm_hour, bootTime->tm_min, bootTime->tm_sec);
  _D(DebugPrintln("RTC ULP_MEM_SIZE: " + String(ACCEL_ULP_MEM_SIZE) + ", ULP_CODE_SIZE: " + String(ACCEL_ULP_CODE_SIZE) + ", ULP_DATA_SIZE: " + String(ACCEL_ULP_DATA_SIZE), DEBUG_LEVEL_2);)
  _D(DebugPrintln("RTC config address: 0x" + String((uint32_t)heidiConfig, HEX), DEBUG_LEVEL_2);)
  _PrintHeidiConfig(DEBUG_LEVEL_2);
  _PrintHeidiAccParams(DEBUG_LEVEL_2);
  _PrintShortSummary(DEBUG_LEVEL_2);
  _PrintFence(DEBUG_LEVEL_2);
  testRTCbounary();
 }
void fillRTCbounary(){
  _D(DebugPrintln("fill RTC memory with test pattern", DEBUG_LEVEL_2);)
  for (int i=(RTC_DATA_SPACE >> 2); i<RTC_TEST_MAX_MEM; i++ ){ RTC_SLOW_MEM[i] = RTC_TEST_PATTERN; }
}

void testRTCbounary(){
  _D(DebugPrint("Test RTC boundary from " + String(RTC_DATA_SPACE >> 2)+ " to " + String(RTC_TEST_MAX_MEM) + ": ", DEBUG_LEVEL_2);)
  for (int i=(RTC_DATA_SPACE >> 2); i<RTC_TEST_MAX_MEM; i++ ){
    if (RTC_SLOW_MEM[i] != RTC_TEST_PATTERN){
      _D(DebugPrintln("RTC boundary at " + String(i) + +", 0x" + String(RTC_SLOW_MEM[i], HEX), DEBUG_LEVEL_2);)
      return;
    }
  }
  _D(DebugPrintln(" ..clean", DEBUG_LEVEL_2);)
}
#endif
#endif
