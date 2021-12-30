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

t_ConfigData* heidiConfig;
t_SendData*   availableDataSet[MAX_DATA_SETS];
t_FenceData*  FenceDataSet[FENCE_MAX_POS];
bool newCycleSettings = false;
bool newFenceSettings = false;
bool newAccSettings   = false;
#ifdef USE_RTC_FAST_MEM
uint32_t fastMemBuffer[RTC_FAST_MEM_SIZE_32];
#endif

void initConfig(bool reset){
  heidiConfig = (t_ConfigData*)&(RTC_SLOW_MEM[RTC_DATA_SPACE_OFFSET]);
  if(reset) {
    _D(DebugPrintln("Power on reset: reset all RTC data", DEBUG_LEVEL_1);)
    heidiConfig->bootCount       = START_FROM_RESET;
    heidiConfig->bootCycles      = DEFAULT_BOOT_CYCLES;
    heidiConfig->sleepMinutes    = DEFAULT_CYCLE_DURATION;
    heidiConfig->nightHourStart  = DEFALUT_NIGHT_HOUR_START_UTC;
    heidiConfig->nightHourEnd    = DEFALUT_NIGHT_HOUR_END_UTC;
    heidiConfig->nightBootCycles = (DEFAULT_BOOT_CYCLES * 2);
    heidiConfig->nightSleepMin   = DEFAULT_CYCLE_DURATION;
    heidiConfig->lastTimeDiffMs  = 0;
    heidiConfig->status          = (NEW_FENCE | RESET_INITS);
    heidiConfig->accThres1       = DEFALUT_ACCELERATION_THRESHOLD_1;
    heidiConfig->accAlertThres1  = DEFALUT_ACCEL_THRES_MAX_COUNT;
    heidiConfig->accThres2       = DEFALUT_ACCELERATION_THRESHOLD_2;
    heidiConfig->accAlertThres2  = DEFALUT_ACCEL_THRES_MAX_COUNT;
    heidiConfig->accNightFactor  = 100;  //equals to 1
    heidiConfig->alertFailCount  = 0;
    heidiConfig->telNo[0][0]     = 0xBB; //empty
    heidiConfig->telNo[1][0]     = 0xBB; //empty
  }
}

void initRTCData(bool reset){
  #ifdef TEST_RTC
  if(reset){ fillRTCbounary(); }
  #endif
  initConfig(reset);
  uint8_t* curSet;
  //_D(DebugPrintln("Init data sets", DEBUG_LEVEL_1); delay(50));
  int p=0;
  #ifdef USE_RTC_FAST_MEM
  RTCfastMemRead();
  curSet = (uint8_t*)fastMemBuffer;
  for (int i=0; i<FAST_MEM_DATA_SETS; i++){
    availableDataSet[p] = (t_SendData*)curSet;
    if(reset) { initDataSet(availableDataSet[p]); }
    curSet += DATA_SET_LEN;
    p++;
  }
  #endif
  #ifdef USE_RTC_SLOW_MEM
  curSet = (uint8_t*)&(RTC_SLOW_MEM[(RTC_DATA_SPACE_OFFSET + HEIDI_CONFIG_LENGTH_RTC)]);
  for (int i=0; i<SLOW_MEM_DATA_SETS; i++){
    availableDataSet[p] = (t_SendData*)curSet;
    if(reset) { initDataSet(availableDataSet[p]); }
    curSet += DATA_SET_LEN;
    p++;
  }
  #endif
  //_D(DebugPrintln("Init fence sets", DEBUG_LEVEL_1); delay(50);)
  curSet = (uint8_t*)&(RTC_SLOW_MEM[(RTC_DATA_SPACE_OFFSET + HEIDI_CONFIG_LENGTH_RTC + DATA_SET_SLOW_MEM_SPACE)]);
  for (int i=0; i<FENCE_MAX_POS; i++){
    FenceDataSet[i] = (t_FenceData*)curSet;
    if(reset) {
      FenceDataSet[i]->latitude  = EMPTY_POINT_VALUE;
      FenceDataSet[i]->longitude = EMPTY_POINT_VALUE;
    }
    curSet += FENCE_SET_LEN;
  }
  #ifdef USE_ULP
  //init ULP Variables
  if(reset){
    _D(DebugPrintln("Power on reset: reset all ULP vars!", DEBUG_LEVEL_1);)
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
    set_IIC_request(0);
    set_IIC_lock(0);
  }
  #endif
  _D(
    if(!reset){
      int c = 0;
      for(int i=0; i<MAX_DATA_SETS; i++){ if(!emptyDataSet(availableDataSet[i])) { c++; } }
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
  DataSet->GPShdop     = 0;
  DataSet->temperature = TEMPERATURE_NOT_SET; //-273,15 .. if this is the real temperature, the sheep is dead.. definitely ;-)
  DataSet->errCode     = 0;
  DataSet->satellites  = 0;
  DataSet->metersOut   = 0;
  DataSet->accThresCnt1   = 0;
  DataSet->accThresCnt2   = 0;

}
bool emptyDataSet(t_SendData* DataSet){
  return (DataSet->date == 0);
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
  _to->GPShdop      = _from->GPShdop   ;
  _to->errCode      = _from->errCode   ;
  _to->metersOut    = _from->metersOut ;
  _to->accThresCnt1 = _from->accThresCnt1 ;
  _to->accThresCnt2 = _from->accThresCnt2 ;

 }
/* this function is deprecated */
String generateSendLine(t_SendData* DataSet){
  //"TrackerID=0002.1235&Longitude=-13.344855&Latitude=51.006709&Altitude=305&Date=2019-06-09&Time=17:20:00&Battery=4.02",
  String result = "";
  if (!emptyDataSet(DataSet)){
    result = "TrackerID=";
    int l = String(herdeID()).length();
    for (int i=0; i<(4-l); i++){ result += "0";}
    result += String(herdeID()) + ".";
    l = String(animalID()).length();
    for (int i=0; i<(4-l); i++){ result += "0";}
    result += String(animalID());
    result += "&Longitude=" + String(IntToGeo(DataSet->longitude), 6);
    result += "&Latitude=" + String(IntToGeo(DataSet->latitude), 6);
    result += "&Altitude=" + String(DataSet->altitude);
    result += "&Date=" + DOSdateString(DataSet->date);
    result += "&Time=" + DOStimeString(DataSet->time);
    result += "&Battery=" + String((double(DataSet->battery) / 1000), 2);
    result += "&FreeValue1=" + String((int)DataSet->satellites);
    result += "&FreeValue2="  + String((float)DataSet->temperature / 100, 2);
    //result += "&FreeValue3="  + String(DataSet->errCode, HEX);
    result += "&FreeValue3="  + String(DataSet->accThresCnt1);
    result += "&FreeValue4="  + String((int)DataSet->GPShdop);
    result += "&FreeValue5="  + String((int)DataSet->metersOut);
  }
  return result;
}
/* this function is deprecated */
String generateMultiSendLine(int first, int last, int* setsDone){
  //short notation with multiple data sets
  //php.ini: max_input_var = 1000 by default (1 data set = 7..12 vars) -> up to 83 data sets possible
  //php.ini: post_max_size = 8M by default - SIM800 312k only! (1 data set = 90..150 byte)
  //don't forget to increase timeouts on web server side
  //"ID1=0002.1235&Lo1=-13.344855&La1=51.006709&Al1=305&Da1=2019-06-09&Ti1=17:20:00&Ba1=4.02&ID1=0002.1235&Lo1=...",
  t_SendData* DataSet;
  String result = "";
  int k = 0;
  for (int a=last; a<first; a--){
    DataSet = availableDataSet[a];
    if (!emptyDataSet(DataSet)) {
      k++;
	    if (k > 1) { result += "&"; }
	    result += "ID" + String(k) + "=";
      int l = String(herdeID()).length();
      for (int j=0; j<(4-l); j++){ result += "0";}
      result += String(herdeID()) + ".";
      l = String(animalID()).length();
      for (int j=0; j<(4-l); j++){ result += "0";}
      result += String(animalID());
      result += "&Lo" + String(k) + "=" + String(IntToGeo(DataSet->longitude), 6);
      result += "&La" + String(k) + "=" + String(IntToGeo(DataSet->latitude), 6);
      result += "&Al" + String(k) + "=" + String(DataSet->altitude);
      result += "&Da" + String(k) + "=" + DOSdateString(DataSet->date);
      result += "&Ti" + String(k) + "=" + DOStimeString(DataSet->time);
      result += "&Ba" + String(k) + "=" + String((double(DataSet->battery) / 1000), 2);
      result += "&F1" + String(k) + "=" + String((int)DataSet->satellites);
      result += "&F2" + String(k) + "=" + String((float)DataSet->temperature / 100, 2);
      //result += "&F3" + String(k) + "=" + String(DataSet->errCode, HEX);
      result += "&F3" + String(k) + "=" + String(DataSet->accThresCnt1);
      result += "&F4" + String(k) + "=" + String((int)DataSet->GPShdop);
      result += "&F5" + String(k) + "=" + String((int)DataSet->metersOut);
      if (k == 83) { break; } //1000 max POST vars reached
    }
  }
  *setsDone = k;
  return result;
}
String generateMulti64SendLine(int first, int last){
  //php.ini: max_input_var = 1000 by default (1 data set = 1 var) -> max 100 datasets -> no problem
  //php.ini: post_max_size = 8M by default - SIM800 312k only! (1 data set = 50 byte) -> no problem
  //don't forget to increase timeouts on web server side
  t_SendData* DataSet;
  String result = "";
  int k = 0;
  for (int a=last; a>=first; a--){
    DataSet = availableDataSet[a];
    if (!emptyDataSet(DataSet)) {
      k++;
      if (k > 1) { result += "&"; }
      _D(_PrintDataSet(DataSet, DEBUG_LEVEL_1));
      // why that complicated and not just memcopy?
      // push_data.phtml expects 8 bit count of values, 16 bit ID, 2x32bit coordinates
      // and than count-3 16 bit values - always 16 bit
      uint8_t hexbuffer[64];
      hexbuffer[0] = HEX_BUFFER_VALUES; //count of data values
      hexbuffer[1] = herdeID();
      hexbuffer[2] = animalID();  //1
      _copyInt32toBuffer(hexbuffer,HEX_BUFFER_OFFSET  +  0, DataSet->latitude); //2
      _copyInt32toBuffer(hexbuffer,HEX_BUFFER_OFFSET  +  4, DataSet->longitude); //3
      _copyInt16toBuffer(hexbuffer,HEX_BUFFER_OFFSET  +  8, DataSet->altitude); //4
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 10, DataSet->date); //5
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 12, DataSet->time); //6
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 14, DataSet->battery); //7
      _copyInt16toBuffer(hexbuffer,HEX_BUFFER_OFFSET  + 16, DataSet->temperature); //8
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 18, DataSet->errCode); //9
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 20, DataSet->satellites); //10
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 22, DataSet->GPShdop); //11
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 24, DataSet->accThresCnt1);  //12
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 26, DataSet->accThresCnt2);  //13
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 28, DataSet->metersOut);  //14 = HEX_BUFFER_VALUES
      _D(
        String HexStr = "";
        for(int i=0; i<HEX_BUFFER_LEN; i++){
          String _hex = String(hexbuffer[i], HEX);
          if (_hex.length() < 2) {_hex = "0" + _hex;}
          HexStr = HexStr + _hex;
        }
        DebugPrintln("bin: " + HexStr, DEBUG_LEVEL_2);
      )
      result += "X" + LenTwo(String(k)) + "=" + b64Encode(hexbuffer, HEX_BUFFER_LEN);
    }
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
      if(newSettingsB64(settingsData)){
        _D(DebugPrintln("new settings set", DEBUG_LEVEL_1));
        return true;
      } _D( else { DebugPrintln("setting settings failed", DEBUG_LEVEL_1); } )
    } _D( else { DebugPrintln("settings CRC error", DEBUG_LEVEL_1); } )
  } _D( else { DebugPrintln("settings data error", DEBUG_LEVEL_1); } )
  return false;
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

bool newSettingsB64(String b64){
  unsigned char buffer[256];
  int dataLen = b64Decode(b64, buffer);
  if (dataLen < 3) { return false; };
  if ( buffer[0] >= 1  ) { setNewSetting8(  buffer, 1, &heidiConfig->bootCycles, NEW_SETTINGS); }
  if ( buffer[0] >= 2  ) { setNewSettingU8( buffer, 3, &heidiConfig->nightBootCycles, NEW_SETTINGS); }
  if ( buffer[0] >= 3  ) { setNewSettingU8( buffer, 5, &heidiConfig->sleepMinutes, NEW_SETTINGS); }
  if ( buffer[0] >= 4  ) { setNewSettingU8( buffer, 7, &heidiConfig->nightSleepMin, NEW_SETTINGS); }
  if ( buffer[0] >= 5  ) { setNewSettingU8( buffer, 9, &heidiConfig->nightHourStart, NEW_SETTINGS); }
  if ( buffer[0] >= 6  ) { setNewSettingU8( buffer,11, &heidiConfig->nightHourEnd, NEW_SETTINGS); }
  if ( buffer[0] >= 7  ) { setNewSettingU16(buffer,13, &heidiConfig->distAlertThres, NEW_SETTINGS); }

  if ( buffer[0] >= 8  ) { setNewSettingU16(buffer,15, &heidiConfig->accThres1, NEW_ACC_DATA); }
  if ( buffer[0] >= 9  ) { setNewSettingU16(buffer,17, &heidiConfig->accAlertThres1, NEW_ACC_DATA); }
  if ( buffer[0] >= 10 ) { setNewSettingU16(buffer,19, &heidiConfig->accThres2, NEW_ACC_DATA); }
  if ( buffer[0] >= 11 ) { setNewSettingU16(buffer,21, &heidiConfig->accAlertThres2, NEW_ACC_DATA); }
  if ( buffer[0] >= 12 ) { setNewSettingU8( buffer,23, &heidiConfig->accNightFactor, NEW_ACC_DATA); }

  if (heidiConfig->sleepMinutes < MIN_CYCLE_DURATION) { heidiConfig->sleepMinutes = MIN_CYCLE_DURATION; }
  if (heidiConfig->nightSleepMin < MIN_CYCLE_DURATION) { heidiConfig->nightSleepMin = MIN_CYCLE_DURATION; }
  if (heidiConfig->sleepMinutes > MAX_CYCLE_DURATION) { heidiConfig->sleepMinutes = MAX_CYCLE_DURATION; }
  if (heidiConfig->nightSleepMin > MAX_CYCLE_DURATION) { heidiConfig->nightSleepMin = MAX_CYCLE_DURATION; }
  if (heidiConfig->bootCycles > MAX_BOOT_CYCLES) { heidiConfig->bootCycles = MAX_BOOT_CYCLES; }
  if (heidiConfig->nightBootCycles > MAX_BOOT_CYCLES) { heidiConfig->nightBootCycles = MAX_BOOT_CYCLES; }
  if (heidiConfig->bootCycles < 1) { heidiConfig->bootCycles = DEFAULT_BOOT_CYCLES; }
  if (heidiConfig->nightBootCycles < 1) { heidiConfig->nightBootCycles = DEFAULT_BOOT_CYCLES; }
  if ((heidiConfig->nightHourStart < 0) || (heidiConfig->nightHourStart > 23)) { heidiConfig->nightHourStart = 0; }
  // if we have a bad night end-Value we set start = end, means no night modus
  if ((heidiConfig->nightHourEnd < 0) || (heidiConfig->nightHourEnd > 23)) { heidiConfig->nightHourEnd = heidiConfig->nightHourStart; }
  // if no night modus - set them to zero
  if (heidiConfig->nightHourStart == heidiConfig->nightHourEnd) { heidiConfig->nightHourStart = 0; heidiConfig->nightHourEnd = 0; }
  return true;
}

void setNewSetting8(uint8_t *buffer, int pufferPos, int8_t *setting, uint32_t status){
  int8_t dummy = _copyBufferToUInt16(buffer, pufferPos);
  if(dummy != *setting){
    *setting = dummy;
    setState(status);
    _DD( DebugPrintln("set [" + String(pufferPos) + "] to " + String(heidiConfig->nightBootCycles), DEBUG_LEVEL_3); )
  } _DD( else { DebugPrintln("setting " + String(pufferPos) +  "keeps the same", DEBUG_LEVEL_3); } )
}
void setNewSettingU8(uint8_t *buffer, int pufferPos, uint8_t *setting, uint32_t status){
  uint8_t dummy = _copyBufferToUInt16(buffer, pufferPos);
  if(dummy != *setting){
    *setting = dummy;
    setState(status);
    _DD( DebugPrintln("set [" + String(pufferPos) + "] to " + String(heidiConfig->nightBootCycles), DEBUG_LEVEL_3); )
  } _DD( else { DebugPrintln("setting " + String(pufferPos) +  "keeps the same", DEBUG_LEVEL_3); } )
}void setNewSettingU16(uint8_t *buffer, int pufferPos, uint16_t *setting, uint32_t status){
  uint16_t dummy = _copyBufferToUInt16(buffer, pufferPos);
  if(dummy != *setting){
    *setting = dummy;
    setState(status);
    _DD( DebugPrintln("set [" + String(pufferPos) + "] to " + String(heidiConfig->nightBootCycles), DEBUG_LEVEL_3); )
  } _DD( else { DebugPrintln("setting " + String(pufferPos) +  "keeps the same", DEBUG_LEVEL_3); } )
}
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

void cleanUpDataSets(bool TransmissionFailed){
  //mark transmission result
  for(int i=0; i<MAX_DATA_SETS; i++){
    if (!emptyDataSet(availableDataSet[i])){
      if (TransmissionFailed)  { setError(availableDataSet[i], E_GSM_TRANSMISSION_FAILED); }
      else { rmError(availableDataSet[i], E_GSM_TRANSMISSION_FAILED); }
    }
  }
  //delete all transmitted data
  for(int i=0; i<MAX_DATA_SETS; i++){
    if(getError(availableDataSet[i], E_GSM_TRANSMISSION_FAILED) == false){
      initDataSet(availableDataSet[i]);
    }
  }
  //concentrate
  int k = 0;
  for(int i=0; i<MAX_DATA_SETS; i++){
    if(!emptyDataSet(availableDataSet[i]) && (k < i) && emptyDataSet(availableDataSet[k])){
      copyDataSet(availableDataSet[i], availableDataSet[k]);
      initDataSet(availableDataSet[i]);
    }
    if(!emptyDataSet(availableDataSet[k])){ k++; }
  }
}
void freeFirstDataSet(void){
  if(emptyDataSet(availableDataSet[0])) { return; }
  for(int i=(MAX_DATA_SETS - 2); i >= 0; i--){
    if(!emptyDataSet(availableDataSet[i])){
      copyDataSet(availableDataSet[i], availableDataSet[i+1]);
      initDataSet(availableDataSet[i]);
    }
  }
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
  return((heidiConfig->status & which) != 0);
}
void setState(uint32_t which){
  heidiConfig->status |= which;
}
void clrState(uint32_t which){
  heidiConfig->status &= ~which;
}

int32_t getCycleTimeMS(void){
  if(_night()){
    return heidiConfig->nightSleepMin * 60000;
  }
  return heidiConfig->sleepMinutes * 60000;
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
  BaseType_t rc = xTaskCreatePinnedToCore(fastMemReadTask, "fastMemReadTask", FAST_MEM_TASK_HEAP_SIZE, NULL, 1, &task_fmem_read, 0);
  if (rc != pdPASS){
    _D(DebugPrintln("unable to create fast mem read Task. " + String(rc), DEBUG_LEVEL_1); delay(50);)
    return false;
  }
  while (task_fmem_read != NULL){ vTaskDelay(100); }
  //_D( PrintRTCFastMemBufferBoundaries(); )
  return true;
}
bool RTCfastMemWrite(void){
  BaseType_t rc = xTaskCreatePinnedToCore(fastMemWriteTask, "fastMemWriteTask", FAST_MEM_TASK_HEAP_SIZE, NULL, 1, &task_fmem_write, 0);
  if (rc != pdPASS){
    _D(DebugPrintln("unable to create fast mem read Task. " + String(rc), DEBUG_LEVEL_1); delay(50);)
    return false;
  }
  while (task_fmem_write != NULL){ vTaskDelay(100); }
  return true;
}

void fastMemReadTask(void *pvParameters) {
  _D(Serial.begin(115200);
    DebugPrintln("Fast mem read task created on core " + String(xPortGetCoreID()), DEBUG_LEVEL_1); delay(50);
    int t = millis();
  )
  for(int i=0; i<RTC_FAST_MEM_SIZE_32; i++){ fastMemBuffer[i] = _realFastMem[i];  }
  _D(DebugPrintln("RTC fast mem read " + String (millis()-t), DEBUG_LEVEL_1); delay(50);)
  task_fmem_read = NULL;
  vTaskDelete(NULL);

}

void fastMemWriteTask(void *pvParameters) {
  _D(Serial.begin(115200);
    DebugPrintln("Fast mem write task created on core " + String(xPortGetCoreID()), DEBUG_LEVEL_1); delay(50);
    int t = millis();
  )
  for(int i=0; i<RTC_FAST_MEM_SIZE_32; i++){ _realFastMem[i] = fastMemBuffer[i]; }
  _D(DebugPrintln("RTC fast mem written " + String (millis()-t), DEBUG_LEVEL_1); delay(50);)
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
  delay(100);
  a = 0;
  for(int i=(RTC_FAST_MEM_SIZE_32-32); i<RTC_FAST_MEM_SIZE_32; i++){
    if ((a++) == 8) { DebugPrintln(hexString8(fastMemBuffer[i]), DEBUG_LEVEL_1); a = 0; }
    else { DebugPrint(hexString8(fastMemBuffer[i]) + ", ", DEBUG_LEVEL_1); }
  }
  delay(100);
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
