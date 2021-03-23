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

void initRTCData(bool reset){
  #ifdef TEST_RTC
  if(reset){ fillRTCbounary(); }
  #endif

  heidiConfig = (t_ConfigData*)&(RTC_SLOW_MEM[RTC_DATA_SPACE_OFFSET]);
  if(reset) {
    _D(DebugPrintln("RESET all RTC data", DEBUG_LEVEL_1);)
    heidiConfig->bootCount    = START_FROM_RESET;
    heidiConfig->bootCycles   = BOOT_CYCLES;
    heidiConfig->sleepMinutes = (SLEEP_DURATION_MSEC / 60000);
    heidiConfig->nightHourStart = NIGHT_HOUR_START;
    heidiConfig->nightHourEnd   = NIGHT_HOUR_END;
    heidiConfig->nightBootCycles= BOOT_CYCLES;
    heidiConfig->nightSleepMin  = (SLEEP_DURATION_MSEC / 30000);
    heidiConfig->lastTimeDiffMs = 0;
  }
  uint8_t* curSet;
  //_D(DebugPrintln("Init data sets", DEBUG_LEVEL_1));
  //_D(delay(100));
  curSet = (uint8_t*)&(RTC_SLOW_MEM[(RTC_DATA_SPACE_OFFSET + HEIDI_CONFIG_LENGTH_RTC)]);
  for (int i=0; i<MAX_DATA_SETS; i++){
    availableDataSet[i] = (t_SendData*)curSet;
    if(reset) { initDataSet(availableDataSet[i]); }
    curSet += DATA_SET_LEN;
  }
  //_D(DebugPrintln("Init fence sets", DEBUG_LEVEL_1));
  //_D(delay(100));
  curSet = (uint8_t*)&(RTC_SLOW_MEM[(RTC_DATA_SPACE_OFFSET + HEIDI_CONFIG_LENGTH_RTC + DATA_SET_MEM_SPACE_RTC)]);
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
    set_accel_excnt1_ULP(1);
    set_accel_exthr1_ULP(ACCEL_LO_THRESHOLD);
    set_accel_wake1_ULP(ACCEL_LO_CNT_WAKE);
    set_accel_excnt2_ULP(2);
    set_accel_exthr2_ULP(ACCEL_HI_THRESHOLD);
    set_accel_wake2_ULP(ACCEL_HI_CNT_WAKE);
  }
#endif
}

void initDataSet(t_SendData* DataSet){
  DataSet->longitude   = 0;
  DataSet->latitude    = 0;
  DataSet->altitude    = 0;
  DataSet->date        = 0;
  DataSet->time        = 0;
  DataSet->battery     = 0;
  DataSet->GPShdop     = 0;
  DataSet->temperature = NO_TEMPERATURE; //-127 .. sheep is dead.. definitely ;-)
  DataSet->errCode     = 0;
  DataSet->satellites  = 0;
  DataSet->metersOut   = 0;
  DataSet->accThres1   = 0;
  DataSet->accThres2   = 0;

}
bool emptyDataSet(t_SendData* DataSet){
  return (DataSet->date == 0);
}
void copyDataSet(t_SendData* _from, t_SendData* _to){
  _to->longitude  = _from->longitude ;
  _to->latitude   = _from->latitude  ;
  _to->altitude   = _from->altitude  ;
  _to->date       = _from->date      ;
  _to->time       = _from->time      ;
  _to->battery    = _from->battery   ;
  _to->temperature= _from->temperature;
  _to->satellites = _from->satellites;
  _to->GPShdop    = _from->GPShdop   ;
  _to->errCode    = _from->errCode   ;
  _to->metersOut  = _from->metersOut ;
  _to->accThres1  = _from->accThres1 ;
  _to->accThres2  = _from->accThres2 ;

 }

String generateSendLine(t_SendData* DataSet){
  //"TrackerID=0002.1235&Longitude=-13.344855&Latitude=51.006709&Altitude=305&Date=2019-06-09&Time=17:20:00&Battery=4.02",
  String result = "";
  if (!emptyDataSet(DataSet)){
    result = "TrackerID=";
    int8_t hour_shift = GetLocalTimeHourShift(); //local time setting to be done
    int l = String(herdeID()).length();
    for (int i=0; i<(4-l); i++){ result += "0";}
    result += String(herdeID()) + ".";
    l = String(animalID()).length();
    for (int i=0; i<(4-l); i++){ result += "0";}
    result += String(animalID());
    result += "&Longitude=" + String(IntToGeo(DataSet->longitude), 6);
    result += "&Latitude=" + String(IntToGeo(DataSet->latitude), 6);
    result += "&Altitude=" + String(DataSet->altitude);
    result += "&Date=" + String(dosYear(DataSet->date)) + "-" + LenTwo(String(dosMonth(DataSet->date))) + "-" + LenTwo(String(dosDay(DataSet->date)));
    result += "&Time=" + LenTwo(String(dosHour(DataSet->time))) + ":" + LenTwo(String(dosMinute(DataSet->time))) + ":" + LenTwo(String(dosSecond(DataSet->time)));
    result += "&Battery=" + String((double(DataSet->battery) / 1000), 2);
    result += "&FreeValue1=" + String((int)DataSet->satellites);
    result += "&FreeValue2="  + String((float)DataSet->temperature / 100, 2);
    //result += "&FreeValue3="  + String(DataSet->errCode, HEX);
    result += "&FreeValue3="  + String(DataSet->accThres1);
    result += "&FreeValue4="  + String((int)DataSet->GPShdop);
    result += "&FreeValue5="  + String((int)DataSet->metersOut);
  }
  return result;
}
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
    if (emptyDataSet(DataSet)) { continue; }
    if ((a < getBootCycles()) || ((DataSet->errCode & GSM_TRANSMISSION_FAILED) == GSM_TRANSMISSION_FAILED)) {
      k++;
	    if (k > 1) { result += "&"; }
	    result += "ID" + String(k) + "=";
      int8_t hour_shift = GetLocalTimeHourShift(); //local time setting to be done
      int l = String(herdeID()).length();
      for (int j=0; j<(4-l); j++){ result += "0";}
      result += String(herdeID()) + ".";
      l = String(animalID()).length();
      for (int j=0; j<(4-l); j++){ result += "0";}
      result += String(animalID());
      result += "&Lo" + String(k) + "=" + String(IntToGeo(DataSet->longitude), 6);
      result += "&La" + String(k) + "=" + String(IntToGeo(DataSet->latitude), 6);
      result += "&Al" + String(k) + "=" + String(DataSet->altitude);
      result += "&Da" + String(k) + "=" + String(dosYear(DataSet->date)) + "-" + LenTwo(String(dosMonth(DataSet->date))) + "-" + LenTwo(String(dosDay(DataSet->date)));
      result += "&Ti" + String(k) + "=" + LenTwo(String(dosHour(DataSet->time))) + ":" + LenTwo(String(dosMinute(DataSet->time))) + ":" + LenTwo(String(dosSecond(DataSet->time)));
      result += "&Ba" + String(k) + "=" + String((double(DataSet->battery) / 1000), 2);
      result += "&F1" + String(k) + "=" + String((int)DataSet->satellites);
      result += "&F2" + String(k) + "=" + String((float)DataSet->temperature / 100, 2);
      //result += "&F3" + String(k) + "=" + String(DataSet->errCode, HEX);
      result += "&F3" + String(k) + "=" + String(DataSet->accThres1);
      result += "&F4" + String(k) + "=" + String((int)DataSet->GPShdop);
      result += "&F5" + String(k) + "=" + String((int)DataSet->metersOut);
    }
    if (k == 83) { break; } //1000 max POST vars reached
  }
  *setsDone = k;
  return result;
}
String generateMulti64SendLine(int first, int last)
{
  //php.ini: max_input_var = 1000 by default (1 data set = 1 var) -> max 100 datasets -> no problem
  //php.ini: post_max_size = 8M by default - SIM800 312k only! (1 data set = 50 byte) -> no problem
  //don't forget to increase timeouts on web server side
  t_SendData* DataSet;
  String result = "";
  int k = 0;
  for (int a=last; a>=first; a--){
    DataSet = availableDataSet[a];
    if (emptyDataSet(DataSet)) { continue; }
      if ((a < getBootCycles()) || ((DataSet->errCode & GSM_TRANSMISSION_FAILED) == GSM_TRANSMISSION_FAILED)) {
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
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 24, DataSet->accThres1);  //12
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 26, DataSet->accThres2);  //13
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 28, DataSet->metersOut);  //14 = HEX_BUFFER_VALUES

      String HexStr = "";
      for(int i=0; i<HEX_BUFFER_LEN; i++){
        String _hex = String(hexbuffer[i], HEX);
        if (_hex.length() < 2) {_hex = "0" + _hex;}
        HexStr = HexStr + _hex;
      }
      _D(DebugPrintln("bin: " + HexStr, DEBUG_LEVEL_2));
      result += "X" + LenTwo(String(k)) + "=" + b64Encode(hexbuffer, HEX_BUFFER_LEN);
    }
  }
  return result;
}

bool setSettingsFromHTTPresponse(String response)
{
  String settingsData = GetCSVvalue(response, 4);
  String settingsCRC = GetCSVvalue(response, 5);
  uint16_t c_crc = crc16F(settingsData);
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

bool newSettingsB64(String b64){
  unsigned char buffer[256];
  int dataLen = b64Decode(b64, buffer);
  if (dataLen < 3) { return false; };
  if ( buffer[0] >= 1 ) { heidiConfig->bootCycles       = _copyBufferToUInt16(buffer, 1); _DD( DebugPrintln("settings bootCycles       " + String(heidiConfig->bootCycles), DEBUG_LEVEL_3); )}
  if ( buffer[0] >= 2 ) { heidiConfig->nightBootCycles  = _copyBufferToUInt16(buffer, 3); _DD( DebugPrintln("settings nightBootCycles  " + String(heidiConfig->nightBootCycles), DEBUG_LEVEL_3); )}
  if ( buffer[0] >= 3 ) { heidiConfig->sleepMinutes     = _copyBufferToUInt16(buffer, 5); _DD( DebugPrintln("settings sleepMinutes     " + String(heidiConfig->sleepMinutes), DEBUG_LEVEL_3); )}
  if ( buffer[0] >= 4 ) { heidiConfig->nightSleepMin    = _copyBufferToUInt16(buffer, 7); _DD( DebugPrintln("settings nightSleepMin    " + String(heidiConfig->nightSleepMin), DEBUG_LEVEL_3); )}
  if ( buffer[0] >= 5 ) { heidiConfig->nightHourStart   = _copyBufferToUInt16(buffer, 9); _DD( DebugPrintln("settings nightHourStart   " + String(heidiConfig->nightHourStart), DEBUG_LEVEL_3); )}
  if ( buffer[0] >= 6 ) { heidiConfig->nightHourEnd     = _copyBufferToUInt16(buffer,11); _DD( DebugPrintln("settings nightHourEnd     " + String(heidiConfig->nightHourEnd), DEBUG_LEVEL_3); )}
  if ( buffer[0] >= 7 ) { heidiConfig->distAlertThres   = _copyBufferToUInt16(buffer,13); _DD( DebugPrintln("settings distAlertThres   " + String(heidiConfig->distAlertThres), DEBUG_LEVEL_3); )}
  if ( buffer[0] >= 8 ) { heidiConfig->accThres1        = _copyBufferToUInt16(buffer,15); _DD( DebugPrintln("settings accThres1        " + String(heidiConfig->accThres1), DEBUG_LEVEL_3); )}
  if ( buffer[0] >= 9 ) { heidiConfig->accAlertThres1   = _copyBufferToUInt16(buffer,17); _DD( DebugPrintln("settings accAlertThres1   " + String(heidiConfig->accAlertThres1), DEBUG_LEVEL_3); )}
  if ( buffer[0] >= 10) { heidiConfig->accThres2        = _copyBufferToUInt16(buffer,19); _DD( DebugPrintln("settings accThres2        " + String(heidiConfig->accThres2), DEBUG_LEVEL_3); )}
  if ( buffer[0] >= 11) { heidiConfig->accAlertThres2   = _copyBufferToUInt16(buffer,21); _DD( DebugPrintln("settings accAlertThres2   " + String(heidiConfig->accAlertThres2), DEBUG_LEVEL_3); )}
  return true;
}

void cleanUpDataSets(bool TransmissionFailed){
  //mark transmission result
  for(int i=0; i<MAX_DATA_SETS; i++){
    if (!emptyDataSet(availableDataSet[i])){
      if (TransmissionFailed)  { setError(availableDataSet[i], GSM_TRANSMISSION_FAILED); }
      else { rmError(availableDataSet[i], GSM_TRANSMISSION_FAILED); }
    }
  }
  //delete all transmitted data
  for(int i=0; i<MAX_DATA_SETS; i++){
    if(getError(availableDataSet[i], GSM_TRANSMISSION_FAILED) == false){
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


void testData()
{
  _D(
     String testCSV1    = ";;3;45;";
     String testCSV2   = "1;2;3;4;5";

     _D(DebugPrintln("testCSV1: " + testCSV1, DEBUG_LEVEL_1));
     for(int i=1; i<7; i++){
       _D(DebugPrintln(String(i) + ": '" + GetCSVvalue(testCSV1,i) + "'", DEBUG_LEVEL_1));
     }
     _D(DebugPrintln("testCSV2: " + testCSV2, DEBUG_LEVEL_1));
     for(int i=1; i<6; i++){
       _D(DebugPrintln(String(i) + ": '" + GetCSVvalue(testCSV2,i) + "'", DEBUG_LEVEL_1));
     }
  )
}

void getRTCDataSpace(uint8_t** buffer){
	*buffer = (uint8_t*)&(RTC_SLOW_MEM[RTC_DATA_SPACE_OFFSET]);
}

int8_t getBootCycles(void){
  if(_night()){
    return heidiConfig->nightBootCycles;
  }
  return heidiConfig->bootCycles;
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


String DateString(tm timestamp){
  return String(timestamp.tm_year) + "-" + LenTwo(String(timestamp.tm_mon)) + "-" + LenTwo(String(timestamp.tm_mday));
}
String TimeString(tm timestamp){
  return LenTwo(String(timestamp.tm_hour)) + ":" + LenTwo(String(timestamp.tm_min)) + ":"+ LenTwo(String(timestamp.tm_sec));
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

#ifdef TEST_RTC

#define RTC_TEST_PATTERN 0x55AA55AA
#define RTC_TEST_MAX_MEM 1124

void testRTC(t_SendData* currentDataSet, tm* bootTime){
  currentDataSet->date = dosDate(bootTime->tm_year, bootTime->tm_mon, bootTime->tm_mday);
  currentDataSet->time = dosTime(bootTime->tm_hour, bootTime->tm_min, bootTime->tm_sec);
  _D(DebugPrintln("RTC ULP_MEM_SIZE: " + String(ACCEL_ULP_MEM_SIZE) + ", ULP_CODE_SIZE: " + String(ACCEL_ULP_CODE_SIZE) + ", ULP_DATA_SIZE: " + String(ACCEL_ULP_DATA_SIZE), DEBUG_LEVEL_2);)
  _D(DebugPrintln("RTC config address: 0x" + String((uint32_t)heidiConfig, HEX), DEBUG_LEVEL_2);)
  _PrintHeidiConfig(DEBUG_LEVEL_2);
  _PrintHeidiAccParams(DEBUG_LEVEL_2);
  _PrintShortSummary(DEBUG_LEVEL_2);
  _PrintFence(DEBUG_LEVEL_2);
  testRTCbounary();
  //goto_sleep(60000);
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

