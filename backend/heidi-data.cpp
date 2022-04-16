/*
 * heidi-data.cpp
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */
#include <Arduino.h>
#include <rom/rtc.h>
#include <esp32/ulp.h>
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
static TaskHandle_t accessLockHandle = NULL;

void initConfig(bool reset){
  heidiConfig = (t_ConfigData*)&(RTC_SLOW_MEM[RTC_DATA_SPACE_OFFSET]);
  if(reset) {
    _D(DebugPrintln("Power on reset: reset all RTC data", DEBUG_LEVEL_1);)
    heidiConfig->bootNumber        = START_FROM_RESET;
    heidiConfig->c.bootCycles      = DEFAULT_BOOT_CYCLES;
    heidiConfig->c.sleepMinutes    = DEFAULT_CYCLE_DURATION;
    heidiConfig->c.nightHourStart  = DEFALUT_NIGHT_HOUR_START_UTC;
    heidiConfig->c.nightHourEnd    = DEFALUT_NIGHT_HOUR_END_UTC;
    heidiConfig->c.nightBootCycles = (DEFAULT_BOOT_CYCLES * 2);
    heidiConfig->c.nightSleepMin   = DEFAULT_CYCLE_DURATION;
    heidiConfig->lastTimeDiffMs    = 0;
    heidiConfig->c.distAlertThres  = DEFAULT_DIST_ALERT_THRES;
    heidiConfig->status            = (NEW_FENCE | RESET_INITS);
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
  return crc16D(buffer, CONF_DATA_C_PAYLOAD_SIZE); //sizeof would deliver 18
}

void initRTCData(bool reset){
  dtAcc_t dtAcc;
  if((dtAcc = getDataSetAccess()) == DATA_NO_ACCESS){_D(DebugPrintln("initRTCData: no data access ", DEBUG_LEVEL_1);) return; }
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
      for(int i=0; i<allDataSets; i++){ if(!isEmptyDataSet(availableDataSet[i])) { c++; } }
      DebugPrintln("Used data sets: " + String(c), DEBUG_LEVEL_2);
    }
  )
  if(dtAcc == DATA_PRIMARY_ACCESS){ freeDataSetAccess(); }
}

void initDataSet(t_SendData* DataSet){
  dtAcc_t dtAcc;
  if((dtAcc = getDataSetAccess()) == DATA_NO_ACCESS) {_D(DebugPrintln("initDataSet: no data access ", DEBUG_LEVEL_1);) return; }
  DataSet->animalID    = 0;      //address 0 = empty data set
  DataSet->longitude   = 0;
  DataSet->latitude    = 0;
  DataSet->altitude    = 0;
  DataSet->date        = 0;
  DataSet->time        = 0;
  DataSet->battery     = 0;
  DataSet->GPSpDOP     = 0;
  DataSet->temperature = TEMPERATURE_NOT_SET; //-128 .. if this is the real temperature, the sheep is dead.. definitely ;-)
  DataSet->errCode     = 0;
  DataSet->satellites  = 0;
  DataSet->metersOut   = 0;
  DataSet->accThresCnt1   = 0;
  DataSet->accThresCnt2   = 0;
  DataSet->accThCnt1Spr   = 0;
  DataSet->accThCnt2Spr   = 0;
  if(dtAcc == DATA_PRIMARY_ACCESS){ freeDataSetAccess(); }
}

void copyDataSet(t_SendData* _from, t_SendData* _to){
  _to->longitude    = _from->longitude ;
  _to->latitude     = _from->latitude  ;
  _to->altitude     = _from->altitude  ;
  _to->date         = _from->date      ;
  _to->time         = _from->time      ;
  _to->battery      = _from->battery   ;
  _to->temperature  = _from->temperature;
  _to->animalID     = _from->animalID;
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
 * find a data set in the RTC data set storage by its copy
 *   - _which points to a copy of the data set
 *   - returns the number of the data set or DATA_SET_NOT_FOUND
 *
 * Why this? Since more then one task can write/move/erase/add data sets we should
 * consider that the data set we read from one position may not longer be there
 */
int findDataSet(t_SendData* _which){
  int i = 0;
  bool found = false;
  while (i < allDataSets){
    found = true;
    if(_which->longitude != availableDataSet[i]->longitude) { found &= false; }
    if(_which->latitude  != availableDataSet[i]->latitude)  { found &= false; }
    if(_which->altitude  != availableDataSet[i]->altitude)  { found &= false; }
    if(_which->date      != availableDataSet[i]->date)      { found &= false; }
    if(_which->time      != availableDataSet[i]->time)      { found &= false; }
    if(_which->animalID  != availableDataSet[i]->animalID)  { found &= false; }
    //should be enough to be sure
    if(found){ break; }
    i++;
  }
  if(found){ return i; }
  return DATA_SET_NOT_FOUND;
}

int findFreeDataSet(int from){
  for(int i = from; i < allDataSets; i++){
    if(isEmptyDataSet(availableDataSet[i])){ return i; }
  }
  return DATA_SET_NOT_FOUND;
}

/*
 * erase a data set in the RTC data set storage, identified by its copy
 *   - _which points to a copy of the data set
 *   returns true if successful
 *
 * Why this? Since more then one task can write/move/erase/add data sets we should
 * consider that the data set we read from one position may not longer be there
 */
bool eraseDataSet(t_SendData* _which){
  dtAcc_t dtAcc;
  bool rc = false;
  if((dtAcc = getDataSetAccess()) == DATA_NO_ACCESS) {_D(DebugPrintln("eraseDataSet: no data access ", DEBUG_LEVEL_1);) return rc; }
  int x = findDataSet(_which);
  if(x != DATA_SET_NOT_FOUND){ initDataSet(availableDataSet[x]); rc = true;}
  if(dtAcc == DATA_PRIMARY_ACCESS){ freeDataSetAccess(); }
  return rc;
}

/*
 * erase a data sets in the RTC data set storage, identified by its copies in buffer
 *   - buffer points to an array of the data sets
 *   - cnt = amount of the data sets
 *   returns actual erased data sets
 */
int eraseDataSets(t_SendData* buffer, int cnt){
  dtAcc_t dtAcc;
  int done = 0;
  if((dtAcc = getDataSetAccess()) == DATA_NO_ACCESS) {_D(DebugPrintln("eraseDataSets: no data access ", DEBUG_LEVEL_1);) return 0; }
  for(int i=0; i<cnt; i++){
    int x = findDataSet(&buffer[i]);
    if(x != DATA_SET_NOT_FOUND){ initDataSet(availableDataSet[x]); }
  }
  if(dtAcc == DATA_PRIMARY_ACCESS){ freeDataSetAccess(); }
  return done;
}

/*
 * add a data set in the RTC data set storage
 *   - _new points to the data set to be added
 *   returns true if successful
 *
 * add copies the data set onto 2nd position. 1st is the current data set, above are old data sets
 */
bool addDataSet(t_SendData* _new){
  dtAcc_t dtAcc;
  bool rc = false;
  if((dtAcc = getDataSetAccess()) == DATA_NO_ACCESS) {_D(DebugPrintln("addDataSet: no data access ", DEBUG_LEVEL_1);) return rc; }
  if (freeDataSet(1)){ copyDataSet(_new, availableDataSet[1]); rc = true; }
  _D( else { DebugPrintln("addDataSet: unable to free 2nd data set", DEBUG_LEVEL_1); })
  if(dtAcc == DATA_PRIMARY_ACCESS){ freeDataSetAccess(); }
  return rc;
}

/*
 * add data sets in the RTC data set storage
 *   - _new points to an data set array
 *   returns actual added data sets
 *
 * add copies the data set onto 2nd position. 1st is the current data set, above are old data sets
 */
int addDataSets(t_SendData* buffer, int cnt){
  dtAcc_t dtAcc;
  int done = 0;
  if((dtAcc = getDataSetAccess()) == DATA_NO_ACCESS) {_D(DebugPrintln("addDataSets: no data access ", DEBUG_LEVEL_1);) return 0; }
  for(int i=0; i<cnt; i++){
    if (freeDataSet(1)) { copyDataSet(&buffer[i], availableDataSet[1]); done++; }
    _D( else { DebugPrintln("addDataSets: unable to free 2nd data set", DEBUG_LEVEL_1); })
  }
  if(dtAcc == DATA_PRIMARY_ACCESS){ freeDataSetAccess(); }
  return done;
}

/*
 * packUpDataSets removes free data sets between used ones and returns count of used data sets
 * Parameter: start position of packing
 */
int packUpDataSets(){
  int k = 1;
  dtAcc_t dtAcc;
  if((dtAcc = getDataSetAccess()) == DATA_NO_ACCESS){_D(DebugPrintln("packUpDataSets: no data access ", DEBUG_LEVEL_1);) return 0; }
  for(int i=1; i<allDataSets; i++){ //data set 0 is always the current set
    if(!isEmptyDataSet(availableDataSet[i]) && (k < i) && isEmptyDataSet(availableDataSet[k])){
      copyDataSet(availableDataSet[i], availableDataSet[k]);
      initDataSet(availableDataSet[i]);
    }
    if(!isEmptyDataSet(availableDataSet[k])){ k++; }
  }
  if(dtAcc == DATA_PRIMARY_ACCESS){ freeDataSetAccess(); }
  return k;
}

/*
 * copies n data sets (oldest first) from RTC data set storage to given buffer
 *   - n - amount of data
 *   - buffer - buffer where to copy
 *   - size - size of buffer
 *   - lastSet - pointer to the last data set gotten - if set and not empty, getNextnDataSets
 *     will search from the following one, if lastSet was found. Otherwise getNextnDataSets will
 *     search all data sets. lastSet may be part of buffer.
 *   returns the actual copied data sets
 */
int getNextnDataSets(int n, t_SendData* buffer, int size, t_SendData* lastSet){
  dtAcc_t dtAcc;
  if(buffer == NULL){ return 0; }
  if((dtAcc = getDataSetAccess()) == DATA_NO_ACCESS){_D(DebugPrintln("packUpDataSets: no data access ", DEBUG_LEVEL_1);) return 0; }
  int availSets = packUpDataSets();
  int lastSetNo = availSets;
  if(lastSet != NULL){
    if(!isEmptyDataSet(lastSet)){
      lastSetNo = findDataSet(lastSet);
      if(lastSetNo == DATA_SET_NOT_FOUND) { lastSetNo = availSets; }
    }
  }
  int copySets = n;
  int copiedSets = 0;
  if(copySets > lastSetNo) { copySets = lastSetNo; }
  if(size >= (copySets * sizeof(t_SendData))){
    for(int i=0; i < copySets; i++){
      if (!isEmptyDataSet(availableDataSet[lastSetNo-i-1])){ //maybe set 0 is ready, maybe not
        copyDataSet(availableDataSet[lastSetNo-i-1], &buffer[i]);
        copiedSets++;
      }
    }
  } _D( else { DebugPrintln("packUpDataSets: buffer too small ", DEBUG_LEVEL_1);} )
  if(dtAcc == DATA_PRIMARY_ACCESS){ freeDataSetAccess(); }
  return copiedSets;
}

/*
 * sets error in data sets in the RTC data set storage, identified by its copies in buffer
 *   - buffer points to an array of the data sets
 *   - code = error code
 *
 */
void setErrorToDataSets(t_SendData* sets, int cnt, uint16_t code){
  dtAcc_t dtAcc;
  if((dtAcc = getDataSetAccess()) == DATA_NO_ACCESS){_D(DebugPrintln("setErrorToDataSets: no data access ", DEBUG_LEVEL_1);) return; }
  for(int i=0; i<cnt; i++){
    int x = findDataSet(&sets[i]);
    if(x != DATA_SET_NOT_FOUND){ setError(availableDataSet[x], code); }
  }
  if(dtAcc == DATA_PRIMARY_ACCESS){ freeDataSetAccess(); }
}

/*
 * freeDataSet shifts free the wanted data position
 *   - _which - position to be shifted free
 *
 */
bool freeDataSet(int _which){
  dtAcc_t dtAcc;
  if((_which < 0) || (_which >= allDataSets)){ return false; }
  if(isEmptyDataSet(availableDataSet[_which])) { return true; }
  if((dtAcc = getDataSetAccess()) == DATA_NO_ACCESS){ return false; }
  int firstfree = findFreeDataSet(_which + 1);
  if(firstfree == DATA_SET_NOT_FOUND) {firstfree = allDataSets - 1; } //last one need to be trashed
  for(int i=(firstfree - 1); i >= _which; i--){
    if(!isEmptyDataSet(availableDataSet[i])){
      copyDataSet(availableDataSet[i], availableDataSet[i+1]);
    }
  }
  initDataSet(availableDataSet[_which]);
  if(dtAcc == DATA_PRIMARY_ACCESS){ freeDataSetAccess(); }
  return true;
}

bool isEmptyDataSet(t_SendData* DataSet){
  return (DataSet->animalID == 0);
}

void initDataSets(t_SendData** sets, int first, int last){
  int current = first;
  if((first < 0) || (first >= allDataSets)){ return; }
  if((last < 0) || (last >= allDataSets)){ return; }
  dtAcc_t dtAcc;
  if((dtAcc = getDataSetAccess()) == DATA_NO_ACCESS){_D(DebugPrintln("initDataSets: no data access ", DEBUG_LEVEL_1);) return; }
  while (current <= last){
    t_SendData* set = sets[current];
    initDataSet(set);
    current++;
  }
  if(dtAcc == DATA_PRIMARY_ACCESS){ freeDataSetAccess(); }
}

/*
 * following functions are needed to manage the access to data sets, which
 * may be written from main task and lora task in parallel
 */
static portMUX_TYPE accMux = portMUX_INITIALIZER_UNLOCKED;
bool getPrimaryDataSetAccess(void){
  int i = 0;
  bool rc = false;
  if (accessLockHandle == xTaskGetCurrentTaskHandle()) { return true; }
  while((i < 100) && (!rc)){
    taskENTER_CRITICAL(&accMux);
    if (accessLockHandle == NULL) {
      accessLockHandle = xTaskGetCurrentTaskHandle();
      rc = true;
    }
    taskEXIT_CRITICAL(&accMux);
    if(rc){ break; }
    i++;
    pause(10);
  }
  return rc;
}
dtAcc_t getDataSetAccess(void){
  if (accessLockHandle == xTaskGetCurrentTaskHandle()) { return DATA_ACCESS; }
  if (accessLockHandle == NULL) {
    if (getPrimaryDataSetAccess()) { return DATA_PRIMARY_ACCESS; }
  }
  return DATA_NO_ACCESS;
}
void freeDataSetAccess(void){
  if (accessLockHandle == xTaskGetCurrentTaskHandle()) { accessLockHandle = NULL; }
}

#ifdef HEIDI_GATEWAY
/*
 * generates a String for transmission of multible data sets
 * sets: array of pointers to data sets
 * first, last: boundaries
 */
String generateMulti64SendLine(t_SendData* sets, int cnt){
  //php.ini: max_input_var = 1000 by default (1 data set = 1 var) -> max 100 datasets -> no problem
  //php.ini: post_max_size = 8M by default - SIM800 312k only! (1 data set = 64 byte) -> no problem
  //don't forget to increase timeouts on web server side
  String result = "";
  int k = 0;
  for (int i=0; i<cnt; i++){
    if (!isEmptyDataSet(&sets[i])) {
      k++;
      if (k > 1) { result += "&"; }
      _DD(_PrintDataSet(&sets[i], DEBUG_LEVEL_3));
      // why that complicated and not just memcopy?
      // push_data.phtml expects 8 bit count of values, 16 bit ID, 2x32bit coordinates
      // and than count-3 16 bit values - always 16 bit - but values in data structure aren't always 16 bit
      uint8_t hexbuffer[64];
      hexbuffer[0] = HEX_BUFFER_DATA_VALUES + 1; //count of data values + animal id
      hexbuffer[1] = herdeID();
      hexbuffer[2] = sets[i].animalID;
      _copyInt32toBuffer(hexbuffer,HEX_BUFFER_OFFSET  +  0, sets[i].latitude); //1
      _copyInt32toBuffer(hexbuffer,HEX_BUFFER_OFFSET  +  4, sets[i].longitude); //2
      _copyInt16toBuffer(hexbuffer,HEX_BUFFER_OFFSET  +  8, sets[i].altitude); //3
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 10, sets[i].date); //4
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 12, sets[i].time); //5
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 14, sets[i].battery); //6
      _copyInt16toBuffer(hexbuffer,HEX_BUFFER_OFFSET  + 16, sets[i].temperature); //7
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 18, sets[i].errCode); //8
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 20, sets[i].satellites); //9
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 22, sets[i].GPSpDOP); //10
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 24, sets[i].accThresCnt1);  //11
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 26, sets[i].accThresCnt2);  //12
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 28, sets[i].metersOut);  //13
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 30, sets[i].accThCnt1Spr);  //14
      _copyUint16toBuffer(hexbuffer,HEX_BUFFER_OFFSET + 32, sets[i].accThCnt2Spr);  //15 = HEX_BUFFER_VALUES
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

static_assert(sizeof(t_ConfigDataC) != 17, "t_ConfigDataC size has changed - something to do here!");

bool newSettingsB64(String b64, t_ConfigDataC* dataBuffer){
  unsigned char buffer[256];
  int size = b64Decode(b64, buffer);
  if (size < 3) { return false; };
  if ((buffer[0] >= 1) && (size >= 3)) { setNewSettingU8( buffer, 1, &dataBuffer->bootCycles, NEW_SETTINGS); }
  if ((buffer[0] >= 2) && (size >= 5)) { setNewSettingU8( buffer, 3, &dataBuffer->nightBootCycles, NEW_SETTINGS); }
  if ((buffer[0] >= 3) && (size >= 7)) { setNewSettingU8( buffer, 5, &dataBuffer->sleepMinutes, NEW_SETTINGS); }
  if ((buffer[0] >= 4) && (size >= 9)) { setNewSettingU8( buffer, 7, &dataBuffer->nightSleepMin, NEW_SETTINGS); }
  if ((buffer[0] >= 5) && (size >= 11)){ setNewSettingU8( buffer, 9, &dataBuffer->nightHourStart, NEW_SETTINGS); }
  if ((buffer[0] >= 6) && (size >= 13)){ setNewSettingU8( buffer,11, &dataBuffer->nightHourEnd, NEW_SETTINGS); }
  if ((buffer[0] >= 7) && (size >= 15)){ setNewSettingU16(buffer,13, &dataBuffer->distAlertThres, NEW_SETTINGS); }

  if ((buffer[0] >= 8) && (size >= 17)) { setNewSettingU16(buffer,15, &dataBuffer->accThres1, NEW_ACC_DATA); }
  if ((buffer[0] >= 9) && (size >= 19)) { setNewSettingU16(buffer,17, &dataBuffer->accAlertThres1, NEW_ACC_DATA); }
  if ((buffer[0] >= 10) && (size >= 21)) {setNewSettingU16(buffer,19, &dataBuffer->accThres2, NEW_ACC_DATA); }
  if ((buffer[0] >= 11) && (size >= 23)) {setNewSettingU16(buffer,21, &dataBuffer->accAlertThres2, NEW_ACC_DATA); }
  if ((buffer[0] >= 12) && (size >= 25)) {setNewSettingU8( buffer,23, &dataBuffer->accNightFactor, NEW_ACC_DATA); }
  checkSettings(dataBuffer);
  return true;
}

bool pushSettingsToBuffer(uint8_t* buffer, int size, t_ConfigDataC* dataBuffer){
  if(size < CONF_DATA_C_PAYLOAD_SIZE) { return false; }
  buffer[0] = dataBuffer->bootCycles;
  buffer[1] = dataBuffer->nightBootCycles;
  buffer[2] = dataBuffer->sleepMinutes;
  buffer[3] = dataBuffer->nightSleepMin;
  buffer[4] = dataBuffer->nightHourStart;
  buffer[5] = dataBuffer->nightHourEnd;
  _copyUint16toBuffer(buffer, 6, dataBuffer->distAlertThres);
  _copyUint16toBuffer(buffer, 8, dataBuffer->accThres1);
  _copyUint16toBuffer(buffer,10, dataBuffer->accAlertThres1);
  _copyUint16toBuffer(buffer,12, dataBuffer->accThres2);
  _copyUint16toBuffer(buffer,14, dataBuffer->accAlertThres2);
  buffer[16] = dataBuffer->accNightFactor;
  return true;
}
#else //HEIDI_GATEWAY
bool setSettingsFromBuffer(uint8_t* buffer, int size,t_ConfigDataC* dataBuffer){
  if (size < CONF_DATA_C_PAYLOAD_SIZE) { return false; };
  setNewSettingU8( buffer, 0, &dataBuffer->bootCycles, NEW_SETTINGS);
  setNewSettingU8( buffer, 1, &dataBuffer->nightBootCycles, NEW_SETTINGS);
  setNewSettingU8( buffer, 2, &dataBuffer->sleepMinutes, NEW_SETTINGS);
  setNewSettingU8( buffer, 3, &dataBuffer->nightSleepMin, NEW_SETTINGS);
  setNewSettingU8( buffer, 4, &dataBuffer->nightHourStart, NEW_SETTINGS);
  setNewSettingU8( buffer, 5, &dataBuffer->nightHourEnd, NEW_SETTINGS);
  setNewSettingU16(buffer, 6, &dataBuffer->distAlertThres, NEW_SETTINGS);

  setNewSettingU16(buffer, 8, &dataBuffer->accThres1, NEW_ACC_DATA);
  setNewSettingU16(buffer,10, &dataBuffer->accAlertThres1, NEW_ACC_DATA);
  setNewSettingU16(buffer,12, &dataBuffer->accThres2, NEW_ACC_DATA);
  setNewSettingU16(buffer,14, &dataBuffer->accAlertThres2, NEW_ACC_DATA);
  setNewSettingU8( buffer,16, &dataBuffer->accNightFactor, NEW_ACC_DATA);
  checkSettings(dataBuffer);
  return true;
}
#endif

void checkSettings(t_ConfigDataC* dataBuffer){
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
}

//#define DEBUG_SET_SETTINGS

void setNewSetting8(uint8_t *buffer, int pufferPos, int8_t *setting, uint32_t status){
  int8_t dummy = _copyBufferToUInt16(buffer, pufferPos);
  if(dummy != *setting){
    *setting = dummy;
    setState(status);
#ifdef DEBUG_SET_SETTINGS
    _DD( DebugPrintln("set [" + String(pufferPos) + "] to " + String(*setting), DEBUG_LEVEL_3); )
  } _DD( else { DebugPrintln("setting " + String(pufferPos) +  "keeps the same", DEBUG_LEVEL_3); } )
#else
  }
#endif
}
void setNewSettingU8(uint8_t *buffer, int pufferPos, uint8_t *setting, uint32_t status){
  uint8_t dummy = _copyBufferToUInt16(buffer, pufferPos);
  if(dummy != *setting){
    *setting = dummy;
    setState(status);
#ifdef DEBUG_SET_SETTINGS
    _DD( DebugPrintln("set [" + String(pufferPos) + "] to " + String(*setting), DEBUG_LEVEL_3); )
  } _DD( else { DebugPrintln("setting " + String(pufferPos) +  "keeps the same", DEBUG_LEVEL_3); } )
#else
  }
#endif
}
void setNewSettingU16(uint8_t *buffer, int pufferPos, uint16_t *setting, uint32_t status){
  uint16_t dummy = _copyBufferToUInt16(buffer, pufferPos);
  if(dummy != *setting){
    *setting = dummy;
    setState(status);
#ifdef DEBUG_SET_SETTINGS
    _DD( DebugPrintln("set [" + String(pufferPos) + "] to " + String(*setting), DEBUG_LEVEL_3); )
  } _DD( else { DebugPrintln("setting " + String(pufferPos) +  "keeps the same", DEBUG_LEVEL_3); } )
#else
  }
#endif
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
#endif //HEIDI_GATEWAY

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
#endif //USE_RTC_FAST_MEM
#ifdef HEIDI_CONFIG_TEST
#ifdef TEST_DATA
void testData(){
  int x;
  pause(3000);

  loadTestData();
  _PrintShortSummary(DEBUG_LEVEL_3);
  int maxSets = 10;
  int bufferSize = sizeof(t_SendData) * maxSets;
  t_SendData* buffer = (t_SendData*)malloc(bufferSize);
  if(buffer == NULL){ return; }
  memset(buffer, 0, bufferSize);
  int setsSent=maxSets;
  while(setsSent > 0){
    _D(DebugPrintln("", DEBUG_LEVEL_1);)
    _D(DebugPrintln("----------------- xxxxxxx ----------------", DEBUG_LEVEL_1);)
    setsSent = getNextnDataSets(maxSets, buffer, bufferSize, &buffer[setsSent-1]);
    _D(DebugPrintln(String(setsSent) + " sets gotten", DEBUG_LEVEL_1);)
    if(setsSent > 0){
      for(int i=0; i<setsSent; i++){ _PrintShortSet(&buffer[i], i, DEBUG_LEVEL_3); }
    }
    eraseDataSets(buffer, setsSent);
  }
  free(buffer);
}

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
#endif //TEST_RTC
#define TEST_DATA_COUNT 96
t_SendData TestDataSets[TEST_DATA_COUNT] = {
 /* 1 */ { 0x030a560d, 0x00cb970f, 0x0126, 0x543e, 0x5f60, 0x0f19, 0x01, 0x04, 0x0000, 0x07, 0x20, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55c1, 0x00cb9714, 0x0119, 0x543e, 0x5e9d, 0x0f19, 0x02, 0x04, 0x0000, 0x07, 0x28, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5574, 0x00cb978a, 0x0122, 0x543e, 0x5de0, 0x0f1a, 0x03, 0x04, 0x0000, 0x06, 0x1e, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5605, 0x00cb97b8, 0x0126, 0x543e, 0x5d1d, 0x0f1a, 0x04, 0x04, 0x0000, 0x07, 0x2e, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55d4, 0x00cb973c, 0x0135, 0x543e, 0x5c5d, 0x0f1d, 0x05, 0x04, 0x0000, 0x08, 0x21, 0x00ae, 0x002e, 0x0000, 0x9f00, 0xf500 },
         { 0x030a5521, 0x00cb96c6, 0x0139, 0x543e, 0x5b9d, 0x0f1d, 0x06, 0x04, 0x0000, 0x09, 0x20, 0x004f, 0x0021, 0x0000, 0xf070, 0xf000 },
         { 0x030a5655, 0x00cb9828, 0x0135, 0x543e, 0x5add, 0x0f23, 0x07, 0x04, 0x0000, 0x06, 0x2e, 0x0012, 0x0000, 0x0000, 0x00f0, 0x0000 },
         { 0x030a51f4, 0x00cb924a, 0x0079, 0x543e, 0x5a20, 0x0f22, 0x08, 0x04, 0x0000, 0x06, 0x28, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5620, 0x00cb963f, 0x00fa, 0x543e, 0x595d, 0x0f23, 0x09, 0x04, 0x0800, 0x09, 0x47, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 10 */ { 0x030a56a9, 0x00cb96ee, 0x011c, 0x543e, 0x589d, 0x0f23, 0x0a, 0x04, 0x0000, 0x09, 0x3a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5490, 0x00cb96f3, 0x0179, 0x543e, 0x575d, 0x0f23, 0x0b, 0x04, 0x0800, 0x06, 0x4b, 0x0000, 0x0000, 0x0008, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x569d, 0x0f25, 0x0c, 0x04, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5cd1, 0x00cb99c6, 0x0238, 0x543e, 0x55e0, 0x0f26, 0x0d, 0x04, 0x0800, 0x06, 0x55, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x5520, 0x0f26, 0x0e, 0x04, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a54eb, 0x00cb9647, 0x0000, 0x543e, 0x545c, 0x0f2a, 0x0f, 0x04, 0x0800, 0x03, 0x5d, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a54e0, 0x00cb96bd, 0x0000, 0x543e, 0x53a1, 0x0f28, 0x10, 0x04, 0x0000, 0x03, 0x20, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55a2, 0x00cb9743, 0x0142, 0x543e, 0x52dc, 0x0f2a, 0x11, 0x04, 0x0800, 0x07, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a557c, 0x00cb978d, 0x0000, 0x543e, 0x5220, 0x0f2b, 0x12, 0x04, 0x0000, 0x03, 0x27, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5755, 0x00cb9587, 0x0133, 0x543e, 0x515d, 0x0f2b, 0x13, 0x04, 0x0800, 0x08, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 20 */ { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x509d, 0x0f2b, 0x14, 0x04, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x4f60, 0x0f2d, 0x15, 0x04, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x4e9d, 0x0f2d, 0x16, 0x04, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a57b4, 0x00cb948b, 0x0166, 0x543e, 0x4ddd, 0x0f2d, 0x17, 0x04, 0x0800, 0x06, 0x43, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x4da1, 0x0f2d, 0x18, 0x04, 0x0000, 0x00, 0x03, 0x000e, 0x0000, 0x0000, 0x000f, 0x0000 },
         { 0x030a5742, 0x00cb94e8, 0x0120, 0x543e, 0x4ce1, 0x0f2e, 0x19, 0x04, 0x0800, 0x07, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x4c40, 0x0f30, 0x1a, 0x04, 0x6858, 0x00, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a59b7, 0x00cba0c3, 0x0000, 0x543e, 0x4ba0, 0x0f31, 0x1b, 0x04, 0x0840, 0x03, 0x79, 0x0000, 0x0000, 0x0074, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x4ae0, 0x0f30, 0x1c, 0x04, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5661, 0x00cb957a, 0x014f, 0x543e, 0x4a1d, 0x0f33, 0x1d, 0x04, 0x0800, 0x07, 0x62, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 30 */ { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x4960, 0x0f33, 0x1e, 0x04, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a56b9, 0x00cb96b1, 0x00dc, 0x543e, 0x489d, 0x0f36, 0x1f, 0x04, 0x0800, 0x08, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55b5, 0x00cb9722, 0x0000, 0x543e, 0x4760, 0x0f34, 0x20, 0x04, 0x0800, 0x03, 0x41, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a56db, 0x00cb95d3, 0x00dd, 0x543e, 0x46a0, 0x0f39, 0x21, 0x04, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5624, 0x00cb94fc, 0x0000, 0x543e, 0x45dc, 0x0f39, 0x22, 0x04, 0x0800, 0x03, 0x57, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5709, 0x00cb97d6, 0x007c, 0x543e, 0x4521, 0x0f37, 0x23, 0x04, 0x0000, 0x05, 0x19, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a56c4, 0x00cb95c0, 0x00ac, 0x543e, 0x445c, 0x0f3c, 0x24, 0x04, 0x0800, 0x06, 0xff, 0x0034, 0x000f, 0x0000, 0x00f0, 0x00f0 },
         { 0x030a5669, 0x00cb984b, 0x0000, 0x543e, 0x43a0, 0x0f3c, 0x25, 0x04, 0x0000, 0x03, 0x35, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5742, 0x00cb94f2, 0x0012, 0x543e, 0x42dd, 0x0f3f, 0x26, 0x04, 0x0800, 0x05, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5552, 0x00cb972f, 0x0000, 0x543e, 0x4220, 0x0f3f, 0x27, 0x04, 0x0800, 0x03, 0x76, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 40 */ { 0x030a555a, 0x00cb96d8, 0x0125, 0x543e, 0x415d, 0x0f41, 0x28, 0x04, 0x0800, 0x05, 0x79, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a558b, 0x00cb9763, 0x010e, 0x543e, 0x40a0, 0x0f42, 0x29, 0x04, 0x0000, 0x05, 0x2b, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55c8, 0x00cb97b8, 0x0131, 0x543e, 0x3f5d, 0x0f44, 0x2a, 0x04, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5578, 0x00cb9777, 0x0000, 0x543e, 0x3e9d, 0x0f45, 0x2b, 0x04, 0x0800, 0x03, 0x43, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5412, 0x00cb9666, 0x0000, 0x543e, 0x3ddd, 0x0f44, 0x2c, 0x04, 0x0800, 0x03, 0x4a, 0x0000, 0x0000, 0x0007, 0x0000, 0x0000 },
         { 0x030a55ef, 0x00cb97c3, 0x0127, 0x543e, 0x3d20, 0x0f45, 0x2d, 0x04, 0x0800, 0x06, 0x61, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55db, 0x00cb9724, 0x0125, 0x543e, 0x3c60, 0x0f48, 0x2e, 0x04, 0x0800, 0x05, 0x58, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a556d, 0x00cb9734, 0x013b, 0x543e, 0x3b9d, 0x0f45, 0x2f, 0x04, 0x0800, 0x06, 0x3c, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55e7, 0x00cb9828, 0x0113, 0x543e, 0x3ae0, 0x0f48, 0x30, 0x04, 0x0000, 0x07, 0x3a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a559b, 0x00cb9710, 0x013c, 0x543e, 0x3a1d, 0x0f4a, 0x31, 0x04, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 50 */ { 0x030a56d7, 0x00cb96d6, 0x0000, 0x543e, 0x395d, 0x0f4a, 0x32, 0x04, 0x0800, 0x03, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5552, 0x00cb9737, 0x012c, 0x543e, 0x38a0, 0x0f4a, 0x33, 0x04, 0x0800, 0x05, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5569, 0x00cb96ec, 0x0000, 0x543e, 0x3760, 0x0f4e, 0x34, 0x04, 0x0800, 0x03, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a58b0, 0x00cb95e3, 0x0000, 0x543e, 0x369d, 0x0f52, 0x35, 0x04, 0x0800, 0x03, 0x62, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a56a2, 0x00cb963d, 0x0000, 0x543e, 0x35e0, 0x0f52, 0x36, 0x04, 0x0800, 0x03, 0x5a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a559e, 0x00cb96e6, 0x012d, 0x543e, 0x3520, 0x0f56, 0x37, 0x04, 0x0800, 0x05, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a56db, 0x00cb98ef, 0x0000, 0x543e, 0x345d, 0x0f58, 0x38, 0x04, 0x0800, 0x03, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5637, 0x00cb9869, 0x0110, 0x543e, 0x33a0, 0x0f58, 0x39, 0x04, 0x0000, 0x04, 0x24, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5552, 0x00cb96e3, 0x013d, 0x543e, 0x32dc, 0x0f58, 0x3a, 0x04, 0x0000, 0x06, 0x3c, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5578, 0x00cb978f, 0x0000, 0x543e, 0x321d, 0x0f56, 0x3b, 0x04, 0x0000, 0x03, 0x1e, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 60 */ { 0x030a5569, 0x00cb9714, 0x0141, 0x543e, 0x3160, 0x0f5b, 0x3c, 0x04, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5665, 0x00cb993f, 0x0000, 0x543e, 0x309d, 0x0f5c, 0x3d, 0x04, 0x0800, 0x03, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55cc, 0x00cb97fa, 0x011e, 0x543e, 0x2f61, 0x0f5c, 0x3e, 0x04, 0x0800, 0x04, 0x55, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a558f, 0x00cb97d8, 0x011a, 0x543e, 0x2e9c, 0x0f5e, 0x3f, 0x04, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a54fa, 0x00cb960b, 0x0000, 0x543e, 0x2de1, 0x0f5e, 0x40, 0x04, 0x0000, 0x03, 0x33, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5543, 0x00cb96e3, 0x0137, 0x543e, 0x2d1c, 0x0f5f, 0x41, 0x04, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a54eb, 0x00cb95a3, 0x0000, 0x543e, 0x2c5d, 0x0f5f, 0x42, 0x04, 0x0000, 0x03, 0x2c, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5593, 0x00cb9766, 0x012f, 0x543e, 0x2ba0, 0x0f62, 0x43, 0x04, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a554e, 0x00cb961c, 0x0000, 0x543e, 0x2add, 0x0f67, 0x44, 0x04, 0x0800, 0x03, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x2a20, 0x0f67, 0x45, 0x04, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 70 */ { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x295d, 0x0f67, 0x46, 0x04, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a52d5, 0x00cb97fc, 0x01af, 0x543e, 0x28a0, 0x0f69, 0x47, 0x04, 0x0800, 0x05, 0x75, 0x0000, 0x0000, 0x0036, 0x0000, 0x0000 },
         { 0x030a55b9, 0x00cb9709, 0x0127, 0x543e, 0x275d, 0x0f6c, 0x48, 0x04, 0x0800, 0x05, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a05cb, 0x00caf5c0, 0x0000, 0x543e, 0x26a0, 0x0f6a, 0x49, 0x04, 0x0800, 0x03, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55b5, 0x00cb96ab, 0x0132, 0x543e, 0x25dd, 0x0f70, 0x4a, 0x04, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55c8, 0x00cb9734, 0x0126, 0x543e, 0x2520, 0x0f72, 0x4b, 0x04, 0x0800, 0x04, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a573e, 0x00cb9806, 0x0000, 0x543e, 0x245d, 0x0f78, 0x4c, 0x04, 0x0800, 0x03, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x239d, 0x0f76, 0x4d, 0x04, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55e7, 0x00cb976a, 0x012f, 0x543e, 0x22e0, 0x0f7b, 0x4e, 0x04, 0x0800, 0x04, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x221d, 0x0f81, 0x4f, 0x04, 0x0818, 0x00, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 80 */ { 0x030a5701, 0x00cb9c2b, 0x0000, 0x543e, 0x2160, 0x0f80, 0x50, 0x04, 0x0800, 0x03, 0x79, 0x0000, 0x0000, 0x0021, 0x0000, 0x0000 },
         { 0x030a58e2, 0x00cb9863, 0x00f6, 0x543e, 0x20a0, 0x0f83, 0x51, 0x04, 0x0800, 0x05, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x2080, 0x0f83, 0x52, 0x04, 0x0000, 0x00, 0x02, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5602, 0x00cb9704, 0x0122, 0x543e, 0x1f40, 0x0f87, 0x53, 0x04, 0x0800, 0x05, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030b4b16, 0x00c8a656, 0x0000, 0x543e, 0x1e9d, 0x0f87, 0x54, 0x04, 0x0840, 0x03, 0x79, 0x0000, 0x0000, 0x3718, 0x0000, 0x0000 },
         { 0x030a559b, 0x00cb984f, 0x0000, 0x543e, 0x1ddd, 0x0f89, 0x55, 0x04, 0x0800, 0x03, 0x64, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x1d20, 0x0f8a, 0x56, 0x04, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x1c5d, 0x0f8a, 0x57, 0x04, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x1ba0, 0x0f8e, 0x58, 0x04, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a56c8, 0x00cb9f08, 0x005d, 0x543e, 0x1add, 0x0f8f, 0x59, 0x04, 0x0800, 0x04, 0x79, 0x0000, 0x0000, 0x0054, 0x0000, 0x0000 },
/* 90 */ { 0x030a54ba, 0x00cb95a9, 0x0000, 0x543e, 0x1a1d, 0x0f8f, 0x5a, 0x04, 0x0000, 0x03, 0x1b, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a555a, 0x00cb979b, 0x0123, 0x543e, 0x1960, 0x0f8e, 0x5b, 0x04, 0x0000, 0x06, 0x22, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5524, 0x00cb9718, 0x0129, 0x543e, 0x189d, 0x0f8e, 0x5c, 0x04, 0x0800, 0x06, 0x3e, 0x0000, 0x0000, 0x0001, 0x0000, 0x0000 },
         { 0x030a554a, 0x00cb973f, 0x0000, 0x543e, 0x175d, 0x0f8f, 0x5d, 0x04, 0x0000, 0x03, 0x21, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55aa, 0x00cb97ac, 0x012b, 0x543e, 0x16a0, 0x0f8e, 0x5e, 0x04, 0x0800, 0x07, 0x40, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a555a, 0x00cb977a, 0x013c, 0x543e, 0x15dd, 0x0f8f, 0x5f, 0x04, 0x0800, 0x06, 0x59, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 96 */ { 0x030a55fa, 0x00cb9746, 0x0138, 0x543e, 0x151d, 0x0f8e, 0x60, 0x04, 0x0800, 0x06, 0x40, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }
  };
void loadTestData(void){
  for(int i=0; i<TEST_DATA_COUNT; i++){
    if(i<allDataSets) {
      copyDataSet(&TestDataSets[i], availableDataSet[i]);
      availableDataSet[i]->animalID = animalID();
    } else { break; }
  }
}
t_SendData* getTestData(int _which){
  if((_which >= 0) && (_which < TEST_DATA_COUNT)) {return &TestDataSets[_which];}
  return &TestDataSets[TEST_DATA_COUNT];
}
#endif //HEIDI_CONFIG_TEST
