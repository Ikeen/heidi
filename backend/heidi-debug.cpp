/*
 * heidi-debug.cpp
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */
#include <Arduino.h>
#include "heidi-debug.h"

#include "heidi-acc.h"
#include "heidi-sys.h"
#include "heidi-fence.h"
#include "heidi-data.h"
#include <esp32/ulp.h>

#if (DEBUG_LEVEL > 0)

static bool dbgInUse;

void DebugPrint(String text, int level){ if (level <= DEBUG_LEVEL){ if(dbgGetSem()) {Serial.print(text); dbgFreeSem(); }}}
void DebugPrintln(String text, int level){ if (level <= DEBUG_LEVEL){ if(dbgGetSem()) {Serial.println(text); dbgFreeSem(); }}}
void DebugPrint(double number, int digits, int level){ if (level <= DEBUG_LEVEL){ if(dbgGetSem()) {Serial.print(number, digits); dbgFreeSem(); }}}
void DebugPrintln(double number, int digits, int level){ if (level <= DEBUG_LEVEL){ if(dbgGetSem()) {Serial.println(number, digits); dbgFreeSem(); }}}
void DebugPrint(int number, int level){ if (level <= DEBUG_LEVEL){ if(dbgGetSem()) {Serial.print(number); dbgFreeSem(); }}}
void DebugPrintln(int number, int level){ if (level <= DEBUG_LEVEL){ if(dbgGetSem()) {Serial.println(number); dbgFreeSem(); }}}
void DebugPrint(unsigned int number, int level){ if (level <= DEBUG_LEVEL){ if(dbgGetSem()) {Serial.print(number); dbgFreeSem(); }}}
void DebugPrintln(unsigned int number, int level){ if (level <= DEBUG_LEVEL){ if(dbgGetSem()) {Serial.println(number); dbgFreeSem(); }}}

#endif

void setupDebug(int startMS, bool powerOnReset){
  Serial.begin(115200);
  if(powerOnReset){
    Serial.println("build date: " + String(__DATE__));
    Serial.print("hardware  :");
    #ifdef GPS_MODULE
    Serial.print(" GPS");
    #endif
    #ifdef GSM_MODULE
    Serial.print(" GSM");
    #endif
    #ifdef TEMP_SENSOR
    Serial.print(" TMP");
    #endif
    #ifdef ACCELEROMETER
    Serial.print(" ACC");
    #endif
    #ifdef USE_LORA
    Serial.print(" LOR");
    #endif
    #ifdef USE_OLED
    Serial.print(" OLD");
    #endif
    Serial.println("");
    #if (HEIDI_SW_BRAND > 0)
    Serial.println("software  : V" + String(HEIDI_SW_BRAND));
    #else
    Serial.println("software  : TEST");
    #endif
    Serial.println("tracker   : " + _herdeID()+ "." + _animalID());
    pause(100);
  }
#if (DEBUG_LEVEL == 0)
}
#else
  dbgInUse = false;
  //if(powerOnReset) { pause(3000); }
  checkWakeUpReason();
  #if (MAX_AWAKE_TIME_POWER_MSEC == 0) || (MAX_AWAKE_TIME_TIMER_MSEC == 0)
  DebugPrintln("!!!! timeouts disabled !!!!!", DEBUG_LEVEL_1);
  #endif
  DebugPrintln("Enter main loop after " + String(startMS) + " ms", DEBUG_LEVEL_1);
}
#endif
#if (DEBUG_LEVEL > 0)

bool dbgGetSem(){
  int t = millis();
  while(dbgInUse && (millis()-t <= 10)){ pause(10); }
  if (dbgInUse) { return false; }
  dbgInUse = true;
  return true;
}
void dbgFreeSem(){
  dbgInUse = false;
}

void debugHeidiState(bool powerOnReset){

  if(powerOnReset) { DebugPrintln("BOOT: was power on reset", DEBUG_LEVEL_1); }
  else { DebugPrintln("BOOT: cycle number: " + String(heidiConfig->bootNumber), DEBUG_LEVEL_1); }
  DebugPrintln("BOOT: Heidi state: 0x" + String(heidiConfig->status, HEX) ,DEBUG_LEVEL_2);
}

void _PrintDataSet(t_SendData* DataSet, int dLevel){
  _D(
    DebugPrint("DataSet Lat= ", dLevel);
    DebugPrint(DataSet->latitude, dLevel);
    DebugPrint(" Lng= ", dLevel);
    DebugPrint(DataSet->longitude, dLevel);
    DebugPrint(" Alt= ", dLevel);
    DebugPrint(DataSet->altitude, dLevel);
    DebugPrint(" Time= ", dLevel);
    DebugPrint(String(dosYear(DataSet->date)) + "-" + LenTwo(String(dosMonth(DataSet->date))) + "-" + LenTwo(String(dosDay(DataSet->date))), dLevel);
    DebugPrint(" " + LenTwo(String(dosHour(DataSet->time))) + ":" + LenTwo(String(dosMinute(DataSet->time))) + ":" + LenTwo(String(dosSecond(DataSet->time))), dLevel);
    DebugPrint(" Bat= ", dLevel);
    DebugPrint(String(double(DataSet->battery)/1000, 2), dLevel);
    DebugPrint(" Err= ", dLevel);
    DebugPrint(String(DataSet->errCode, HEX), dLevel);
    DebugPrint(" pDOP= ", dLevel);
    DebugPrint(DataSet->GPSpDOP, dLevel);
    DebugPrint(" Sat= ", dLevel);
    DebugPrint(DataSet->satellites, dLevel);
    DebugPrint(" acc1= ", dLevel);
    DebugPrint(DataSet->accThresCnt1, dLevel);
    DebugPrint(" acc2= ", dLevel);
    DebugPrintln(DataSet->accThresCnt2, dLevel);
  )
}
void _PrintShortSummary(int dLevel){
  for(int i=0; i<allDataSets; i++){
    t_SendData* DataSet = availableDataSet[i];
    if ((!isEmptyDataSet(availableDataSet[i])) || (i == 0)){
      _PrintShortSet(availableDataSet[i], i, dLevel);
    }
  }
}
void _PrintShortSet(t_SendData* DataSet, int n, int dLevel){
  DebugPrint("Data set ", dLevel);
  if (n >= 0) { DebugPrint(String(n), dLevel); }
  #ifdef HEIDI_CONFIG_TEST
  DebugPrint(": [" + LenTwo(String(DataSet->temperature)) + "] ", dLevel); //temperature = data set ID [0x01 .. 0x60] in test data
  #else
  DebugPrint("Data set " + String(n) + ": ", dLevel);
  #endif
  DebugPrint(LenTwo(String(dosYear(DataSet->date))) + "-" + LenTwo(String(dosMonth(DataSet->date))) + "-" + LenTwo(String(dosDay(DataSet->date))) + " ", dLevel);
  DebugPrint(LenTwo(String(dosHour(DataSet->time))) + ":" + LenTwo(String(dosMinute(DataSet->time))) + ":" + LenTwo(String(dosSecond(DataSet->time))), dLevel);
  DebugPrintln(", client: " + LenTwo(String(DataSet->animalID)), dLevel);
}

void _PrintFence(int dLevel){
  for(int i=0; i<FENCE_MAX_POS; i++){
    _t_FenceData* Fencepole = FenceDataSet[i];
    if (!emptyPoint(Fencepole)){
      DebugPrint("FencePoint " + String(i) + ": ", dLevel);
      DebugPrintln(String(Fencepole->latitude) + ", " + String(Fencepole->longitude), dLevel);
    }
  }
}
void _PrintHeidiConfig(int dLevel){
  DebugPrintln("", dLevel);
  DebugPrintln("heidi-config [len = " + String(sizeof(t_ConfigDataC)) + "]:", dLevel);
  t_ConfigData* heidiConfig = (t_ConfigData*)&(RTC_SLOW_MEM[RTC_DATA_SPACE_OFFSET]);
  DebugPrintln("bootCycles:      " + String(heidiConfig->c.bootCycles), dLevel);
  DebugPrintln("sleepMinutes:    " + String(heidiConfig->c.sleepMinutes), dLevel);
  DebugPrintln("nightBootCycles: " + String(heidiConfig->c.nightBootCycles), dLevel);
  DebugPrintln("nightSleepMin:   " + String(heidiConfig->c.nightSleepMin), dLevel);
  DebugPrintln("nightHourStart:  " + String(heidiConfig->c.nightHourStart), dLevel);
  DebugPrintln("nightHourEnd:    " + String(heidiConfig->c.nightHourEnd), dLevel);

  DebugPrintln("distAlertThres:  " + String(heidiConfig->c.distAlertThres), dLevel);
  DebugPrintln("accThres1:       " + String(heidiConfig->c.accThres1), dLevel);
  DebugPrintln("accAlertThres1:  " + String(heidiConfig->c.accAlertThres1), dLevel);
  DebugPrintln("accThres2:       " + String(heidiConfig->c.accThres2), dLevel);
  DebugPrintln("accAlertThres2:  " + String(heidiConfig->c.accAlertThres2), dLevel);
  DebugPrintln("accNightFactor:  " + String(heidiConfig->c.accNightFactor), dLevel);
  DebugPrintln("checksum:        " + String(clientConfigCRC(&heidiConfig->c), HEX), dLevel);
}
/*
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

 */


void checkWakeUpReason(){
  esp_sleep_wakeup_cause_t wakeup_reason = getWakeUpReason();
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : DebugPrintln("Wake up caused by external signal using RTC_IO", DEBUG_LEVEL_1); break;
    case ESP_SLEEP_WAKEUP_EXT1 : DebugPrintln("Wake up caused by external signal using RTC_CNTL", DEBUG_LEVEL_1); break;
    case ESP_SLEEP_WAKEUP_TIMER : DebugPrintln("Wake up caused by timer", DEBUG_LEVEL_1); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : DebugPrintln("Wake up caused by touchpad", DEBUG_LEVEL_1); break;
    case ESP_SLEEP_WAKEUP_ULP : DebugPrintln("Wake up caused by ULP program", DEBUG_LEVEL_1); break;
    default : _D(DebugPrint("Wake up was not caused by deep sleep: ", DEBUG_LEVEL_1);) DebugPrintln(String(wakeup_reason), DEBUG_LEVEL_1); break;

  }
}

void _PrintNotDoneGSMHandling(int dLevel){
  int dataSets = packUpDataSets();
  if (doDataTransmission()){
    DebugPrintln("[GSM]: No GSM Module built in", dLevel);
    if(random(5) != 3){
      if (getState(GPS_ALERT_1 | GPS_ALERT_2) && !getState(GPS_ALERT_PSD)){
        #ifdef SEND_ALERT_SMS
        DebugPrintln("[GSM]: would send GPS alert SMS", dLevel);
        #else
        DebugPrintln("[GSM]: would handle GPS alert silent", dLevel);
        #endif
      }
      DebugPrintln("[GSM]: would transmit " + String(dataSets) + " data sets to " + HEIDI_SERVER_PUSH_URL, dLevel);
      int buffersize = (MAX_DATA_SETS_PER_SEND_LINE * sizeof(t_SendData));
      t_SendData* buffer = (t_SendData*)malloc(buffersize);
      if (buffer != NULL){
        int setsSent = MAX_DATA_SETS_PER_SEND_LINE;
        while (setsSent > 0){
          setsSent = getNextnDataSets(MAX_DATA_SETS_PER_SEND_LINE, buffer, buffersize);
          if(setsSent > 0){
            String sendLine = generateMulti64SendLine(buffer, setsSent);
            DebugPrintln("[GSM]: " + sendLine, dLevel);
            eraseDataSets(buffer, setsSent);
          }
        }
        free(buffer);
      }
      clrState(RESET_INITS);
      clrState(TRSMT_DATA_ERROR);
      for(int i = 0; i < dataSets; i++) { initDataSet(availableDataSet[i]); }
    } else {
      DebugPrintln("[GSM]: simulate failed transmission", dLevel);
      setState(TRSMT_DATA_ERROR);
    }
  }
}

#ifdef ACCELEROMETER
void _PrintHeidiAccParams(int dLevel){
  DebugPrintln("", dLevel);
  DebugPrintln("Accel parameters", dLevel);
  DebugPrintln("Thres 1: " + String(get_accel_excnt1_ULP()) + " / " + String(get_accel_exthr1_ULP()) + " / " + String(get_accel_wake1_ULP()), dLevel);
  DebugPrintln("Thres 2: " + String(get_accel_excnt2_ULP()) + " / " + String(get_accel_exthr2_ULP()) + " / " + String(get_accel_wake2_ULP()), dLevel);
}
#endif
#endif (DEBUG_LEVEL > 0)


