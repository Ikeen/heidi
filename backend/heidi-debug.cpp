/*
 * heidi-debug.cpp
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */
#include <Arduino.h>
#include "heidi-debug.h"
#if (DEBUG_LEVEL > 0)
#include "heidi-acc.h"
#include "heidi-fence.h"
#include <esp32/ulp.h>

void DebugPrint(String text, int level){ if (level <= DEBUG_LEVEL){ Serial.print(text); }}
void DebugPrintln(String text, int level){ if (level <= DEBUG_LEVEL){ Serial.println(text); }}
void DebugPrint(double number, int digits, int level){ if (level <= DEBUG_LEVEL){ Serial.print(number, digits); }}
void DebugPrintln(double number, int digits, int level){ if (level <= DEBUG_LEVEL){ Serial.println(number, digits); }}
void DebugPrint(int number, int level){ if (level <= DEBUG_LEVEL){ Serial.print(number); }}
void DebugPrintln(int number, int level){ if (level <= DEBUG_LEVEL){ Serial.println(number); }}
void DebugPrint(unsigned int number, int level){ if (level <= DEBUG_LEVEL){ Serial.print(number); }}
void DebugPrintln(unsigned int number, int level){ if (level <= DEBUG_LEVEL){ Serial.println(number); }}

void setupDebug(int startMS, bool powerOnReset){
_D(
  Serial.begin(115200);
  //if(powerOnReset) { delay(3000); }
  DebugPrintln("build date: " + String(__DATE__), DEBUG_LEVEL_1); DebugPrintln("Enter Main Loop. " + String(startMS), DEBUG_LEVEL_1); delay(50);
  checkWakeUpReason();
  #if (MAX_AWAKE_TIME_POWER_MSEC == 0) || (MAX_AWAKE_TIME_TIMER_MSEC == 0)
  DebugPrintln("!!!! timeouts disabled !!!!!", DEBUG_LEVEL_1);
  #endif
  )
}
void debugHeidiState(bool powerOnReset){

  if(powerOnReset) { DebugPrintln("BOOT: was power on reset", DEBUG_LEVEL_1); }
  else { DebugPrintln("BOOT: cycle number: " + String(heidiConfig->bootCount), DEBUG_LEVEL_1); }
  DebugPrintln("BOOT: Heidi state: 0x" + String(heidiConfig->status, HEX) ,DEBUG_LEVEL_2);
}

void _PrintDataSet(t_SendData* DataSet, int dLevel){
  _D(DebugPrint("DataSet Lat= ", dLevel));
  _D(DebugPrint(DataSet->latitude, dLevel));
  _D(DebugPrint(" Lng= ", dLevel));
  _D(DebugPrint(DataSet->longitude, dLevel));
  _D(DebugPrint(" Alt= ", dLevel));
  _D(DebugPrint(DataSet->altitude, dLevel));
  _D(DebugPrint(" Time= ", dLevel));
  _D(DebugPrint(String(dosYear(DataSet->date)) + "-" + LenTwo(String(dosMonth(DataSet->date))) + "-" + LenTwo(String(dosDay(DataSet->date))), dLevel));
  _D(DebugPrint(" " + LenTwo(String(dosHour(DataSet->time))) + ":" + LenTwo(String(dosMinute(DataSet->time))) + ":" + LenTwo(String(dosSecond(DataSet->time))), dLevel));
  _D(DebugPrint(" Bat= ", dLevel));
  _D(DebugPrint(String(double(DataSet->battery)/1000, 2), dLevel));
  _D(DebugPrint(" Err= ", dLevel));
  _D(DebugPrint(String(DataSet->errCode, HEX), dLevel));
  _D(DebugPrint(" GPSHDOP= ", dLevel));
  _D(DebugPrint(DataSet->GPShdop, dLevel));
  _D(DebugPrint(" Sat= ", dLevel));
  _D(DebugPrint(DataSet->satellites, dLevel));
  _D(DebugPrint(" accTh1= ", dLevel));
  _D(DebugPrintln(DataSet->accThresCnt1, dLevel));
}
void _PrintShortSummary(int dLevel){
  for(int i=0; i<allDataSets; i++){
    t_SendData* DataSet = availableDataSet[i];
    if (!emptyDataSet(availableDataSet[i])){
      _D(DebugPrint("Dataset " + String(i) + ": ", dLevel);)
      _D(DebugPrint(LenTwo(String(dosHour(DataSet->time))) + ":" + LenTwo(String(dosMinute(DataSet->time))) + ":" + LenTwo(String(dosSecond(DataSet->time))), dLevel);)
      _D(DebugPrintln("; " + String(DataSet->errCode, HEX), dLevel);)
    }
  }
}
void _PrintFence(int dLevel){
  for(int i=0; i<FENCE_MAX_POS; i++){
    _t_FenceData* Fencepole = FenceDataSet[i];
    if (!emptyPoint(Fencepole)){
      _D(DebugPrint("FencePoint " + String(i) + ": ", dLevel);)
      _D(DebugPrintln(String(Fencepole->latitude) + ", " + String(Fencepole->longitude), dLevel);)
    }
  }
}
void _PrintHeidiConfig(int dLevel){
  _D(DebugPrintln("", dLevel);)
  _D(DebugPrintln("heidi-config:", dLevel);)
  t_ConfigData* heidiConfig = (t_ConfigData*)&(RTC_SLOW_MEM[RTC_DATA_SPACE_OFFSET]);
  _D(DebugPrintln("bootCount:       " + String(heidiConfig->bootCount), dLevel);)
  _D(DebugPrintln("bootCycles:      " + String(heidiConfig->bootCycles), dLevel);)
  _D(DebugPrintln("sleepMinutes:    " + String(heidiConfig->sleepMinutes), dLevel);)
  _D(DebugPrintln("nightHourStart:  " + String(heidiConfig->nightHourStart), dLevel);)
  _D(DebugPrintln("nightHourEnd:    " + String(heidiConfig->nightHourEnd), dLevel);)
  _D(DebugPrintln("nightBootCycles: " + String(heidiConfig->nightBootCycles), dLevel);)
  _D(DebugPrintln("nightSleepMin:   " + String(heidiConfig->nightSleepMin), dLevel);)
  _D(DebugPrintln("lastTimeDiffMs:  " + String(heidiConfig->lastTimeDiffMs), dLevel);)
}

void checkWakeUpReason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : _D(DebugPrintln("Wakeup caused by external signal using RTC_IO", DEBUG_LEVEL_1);) break;
    case ESP_SLEEP_WAKEUP_EXT1 : _D(DebugPrintln("Wakeup caused by external signal using RTC_CNTL", DEBUG_LEVEL_1);) break;
    case ESP_SLEEP_WAKEUP_TIMER : _D(DebugPrintln("Wakeup caused by timer", DEBUG_LEVEL_1);) break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : _D(DebugPrintln("Wakeup caused by touchpad", DEBUG_LEVEL_1);) break;
    case ESP_SLEEP_WAKEUP_ULP : _D(DebugPrintln("Wakeup caused by ULP program", DEBUG_LEVEL_1);) break;
    default : _D(DebugPrint("Wakeup was not caused by deep sleep: ", DEBUG_LEVEL_1);) _D(DebugPrintln(String(wakeup_reason), DEBUG_LEVEL_1);) break;

  }
}

#ifdef ACCELEROMETER
void _PrintHeidiAccParams(int dLevel){
  _D(DebugPrintln("", dLevel);)
  _D(DebugPrintln("Accel parameters", dLevel);)
  _D(DebugPrintln("Thres 1: " + String(get_accel_excnt1_ULP()) + " / " + String(get_accel_exthr1_ULP()) + " / " + String(get_accel_wake1_ULP()), dLevel);)
  _D(DebugPrintln("Thres 2: " + String(get_accel_excnt2_ULP()) + " / " + String(get_accel_exthr2_ULP()) + " / " + String(get_accel_wake2_ULP()), dLevel);)
}
#endif
#endif


