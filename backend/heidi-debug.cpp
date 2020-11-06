/*
 * heidi-debug.cpp
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */
#include <Arduino.h>
#include "heidi-debug.h"
#if (DEBUG_LEVEL > 0)

void DebugPrint(String text, int level){ if (level <= DEBUG_LEVEL){ Serial.print(text); }}
void DebugPrintln(String text, int level){ if (level <= DEBUG_LEVEL){ Serial.println(text); }}
void DebugPrint(double number, int digits, int level){ if (level <= DEBUG_LEVEL){ Serial.print(number, digits); }}
void DebugPrintln(double number, int digits, int level){ if (level <= DEBUG_LEVEL){ Serial.println(number, digits); }}
void DebugPrint(int number, int level){ if (level <= DEBUG_LEVEL){ Serial.print(number); }}
void DebugPrintln(int number, int level){ if (level <= DEBUG_LEVEL){ Serial.println(number); }}
void DebugPrint(unsigned int number, int level){ if (level <= DEBUG_LEVEL){ Serial.print(number); }}
void DebugPrintln(unsigned int number, int level){ if (level <= DEBUG_LEVEL){ Serial.println(number); }}

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
  _D(DebugPrint(DataSet->errCode, dLevel));
  _D(DebugPrint(" GPSHDOP= ", dLevel));
  _D(DebugPrint(DataSet->GPShdop, dLevel));
  _D(DebugPrint(" Sat= ", dLevel));
  _D(DebugPrintln(DataSet->satellites, dLevel));
}
void _PrintShortSummary(int dLevel){
  for(int i=0; i<(BOOT_CYCLES * (DATA_SET_BACKUPS + 1)); i++){
    t_SendData* DataSet = availableDataSet[i];
    _D(DebugPrint("Dataset " + String(i) + ": ", dLevel));
    _D(DebugPrint(LenTwo(String(dosHour(DataSet->time))) + ":" + LenTwo(String(dosMinute(DataSet->time))) + ":" + LenTwo(String(dosSecond(DataSet->time))), dLevel));
    _D(DebugPrintln("; " + String(DataSet->errCode, HEX), dLevel));
  }
}
#endif


