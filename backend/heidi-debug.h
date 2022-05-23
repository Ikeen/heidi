/*
 * heidi-debug.h
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */

#ifndef HEIDI_DEBUG_H_
#define HEIDI_DEBUG_H_
#include <WString.h>
#include "heidi-data.h"

typedef enum {
  DEBUG_LEVEL_0,
  DEBUG_LEVEL_1,
  DEBUG_LEVEL_2,
  DEBUG_LEVEL_3
};

#define DEBUG_LEVEL 1 //0 (no prints) .. 3 (all prints)

#if (DEBUG_LEVEL > 0 )
#  define _D(x) x
#else
#  define _D(x)
#endif
#if (DEBUG_LEVEL > 2 )
#  define _DD(x) x
#else
#  define _DD(x)
#endif
#if (DEBUG_LEVEL > 0)
//#define DEBUG_SERIAL_GPS
//#define MEAS_ACQUIRNG_TIME
//#define TRACK_HEIDI_STATE
#endif
void setupDebug(int startMS, bool powerOnReset);
#if (DEBUG_LEVEL > 0)
void debugHeidiState(bool powerOnReset);
void checkWakeUpReason();
void DebugPrint(String text, int level);
void DebugPrintln(String text, int level);
void DebugPrint(double number, int digits, int level);
void DebugPrintln(double number, int digits, int level);
void DebugPrint(int number, int level);
void DebugPrintln(int number, int level);
void DebugPrint(unsigned int number, int level);
void DebugPrintln(unsigned int number, int level);

bool dbgGetSem();
void dbgFreeSem();

void _PrintDataSet(t_SendData* DataSet, int dLevel);
void _PrintShortSummary(int dLevel);
void _PrintShortSet(t_SendData* DataSet, int n, int dLevel);
void _PrintFence(int dLevel);
void _PrintHeidiConfig(int dLevel);
void _PrintNotDoneGSMHandling(int dLevel);
#ifdef ACCELEROMETER
void _PrintHeidiAccParams(int dLevel);
#endif
#endif

#define PRINT_CYCLE_STATUS \
    _D(DebugPrintln("Boot time:   " + DateString(&bootTime) + " " + TimeString(&bootTime), DEBUG_LEVEL_2);) \
    _DD( \
      DebugPrintln("bootCycles day: " + String(heidiConfig->c.bootCycles) + ", night: " + String(heidiConfig->c.nightBootCycles), DEBUG_LEVEL_3); \
      DebugPrintln("_currentCycles: " + String(__currentCyclesD(_night())) + "( == " + String(_currentCycles()) + ")", DEBUG_LEVEL_3); \
      DebugPrintln("night from: " + String(heidiConfig->c.nightHourStart) + " to: " + String(heidiConfig->c.nightHourEnd), DEBUG_LEVEL_3); \
      if (_night()) {DebugPrintln("_night = true", DEBUG_LEVEL_2);} else {DebugPrintln("_night = false", DEBUG_LEVEL_3);} \
      if (getState(POWER_SAVE_1)) {DebugPrintln("POWER_SAVE_1 = true", DEBUG_LEVEL_2);} else {DebugPrintln("POWER_SAVE_1 = false", DEBUG_LEVEL_3);} \
    )


#define PRINT_ALERT_STATUS \
  _DD( \
    DebugPrintln("heidiConfig->bootNumber: " + String(heidiConfig->bootNumber), DEBUG_LEVEL_3); \
    if (GPSalert()) {DebugPrintln("GPSalert = true", DEBUG_LEVEL_3);} else {DebugPrintln("GPSalert = false", DEBUG_LEVEL_3);} \
    if (doDataTransmission()) {DebugPrintln("doDataTransmission = true", DEBUG_LEVEL_3);} else {DebugPrintln("doDataTransmission = false", DEBUG_LEVEL_3);} \
    if (getState(PRE_GPS_ALERT)) {DebugPrintln("PRE_GPS_ALERT = true", DEBUG_LEVEL_3);} else {DebugPrintln("PRE_GPS_ALERT = false", DEBUG_LEVEL_3);} \
    if (getState(GPS_ALERT_1)) {DebugPrintln("GPS_ALERT_1 = true", DEBUG_LEVEL_3);} else {DebugPrintln("GPS_ALERT_1 = false", DEBUG_LEVEL_3);} \
    if (getState(GPS_ALERT_2)) {DebugPrintln("GPS_ALERT_2 = true", DEBUG_LEVEL_3);} else {DebugPrintln("GPS_ALERT_2 = false", DEBUG_LEVEL_3);} \
    if (getState(POWER_SAVE_2)) {DebugPrintln("POWER_SAVE_2 = true", DEBUG_LEVEL_3);} else {DebugPrintln("POWER_SAVE_2 = false", DEBUG_LEVEL_3);} \
  )

#endif /* HEIDI_DEBUG_H_ */


