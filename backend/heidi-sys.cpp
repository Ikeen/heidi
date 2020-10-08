/*
 * heidi-sys.cpp
 *
 *  Created on: 08.07.2020
 *      Author: frank
 */
#include <Arduino.h>
#include "heidi-defines.h"
#include "heidi-error.h"
#include "heidi-debug.h"
#include "heidi-sys.h"

extern tm bootTime;
extern tm expBootTime;
extern int currentTimeDiffMs;

static int bootTimeStampMs;
static int expectedBootTimeMs;

uint8_t herdeID(){
  return HEIDI_HERDE;
}
uint8_t animalID(){
  return HEIDI_ANIMAL;
}
bool isInCycle(int firstCycleInHour, int8_t* bootCount){
  if (bootTimeStampMs == INVALID_TIME_VALUE) { return false; } // should never happen
  bool inCycle = false;
  int i = 0;
  int t = firstCycleInHour; //minute of 1st cycle within a new hour
  while (t < (60 + firstCycleInHour)){
    if (isInTime(t, bootTime.tm_min, bootTime.tm_sec)){
      inCycle = true;
   	  if (*bootCount == i) {
        _D(DebugPrintln("Boot: regular cycle " + String(i) + " boot", DEBUG_LEVEL_1));
      } else {
        _D(DebugPrintln("Boot: need to set boot count to " + String(i), DEBUG_LEVEL_1));
        *bootCount = i;
      }
      break;
    } else {
      if (bootTime.tm_min >= firstCycleInHour) {
		if(t > bootTime.tm_min) { break; }
	  } else {
		if(t > (bootTime.tm_min + 60)) { break; }
	  }
    }
    t += CYCLE_DURATION_MIN;
    i++;
    if (i == BOOT_CYCLES) { i = 0; }
  }
  if (t >=60 ) { t -= 60; }
  //now we have the current (if in cycle) or the next (if out of cycle) boot minute -> t
  //time for calculations - 1st: expected boot time
  int x = (bootTime.tm_hour * 3600 + t * 60) * 1000; //ms time stamp for the next / current cycle so far
  if((x - bootTimeStampMs) < -SLEEP_DURATION_MSEC){
	  //woken up early (maybe woken up yesterday, but exp. today)
	  if ((bootTime.tm_hour + 1) > 23) { expBootTime.tm_hour = 0; } else { expBootTime.tm_hour = bootTime.tm_hour + 1; }
  } else if ((x - bootTimeStampMs) > SLEEP_DURATION_MSEC){
	  //woken up late (maybe woken up today, but exp. yesterday)
	  if ((bootTime.tm_hour - 1) < 0) { expBootTime.tm_hour = 23; } else { expBootTime.tm_hour = bootTime.tm_hour - 1; }
  } else {
    expBootTime.tm_hour = bootTime.tm_hour;
  }
  expBootTime.tm_min = t;
  expBootTime.tm_sec = 0;
  expectedBootTimeMs = expBootTime.tm_hour * 3600 + t * 60;
  expectedBootTimeMs *= 1000;
  //2nd time diff
  if (!calcCurrentTimeDiff()) { return false; };
  return inCycle;
}
bool doDataTransmission(int8_t bootCount){
  if (bootCount == BOOT_CYCLES - 1){
    if ((bootTime.tm_hour > NIGHT_HOUR_START) && (bootTime.tm_hour < NIGHT_HOUR_END)){
      if ((bootTime.tm_hour % 2) == 0) {return true;} else {return false;}
    }
    return true;
  }else{
    return false;
  }
}

bool calcCurrentTimeDiff(){
  if ((expectedBootTimeMs == INVALID_TIME_VALUE) || (bootTimeStampMs == INVALID_TIME_VALUE)) {
    currentTimeDiffMs = 0;
	return false;
  }
  int y = expectedBootTimeMs - bootTimeStampMs;
  if(y < (SLEEP_DURATION_MSEC - MS_PER_DAY)){
    currentTimeDiffMs = y + MS_PER_DAY;
  } else if (y > (MS_PER_DAY - SLEEP_DURATION_MSEC)){
	currentTimeDiffMs = y - MS_PER_DAY;
  } else {
    currentTimeDiffMs = y;
  }
  return true;
}

bool isInTime(const int target_m, const int current_m, const int current_s){
  int current = current_m;
  int target  = target_m;
  int wrap_border = 3600 - (SLEEP_DURATION_MSEC / 1000);

  if (target > 60)  { target -= 60; }
  current *= 60;
  current += current_s;
  target  *= 60;
  int diffToRegularS = target - current;
  if (diffToRegularS > wrap_border) { diffToRegularS -= 3600; }
  else if (diffToRegularS < (wrap_border * -1)) { diffToRegularS += 3600; }
  #if DEBUG_LEVEL > 0
  if ((diffToRegularS >= -SLEEP_MAX_SHIFT_S) && (diffToRegularS <= SLEEP_MAX_SHIFT_S)) {
    _D(DebugPrintln("InTime: target: " + String(target) + "; current: " + String(current) + "diff: " + String(diffToRegularS) + "max_diff: " + String(SLEEP_MAX_SHIFT_S), DEBUG_LEVEL_1));
  }
  #endif
  return ((diffToRegularS >= -SLEEP_MAX_SHIFT_S) && (diffToRegularS <= SLEEP_MAX_SHIFT_S));
}

void SetBootTimeFromMs(int timeStampMs){
  int bootms = timeStampMs;
  if (bootms < 0) { bootms += MS_PER_DAY; }
  bootTimeStampMs = bootms;
  bootTime.tm_hour = (int)(bootms / 3600000);
  bootms -= bootTime.tm_hour * 3600000;
  bootTime.tm_min = (int)(bootms / 60000);
  bootms -= bootTime.tm_min * 60000;
  bootTime.tm_sec = bootms / 1000;
}

int8_t  GetLocalTimeHourShift(){
  /*
  tm cur, summer, winter;
  if (!GetSysTime(&cur)){ return 0; }
  summer.tm_year = cur.tm_year;
  summer.tm_mon  = 3;
  summer.tm_mday = 31;
  */
  return 2;
}
bool GetSysTime(tm *info){
  uint32_t count = 500;
  time_t now;
  do{
    time(&now);
    localtime_r(&now, info);
    if(info->tm_year > (2016 - 1900)){ info->tm_year += 1900; info->tm_mon++; return true; }
    delay(10);
  }while(count--);
  return false;
}
void _copyTime(tm *from, tm *to){
  to->tm_hour = from->tm_hour;
  to->tm_min  = from->tm_min;
  to->tm_sec  = from->tm_sec;
}
void _copyDate(tm *from, tm *to){
  to->tm_wday = from->tm_wday;
  to->tm_mday = from->tm_mday;
  to->tm_yday = from->tm_yday;
  to->tm_mon  = from->tm_mon;
  to->tm_year = from->tm_year;
}
void _copyInt32toBuffer(byte *buffer, int pos, int32_t value){
  buffer[pos]   = (byte) value;
  buffer[pos+1] = (byte) (value >> 8);
  buffer[pos+2] = (byte) (value >> 0x10);
  buffer[pos+3] = (byte) (value >> 0x18);
}
void _copyint32toBuffer(byte *buffer, int pos, uint32_t value){
  buffer[pos]   = (byte) value;
  buffer[pos+1] = (byte) (value >> 8);
  buffer[pos+2] = (byte) (value >> 0x10);
  buffer[pos+3] = (byte) (value >> 0x18);
}
void _copyInt16toBuffer(byte *buffer, int pos, int16_t value){
  buffer[pos]   = (byte) value;
  buffer[pos+1] = (byte) (value >> 8);
}
void _copyint16toBuffer(byte *buffer, int pos, uint16_t value){
  buffer[pos]   = (byte) value;
  buffer[pos+1] = (byte) (value >> 8);
}

