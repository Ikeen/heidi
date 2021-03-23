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
#include "heidi-data.h"
#include "heidi-sys.h"

extern tm bootTime;
extern tm expBootTime;
extern int currentTimeDiffMs;
extern t_ConfigData* heidiConfig;

static int bootTimeStampMs;
static int expectedBootTimeMs;

uint8_t herdeID(){
  return HEIDI_HERDE;
}
uint8_t animalID(){
  return HEIDI_ANIMAL;
}

String _herdeID(){
  if (herdeID() < 10) { return "000" + String(herdeID()); }
  if (herdeID() < 100) { return "00" + String(herdeID()); }
  if (herdeID() < 1000) { return "0" + String(herdeID()); }
  return String(herdeID());
}

bool isInCycle(int firstCycleInDay, int8_t* bootCount){
  if (bootTimeStampMs == INVALID_TIME_VALUE) { return false; } // should never happen
  bool inCycle = false;
  int i = 0;
  int t = firstCycleInDay; //minute of 1st cycle within a new day
  while (t < (1440 + firstCycleInDay)){
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
      if (bootTime.tm_min >= firstCycleInDay) {
		if(t > bootTime.tm_min) { break; }
	  } else {
		if(t > (bootTime.tm_min + 60)) { break; }
	  }
    }
    t += CYCLE_DURATION_MIN;
    i++;
    if (i == getBootCycles()) { i = 0; }
  }
  if (t >=60 ) { t -= 60; }
  //now we have the current (if in cycle) or the next (if out of cycle) boot minute -> t
  //time for calculations - 1st: expected boot time
  int x = (bootTime.tm_hour * 3600 + t * 60) * 1000; //ms time stamp for the next / current cycle so far
  //if((x - bootTimeStampMs) < -SLEEP_DURATION_MSEC){
  if((bootTimeStampMs - x) > getCycleTimeMS()){
	  //woken up early (maybe woken up yesterday, but exp. today)
	  if ((bootTime.tm_hour + 1) > 23) { expBootTime.tm_hour = 0; } else { expBootTime.tm_hour = bootTime.tm_hour + 1; }
  } else if ((x - bootTimeStampMs) > getCycleTimeMS()){
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
  if (bootCount == getBootCycles() - 1){
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
  if(y < (getCycleTimeMS() - MS_PER_DAY)){
    currentTimeDiffMs = y + MS_PER_DAY;
  } else if (y > (MS_PER_DAY - getCycleTimeMS())){
	currentTimeDiffMs = y - MS_PER_DAY;
  } else {
    currentTimeDiffMs = y;
  }
  return true;
}

bool isInTime(const int target_m, const int current_m, const int current_s){
  int current = current_m;
  int target  = target_m;
  int wrap_border = 3600 - (getCycleTimeMS() / 1000);

  if (target > 60)  { target -= 60; }
  current *= 60;
  current += current_s;
  target  *= 60;
  int diffToRegularS = target - current;
  if (diffToRegularS > wrap_border) { diffToRegularS -= 3600; }
  else if (diffToRegularS < (wrap_border * -1)) { diffToRegularS += 3600; }
  #if DEBUG_LEVEL > 0
  if ((diffToRegularS >= -SLEEP_MAX_SHIFT_S) && (diffToRegularS <= SLEEP_MAX_SHIFT_S)) {
    _D(DebugPrintln("InTime: target: " + String(target) + "; current: " + String(current) + "; diff: " + String(diffToRegularS) + "; max_diff: " + String(SLEEP_MAX_SHIFT_S), DEBUG_LEVEL_1));
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
String GetCSVvalue(String line, int no)
{
  int pos;
  int end = -1;
  int len = line.length();
  for(int i=0; i<no; i++){
    pos = end+1;
    end = line.indexOf(';', pos);
    if (end == -1) { end = len; }
  }
  return line.substring(pos, end);
}
bool _night(void) {
  return ((bootTime.tm_hour >= heidiConfig->nightHourStart) | (bootTime.tm_hour < heidiConfig->nightHourEnd));
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
void _copyInt32toBuffer(uint8_t *buffer, int pos, int32_t value){
  buffer[pos]   = (uint8_t) value;
  buffer[pos+1] = (uint8_t) (value >> 8);
  buffer[pos+2] = (uint8_t) (value >> 0x10);
  buffer[pos+3] = (uint8_t) (value >> 0x18);
}
void _copyUint32toBuffer(uint8_t *buffer, int pos, uint32_t value){
  buffer[pos]   = (uint8_t) value;
  buffer[pos+1] = (uint8_t) (value >> 8);
  buffer[pos+2] = (uint8_t) (value >> 0x10);
  buffer[pos+3] = (uint8_t) (value >> 0x18);
}
void _copyInt16toBuffer(uint8_t *buffer, int pos, int16_t value){
  buffer[pos]   = (uint8_t) value;
  buffer[pos+1] = (uint8_t) (value >> 8);
}
void _copyUint16toBuffer(uint8_t *buffer, int pos, uint16_t value){
  buffer[pos]   = (uint8_t) value;
  buffer[pos+1] = (uint8_t) (value >> 8);
}
int32_t _copyBufferToInt32(uint8_t *buffer, int pos){
  int32_t value;
  value  = buffer[pos];
  value |= buffer[pos+1] << 8;
  value |= buffer[pos+2] << 0x10;
  value |= buffer[pos+3] << 0x18;
  return value;
}
uint16_t _copyBufferToUInt16(uint8_t *buffer, int pos){
  uint16_t value;
  value  = buffer[pos];
  value |= buffer[pos+1] << 8;
  return value;
}
String b64u = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";   // base64url dictionary
String b64Encode(uint8_t* Buffer, int length)
{
  String Base64Str = "";
  for (int i=0; i<=(length); i+=3){
    Base64Str += b64u.charAt(Buffer[i] >> 2);
    Base64Str += b64u.charAt(((Buffer[i]   & 3)  << 4) | (Buffer[i+1] >> 4));
    Base64Str += b64u.charAt(((Buffer[i+1] & 15) << 2) | (Buffer[i+2] >> 6));
    Base64Str += b64u.charAt(Buffer[i+2]   & 63);
  }
  if (length % 3 == 2){
    Base64Str += b64u.charAt(Buffer[HEX_BUFFER_LEN-2] >> 2);
    Base64Str += b64u.charAt(((Buffer[HEX_BUFFER_LEN-2] & 3)<< 4) | (Buffer[HEX_BUFFER_LEN-1] >> 4));
    Base64Str += b64u.charAt((Buffer[HEX_BUFFER_LEN-1] & 15) << 2);
  }
  else if (length % 3 == 1){
    Base64Str += b64u.charAt(Buffer[HEX_BUFFER_LEN-1] >> 2);
    Base64Str += b64u.charAt((Buffer[HEX_BUFFER_LEN-1] & 3)<< 4);
  }
  return Base64Str;
}
//Bc6XywAsVgoDH5jLAFBVCgOsmcsAhlUKAwWZywDvVQoDrGc0_x5WCgM
int b64Decode(String Base64Str, uint8_t* Buffer) //returns data length
{
  int a, b, c, d;
  int p = 0;
  int length = Base64Str.length();
  for (int i=0; i<length; i+=4) {
    a = b64u.indexOf(Base64Str[i]);
    b = b64u.indexOf(Base64Str[i+1]);
    if (i < length-2) { c = b64u.indexOf(Base64Str[i+2]); } else { c = -1; }
    if (i < length-3) { d = b64u.indexOf(Base64Str[i+3]); } else { d = -1; }
    Buffer[p++] = (uint8_t)((a << 2) | ((b >> 4) & 0x03));
    if (c >= 0) { Buffer[p++] = (uint8_t)(((b << 4) & 0xF0) | ((c >> 2) & 0x0F)); }
    if (d >= 0) { Buffer[p++] = (uint8_t)(((c << 6) & 0xC0) | (d & 0x3F)); }
  }
  return p;
}
uint16_t crc16F(String data) //CRC-16/CCITT-FALSE
{
  uint16_t crc = 0xFFFF;
  int len = data.length();
  for (int i = 0; i < len; i++)
  {
    uint32_t x = ((crc >> 8) ^ (uint8_t)data.charAt(i)) & 0xFF;
    x ^= (x >> 4);
    crc = ((crc << 8) ^ (x << 12) ^ (x << 5) ^ x) & 0xFFFF;
  }
  return crc;
}
uint32_t hex2int(String data)
{
  String   _alpabet = "0123456789ABCDEF";
  String   _data = data;
  uint32_t result = 0;
  int len = _data.length();
  _data.toUpperCase();
  for (int i=0; i<len;i++){
    result = result << 4;
    result += _alpabet.indexOf(_data.charAt(i));
  }
  return result;
}

