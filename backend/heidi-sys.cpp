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
static int currentBootTableEntry = NO_TABLE_ENTRY;
static int nextBootTableEntry = NO_TABLE_ENTRY;
static int prevBootCycle = REFETCH_SYS_TIME;
static uint8_t bootTimeTable[MAX_CYCLES_PER_DAY][4];

uint8_t herdeID(){
  return HEIDI_HERDE;
}
uint8_t animalID(){
  return HEIDI_ANIMAL;
}

String intString4(uint16_t val){
  if (val < 10) { return "000" + String(val); }
  if (val < 100) { return "00" + String(val); }
  if (val < 1000) { return "0" + String(val); }
  return String(val);
}


String _herdeID(){
  return intString4(herdeID());
}
String _animalID(){
  return intString4(animalID());
}

void setupCycleTable(void){
  bool wrap = false;
  bool daywrap = false;
  uint8_t cycleNo;
  uint8_t cycleMinute;
  uint8_t cycleHour;
  int turnbase;
  int counter = 0;
  for(int i=0; i<MAX_CYCLES_PER_DAY; i++){
    bootTimeTable[i][0] = 0; bootTimeTable[i][1] = 0; bootTimeTable[i][2] = 0;
  }
  //night
  if (heidiConfig->nightHourStart != heidiConfig->nightHourEnd){
    wrap = false;
    daywrap = false;
    cycleNo = 1;
    cycleHour = heidiConfig->nightHourStart;
    cycleMinute = (herdeID() % 12) * 5; //1st cycle is odd to avoid LoRa jamming each other
    turnbase = counter;
    while ((counter < MAX_CYCLES_PER_DAY) && __night(cycleHour) ){
       bootTimeTable[counter][0] = cycleNo;
      bootTimeTable[counter][1] = cycleMinute;
      bootTimeTable[counter][2] = cycleHour;
      counter++;
      cycleNo++;
      cycleMinute += heidiConfig->nightSleepMin;
      if (cycleNo > __currentCycles(true)) { cycleNo = 1; }
      if (cycleMinute >= 60) { cycleHour++; cycleMinute -= 60; }
      if (cycleHour >= 24) { cycleHour = 0; daywrap = true;}
      if ((heidiConfig->nightHourEnd < heidiConfig->nightHourStart) && (!daywrap)){ continue; }
      if ((cycleHour >= heidiConfig->nightHourEnd) && (!wrap)) { cycleHour = heidiConfig->nightHourStart; wrap = true; }
      if (wrap && (    (cycleHour > bootTimeTable[turnbase][2])
                    || ((cycleHour == bootTimeTable[turnbase][2]) && (cycleMinute >= bootTimeTable[turnbase][1])))) { break; }
    }
  }
  //day
  wrap = false;
  daywrap = false;
  cycleNo = 1;
  cycleHour = heidiConfig->nightHourEnd;
  cycleMinute = (herdeID() % 12) * 5; //1st cycle is odd to avoid LoRa jamming each other
  turnbase = counter;
  while ((counter < MAX_CYCLES_PER_DAY)){
    bootTimeTable[counter][0] = cycleNo;
    bootTimeTable[counter][1] = cycleMinute;
    bootTimeTable[counter][2] = cycleHour;
    counter++;
    cycleNo++;
    cycleMinute += heidiConfig->sleepMinutes;
    if (cycleNo > __currentCycles(false)) { cycleNo = 1; }
    if (cycleMinute >= 60) { cycleHour++; cycleMinute -= 60; }
    if (cycleHour >= 24) { cycleHour = 0; daywrap = true; }
    if (heidiConfig->nightHourStart == heidiConfig->nightHourEnd) { wrap = daywrap; }
    else if  (heidiConfig->nightHourStart > heidiConfig->nightHourEnd) {
      if ((cycleHour >= heidiConfig->nightHourStart) && (!wrap)) { cycleHour = heidiConfig->nightHourEnd; wrap = true; }
    } else {
      if ((cycleHour >= heidiConfig->nightHourStart) && (!wrap) && (daywrap)) { cycleHour = heidiConfig->nightHourEnd; wrap = true; }
    }
    if (wrap) {
      if (cycleHour > bootTimeTable[turnbase][2]) { break; }
      if ((cycleHour == bootTimeTable[turnbase][2]) && (cycleMinute >= bootTimeTable[turnbase][1])){ break; }
    }
  }
  //sort
  for(int i=0; i<(counter-1); i++){
    for (int j=i+1; j<counter; j++){
      if ((bootTimeTable[j][2] * 60 + bootTimeTable[j][1]) < (bootTimeTable[i][2] * 60 + bootTimeTable[i][1])){
        uint8_t dummy;
        dummy = bootTimeTable[i][0]; bootTimeTable[i][0] = bootTimeTable[j][0]; bootTimeTable[j][0] = dummy;
        dummy = bootTimeTable[i][1]; bootTimeTable[i][1] = bootTimeTable[j][1]; bootTimeTable[j][1] = dummy;
        dummy = bootTimeTable[i][2]; bootTimeTable[i][2] = bootTimeTable[j][2]; bootTimeTable[j][2] = dummy;
      }
    }
  }
}

bool isInCycle(int8_t* bootCount){
  currentBootTableEntry = NO_TABLE_ENTRY;
  nextBootTableEntry = NO_TABLE_ENTRY;
  if (bootTimeStampMs == INVALID_TIME_VALUE) { return false; } // should never happen
  bool inCycle = false;
  int cnt = 0;
  //find matching cycle
  while((bootTimeTable[cnt][0] != 0) && (cnt < MAX_CYCLES_PER_DAY)){
    bool upperwrap = false;
    int upperborder = (bootTimeTable[cnt][2] * 3600) + (bootTimeTable[cnt][1] * 60) + _cyleMaxDiff_s(bootTimeTable[cnt][2]);
    upperborder *= S_TO_mS_FACTOR;
    if (upperborder > MS_PER_DAY) { upperwrap = true; upperborder -= MS_PER_DAY; }
    bool lowerwrap = false;
    int lowerborder = (bootTimeTable[cnt][2] * 3600) + (bootTimeTable[cnt][1] * 60) - _cyleMaxDiff_s(bootTimeTable[cnt][2]);
    lowerborder *= S_TO_mS_FACTOR;
    if (lowerborder > MS_PER_DAY) { lowerwrap = true; lowerborder += MS_PER_DAY; }
    nextBootTableEntry = cnt;
    /* _D(DebugPrintln("Check boot table [" + String(cnt) + "]: upper border: " + String(upperborder) \
                    + " / lower border: " + String(lowerborder)  + " / boot time ms: " + String(bootTimeStampMs) \
                    + "(" + LenTwo(String(bootTimeTable[nextBootTableEntry][2])) + ":" \
                    + LenTwo(String(bootTimeTable[nextBootTableEntry][1])) + ")", DEBUG_LEVEL_1);)*/
    if ((!lowerwrap) && (!upperwrap)){
      if ((bootTimeStampMs > lowerborder) && (bootTimeStampMs < upperborder)) { currentBootTableEntry = cnt; break; }
    }
    if (lowerwrap || upperwrap){
      if ((bootTimeStampMs > lowerborder) || (bootTimeStampMs < upperborder)) { currentBootTableEntry = cnt; break; }
    }
    if (lowerborder > bootTimeStampMs) { break; }
    cnt++;
  }
  if (currentBootTableEntry != NO_TABLE_ENTRY){
    inCycle = true;
    if (*bootCount == bootTimeTable[currentBootTableEntry][0]) {
      _D(DebugPrintln("Boot: regular cycle " + String(bootTimeTable[currentBootTableEntry][0]) + " boot", DEBUG_LEVEL_1);)
    } else {
      _D(DebugPrintln("Boot: need to set boot count to " + String(bootTimeTable[currentBootTableEntry][0]), DEBUG_LEVEL_1);)
      *bootCount = bootTimeTable[currentBootTableEntry][0];
    }
    nextBootTableEntry = currentBootTableEntry + 1;
    if (nextBootTableEntry >= MAX_CYCLES_PER_DAY) { nextBootTableEntry = 0; }
    prevBootCycle = bootTimeTable[currentBootTableEntry][0] - 1;
    if (prevBootCycle <= 0) { prevBootCycle = _currentCycles(); }
    expBootTime.tm_hour = bootTimeTable[currentBootTableEntry][2];
    expBootTime.tm_min  = bootTimeTable[currentBootTableEntry][1];
    expBootTime.tm_sec  = 0;
  } else {
    nextBootTableEntry = cnt;
    if (nextBootTableEntry >= MAX_CYCLES_PER_DAY) { nextBootTableEntry = 0; }
    prevBootCycle = bootTimeTable[nextBootTableEntry][0] - 1;
    if (prevBootCycle <= 0) { prevBootCycle = _currentCycles(); }
    expBootTime.tm_hour = bootTimeTable[nextBootTableEntry][2];
    expBootTime.tm_min  = bootTimeTable[nextBootTableEntry][1];
    expBootTime.tm_sec  = 0;
  }
  expectedBootTimeMs  = expBootTime.tm_hour * 3600 + expBootTime.tm_min * 60;
  expectedBootTimeMs *= S_TO_mS_FACTOR;
  if (!calcCurrentTimeDiff()) { return false; };
  _D(
    if(currentBootTableEntry != NO_TABLE_ENTRY){
      DebugPrintln("Current cycle time " + LenTwo(String(bootTimeTable[currentBootTableEntry][2])) + ":"
                    + LenTwo(String(bootTimeTable[currentBootTableEntry][1])), DEBUG_LEVEL_1);
    } else {
      DebugPrintln("No current cycle time ", DEBUG_LEVEL_1);

    }
    if(nextBootTableEntry != NO_TABLE_ENTRY){
      DebugPrintln("Next cycle time " + LenTwo(String(bootTimeTable[nextBootTableEntry][2])) + ":"
                    + LenTwo(String(bootTimeTable[nextBootTableEntry][1])), DEBUG_LEVEL_1);
    } else {
      DebugPrintln("No next cycle time ", DEBUG_LEVEL_1);

  }

  )
  return inCycle;
}

int  getNextBootMS(){
  if (nextBootTableEntry == NO_TABLE_ENTRY) { return NO_TIME_MS; }
  int result = bootTimeTable[nextBootTableEntry][2] * 3600; //bootHour
  result += bootTimeTable[nextBootTableEntry][1] * 60; //bootHour
  result *= 1000;
  return result;
}

int  timeToNextBootMS(){
  int result;
  if (GPSalert()){
    _DD(DebugPrintln("timeToNextBoot = ALERT Boot time: ", DEBUG_LEVEL_3);)
    result = SLEEP_DUR_ALERT;
  } else {
    result = getNextBootMS();
    if (result == NO_TIME_MS) { return getCycleTimeMS(); }
    result -= bootTimeStampMs;
    if (result < 0) { result += MS_PER_DAY; }
  }
  _DD(DebugPrintln("timeToNextBoot (seconds): " + String((int)( result / 1000)), DEBUG_LEVEL_3);)
  return result;
}

bool doDataTransmission(){
  bool result = (heidiConfig->bootCount >= _currentCycles());
  /* not "==" due to a weird night-cycles setting it's possible to start a day with values >= _currentCycles() */
  result |= getState(RESET_INITS);
  /* after power up we need settings... */
  result &= !getState(POWER_SAVE_2);
  /* ...but do get or not transmit data on very low battery - that may fail and lead us to a endless boot loop ... */

  if (result) { setError(E_TRANSMIT_REGULAR); } //!!!!!!!!!!!!!!!

  result |= (GPSalert() && !getError(E_COULD_NOT_FETCH_GPS));
  /* ...but we definitely give a try if we have an alert
   * On PRE_GPS_ALERT state we want to check for new fence data - that should work, because PRE_GPS_ALERT state could not be reached
   * without GPS data. But in case while stepping through alert states we cannot fetch position one time, it is senseless to send a
   * SMS without transferring current position data. Therefore:  && !getError(E_COULD_NOT_FETCH_GPS)
   */

  if(GPSalert()) {
    setError(E_TRANSMIT_ALERT);  //!!!!!!!!!!!!!!!
    if(getError(E_COULD_NOT_FETCH_GPS)) { setError(E_TRANSMIT_ALERT_NO_GPS); } //!!!!!!!!!!!!!!!
  }
  return result;
}

bool GPSalert(){
  #ifdef GPS_MODULE
  return (getState(PRE_GPS_ALERT | GPS_ALERT_1 | GPS_ALERT_2 ) && !getState(GPS_ALERT_PSD));
  #else
  return false;
  #endif
}

int prevBootCycleNo(void){
  return(prevBootCycle);
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
  int sleep_max_shift = _currentCyleLen_m();
  sleep_max_shift *= 3; //in seconds (*60), 5% (/20)
  if (target > 60)  { target -= 60; }
  current *= 60;
  current += current_s;
  target  *= 60;
  int diffToRegularS = target - current;
  if (diffToRegularS > wrap_border) { diffToRegularS -= 3600; }
  else if (diffToRegularS < (wrap_border * -1)) { diffToRegularS += 3600; }
  #if DEBUG_LEVEL > 0
  if ((diffToRegularS >= (sleep_max_shift * -1)) && (diffToRegularS <= sleep_max_shift)) {
    _D(DebugPrintln("InTime: target: " + String(target) + "; current: " + String(current) + "; diff: " + String(diffToRegularS) + "; max_diff: " + String(sleep_max_shift), DEBUG_LEVEL_1));
  }
  #endif
  return ((diffToRegularS >= (sleep_max_shift * -1)) && (diffToRegularS <= sleep_max_shift));
}

void setBootTimeFromCurrentTime(tm* sysTime, tm* bootTime){
  int sysMs = sysTime->tm_hour * 3600 + sysTime->tm_min * 60 + sysTime->tm_sec;
  sysMs *= 1000;
  int bootMs = sysMs - millis();
  _copyDate(sysTime, bootTime);
  _copyTime(sysTime, bootTime);
  if (bootMs < 0) {
    bootMs += MS_PER_DAY;
    bootTime->tm_mday -= 1;
    mktime(bootTime);
  }
  _DD(DebugPrintln("Set boot time ms to: " + String(bootMs), DEBUG_LEVEL_3);)
  bootTimeStampMs = bootMs;
  bootTime->tm_hour = (int)(bootMs / 3600000);
  bootMs -= bootTime->tm_hour * 3600000;
  bootTime->tm_min = (int)(bootMs / 60000);
  bootMs -= bootTime->tm_min * 60000;
  bootTime->tm_sec = bootMs / 1000;
  mktime(bootTime);
}

void initSysTimeMS(void){
  bootTimeStampMs     = INVALID_TIME_VALUE;
  expectedBootTimeMs  = INVALID_TIME_VALUE;
}

bool getSysTime(tm *info){
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

String getCSVvalue(String line, int no)
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
  return __night(bootTime.tm_hour);
}
bool __night(uint8_t hour) {
  if (heidiConfig->nightHourStart == heidiConfig->nightHourEnd) { return false; }
  if (heidiConfig->nightHourStart < heidiConfig->nightHourEnd){
    return ((hour >= heidiConfig->nightHourStart) && (hour < heidiConfig->nightHourEnd));
  }
  if (heidiConfig->nightHourStart > heidiConfig->nightHourEnd){
    return ((hour >= heidiConfig->nightHourStart) || (hour < heidiConfig->nightHourEnd));
  }
  return false;
}

uint8_t _currentCyleLen_m(){ //minutes (max 60)
  if (_night()) { return heidiConfig->nightSleepMin; }
  else { return heidiConfig->sleepMinutes; }
}
uint8_t  _currentCyleMaxDiff_s(){ //seconds (max 180 = 5% from 3600)
  if (_night()) { return heidiConfig->nightSleepMin * 3; } //(*60 / 20 -> *3)
  else { return heidiConfig->sleepMinutes * 3; }
}
uint8_t  _cyleMaxDiff_s(uint8_t hour){ //seconds (max 180 = 5% from 3600)
  if (__night(hour)) { return heidiConfig->nightSleepMin * 3; } //(*60 / 20 -> *3)
  else { return heidiConfig->sleepMinutes * 3; }
}
uint8_t _currentCycles(){
  return __currentCycles(_night());
}
uint8_t __currentCycles(bool isNight){
  uint8_t result = heidiConfig->bootCycles;
  if(isNight){ result = heidiConfig->nightBootCycles;}
  if (getState(POWER_SAVE_1)) {result *= 2;}
  if (result > MAX_BOOT_CYCLES) { result = MAX_BOOT_CYCLES; }
  return result;
}
_D(
  uint8_t __currentCyclesD(bool isNight){
    uint8_t result = heidiConfig->bootCycles;
    if(isNight){
      result = heidiConfig->nightBootCycles;
      _DD(DebugPrint(" __curCyc: night", DEBUG_LEVEL_3); } else { DebugPrint(" __curCyc: day", DEBUG_LEVEL_3);)
    }
    if (getState(POWER_SAVE_1)) {
      result *= 2;
      _DD(DebugPrintln(", powerSave", DEBUG_LEVEL_3); } else { DebugPrintln(", no powerSave", DEBUG_LEVEL_3);)
    }
    return result;
  }
)

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

