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

static int bootTimeStampMs = INVALID_TIME_VALUE;
static int expectedBootTimeMs = INVALID_TIME_VALUE;
static int16_t currentBootTableEntry = NO_TABLE_ENTRY;
static int16_t nextBootTableEntry    = NO_TABLE_ENTRY;
static int16_t bootTableLenghtVal    = NO_VALID_TABLE;
static int prevBootCycle = REFETCH_SYS_TIME;

uint8_t bootTimeTable[MAX_CYCLES_PER_DAY][3]; // 0=minute, 1=hour, 2=type

uint8_t herdeID(){
  return HEIDI_HERDE;
}
uint8_t animalID(){
  return HEIDI_ANIMAL;
}

bool wasPowerOnReset(void){
  return rtc_get_reset_reason(0) == POWERON_RESET;
}
bool wasReset(void){
  return rtc_get_reset_reason(0) != NO_MEAN;
}
bool sysWasbrownOut(void){
  return (rtc_get_reset_reason(0) == RTCWDT_BROWN_OUT_RESET);
}


bool wasRegularWakeUp(void){
  bool rst = rtc_get_reset_reason(0) == DEEPSLEEP_RESET;
  bool tmr = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER;
  bool ulp = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_ULP;
  return (rst & (tmr | ulp));
}
bool wasULPWakeUp(void){
  return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_ULP;
}

esp_sleep_wakeup_cause_t getWakeUpReason(void){
  return esp_sleep_get_wakeup_cause();
}


/*
 * calculates the shift of wake-up times for different Heidi-networks to avoid LoRa jamming
 * depending on minimum cycle time, so different networks will usually wake up on different times
 *   -> the shorter MIN_CYCLE_DURATION, the less different networks can operate at the same area
 *   -> networks operating at the same area should have successive herd IDs
 */
uint8_t cycleShift(){
  return uint8_t(herdeID() % MIN_CYCLE_DURATION);
}

String intString4(uint16_t val){
  if (val < 10) { return "000" + String(val); }
  if (val < 100) { return "00" + String(val); }
  if (val < 1000) { return "0" + String(val); }
  return String(val);
}
String hexString2(uint8_t val){
  String rc = String(val, HEX);
  if (rc.length() < 2) { return "0x0" + rc; }
  return "0x" + rc;
}
String hexString8(uint16_t val){
  String rc = String(val, HEX);
  int l = rc.length();
  if (l < 2) { return "0x0000000" + rc; }
  if (l < 3) { return "0x000000" + rc; }
  if (l < 4) { return "0x00000" + rc; }
  if (l < 5) { return "0x0000" + rc; }
  if (l < 6) { return "0x000" + rc; }
  if (l < 7) { return "0x00" + rc; }
  if (l < 8) { return "0x0" + rc; }
  return "0x" + rc;
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
  uint16_t cycleNo;
  uint16_t cycleMinute;
  uint16_t cycleHour;
  int turnbase;
  int counter = 0;
  for(int i=0; i<MAX_CYCLES_PER_DAY; i++){
    bootTimeTable[i][0] = 0; bootTimeTable[i][1] = 0; bootTimeTable[i][2] = CYCLE_NOT_SET;
  }
  //night
  if (heidiConfig->c.nightHourStart != heidiConfig->c.nightHourEnd){
    wrap = false;
    daywrap = false;
    cycleNo = 0;
    cycleHour = heidiConfig->c.nightHourStart;
    cycleMinute = cycleShift(); //to avoid LoRa jamming -
    turnbase = counter;
    while ((counter < MAX_CYCLES_PER_DAY) && __night(cycleHour) ){
      cycleNo++;
      bootTimeTable[counter][0] = cycleMinute;
      bootTimeTable[counter][1] = cycleHour;
      bootTimeTable[counter][2] = MEASURING_CYCLE;
      if (cycleNo == __currentCycles(true)) { bootTimeTable[counter][2] |= TRANSMISSION_CYCLE; cycleNo = 0; }
      counter++;
      //calculate next cycle
      cycleMinute += heidiConfig->c.nightSleepMin;
      if (cycleMinute >= 60) { cycleHour++; cycleMinute -= 60; }
      if (cycleHour >= 24) { cycleHour = 0; daywrap = true;}
      if ((heidiConfig->c.nightHourEnd < heidiConfig->c.nightHourStart) && (!daywrap)){ continue; }
      if ((cycleHour >= heidiConfig->c.nightHourEnd) && (!wrap)) { cycleHour = heidiConfig->c.nightHourStart; wrap = true; }
      if (wrap && (    (cycleHour > bootTimeTable[turnbase][1])
                    || ((cycleHour == bootTimeTable[turnbase][1]) && (cycleMinute >= bootTimeTable[turnbase][0])))) { break; }
    }
  }
  //day
  wrap = false;
  daywrap = false;
  cycleNo = 0;
  cycleHour = heidiConfig->c.nightHourEnd;
  cycleMinute = cycleShift(); //1st cycle is odd to avoid LoRa jamming each other
  turnbase = counter;
  while ((counter < MAX_CYCLES_PER_DAY)){
    cycleNo++;
    bootTimeTable[counter][0] = cycleMinute;
    bootTimeTable[counter][1] = cycleHour;
    bootTimeTable[counter][2] = MEASURING_CYCLE;
    if (cycleNo == __currentCycles(false)) { bootTimeTable[counter][2] |= TRANSMISSION_CYCLE; cycleNo = 0; }
    counter++;
    //calculate next cycle
    cycleMinute += heidiConfig->c.sleepMinutes;
    if (cycleMinute >= 60) { cycleHour++; cycleMinute -= 60; }
    if (cycleHour >= 24) { cycleHour = 0; daywrap = true; }
    if (heidiConfig->c.nightHourStart == heidiConfig->c.nightHourEnd) { wrap = daywrap; }
    else if  (heidiConfig->c.nightHourStart > heidiConfig->c.nightHourEnd) {
      if ((cycleHour >= heidiConfig->c.nightHourStart) && (!wrap)) { cycleHour = heidiConfig->c.nightHourEnd; wrap = true; }
    } else {
      if ((cycleHour >= heidiConfig->c.nightHourStart) && (!wrap) && (daywrap)) { cycleHour = heidiConfig->c.nightHourEnd; wrap = true; }
    }
    if (wrap) {
      if (cycleHour > bootTimeTable[turnbase][1]) { break; }
      if ((cycleHour == bootTimeTable[turnbase][1]) && (cycleMinute >= bootTimeTable[turnbase][0])){ break; }
    }
  }
  //fix points
  uint16_t fixHour = FIXPOINT_1_UTC;
  for(int k=0; k<2; k++){
    int fixPoint = (fixHour * 60) + cycleShift();
    bool meetCycle = false;
    for(int i=0; i<counter; i++){
      if ((bootTimeTable[i][1] * 60 + bootTimeTable[i][0]) == fixPoint){
        meetCycle = true;
        bootTimeTable[i][2] |= FIX_POINT_CYCLE;
        break;
      }
    }
    if (!meetCycle){
      bootTimeTable[counter][0] = cycleShift();;
      bootTimeTable[counter][1] = fixHour;
      bootTimeTable[counter][2] = FIX_POINT_CYCLE;
      counter++;
    }
    if(FIXPOINT_2_UTC == FIXPOINT_1_UTC) { break; }
    fixHour = FIXPOINT_2_UTC;
  }
  //sort
  for(int i=0; i<(counter-1); i++){
    for (int j=i+1; j<counter; j++){
      if ((bootTimeTable[j][1] * 60 + bootTimeTable[j][0]) < (bootTimeTable[i][1] * 60 + bootTimeTable[i][0])){
        uint8_t dummy;
        for(int k=0; k<3; k++){
          dummy = bootTimeTable[i][k]; bootTimeTable[i][k] = bootTimeTable[j][k]; bootTimeTable[j][k] = dummy;
        }
      }
    }
  }
  /*
  _D(
    for(int i=0; i<bootTableLength(); i++){
      DebugPrintln("Table entry " + String (i) + ", " + LenTwo(String(bootTimeTable[i][1]))
            + ":" + LenTwo(String(bootTimeTable[i][0])) + ", 0x"
            + LenTwo(String(bootTimeTable[i][2], HEX)), DEBUG_LEVEL_1);
      pause(20);
    }
  )
  */
  bootTableLenghtVal = counter;
}

bool isInCycle(void){
  currentBootTableEntry = NO_TABLE_ENTRY;
  nextBootTableEntry = NO_TABLE_ENTRY;
  if (bootTimeStampMs == INVALID_TIME_VALUE) { return false; } // should never happen
  bool inCycle = false;
  int cnt = 0;
  //find matching cycle
  while(cnt < MAX_CYCLES_PER_DAY){
    if (bootTimeTable[cnt][2] == CYCLE_NOT_SET) { cnt++; continue; } //should never happen
    bool upperwrap = false;
    int upperborder = (bootTimeTable[cnt][1] * 3600) + (bootTimeTable[cnt][0] * 60) + _cyleMaxDiff_s(bootTimeTable[cnt][1]);
    upperborder *= S_TO_mS_FACTOR;
    if (upperborder > MS_PER_DAY) { upperwrap = true; upperborder -= MS_PER_DAY; }
    bool lowerwrap = false;
    int lowerborder = (bootTimeTable[cnt][1] * 3600) + (bootTimeTable[cnt][0] * 60) - _cyleMaxDiff_s(bootTimeTable[cnt][1]);
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
    if (heidiConfig->bootNumber == currentBootTableEntry) {
      _D(DebugPrintln("Boot: regular cycle " + String(currentBootTableEntry) + ", type 0x" + String(bootTimeTable[currentBootTableEntry][2], HEX) + " boot", DEBUG_LEVEL_1);)
    } else {
      _D(DebugPrintln("Boot: need to set boot count to " + String(currentBootTableEntry) + ", type 0x" + String(bootTimeTable[currentBootTableEntry][2], HEX), DEBUG_LEVEL_1);)
      heidiConfig->bootNumber = currentBootTableEntry;
    }
    nextBootTableEntry = currentBootTableEntry + 1;
    if (nextBootTableEntry >= MAX_CYCLES_PER_DAY) { nextBootTableEntry = 0; }
    prevBootCycle = currentBootTableEntry - 1;
    if (prevBootCycle <= 0) { prevBootCycle = bootTableLength() - 1; }
    expBootTime.tm_hour = bootTimeTable[currentBootTableEntry][1];
    expBootTime.tm_min  = bootTimeTable[currentBootTableEntry][0];
    expBootTime.tm_sec  = 0;
  } else {
    nextBootTableEntry = cnt;
    if (nextBootTableEntry >= MAX_CYCLES_PER_DAY) { nextBootTableEntry = 0; }
    prevBootCycle = nextBootTableEntry;
    if (prevBootCycle <= 0) { prevBootCycle = bootTableLength() - 1; }
    expBootTime.tm_hour = bootTimeTable[nextBootTableEntry][1];
    expBootTime.tm_min  = bootTimeTable[nextBootTableEntry][0];
    expBootTime.tm_sec  = 0;
  }
  expectedBootTimeMs  = expBootTime.tm_hour * 3600 + expBootTime.tm_min * 60;
  expectedBootTimeMs *= S_TO_mS_FACTOR;
  if (!calcCurrentTimeDiff()) { return false; };
  _D(
    if(currentBootTableEntry != NO_TABLE_ENTRY){
      DebugPrintln("Current [" + String(currentBootTableEntry) + "] cycle time "
                    + LenTwo(String(bootTimeTable[currentBootTableEntry][1])) + ":"
                    + LenTwo(String(bootTimeTable[currentBootTableEntry][0]))
                    + ", type 0x" + String(bootTimeTable[currentBootTableEntry][2], HEX), DEBUG_LEVEL_1);
    } else {
      DebugPrintln("No current cycle time ", DEBUG_LEVEL_1);
    }
    if(nextBootTableEntry != NO_TABLE_ENTRY){
      DebugPrintln("Next [" + String(nextBootTableEntry) + "] cycle time "
                    + LenTwo(String(bootTimeTable[nextBootTableEntry][1])) + ":"
                    + LenTwo(String(bootTimeTable[nextBootTableEntry][0]))
                    + ", type 0x" + String(bootTimeTable[nextBootTableEntry][2], HEX), DEBUG_LEVEL_1);
    } else {
      DebugPrintln("No next cycle time ", DEBUG_LEVEL_1);
    }
  )
  return inCycle;
}
/*
 * returns the millisecond time stamp of the next cycle in table,
 * counted from beginning of the day
 */
int  getNextBootMS(){
  if (nextBootTableEntry == NO_TABLE_ENTRY) { return NO_TIME_MS; }
  if (nextBootTableEntry >= MAX_CYCLES_PER_DAY) { nextBootTableEntry = NO_TABLE_ENTRY; return NO_TIME_MS; }
  if (bootTimeTable[nextBootTableEntry][2] == CYCLE_NOT_SET) { return NO_TIME_MS; }
  int result = bootTimeTable[nextBootTableEntry][1] * 3600; //bootHour
  result += bootTimeTable[nextBootTableEntry][0] * 60; //bootHour
  result *= 1000;
  return result;
}

/*
 * returns the millisecond time difference to the next cycle in table,
 * counted from the current boot time
 */
int  timeToNextBootMS(){
  int result;
  if (GPSalert()){
    _DD(DebugPrintln("timeToNextBoot = ALERT Boot time: ", DEBUG_LEVEL_3);)
    result = SLEEP_DUR_ALERT;
  } else {
    result = getNextBootMS();
    if ((result == NO_TIME_MS) || (bootTimeStampMs == NO_TIME_MS)) { return getCycleTimeMS(); }
    result -= bootTimeStampMs;
    if (result < 0) { result += MS_PER_DAY; }
  }
  _DD(DebugPrintln("timeToNextBoot (seconds): " + String(lround( result / 1000)), DEBUG_LEVEL_3);)
  return result;
}
/*
 * returns true if the current cycle is one for data transmission
 */
bool isTransmissionCycle(void){
  if((heidiConfig->bootNumber >= 0) && (heidiConfig->bootNumber < bootTableLength())){
    return((bootTimeTable[heidiConfig->bootNumber][2] & TRANSMISSION_CYCLE) != 0);
  }
  return false;
}
/*
 * returns true if the current cycle is a fix point
 */
bool isFixPointCycle(void){
  if((heidiConfig->bootNumber >= 0) && (heidiConfig->bootNumber < bootTableLength())){
    return((bootTimeTable[heidiConfig->bootNumber][2] & FIX_POINT_CYCLE) != 0);
  }
  return false;
}

/*
 * returns true if a data transmission has to be done
 */
bool doDataTransmission(){
  bool result = isTransmissionCycle();
  /* after power up we need settings... */
  result |= getState(RESET_INITS);
  /* ...but not in case of a very low battery - that may fail and lead us to a endless boot loop ... */
  result &= !getState(POWER_SAVE_2);

  if (result) { setError(E_TRANSMIT_REGULAR); } //!!!!!!!!!!!!!!!

  result |= (GPSalert() && !getError(E_COULD_NOT_FETCH_GPS));
  /* ...but we definitely give a try if we have an alert, regardless power save state
   * On PRE_GPS_ALERT state we want to check for new fence data - that should work, because PRE_GPS_ALERT state could not be reached
   * without GPS data. But in case while stepping through alert states we cannot fetch position one time, it is senseless to send a
   * SMS without transferring current position data. Therefore:  && !getError(E_COULD_NOT_FETCH_GPS)
   */

  if(GPSalert()) {
    setError(E_TRANSMIT_ALERT);  //!!!!!!!!!!!!!!!
    if(getError(E_COULD_NOT_FETCH_GPS)) { setError(E_TRANSMIT_ALERT_NO_GPS); } //!!!!!!!!!!!!!!!
  }
  //_D(DebugPrintln("doDataTransmission: " + String(result), DEBUG_LEVEL_2);)
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

void pause(uint32_t ms){
  uint32_t ticks = pdMS_TO_TICKS(ms) + 0.5;
  if(ticks == 0) { ticks = 1; }
  vTaskDelay(ticks);
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
  bootTimeStampMs = bootMs;
  bootTime->tm_hour = (int)(bootMs / 3600000);
  bootMs -= bootTime->tm_hour * 3600000;
  bootTime->tm_min = (int)(bootMs / 60000);
  bootMs -= bootTime->tm_min * 60000;
  bootTime->tm_sec = bootMs / 1000;
  mktime(bootTime);
}

void initBootTimeMS(void){
  bootTimeStampMs     = INVALID_TIME_VALUE;
  expectedBootTimeMs  = INVALID_TIME_VALUE;
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
  if (heidiConfig->c.nightHourStart == heidiConfig->c.nightHourEnd) { return false; }
  if (heidiConfig->c.nightHourStart < heidiConfig->c.nightHourEnd){
    return ((hour >= heidiConfig->c.nightHourStart) && (hour < heidiConfig->c.nightHourEnd));
  }
  if (heidiConfig->c.nightHourStart > heidiConfig->c.nightHourEnd){
    return ((hour >= heidiConfig->c.nightHourStart) || (hour < heidiConfig->c.nightHourEnd));
  }
  return false;
}

/*
 * returns the amount of boot cycles per day
 */
int16_t bootTableLength(void){
  if(bootTableLenghtVal == NO_VALID_TABLE){ setupCycleTable(); }
  return bootTableLenghtVal;
}
uint8_t _currentCyleLen_m(){ //minutes (max 60)
  if (_night()) { return heidiConfig->c.nightSleepMin; }
  else { return heidiConfig->c.sleepMinutes; }
}
uint8_t  _currentCyleMaxDiff_s(){ //seconds (max 180 = 5% from 3600)
  if (_night()) { return heidiConfig->c.nightSleepMin * 3; } //(*60 / 20 -> *3)
  else { return heidiConfig->c.sleepMinutes * 3; }
}

/*
 * returns maximum difference in seconds, for counting cycle as "in"
 *  = 5% of cycle time = 3 seconds per cycle minute
 */
uint8_t  _cyleMaxDiff_s(uint8_t hour){ //seconds (max 180 = 5% from 3600)
  if (__night(hour)) { return heidiConfig->c.nightSleepMin * 3; } //(*60 / 20 -> *3)
  else { return heidiConfig->c.sleepMinutes * 3; }
}
uint8_t _currentCycles(){
  return __currentCycles(_night());
}

uint8_t __currentCycles(bool isNight){
  uint8_t result = heidiConfig->c.bootCycles;
  if(isNight){ result = heidiConfig->c.nightBootCycles;}
  if (getState(POWER_SAVE_1)) {result *= 2;}
  if (result > MAX_BOOT_CYCLES) { result = MAX_BOOT_CYCLES; }
  return result;
}
_D(
  uint8_t __currentCyclesD(bool isNight){
    uint8_t result = heidiConfig->c.bootCycles;
    if(isNight){
      result = heidiConfig->c.nightBootCycles;
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
/*
 * copies a int32_t to a buffer
 *   - *buffer -> pointer to buffer
 *   - pos     -> offset
 *   - value   -> value to copy
 */
void _copyInt32toBuffer(uint8_t *buffer, int pos, int32_t value){
  buffer[pos]   = (uint8_t) value;
  buffer[pos+1] = (uint8_t) (value >> 8);
  buffer[pos+2] = (uint8_t) (value >> 0x10);
  buffer[pos+3] = (uint8_t) (value >> 0x18);
}
/*
 * copies a uint32_t to a buffer
 *   - *buffer -> pointer to buffer
 *   - pos     -> offset
 *   - value   -> value to copy
 */
void _copyUint32toBuffer(uint8_t *buffer, int pos, uint32_t value){
  buffer[pos]   = (uint8_t) value;
  buffer[pos+1] = (uint8_t) (value >> 8);
  buffer[pos+2] = (uint8_t) (value >> 0x10);
  buffer[pos+3] = (uint8_t) (value >> 0x18);
}
/*
 * copies a int16_t to a buffer
 *   - *buffer -> pointer to buffer
 *   - pos     -> offset
 *   - value   -> value to copy
 */
void _copyInt16toBuffer(uint8_t *buffer, int pos, int16_t value){
  buffer[pos]   = (uint8_t) value;
  buffer[pos+1] = (uint8_t) (value >> 8);
}
/*
 * copies a uint16_t to a buffer
 *   - *buffer -> pointer to buffer
 *   - pos     -> offset
 *   - value   -> value to copy
 */
void _copyUint16toBuffer(uint8_t *buffer, int pos, uint16_t value){
  buffer[pos]   = (uint8_t) value;
  buffer[pos+1] = (uint8_t) (value >> 8);
}
/*
 * copies a buffer to a int32_t
 *   - *buffer -> pointer to buffer
 *   - pos     -> offset
 *   returns int32 value
 */
int32_t _copyBufferToInt32(uint8_t *buffer, int pos){
  int32_t value;
  value  = buffer[pos];
  value |= buffer[pos+1] << 8;
  value |= buffer[pos+2] << 0x10;
  value |= buffer[pos+3] << 0x18;
  return value;
}
/*
 * copies a buffer to a uint32_t
 *   - *buffer -> pointer to buffer
 *   - pos     -> offset
 *   returns uint32 value
 */
uint32_t _copyBufferToUInt32(uint8_t *buffer, int pos){
  uint32_t value;
  value  = buffer[pos];
  value |= buffer[pos+1] << 8;
  value |= buffer[pos+2] << 0x10;
  value |= buffer[pos+3] << 0x18;
  return value;
}
/*
 * copies a buffer to a uint16_t
 *   - *buffer -> pointer to buffer
 *   - pos     -> offset
 *   returns uint16 value
 */
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

/*
 * calculates CRC16 over string data
 *   data - string
 *   returns crc16
 */
uint16_t crc16F(String data){ //CRC-16/CCITT-FALSE
  uint16_t crc = 0xFFFF;
  int len = data.length();
  for (int i = 0; i < len; i++){
    uint32_t x = ((crc >> 8) ^ (uint8_t)data.charAt(i)) & 0xFF;
    x ^= (x >> 4);
    crc = ((crc << 8) ^ (x << 12) ^ (x << 5) ^ x) & 0xFFFF;
  }
  return crc;
}

/*
 * calculates CRC16 over binary data
 *   *data - pointer to buffer containing data
 *    len  - length of buffer
 *    returns crc16
 */
uint16_t crc16D(uint8_t *data, int len){ //CRC-16/CCITT-FALSE
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++){
    uint32_t x = ((crc >> 8) ^ data[i]) & 0xFF;
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

double mkDouble(uint32_t val1, uint32_t val2){
  double y;
  uint32_t *py = (uint32_t*)&y;
  py[0] = val1;
  py[1] = val2;
  return y;
}

float mkFloat(uint32_t val){
  float y;
  uint32_t *py = (uint32_t*)&y;
  *py = val;
  return y;
}
