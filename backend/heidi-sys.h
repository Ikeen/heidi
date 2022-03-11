/*
 * heidi-sys.h
 *
 *  Created on: 08.07.2020
 *      Author: frank
 */

#ifndef HEIDI_SYS_H_
#define HEIDI_SYS_H_
#include <stdint.h>
#include <time.h>
#include <rom/rtc.h>
#include <esp_sleep.h>
#include "heidi-defines.h"
#include "heidi-debug.h"

#define NO_TABLE_ENTRY -1
#define NO_VALID_TABLE -1
#define NO_TIME_MS -1

#define CYCLE_NOT_SET       0x00
#define MEASURING_CYCLE     0x01
#define TRANSMISSION_CYCLE  0x02
#define FIX_POINT_CYCLE     0x04

uint8_t herdeID();
String _herdeID();
uint8_t animalID();
String _animalID();

uint8_t cycleShift(void);

void     setupCycleTable(void);
int16_t  bootTableLength(void);

bool wasPowerOnReset(void);
bool wasReset(void);
bool sysWasbrownOut(void);
bool wasRegularWakeUp(void);
bool wasULPWakeUp(void);
esp_sleep_wakeup_cause_t getWakeUpReason(void);

bool isInCycle(void);
int  getNextBootMS(void);
int  timeToNextBootMS(void);
int  prevBootCycleNo(void);
bool doDataTransmission(void);
bool isTransmissionCycle(void);
bool isFixPointCycle(void);
bool GPSalert(void);
bool calcCurrentTimeDiff(void);
bool isInTime(const int target_m, const int current_m, const int current_s);
void setBootTimeFromCurrentTime(tm* sysTime, tm* bootTime);
void initBootTimeMS(void);
void pause(uint32_t ms);

String getCSVvalue(String line, int no);

bool     _night(void);
bool     __night(uint8_t hour);
uint8_t  _currentCyleLen_m();
uint8_t  _currentCyleMaxDiff_s();
uint8_t  _cyleMaxDiff_s(uint8_t hour);
uint8_t  _currentCycles();
uint8_t  __currentCycles(bool isNight);
_D(
uint8_t  __currentCyclesD(bool isNight);)
void     _copyTime(tm *from, tm *to);
void     _copyDate(tm *from, tm *to);
void     _copyInt32toBuffer(uint8_t *buffer, int pos, int32_t value);
void     _copyInt16toBuffer(uint8_t *buffer, int pos, int16_t value);
void     _copyUint32toBuffer(uint8_t *buffer, int pos, uint32_t value);
void     _copyUint16toBuffer(uint8_t *buffer, int pos, uint16_t value);
int32_t  _copyBufferToInt32(uint8_t *buffer, int pos);
uint32_t _copyBufferToUInt32(uint8_t *buffer, int pos);
uint16_t _copyBufferToUInt16(uint8_t *buffer, int pos);

String   b64Encode(uint8_t* Buffer, int length);
int      b64Decode(String Base64Str, uint8_t* Buffer);
uint16_t crc16F(String data);
uint16_t crc16D(uint8_t *data, int len);
uint32_t hex2int(String data);
String   hexString2(uint8_t val);
String   hexString8(uint16_t val);
String   intString4(uint16_t val);

double mkDouble(uint32_t val1, uint32_t val2);
float mkFloat(uint32_t val);

#endif /* HEIDI_SYS_H_ */
