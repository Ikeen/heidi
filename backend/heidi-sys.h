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
#include "heidi-defines.h"
#include "heidi-debug.h"

#define NO_TABLE_ENTRY -1
#define NO_TIME_MS -1

uint8_t herdeID();
String _herdeID();
uint8_t animalID();

bool isInCycle(int8_t* bootCount);
int  getNextBootMS(void);
int  timeToNextBootMS(void);
int  prevBootCycleNo(void);
bool doDataTransmission();
bool GPSalert();
void calcCycleTable();
bool calcCurrentTimeDiff();
bool isInTime(const int target_m, const int current_m, const int current_s);
void setBootTimeFromCurrentTime(tm* sysTime, tm* bootTime);
void   initSysTimeMS(void);
bool   getSysTime(tm *info);
String getCSVvalue(String line, int no);
bool     _night(void);
bool     __night(uint8_t hour);
uint8_t  _currentCyleLen_m();
uint8_t  _currentCyleMaxDiff_s();
uint8_t  _cyleMaxDiff_s(uint8_t hour);
uint8_t  _currentCycles();
uint8_t  __currentCycles(bool isNight);
_D(uint8_t __currentCyclesD(bool isNight);)
void     _copyTime(tm *from, tm *to);
void     _copyDate(tm *from, tm *to);
void     _copyInt32toBuffer(uint8_t *buffer, int pos, int32_t value);
void     _copyInt16toBuffer(uint8_t *buffer, int pos, int16_t value);
void     _copyUint32toBuffer(uint8_t *buffer, int pos, uint32_t value);
void     _copyUint16toBuffer(uint8_t *buffer, int pos, uint16_t value);
int32_t  _copyBufferToInt32(uint8_t *buffer, int pos);
uint16_t _copyBufferToUInt16(uint8_t *buffer, int pos);

String   b64Encode(uint8_t* Buffer, int length);
int      b64Decode(String Base64Str, uint8_t* Buffer);
uint16_t crc16F(String data);
uint32_t hex2int(String data);

#endif /* HEIDI_SYS_H_ */
