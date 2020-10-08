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


uint8_t herdeID();
uint8_t animalID();

bool isInCycle(int firstCycleInHour, int8_t* bootCount);
bool doDataTransmission(int8_t bootCount );
bool calcCurrentTimeDiff();
bool isInTime(const int target_m, const int current_m, const int current_s);
void SetBootTimeFromMs(int timeStampMs);
int8_t  GetLocalTimeHourShift();
bool GetSysTime(tm *info);
void _copyTime(tm *from, tm *to);
void _copyDate(tm *from, tm *to);
void _copyInt32toBuffer(byte *buffer, int pos, int32_t value);
void _copyint32toBuffer(byte *buffer, int pos, uint32_t value);
void _copyInt16toBuffer(byte *buffer, int pos, int16_t value);
void _copyint16toBuffer(byte *buffer, int pos, uint16_t value);
#endif /* HEIDI_SYS_H_ */
