/*
 * heidi-data.h
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */

#ifndef HEIDI_DATA_H_
#define HEIDI_DATA_H_

#include <stdint.h>
#include <esp_attr.h>
#include "heidi-defines.h"

#define DATA_SET_LEN     32
#define DATA_SET_BACKUPS 25
#define MAX_DATA_SETS (BOOT_CYCLES * (DATA_SET_BACKUPS + 1))
#define DATA_SET_MEM_SPACE (DATA_SET_LEN * MAX_DATA_SETS)
#if (DATA_SET_MEM_SPACE > 3328)
  #error 'Too much RTC space for data sets\n'
#endif

typedef struct _t_SendData{
  int32_t  latitude;
  int32_t  longitude;
  uint16_t altitude;
  uint16_t date; // 0-4 Day of the month / 5-8 Month /  9-15 Year offset from 1980
  uint16_t time; // 0-4 Second divided by 2 / 5-10 Minute / 11-15 Hour
  uint16_t battery; //1/1000 volt
  int16_t  temperature; //1/100 °C
  uint32_t errCode; //
  int8_t   secGPS; //seconds to fetch GPS position
  //uint8_t  id;
  uint8_t  satellites;
  //total size: 24Bytes -> 28 = 7*4;
}t_SendData;

extern t_SendData* availableDataSet[MAX_DATA_SETS];

void     initDataSets(void);
void     initDataSet(t_SendData* DataSet);
bool     emptyDataSet(t_SendData* DataSet);
void     copyDataSet(t_SendData* _from, t_SendData* _to);

String generateSendLine(t_SendData* DataSet);
String generateMultiSendLine(int first, int last, int backups = 0);

void     getRTCDataSpace(uint8_t**);

String   DateString(tm timestamp);
String   TimeString(tm timestamp);
String   LenTwo(const String No);

uint16_t dosDate(const uint8_t year, const uint8_t month, const uint8_t day);
uint16_t dosYear(const uint16_t date);
uint8_t  dosMonth(const uint16_t date);
uint8_t  dosDay(const uint16_t date);
uint16_t dosTime(const uint8_t hour, const uint8_t minute, const uint8_t second);
uint8_t  dosHour(const uint16_t time);
uint8_t  dosMinute(const uint16_t time);
uint8_t  dosSecond(const uint16_t time);





#endif /* HEIDI_DATA_H_ */
