/*
 * heidi-data.h
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */

#ifndef HEIDI_DATA_H_
#define HEIDI_DATA_H_

#include <stdint.h>
#include <WString.h>
#include <esp_attr.h>
#include <time.h>
#include "heidi-defines.h"

#define HEX_BUFFER_OFFSET 3
#define HEX_BUFFER_VALUES 12 //_t_SendData values + ID
#define HEX_BUFFER_LEN (HEX_BUFFER_OFFSET + 2 + 4 + 4 + (2 * (HEX_BUFFER_VALUES - 3)))

#define DATA_SET_LEN     28
#define DATA_SET_BACKUPS 24
#define MAX_DATA_SETS (BOOT_CYCLES * (DATA_SET_BACKUPS + 1))
#define DATA_SET_MEM_SPACE (DATA_SET_LEN * MAX_DATA_SETS)

#define FENCE_MAX_POS    16
#define FENCE_SET_LEN    (4*2)
#define FENCE_MEM_SPACE  (FENCE_MAX_POS * FENCE_SET_LEN)

#define STATUS_MEM_SPACE  (4+2) //see heidi-data.cpp

#define RTC_DATA_SPACE (DATA_SET_MEM_SPACE + FENCE_MEM_SPACE + STATUS_MEM_SPACE)
#if (RTC_DATA_SPACE > 3328)
  #error 'Too much RTC space for data sets\n'
#endif

typedef struct _t_SendData{
  int32_t  latitude;
  int32_t  longitude;
  int16_t  altitude;
  uint16_t date; // DOS-format: 0-4 Day of the month / 5-8 Month /  9-15 Year offset from 2020
  uint16_t time; // DOS-format: 0-4 Second divided by 2 / 5-10 Minute / 11-15 Hour
  uint16_t battery; //1/1000 volt
  int16_t  temperature; //1/100 °C
  uint16_t errCode; //
  uint8_t  GPShdop; //horizontal position dilution
  uint8_t  satellites;
  uint16_t metersOut;
  //total size: 24 Bytes;
}t_SendData;

typedef struct _t_FenceData{
  int32_t  latitude;
  int32_t  longitude;
}t_FenceData;


extern t_SendData* availableDataSet[MAX_DATA_SETS];
extern t_FenceData* FenceDataSet[FENCE_MAX_POS];
extern int8_t     bootCount;
extern int8_t     lastWrongResetReason;
extern int32_t    lastTimeDiffMs;

void     initRTCData(void);
void     initDataSet(t_SendData* DataSet);
bool     emptyDataSet(t_SendData* DataSet);
void     copyDataSet(t_SendData* _from, t_SendData* _to);
void     cleanUpDataSets(bool TransmissionFailed);

String   generateSendLine(t_SendData* DataSet);
String   generateMultiSendLine(int first, int last, int backups = 0);
String   generateMulti64SendLine(int first, int last, int backups = 0);

void     getRTCDataSpace(uint8_t**);

int32_t  GeoToInt(double geo);
double   IntToGeo(int32_t val);
String   b64Encode(uint8_t* Buffer, int length);
int      b64Decode(String Base64Str, uint8_t* Buffer);

void     testData(void);

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
