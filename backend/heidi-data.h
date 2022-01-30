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
#ifdef ACCELEROMETER
#include "heidi-acc.h"
#define RTC_DATA_SPACE_OFFSET ACCEL_ULP_MEM_SIZE //size in uint32
#else
#define RTC_DATA_SPACE_OFFSET 0
#endif

#define TEL_NO_LEN 12
#define TEL_NO_CNT 2
#define HEX_BUFFER_OFFSET 3
#define HEX_BUFFER_VALUES 16 //_t_SendData values + ID
#define HEX_BUFFER_LEN (HEX_BUFFER_OFFSET + 2 + 4 + 4 + (2 * (HEX_BUFFER_VALUES - 3)))

typedef __attribute__((__packed__)) struct _t_ConfigData{
   int8_t bootCount;
   int8_t bootCycles; //...cycles between data transmission
  uint8_t sleepMinutes;
  uint8_t nightHourStart;
  uint8_t nightHourEnd;
  uint8_t nightBootCycles;
  uint8_t nightSleepMin;
  uint8_t accNightFactor; //percent
 uint16_t accThres1;
 uint16_t accThres2;
 uint16_t accAlertThres1;
 uint16_t accAlertThres2;
 uint16_t distAlertThres;
 uint32_t status;
  int32_t lastTimeDiffMs;
  uint8_t alertFailCount;
  uint8_t gpsStatus;
  uint8_t telNo[TEL_NO_CNT][TEL_NO_LEN]; // BCD coded (A='+'; B=empty; C-F=not allowed)
}t_ConfigData;

#define HEIDI_CONFIG_LENGTH (22+(TEL_NO_LEN*TEL_NO_CNT)) //must be a multiple of 4
#define HEIDI_CONFIG_LENGTH_RTC (HEIDI_CONFIG_LENGTH >> 2)

typedef __attribute__((__packed__)) struct {
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
  uint16_t accThresCnt1;
  uint16_t accThresCnt2;
  uint16_t accThCnt1Spr; //spreading of counter 1 values
  uint16_t accThCnt2Spr; //spreading of counter 2 values
  //total size: 32 Bytes;
}t_SendData;

#ifdef SAVE_AOP_DATA
  typedef __attribute__((__packed__)) struct _t_aopData{
    uint8_t svId;
    uint8_t data[59];
  }t_aopData;
  #define AOP_DATA_SET_LEN 60
  #define AOP_DATA_SETS    32
  #define AOP_DATA_LEN (AOP_DATA_SET_LEN * AOP_DATA_SETS)
#else
  #define AOP_DATA_LEN 0
#endif

#define DATA_SET_LEN     32 //must be a multiple of 4

#ifdef USE_RTC_FAST_MEM
#define FAST_MEM_DATA_SETS    252
#else
#define FAST_MEM_DATA_SETS 0
#endif

#ifdef SAVE_AOP_DATA
#define SLOW_MEM_DATA_SETS    20
#else
#define SLOW_MEM_DATA_SETS    71//80
#endif

#define DATA_SET_FAST_MEM_SPACE (DATA_SET_LEN * FAST_MEM_DATA_SETS)
#define DATA_SET_SLOW_MEM_SPACE (DATA_SET_LEN * SLOW_MEM_DATA_SETS)
#define MAX_DATA_SETS    (FAST_MEM_DATA_SETS + SLOW_MEM_DATA_SETS)

typedef __attribute__((__packed__)) struct _t_FenceData{
  int32_t  latitude;
  int32_t  longitude;
}t_FenceData;

#define FENCE_MAX_POS    16
#define FENCE_SET_LEN    (4*2)
#define FENCE_MEM_SPACE  (FENCE_MAX_POS * FENCE_SET_LEN) //must be a multiple of 4
#define FENCE_MEM_SPACE_RTC (FENCE_MEM_SPACE >> 2)

#ifdef USE_RTC_FAST_MEM
#define RTC_FAST_MEM_SIZE_32 2016
extern uint32_t *fastMemBuffer;
#define RTC_MAX_FAST_DATA_SPACE (RTC_FAST_MEM_SIZE_32 << 2)
#define RTC_FAST_DATA_SPACE DATA_SET_FAST_MEM_SPACE

#if (RTC_FAST_DATA_SPACE > RTC_MAX_FAST_DATA_SPACE)
   #error 'Too much data for RTC fast mem\n'
#endif
#endif

#define RTC_SLOW_DATA_SPACE ((RTC_DATA_SPACE_OFFSET << 2) + HEIDI_CONFIG_LENGTH + FENCE_MEM_SPACE + AOP_DATA_LEN + DATA_SET_SLOW_MEM_SPACE )
#define RTC_SLOW_MAX_DATA_SPACE 3840
#if (RTC_SLOW_DATA_SPACE > RTC_SLOW_MAX_DATA_SPACE)
  #error 'Too much data for RTC slow mem\n'
#endif

#if (MAX_DATA_SETS < 96)
#pragma warning "low data set memory - you may enable RTC fast memory"
#endif

extern t_ConfigData* heidiConfig;
extern t_SendData*   availableDataSet[MAX_DATA_SETS];
extern t_FenceData*  FenceDataSet[FENCE_MAX_POS];
#ifdef SAVE_AOP_DATA
extern t_aopData*    aopDataSet[AOP_DATA_SETS];
#endif
extern int allDataSets;
extern bool newCycleSettings;
extern bool newAccSettings;

void     initConfig(bool reset);
void     initRTCData(bool reset);
void     initDataSet(t_SendData* DataSet);
void     initDataSets(t_SendData* first, t_SendData* last);
bool     emptyDataSet(t_SendData* DataSet);
void     copyDataSet(t_SendData* _from, t_SendData* _to);
int      packUpDataSets(void);
void     freeFirstDataSet(void);
String   getTelNo(int which);

bool     getState(uint32_t which);
void     setState(uint32_t which);
void     clrState(uint32_t which);

String   generateMulti64SendLine(t_SendData** sets, int first, int last);

bool     setSettingsFromHTTPresponse(String response);
bool     setTelNoFromHTTPresponse(String response);
bool     newSettingsB64(String b64);
bool     newTelNoB64(String b64);

void     setNewSetting8(uint8_t *buffer, int pufferPos, int8_t *setting, uint32_t status);
void     setNewSettingU8(uint8_t *buffer, int pufferPos, uint8_t *setting, uint32_t status);
void     setNewSettingU16(uint8_t *buffer, int pufferPos, uint16_t *setting, uint32_t status);

void     getRTCDataSpace(uint8_t**);
int32_t  getCycleTimeMS(void);

int32_t  GeoToInt(double geo);
double   IntToGeo(int32_t val);
String   b64Encode(uint8_t* Buffer, int length);
int      b64Decode(String Base64Str, uint8_t* Buffer);

void     testData(void);

String   DateString(tm* timestamp);
String   TimeString(tm* timestamp);
String   DOSdateString(uint16_t _dosDate);
String   DOStimeString(uint16_t _dosTime);
String   LenTwo(const String No);

uint16_t dosDate(const uint8_t year, const uint8_t month, const uint8_t day);
uint16_t dosYear(const uint16_t date);
uint8_t  dosMonth(const uint16_t date);
uint8_t  dosDay(const uint16_t date);
uint16_t dosTime(const uint8_t hour, const uint8_t minute, const uint8_t second);
uint8_t  dosHour(const uint16_t time);
uint8_t  dosMinute(const uint16_t time);
uint8_t  dosSecond(const uint16_t time);

#ifdef USE_RTC_FAST_MEM
bool RTCfastMemRead(void);
bool RTCfastMemWrite(void);
void fastMemReadTask(void *pvParameters);
void fastMemWriteTask(void *pvParameters);
#endif

//void PrintRTCFastMemBufferBoundaries(void);
#ifdef TEST_RTC
void testRTC(t_SendData* currentDataSet, tm* bootTime);
void fillRTCbounary();
void testRTCbounary();
#endif




#endif /* HEIDI_DATA_H_ */
