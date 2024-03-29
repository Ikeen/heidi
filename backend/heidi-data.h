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
#include <assert.h>
#include "heidi-defines.h"
#ifdef ACCELEROMETER
#include "heidi-acc.h"
#define RTC_DATA_SPACE_OFFSET ACCEL_ULP_MEM_SIZE //size in uint32
#else
#define RTC_DATA_SPACE_OFFSET 0
#endif

#define TEL_NO_LEN 12
#define TEL_NO_CNT 2
#define HEX_BUFFER_OFFSET 3  //1. count, 2. herd id, 3. animal id
#define HEX_BUFFER_DATA_VALUES 15 // = _t_SendData values
#define HEX_BUFFER_LEN (HEX_BUFFER_OFFSET + 4 + 4 + (2 * (HEX_BUFFER_DATA_VALUES - 2)) + 2)
                                        //lat, lng,   all other data as uint16,      checksum (not used)

typedef __attribute__((__packed__)) struct _t_ConfigDataC{
  uint8_t bootCycles;      //cycles between data transmission
  uint8_t sleepMinutes;    //cycle duration - should match 180 minute big cycle
  uint8_t nightHourStart;  //start night modus - GMT!
  uint8_t nightHourEnd;    //end night modus - GMT!
  uint8_t nightBootCycles; //cycles between data transmission night modus
  uint8_t nightSleepMin;   //cycle duration night modus
 uint16_t distAlertThres;  //distance for geo-fencing alert in meters
 uint16_t accThres1;       //threshold for counting acceleration index 1
 uint16_t accThres2;       //threshold for counting acceleration index 2
 uint16_t accAlertThres1;  //alert threshold for acceleration index 1 counter
 uint16_t accAlertThres2;  //alert threshold for acceleration index 2 counter
  uint8_t accNightFactor;  //night modus accThres percent of day modus (100 -> night = day)
}t_ConfigDataC;
#define CONF_DATA_C_PAYLOAD_SIZE 17 //consider: there are many functions using this number!

typedef __attribute__((__packed__)) struct _t_ConfigData{
 t_ConfigDataC c;
 uint32_t status;          //heidi status
  int16_t bootNumber;      //current boot-table position
  uint8_t gpsStatus;
  int32_t lastTimeDiffMs;
#ifdef HEIDI_GATEWAY
  uint8_t telNo[TEL_NO_CNT][TEL_NO_LEN]; // BCD coded (A='+'; B=empty; C-F=not allowed)
  uint8_t alertFailCount;
  uint8_t lastClientTrig;  //last client asked for data
 uint32_t clients;         //bit-field - marks present clients
 uint32_t clientsNeedConf; //bit-field - marks clients not sending the right config CRC
#endif
}t_ConfigData;

#ifdef HEIDI_GATEWAY
#define HEIDI_CONFIG_LENGTH (44+(TEL_NO_LEN*TEL_NO_CNT))   //must be a multiple of 4
#else
#define HEIDI_CONFIG_LENGTH 32   //must be a multiple of 4
#endif
static_assert(sizeof(t_ConfigData) <= HEIDI_CONFIG_LENGTH, "HEIDI_CONFIG_LENGTH too small");
#define HEIDI_CONFIG_LENGTH_RTC (HEIDI_CONFIG_LENGTH >> 2) //counted as uint32

typedef __attribute__((__packed__)) struct {
  int32_t  latitude;
  int32_t  longitude;
  int16_t  altitude;
  uint16_t date; // DOS-format: 0-4 Day of the month / 5-8 Month /  9-15 Year offset from 2020
  uint16_t time; // DOS-format: 0-4 Second divided by 2 / 5-10 Minute / 11-15 Hour
  uint16_t battery; //1/1000 volt
  int8_t   temperature; //°C
  uint8_t  animalID;
  uint16_t errCode; //
  uint8_t  GPSpDOP; //horizontal position dilution
  uint8_t  satellites;
  uint16_t metersOut;
  uint16_t accThresCnt1;
  uint16_t accThresCnt2;
  uint16_t accThCnt1Spr; //spreading of counter 1 values
  uint16_t accThCnt2Spr; //spreading of counter 2 values
  //total size: 32 Bytes;
}t_SendData;

#define DATA_SET_LEN     32 //must be a multiple of 4
static_assert(sizeof(t_SendData) == DATA_SET_LEN, "wrong DATA_SET_LEN");

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

typedef enum _dtAcc_t {
  DATA_NO_ACCESS,
  DATA_PRIMARY_ACCESS,
  DATA_ACCESS
}dtAcc_t;

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

#define DATA_SET_NOT_FOUND  -1

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
void     initDataSets(t_SendData** sets, int first, int last);
bool     isEmptyDataSet(t_SendData* DataSet);
void     copyDataSet(t_SendData* _from, t_SendData* _to);
int      findDataSet(t_SendData* _which);
int      findFreeDataSet(int from);
bool     addDataSet(t_SendData* _new);
int      addDataSets(t_SendData* buffer, int cnt);
bool     eraseDataSet(t_SendData* _which);
int      eraseDataSets(t_SendData* buffer, int cnt);
int      packUpDataSets(void);
int      freeDataSets(void);
int      getNextnDataSets(int n, t_SendData* buffer, int size, t_SendData* lastSet = NULL);
bool     freeDataSet(int _which);
void     setErrorToDataSets(t_SendData* sets, int cnt, uint16_t code);

bool     getPrimaryDataSetAccess(void);
dtAcc_t  getDataSetAccess(void);
void     freeDataSetAccess(void);

bool     getState(uint32_t which);
void     setState(uint32_t which);
void     clrState(uint32_t which);

String   generateMulti64SendLine(t_SendData* sets, int cnt);

bool     setSettingsFromHTTPresponse(String response);
bool     newSettingsB64(String b64, t_ConfigDataC* dataBuffer);
bool     setSettingsFromBuffer(uint8_t* buffer, int size,t_ConfigDataC* dataBuffer);
bool     setSettingsFromBuffer(uint8_t* buffer, int size,t_ConfigDataC* dataBuffer);
bool     pushSettingsToBuffer(uint8_t* buffer, int size,t_ConfigDataC* dataBuffer);
void     checkSettings(t_ConfigDataC* dataBuffer);
uint16_t clientConfigCRC(t_ConfigDataC* data);

#ifdef HEIDI_GATEWAY
bool     newTelNoB64(String b64);
bool     setTelNoFromHTTPresponse(String response);
String   getTelNo(int which);
#endif

void     setNewSetting8(uint8_t *buffer, int pufferPos, int8_t *setting, uint32_t status);
void     setNewSettingU8(uint8_t *buffer, int pufferPos, uint8_t *setting, uint32_t status);
void     setNewSettingU16(uint8_t *buffer, int pufferPos, uint16_t *setting, uint32_t status);

void     getRTCDataSpace(uint8_t**);
int32_t  getCycleTimeMS(void);

int32_t  GeoToInt(double geo);
double   IntToGeo(int32_t val);
String   b64Encode(uint8_t* Buffer, int length);
int      b64Decode(String Base64Str, uint8_t* Buffer);
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

#ifdef HEIDI_CONFIG_TEST
#define TEST_DATA_COUNT 96
#ifdef TEST_DATA
void testData(void);
#endif
void loadTestData(void);
t_SendData* getTestData(int _which);

//void PrintRTCFastMemBufferBoundaries(void);
#ifdef TEST_RTC
void testRTC(t_SendData* currentDataSet, tm* bootTime);
void fillRTCbounary();
void testRTCbounary();
#endif
#endif //HEIDI_CONFIG_TEST



#endif /* HEIDI_DATA_H_ */
