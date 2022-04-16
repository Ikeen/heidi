/*
 * heidi-backend.h
 *
 *  Created on: 14.03.2019
 *      Author: frank
 */
#ifndef HEIDI_BACKEND_H_
#define HEIDI_BACKEND_H_
#include <stdint.h>
//#include <time.h>
#include <assert.h>
#include <esp_attr.h>
#include <Wire.h>
#include <esp_deep_sleep.h>
#include <esp_timer.h>
#include <driver/adc.h>
#include "heidi-defines.h"
#include "heidi-data.h"
#include "heidi-debug.h"

uint8_t herdeID();
uint8_t animalID();

void setupData(bool powerOnReset);
bool setupSystemBootTime(tm* boottime, int timeOut);
void setupWatchDog(uint32_t timeOutMS);

void   finalizeDataSet(t_SendData* currentDataSet);
void   finalizeHeidiStatus(bool powerOnReset);

#ifdef PRE_MEASURE_HANDLING
void   handlePreMeasuring(void);
#endif
#ifdef GSM_MODULE
void   transmitData(t_SendData* DatcurrentDataSetaSet);
#endif
void   checkGPSalert(t_SendData* currentDataSet);
void   checkGPSposition(t_SendData* currentDataSet, int timeOut, bool force);
double checkBattery(void);
void   checkCycle(void);
#ifdef ACCELEROMETER
void   checkAccData(t_SendData* currDataSet);
#endif

void restartCycling();
void gotoSleep(int32_t mseconds);
void doSleepRTCon(int32_t ms);

static void watchDog(void* arg);

#ifdef HEIDI_CONFIG_TEST
void doTests(t_SendData*);
#ifdef TEST_RTC
void testRTC(t_SendData*);
#endif
#endif
#endif /* HEIDI_BACKEND_H_ */
