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
#include <SPI.h>
#include <LoRa.h>
#include <esp_deep_sleep.h>
#include <esp_timer.h>
#include <driver/adc.h>
#include <rom/rtc.h>
#include "heidi-defines.h"
#include "heidi-data.h"
#include "heidi-debug.h"
#include "heidi-gsm.h"
#include "heidi-gps.h"
#include "heidi-fence.h"
#include "heidi-error.h"
#include "heidi-flash.h"
#include "heidi-lora.h"


#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)


/*
 * LoRa Spec
 *
 *Europe:
 *        433,05 MHz - 434,79 MHz (ISM-Band Region 1)
 *        863,00 MHz - 870,00 MHz (SRD-Band Europa)
 *North America:
 *        902,00 MHz - 928,00 MHz (ISM-Band Region 2)
 *
 *LoRa data rate = spreadingFactor * (SignalBandwidth / 2^spreadingFactor) * 4 / codingRateDenominator [bps]
 *               = 1098 bps
 */
#define BAND 865000000.00 //#define BAND    868E6
#define spreadingFactor 10 //7..12 +1 ~ +2.5 dB SNR
#define SignalBandwidth 125E3 // BW/2 ~ +3-4 dB SNR
#define preambleLength  8
#define codingRateDenominator 8 //5;8 -> 4/5; 4/8

#ifdef USE_LORA
void SetupLoRa();
#endif

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
void   transmitData(t_SendData*);
#endif
void   checkGPSalert(t_SendData*);
void   checkGPSposition(t_SendData* currentDataSet, int timeOut, bool force);
double checkBattery(void);
void   checkCycle(void);

void restartCycling();
void gotoSleep(int32_t mseconds);
void doSleepRTCon(int32_t ms);

bool wasPowerOnReset(void);
bool wasbrownOut(void);
bool wasRegularWakeUp(void);
bool wasULPWakeUp(void);

static void watchDog(void* arg);

#ifdef HEIDI_CONFIG_TEST
void doTests(t_SendData*);
#ifdef TEST_RTC
void testRTC(t_SendData*);
#endif
#endif
#endif /* HEIDI_BACKEND_H_ */
