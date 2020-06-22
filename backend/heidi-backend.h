/*
 * heidi-backend.h
 *
 *  Created on: 14.03.2019
 *      Author: frank
 */
#ifndef HEIDI_BACKEND_H_
#define HEIDI_BACKEND_H_
#include <stdint.h>
#include <time.h>
#include <assert.h>
#include <esp_attr.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <esp_deep_sleep.h>
#include <esp_timer.h>
#include <HardwareSerial.h>
#include <driver/adc.h>
#include <rom/rtc.h>
#include "heidi-defines.h"
#include "heidi-data.h"
#include "heidi-debug.h"
#include "heidi-gsm.h"
#include "heidi-error.h"


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

#define LED      2     // GPIO2   -- LED
#define LED_ON   HIGH
#define LED_OFF  LOW

/*
 * GPS
 */
#define GPS      25    // GPIO25  -- GPS (all measures)
#define GPS_RX   16    //GPIO16
#define GPS_TX   17    //GPIO17
#define GPS_UART_NO 2
#define GPS_ON   LOW
#define GPS_OFF  HIGH
#define WAIT_FOR_GPS_TIME   120000

/*
 * Voltage measuring
 */
#define BATTERY_ANALOG_PIN     36
//#define BATTERY_ANALOG_ENABLE  15
#define ANALOG_MEASURE_OFFSET  166
#define ANALOG_MEASURE_DIVIDER 605

/*
 * Temperature measuring
 */
#define TEMP_SENSOR_PIN        22
#define NO_TEMPERATURE      -127

void Measures_On();
void Measures_Off();
#ifdef GPS_MODULE
int  GPSGetPosition(t_SendData* DataSet, int averages, int timeoutms);
bool SetSysToGPSTime();
bool SetSysToGPS();
#endif

#ifdef OLED_DISPLAY
void displayLocationData(int cnt, double latt, double lngg, int volt);
#endif

#ifdef USE_LORA
void SetupLoRa(){
#endif
#ifdef OLED_DISPLAY
  void SetupDisplay();
#endif

uint16_t herdeID();
uint16_t animalID();
uint16_t measurePin(const uint8_t pin);
bool     GetSysTime(tm* info);
int8_t   GetLocalTimeHourShift();
double   GetVoltage();

bool isInTime(const int target_m, const int current_m, const int current_s);
void initGlobalVar();
bool isInCycle(int firstCycleInHour);
void SetBootTimeFromMs(int timeStampMs);
bool calcCurrentTimeDiff();

void restartCycling();
void goto_sleep(int mseconds);
void checkWakeUpReason();

static void watchDog(void* arg);
void setupWatchDog(void);


void DebugPrint(String text, int level);
void DebugPrintln(String text, int level);
void DebugPrint(double number, int digits, int level);
void DebugPrintln(double number, int digits, int level);
void DebugPrint(int number, int level);
void DebugPrintln(int number, int level);
void DebugPrint(unsigned int number, int level);
void DebugPrintln(unsigned int number, int level);
void testMeasure();

#endif /* HEIDI_BACKEND_H_ */
