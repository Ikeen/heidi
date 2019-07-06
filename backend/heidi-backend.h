/*
 * heidi-backend.h
 *
 *  Created on: 14.03.2019
 *      Author: frank
 */
#ifndef HEIDI_BACKEND_H_
#define HEIDI_BACKEND_H_
#include <time.h>

#define DEBUG_LEVEL 2 //0 (no prints) .. 3 (all prints)

//#define OLED_DISPLAY
#define SIM_MODULE
#define GPS_MODULE
#define TEMP_SENSOR
#define HEIDI_GATE_WAY

#define DATA_SET_BACKUPS   1
#define BOOT_CYCLES        4
#define MAX_DATA_SETS      ((DATA_SET_BACKUPS + 1) * BOOT_CYCLES)
#define START_FROM_RESET   -2
#define REFETCH_SYS_TIME   -1
#define INVALID_TIME_VALUE -1

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

/** error handling **/

#define COULD_NOT_FETCH_GPS_TIME 0x0001
#define GSM_CONNECTION_FAILED    0x0002
#define COULD_NOT_FETCH_GPS_1    0x0010
#define COULD_NOT_FETCH_GPS_2    0x0020
#define COULD_NOT_FETCH_GPS_3    0x0040
#define COULD_NOT_FETCH_GPS_4    0x0080
#define WRONG_GPS_VALUES_1       0x0100
#define WRONG_GPS_VALUES_2       0x0200
#define WRONG_GPS_VALUES_3       0x0400
#define WRONG_GPS_VALUES_4       0x0800
#define WRONG_BOOT_REASON        0x00010000
#define WRONG_BOOT_REASON_MASK   0xFFFF0000

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


/*
 * Wires
 */
#define LED      2     // GPIO2   -- LED
#define LED_ON   HIGH
#define LED_OFF  LOW

#define GPS      25    // GPIO25  -- GPS
#define GPS_RX   16    //GPIO16
#define GPS_TX   17    //GPIO17
#define GPS_UART_NO 2
#define GPS_ON   LOW
#define GPS_OFF  HIGH

#define GSM      13    // GPIO13  -- GSM
#define GSM_RST  21
#define GSM_RXD  23    //!!!
#define GSM_TXD  4     //!!!
#define GSM_UART_NO 1
#define GSM_ON   LOW
#define GSM_OFF  HIGH

#define BATTERY_ANALOG_PIN     36
#define BATTERY_ANALOG_ENABLE  15

#define TEMP_SENSOR_PIN        22


/*
 * Sleeps
 */
#define uS_TO_S_FACTOR      1000000   /* Conversion factor for micro seconds to seconds */
#define uS_TO_mS_FACTOR     1000      /* Conversion factor for milli seconds to seconds */
#define SLEEP_DURATION_MSEC 900000    /* 15 minutes */
#define CYCLE_DURATION_MIN  (SLEEP_DURATION_MSEC / 60000)
#define SLEEP_MAX_SHIFT_MS  (SLEEP_DURATION_MSEC / 20) /* 5% max RTC shift */
#define SLEEP_MAX_SHIFT_S   (SLEEP_DURATION_MSEC / 20000) /* 5% max RTC shift */
#define WAIT_FOR_GPS_TIME   120000
#define MS_PER_DAY          86400000

#define NO_TEMPERATURE      -127


typedef struct _t_SendData{
  int32_t  latitude;
  int32_t  longitude;
  uint16_t altitude;
  uint16_t date; // 0-4 Day of the month / 5-8 Month /  9-15 Year offset from 1980
  uint16_t time; // 0-4 Second divided by 2 / 5-10 Minute / 11-15 Hour
  uint16_t battery; //1/1000 volt
  int16_t  temperature; //1/100 °C
  uint32_t errCode; //
  int8_t   secdiff; //seconds difference from expected to current time stamp
  uint8_t  id;
  uint8_t  satellites;
}t_SendData;
typedef enum {
  DEBUG_LEVEL_0,
  DEBUG_LEVEL_1,
  DEBUG_LEVEL_2,
  DEBUG_LEVEL_3
};

#ifdef GPS_MODULE
void GPS_on();
void GPS_off();
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

#ifdef SIM_MODULE
void GSM_on();
void GSM_off();
bool GSMsetup();
bool GSMshutDown();
int GSMinitiateHTTP(String url);
int GSMdoPost(String url, String contentType, String payload, unsigned int clientWriteTimeoutMs, unsigned int serverReadTimeoutMs);
int GSMdoGet(const char* url, unsigned int serverReadTimeoutMs);
int GSMterminateHTTP();
String GSMsendCommand(const String command, int timeoutMs = 5000);
bool  GSMsendCommand2(const String command, String response, int timeoutMs = 5000);
#endif

String generateSendLine(t_SendData* DataSet);
String generateMultiSendLine(int first, int last, int backups = 0);
uint16_t herdeID();
uint16_t animalID();
uint16_t measurePin(const uint8_t pin);
uint16_t dosDate(const uint8_t year, const uint8_t month, const uint8_t day);
uint16_t dosTime(const uint8_t hour, const uint8_t minute, const uint8_t second);
uint16_t dosYear(const uint16_t date);
uint8_t  dosMonth(const uint16_t date);
uint8_t  dosDay(const uint16_t date);
uint8_t  dosHour(const uint16_t time);
uint8_t  dosMinute(const uint16_t time);
uint8_t  dosSecond(const uint16_t time);
int8_t   GetLocalTimeHourShift();
String   LenTwo(const String No);
bool isInTime(const int target_m, const int current_m, const int current_s);
void initDataSet(t_SendData* DataSet);
bool emptyDataSet(t_SendData* DataSet);
void copyDataSet(t_SendData* _from, t_SendData* _to);
void initGlobalVar();
bool isInCycle(int firstCycleInHour);
void SetBootTimeFromMs(int timeStampMs);
bool calcCurrentTimeDiff();

void restartCycling();
void goto_sleep(int mseconds);
void checkWakeUpReason();

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
