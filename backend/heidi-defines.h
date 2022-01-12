/*
 * heidi-defines.h
 * global defines
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */
#include "heidi-secrets.h"

#ifndef HEIDI_DEFINES_H_
#define HEIDI_DEFINES_H_

/*
 * hardware configuration
 *
 * the following options are enabled by defining the corresponding item, leaving the define out disables the option
 *
 * GSM_MODULE            - there is a GSM module, e.g SIM800L
 * GPS_MODULE            - there is a GPS module, e.g ublox NEO-7M
 * COMMON_SERIAL         - GSM module and GPS module are sharing one serial interface
 * TEMP_SENSOR           - there is a temperature sensor, e.g. dallas DS 18B20
 * CHECK_BATTERY         - check battery status (disable this only for test configurations)
 * USE_VOLTAGE_MEAS_PIN  - use a dedicated pin to activate battery voltage measuring.
 *                         In the current version we double use the SIM800L reset wire for that, because SIM800L is
 *                         usually powered off and reset pin is HIGHZ at the start. For voltage measuring it is pulled
 *                         to ground. Not using this option means power up all sensors for just checking battery
 *                         status. In case of low battery this is semi-best.
 * I2C_BUS               - enable i²c bus
 * ACCELEROMETER         - there is a accelerometer module, e.g. ADXL345
 * USE_ULP               - ultra low power processor of ESP32 is used for monitoring
 * USE_GPS_ALERT         - enable alerting modes for geo-fencing
 * USE_ACC_ALERT         - enable alerting modes for activity-monitoring (currently not implemented)
 * SEND_ALERT_SMS        - enable sending of alerting SMS
 * PRE_MEASURE_HANDLING  - power up all sensors some time before the measurements are scheduled
 *                         This is used for saving power, because e.g. the GPS module needs some seconds to get valid
 *                         results. The ESP32 may sleep meanwhile.
 * USE_RTC_FAST_MEM      - additionally use RTC fast memory to store data sets
 * SAVE_AOP_DATA         - store GPS-AOP data on ESP32 RTC memory. (AOP - automatic orbit prediction - see ublox manual)
 *                         AOP is enabled for Heidi because it speeds up position fixes significantly. There are 32
 *                         satellites in orbit for whom NEO-7M calculates orbits, but it can only store 20 AOP data sets.
 *                         Storing all data sets and uploading them on start should help to further speed up, but in fact
 *                         it seems it doesn't. Furthermore it seems NEO-7M is able to store more than 20 data sets.
 * DEFAULT_BOOT_CYCLES   - boot cycles until transferring data (if there is no setup stored in flash)
 * DEFAULT_CYCLE_DURATION- in minutes (if there is no setup stored in flash)
 * MAX_AWAKE_TIME_POWER  - extended measuring time after power up or in case of bad GPS-conditions (minutes)
 *                         On powering up from scratch, GPS-module has no orbit-data (ephemeris-data). We need to give
 *                         up-time for downloading it.
 * MAX_AWAKE_TIME_TIMER  - usual maximum up-time (minutes)
 *
 */

#define HEIDI_CONFIG_2

#ifdef HEIDI_CONFIG_1
#define GSM_MODULE
#define GPS_MODULE
#define TEMP_SENSOR
#define CHECK_BATTERY
#define USE_GPS_ALERT
#define SEND_ALERT_SMS
//#define PRE_MEASURE_HANDLING
#define DEFAULT_BOOT_CYCLES         4         // ..until transferring data
#define DEFAULT_CYCLE_DURATION      15        // 15 minutes
#define MAX_AWAKE_TIME_POWER        5         //  5 minutes
#define MAX_AWAKE_TIME_TIMER        2         //  2 minutes
#endif
#ifdef HEIDI_CONFIG_2
#define GSM_MODULE
#define GPS_MODULE
#define COMMON_SERIAL
#define TEMP_SENSOR
#define CHECK_BATTERY
#define USE_VOLTAGE_MEAS_PIN
#define I2C_BUS
#define ACCELEROMETER
//#define I2C_SWITCH
#define USE_ULP
#define USE_GPS_ALERT
//#define SEND_ALERT_SMS
//#define USE_ACC_ALERT
//#define PRE_MEASURE_HANDLING
#define USE_RTC_FAST_MEM
//#define SAVE_AOP_DATA
#define DEFAULT_BOOT_CYCLES         4         // ..until transferring data
#define DEFAULT_CYCLE_DURATION      15        // 15 minutes
#define MAX_AWAKE_TIME_POWER        5         //  5 minutes
#define MAX_AWAKE_TIME_TIMER        2         //  2 minutes
#endif
#ifdef HEIDI_CONFIG_TEST
//#define GSM_MODULE
#define GPS_MODULE
#define COMMON_SERIAL
//#define TEMP_SENSOR
//#define CHECK_BATTERY
#define USE_VOLTAGE_MEAS_PIN
#define I2C_BUS
#define ACCELEROMETER
//#define I2C_SWITCH
#define USE_ULP
//#define USE_GPS_ALERT
//#define SEND_ALERT_SMS
//#define USE_ACC_ALERT
//#define PRE_MEASURE_HANDLING
#define USE_RTC_FAST_MEM
#define SAVE_AOP_DATA
#define DEFAULT_BOOT_CYCLES         32        // ..until transferring data
#define DEFAULT_CYCLE_DURATION      5         // 5 minutes
#define MAX_AWAKE_TIME_POWER        0         // infinite
#define MAX_AWAKE_TIME_TIMER        0         // infinite
#undef DEBUG_LEVEL
#define DEBUG_LEVEL 3
#endif

/*
 * software configuration
 */
#ifdef HEIDI_CONFIG_1
#define HEIDI_GATE_WAY
#define HEIDI_HERDE         2
#define HEIDI_ANIMAL        1
#endif
#ifdef HEIDI_CONFIG_2
#define HEIDI_GATE_WAY
#define HEIDI_HERDE         1
#define HEIDI_ANIMAL        1
#endif
#ifdef HEIDI_CONFIG_TEST
#define HEIDI_GATE_WAY
#define HEIDI_HERDE         0
#define HEIDI_ANIMAL        1
#endif

#ifdef HEIDI_CONFIG_TEST
#define MIN_CYCLE_DURATION_MSEC     60000     //  1 minutes
#define MIN_CYCLE_DURATION          1         //  1 minutes
#else
#define MIN_CYCLE_DURATION_MSEC     300000    //  5 minutes
#define MIN_CYCLE_DURATION          5         //  5 minutes
#endif
#define MAX_CYCLE_DURATION_MSEC     3600000   //  1 hour
#define MAX_CYCLE_DURATION          60        //  1 hour
#define MAX_CYCLES_PER_DAY          288       //  5 minutes each
#define PRE_CYCLE_TIME              30000     //  wake GPS 30 seconds before measuring (PRE_MEASURE_HANDLINg)
#define MAX_BOOT_CYCLES             (MAX_DATA_SETS >> 1) // ..until transferring data = half count of data sets
#define MAX_MEAS_TIME               5000      //  5 seconds
#define MAX_FAILED_ALERTS           10        //  abort alerting after 10 fails


#define START_FROM_RESET   -2
#define REFETCH_SYS_TIME   -1
#define INVALID_TIME_VALUE -1

#define S_TO_mS_FACTOR      1000      /* Conversion factor for seconds to milli seconds */
#define uS_TO_S_FACTOR      1000000   /* Conversion factor for micro seconds to seconds */
#define uS_TO_mS_FACTOR     1000      /* Conversion factor for milli seconds to seconds */
#define SLEEP_DUR_NOTIME    300000    /* 5 minutes if systime could not be set*/
#define SLEEP_DUR_ALERT     300000    /* 5 minutes */

#define MIN_SLEEP_TIME_MS   1000      /* 1 second */
#define MAX_SLEEP_TIME_MS   3600000   /* 1 hour */
#define ONE_MINUTE          60000
#define MS_PER_DAY          86400000

#define MAX_AWAKE_TIME_POWER_MSEC (MAX_AWAKE_TIME_POWER * ONE_MINUTE)
#define MAX_AWAKE_TIME_TIMER_MSEC (MAX_AWAKE_TIME_TIMER * ONE_MINUTE)
#define DEFAULT_CYCLE_DURATION_MSEC (DEFAULT_CYCLE_DURATION * ONE_MINUTE)
#if (MAX_AWAKE_TIME_POWER_MSEC == 0) || (MAX_AWAKE_TIME_TIMER_MSEC == 0)
#pragma warning "timeout disabled!"
#endif

#ifdef HEIDI_CONFIG_TEST
#define DEFALUT_ACCELERATION_THRESHOLD_1    50
#define DEFALUT_ACCELERATION_THRESHOLD_2   150
#define DEFALUT_ACCEL_THRES_MAX_COUNT    10000 /* never ever do an alert */
#else
#define DEFALUT_ACCELERATION_THRESHOLD_1  1000
#define DEFALUT_ACCELERATION_THRESHOLD_2  3000
#define DEFALUT_ACCEL_THRES_MAX_COUNT    10000 /* never ever do an alert */
#endif

#define DEFALUT_NIGHT_HOUR_START_UTC    18
#define DEFALUT_NIGHT_HOUR_END_UTC      5

#define GSM_POWER_SAVE_1_VOLTAGE    3.6   //double transmission time
#define GSM_POWER_SAVE_2_VOLTAGE    3.5   //no more transmissions
#define GSM_POWER_SAVE_3_VOLTAGE    3.4   //do nothing just deep sleep
#define GSM_POWER_SAVE_4_VOLTAGE    3.3   //do nothing deep sleep for 1 hour

#define PRE_MEAS_STATE      0x00000001
#define PRE_GPS_ALERT       0x00000002
#define GPS_ALERT_1         0x00000004
#define GPS_ALERT_2         0x00000008
#define GPS_ALERT_PSD       0x00000010 //alert passed
#define PRE_ACC_ALERT       0x00000020
#define ACC_ALERT_1         0x00000040
#define ACC_ALERT_2         0x00000080
#define ACC_ALERT_PSD       0x00000100
#define POWER_SAVE_1        0x00000200
#define POWER_SAVE_2        0x00000400
#define NOT_IN_CYCLE        0x00000800
#define NEW_FENCE           0x00001000
#define NEW_SETTINGS        0x00002000
#define NEW_ACC_DATA        0x00004000
#define RESET_INITS         0x00008000
#define TRSMT_DATA_ERROR    0x00010000
#define ULP_RUNNING         0x00020000
#define FROM_PWR_SAVE_SLEEP 0x00040000

#ifdef HEIDI_CONFIG_1
#ifdef HEIDI_CONFIG_2
#error "HEIDI_2 and HEIDI_1 enabled"
#endif
#endif
#ifdef HEIDI_CONFIG_1
#ifdef HEIDI_CONFIG_TEST
#error "HEIDI_TEST and HEIDI_1 enabled"
#endif
#endif
#ifdef HEIDI_CONFIG_2
#ifdef HEIDI_CONFIG_TEST
#error "HEIDI_TEST and HEIDI_2 enabled"
#endif
#endif


#ifndef HEIDI_CONFIG_1
#ifndef HEIDI_CONFIG_2
#ifndef HEIDI_CONFIG_TEST
#error "no hardware configuration set"
#endif
#endif
#endif

#ifndef USE_RTC_FAST_MEM
#ifndef USE_RTC_SLOW_MEM
#define USE_RTC_SLOW_MEM
#endif
#endif

#ifdef  SAVE_AOP_DATA
#ifndef USE_RTC_FAST_MEM
#error "AOP data resides in RTC slow memory, which is currently used for tracker data too."
#error "30 datasets might be too less. If you're sure, uncomment this."
#endif
#endif

#ifdef ACCELEROMETER
#ifndef I2C_BUS
#error "I2C bus for accelerometer not enabled"
#endif
#ifndef USE_ULP
#error "ULP for accelerometer not enabled"
#endif
#endif

#ifdef USE_ULP
#ifndef ACCELEROMETER
#error "No accelerometer for ULP measurement"
#endif
#endif


#ifdef I2C_SWITCH
#ifndef I2C_BUS
#error "I2C bus for switch not enabled"
#endif
#endif

#ifdef TEST_ACC
#ifndef ACCELEROMETER
#error "accelerometer not enabled"
#endif
#endif

#ifdef USE_GPS_ALERT
#ifndef GSM_MODULE
#error "No GSM module -> no GPS alert"
#endif
#endif

#ifdef USE_GPS_ALERT
#ifndef GPS_MODULE
#error "No GPS module -> no GPS alert"
#endif
#endif

#ifdef USE_ACC_ALERT
#ifndef GSM_MODULE
#error "No GSM module -> no acceleration alert"
#endif
#endif

#ifdef USE_ACC_ALERT
#ifndef USE_ULP
#error "No ULP measurement -> no acceleration alert"
#endif
#endif

#if (MAX_AWAKE_TIME_POWER_MSEC == 0) || (MAX_AWAKE_TIME_TIMER_MSEC == 0)
#ifndef HEIDI_CONFIG_TEST
#error "Timeout disabled!"
#endif
#endif

extern bool GPSenabled;
extern bool GSMenabled;
extern bool MEASenabled;
extern bool CTLenabled;


#endif /* HEIDI_DEFINES_H_ */
