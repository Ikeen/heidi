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
#define DEFAULT_CYCLE_DURATION_MSEC 900000    // 15 minutes
#define DEFAULT_CYCLE_DURATION      15        // 15 minutes
#define MAX_AWAKE_TIME_POWER        300000    //  5 minutes
#define MAX_AWAKE_TIME_TIMER        120000    //  2 minutes
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
#define SEND_ALERT_SMS
//#define USE_ACC_ALERT
//#define PRE_MEASURE_HANDLING
#define USE_RTC_FAST_MEM
#define DEFAULT_BOOT_CYCLES         4         // ..until transferring data
#define DEFAULT_CYCLE_DURATION_MSEC 900000    // 15 minutes
#define DEFAULT_CYCLE_DURATION      15        // 15 minutes
#define MAX_AWAKE_TIME_POWER        300000    //  5 minutes
#define MAX_AWAKE_TIME_TIMER        120000    //  2 minutes
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
#define DEFAULT_BOOT_CYCLES         32        // ..until transferring data
#define DEFAULT_CYCLE_DURATION_MSEC 300000    // 5 minutes
#define DEFAULT_CYCLE_DURATION      5         // 5 minutes
#define MAX_AWAKE_TIME_POWER        0         // infinite
#define MAX_AWAKE_TIME_TIMER        0         // infinite
#endif

#if (MAX_AWAKE_TIME_POWER == 0) || (MAX_AWAKE_TIME_TIMER == 0)
#pragma warning "timeout disabled!"
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

#if (MAX_AWAKE_TIME_POWER == 0) || (MAX_AWAKE_TIME_TIMER == 0)
#ifndef HEIDI_CONFIG_TEST
#error "Timeout disabled!"
#endif
#endif

extern bool GPSenabled;
extern bool GSMenabled;
extern bool MEASenabled;
extern bool CTLenabled;


#endif /* HEIDI_DEFINES_H_ */
