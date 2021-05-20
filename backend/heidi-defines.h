/*
 * heidi-defines.h
 * global defines
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */

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
//#define SEND_ALERT_SMS
//#define PRE_MEASURE_HANDLING
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


#define DEFAULT_BOOT_CYCLES         4         // ..until transferring data

#define DEFAULT_CYCLE_DURATION_MSEC 900000    // 15 minutes
#define DEFAULT_CYCLE_DURATION      15        // 15 minutes
#define MIN_CYCLE_DURATION_MSEC     300000    //  5 minutes
#define MIN_CYCLE_DURATION          5         //  5 minutes
#define MAX_CYCLE_DURATION_MSEC     3600000   //  1 hour
#define MAX_CYCLE_DURATION          60        //  1 hour
#define MAX_CYCLES_PER_DAY          288       //  5 minutes each
#define PRE_CYCLE_TIME              60000     //  wake GPS 1 minute before measuring
#define MAX_BOOT_CYCLES             12        // ..until transferring data
#define MAX_AWAKE_TIME_TIMER        120000    //  2 minutes !!!!
#define MAX_AWAKE_TIME_POWER        180000    //  3 minutes
#define MAX_MEAS_TIME               5000      //  5 seconds

#define START_FROM_RESET   -2
#define REFETCH_SYS_TIME   -1
#define INVALID_TIME_VALUE -1

#define S_TO_mS_FACTOR      1000      /* Conversion factor for seconds to milli seconds */
#define uS_TO_S_FACTOR      1000000   /* Conversion factor for micro seconds to seconds */
#define uS_TO_mS_FACTOR     1000      /* Conversion factor for milli seconds to seconds */
#define SLEEP_DUR_NOTIME    300000    /* 5 minutes if systime could not be set*/
#define SLEEP_DUR_ALERT     300000    /* 5 minutes */

#define MIN_SLEEP_TIME_MS   1000
#define MS_PER_DAY          86400000

#define DEFALUT_ACCELERATION_THRESHOLD  10000 /* never ever do an alert */
#define DEFALUT_ACCEL_THRES_MAX_COUNT   10000 /* never ever do an alert */

#define DEFALUT_NIGHT_HOUR_START_UTC    18
#define DEFALUT_NIGHT_HOUR_END_UTC      5

#define GSM_POWER_SAVE_1_VOLTAGE    3.65  //double transmission time
#define GSM_POWER_SAVE_2_VOLTAGE    3.5   //no more transmissions
#define GSM_POWER_SAVE_3_VOLTAGE    3.4   //do nothing just deep sleep
#define GSM_POWER_SAVE_4_VOLTAGE    3.3   //do nothing deep sleep for 1 hour

#define PRE_MEAS_STATE     0x0001
#define PRE_GPS_ALERT      0x0002
#define GPS_ALERT_1        0x0004
#define GPS_ALERT_2        0x0008
#define GPS_ALERT_PSD      0x0010
#define PRE_ACC_ALERT      0x0020
#define ACC_ALERT_1        0x0040
#define ACC_ALERT_2        0x0080
#define ACC_ALERT_PSD      0x0100
#define POWER_SAVE_1       0x0200
#define POWER_SAVE_2       0x0400
#define NOT_IN_CYCLE       0x0800
#define NEW_FENCE          0x1000
#define NEW_SETTINGS       0x2000
#define RESET_INITS        0x4000
#define TRSMT_DATA_ERROR   0x8000
//#define GOOD_NIGHT         0x2000

#ifdef HEIDI_CONFIG_1
#ifdef HEIDI_CONFIG_2
#error "HEIDI_CIRCUIT_2 and HEIDI_CIRCUIT_1 enabled"
#endif
#endif
#ifndef HEIDI_CONFIG_1
#ifndef HEIDI_CONFIG_2
#error "no hardware configuration set"
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


extern bool GPSenabled;
extern bool GSMenabled;
extern bool MEASenabled;
extern bool CTLenabled;


#endif /* HEIDI_DEFINES_H_ */
