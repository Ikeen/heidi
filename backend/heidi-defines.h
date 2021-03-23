/*
 * heidi-defines.h
 * global values
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */

#ifndef HEIDI_DEFINES_H_
#define HEIDI_DEFINES_H_

/*
 * configuration
 */
#define HEIDI_GATE_WAY
#define HEIDI_HERDE         1
#define HEIDI_ANIMAL        1

#define GSM_MODULE
#define GPS_MODULE
#define TEMP_SENSOR
#define I2C_BUS
#define CHECK_BATTERY
#define ACCELEROMETER
#define I2C_SWITCH
#define USE_ULP

#define TEST_RTC
//#define TEST_ACC

#ifdef ACCELEROMETER
#ifndef I2C_BUS
#error "I2C bus for accelerometer not enabled"
#endif
#ifndef USE_ULP
#error "ULP for accelerometer not enabled"
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


#define BOOT_CYCLES           4
#define SLEEP_DURATION_MSEC   900000    /* 15 minutes */

#define START_FROM_RESET   -2
#define REFETCH_SYS_TIME   -1
#define INVALID_TIME_VALUE -1

#define uS_TO_S_FACTOR      1000000   /* Conversion factor for micro seconds to seconds */
#define uS_TO_mS_FACTOR     1000      /* Conversion factor for milli seconds to seconds */
#define SLEEP_DUR_NOTIME    300000    /* 5 minutes if systime could not be set*/
#define CYCLE_DURATION_MIN  (SLEEP_DURATION_MSEC / 60000)
#define SLEEP_MAX_SHIFT_MS  (SLEEP_DURATION_MSEC / 20) /* 5% max RTC shift */
#define SLEEP_MAX_SHIFT_S   (SLEEP_DURATION_MSEC / 20000) /* 5% max RTC shift */
#define MAX_SLEEP_TIME_MS   3600000
#define MIN_SLEEP_TIME_MS   1000
#define MS_PER_DAY          86400000

#define NIGHT_HOUR_START    20
#define NIGHT_HOUR_END      6

#define GSM_MINIMUM_VOLTAGE 3.6

extern bool GPSenabled;
extern bool GSMenabled;
extern bool MEASenabled;
extern bool CTLenabled;

#endif /* HEIDI_DEFINES_H_ */
