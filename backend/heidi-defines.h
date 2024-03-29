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
 * GSM_MODULE              - there is a GSM module, e.g SIM800L
 * GPS_MODULE              - there is a GPS module, e.g ublox NEO-7M
 * COMMON_SERIAL           - GSM module and GPS module are sharing one serial interface
 * TEMP_SENSOR             - there is a temperature sensor, e.g. dallas DS 18B20
 * CHECK_BATTERY           - check battery status (disable this only for test configurations)
 * USE_VOLT_MEAS_EN_PIN    - use a dedicated pin to activate battery voltage measuring.
 *                           In the current version we double use the SIM800L reset wire for that, because SIM800L is
 *                           usually powered off and reset pin is HIGHZ at the start. For voltage measuring it is pulled
 *                           to ground. Not using this option means power up all sensors for just checking battery
 *                           status. In case of low battery this is semi-best.
 * USE_HEIDI_CONFIG_1_PINS - use GPIOs in HEIDI_1 configuration
 * USE_HEIDI_CONFIG_2_PINS - use GPIOs in HEIDI_2 configuration
 * USE_HEIDI_CONFIG_3_PINS - use GPIOs in HEIDI_3 configuration
 * I2C_BUS                 - enable i²c bus
 * I2C_SWITCH              - use i²c GPIO extender for enabling sensor power and GSM power, e.g PCA9536D
 * ACCELEROMETER           - there is a accelerometer module, e.g. ADXL345
 * USE_ULP                 - ultra low power processor of ESP32 is used for monitoring
 * USE_GPS_ALERT           - enable alerting modes for geo-fencing
 * USE_ACC_ALERT           - enable alerting modes for activity-monitoring (currently not implemented)
 * SEND_ALERT_SMS          - enable sending of alerting SMS
 * PRE_MEASURE_HANDLING    - power up all sensors some time before the measurements are scheduled
 *                           This is used for saving power, because e.g. the GPS module needs some seconds to get valid
 *                           results. The ESP32 may sleep meanwhile.
 * USE_RTC_FAST_MEM        - additionally use RTC fast memory to store data sets
 * USE_AOP_STATUS          - check AOP-status (for testing purposes) -> works not for CASIC modules
 * SAVE_AOP_DATA           - store GPS-AOP data on ESP32 RTC memory. (AOP - automatic orbit prediction - see ublox manual)
 *                           AOP is enabled for Heidi because it speeds up position fixes significantly. There are 32
 *                           satellites in orbit for whom NEO-7M calculates orbits, but it can only store 20 AOP data sets.
 *                           Storing all data sets and uploading them on start should help to further speed up, but in fact
 *                           it seems it doesn't. Furthermore it seems NEO-7M is able to store more than 20 data sets.
 *                           -> works not for CASIC modules
 * USE_CASIC_GPS           - switch to CASIC protocol modus for e.g. AT6558 chips
 *                           if the ublox-protocol won't work - maybe you got a fake module (as it happened to me) -
 *                           to find it out, enable DEBUG_SERIAL_GPS in heidi-debug.h and have a look on the boot prints
 *                           of the module. If you find something like "IC=AT6558-5N..." and "SW=URANUS4..." - bingo!
 * USE_OLED                - use SSD1306 OLED display - just for testing purposes
 * DEFAULT_BOOT_CYCLES     - boot cycles until transferring data (if there is no setup stored in flash)
 * DEFAULT_CYCLE_DURATION  - in minutes (if there is no setup stored in flash)
 * MAX_AWAKE_TIME_POWER    - extended measuring time after power up or in case of bad GPS-conditions (minutes)
 *                           On powering up from scratch, GPS-module has no orbit-data (ephemeris-data). We need to give
 *                           up-time for downloading it.
 * MAX_AWAKE_TIME_TIMER    - maximum up-time (minutes)
 * CHANGE_HERDE_ID_TO      - transmit another herd ID than defined (only for testing purposes) Needs a number , e.g.:
 *                           #define CHANGE_HERDE_ID_TO 2
 * NO_STEP_UP_CONV         - we use no step up converter - more strict voltage thresholds for power save - not good
 *
 */

#define HEIDI_CONFIG_3
//#define HEIDI_GATEWAY


//------------------------ heidi V1 ------------------------------------
#ifdef HEIDI_CONFIG_1
#define HEIDI_SW_BRAND              1
//#define LORA_V1_1_OLED
#define HEIDI_HERDE                 1
#define USE_HEIDI_CONFIG_1_PINS
#ifdef HEIDI_GATEWAY
#define GSM_MODULE
//#define SEND_ALERT_SMS
#endif
#define GPS_MODULE
#define USE_CASIC_GPS
#define USE_LORA
#define TEMP_SENSOR
#define CHECK_BATTERY
//#define USE_GPS_ALERT
//#define PRE_MEASURE_HANDLING
#define USE_RTC_FAST_MEM
#define DEFAULT_BOOT_CYCLES         12        // cycles until transferring data
#define DEFAULT_CYCLE_DURATION      15        // minutes
#define MAX_AWAKE_TIME_POWER        3         // minutes 0 = infinite
#define MAX_AWAKE_TIME_TIMER        2         // minutes 0 = infinite
#define DEFAULT_DIST_ALERT_THRES    100       // meters
#endif
//------------------------ heidi V2 ------------------------------------
#ifdef HEIDI_CONFIG_2
#define HEIDI_SW_BRAND              2
#define LORA_V1_3_OLED
#define USE_HEIDI_CONFIG_2_PINS
#define HEIDI_HERDE                 1
#ifdef HEIDI_GATEWAY
#define GSM_MODULE
//#define SEND_ALERT_SMS
#endif
#define GPS_MODULE
#define COMMON_SERIAL
#define TEMP_SENSOR
#define CHECK_BATTERY
#define USE_VOLT_MEAS_EN_PIN
#define I2C_BUS
#define ACCELEROMETER
#define USE_ULP
#define USE_GPS_ALERT
//#define USE_ACC_ALERT
#define USE_LORA
#define USE_RTC_FAST_MEM
#define DEFAULT_BOOT_CYCLES         12        // cycles until transferring data
#define DEFAULT_CYCLE_DURATION      15        // minutes
#define MAX_AWAKE_TIME_POWER        3         // minutes 0 = infinite
#define MAX_AWAKE_TIME_TIMER        2         // minutes 0 = infinite
#define DEFAULT_DIST_ALERT_THRES    100       // meters
#endif
//----------------------- heidi V3 ------------------------------------
#ifdef HEIDI_CONFIG_3
#define HEIDI_SW_BRAND              3
#define LORA_V1_3_OLED
#define USE_HEIDI_CONFIG_3_PINS
#define HEIDI_HERDE                 1
#ifdef HEIDI_GATEWAY
#define GSM_MODULE
#define SEND_ALERT_SMS
#endif
#define GPS_MODULE
#define COMMON_SERIAL
#define TEMP_SENSOR
#define CHECK_BATTERY
#define USE_VOLT_MEAS_EN_PIN
#define I2C_BUS
#define ACCELEROMETER
#define USE_ULP
#define USE_GPS_ALERT
//#define USE_ACC_ALERT
#define USE_LORA
#define USE_RTC_FAST_MEM
#define DEFAULT_BOOT_CYCLES         4         // cycles until transferring data
#define DEFAULT_CYCLE_DURATION      15        // minutes
#define MAX_AWAKE_TIME_POWER        3         // minutes 0 = infinite
#define MAX_AWAKE_TIME_TIMER        2         // minutes 0 = infinite
#define DEFAULT_DIST_ALERT_THRES    100       // meters
#endif
//------------------------ heidi test configuration ------------------------------------

#ifdef HEIDI_CONFIG_TEST
#define HEIDI_SW_BRAND              0
#define HEIDI_HERDE                 1
#ifdef HEIDI_GATEWAY
  //#define CHANGE_HERDE_ID_TO        2
  #define USE_HEIDI_CONFIG_3_PINS
  //#define LORA_V1_3_OLED
  #define GSM_MODULE
  #define GPS_MODULE
  //#define USE_CASIC_GPS
  //#define USE_NO_MEASURES
  //#define USE_NO_POSITION
  //#define CHECK_BATTERY
  #define TEMP_SENSOR
  #define COMMON_SERIAL
  #define USE_LORA
  #define USE_VOLT_MEAS_EN_PIN
  #define I2C_BUS
  #define ACCELEROMETER
  //#define ULP_LED_BLINK
  #define USE_ULP
  #define USE_RTC_FAST_MEM
#else
  #define USE_HEIDI_CONFIG_2_PINS
  #define LORA_V1_1_OLED
  #define GPS_MODULE
  //#define USE_CASIC_GPS
  #define COMMON_SERIAL
  #define TEMP_SENSOR
  //#define CHECK_BATTERY
  #define USE_LORA
  //#define USE_NO_MEASURES
  //#define USE_NO_POSITION
  #define USE_VOLT_MEAS_EN_PIN
  #define I2C_BUS
  //#define I2C_SWITCH
  #define ACCELEROMETER
  //#define ULP_LED_BLINK
  #define USE_ULP
  //#define USE_GPS_ALERT
  //#define USE_ACC_ALERT
  //#define PRE_MEASURE_HANDLING
  //#define USE_RTC_FAST_MEM
  //#define SAVE_AOP_DATA
  //#define USE_AOP_STATUS
#endif
#define DEFAULT_BOOT_CYCLES         3        // ..until transferring data
#define DEFAULT_CYCLE_DURATION      10       // minutes
#define MAX_AWAKE_TIME_POWER        3        // minutes 0 = infinite
#define MAX_AWAKE_TIME_TIMER        2        // minutes 0 = infinite
#define DEFAULT_DIST_ALERT_THRES    100      // meters
//#undef DEBUG_LEVEL
//#define DEBUG_LEVEL 3
#endif
/*
 * test defines
 */
#ifdef HEIDI_CONFIG_TEST
//#define TEST_ON_BOOT
//#define TEST_RTC
//#define TEST_CYCLES
//#define TEST_DATA
//#define TEST_GSM
//#define TEST_GPS
//#define TEST_ACC
//#define TEST_LORA
//#define TEST_TEMP
//#define TEST_VOLT
#endif
#ifndef TEST_ON_BOOT
  #ifndef TEST_RTC
    #ifndef TEST_CYCLES
      #ifndef TEST_DATA
        #ifndef TEST_GSM
          #ifndef TEST_GPS
            #ifndef TEST_ACC
              #ifndef TEST_LORA
                #ifndef TEST_TEMP
                  #ifndef TEST_VOLT
                    #define NO_TESTS
                  #endif
                #endif
              #endif
            #endif
          #endif
        #endif
      #endif
    #endif
  #endif
#endif

/*
 * system defines
 */
#define HEIDI_MAX_CLIENTS  32   //currently not more than 32 are supported because they are represented in a uint32_t bitmap
#define HEIDI_GATEWAY_ADDRESS 1 //address 0 = not set = empty data set
#define HEIDI_FIRST_CLIENT 2
#define MAX_DATA_SETS_PER_SEND_LINE 96

#ifdef HEIDI_GATEWAY
#define HEIDI_ANIMAL        HEIDI_GATEWAY_ADDRESS
#else
#ifdef TEST_LORA
#define HEIDI_ANIMAL        HEIDI_FIRST_CLIENT
#else
#define HEIDI_ANIMAL        HEIDI_FIRST_CLIENT+1
#endif
#endif


/*
 * data defines
 */
#define MIN_FREE_SETS_FOR_REQUEST  100

/*
 * timing defines
 */
#define MIN_CYCLE_DURATION          5         //  5 minutes
#define MAX_CYCLE_DURATION          60        //  1 hour
#define MAX_CYCLES_PER_DAY          288       //  5 minutes each a cycle + 2 fix points - but if we have 5 minutes each a cycle, the fix points will meet a cycle
#define PRE_CYCLE_TIME              30000     //  wake GPS 30 seconds before measuring (PRE_MEASURE_HANDLINg)
#define MAX_BOOT_CYCLES             (MAX_DATA_SETS >> 1) // ..until transferring data = half count of data sets
#define MAX_MEAS_TIME               5000      //  5 seconds
#define MAX_FAILED_ALERTS           10        //  abort alerting after 10 fails

#define START_FROM_RESET     -2
#define REFETCH_SYS_TIME     -1
#define FIRST_REGULAR_CYCLE  0
#define INVALID_TIME_VALUE   -1

#define S_TO_mS_FACTOR      1000      /* Conversion factor for seconds to milli seconds */
#define uS_TO_S_FACTOR      1000000   /* Conversion factor for micro seconds to seconds */
#define uS_TO_mS_FACTOR     1000      /* Conversion factor for milli seconds to seconds */
#define SLEEP_DUR_NOTIME    300000    /* 5 minutes if systime could not be set*/
#define SLEEP_DUR_ALERT     300000    /* 5 minutes */

#define MIN_SLEEP_TIME_MS   1000      /* 1 second */
#define MAX_SLEEP_TIME_MS   (MAX_CYCLE_DURATION * 60000)   /* 1 hour */
#define ONE_MINUTE          60000
#define MS_PER_DAY          86400000
#define FIX_POINT_MIN_MS    15000     //15 sec. minimum fix point duration


#ifdef HEIDI_GATEWAY
#define HEIDI_EARLIER_WAKEUP_S 0
#else
#define HEIDI_EARLIER_WAKEUP_S 3 //3 sec. earlier wake up
#endif
#define HEIDI_EARLIER_WAKEUP_MS (HEIDI_EARLIER_WAKEUP_S * S_TO_mS_FACTOR)
#define LORA_MAX_ONAIR_TIME_MS  (45000 + HEIDI_EARLIER_WAKEUP_MS) //must be less than 1 minute with a little guard time

#define MAX_AWAKE_TIME_POWER_MSEC (MAX_AWAKE_TIME_POWER * ONE_MINUTE)
#define MAX_AWAKE_TIME_TIMER_MSEC (MAX_AWAKE_TIME_TIMER * ONE_MINUTE)
#define AWAKE_TM_TRANSMIT_MSEC_OFFS ONE_MINUTE
#define DEFAULT_CYCLE_DURATION_MSEC (DEFAULT_CYCLE_DURATION * ONE_MINUTE)
#if (MAX_AWAKE_TIME_POWER_MSEC == 0) || (MAX_AWAKE_TIME_TIMER_MSEC == 0)
#pragma warning "timeout disabled!"
#endif

#define DEFALUT_NIGHT_HOUR_START_UTC    0
#define DEFALUT_NIGHT_HOUR_END_UTC      0
#define FIXPOINT_1_UTC 6
#define FIXPOINT_2_UTC 18

/*
 * measurement defines
 */
#ifdef HEIDI_CONFIG_TEST
#define DEFALUT_ACCELERATION_THRESHOLD_1    50
#define DEFALUT_ACCELERATION_THRESHOLD_2   150
#define DEFALUT_ACCEL_THRES_MAX_COUNT_1   16000 /* never ever do an alert */
#define DEFALUT_ACCEL_THRES_MAX_COUNT_2   16000 /* never ever do an alert */
#else
#define DEFALUT_ACCELERATION_THRESHOLD_1  100
#define DEFALUT_ACCELERATION_THRESHOLD_2  250
#define DEFALUT_ACCEL_THRES_MAX_COUNT_1   16000 /* never ever do an alert */
#define DEFALUT_ACCEL_THRES_MAX_COUNT_2   16000 /* never ever do an alert */
#endif
#ifdef NO_STEP_UP_CONV
#define GSM_POWER_SAVE_1_VOLTAGE    3.8   //double transmission time
#define GSM_POWER_SAVE_2_VOLTAGE    3.7   //no more transmissions
#define GSM_POWER_SAVE_3_VOLTAGE    3.3   //do nothing just deep sleep
#define GSM_POWER_SAVE_4_VOLTAGE    3.2   //do nothing deep sleep for 1 hour
#else
#define GSM_POWER_SAVE_1_VOLTAGE    3.6   //double transmission time
#define GSM_POWER_SAVE_2_VOLTAGE    3.4   //no more transmissions
#define GSM_POWER_SAVE_3_VOLTAGE    3.3   //do nothing just deep sleep
#define GSM_POWER_SAVE_4_VOLTAGE    3.2   //do nothing deep sleep for 1 hour
#endif

typedef enum _heidiStatus_t {
  PRE_MEAS_STATE      = 0x00000001,
  PRE_GPS_ALERT       = 0x00000002,
  GPS_ALERT_1         = 0x00000004,
  GPS_ALERT_2         = 0x00000008,
  GPS_ALERT_PSD       = 0x00000010, //alert passed
  PRE_ACC_ALERT       = 0x00000020,
  ACC_ALERT_1         = 0x00000040,
  ACC_ALERT_2         = 0x00000080,
  ACC_ALERT_PSD       = 0x00000100,
  POWER_SAVE_1        = 0x00000200,
  POWER_SAVE_2        = 0x00000400,
  NOT_IN_CYCLE        = 0x00000800,
  NEW_FENCE           = 0x00001000,
  NEW_SETTINGS        = 0x00002000,
  NEW_ACC_DATA        = 0x00004000,
  RESET_INITS         = 0x00008000,
  TRSMT_DATA_ERROR    = 0x00010000,
  ULP_RUNNING         = 0x00020000,
  FROM_PWR_SAVE_SLEEP = 0x00040000,
  IS_FIX_POINT        = 0x00080000
}heidiStatus_t;

/*
 * checks
 */

#ifdef HEIDI_CONFIG_1
#ifdef HEIDI_CONFIG_2
#error "HEIDI_1 and HEIDI_2 enabled"
#endif
#endif

#ifdef HEIDI_CONFIG_1
#ifdef HEIDI_CONFIG_3
#error "HEIDI_1 and HEIDI_3 enabled"
#endif
#endif

#ifdef HEIDI_CONFIG_3
#ifdef HEIDI_CONFIG_2
#error "HEIDI_2 and HEIDI_3 enabled"
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

#ifdef HEIDI_CONFIG_3
#ifdef HEIDI_CONFIG_TEST
#error "HEIDI_TEST and HEIDI_3 enabled"
#endif
#endif


#ifndef HEIDI_CONFIG_1
#ifndef HEIDI_CONFIG_2
#ifndef HEIDI_CONFIG_3
#ifndef HEIDI_CONFIG_TEST
#error "no hardware configuration set"
#endif
#endif
#endif
#endif

#ifndef USE_RTC_FAST_MEM
#ifndef USE_RTC_SLOW_MEM
#define USE_RTC_SLOW_MEM
#endif
#endif

#ifdef USE_HEIDI_CONFIG_1_PINS
#ifdef USE_HEIDI_CONFIG_2_PINS
#error "GPIO configuration clash"
#endif
#endif

#ifdef USE_HEIDI_CONFIG_1_PINS
#ifdef USE_HEIDI_CONFIG_3_PINS
#error "GPIO configuration clash"
#endif
#endif

#ifdef USE_HEIDI_CONFIG_2_PINS
#ifdef USE_HEIDI_CONFIG_3_PINS
#error "GPIO configuration clash"
#endif
#endif


#ifdef  SAVE_AOP_DATA
#ifndef USE_RTC_FAST_MEM
#error "AOP data resides in RTC slow memory, which is currently used for tracker data too."
#endif
#endif

#ifdef ACCELEROMETER
#ifndef I2C_BUS
#error "I2C bus for accelerometer not enabled"
#stop
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

#ifdef I2C_BUS
#ifndef COMMON_SERIAL
#error "I2C and 2nd UART does not work together"
#endif
#endif

#ifdef I2C_SWITCH
#ifndef I2C_BUS
#error "I2C bus for switch not enabled"
#endif
#ifdef USE_HEIDI_CONFIG_1_PINS
#error "USE_HEIDI_CONFIG_1_PINS and I2C_SWITCH does not work together"
#endif
#ifdef USE_HEIDI_CONFIG_2_PINS
#error "USE_HEIDI_CONFIG_2_PINS and I2C_SWITCH does not work together"
#endif
#ifdef USE_HEIDI_CONFIG_3_PINS
#error "USE_HEIDI_CONFIG_3_PINS and I2C_SWITCH does not work together"
#endif
#endif

#ifdef TEST_ACC
#ifndef ACCELEROMETER
#error "accelerometer not enabled"
#endif
#endif

#ifdef SEND_ALERT_SMS
#ifndef GSM_MODULE
#error "No GSM module -> no alert SMS"
#endif
#endif

#ifdef USE_GPS_ALERT
#ifndef GPS_MODULE
#error "No GPS module -> no GPS alert"
#endif
#endif

#ifdef USE_ACC_ALERT
#ifndef ACCELEROMETER
#error "No GSM module -> no acceleration alert"
#endif
#endif

#ifdef USE_ACC_ALERT
#ifndef USE_ULP
#error "No ULP measurement -> no acceleration alert"
#endif
#endif

#ifndef HEIDI_GATEWAY
#ifdef GSM_MODULE
#error "GSM-module works not for clients"
#endif
#endif


#ifdef USE_OLED
#ifndef USE_NO_MEASURES
#define USE_NO_MEASURES
#endif
#endif

#ifndef HEIDI_CONFIG_TEST
#if (MAX_AWAKE_TIME_POWER_MSEC == 0) || (MAX_AWAKE_TIME_TIMER_MSEC == 0)
#error "Timeout disabled!"
#endif

#ifdef USE_OLED
#error "Use OLED display for tests only"
#endif

#ifdef USE_NO_MEASURES
#error "USE_NO_MEASURES for tests only"
#endif

#ifdef USE_NO_POSITION
#error "USE_NO_POSITION for tests only"
#endif

#ifdef CHANGE_HERDE_ID_TO
#error "CHANGE_HERDE_ID_TO for tests only"
#endif
#endif //#ifndef HEIDI_CONFIG_TEST

#ifndef HEIDI_GATEWAY
#ifdef GSM_MODULE
#error "Clients usually have no GSM module"
#endif
#ifdef CHANGE_HERDE_ID_TO
#error "that makes no sense for clients"
#endif
#endif


#ifndef HEIDI_ANIMAL
#error "Please set a default value."
#endif
#ifndef HEIDI_HERDE
#error "Please set a default value."
#endif
#ifndef HEIDI_SERVER_PUSH_URL
#error "Please set a default value."
#endif
#ifndef HEIDI_MOBILE_APN
#error "Please set a default value."
#endif
#ifndef HEIDI_MOBILE_USER
#error "Please set a default value."
#endif
#ifndef HEIDI_MOBILE_PASSWD
#error "Please set a default value."
#endif
#ifndef HEIDI_SIM_PIN
#error "Please set a default value."
#endif
#ifdef USE_CASIC_GPS
#ifdef SAVE_AOP_DATA
#error "CASIC does not support AOP "
#endif
#ifdef USE_AOP_STATUS
#error "CASIC does not support AOP "
#endif
#endif

#ifdef TEST_VOLT
#undef CHECK_BATTERY
#endif

extern bool GPSenabled;
extern bool GSMenabled;
extern bool MEASenabled;
extern bool CTLenabled;
extern bool VOLTenabled;


#endif /* HEIDI_DEFINES_H_ */
