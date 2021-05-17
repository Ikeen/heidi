/*
 * TestAccel.h
 *
 *  Created on: 14.12.2020
 *      Author: frank
 *
 *  and code from
 *  2008 The Android Open Source Project
 *  updated by K. Townsend (Adafruit Industries)
 *
 *  and code from
 *  SparkFun_ADXL345.cpp
 *  E.Robert @ SparkFun Electronics
 *
 */

#ifndef TESTACCEL_H_
#define TESTACCEL_H_

#include <Arduino.h>
#include <stdint.h>
#include "esp32/ulp.h"

#ifdef ACCELEROMETER

extern bool _ADXL345_status;

#define ULP_INTERVALL_US  200000
#define ACCEL_HI_THRESHOLD   100  //high activity threshold
#define ACCEL_LO_THRESHOLD    50  //medium activity threshold
#define ACCEL_HI_CNT_WAKE   3000  //max high activity counts before wake SOC
#define ACCEL_LO_CNT_WAKE   6000  //max lo activity counts before wake SOC
/*
 * consider these values refer to a moving average that is determined 5 times per second.
 * a value of 3000 for ACCEL_HI_CNT_WAKE means: wake SOC if moving average of acceleration
 * exceeds for a total of 10 minutes since last reset of this counter
 */
#define USE_MORE_THAN_128_INSN 1 //use more than 128 ULP instructions
/*
 * increase ULP code Memory:
 * enable following define USE_MORE_THAN_128_INSN
 * set CONFIG_ULP_COPROC_RESERVE_MEM in "sdkconfig.h" AND "../tools/sdk/sdkconfig" to a higher value (512 = step size)
 * at least ULP_RTC_MEMORY_SIZE*4
 */

#define ADXL345_DEFAULT_ADDRESS byte(0x53)

/*************************** REGISTER MAP ***************************/
#define ADXL345_DEVID          0x00    // Device ID
#define ADXL345_RESERVED1      0x01    // Reserved. Do Not Access.
#define ADXL345_THRESH_TAP     0x1D    // Tap Threshold.
#define ADXL345_OFSX           0x1E    // X-Axis Offset.
#define ADXL345_OFSY           0x1F    // Y-Axis Offset.
#define ADXL345_OFSZ           0x20    // Z- Axis Offset.
#define ADXL345_DUR            0x21    // Tap Duration.
#define ADXL345_LATENT         0x22    // Tap Latency.
#define ADXL345_WINDOW         0x23    // Tap Window.
#define ADXL345_THRESH_ACT     0x24    // Activity Threshold
#define ADXL345_THRESH_INACT   0x25    // Inactivity Threshold
#define ADXL345_TIME_INACT     0x26    // Inactivity Time
#define ADXL345_ACT_INACT_CTL  0x27    // Axis Enable Control for Activity and Inactivity Detection
#define ADXL345_THRESH_FF      0x28    // Free-Fall Threshold.
#define ADXL345_TIME_FF        0x29    // Free-Fall Time.
#define ADXL345_TAP_AXES       0x2A    // Axis Control for Tap/Double Tap.
#define ADXL345_ACT_TAP_STATUS 0x2B    // Source of Tap/Double Tap
#define ADXL345_BW_RATE        0x2C    // Data Rate and Power mode Control
#define ADXL345_POWER_CTL      0x2D    // Power-Saving Features Control
#define ADXL345_INT_ENABLE     0x2E    // Interrupt Enable Control
#define ADXL345_INT_MAP        0x2F    // Interrupt Mapping Control
#define ADXL345_INT_SOURCE     0x30    // Source of Interrupts
#define ADXL345_DATA_FORMAT    0x31    // Data Format Control
#define ADXL345_DATAX0         0x32    // X-Axis Data 0
#define ADXL345_DATAX1         0x33    // X-Axis Data 1
#define ADXL345_DATAY0         0x34    // Y-Axis Data 0
#define ADXL345_DATAY1         0x35    // Y-Axis Data 1
#define ADXL345_DATAZ0         0x36    // Z-Axis Data 0
#define ADXL345_DATAZ1         0x37    // Z-Axis Data 1
#define ADXL345_FIFO_CTL       0x38    // FIFO Control
#define ADXL345_FIFO_STATUS    0x39    // FIFO Status

#define ADXL345_BW_1600     0xF     // 1111   IDD = 40uA
#define ADXL345_BW_800      0xE     // 1110   IDD = 90uA
#define ADXL345_BW_400      0xD     // 1101   IDD = 140uA
#define ADXL345_BW_200      0xC     // 1100   IDD = 140uA
#define ADXL345_BW_100      0xB     // 1011   IDD = 140uA
#define ADXL345_BW_50       0xA     // 1010   IDD = 140uA
#define ADXL345_BW_25       0x9     // 1001   IDD = 90uA
#define ADXL345_BW_12_5     0x8     // 1000   IDD = 60uA
#define ADXL345_BW_6_25     0x7     // 0111   IDD = 50uA
#define ADXL345_BW_3_13     0x6     // 0110   IDD = 45uA
#define ADXL345_BW_1_56     0x5     // 0101   IDD = 40uA
#define ADXL345_BW_0_78     0x4     // 0100   IDD = 34uA
#define ADXL345_BW_0_39     0x3     // 0011   IDD = 23uA
#define ADXL345_BW_0_20     0x2     // 0010   IDD = 23uA
#define ADXL345_BW_0_10     0x1     // 0001   IDD = 23uA
#define ADXL345_BW_0_05     0x0     // 0000   IDD = 23uA


 /************************** INTERRUPT PINS **************************/
#define ADXL345_INT1_PIN    0x00    //INT1: 0
#define ADXL345_INT2_PIN    0x01    //INT2: 1


 /********************** INTERRUPT BIT POSITION **********************/
#define ADXL345_INT_DATA_READY_BIT    0x07
#define ADXL345_INT_SINGLE_TAP_BIT    0x06
#define ADXL345_INT_DOUBLE_TAP_BIT    0x05
#define ADXL345_INT_ACTIVITY_BIT      0x04
#define ADXL345_INT_INACTIVITY_BIT    0x03
#define ADXL345_INT_FREE_FALL_BIT     0x02
#define ADXL345_INT_WATERMARK_BIT     0x01
#define ADXL345_INT_OVERRUNY_BIT      0x00

#define ADXL345_DATA_READY        0x07
#define ADXL345_SINGLE_TAP        0x06
#define ADXL345_DOUBLE_TAP        0x05
#define ADXL345_ACTIVITY          0x04
#define ADXL345_INACTIVITY        0x03
#define ADXL345_FREE_FALL         0x02
#define ADXL345_WATERMARK         0x01
#define ADXL345_OVERRUNY          0x00

/****************************** ERRORS ******************************/
#define ADXL345_OK      1   // No Error
#define ADXL345_ERROR   0   // Error Exists

#define ADXL345_NO_ERROR    0   // Initial State
#define ADXL345_READ_ERROR  1   // Accelerometer Reading Error
#define ADXL345_BAD_ARG     2   // Bad Argument

/**
 * @brief Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth

*/
typedef enum {
  ADXL345_DATARATE_3200_HZ = 0b1111, ///< 1600Hz Bandwidth   140uA IDD
  ADXL345_DATARATE_1600_HZ = 0b1110, ///<  800Hz Bandwidth    90uA IDD
  ADXL345_DATARATE_800_HZ  = 0b1101, ///<  400Hz Bandwidth   140uA IDD
  ADXL345_DATARATE_400_HZ  = 0b1100, ///<  200Hz Bandwidth   140uA IDD
  ADXL345_DATARATE_200_HZ  = 0b1011, ///<  100Hz Bandwidth   140uA IDD
  ADXL345_DATARATE_100_HZ  = 0b1010, ///<   50Hz Bandwidth   140uA IDD
  ADXL345_DATARATE_50_HZ   = 0b1001, ///<   25Hz Bandwidth    90uA IDD
  ADXL345_DATARATE_25_HZ   = 0b1000, ///< 12.5Hz Bandwidth    60uA IDD
  ADXL345_DATARATE_12_5_HZ = 0b0111, ///< 6.25Hz Bandwidth    50uA IDD
  ADXL345_DATARATE_6_25HZ  = 0b0110, ///< 3.13Hz Bandwidth    45uA IDD
  ADXL345_DATARATE_3_13_HZ = 0b0101, ///< 1.56Hz Bandwidth    40uA IDD
  ADXL345_DATARATE_1_56_HZ = 0b0100, ///< 0.78Hz Bandwidth    34uA IDD
  ADXL345_DATARATE_0_78_HZ = 0b0011, ///< 0.39Hz Bandwidth    23uA IDD
  ADXL345_DATARATE_0_39_HZ = 0b0010, ///< 0.20Hz Bandwidth    23uA IDD
  ADXL345_DATARATE_0_20_HZ = 0b0001, ///< 0.10Hz Bandwidth    23uA IDD
  ADXL345_DATARATE_0_10_HZ = 0b0000  ///< 0.05Hz Bandwidth    23uA IDD (default value)
} dataRate_t;

/**
 * @brief  Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range
 *
 */
typedef enum {
  ADXL345_RANGE_16_G = 0b11, ///< +/- 16g
  ADXL345_RANGE_8_G = 0b10,  ///< +/- 8g
  ADXL345_RANGE_4_G = 0b01,  ///< +/- 4g
  ADXL345_RANGE_2_G = 0b00   ///< +/- 2g (default value)
} range_t;


bool    init_ADXL345(void);
bool    wake_config_ADXL345(void);
bool    sleep_ADXL345(void);
bool    _ADXL345_get_bw_code(uint8_t* value);
bool    _ADXL345_set_bw(uint8_t bw_code);
bool    _ADXL345_getFullResBit(bool* value);
bool    _ADXL345_setFullResBit(bool fullResBit);
bool    _ADXL345_getRate(double *value);
bool    _ADXL345_setRate(double rate);
bool    _ADXL345_get_bw_code(uint8_t* value);
bool    _ADXL345_getRange(range_t* value);
bool    _ADXL345_setRange(range_t range);
bool    _ADXL345_setDataRate(dataRate_t dataRate);
bool    _ADXL345_getDataRate(dataRate_t* value);
bool    _ADXL345_getX(int16_t* value);
bool    _ADXL345_getY(int16_t* value);
bool    _ADXL345_getZ(int16_t* value);

#ifdef USE_ULP
/*
 *

RTC GPIO  GPIO   Pad Name     Analog Function
                              1          2          3
0         36     SENSOR_VP    ADC_H      ADC1_CH0   -
1         37     SENSOR_CAPP  ADC_H      ADC1_CH1   -
2         38     SENSOR_CAPN  ADC_H      ADC1_CH2   -
3         39     SENSOR_VN    ADC_H      ADC1_CH3   -
4         34     VDET_1       -          ADC1_CH6   -
5         35     VDET_2       -          ADC1_CH7   -
6         25     GPIO25       DAC_1      ADC2_CH8   -
7         26     GPIO26       DAC_2      ADC2_CH9   -
8         33     32K_XN       XTAL_32K_N ADC1_CH5   TOUCH8
9         32     32K_XP       XTAL_32K_P ADC1_CH4   TOUCH9
10         4     GPIO4        -          ADC2_CH0   TOUCH0
11         0     GPIO0        -          ADC2_CH1   TOUCH1
12         2     GPIO2        -          ADC2_CH2   TOUCH2
13        15     MTDO         -          ADC2_CH3   TOUCH3
14        13     MTCK         -          ADC2_CH4   TOUCH4
15        12     MTDI         -          ADC2_CH5   TOUCH5
16        14     MTMS         -          ADC2_CH6   TOUCH6
17        27     GPIO27       -          ADC2_CH7   TOUCH7

please consider:
RTC_GPIO 17 is controlled by bit 31 of RTC control registers like RTCIO_RTC_GPIO_ENABLE (see reference)
So GPIO2 is controlled by bit 26 (12+14) of control registers

https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/ulp_instruction_set.html#note-about-instruction-execution-time

100.000 cycles = 12ms -> 1ms 8000 cycles -> 1us 8 cycles -> 8 MHz

WAIT(c)  6+c cycles         == I_DELAY(c)
REG_RC   4+4 cycles 1    us
REG_WR   8+4 cycles 1.25 us
M_BL     2+2 cycles 0.5  us
M_BGE    2+2 cycles 0.5  us
M_BX     2+2 cycles 0.5  us
LD       4+4 cycles 1    us
ST       4+4 cycles 1    us
MOVE, SUB, ADD, RSH, LSH, OR, AND, NOP
         2+4 cycles 0.75 us

*/

/*
 * increase ULP code Memory:
 * enable following define
 * set CONFIG_ULP_COPROC_RESERVE_MEM in ".arduinocdt/packages/esp32/hardware/esp32/1.0.0/tools/sdk/include/config/sdkconfig.h"
 *   AND in ".arduinocdt/packages/esp32/hardware/esp32/1.0.0/tools/sdk/sdkconfig" to a higher value (512 = step size)
 */
#define ULP_LED_BLINK          0
#define USE_MORE_THAN_128_INSN 1

#define I2C_DEBUG   0                  //need to increase code size if enabled
#define I2C_SUCCESS 0
#define I2C_FAILED  1
#define ACCEL_ULP_CODE_SIZE 200
#define ACCEL_DATA_HEADER ACCEL_ULP_CODE_SIZE
#define ACCEL_MEAS_CNT 0
#define ACCEL_LOOP_CUR 1
#define ACCEL_DATA_CUR 2
#define AVR_DIFF_VAL 3
#define AVR_EXCD_TH1 4
#define AVR_EXCD_CT1 5
#define AVR_WAKE_TH1 6
#define AVR_EXCD_TH2 7
#define AVR_EXCD_CT2 8
#define AVR_WAKE_TH2 9
#define ACCEL_HD_LEN 10
#define ACCEL_X_VALUES (ACCEL_DATA_HEADER + ACCEL_HD_LEN)
#define I2C_DATA_REG 0
#define I2C_TRNS_RES 1
#define CUR_READ_RES 2
#define LST_READ_RES 3
#define CUR_DIFF_VAL 4
#define ACCEL_DT_LEN 5
#define ACCEL_Y_VALUES (ACCEL_X_VALUES + ACCEL_DT_LEN)
#define ACCEL_Z_VALUES (ACCEL_Y_VALUES + ACCEL_DT_LEN)

#define ACCEL_X_REGISTER (ACCEL_X_VALUES + I2C_DATA_REG)
#define ACCEL_Y_REGISTER (ACCEL_Y_VALUES + I2C_DATA_REG)
#define ACCEL_Z_REGISTER (ACCEL_Z_VALUES + I2C_DATA_REG)

#define EXTRA_RTC_DATA (ACCEL_Z_VALUES + ACCEL_DT_LEN)
#define EXTRA_RTC_DATA_LEN 1

#define DEBUG_LED_MEM EXTRA_RTC_DATA     //one store place for LED flashing

#define ACCEL_ULP_DATA_SIZE (ACCEL_HD_LEN + ACCEL_DT_LEN + ACCEL_DT_LEN + ACCEL_DT_LEN + EXTRA_RTC_DATA_LEN)
#define ACCEL_ULP_MEM_SIZE (ACCEL_ULP_CODE_SIZE + ACCEL_ULP_DATA_SIZE)

/*
 * accelerator data
 * register_address_x, last_measure_x, count_thres_exceed_x, average_x,
 * register_address_y, last_measure_y, count_thres_exceed_y, average_y,
 * register_address_z, last_measure_z, count_thres_exceed_z, average_z,
 * counter, threshold
 *
 * = 14
 */

#define GPIO_NUM_0_RTC  25 //11+14 -- do not use for bus
#define GPIO_NUM_2_RTC  26 //12+14
#define GPIO_NUM_4_RTC  24 //10+14
#define GPIO_NUM_12_RTC 29 //15+14
#define GPIO_NUM_13_RTC 28 //14+14
#define GPIO_NUM_14_RTC 30 //16+14
#define GPIO_NUM_15_RTC 27 //13+14

#define RTC_GPIO_BIT_SDA GPIO_NUM_13_RTC
#define RTC_GPIO_BIT_SCL GPIO_NUM_4_RTC
#define RTC_GPIO_BIT_LED GPIO_NUM_2_RTC

#define SDA_INPUT   I_WR_REG_BIT(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_BIT_SDA, 1)
#define SDA_OUTPUT  I_WR_REG_BIT(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_BIT_SDA, 1)
#define SDA_DRIVE_L I_WR_REG_BIT(RTC_GPIO_OUT_W1TC_REG,RTC_GPIO_BIT_SDA,1)
#define SDA_DRIVE_H I_WR_REG_BIT(RTC_GPIO_OUT_W1TS_REG,RTC_GPIO_BIT_SDA,1)
#define SDA_H SDA_INPUT
#define SDA_L SDA_OUTPUT
#define SDA_READ    I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_BIT_SDA, RTC_GPIO_BIT_SDA)

#define SCL_INPUT   I_WR_REG_BIT(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_BIT_SCL, 1)
#define SCL_OUTPUT  I_WR_REG_BIT(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_BIT_SCL, 1)
#define SCL_DRIVE_L I_WR_REG_BIT(RTC_GPIO_OUT_W1TC_REG,RTC_GPIO_BIT_SCL,1)
#define SCL_DRIVE_H I_WR_REG_BIT(RTC_GPIO_OUT_W1TS_REG,RTC_GPIO_BIT_SCL,1)
#define SCL_H SCL_INPUT
#define SCL_L SCL_OUTPUT
#define SCL_READ    I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_BIT_SCL, RTC_GPIO_BIT_SCL)

void init_accel_ULP(uint32_t intervall_us);
/* get / set measure count value */
uint16_t get_accel_meas_cnt_ULP();
void set_accel_meas_cnt_ULP(uint16_t val);
/* get / set moving averaged sum value */
uint16_t get_accel_avrerage_ULP(void);
void set_accel_avrerage_ULP(uint16_t val);
/* get / set threshold 1 exceed counter value */
uint16_t get_accel_excnt1_ULP();
void set_accel_excnt1_ULP(uint16_t val);
/* get / set threshold 1 */
uint16_t get_accel_exthr1_ULP();
void set_accel_exthr1_ULP(uint16_t val);
/* get / set wake value 1 */
uint16_t get_accel_wake1_ULP();
void set_accel_wake1_ULP(uint16_t val);
/* get / set threshold 2 exceed counter value */
uint16_t get_accel_excnt2_ULP();
void set_accel_excnt2_ULP(uint16_t val);
/* get / set threshold 2 */
uint16_t get_accel_exthr2_ULP();
void set_accel_exthr2_ULP(uint16_t val);
/* get / set wake value 2 */
uint16_t get_accel_wake2_ULP();
void set_accel_wake2_ULP(uint16_t val);

uint16_t _getAccThresCnt(uint16_t dayThres);

#if USE_MORE_THAN_128_INSN

#define ULP_RESERVE_MEM CONFIG_ULP_COPROC_RESERVE_MEM
#define SUB_OPCODE_MACRO_LABELPC 2  /*!< Label pointer macro */

typedef struct {
    uint32_t label : 16;
    uint32_t addr : 11;
    uint32_t unused : 1;
    uint32_t type : 4;
} reloc_info_t;

#define RELOC_TYPE_LABEL   0
#define RELOC_TYPE_BRANCH  1
#define RELOC_TYPE_LABELPC 2
#define SUB_OPCODE_BS  2            /*!< Branch to relative PC, conditional on the stage counter */

/* This record means: there is a label at address
 * insn_addr, with number label_num.
 */
#define RELOC_INFO_LABEL(label_num, insn_addr) (reloc_info_t) { \
    .label = label_num, \
    .addr = insn_addr, \
    .unused = 0, \
    .type = RELOC_TYPE_LABEL }

/* This record means: there is a branch instruction at
 * insn_addr, it needs to be changed to point to address
 * of label label_num.
 */
#define RELOC_INFO_BRANCH(label_num, insn_addr) (reloc_info_t) { \
    .label = label_num, \
    .addr = insn_addr, \
    .unused = 0, \
    .type = RELOC_TYPE_BRANCH }

/* This record means: there is a move instruction at insn_addr,
 * imm needs to be changed to the program counter of the instruction
 * at label label_num.
 */
#define RELOC_INFO_LABELPC(label_num, insn_addr) (reloc_info_t) { \
    .label = label_num, \
    .addr = insn_addr, \
    .unused = 0, \
    .type = RELOC_TYPE_LABELPC }


esp_err_t ulp_process_macros_and_load_big(uint32_t load_addr, const ulp_insn_t* program, size_t* psize);
#endif // USE_MORE_THAN_128_INSN

#else

#define ACCEL_ULP_MEM_SIZE 0

#endif // USE_ULP

#endif // ACCELEROMETER

#endif /* TESTACCEL_H_ */
