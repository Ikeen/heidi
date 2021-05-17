/*
 * TestAccel.cpp
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


 /*
 Europe:
         433,05 MHz - 434,79 MHz (ISM-Band Region 1)
         863,00 MHz - 870,00 MHz (SRD-Band Europa)
 North America:
         902,00 MHz - 928,00 MHz (ISM-Band Region 2)
*/



#include <Arduino.h>
#include <Wire.h>
#include <assert.h>
#include <esp32/ulp.h>
#include <LoRa.h>
#include <driver/rtc_io.h>
#include "heidi-measures.h"
#include "heidi-data.h"
#include "heidi-sys.h"
#include "heidi-acc.h"

#ifdef ACCELEROMETER

bool _ADXL345_status = false;

bool init_ADXL345(){
  byte _ID;
  _D(DebugPrint("Init ADXL345.. ", DEBUG_LEVEL_2));
  //check sensor
  if(iic_readRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_DEVID, &_ID) != I2C_ERROR_OK){
    iic_clockFree();
    if(iic_readRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_DEVID, &_ID)!= I2C_ERROR_OK) {
      _D(DebugPrintln("ADXL345 read ID register failed." , DEBUG_LEVEL_1));
      return false;
    }
    _D(DebugPrintln("successful", DEBUG_LEVEL_2));
  }
  if (_ID != 0xE5){
    _D(DebugPrintln("ADXL345 not found, read 0x" + String(_ID, HEX) + " instead 0xe5." , DEBUG_LEVEL_1));
    return false;
  }
  _D(DebugPrintln("successful", DEBUG_LEVEL_2));
  return true;
}

bool wake_config_ADXL345(void){
  _D(DebugPrint("Wake ADXL345.. ", DEBUG_LEVEL_2));
  // wake up sensor
  if(iic_writeRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_POWER_CTL, 0x08) != I2C_ERROR_OK) { setError(E_IIC_ERROR); return false; }
  _ADXL345_status = true;
  // Set the range to whatever is appropriate for your project
  if (!_ADXL345_setRange(ADXL345_RANGE_16_G)) { setError(E_IIC_ERROR); }
  if (!_ADXL345_setDataRate(ADXL345_DATARATE_12_5_HZ)) { setError(E_IIC_ERROR); }
  if (!_ADXL345_setFullResBit(false)) { setError(E_IIC_ERROR); }
  _D(DebugPrintln("successful", DEBUG_LEVEL_2));
  return true;
}

bool sleep_ADXL345(){
  // wake up sensor
  if(iic_writeRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_POWER_CTL, 0x00) != I2C_ERROR_OK) { setError(E_IIC_ERROR); return false; }
  _ADXL345_status = false;
  return true;
}
/*************************** BANDWIDTH ******************************/
/*                          ~ SET & GET                             */
bool _ADXL345_get_bw_code(uint8_t *value){
  return (iic_readRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_BW_RATE, value) == I2C_ERROR_OK);
}

bool _ADXL345_set_bw(byte bw_code){
  if((bw_code < ADXL345_BW_0_05) || (bw_code > ADXL345_BW_1600)){
    _ADXL345_status = false;
    return false;
  }
  else{
    return (iic_writeRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_BW_RATE, bw_code) == I2C_ERROR_OK);
  }
}
/************************* FULL_RES BIT STATE ***********************/
/*                           ~ GET & SET                            */
bool _ADXL345_getFullResBit(bool* value) {
  return (iic_getRegisterBit(ADXL345_DEFAULT_ADDRESS, ADXL345_DATA_FORMAT, 3, value) == I2C_ERROR_OK);
}

/*  If Set (1) Device is in Full Resolution Mode: Output Resolution Increase with G Range
 *  Set by the Range Bits to Maintain a 4mg/LSB Scale Factor
 *  If Set (0) Device is in 10-bit Mode: Range Bits Determine Maximum G Range
 *  And Scale Factor
 */
bool _ADXL345_setFullResBit(bool fullResBit) {
  return (iic_setRegisterBit(ADXL345_DEFAULT_ADDRESS, ADXL345_DATA_FORMAT, 3, fullResBit) == I2C_ERROR_OK);
}

/*************************** RATE BITS ******************************/
/*                                                                  */
bool _ADXL345_getRate(double *value){
  byte _b;
  *value = 0.0;
  if (iic_readRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_BW_RATE, &_b) != I2C_ERROR_OK) { return false; }
  _b &= B00001111;
  *value = (pow(2,((int) _b)-6)) * 6.25;
  return true;
}

bool _ADXL345_setRate(double rate){
  byte _b,_s;
  int v = (int) (rate / 6.25);
  int r = 0;
  while (v >>= 1)
  { r++; }
  if (r <= 9) {
    if (iic_readRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_BW_RATE, &_b) == I2C_ERROR_OK){
      _s = (byte) (r + 6) | (_b & B11110000);
      return (iic_writeRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_BW_RATE, _b) == I2C_ERROR_OK);
    } else { return false; }
  }
  return true;
}

bool _ADXL345_setDataRate(dataRate_t dataRate) {
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
  return (iic_writeRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_BW_RATE, dataRate) == I2C_ERROR_OK);
}

bool _ADXL345_getDataRate(dataRate_t *value) {
  uint8_t b;
  if (iic_readRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_BW_RATE, &b) != I2C_ERROR_OK) { return false; }
  *value = (dataRate_t)(b & 0x0F);
  return true;
}

bool _ADXL345_getRange(range_t *value) {
  /* Read the data format register to preserve bits */
  uint8_t b;
  if (iic_readRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_DATA_FORMAT, &b) != I2C_ERROR_OK) { return false; }
  *value = (range_t)(b & 0x03);
  return true;
}

bool _ADXL345_getX(int16_t *value) {
  return (iic_readRegister16(ADXL345_DEFAULT_ADDRESS, ADXL345_DATAX0, value) == I2C_ERROR_OK);
}
bool _ADXL345_getY(int16_t *value) {
  return (iic_readRegister16(ADXL345_DEFAULT_ADDRESS, ADXL345_DATAY0, value) == I2C_ERROR_OK);
}
bool _ADXL345_getZ(int16_t *value) {
  return (iic_readRegister16(ADXL345_DEFAULT_ADDRESS, ADXL345_DATAZ0, value) == I2C_ERROR_OK);
}

bool _ADXL345_setRange(range_t range) {
  /* Read the data format register to preserve bits */
  uint8_t format;
  if (iic_readRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_DATA_FORMAT, &format) != I2C_ERROR_OK) { return false; }
  /* Update the data rate */
  format &= ~0x0F;
  format |= range;
  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;
  /* Write the register back to the IC */
  return (iic_writeRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_DATA_FORMAT, format) == I2C_ERROR_OK);
}

#ifdef USE_ULP

void init_accel_ULP(uint32_t intervall_us) {
  //for(int i=0; i<ACCEL_ULP_CODE_SIZE; i++){RTC_SLOW_MEM[i] = 0;}
  ulp_set_wakeup_period(0, intervall_us);
  const ulp_insn_t ulp_accel[] = {

#if ULP_LED_BLINK
  //LED flashing in debug mode for check "ULP is running"
  I_WR_REG_BIT(RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_HOLD_S, 0), // HOLD off GPIO 2
  I_MOVI(R3,DEBUG_LED_MEM),               // #mem -> R3
  I_LD(R0,R3,0),                          // R0 = mem]
  I_ADDI(R0,R0,1),
  M_BGE(91,10),                           //R0 >= 10?
  I_WR_REG_BIT(RTC_GPIO_OUT_W1TC_REG,RTC_GPIO_BIT_LED,1),
  M_BX(92),
  M_LABEL(91),
  I_WR_REG_BIT(RTC_GPIO_OUT_W1TS_REG,RTC_GPIO_BIT_LED,1),
  M_BL(92,11),                            //R0 < 20?
  I_MOVI(R0,0),
  M_LABEL(92),
  I_ST(R0,R3,0),                          // mem = R0
  I_WR_REG_BIT(RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_HOLD_S, 1), // HOLD on LED
#endif


    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_0_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_0_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD1_REG, RTC_IO_TOUCH_PAD1_HOLD_S, 0), // HOLD off GPIO 0
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_2_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_2_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_HOLD_S, 0), // HOLD off GPIO 2
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_4_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_4_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD0_REG, RTC_IO_TOUCH_PAD0_HOLD_S, 0), // HOLD off GPIO 4
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_12_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_12_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD5_REG, RTC_IO_TOUCH_PAD5_HOLD_S, 0), // HOLD off GPIO 12
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_13_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_13_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD4_REG, RTC_IO_TOUCH_PAD4_HOLD_S, 0), // HOLD off GPIO 13
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_14_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_14_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD6_REG, RTC_IO_TOUCH_PAD6_HOLD_S, 0), // HOLD off GPIO 14
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_15_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_15_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD3_REG, RTC_IO_TOUCH_PAD3_HOLD_S, 0), // HOLD off GPIO 15
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_27_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_27_RTC)
      I_WR_REG_BIT(RTC_IO_TOUCH_PAD7_REG, RTC_IO_TOUCH_PAD7_HOLD_S, 0), // HOLD off GPIO 27
    #endif

    SDA_INPUT, //(=HIGH)
    SCL_INPUT, //(=HIGH)
    SDA_DRIVE_L,
    SCL_DRIVE_L,

  M_LABEL(81), //loop for reading 3 registers

    I_MOVI(R0,I2C_FAILED), //set status to failed
    I_MOVI(R2,ACCEL_DATA_HEADER),
    I_LD(R2,R2,ACCEL_DATA_CUR),
    I_ST(R0,R2,I2C_TRNS_RES),
    I_LD(R0,R2,CUR_READ_RES), //store last read result
    I_ST(R0,R2,LST_READ_RES),
    I_MOVI(R3,2),     //write 3 bytes
    /*
     * 1st round: device Address
     */
    I_MOVI(R2,ADXL345_DEFAULT_ADDRESS),   // device address
    I_LSHI(R2,R2,2),   // 1 more because of limited conditional branch capability
    //I_ORI(R2,R2,0x00), // R/W - bit, write = 0;

    M_LABEL(9),
    //start condition
    SDA_L,  //1.25us per I_WR_REG_BIT
    SCL_L,

  M_LABEL(1),
     /*
      *write byte in R2
      */
      I_MOVI(R0, 0x08),
    M_LABEL(2),
      I_RSHR(R1,R2,R0),
      I_ANDI(R1,R1,0x01),
      M_BXZ(3),  //1
      SDA_H,
      M_BX(4),   //2
      M_LABEL(3),
      SDA_L,
    M_LABEL(4),
      SCL_H,
      SCL_L,
      I_SUBI(R0,R0,1),
      M_BGE(2,1),  //3
    SDA_INPUT,
    SCL_H,
    SDA_READ,  //ACK
    SCL_L,
    M_BGE(99,1),  //4
    I_SUBI(R3,R3,1),
    M_BXZ(5),        //if loop-counter == 0   //5
    M_BXF(6),        //if loop-counter == -1  //6
    /*
    * write register to read next
    */
    I_MOVI(R2,ACCEL_DATA_HEADER),
    I_LD(R2,R2,ACCEL_DATA_CUR),
    I_LD(R2,R2,I2C_DATA_REG),        // current register address
    I_LSHI(R2,R2,1),                 // 1 more because of limited conditional branches
    M_BX(1),   //7
  M_LABEL(5),

    /*
    * send read request
    */
    SCL_H, //prepare restart condition + a little delay done by the following instructions
    I_MOVI(R2,ADXL345_DEFAULT_ADDRESS),   // device address
    I_LSHI(R2,R2,2),   // 1 more because of limited conditional branches
    I_ORI(R2,R2,0x02), // R/W - bit, read = 1;
    M_BX(9),  //8
  M_LABEL(6),

    /*
     * now read 2 bytes
     */
    I_MOVI(R3,2),     //write 2 bytes
    SDA_INPUT, // maybe already done by reading ACK
    I_MOVI(R1,0),
  M_LABEL(11),
    I_MOVI(R2, 0x08),
  M_LABEL(12),
    SCL_H,
    SDA_READ,
    SCL_L,
    I_LSHI(R1,R1,1),
    I_ORR(R1,R1,R0),
    I_SUBI(R2,R2,1),
    M_BXZ(13),  //9
    M_BX(12),   //10
  M_LABEL(13),
    I_SUBI(R3,R3,1),
    M_BXZ(14),     //if loop-counter == 0  //11
    SDA_L,         //ack fromMaser later on SDA_L = SDA_OUTPUT
    SCL_H,
    I_DELAY(2),    //give a little delay
    SCL_L,
    SDA_H,         //SDA_H = SDA_INPUT
    I_DELAY(2),    //give a little delay
    M_BX(11),      //12

  M_LABEL(14),
    I_RSHI(R2,R1,8),
    I_LSHI(R1,R1,8),
    I_ORR(R1,R1,R2),
    I_MOVI(R2,ACCEL_DATA_HEADER),
    I_LD(R2,R2,ACCEL_DATA_CUR),
    I_ST(R1,R2,CUR_READ_RES),
    I_MOVI(R0,I2C_SUCCESS),
    I_ST(R0,R2,I2C_TRNS_RES),
  M_LABEL(99),
    //stop condition
    SDA_L,
    I_DELAY(10), //give a little delay
    SCL_H,
    I_DELAY(6),
    SDA_H,
    //now calculations
    //R0 still holds transmission result, R1 holds current acc value
    M_BGE(83,1), //no calculation if transmission fails
    I_MOVI(R3,ACCEL_DATA_HEADER),
    I_LD(R3,R3,ACCEL_DATA_CUR),
    I_LD(R2,R3,LST_READ_RES),
    I_SUBR(R0,R2,R1),
    I_ANDI(R3,R0,0x8000),
    M_BXZ(82),
    I_SUBR(R0,R1,R2),
  M_LABEL(82),
    I_MOVI(R3,ACCEL_DATA_HEADER),
    I_LD(R3,R3,ACCEL_DATA_CUR),
    I_ST(R0,R3,CUR_DIFF_VAL),

  M_LABEL(83),
    I_MOVI(R3,ACCEL_DATA_HEADER),
    I_LD(R0,R3,ACCEL_LOOP_CUR),
    I_ADDI(R0,R0,1),
    M_BGE(84,3),        //3 loops - we're ready
    I_ST(R0,R3,ACCEL_LOOP_CUR), //else prepare next loop
    I_LD(R0,R3,ACCEL_DATA_CUR),
    I_ADDI(R0,R0,ACCEL_DT_LEN),
    I_ST(R0,R3,ACCEL_DATA_CUR),
    M_BX(81),

    //end of transmissions
    M_LABEL(84),
    //count measures (R3 holds still ACCEL_DATA_HEADER);
    I_LD(R0,R3,ACCEL_MEAS_CNT),
    I_ADDI(R0,R0,1),
    I_ST(R0,R3,ACCEL_MEAS_CNT),

    I_MOVI(R0,0),
    I_ST(R0,R3,ACCEL_LOOP_CUR), //reset loop counter
    I_MOVI(R0,ACCEL_X_VALUES),
    I_ST(R0,R3,ACCEL_DATA_CUR), //reset data pointer

    //calculate averaged values
    I_LD(R1,R3,AVR_DIFF_VAL),
    I_LSHI(R2,R1,4),
    I_SUBR(R2,R2,R1),  //averaged value * 31
    I_RSHI(R2,R2,4),   //averaged value / 32

    I_MOVI(R1,ACCEL_X_VALUES),
    I_LD(R0,R1,CUR_DIFF_VAL),
    I_ADDR(R2,R2,R0),  //averaged value + current X value
    I_LD(R0,R1,CUR_DIFF_VAL+ACCEL_DT_LEN),
    I_ADDR(R2,R2,R0),  //averaged value + current Y value
    I_LD(R0,R1,CUR_DIFF_VAL+ACCEL_DT_LEN+ACCEL_DT_LEN),
    I_ADDR(R2,R2,R0),  //averaged value + current Z value
    I_ST(R2,R3,AVR_DIFF_VAL),

    //check thresholds
    I_LD(R1,R3,AVR_EXCD_TH2),
    I_SUBR(R0,R2,R1),
    M_BXF(85),
    I_LD(R1,R3,AVR_EXCD_CT2),
    I_ADDI(R1,R1,1),
    I_ST(R1,R3,AVR_EXCD_CT2),
    I_LD(R0,R3,AVR_WAKE_TH2),
    I_SUBR(R1,R1,R0),
    M_BXF(85),
    I_WAKE(),
    M_BX(86),
  M_LABEL(85),
    I_LD(R1,R3,AVR_EXCD_TH1),
    I_SUBR(R0,R2,R1),
    M_BXF(86),
    I_LD(R1,R3,AVR_EXCD_CT1),
    I_ADDI(R1,R1,1),
    I_ST(R1,R3,AVR_EXCD_CT1),
    I_LD(R0,R3,AVR_WAKE_TH1),
    I_SUBR(R1,R1,R0),
    M_BXF(86),
    I_WAKE(),

    M_LABEL(86),

    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_0_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_0_RTC)
    I_WR_REG_BIT(RTC_IO_TOUCH_PAD1_REG, RTC_IO_TOUCH_PAD1_HOLD_S, 1), // HOLD on GPIO 0
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_2_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_2_RTC)
    I_WR_REG_BIT(RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_HOLD_S, 1), // HOLD on GPIO 2
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_4_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_4_RTC)
    I_WR_REG_BIT(RTC_IO_TOUCH_PAD0_REG, RTC_IO_TOUCH_PAD0_HOLD_S, 1), // HOLD on GPIO 4
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_12_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_12_RTC)
    I_WR_REG_BIT(RTC_IO_TOUCH_PAD5_REG, RTC_IO_TOUCH_PAD5_HOLD_S, 1), // HOLD on GPIO 12
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_13_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_13_RTC)
    I_WR_REG_BIT(RTC_IO_TOUCH_PAD4_REG, RTC_IO_TOUCH_PAD4_HOLD_S, 1), // HOLD on GPIO 13
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_14_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_14_RTC)
    I_WR_REG_BIT(RTC_IO_TOUCH_PAD6_REG, RTC_IO_TOUCH_PAD6_HOLD_S, 1), // HOLD on GPIO 14
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_15_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_15_RTC)
    I_WR_REG_BIT(RTC_IO_TOUCH_PAD3_REG, RTC_IO_TOUCH_PAD3_HOLD_S, 1), // HOLD on GPIO 15
    #endif
    #if (RTC_GPIO_BIT_SDA == GPIO_NUM_27_RTC) || (RTC_GPIO_BIT_SCL == GPIO_NUM_27_RTC)
    I_WR_REG_BIT(RTC_IO_TOUCH_PAD7_REG, RTC_IO_TOUCH_PAD7_HOLD_S, 1), // HOLD on GPIO 27
    #endif

    I_HALT()                                // HALT COPROCESSOR
  };
  _D(static_assert(sizeof(ulp_accel) <= (ACCEL_ULP_CODE_SIZE * 4), "ACCEL_ULP_CODE_SIZE TOO SMALL");)

  rtc_gpio_init(I2C_SDA);
  rtc_gpio_set_direction(I2C_SDA, RTC_GPIO_MODE_INPUT_OUTPUT);
  rtc_gpio_init(I2C_SCL);
  rtc_gpio_set_direction(I2C_SCL, RTC_GPIO_MODE_INPUT_OUTPUT);
#if ULP_LED_BLINK
  rtc_gpio_init(GPIO_NUM_2);
  rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_OUTPUT_ONLY);
#endif

  _D(DebugPrintln("init ULP code", DEBUG_LEVEL_1); delay(50);)
  //init variables for ULP
  RTC_SLOW_MEM[ACCEL_DATA_HEADER+ACCEL_MEAS_CNT] = 0;
  RTC_SLOW_MEM[ACCEL_DATA_HEADER+ACCEL_LOOP_CUR] = 0;
  RTC_SLOW_MEM[ACCEL_DATA_HEADER+ACCEL_DATA_CUR] = ACCEL_X_VALUES;
  RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EXCD_CT1] = 0;
  RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EXCD_TH1] = heidiConfig->accThres1;
  RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_WAKE_TH1] = _getAccThresCnt(heidiConfig->accAlertThres1);
  RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EXCD_CT2] = 0;
  RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EXCD_TH2] = heidiConfig->accThres2;
  RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_WAKE_TH2] = _getAccThresCnt(heidiConfig->accAlertThres2);
  RTC_SLOW_MEM[ACCEL_X_REGISTER] = ADXL345_DATAX0;
  RTC_SLOW_MEM[ACCEL_Y_REGISTER] = ADXL345_DATAY0;
  RTC_SLOW_MEM[ACCEL_Z_REGISTER] = ADXL345_DATAZ0;

  size_t size = sizeof(ulp_accel) / sizeof(ulp_insn_t);
  _D(DebugPrintln("ULP code length: " + String(sizeof(ulp_accel) / sizeof(ulp_insn_t)), DEBUG_LEVEL_1); delay(50);)
#if USE_MORE_THAN_128_INSN
  ulp_process_macros_and_load_big(0, ulp_accel, &size);
#else
  ulp_process_macros_and_load(0, ulp_accel, &size);
#endif
  ulp_run(0);

}
/* get / set measure count value */
uint16_t get_accel_meas_cnt_ULP(){  return (uint16_t)RTC_SLOW_MEM[ACCEL_DATA_HEADER+ACCEL_MEAS_CNT]; }
void set_accel_meas_cnt_ULP(uint16_t val){ RTC_SLOW_MEM[ACCEL_DATA_HEADER+ACCEL_MEAS_CNT] = val; }

/* get / set moving averaged sum value */
uint16_t get_accel_avrerage_ULP(){  return (uint16_t)RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_DIFF_VAL]; }
void set_accel_avrerage_ULP(uint16_t val){ RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_DIFF_VAL] = val; }

/* get / set threshold 1 exceed counter value */
uint16_t get_accel_excnt1_ULP(){ return (uint16_t)RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EXCD_CT1]; }
void set_accel_excnt1_ULP(uint16_t val){ RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EXCD_CT1] = val; }

/* get / set threshold 1 */
uint16_t get_accel_exthr1_ULP(){ return (uint16_t)RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EXCD_TH1]; }
void set_accel_exthr1_ULP(uint16_t val){ RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EXCD_TH1] = val; }

/* get / set wake value 1 */
uint16_t get_accel_wake1_ULP(){ return (uint16_t)RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_WAKE_TH1]; }
void set_accel_wake1_ULP(uint16_t val){ RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_WAKE_TH1] = val; }

/* get / set threshold 2 exceed counter value */
uint16_t get_accel_excnt2_ULP(){ return (uint16_t)RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EXCD_CT2]; }
void set_accel_excnt2_ULP(uint16_t val){ RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EXCD_CT2] = val; }

/* get / set threshold 2 */
uint16_t get_accel_exthr2_ULP(){ return (int16_t)RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EXCD_TH2]; }
void set_accel_exthr2_ULP(uint16_t val){ RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EXCD_TH2] = val; }

/* get / set wake value 2 */
uint16_t get_accel_wake2_ULP(){ return (uint16_t)RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_WAKE_TH2]; }
void set_accel_wake2_ULP(uint16_t val){ RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_WAKE_TH2] = val; }

uint16_t _getAccThresCnt(uint16_t dayThres){
  if (_night()){
    return (uint16_t)((uint32_t)(dayThres * heidiConfig->accNightFactor) / 100);
  }
  return dayThres;
}
#if USE_MORE_THAN_128_INSN
/*
 *  need to use a copy of ulp_process_macros_and_load to avoid code too big error - which seems useless
 *
 */

/* Comparison function used to sort the relocations array */
static int reloc_sort_func(const void* p_lhs, const void* p_rhs)
{
    const reloc_info_t lhs = *(const reloc_info_t*) p_lhs;
    const reloc_info_t rhs = *(const reloc_info_t*) p_rhs;
    if (lhs.label < rhs.label) {
        return -1;
    } else if (lhs.label > rhs.label) {
        return 1;
    }
    // label numbers are equal
    if (lhs.type < rhs.type) {
        return -1;
    } else if (lhs.type > rhs.type) {
        return 1;
    }

    // both label number and type are equal
    return 0;
}

static esp_err_t do_single_reloc(ulp_insn_t* program, uint32_t load_addr,
        reloc_info_t label_info, reloc_info_t the_reloc)
{
    size_t insn_offset = the_reloc.addr - load_addr;
    ulp_insn_t* insn = &program[insn_offset];

    switch (the_reloc.type) {
        case RELOC_TYPE_BRANCH: {
            // B, BS and BX have the same layout of opcode/sub_opcode fields,
            // and share the same opcode. B and BS also have the same layout of
            // offset and sign fields.
            assert(insn->b.opcode == OPCODE_BRANCH
                    && "branch macro was applied to a non-branch instruction");
            switch (insn->b.sub_opcode) {
                case SUB_OPCODE_B:
                case SUB_OPCODE_BS:{
                    int32_t offset = ((int32_t) label_info.addr) - ((int32_t) the_reloc.addr);
                    uint32_t abs_offset = abs(offset);
                    uint32_t sign = (offset >= 0) ? 0 : 1;
                    if (abs_offset > 127) {
                        ESP_LOGW(TAG, "target out of range: branch from %x to %x",
                                the_reloc.addr, label_info.addr);
                        return ESP_ERR_ULP_BRANCH_OUT_OF_RANGE;
                    }
                    insn->b.offset = abs_offset; //== insn->bs.offset = abs_offset;
                    insn->b.sign = sign;         //== insn->bs.sign = sign;
                    break;
                }
                case SUB_OPCODE_BX:{
                    assert(insn->bx.reg == 0 &&
                            "relocation applied to a jump with offset in register");
                    insn->bx.addr = label_info.addr;
                    break;
                }
                default:
                    assert(false && "unexpected branch sub-opcode");
            }
            break;
        }
        case RELOC_TYPE_LABELPC: {
            assert((insn->alu_imm.opcode == OPCODE_ALU && insn->alu_imm.sub_opcode == SUB_OPCODE_ALU_IMM && insn->alu_imm.sel == ALU_SEL_MOV)
                        && "pc macro was applied to an incompatible instruction");
            insn->alu_imm.imm = label_info.addr;
            break;
        }
        default:
            assert(false && "unknown reloc type");
    }
    return ESP_OK;
}


esp_err_t ulp_process_macros_and_load_big(uint32_t load_addr, const ulp_insn_t* program, size_t* psize)
{
    const ulp_insn_t* read_ptr = program;
    const ulp_insn_t* end = program + *psize;
    size_t macro_count = 0;
    // step 1: calculate number of macros
    while (read_ptr < end) {
        ulp_insn_t r_insn = *read_ptr;
        if (r_insn.macro.opcode == OPCODE_MACRO) {
            ++macro_count;
        }
        ++read_ptr;
    }
    size_t real_program_size = *psize - macro_count;
    const size_t ulp_mem_end = ULP_RESERVE_MEM / sizeof(ulp_insn_t);
    if (load_addr > ulp_mem_end) {
        ESP_LOGW(TAG, "invalid load address %x, max is %x",
                load_addr, ulp_mem_end);
        return ESP_ERR_ULP_INVALID_LOAD_ADDR;
    }
    if (real_program_size + load_addr > ulp_mem_end) {
        ESP_LOGE(TAG, "program too big: %d words, max is %d words",
                real_program_size, ulp_mem_end);
        return ESP_ERR_ULP_SIZE_TOO_BIG;
    }
    // If no macros found, copy the program and return.
    if (macro_count == 0) {
        memcpy(((ulp_insn_t*) RTC_SLOW_MEM) + load_addr, program, *psize * sizeof(ulp_insn_t));
        return ESP_OK;
    }
    reloc_info_t* reloc_info =
            (reloc_info_t*) malloc(sizeof(reloc_info_t) * macro_count);
    if (reloc_info == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // step 2: record macros into reloc_info array
    // and remove them from then program
    read_ptr = program;
    ulp_insn_t* output_program = ((ulp_insn_t*) RTC_SLOW_MEM) + load_addr;
    ulp_insn_t* write_ptr = output_program;
    uint32_t cur_insn_addr = load_addr;
    reloc_info_t* cur_reloc = reloc_info;
    while (read_ptr < end) {
        ulp_insn_t r_insn = *read_ptr;
        if (r_insn.macro.opcode == OPCODE_MACRO) {
            switch (r_insn.macro.sub_opcode) {
                case SUB_OPCODE_MACRO_LABEL:
                    *cur_reloc = RELOC_INFO_LABEL(r_insn.macro.label,cur_insn_addr);
                    break;
                case SUB_OPCODE_MACRO_BRANCH:
                    *cur_reloc = RELOC_INFO_BRANCH(r_insn.macro.label,cur_insn_addr);
                    break;
                case SUB_OPCODE_MACRO_LABELPC:
                    *cur_reloc = RELOC_INFO_LABELPC(r_insn.macro.label,cur_insn_addr);
                    break;
                default:
                    assert(0 && "invalid sub_opcode for macro insn");
            }
            ++read_ptr;
            assert(read_ptr != end && "program can not end with macro insn");
            ++cur_reloc;
        } else {
            // normal instruction (not a macro)
            *write_ptr = *read_ptr;
            ++read_ptr;
            ++write_ptr;
            ++cur_insn_addr;
        }
    }

    // step 3: sort relocations array
    qsort(reloc_info, macro_count, sizeof(reloc_info_t),
            reloc_sort_func);

    // step 4: walk relocations array and fix instructions
    reloc_info_t* reloc_end = reloc_info + macro_count;
    cur_reloc = reloc_info;
    while(cur_reloc < reloc_end) {
        reloc_info_t label_info = *cur_reloc;
        assert(label_info.type == RELOC_TYPE_LABEL);
        ++cur_reloc;
        while (cur_reloc < reloc_end) {
            if (cur_reloc->type == RELOC_TYPE_LABEL) {
                if(cur_reloc->label == label_info.label) {
                    ESP_LOGE(TAG, "duplicate label definition: %d",
                            label_info.label);
                    free(reloc_info);
                    return ESP_ERR_ULP_DUPLICATE_LABEL;
                }
                break;
            }
            if (cur_reloc->label != label_info.label) {
                ESP_LOGE(TAG, "branch to an inexistent label: %d",
                        cur_reloc->label);
                free(reloc_info);
                return ESP_ERR_ULP_UNDEFINED_LABEL;
            }
            esp_err_t rc = do_single_reloc(output_program, load_addr,
                    label_info, *cur_reloc);
            if (rc != ESP_OK) {
                free(reloc_info);
                return rc;
            }
            ++cur_reloc;
        }
    }
    free(reloc_info);
    *psize = real_program_size;
    return ESP_OK;
}
#endif //USE_MORE_THAN_128_INSN
#endif //USE_ULP
#endif //ACCELEROMETER


