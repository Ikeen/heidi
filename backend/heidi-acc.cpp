/*
 * TestAccel.cpp
 *
 *  Created on: 14.12.2020
 *      Author: frank
 *
 */



#include <Arduino.h>
#include <Wire.h>
#include <assert.h>
#include <esp32/ulp.h>
#include <LoRa.h>
#include <driver/rtc_io.h>
#include "heidi-debug.h"
#include "heidi-measures.h"
#include "heidi-data.h"
#include "heidi-sys.h"
#include "heidi-acc.h"

#ifdef ACCELEROMETER

bool _ADXL345_status = false;
bool _ADXL345_sleep  = false;
bool _ADXL345_avail  = false;

bool init_ADXL345(){
  byte _ID;
  //getIIC is done in the init-function above
  if (!gotULPlock()) { return false; }
  _D(DebugPrint("Init ADXL345.. ", DEBUG_LEVEL_2));
  //check sensor
  if(iic_readRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_DEVID, &_ID) != I2C_ERROR_OK){
    _D(DebugPrintln("ADXL345 read ID register failed." , DEBUG_LEVEL_1));
    return false;
  }
  if (_ID != 0xE5){
    _D(DebugPrintln("ADXL345 not found, read 0x" + String(_ID, HEX) + " instead 0xe5." , DEBUG_LEVEL_1));
    return false;
  }
  _D(DebugPrintln("successful", DEBUG_LEVEL_2));
  _ADXL345_avail = true;
  return true;
}

bool wake_config_ADXL345(void){
  if (!_ADXL345_avail) { return false; }
  if (!getIIC()) { return false; } //do  not combine these 2 if's - compiler optimizations will cut second condition if first is true
  _D(DebugPrint("Wake ADXL345.. ", DEBUG_LEVEL_2);)
  // wake up sensor
  if(iic_writeRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_POWER_CTL, 0x08) != I2C_ERROR_OK) { setError(E_IIC_ERROR); _D(DebugPrintln(" _ADXL345_wake up failed", DEBUG_LEVEL_1);) return false; }
  _ADXL345_status = true;
  // Set the range to whatever is appropriate for your project
  if (!_ADXL345_setRange(ADXL345_RANGE_16_G)) { setError(E_IIC_ERROR); _D(DebugPrint (" (_ADXL345_setRange failed) ", DEBUG_LEVEL_1);)}
  if (!_ADXL345_setDataRate(ADXL345_DATARATE_12_5_HZ)) { setError(E_IIC_ERROR); _D(DebugPrint (" (_ADXL345_setDataRate failed) ", DEBUG_LEVEL_1);)}
  if (!_ADXL345_setFullResBit(false)) { setError(E_IIC_ERROR); _D(DebugPrint (" (_ADXL345_setDataRate failed) ", DEBUG_LEVEL_1);) }
  _D(if (getError(E_IIC_ERROR)) { DebugPrintln("failed", DEBUG_LEVEL_2); } else { DebugPrintln("successful", DEBUG_LEVEL_2);})
  freeIIC();
  return true;
}

bool sleep_ADXL345(){
  // knock out sensor
  if (!_ADXL345_avail) { return false; }
  if (!getIIC()) { return false; } //do  not combine these 2 if's - compiler optimizations will cut second condition if first is true
  if(iic_writeRegister(ADXL345_DEFAULT_ADDRESS, ADXL345_POWER_CTL, 0x00) != I2C_ERROR_OK) { setError(E_IIC_ERROR); return false; }
  _ADXL345_status = false;
  #ifndef I2C_SWITCH
  _ADXL345_sleep = true;
  pinMode(I2C_SDA, INPUT_PULLDOWN);
  pinMode(I2C_SCL, INPUT_PULLDOWN);
  #endif
  freeIIC();
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

  //check if iic is available / or configuration is accessed by main cpu
  I_MOVI(R3,IIC_STATUS),           //load data structure offset
  I_LD(R0,R3,IIC_REQUESTED),       //load request value
  M_BL(1,1),                       //access requested by main CPU?
  I_HALT(),                        //then end preocedure and halt ULP until
  M_LABEL(1),
  I_MOVI(R0,1),                    // set R0 to true
  I_ST(R0,R3,IIC_LOCKED),          // store it to IIC_LOCKED

#ifdef ULP_LED_BLINK
  //LED flashing in debug mode for check "ULP is running"
  I_WR_REG_BIT(RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_HOLD_S, 0), // HOLD off GPIO 2
  I_MOVI(R3,DEBUG_LED_MEM),               // #mem -> R3
  I_LD(R0,R3,0),                          // R0 = mem]
  I_ADDI(R0,R0,1),
  M_BGE(91,2),                            // on?
  I_WR_REG_BIT(RTC_GPIO_OUT_W1TC_REG,RTC_GPIO_BIT_LED,1),
  M_BX(92),
  M_LABEL(91),
  I_WR_REG_BIT(RTC_GPIO_OUT_W1TS_REG,RTC_GPIO_BIT_LED,1),
  M_BL(92,3),                             // off next turn?
  I_MOVI(R0,0),
  M_LABEL(92),
  I_ST(R0,R3,0),                          // mem = R0
  I_WR_REG_BIT(RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_HOLD_S, 1), // HOLD on LED
#endif

    //setup GPIOs
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

    /*
     * initialize i²c bus
     * we do not drive GPIOs actively high and low but switch from mode 'input' (=HIGH) to mode 'driven' (=LOW) instead.
     */
    SDA_INPUT,                     //(=HIGHZ)
    SCL_INPUT,                     //(=HIGHZ)
    SDA_DRIVE_L,                   //set driven state of GPIO to 'low'
    SCL_DRIVE_L,                   //set driven state of GPIO to 'low'
    /*
     * Both GPIOs are 'high' now for a couple of microseconds during the following instructions.
     * This is needed for a valid i²c start-condition, following up.
     */

  M_LABEL(2),                      //loop for reading 3 registers

    I_MOVI(R0,I2C_FAILED),         //set status to failed
    I_MOVI(R2,ACCEL_DATA_HEADER),  //load offset data sets start address
    I_LD(R2,R2,ACCEL_DATA_CUR),    //load offset to current data structure
    I_ST(R0,R2,I2C_TRNS_RES),      //store "failed" status to transfer result
    I_LD(R0,R2,CUR_READ_RES),      //load last read result (still stored in 'current')
    I_ST(R0,R2,LST_READ_RES),      //move it to 'last'
    I_MOVI(R3,2),                  //prepare writing 2 bytes + 1 more (see below)
    /*
     * 1st: transfer device Address with R/W-bit = 'write' - we want to write the register address to device
     */
    I_MOVI(R2,ADXL345_DEFAULT_ADDRESS),   //load device address (7 bits)
    I_LSHI(R2,R2,2),               // shift 1 more because we want use same value for shift and for loop counting
                                   // so we start shifting with '8' and end with '1', means: we use bit 1 to 8, but not bit 0
    //I_ORI(R2,R2,0x00),           // R/W - bit, write = 0 -> 'or' - operation would be senseless, but still noted here for understanding i²c protocol
                                   // keep in mind: R/W - bit 'read' = 1 would mean 'OR 0x02' bit 0 is not used

  M_LABEL(3),                      //start a new transfer, start condition = 1st drive SDA low, afterwards SCL
    SDA_L,                         //SDA = low (1.25us per I_WR_REG_BIT)
    SCL_L,                         //SCL = low

  M_LABEL(4),                      //write byte in R2
    I_MOVI(R0, 0x08),              //load loop count to R0 (need to write 8 bits -> device address + R/W)
  M_LABEL(5),
    I_RSHR(R1,R2,R0),              //shift current bit into LSB position
    I_ANDI(R1,R1,0x01),            //mask it
    M_BXZ(6),                      // '1' or '0'?
    SDA_H,                         // '1'
    M_BX(7),
    M_LABEL(6),
    SDA_L,                         // '0'
  M_LABEL(7),
    SCL_H,                         // trigger SCL
    SCL_L,
    I_SUBI(R0,R0,1),               //decrement counter
    M_BGE(5,1),                    //loop while not '0'
    SDA_INPUT,                     //switch SDA to input
    SCL_H,                         //trigger SCL
    SDA_READ,                      //read ACK
    SCL_L,
    M_BGE(15,1),                   //break if ACK = '1' (means failed)
    I_SUBI(R3,R3,1),               //decrement byte loop counter
    M_BXZ(8),                      //if R3 == 0, 2 bytes are written, we need do write 'read'-request to device
    M_BXF(9),                      //if R3 == -1, we are through, goto read
                                   //otherwise prepare writing the register to be read out
    I_MOVI(R2,ACCEL_DATA_HEADER),  //load offset data sets start address
    I_LD(R2,R2,ACCEL_DATA_CUR),    //load offset to current data set
    I_LD(R2,R2,I2C_DATA_REG),      //current register address
    I_LSHI(R2,R2,1),               //shift 1 more (see above)
    M_BX(4),                       //next turn
  M_LABEL(8),                      //prepare 'read' request
    SCL_H,                         //prepare restart condition + a little delay done by the following instructions
    I_MOVI(R2,ADXL345_DEFAULT_ADDRESS),   //load device address
    I_LSHI(R2,R2,2),               //shift 1 more (see above)
    I_ORI(R2,R2,0x02),             //R/W - bit, read = 1;
    M_BX(3),                       //start a read transfer on device, ...
  M_LABEL(9),
                                   //... but now we end up here after starting read transfer
    I_MOVI(R3,2),                  //now we read 2 bytes
    SDA_INPUT,                     //this should already be done by reading last ACK .. we want to be sure
    I_MOVI(R1,0),                  //clear R1
  M_LABEL(11),
    I_MOVI(R2, 0x08),              //load loop counter
  M_LABEL(12),
    SCL_H,                         //trigger SCL
    SDA_READ,                      //read 1 bit
    SCL_L,
    I_LSHI(R1,R1,1),               //shift R1
    I_ORR(R1,R1,R0),               //add new bit
    I_SUBI(R2,R2,1),               //decrement counter
    M_BXZ(13),                     //if R2 == 0, we are through
    M_BX(12),                      //otherwise next read bit
  M_LABEL(13),
    I_SUBI(R3,R3,1),               //decrement byte counter
    M_BXZ(14),                     //if byte-counter == 0  we are trough
    SDA_L,                         //ACK fromMaser
    SCL_H,                         //trigger it
    I_DELAY(2),                    //give a little delay
    SCL_L,
    SDA_H,                         //SDA_H = SDA_INPUT
    I_DELAY(2),                    //give a little delay
    M_BX(11),                      //read next byte

  M_LABEL(14),
    SDA_H,                         //NACK from master (want no more bytes)
    SCL_H,                         //trigger it
    I_DELAY(2),                    //give a little delay
    SCL_L,
    I_RSHI(R2,R1,8),               //swap low- and high-byte in R1
    I_LSHI(R1,R1,8),
    I_ORR(R1,R1,R2),
    I_MOVI(R2,ACCEL_DATA_HEADER),  //load offset data sets start address
    I_LD(R2,R2,ACCEL_DATA_CUR),    //load offset to current data set
    I_ST(R1,R2,CUR_READ_RES),      //store read value
    I_MOVI(R0,I2C_SUCCESS),        //load transfer success value
    I_ST(R0,R2,I2C_TRNS_RES),      //store it
  M_LABEL(15),                     //generate a stop condition
    SDA_L,
    I_DELAY(10),                   //give a little delay
    SCL_H,
    I_DELAY(6),
    SDA_H,
    /*
     * now calculations
     * R0 still holds transmission result,
     * if transmission was successful: R1 holds current acceleration value
     */
    M_BGE(16,1),                   //no calculation if transmission fails
    I_MOVI(R3,ACCEL_DATA_HEADER),  //load data sets start address
    I_LD(R3,R3,ACCEL_DATA_CUR),    //load offset to current data set (x=0,y=size,z=2*size)
    I_LD(R2,R3,LST_READ_RES),      //load last measured value
    I_SUBR(R0,R2,R1),              //calculate difference
    I_ANDI(R3,R0,0x8000),          //negative ?
    M_BXZ(17),
    I_SUBR(R0,R1,R2),              //yes - then the opposite value
    M_BX(17),                      //now we have the absolute value of difference - goto storing it
  M_LABEL(16),
    I_MOVI(R0,0),                  //transmission failed - set difference value to 0
  M_LABEL(17),
    I_MOVI(R3,ACCEL_DATA_HEADER),  //load data sets start address
    I_LD(R3,R3,ACCEL_DATA_CUR),    //load offset to current data set
    I_ST(R0,R3,CUR_DIFF_VAL),      //store the absolute value of difference

    I_MOVI(R3,ACCEL_DATA_HEADER),  //load data sets start address
    I_LD(R0,R3,ACCEL_LOOP_CUR),    //load offset to loop counter
    I_ADDI(R0,R0,1),               //increment
    M_BGE(18,3),                   //3 loops? if yes - we're ready
    I_ST(R0,R3,ACCEL_LOOP_CUR),    //else store loop counter
    I_LD(R0,R3,ACCEL_DATA_CUR),    //load current data set offset
    I_ADDI(R0,R0,ACCEL_DT_LEN),    //increment to next data set
    I_ST(R0,R3,ACCEL_DATA_CUR),    //store it
    M_BX(2),                       //loop to next read next register

    //end of transmissions
  M_LABEL(18),                     //now we have 3 difference values for each axis (x,y,z)
                                   //if transmission was failed for 1 value, we take the previous data
    I_MOVI(R3,IIC_STATUS),         //release access lock
    I_MOVI(R0,0),                  // ...
    I_ST(R0,R3,IIC_LOCKED),        // set IIC_LOCKED to false

    I_MOVI(R3,ACCEL_DATA_HEADER),  //load current data set offset

    I_MOVI(R0,0),                  //zero R0
    I_ST(R0,R3,ACCEL_LOOP_CUR),    //reset loop counter to zero
    I_MOVI(R0,ACCEL_X_VALUES),     //load offset to 1st data set (x)
    I_ST(R0,R3,ACCEL_DATA_CUR),    //reset data set pointer to 1st one

    /* calculate averaged value
     * this is a moving average calculated by new_avg = ((old_avg * 31) / 32) + diff_x + diff_y + diff_z
     */
    I_LD(R1,R3,AVR_DIFF_VAL),      //load old_avg
    I_LSHI(R2,R1,4),               //old_avg * 32
    I_SUBR(R2,R2,R1),              //(old_avg * 32) - old_avg [== old_avg * 31]
    I_RSHI(R2,R2,4),               //(old_avg * 31) / 32

    I_MOVI(R1,ACCEL_X_VALUES),     //load offset to 1st data set (x)
    I_LD(R0,R1,CUR_DIFF_VAL),      //load diff_x
    I_ADDR(R2,R2,R0),              //((old_avg * 31) / 32) + diff_x
    I_LD(R0,R1,CUR_DIFF_VAL+ACCEL_DT_LEN), //load diff_y
    I_ADDR(R2,R2,R0),              //((old_avg * 31) / 32) + diff_x+ diff_y
    I_LD(R0,R1,CUR_DIFF_VAL+ACCEL_DT_LEN+ACCEL_DT_LEN), //load diff_z
    I_ADDR(R2,R2,R0),              //((old_avg * 31) / 32) + diff_x + diff_z
    I_ST(R2,R3,AVR_DIFF_VAL),      //store new_avg

    //check thresholds
    I_LD(R1,R3,AVR_EXCD_TH2),      //load threshold 2 (the higher one) for counter 2
    I_SUBR(R0,R2,R1),              //check averaged value against threshold 2
    M_BXF(19),                     //if less, goto next
    I_LD(R1,R3,AVR_EXCD_CT2),      //otherwise load counter 2
    I_ADDI(R1,R1,1),               //increment
    I_ST(R1,R3,AVR_EXCD_CT2),      //store
    I_LD(R0,R3,AVR_WAKE_TH2),      //load wake threshold 2
    I_SUBR(R1,R1,R0),              //check averaged value against wake threshold 2
    M_BXF(19),                     //if less, goto next
    I_WAKE(),                      //otherwise alert - wake main CPU..
    M_BX(20),                      //..and goto end
  M_LABEL(19),
    I_LD(R1,R3,AVR_EXCD_TH1),      //load threshold 1 (the lower one) for counter 1
    I_SUBR(R0,R2,R1),              //check averaged value against threshold 1
    M_BXF(20),                     //if less, goto end
    I_LD(R1,R3,AVR_EXCD_CT1),      //otherwise load counter 1
    I_ADDI(R1,R1,1),               //increment
    I_ST(R1,R3,AVR_EXCD_CT1),      //store
    I_LD(R0,R3,AVR_WAKE_TH1),      //load wake threshold 1
    I_SUBR(R1,R1,R0),              //check averaged value against wake threshold 1
    M_BXF(20),                     //if less, goto end
    I_WAKE(),                      //otherwise alert - wake main CPU
    /*
     * now we have to take care for the interim results. Data header offset still resides in R3
     */
  M_LABEL(20),
    I_LD(R0,R3,ACCEL_MEAS_CNT),    //load measurement counter
    I_ADDI(R0,R0,1),               //increment it
    I_ST(R0,R3,ACCEL_MEAS_CNT),    //store it

    I_LD(R1,R3,AVR_INTERRIM_1),    //load threshold for 1st interim counter threshold[0]
    I_SUBR(R2,R1,R0),              //check against current counter (threshold - current < 0?)
    M_BXF(21),                     //current is above threshold - go ahead
    I_LD(R1,R3,AVR_EXCD_CT1),      //load exceed-th1-counter
    I_ST(R1,R3,AVR_EX_IR_CT1_1),   //copy it to interim 1 data set
    I_LD(R1,R3,AVR_EXCD_CT2),      //load exceed-th2-counter
    I_ST(R1,R3,AVR_EX_IR_CT2_1),   //copy it to interim 1 data set
    M_BX(23),                      //goto end
  M_LABEL(21),                     //current > threshold[0]
    I_LD(R1,R3,AVR_INTERRIM_2),    //load threshold for 2nd interim counter threshold[1]
    I_SUBR(R2,R1,R0),              //check against current counter (threshold - current < 0?)
    M_BXF(22),                     //current is above threshold - go ahead
    I_LD(R1,R3,AVR_EXCD_CT1),      //load exceed-th1-counter
    I_ST(R1,R3,AVR_EX_IR_CT1_2),   //copy it to interim 2 data set
    I_LD(R1,R3,AVR_EXCD_CT2),      //load exceed-th2-counter
    I_ST(R1,R3,AVR_EX_IR_CT2_2),   //copy it to interim 2 data set
    M_BX(23),                      //goto end
  M_LABEL(22),                     //current > threshold[1]
    I_LD(R1,R3,AVR_INTERRIM_3),    //load threshold for 3rd interim counter threshold[2]
    I_SUBR(R2,R1,R0),              //check against current counter (threshold - current < 0?)
    M_BXF(23),                     //current is above threshold - go ahead
    I_LD(R1,R3,AVR_EXCD_CT1),      //load exceed-th1-counter
    I_ST(R1,R3,AVR_EX_IR_CT1_3),   //copy it to interim 2 data set
    I_LD(R1,R3,AVR_EXCD_CT2),      //load exceed-th2-counter
    I_ST(R1,R3,AVR_EX_IR_CT2_3),   //copy it to interim 2 data set
  M_LABEL(23),                     //current > threshold[3] - means: do nothing

  //end procedure
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
  static_assert(sizeof(ulp_accel) <= (ACCEL_ULP_CODE_SIZE * 4), "ACCEL_ULP_CODE_SIZE TOO SMALL");
  getIIC();
  rtc_gpio_init(I2C_SDA);
  rtc_gpio_set_direction(I2C_SDA, RTC_GPIO_MODE_INPUT_OUTPUT);
  rtc_gpio_init(I2C_SCL);
  rtc_gpio_set_direction(I2C_SCL, RTC_GPIO_MODE_INPUT_OUTPUT);
#ifdef ULP_LED_BLINK
  rtc_gpio_init(GPIO_NUM_2);
  rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_OUTPUT_ONLY);
#endif

  _D(DebugPrintln("initialize ULP code", DEBUG_LEVEL_1); pause(50);)
  //initialize static variables for ULP
  RTC_SLOW_MEM[ACCEL_DATA_HEADER+ACCEL_LOOP_CUR] = 0;
  RTC_SLOW_MEM[ACCEL_DATA_HEADER+ACCEL_DATA_CUR] = ACCEL_X_VALUES;
  RTC_SLOW_MEM[ACCEL_X_REGISTER] = ADXL345_DATAX0;
  RTC_SLOW_MEM[ACCEL_Y_REGISTER] = ADXL345_DATAY0;
  RTC_SLOW_MEM[ACCEL_Z_REGISTER] = ADXL345_DATAZ0;
  RTC_SLOW_MEM[IIC_STATUS + IIC_REQUESTED] = 0;
  RTC_SLOW_MEM[IIC_STATUS + IIC_LOCKED]    = 0;

  init_accel_data_ULP(intervall_us, _currentCyleLen_m());

  size_t size = sizeof(ulp_accel) / sizeof(ulp_insn_t);
  _D(DebugPrintln("ULP code length: " + String(sizeof(ulp_accel) / sizeof(ulp_insn_t)), DEBUG_LEVEL_1); pause(50);)
#if USE_MORE_THAN_128_INSN
  ulp_process_macros_and_load_big(0, ulp_accel, &size);
#else
  ulp_process_macros_and_load(0, ulp_accel, &size);
#endif
  setState(ULP_RUNNING);
  ulp_run(0);

}
void init_accel_data_ULP(uint32_t intervall_us, uint8_t cycleLenMinute){
  //initialize measurement cycle variables for ULP
  if (getULPLock()) {
    uint16_t interim_ticks = (uint16_t)((uint32_t)(cycleLenMinute * 60000) / (intervall_us / 1000) / 4);
    set_accel_exthr1_ULP(heidiConfig->c.accThres1); //set threshold 1 to current value
    set_accel_exthr2_ULP(heidiConfig->c.accThres2); //set threshold 2 to current value
    set_accel_excnt1_ULP(0);   //set threshold 1 exceeding counter to zero
    set_accel_excnt2_ULP(0);   //set threshold 1 exceeding counter to zero
    set_accel_wake1_ULP(_getAccThresCnt(heidiConfig->c.accAlertThres1)); //set wake (alert) threshold 1 to current value
    set_accel_wake2_ULP(_getAccThresCnt(heidiConfig->c.accAlertThres2)); //set wake (alert) threshold 2 to current value
    set_accel_meas_cnt_ULP(0); //set measurement counter to zero
    for (int i=0; i<3; i++){
      set_accel_interrim_ct1_ULP(i,0);
      set_accel_interrim_ct2_ULP(i,0);
      set_accel_interrim_thr_ULP(i,interim_ticks*(i+1));
    }
  }
  freeULP();
}
#ifdef TEST_ACC
void testAcc(bool poweOnReset){
  if(poweOnReset){
    wake_config_ADXL345();
    init_accel_ULP(ULP_INTERVALL_US);
    DebugPrintln("ACC: init ULP", DEBUG_LEVEL_1);
  } else {
    DebugPrintln("ACC: measurement count: " + String(get_accel_meas_cnt_ULP()), DEBUG_LEVEL_1);
    DebugPrintln("ACC: transmission result x: " + String((uint8_t)RTC_SLOW_MEM[ACCEL_X_VALUES+I2C_TRNS_RES]), DEBUG_LEVEL_1);
    DebugPrintln("ACC: transmission result y: " + String((uint8_t)RTC_SLOW_MEM[ACCEL_X_VALUES+I2C_TRNS_RES]), DEBUG_LEVEL_1);
    DebugPrintln("ACC: transmission result z: " + String((uint8_t)RTC_SLOW_MEM[ACCEL_X_VALUES+I2C_TRNS_RES]), DEBUG_LEVEL_1);
    for(int i=0; i<4; i++){
      DebugPrintln("ACC: cnt1/" + String(i+1) + " :" + String(get_accel_interrim_ct_ULP(1,i)), DEBUG_LEVEL_1);
      DebugPrintln("ACC: cnt2/" + String(i+1) + " :" + String(get_accel_interrim_ct_ULP(2,i)), DEBUG_LEVEL_1);
    }
    init_accel_data_ULP(ULP_INTERVALL_US, 1);
  }
  DebugPrintln("ACC: sleep 58 seconds for measuring", DEBUG_LEVEL_1);
  pause(58000);
}
#endif

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

/* get / set interim threshold values */
uint16_t get_accel_interrim_thr_ULP(int which){
  if ((which < 3) && (which >= 0)) { return (uint16_t)(RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_INTERRIM+which]); }
  _D( else { DebugPrintln("wrong index!", DEBUG_LEVEL_1);})
  return 0;
}
void set_accel_interrim_thr_ULP(int which, uint16_t val){
  if ((which < 3) && (which >= 0)) { RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_INTERRIM+which] = val; }
  _D( else { DebugPrintln("wrong index!", DEBUG_LEVEL_1);})
}

/* get / set interim counter 1 values */
uint16_t get_accel_interrim_ct1_ULP(int which){
  if ((which < 3) && (which >= 0)) { return (uint16_t)(RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EX_IR_CT1+which]); }
  else if (which == 3) { return get_accel_excnt1_ULP(); }
  _D( else { DebugPrintln("wrong counter index!", DEBUG_LEVEL_1);})
  return 0;
}
void set_accel_interrim_ct1_ULP(int which, uint16_t val){
  if ((which < 3) && (which >= 0)) {RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EX_IR_CT1+which] = val; }
  else if (which == 3) { set_accel_excnt1_ULP(val); }
  _D( else { DebugPrintln("wrong counter index!", DEBUG_LEVEL_1);})
}

/* get / set interim counter 2 values */
uint16_t get_accel_interrim_ct2_ULP(int which){
  if ((which < 3) && (which >= 0)) { return (uint16_t)(RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EX_IR_CT2+which]); }
  else if (which == 3) { return get_accel_excnt2_ULP(); }
  _D( else { DebugPrintln("wrong counter index!", DEBUG_LEVEL_1);})
  return 0;
}
void set_accel_interrim_ct2_ULP(int which, uint16_t val){
  if ((which < 3) && (which >= 0)) { RTC_SLOW_MEM[ACCEL_DATA_HEADER+AVR_EX_IR_CT2+which] = val; }
  else if (which == 3) { set_accel_excnt2_ULP(val); }
  _D( else { DebugPrintln("wrong counter index!", DEBUG_LEVEL_1);})
}
uint16_t get_accel_interrim_ct_ULP(int which_ct, int which_val){
  if(which_ct == 1) { return get_accel_interrim_ct1_ULP(which_val);}
  if(which_ct == 2) { return get_accel_interrim_ct2_ULP(which_val);}
  return 0xffff;
}
void set_accel_interrim_ct_ULP(int which_ct, int which_val, uint16_t value){
  if(which_ct == 1) { set_accel_interrim_ct1_ULP(which_val, value);}
  if(which_ct == 2) { set_accel_interrim_ct2_ULP(which_val, value);}
}
/*
 * If collecting interim acceleration values is used, this function calculates a 4x4bit representing
 * of distribution of measured data. The measuring time is divided into 4 periods and each nibble of
 * returned value represents the share of one period (1st period = highest nibble).
 * The sum of all nibbles = 100%
 * which = 1 or 2 -> ct1 or ct2
 */
uint16_t get_accel_ct_spreading(int which){
  uint16_t s_value[4];
  uint8_t  n_value;
  uint16_t max_val, result = 0;
  s_value[0] = get_accel_interrim_ct_ULP(which, 0);
  max_val = s_value[0];
  for(int i=1; i<4; i++){
    s_value[i] =  get_accel_interrim_ct_ULP(which, i) - get_accel_interrim_ct_ULP(which, i-1);
    if (s_value[i] > max_val){ max_val = s_value[i]; }
  }
  if (max_val> 0){
    for(int i=0; i<4; i++){
      result = result << 4;
      n_value = (uint8_t)lround(((float)s_value[i] * 0x0f) / (float)max_val);
      result |= (n_value & 0x0f);
    }
  }
  return result;
}


uint16_t _getAccThresCnt(uint16_t dayThres){
  if (_night()){
    return (uint16_t)((uint32_t)(dayThres * heidiConfig->c.accNightFactor) / 100);
  }
  return dayThres;
}

void set_ULP_request(uint16_t value){
  RTC_SLOW_MEM[IIC_STATUS + IIC_REQUESTED] = value;
}

void set_ULP_lock(uint16_t value){
  RTC_SLOW_MEM[IIC_STATUS + IIC_LOCKED]    = value;
}

bool ULPisLocked(void){
  return ((uint16_t)RTC_SLOW_MEM[IIC_STATUS + IIC_LOCKED] != 0);
}

bool gotULPlock(void){
  return ((uint16_t)RTC_SLOW_MEM[IIC_STATUS + IIC_REQUESTED] != 0);
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


