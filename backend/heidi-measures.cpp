/*
 * heidi-measures.cpp
 *
 *  Created on: 08.07.2020
 *      Author: frank
 */
#include <Arduino.h>
#include <assert.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include "heidi-measures.h"
#include "heidi-acc.h"
#include "heidi-gsm.h"
#ifdef TEMP_SENSOR
#include "OneWire.h"
#include "DallasTemperature.h"
#endif

int  running_measures = 0;
bool MEASenabled   = false;
bool CTLenabled    = false;

#ifdef TEMP_SENSOR
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);
#endif

bool enableControls(void){
  bool success = true;
  if(!CTLenabled){
    #ifdef I2C_BUS
    if (getIIC()){
      #ifdef ACCELEROMETER
      if (!init_ADXL345()){
        setError(E_IIC_ERROR);
      }
      #endif
    } else { success = false; setError(E_IIC_ERROR);}
    #endif
    #ifdef I2C_SWITCH
    if (gotIIC()){
      if (!init_PCA9536()){
        success = false;
        setError(E_IIC_ERROR);
      }
    }
    #else
    pinMode(MEASURES_ENABLE_PIN,OUTPUT);
    digitalWrite(MEASURES_ENABLE_PIN, MEASURES_OFF);
    #ifdef I2C_BUS
    freeIIC();
    #endif
    #ifdef GSM_MODULE
    pinMode(GSM_ENABLE_PIN,OUTPUT);
    digitalWrite(GSM_ENABLE_PIN, MEASURES_OFF);
    #endif
    #endif
    _D(if(!success){ DebugPrintln("open Measures failed", DEBUG_LEVEL_1); delay(10);})
    CTLenabled = success;
    _D(DebugPrintln("Controls enabled", DEBUG_LEVEL_1); )
  } _D(else { DebugPrintln("Controls already enabled", DEBUG_LEVEL_2); })
  return success;
}

void disableControls(bool force){
  if(CTLenabled || force){
    #ifdef I2C_SWITCH
    getIIC();
    //configure all pins as input
    iic_writeRegister(PCA_9536_DEFAULT_ADDRESS, PCA_9536_PORT_REG, 0xff);
    iic_writeRegister(PCA_9536_DEFAULT_ADDRESS, PCA_9536_CONFIG_REG, 0xff);
    freeIIC();
    #else
    pinMode(MEASURES_ENABLE_PIN,OUTPUT);
    digitalWrite(MEASURES_ENABLE_PIN, MEASURES_OFF);
    pinMode(MEASURES_ENABLE_PIN,INPUT);
    #ifdef GSM_MODULE
    pinMode(GSM_ENABLE_PIN,OUTPUT);
    digitalWrite(GSM_ENABLE_PIN, MEASURES_OFF);
    pinMode(GSM_ENABLE_PIN,INPUT);
    #endif
    #endif
    #ifdef CHECK_BATTERY
    #ifdef USE_VOLTAGE_MEAS_PIN
    pinMode(VOLT_ENABLE_PIN,OUTPUT);
    digitalWrite(VOLT_ENABLE_PIN, MEASURES_OFF);
    pinMode(VOLT_ENABLE_PIN,INPUT);
    #endif
    #endif
    _D(DebugPrintln("Close measures", DEBUG_LEVEL_1));
  } _D(else { DebugPrintln("Measures already closed", DEBUG_LEVEL_2); })
  CTLenabled = false;
}

#ifdef I2C_SWITCH
bool init_PCA9536(void){
  //getIIC() is done in the init-function above
  if (!gotIIC()) { return false; }
  // set all pins to HIGH
  if (iic_writeRegister(PCA_9536_DEFAULT_ADDRESS, PCA_9536_PORT_REG, 0xff) != I2C_ERROR_OK){
    _D(DebugPrintln("No PCA9536 detected - Measures disabled", DEBUG_LEVEL_1);)
    return false;
  }
  // configure pin 0-2 as output
  return (iic_writeRegister(PCA_9536_DEFAULT_ADDRESS, PCA_9536_CONFIG_REG, 0xf8) == I2C_ERROR_OK);
}
#endif

void disableULP(void){
  CLEAR_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
  #if ULP_LED_BLINK
  rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_DISABLED);
  #endif
  clrState(ULP_RUNNING);
}
void enableULP(void){
  rtc_gpio_init(I2C_SDA);
  rtc_gpio_set_direction(I2C_SDA, RTC_GPIO_MODE_INPUT_OUTPUT);
  rtc_gpio_init(I2C_SCL);
  rtc_gpio_set_direction(I2C_SCL, RTC_GPIO_MODE_INPUT_OUTPUT);
  SET_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
}
void enableHoldPin(gpio_num_t which){
  gpio_hold_en(which);
  //gpio_deep_sleep_hold_en();
}
void disableHoldPin(gpio_num_t which){
  gpio_hold_dis(which);
  //gpio_deep_sleep_hold_dis();
}

void disableGPIOs(void){
  setGPIOInput(GPIO_NUM_4 ); //SCL
  setGPIOInput(GPIO_NUM_5 );
  setGPIOInput(GPIO_NUM_13); //SDA, (GSM_ENABLE_PIN)
  setGPIOInput(GPIO_NUM_14);
  setGPIOInput(GPIO_NUM_15);
  setGPIOInput(GPIO_NUM_16); //RXD
  setGPIOInput(GPIO_NUM_17); //TXD
  setGPIOInput(GPIO_NUM_18);
  setGPIOInput(GPIO_NUM_19);
  setGPIOInput(GPIO_NUM_21); //VOLT_ENABLE_PIN, GSM_RST (TXD)
  setGPIOInput(GPIO_NUM_22); //TEMP_SENSOR_PIN
  setGPIOInput(GPIO_NUM_23); //GSM_ENABLE_PIN, (RXD)
  setGPIOInput(GPIO_NUM_25); //MEASURES_ENABLE_PIN
  setGPIOInput(GPIO_NUM_26);
  setGPIOInput(GPIO_NUM_27);
}

void setGPIOInput(gpio_num_t which){
  #ifdef USE_ULP
  if ((which != I2C_SDA) && (which != I2C_SCL)) {
    pinMode(which, INPUT);
  }
  #else
  pinMode(GPIO_NUM_27, INPUT);
  #endif
}

#ifdef TEMP_SENSOR
float measureTemperature(){
  float temp;
  int start = millis();
  tempSensor.begin();
  _D(DebugPrintln("measure temperature", DEBUG_LEVEL_1); delay(50);)
  for(int i=0; i<500; i++){
    tempSensor.requestTemperaturesByIndex(0);
    temp = tempSensor.getTempCByIndex(0);
    if(temp != NO_TEMPERATURE) { break; }
    if(millis() - start > MAX_MEAS_TIME) { temp = NO_TEMPERATURE; break; }
  }
  return temp;
}
#endif

int MeasureVoltage(uint8_t pin){
  int analog_value = 0;
  #ifdef USE_VOLTAGE_MEAS_PIN
  VoltOn();
  #endif
  for(int i=0; i<1000; i++){ analog_value += analogRead(pin); }
  #ifdef USE_VOLTAGE_MEAS_PIN
  VoltOff();
  #endif
  analog_value /= 1000;
  return(analog_value);
}

bool enableMeasures(){
  if (!CTLenabled){
    _D(DebugPrintln("controls not enabled - cannot measure", DEBUG_LEVEL_1); delay(50););
    return false;
  }
  if (GSMenabled){
    _D(DebugPrintln("GSM still enabled - cannot open Measures", DEBUG_LEVEL_1); delay(50);)
    return false;
  }
  if (running_measures == 0){
    /* pulsing probably not needed due to using a backup-battery for the Arduino
     * ...probably... there is a nervous short circuit detection in loader modules and we have cabs ...
     */
    #ifdef I2C_SWITCH
    int i = 0;
    getIIC();
    while ((iic_setRegisterBit(PCA_9536_DEFAULT_ADDRESS, PCA_9536_PORT_REG, PCA_9536_MEAS_BIT, MEASURES_ON) != I2C_ERROR_OK) && (i < 100)){
      delay(10); i++;
    }
    freeIIC();
    #else
    pinMode(MEASURES_ENABLE_PIN,OUTPUT);
    for(int i=0; i<5; i++){
      digitalWrite(MEASURES_ENABLE_PIN, MEASURES_OFF);
      delayMicroseconds(50);
      digitalWrite(MEASURES_ENABLE_PIN, MEASURES_ON);
      delayMicroseconds(50);
    }
    digitalWrite(MEASURES_ENABLE_PIN, MEASURES_ON);
    #endif
    _D(DebugPrintln("Measures on", DEBUG_LEVEL_1));
    MEASenabled = true;
  }
  _D( else {DebugPrintln("Increment measures to " + String(running_measures + 1), DEBUG_LEVEL_1);})
  running_measures++;
  return true;
}
void disableMeasures(){
  running_measures--;
  if (running_measures == 0){
    #ifdef I2C_SWITCH
    int i = 0;
    getIIC();
    while ((iic_setRegisterBit(PCA_9536_DEFAULT_ADDRESS, PCA_9536_PORT_REG, PCA_9536_MEAS_BIT, MEASURES_OFF) != I2C_ERROR_OK) && (i < 100)){
      delay(10); i++;
    }
    freeIIC();
    #else
    pinMode(MEASURES_ENABLE_PIN,OUTPUT);
    digitalWrite(MEASURES_ENABLE_PIN, MEASURES_OFF);
    #endif
    _D(DebugPrintln("Measures off", DEBUG_LEVEL_1);)
    MEASenabled = false;
  }
  _D( else {DebugPrintln("Decrement measures to " + String(running_measures), DEBUG_LEVEL_1);})
  if (running_measures < 0) { running_measures = 0; }
}

#ifdef USE_VOLTAGE_MEAS_PIN
void VoltOn(){
  pinMode(VOLT_ENABLE_PIN,OUTPUT);
  digitalWrite(VOLT_ENABLE_PIN, MEASURES_ON);
  _D(DebugPrintln("Voltage measuring on", DEBUG_LEVEL_1));
}
void VoltOff(){
  pinMode(VOLT_ENABLE_PIN,OUTPUT);
  digitalWrite(VOLT_ENABLE_PIN, MEASURES_OFF);
  _D(DebugPrintln("Voltage measuring off", DEBUG_LEVEL_1));
}
#endif
#ifdef GSM_MODULE
void GSMOn(){
  pinMode(GSM_RST,OUTPUT);
  digitalWrite(GSM_RST, LOW);
  /* pulsing probably not needed due to using a backup-battery for the Arduino and the boost converter has a own soft start implementation
   * ...probably... there is a nervous short circuit detection in loader modules and we have huge cabs ...
   */
  #ifdef I2C_SWITCH
  int i = 0;
  getIIC();
  while ((iic_setRegisterBit(PCA_9536_DEFAULT_ADDRESS, PCA_9536_PORT_REG, PCA_9536_GSM_BIT, MEASURES_ON) != I2C_ERROR_OK) && (i < 100)){
    delay(10); i++;
  }
  freeIIC();
  #else
  pinMode(GSM_ENABLE_PIN,OUTPUT);
  for(int i=0; i<2000; i++){
    digitalWrite(GSM_ENABLE_PIN, MEASURES_OFF);
    delayMicroseconds(2010-i);
    digitalWrite(GSM_ENABLE_PIN, MEASURES_ON);
    delayMicroseconds(10+i);
  }
  digitalWrite(GSM_ENABLE_PIN, MEASURES_ON);
  #endif
  //_DD(DebugPrintln("GSM Voltage on", DEBUG_LEVEL_3));
  delay(1000); //setup voltage time
  digitalWrite(GSM_RST, HIGH);
  //_DD(DebugPrintln("GSM Reset released", DEBUG_LEVEL_3));
  delay(5000); //boot time
}
void GSMOff(){
  #ifdef I2C_SWITCH
  int i = 0;
  getIIC();
  while ((iic_setRegisterBit(PCA_9536_DEFAULT_ADDRESS, PCA_9536_PORT_REG, PCA_9536_GSM_BIT, MEASURES_OFF) != I2C_ERROR_OK) && (i < 100)){
    delay(10); i++;
  }
  freeIIC();
  #else
  pinMode(GSM_ENABLE_PIN,OUTPUT);
  digitalWrite(GSM_ENABLE_PIN, MEASURES_OFF);
  #endif
  //_DD(DebugPrintln("GSM Voltage off", DEBUG_LEVEL_3));
}
#endif
void LED_off(){
  pinMode(LED_ENABLE_PIN,OUTPUT);
  digitalWrite(LED_ENABLE_PIN, LED_OFF);
}
#ifdef I2C_BUS
bool getIIC(void){
  set_IIC_request(1);
  //wait a short time - maybe ULP was just on the way to lock...
  delay(1);
  int i = 0;
  while(IICisLocked()){
    delay(1); i++;
    if(i >= 20){ set_IIC_request(0); _D(DebugPrintln("Can't get IIC.", DEBUG_LEVEL_1);) return false; }
  }
  if (i>0) { _DD(DebugPrintln("Got IIC after " + String(i) + " wait-loops.", DEBUG_LEVEL_3);) }
  //ULP just checks the request, no need to set LOCK here
  rtc_gpio_hold_dis(I2C_SCL);
  rtc_gpio_hold_dis(I2C_SDA);
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  return true;
}
void freeIIC(void){
  rtc_gpio_init(I2C_SCL);
  rtc_gpio_set_direction(I2C_SCL, RTC_GPIO_MODE_INPUT_OUTPUT);
  rtc_gpio_hold_en(I2C_SCL);
  rtc_gpio_init(I2C_SDA);
  rtc_gpio_set_direction(I2C_SDA, RTC_GPIO_MODE_INPUT_OUTPUT);
  rtc_gpio_hold_en(I2C_SDA);
  set_IIC_request(0);

}i2c_err_t iic_setRegisterBit(uint8_t devAdress, byte regAdress, int bitPos, bool state) {
  byte _b;
  i2c_err_t rc = iic_readRegister(devAdress, regAdress, &_b);
  if (rc != I2C_ERROR_OK) { return rc; }
  if (state) {
    _b |= (1 << bitPos);  // Forces nth Bit of _b to 1. Other Bits Unchanged.
  }
  else {
    _b &= ~(1 << bitPos); // Forces nth Bit of _b to 0. Other Bits Unchanged.
  }
  return iic_writeRegister(devAdress, regAdress, _b);
}

i2c_err_t iic_getRegisterBit(uint8_t devAdress, uint8_t regAdress, int bitPos, bool *value) {
  uint8_t _b;
  i2c_err_t rc = iic_readRegister(devAdress, regAdress, &_b);
  *value = ((_b >> bitPos) & 1);
  return rc;
}

i2c_err_t iic_readRegister(uint8_t devAdress, uint8_t regAdress, uint8_t *value) {
  byte cnt;
  if (!gotIIC) { return I2C_ERROR_BUSY; }
  for(int i=0; i<2; i++){
    if(i == 1) { iic_clockFree(); delay(10); }
    Wire.beginTransmission(devAdress);  //begin does not touch device
    Wire.write(regAdress);            //write does not touch device
    if(Wire.endTransmission() != I2C_ERROR_OK){
      _DD(DebugPrintln("i2c read device 0x" + String(devAdress, HEX) + " register 0x" + String(regAdress, HEX) + " error: " + String(Wire.lastError()), DEBUG_LEVEL_3));
      if(i == 0){ continue; } else { return (i2c_err_t)Wire.lastError(); }
    } //endTransmission writes to device and returns an error code
    if((cnt = Wire.requestFrom(devAdress, (uint8_t)1)) != 1){
      _DD(DebugPrintln("i2c read device 0x" + String(devAdress, HEX) + " register 0x" + String(regAdress, HEX) + " count: " + String(cnt) + " error: " + String(Wire.lastError()), DEBUG_LEVEL_3));
      if(i == 0){ continue; } else { return (i2c_err_t)I2C_ERROR_READ_COUNT; }
    } //requestFrom returns count of bytes read from device
    *value =(uint8_t)Wire.read(); //read does not touch device
    break;
  }
  _DD(DebugPrintln("i2c read device 0x" + String(devAdress, HEX) + " register 0x" + String(regAdress, HEX) + ": 0x" + String(*value, HEX), DEBUG_LEVEL_3));
  return I2C_ERROR_OK;
}
i2c_err_t iic_writeRegister(uint8_t devAdress, uint8_t regAdress, uint8_t value) {
  i2c_err_t rc = (i2c_err_t)I2C_ERROR_UNDEFINED;
  if (!gotIIC) { return I2C_ERROR_BUSY; }
  for(int i=0; i<2; i++){
    if(i == 1) { iic_clockFree(); delay(10); }
    Wire.beginTransmission(devAdress); //begin does not touch device
    Wire.write(regAdress);  //write does not touch device
    Wire.write(value);      //write does not touch device
    rc = (i2c_err_t)Wire.endTransmission(); //endTransmission writes to device and returns an error code
    if(rc == I2C_ERROR_OK) { break; }
  }
  return rc;
}

i2c_err_t iic_readRegister16(uint8_t devAdress, uint8_t regAdress, int16_t *value) {
  byte cnt;
  i2c_err_t err;
  if (!gotIIC) { return I2C_ERROR_BUSY; }
  for(int i=0; i<2; i++){
    if(i == 1) { iic_clockFree(); delay(10); }
    Wire.beginTransmission(devAdress); //begin does not touch device
    Wire.write(regAdress);             //write does not touch device
    if(Wire.endTransmission() != I2C_ERROR_OK){
      _DD(DebugPrintln("i2c read device 0x" + String(devAdress, HEX) + " register 0x" + String(regAdress, HEX) + "error: " + String(Wire.lastError()), DEBUG_LEVEL_3));
      if(i == 0){ continue; } else { return (i2c_err_t)Wire.lastError(); }
    }//endTransmission writes to device and returns an error code
    if((cnt = Wire.requestFrom(devAdress, regAdress, (uint8_t)2)) != 2){
      _DD(DebugPrintln("i2c read device 0x" + String(devAdress, HEX) + " register 0x" + String(regAdress, HEX) + " count: " + String(cnt) + "error: " + String(Wire.lastError()), DEBUG_LEVEL_3));
      if(i == 0){ continue; } else { return (i2c_err_t)I2C_ERROR_READ_COUNT; }
    } //requestFrom returns count of bytes read from device
    *value =(int16_t)(Wire.read() | (Wire.read() << 8)); //read does not touch device
    break;
  }
  return I2C_ERROR_OK;
}
void iic_clockFree(void) {
  _DD(DebugPrint("I2C clocking SDA:", DEBUG_LEVEL_3);)
  if (!gotIIC) { return; }
  pinMode(I2C_SDA,INPUT);
  pinMode(I2C_SCL,OUTPUT);
  digitalWrite(I2C_SCL, LOW);
  delay(10);
  for(int i=0; i<10; i++){
    digitalWrite(I2C_SCL, HIGH);
    delay(1);
    digitalWrite(I2C_SCL, LOW);
    delay(1);
    _DD(DebugPrint(String(digitalRead(I2C_SDA)) + ",", DEBUG_LEVEL_3));
  }
  _DD(DebugPrintln("", DEBUG_LEVEL_3));
  delay(10);
  pinMode(I2C_SCL,INPUT);
  delay(10);
}
#endif
