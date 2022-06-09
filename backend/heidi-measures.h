/*
 * heidi-mesures.h
 *
 *  Created on: 08.07.2020
 *      Author: frank
 */

#ifndef HEIDI_MEASURES_H_
#define HEIDI_MEASURES_H_
#include "heidi-defines.h"
#ifdef TEMP_SENSOR
#include "OneWire.h"
#include "DallasTemperature.h"
#endif
#ifdef ACCELEROMETER
#include "heidi-acc.h"
#endif
#ifdef I2C_BUS
#define I2C_ERROR_READ_COUNT (I2C_ERROR_NO_BEGIN + 1)
#define I2C_ERROR_UNDEFINED (I2C_ERROR_NO_BEGIN + 2)
#endif
#ifdef I2C_BUS
#include <Wire.h>
#endif
#include "heidi-debug.h"
#include "heidi-error.h"

#define I2C_SDA GPIO_NUM_13      //when change - take care for gotoSleep() - function settings
#define I2C_SCL GPIO_NUM_4       //when change - take care for gotoSleep() - function settings
#define I2C_FREQ 100000L

#define UART1_RXD  GPIO_NUM_16
#define UART1_TXD  GPIO_NUM_17
#define UART1 1
#define UART2_RXD  GPIO_NUM_23
#define UART2_TXD  GPIO_NUM_4
#define UART2 2
#define UART_RX_BUFFER_SIZE 1024

#ifdef COMMON_SERIAL
#define NUM_OF_UARTS 1
#else
#define NUM_OF_UARTS 2
#endif

#define PCA_9536_DEFAULT_ADDRESS 0x41
#define PCA_9536_CONFIG_REG      0x03
#define PCA_9536_PORT_REG        0x01
#define PCA_9536_GSM_BIT         0x00
#define PCA_9536_MEAS_BIT        0x01
#define PCA_9536_VOLT_BIT        0x02

#ifndef I2C_SWITCH
#ifdef USE_HEIDI_CONFIG_1_PINS
#define GSM_ENABLE_PIN        GPIO_NUM_13
#else
#define GSM_ENABLE_PIN        GPIO_NUM_23
#endif
#define MEASURES_ENABLE_PIN   GPIO_NUM_25
#endif

#ifdef USE_VOLT_MEAS_EN_PIN
#ifdef USE_HEIDI_CONFIG_3_PINS
#define VOLT_ENABLE_PIN       GPIO_NUM_22
#else
#define VOLT_ENABLE_PIN       GPIO_NUM_21
#endif
#endif

#define MEASURES_ON   LOW
#define MEASURES_OFF  HIGH

#define LED_ENABLE_PIN        2     // GPIO2   -- LED_ENABLE_PIN
#define LED_ON   HIGH
#define LED_OFF  LOW

/*
 * Voltage measuring
 */
#define BATTERY_MEASURE_PIN     36

#ifdef HEIDI_CONFIG_1
#define ANALOG_MEASURE_OFFSET  106
#define ANALOG_MEASURE_DIVIDER 605
#endif

#ifdef HEIDI_CONFIG_2
#define ANALOG_MEASURE_OFFSET  -138
#define ANALOG_MEASURE_DIVIDER 621
#endif

#ifdef HEIDI_CONFIG_3
#if HEIDI_HERDE == 1
#if HEIDI_ANIMAL == 1
#define ANALOG_MEASURE_OFFSET  35
#define ANALOG_MEASURE_DIVIDER 597
#endif
#if HEIDI_ANIMAL == 3
#define ANALOG_MEASURE_OFFSET  -55
#define ANALOG_MEASURE_DIVIDER 612
#endif
#endif
#endif

#ifdef HEIDI_CONFIG_TEST
#if HEIDI_ANIMAL == 3
#define ANALOG_MEASURE_OFFSET   64
#define ANALOG_MEASURE_DIVIDER 590
#else
#define ANALOG_MEASURE_OFFSET  -138
#define ANALOG_MEASURE_DIVIDER 621
#endif
#endif

#ifdef ANALOG_MEASURE_DIVIDER
#if ANALOG_MEASURE_DIVIDER == 0
#error "divider set to zero!"
#endif
#endif

#define CALCULATE_VOLTAGE(x) (float(x - (ANALOG_MEASURE_OFFSET)) / ANALOG_MEASURE_DIVIDER)

/*
 * Temperature measuring
 */

#ifdef USE_HEIDI_CONFIG_3_PINS
#define TEMP_SENSOR_PIN        GPIO_NUM_21
#else
#define TEMP_SENSOR_PIN        GPIO_NUM_22
#endif
#define NO_TEMPERATURE        -127 //sent by sensor
#define TEMPERATURE_NOT_SET   -128 //(0xff)

#ifdef TEMP_SENSOR
float measureTemperature(void);
#ifndef NO_TESTS
void testTemp(void);
#endif
#endif

bool enableControls(void);
void disableControls(bool);
bool enableMeasures(void);
void disableMeasures(void);
void enableULP(void);
void disableULP(void);
void enableHoldPin(gpio_num_t which);
void disableHoldPin(gpio_num_t which);
void disableGPIOs(void);

bool openUart(uint8_t uartNo, uint32_t baud, uint8_t inputType);

void VoltOn(void);
void VoltOff(void);
void GSMOn(void);
void GSMOff(void);
void LED_off(void);
void setGPIOInput(gpio_num_t which);
void setGPIOInputHigh(gpio_num_t which);

int MeasureVoltage(uint8_t pin);
#ifdef TEST_VOLT
void testVolt(void);
#endif
#ifdef USE_ULP
bool getULPLock(void);
void freeULP(void);
#endif
#ifdef I2C_SWITCH
bool init_PCA9536(void);
#endif
#ifdef I2C_BUS
bool getIIC(void);
void freeIIC(void);
bool gotIIC(void);

i2c_err_t iic_readRegister(uint8_t devAdress, uint8_t regAdress, uint8_t *value);
i2c_err_t iic_readRegister16(uint8_t devAdress, uint8_t regAdress, int16_t *value);
i2c_err_t iic_writeRegister(uint8_t devAdress, uint8_t regAdress, uint8_t value);
i2c_err_t iic_setRegisterBit(uint8_t devAdress, uint8_t regAdress, int bitPos, bool state);
i2c_err_t iic_getRegisterBit(uint8_t devAdress, uint8_t regAdress, int bitPos, bool *value);
#endif

void iic_clockFree(void);

#endif /* HEIDI_MEASURES_H_ */
