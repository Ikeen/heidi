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
#ifdef I2C_BUS
#define I2C_ERROR_READ_COUNT (I2C_ERROR_NO_BEGIN + 1)
#define I2C_ERROR_UNDEFINED (I2C_ERROR_NO_BEGIN + 2)
#endif
#ifdef I2C_BUS
#include <Wire.h>
#endif
#include "heidi-debug.h"
#include "heidi-error.h"

#define I2C_SDA GPIO_NUM_13
#define I2C_SCL GPIO_NUM_4
#define I2C_FREQ 100000L


#define PCA_9536_DEFAULT_ADDRESS 0x41
#define PCA_9536_CONFIG_REG      0x03
#define PCA_9536_PORT_REG        0x01
#define PCA_9536_GSM_BIT         0x00
#define PCA_9536_MEAS_BIT        0x01
#define PCA_9536_VOLT_BIT        0x02

#ifndef I2C_SWITCH
#ifdef HEIDI_CONFIG_1
#define GSM_ENABLE_PIN        GPIO_NUM_13
#else
#define GSM_ENABLE_PIN        GPIO_NUM_23
#endif
#define MEASURES_ENABLE_PIN   GPIO_NUM_25
#endif

#ifdef USE_VOLTAGE_MEAS_PIN
#define VOLT_ENABLE_PIN       GPIO_NUM_21
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
#define ANALOG_MEASURE_OFFSET  302
#define ANALOG_MEASURE_DIVIDER 650
#endif

#ifdef HEIDI_CONFIG_TEST
#define ANALOG_MEASURE_OFFSET  303
#define ANALOG_MEASURE_DIVIDER 659

//#define ANALOG_MEASURE_OFFSET  315
//#define ANALOG_MEASURE_DIVIDER 633
#endif

/*
 * Temperature measuring
 */

#define TEMP_SENSOR_PIN        GPIO_NUM_22
#define NO_TEMPERATURE        -127
#define TEMPERATURE_NOT_SET   -27315

#ifdef TEMP_SENSOR
float MeasureTemperature();
#endif

bool enableControls(void);
void disableControls(bool);
bool openMeasures(void);
void closeMeasures(void);
void stopULP(void);
void enableULP(void);
void EnableHoldPin(gpio_num_t which);
void DisableHoldPin(gpio_num_t which);

void VoltOn(void);
void VoltOff(void);
void GSMOn(void);
void GSMOff(void);
void LED_off(void);

int MeasureVoltage(uint8_t pin);

#ifdef I2C_SWITCH
bool init_PCA9536(void);
#endif
#ifdef I2C_BUS
bool getIIC(void);
void freeIIC(void);

i2c_err_t iic_readRegister(uint8_t devAdress, uint8_t regAdress, uint8_t *value);
i2c_err_t iic_readRegister16(uint8_t devAdress, uint8_t regAdress, int16_t *value);
i2c_err_t iic_writeRegister(uint8_t devAdress, uint8_t regAdress, uint8_t value);
i2c_err_t iic_setRegisterBit(uint8_t devAdress, uint8_t regAdress, int bitPos, bool state);
i2c_err_t iic_getRegisterBit(uint8_t devAdress, uint8_t regAdress, int bitPos, bool *value);
#endif

void iic_clockFree(void);

#endif /* HEIDI_MEASURES_H_ */
