/*
 * heidi.mesures.h
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
#include "heidi-debug.h"
#include "heidi-error.h"


#define MEASURES_ENABLE_PIN   25   // GPIO25  -- enable measures, including GPS
#define MEASURES_ON   LOW
#define MEASURES_OFF  HIGH

#define LED_ENABLE_PIN        2     // GPIO2   -- LED_ENABLE_PIN
#define LED_ON   HIGH
#define LED_OFF  LOW

/*
 * Voltage measuring
 */
#define BATTERY_ANALOG_PIN     36
//#define BATTERY_ANALOG_ENABLE  15
#define ANALOG_MEASURE_OFFSET  166
#define ANALOG_MEASURE_DIVIDER 605

/*
 * Temperature measuring
 */

#define TEMP_SENSOR_PIN        22
#define NO_TEMPERATURE        -127


#ifdef TEMP_SENSOR
float MeasureTemperature();
#endif

double MeasureVoltage();
void MeasuresOn();
void MeasuresOff();
void forceMeasuresOff();
void LED_off();

#endif /* HEIDI_MEASURES_H_ */
