/*
 * heidi-gsm.h
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */

#ifndef HEIDI_GPS_H_
#define HEIDI_GPS_H_
#include "heidi-defines.h"
#ifdef GPS_MODULE
#include <stdint.h>
#include <WString.h>
#include "esp32-hal-gpio.h"
#include "TinyGPS++.h"
#include "heidi-data.h"

#define GPS_RXD   GPIO_NUM_16
#define GPS_TXD   GPIO_NUM_17
#define GPS_UART_NO 1
#define GPS_BAUD 9600

#define WAIT_FOR_GPS_TIME 180000

int  GPSGetPosition(t_SendData* DataSet, int averages, int timeOut);
bool setSysTimeToGPSTime(int timeOut);
bool SetSysToGPS();
bool openGPS();
void closeGPS();
#if DEBUG_LEVEL >= DEBUG_LEVEL_1
void _PrintDataGPS();
void testGPS();
#endif

#endif /*GSM_MODULE*/
#endif /* HEIDI_GSM_H_ */
