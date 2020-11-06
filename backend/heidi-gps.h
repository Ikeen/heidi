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
#include "TinyGPS++.h"
#include "heidi-data.h"

#define GPS_RX   16    //GPIO16
#define GPS_TX   17    //GPIO17
#define GPS_UART_NO 2
#define WAIT_FOR_GPS_TIME   120000

int  GPSGetPosition(t_SendData* DataSet, int averages, int timeoutms);
bool SetSysToGPSTime();
bool SetSysToGPS();
void openGPS();
void closeGPS();
#if DEBUG_LEVEL >= DEBUG_LEVEL_1
void _PrintDataGPS();
void testGPS();
#endif

#endif /*GSM_MODULE*/
#endif /* HEIDI_GSM_H_ */
