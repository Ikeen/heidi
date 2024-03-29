/*
 * heidi-error.h
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */

#ifndef HEIDI_ERROR_H_
#define HEIDI_ERROR_H_
#include "heidi-data.h"

/** error handling **/

#define E_WRONG_BOOT_REASON        0x0001
#define E_GSM_TRANSMISSION_FAILED  0x0002
#define E_COULD_NOT_FETCH_GPS_TIME 0x0004
#define E_COULD_NOT_FETCH_GPS      0x0008
#define E_WRONG_GPS_VALUES         0x0010
#define E_NEW_FENCE_STATE          0x0020
#define E_GPS_ALERT                0x0040
#define E_ACC_ALERT                0x0080
#define E_ALERT_SMS                0x0100
#define E_IIC_ERROR                0x0200
#define E_POWER_SAVE_1             0x0400
#define E_GPS_LATE_LOCK            0x0800

#define E_TRANSMIT_REGULAR         0x1000
#define E_TRANSMIT_ALERT           0x2000
#define E_TRANSMIT_ALERT_NO_GPS    0x4000
#define E_RECEIVE_DATA_ERROR       0x8000

//    case 1 : Serial.println ("POWERON_RESET");break;          /**<1, Vbat power on reset*/
//    case 3 : Serial.println ("SW_RESET");break;               /**<3, Software reset digital core*/
//    case 4 : Serial.println ("OWDT_RESET");break;             /**<4, Legacy watch dog reset digital core*/
//    case 5 : Serial.println ("DEEPSLEEP_RESET");break;        /**<5, Deep Sleep reset digital core*/
//    case 6 : Serial.println ("SDIO_RESET");break;             /**<6, Reset by SLC module, reset digital core*/
//    case 7 : Serial.println ("TG0WDT_SYS_RESET");break;       /**<7, Timer Group0 Watch dog reset digital core*/
//    case 8 : Serial.println ("TG1WDT_SYS_RESET");break;       /**<8, Timer Group1 Watch dog reset digital core*/
//    case 9 : Serial.println ("RTCWDT_SYS_RESET");break;       /**<9, RTC Watch dog Reset digital core*/
//    case 10 : Serial.println ("INTRUSION_RESET");break;       /**<10, Instrusion tested to reset CPU*/
//    case 11 : Serial.println ("TGWDT_CPU_RESET");break;       /**<11, Time Group reset CPU*/
//    case 12 : Serial.println ("SW_CPU_RESET");break;          /**<12, Software reset CPU*/
//    case 13 : Serial.println ("RTCWDT_CPU_RESET");break;      /**<13, RTC Watch dog Reset CPU*/
//    case 14 : Serial.println ("EXT_CPU_RESET");break;         /**<14, for APP CPU, reseted by PRO CPU*/
//    case 15 : Serial.println ("RTCWDT_BROWN_OUT_RESET");break;/**<15, Reset when the vdd voltage is not stable*/
//    case 16 : Serial.println ("RTCWDT_RTC_RESET");break;      /**<16, RTC Watch dog reset digital core and rtc module*/

void setupError();
void setError(uint16_t code);
void rmError(uint16_t code);
bool getError(uint16_t code);
void setError(t_SendData* DataSet, uint16_t code);
void rmError(t_SendData* DataSet, uint16_t code);
bool getError(t_SendData* DataSet, uint16_t code);
uint16_t getErrorCode();

#endif /* HEIDI_ERROR_H_ */
