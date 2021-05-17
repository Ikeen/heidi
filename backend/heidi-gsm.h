/*
 * heidi-gsm.h
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */

#ifndef HEIDI_GSM_H_
#define HEIDI_GSM_H_
#include "heidi-defines.h"
#include "esp32-hal-gpio.h"

#ifdef GSM_MODULE
#include <stdint.h>
#include <WString.h>

#ifdef COMMON_SERIAL
#define GSM_RXD  GPIO_NUM_16
#define GSM_TXD  GPIO_NUM_17
#define GSM_UART_NO 1
#else
#define GSM_RXD  GPIO_NUM_23
#define GSM_TXD  GPIO_NUM_4
#define GSM_UART_NO 2
#endif
#define GSM_RST  GPIO_NUM_21
#define GSM_BAUD 19200 //57600

bool openGSM();
void closeGSM();

bool hGSMsetup();
bool hGSMshutDown();
bool hGSMsetupGPRS(const String apn, const String user, const String pwd);
int  hGSMinitiateHTTP(String url);
int  hGSMdoPost(String url, String contentType, String payload, unsigned int clientWriteTimeoutMs, unsigned int serverReadTimeoutMs);
int  hGSMdoGet(const char* url, unsigned int serverReadTimeoutMs);
int  hGSMterminateHTTP();
String hGSMGetLastResponse(void);
String hGSMsendCommand(const String command, int timeoutMs);
String hGSMsendCommand(const String command);
bool  hGSMsendCommand2(const String command, String response, int timeoutMs = 5000);
void  hGSMCheckSignalStrength();

#endif /*GSM_MODULE*/
#endif /* HEIDI_GSM_H_ */
