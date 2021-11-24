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

bool GSMsetup();
bool GSMshutDown();
int  GSMdoPost(String url, String contentType, String payload, unsigned int clientWriteTimeoutMs, unsigned int serverReadTimeoutMs);
bool GSMhandleAlerts(void);
bool GSMsendSMS(String TelNo, String Message);

bool   GSMRestartModem(void);
bool   GSMwaitForModem(uint32_t timeOutMS = 5000);
bool   GSMwaitForNetwork(uint32_t timeOutMS = 15000);
bool   GSMsimUnlock(String pin);
int    GSMinitiateHTTP(String url);
int    GSMterminateHTTP();
bool   GSMsetupGPRS(const String apn, const String user, const String pwd);
bool   GSMopenGPRS();
int    GSMterminateGPRS();
String GSMGetLastResponse(void);
String GSMsendCommand(const String command, int timeoutMs);
bool   GSMsendCommand(const String command, const String okPhrase, int timeOutMs);
bool   GSMsendCommand(const String command);
bool   GSMwriteDown(const String payload);

int  _responseGetInt(int position, int errValue = -1);
int  _responseGetInt(int position, String response, int errValue);

_D(void   GSMCheckSignalStrength();)

#endif /*GSM_MODULE*/
#endif /* HEIDI_GSM_H_ */
