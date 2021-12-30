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
//SIM800L max auto-bauding rate is 57600
#define GSM_BAUD 57600
/* you need to adjust READ_STRING_TIMEOUT if you intend to use smaller Baud-rates, consider at least 100 character times
 * as enough (56700 are about 5000 char/s = 100 char/20ms)
 * consider that Stream::readString() does not use timeout as over all timeout but as timeout on silence, so a 100 char
 * times timeout will not break a read of 5000 chars on block - if we wait for the first char previously, and we do so
 */
#define READ_STRING_TIMEOUT 50


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
bool   GSMsendCommand(const String command, const String okPhrase, int timeOutMs);
bool   GSMsendCommand(const String command);
bool   GSMwriteDown(const String payload);

bool GSMsetCFUN(uint32_t timeOutMS);
bool GSMwaitPIN(uint32_t timeOutMS);

void testGSM(void);

int  _responseGetIntKeyWord(int position, String keyWord, int errValue = -1);
int  _responseGetInt(int position, int errValue = -1);
int  _responseGetInt(int position, String response, int errValue);

_D(void   GSMCheckSignalStrength();)

#endif /*GSM_MODULE*/
#endif /* HEIDI_GSM_H_ */
