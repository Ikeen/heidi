/*
 * heidi-gsm.h
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */

#ifndef HEIDI_GSM_H_
#define HEIDI_GSM_H_
#include "heidi-defines.h"
#define GSM      13    // On/Off

#ifdef GSM_MODULE
#include <stdint.h>
#include <WString.h>
#define GSM_RST  21
#define GSM_RXD  23    //!!!
#define GSM_TXD  4     //!!!
#define GSM_UART_NO 1
#define GSM_ON   LOW
#define GSM_OFF  HIGH

void hGSM_on();
void hGSM_off();
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
