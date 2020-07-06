/*
 * heidi-gsm.h
 *
 *  Created on: 22.06.2020
 *      Author: frank
 */

#ifndef HEIDI_GSM_H_
#define HEIDI_GSM_H_
#include "heidi-defines.h"
#ifdef GSM_MODULE
#include <stdint.h>
#include <WString.h>

#define GSM      13    // On/Off
#define GSM_RST  21
#define GSM_RXD  23    //!!!
#define GSM_TXD  4     //!!!
#define GSM_UART_NO 1
#define GSM_ON   LOW
#define GSM_OFF  HIGH

void GSM_on();
void GSM_off();
bool GSMsetup();
bool GSMshutDown();
bool GSMsetupGPRS(const String apn, const String user, const String pwd);
int GSMinitiateHTTP(String url);
int GSMdoPost(String url, String contentType, String payload, unsigned int clientWriteTimeoutMs, unsigned int serverReadTimeoutMs);
int GSMdoGet(const char* url, unsigned int serverReadTimeoutMs);
int GSMterminateHTTP();
String GSMsendCommand(const String command, int timeoutMs = 5000);
bool  GSMsendCommand2(const String command, String response, int timeoutMs = 5000);
void GSMCheckSignalStrength();

#endif /*GSM_MODULE*/
#endif /* HEIDI_GSM_H_ */
