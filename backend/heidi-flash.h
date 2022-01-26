/*
 * heidi-flash1.h
 *
 *  Created on: 23.11.2021
 *      Author: frank
 */

#ifndef HEIDI_FLASH_H_
#define HEIDI_FLASH_H_

#include "heidi-debug.h"

#define FLASH_INT_UNSET -1
#define FLASH_STR_UNSET ""

typedef __attribute__((__packed__)) struct _t_flashRawData{
 byte herdeID[2];
 byte animalID[2];
 //#ifdef GSM_MODULE
 byte pushDataWebAddress[256];
 byte mobileAPN[64];
 byte mobileUSER[16];
 byte mobilePASSWD[16];
 byte mobilePIN[8];
 //#endif
 byte checkSum[2];
}t_flashRawData;

typedef __attribute__((__packed__)) struct _t_flashData{
 int16_t herdeID;
 int16_t animalID;
 //#ifdef GSM_MODULE
 String pushDataWebAddress;
 String mobileAPN;
 String mobileUSER;
 String mobilePASSWD;
 String mobilePIN;
 //#endif
}t_flashData;

typedef enum{
  HEIDI_MENU_MAIN,
  HEIDI_MENU_IDENTITY,
  HEIDI_MENU_WEB_INTERFACE
}t_heidiMenu;

void setupFlashData(bool powerOnReset);

void handleHook();
void printMenu(t_flashData* flashData);
bool handleMenu(char key);
void printMainMenu(t_flashData* flashData);
bool handleMainMenu(char key);
void printIdentityMenu(t_flashData* flashData);
bool handleIdentityMenu(char key);
void printWebInterfaceMenu(t_flashData* flashData);
bool handleWebInterfaceMenu(char key);
void printFooter(void);

void loadDefaultData(t_flashData* flashData);
bool loadFlashData(t_flashData* flashData, t_flashRawData* flashRawData);
void writeFlashData(t_flashData* flashData, t_flashRawData* flashRawData);

void setFlashString(byte* ,int ,const String );
String getFlashString(byte* ,int );
int16_t getFlashInt(byte* );
void setFlashInt(byte* buffer, int16_t );
void clearFlashString(byte*, int );

String readNewValue(const String prompt, bool* timeout);
char getSerialKey(void);
bool checkData(t_flashRawData* flashRawData);
void setCheckSum(t_flashRawData* flashRawData);

_D( void printFlashString(byte* , int ); )


#endif /* HEIDI_FLASH_H_ */
