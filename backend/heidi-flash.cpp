/*
 * heidi-flash1.cpp
 *
 *  Created on: 23.11.2021
 *      Author: frank
 */

#include <HardwareSerial.h>
#include <EEPROM.h>
#include "heidi-flash.h"
#include "heidi-sys.h"

t_flashData flashData;
t_flashRawData flashRawData;

void setupFlashData(bool powerOnReset){
  if(!powerOnReset) { return; }
  int t = millis();
  #if DEBUG_LEVEL > DEBUG_LEVEL_0
  Serial.begin(115200);
  #endif
  Serial.println("enter hook: press 'x'");
  String Rs232in = "";
  Serial.flush();
  while((Rs232in.indexOf("x") == -1) && (millis()-t < 3000)){
    if (Serial.available() > 0) {Rs232in += (char)Serial.read();}
    delay(1);
  }
  if (Rs232in.indexOf('x') == -1){ return; }
  Serial.println("hook entered.");

  if (!EEPROM.begin(sizeof(t_flashRawData))){
    Serial.println("Failed to setup EEPROM with " + String(sizeof(t_flashRawData)) + " bytes size.");
    return;
  }
  Serial.println("EEPROM set up");
  loadFlashData(&flashData, &flashRawData);
  printFlashDataMenu(&flashData);
  char key = ' ';
  while(key != 'q'){
    key = getSerialKey();
    bool timeout = false;
    switch(key) {
      case '1':{
        String NewValue = readNewValue("herd ID", 10, &timeout);
        if (!timeout) { flashData.herdeID = int16_t(NewValue.toInt()); }
        break;
      }
      case '2':{
        String NewValue = readNewValue("animal ID", 10, &timeout);
        if (!timeout) { flashData.animalID = int16_t(NewValue.toInt()); }
        break;
      }
      //#ifdef GSM_MODULE
      case '3':{
        String NewValue = readNewValue("push data web address", 10, &timeout);
        if (!timeout) { flashData.pushDataWebAddress = NewValue; }
        break;
      }
      case '4':{
        String NewValue = readNewValue("mobile APN", 10, &timeout);
        if (NewValue != "") { flashData.mobileAPN = NewValue; }
        break;
      }
      case '5':{
        String NewValue = readNewValue("mobile USER", 10, &timeout);
        if (!timeout) { flashData.mobileUSER = NewValue; }
        break;
      }
      case '6':{
        String NewValue = readNewValue("mobile PASSWD", 10, &timeout);
        if (!timeout) { flashData.mobilePASSWD = NewValue; }
        break;
      }
      case '7':{
        String NewValue = readNewValue("mobile PIN", 10, &timeout);
        if (!timeout) { flashData.mobilePIN = NewValue; }
        break;
      }
      case 'w':{
        break;
      }
      //#endif
      default:
        break;
    }
    for(int i=0; i<5; i++) { Serial.println(""); }
    printFlashDataMenu(&flashData);
  }
}

void printFlashDataMenu(t_flashData* flashData){
  Serial.println("");
  Serial.println("<----------------- Heidi set persistent data ---------->");
  Serial.println("");
  Serial.println("1: set herd   ID");
  if (flashData->herdeID == FLASH_INT_UNSET){
    #ifdef HEIDI_HERDE
    flashData->herdeID = HEIDI_HERDE;
    Serial.println("   \"" + intString4(flashData->herdeID) + "\" (*)");
    #else
    Serial.println("   -");
    #endif
  } else {
    Serial.println("   \"" + intString4(flashData->herdeID) + "\"");
  }
  Serial.println("2: set animal ID");
  if (flashData->animalID == FLASH_INT_UNSET){
    #ifdef HEIDI_ANIMAL
    flashData->animalID = HEIDI_ANIMAL;
    Serial.println("   \"" + intString4(flashData->animalID) + "\" (*)");
    #else
    Serial.println("   -");
    #endif
  } else {
    Serial.println("   \"" + intString4(flashData->animalID) + "\"");
  }
  Serial.println("3: set push data web address");
  if (flashData->pushDataWebAddress == FLASH_STR_UNSET){
    #ifdef HEIDI_SERVER_PUSH_URL
    flashData->pushDataWebAddress = HEIDI_SERVER_PUSH_URL;
    Serial.println("   \"" + flashData->pushDataWebAddress + "\" (*)");
    #else
    Serial.println("   -");
    #endif
  } else {
    Serial.println("   \"" + flashData->pushDataWebAddress + "\"");
  }
  Serial.println("4: set mobile APN");
  if (flashData->mobileAPN == FLASH_STR_UNSET){
    #ifdef HEIDI_MOBILE_APN
    flashData->mobileAPN = HEIDI_MOBILE_APN;
    Serial.println("   \"" + flashData->mobileAPN + "\" (*)");
    #else
    Serial.println("   -");
    #endif
  } else {
    Serial.println("   \"" + flashData->mobileAPN + "\"");
  }
  Serial.println("5: set mobile USER");
  if (flashData->mobileUSER == FLASH_STR_UNSET){
    #ifdef HEIDI_MOBILE_USER
    if (HEIDI_MOBILE_USER == FLASH_STR_UNSET){
      Serial.println("   \"" + flashData->mobileUSER + "\"");
    } else {     flashData->mobileUSER = HEIDI_MOBILE_USER;
      Serial.println("   \"" + flashData->mobileUSER + "\" (*)");
    }
    #else
    Serial.println("   -");
    #endif
  } else {
    Serial.println("   \"" + flashData->mobileUSER + "\"");
  }
  Serial.println("6: set mobile PASSWD");
  if (flashData->mobilePASSWD == FLASH_STR_UNSET){
    #ifdef HEIDI_MOBILE_PASSWD
    if (HEIDI_MOBILE_PASSWD == FLASH_STR_UNSET){
      Serial.println("   \"" + flashData->mobilePASSWD + "\"");
    } else {
      flashData->mobilePASSWD = HEIDI_MOBILE_PASSWD;
      Serial.println("   \"" + flashData->mobilePASSWD + "\" (*)");
    }
    #else
    Serial.println("   -");
    #endif
  } else {
    Serial.println("   \"" + flashData->mobilePASSWD + "\"");
  }
  Serial.println("7: set SIM PIN");
  if (flashData->mobilePIN == FLASH_STR_UNSET){
    #ifdef HEIDI_SIM_PIN
    if (HEIDI_SIM_PIN == FLASH_STR_UNSET){
      Serial.println("   \"" + flashData->mobilePIN + "\"");
    } else {
      flashData->mobilePIN = HEIDI_SIM_PIN;
      Serial.println("   \"" + flashData->mobilePIN + "\" (*)");
    }
    #else
    Serial.println("   -");
    #endif
  } else {
    Serial.println("   \"" + flashData->mobilePIN + "\"");
  }
  Serial.println("w: write data");
  Serial.println("q: quit");
  Serial.println("");
  Serial.println("(*) values taken from default settings");
  Serial.println("");
  Serial.println("<------------------------------------------------------>");
  Serial.println("");
}

void loadFlashData(t_flashData* flashData, t_flashRawData* flashRawData){
  byte* rawData = (byte*)flashRawData;
  Serial.println("EEPROM load data length: " + String(sizeof(t_flashRawData)));
  for(int i=0; i<sizeof(t_flashRawData); i++){
    rawData[i] = EEPROM.read(i);
  }
  flashData->animalID = getFlashInt(flashRawData->animalID);
  flashData->herdeID  = getFlashInt(flashRawData->herdeID);
  flashData->mobileAPN = getFlashString(flashRawData->mobileAPN, sizeof(flashRawData->mobileAPN));
  flashData->mobilePASSWD = getFlashString(flashRawData->mobilePASSWD, sizeof(flashRawData->mobilePASSWD));
  flashData->mobilePIN = getFlashString(flashRawData->mobilePIN, sizeof(flashRawData->mobilePIN));
  flashData->mobileUSER = getFlashString(flashRawData->mobileUSER, sizeof(flashRawData->mobileUSER));
  flashData->pushDataWebAddress = getFlashString(flashRawData->pushDataWebAddress, sizeof(flashRawData->pushDataWebAddress));
}

String getFlashString(byte* buffer, int len){
  int i = 0;
  String result = "";
  while(i < len){
    if(buffer[i] == 0xff) break;
    result += (char)(buffer[i]);
    i++;
  }
  return result;
}

void setFlashString(byte* buffer, int len,const String value){
  int i = 0;
  int p = 0;
  int l = value.length();
  byte buf, ch;
  while(i < len){
    if (p < l){
      ch = (byte)(value.charAt(p));
      p++;
    } else { ch = 0xff; }
    buffer[i] = ch;
    i++;
  }
}

int16_t getFlashInt(byte* buffer){
  return (int16_t)((buffer[0] << 8) | buffer[1]);
}

void setFlashInt(byte* buffer, int16_t value){
  buffer[0] = (byte)(value >> 8);
  buffer[1] = (byte)(value & 0xff);
}

void clearFlashString(byte* buffer, int len){
  for(int i = 0; i<len; i++) { buffer[i] = 0xff; }
}

String readNewValue(const String prompt, char terminatior, bool* timeout){
  String NewValue = "";
  while(Serial.available() > 0){ Serial.read(); }
  Serial.println("set new " + prompt + ":");
  Serial.setTimeout(60000);
  int t = millis();
  *timeout = false;
  NewValue = Serial.readStringUntil(terminatior);
  if ((millis() - t) >= 60000) *timeout = true;
  Serial.setTimeout(100);
  delay(1);
  while(Serial.available() > 0){ Serial.read(); delay(1);}
  return(NewValue);
}

char getSerialKey(void){
  while(Serial.available() == 0){
    delay(1);
  }
  char ch = (char)Serial.read();
  while(Serial.available() > 0){ Serial.read(); delay(1);}
  return ch;
}

_D(
void printFlashString(byte* buffer, int len){
  int x = 0;
  for(int i = 0; i<len; i++) {
    if( x > 0 ) {Serial.print(", "); }
    Serial.print("0x" + String(buffer[i], HEX));
    x++;
    if( x >= 16 ){ Serial.println(""); x = 0; }
  }
  if( x < 16 ){ Serial.println("");}
}
)

//unint16_t
