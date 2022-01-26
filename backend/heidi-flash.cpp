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
t_heidiMenu currentMenu = HEIDI_MENU_MAIN;

void setupFlashData(bool powerOnReset){
  if (EEPROM.begin(sizeof(t_flashRawData) * 2)){
    if (!loadFlashData){ loadDefaultData(&flashData); }
    if (powerOnReset) { handleHook(); }
    EEPROM.end();
  }
}

void loadDefaultData(t_flashData* flashData){
  flashData->animalID = HEIDI_ANIMAL;
  flashData->herdeID = HEIDI_HERDE;
  flashData->pushDataWebAddress = HEIDI_SERVER_PUSH_URL;
  flashData->mobileAPN = HEIDI_MOBILE_APN;
  flashData->mobileUSER = HEIDI_MOBILE_USER;
  flashData->mobilePASSWD = HEIDI_MOBILE_PASSWD;
  flashData->mobilePIN = HEIDI_SIM_PIN;
}

void handleHook(){
  int t = millis();
  #if DEBUG_LEVEL == DEBUG_LEVEL_0
  Serial.begin(115200);
  #endif
  Serial.println("enter hook: press 'x'");
  String Rs232in = "";
  Serial.flush();
  while((Rs232in.indexOf("x") == -1) && (millis()-t < 10000)){
    if (Serial.available() > 0) {Rs232in += (char)Serial.read();}
    delay(1);
  }
  if (Rs232in.indexOf('x') != -1){
    Serial.println("hook entered.");
    loadFlashData(&flashData, &flashRawData);
    char key = ' ';
    bool leave = false;
    while(!leave){
      printMenu(&flashData);
      delay(100);
      key = getSerialKey();
      bool timeout = false;
      leave = handleMenu(key);
      for(int i=0; i<5; i++) { Serial.println(""); }
    }
    Serial.println("leave hook");
  } else {
    Serial.println("start Heidi - application");
  }
  delay(100);
  #if DEBUG_LEVEL == DEBUG_LEVEL_0
  Serial.end();
  #endif
}

void printMenu(t_flashData* flashData){
  switch(currentMenu) {
    case HEIDI_MENU_MAIN:{
      printMainMenu(flashData);
      break;
    }
    case HEIDI_MENU_IDENTITY:{
      printIdentityMenu(flashData);
      break;
    }
    case HEIDI_MENU_WEB_INTERFACE:{
      printWebInterfaceMenu(flashData);
      break;
    }
    default:
      break;

  }
}
bool handleMenu(char key){
  switch(currentMenu) {
    case HEIDI_MENU_MAIN:{
      return handleMainMenu(key);
      break;
    }
    case HEIDI_MENU_IDENTITY:{
      return handleIdentityMenu(key);
      break;
    }
    case HEIDI_MENU_WEB_INTERFACE:{
      return handleWebInterfaceMenu(key);
      break;
    }
    default:
      break;
  }
  return false;
}

void printMainMenu(t_flashData* flashData){
  Serial.println("");
  Serial.println("<----------------- Heidi main menu ----------------->");
  Serial.println("");
  Serial.println("1: identity settings");
  Serial.println("2: web interface settings");
  Serial.println("3: test measuring - not implemented yet");
  Serial.println("");
  Serial.println("w: write data to flash");
  printFooter();
}
/*
 * returns "true" if "quit" was chosen
 */
bool handleMainMenu(char key){
  switch(key) {
    case '1':{
      currentMenu = HEIDI_MENU_IDENTITY;
      break;
    }
    case '2':{
      currentMenu = HEIDI_MENU_WEB_INTERFACE;
      break;
    }
    case 'w':{
      writeFlashData(&flashData, &flashRawData);
      break;
    }
    case 'q':{
      return true;
    }
    default:
      break;
  }
  return false;
}

void printIdentityMenu(t_flashData* flashData){
  Serial.println("");
  Serial.println("<----------------- Heidi identity settings ----------------->");
  Serial.println("");
  Serial.println("a: set herd   ID");
  Serial.println("   \"" + intString4(flashData->herdeID) + "\"");
  Serial.println("b: set animal ID");
  Serial.println("   \"" + intString4(flashData->animalID) + "\"");
  printFooter();
}

bool handleIdentityMenu(char key){
  bool timeout = false;
  switch(key) {
    case 'a':{
      String NewValue = readNewValue("herd ID", &timeout);
      if (!timeout) { flashData.herdeID = int16_t(NewValue.toInt()); }
      break;
    }
    case 'b':{
      String NewValue = readNewValue("animal ID", &timeout);
      if (!timeout) { flashData.animalID = int16_t(NewValue.toInt()); }
      break;
    }
    case 'q':{
      currentMenu = HEIDI_MENU_MAIN;
      break;
    }
    default:
      break;
  }
  return false;
}


void printWebInterfaceMenu(t_flashData* flashData){
  Serial.println("");
  Serial.println("<----------------- Heidi web interface settings ----------------->");
  Serial.println("");
  Serial.println("a: set push data web address");
  Serial.println("   \"" + flashData->pushDataWebAddress + "\"");
  Serial.println("b: set mobile APN");
  Serial.println("   \"" + flashData->mobileAPN + "\"");
  Serial.println("c: set mobile USER");
  Serial.println("   \"" + flashData->mobileUSER + "\"");
  Serial.println("d: set mobile PASSWD");
  Serial.println("   \"" + flashData->mobilePASSWD + "\"");
  Serial.println("e: set SIM PIN");
  Serial.println("   \"" + flashData->mobilePIN + "\"");
  printFooter();
}

bool handleWebInterfaceMenu(char key){
  bool timeout = false;
  switch(key) {
    case 'a':{
      String NewValue = readNewValue("push data web address", &timeout);
      if (!timeout) { flashData.pushDataWebAddress = NewValue; }
      break;
    }
    case 'b':{
      String NewValue = readNewValue("mobile APN", &timeout);
      if (NewValue != "") { flashData.mobileAPN = NewValue; }
      break;
    }
    case 'c':{
      String NewValue = readNewValue("mobile USER", &timeout);
      if (!timeout) { flashData.mobileUSER = NewValue; }
      break;
    }
    case 'd':{
      String NewValue = readNewValue("mobile PASSWD", &timeout);
      if (!timeout) { flashData.mobilePASSWD = NewValue; }
      break;
    }
    case 'e':{
      String NewValue = readNewValue("mobile PIN", &timeout);
      if (!timeout) { flashData.mobilePIN = NewValue; }
      break;
    }
    case 'q':{
      currentMenu = HEIDI_MENU_MAIN;
      break;
    }
    default:
      break;
  }
  return false;
}


void printFooter(void){
  if(currentMenu == HEIDI_MENU_MAIN){
    Serial.println("q: quit");
  } else {
    Serial.println("q: back");
    Serial.println("");
    Serial.println("(*) value taken from default settings");
  }
  Serial.println("");
  Serial.println("<------------------------------------------------------>");
  Serial.println("");
}

bool loadFlashData(t_flashData* flashData, t_flashRawData* flashRawData){
  bool good = false;
  int  p = 0;
  byte* rawData = (byte*)flashRawData;
  _D(DebugPrintln("EEPROM load data length: " + String(sizeof(t_flashRawData)), DEBUG_LEVEL_2);)
  for(int t=0; t<2; t++){
    for(int i=0; i<sizeof(t_flashRawData); i++){
      rawData[i] = EEPROM.read(p);
      p++;
    }
    if (checkData((t_flashRawData*)rawData)) { good = true; break; }
    _D(DebugPrintln("EEPROM data set " + String(t) + " not valid!", DEBUG_LEVEL_1);)
  }
  if(good) {
    flashData->animalID = getFlashInt(flashRawData->animalID);
    flashData->herdeID  = getFlashInt(flashRawData->herdeID);
    flashData->mobileAPN = getFlashString(flashRawData->mobileAPN, sizeof(flashRawData->mobileAPN));
    flashData->mobilePASSWD = getFlashString(flashRawData->mobilePASSWD, sizeof(flashRawData->mobilePASSWD));
    flashData->mobilePIN = getFlashString(flashRawData->mobilePIN, sizeof(flashRawData->mobilePIN));
    flashData->mobileUSER = getFlashString(flashRawData->mobileUSER, sizeof(flashRawData->mobileUSER));
    flashData->pushDataWebAddress = getFlashString(flashRawData->pushDataWebAddress, sizeof(flashRawData->pushDataWebAddress));
  }
  return good;
}

void writeFlashData(t_flashData* flashData, t_flashRawData* flashRawData){
  byte* rawData = (byte*)flashRawData;
  int  p = 0;
  _D(DebugPrintln("EEPROM write data length: " + String(sizeof(t_flashRawData)), DEBUG_LEVEL_2);)
  setFlashInt(flashRawData->animalID, flashData->animalID);
  setFlashInt(flashRawData->herdeID, flashData->herdeID);
  setFlashString(flashRawData->mobileAPN, sizeof(flashRawData->mobileAPN), flashData->mobileAPN);
  setFlashString(flashRawData->mobilePASSWD, sizeof(flashRawData->mobilePASSWD), flashData->mobilePASSWD);
  setFlashString(flashRawData->mobilePIN, sizeof(flashRawData->mobilePIN), flashData->mobilePIN);
  setFlashString(flashRawData->mobileUSER, sizeof(flashRawData->mobileUSER),flashData->mobileUSER);
  setFlashString(flashRawData->pushDataWebAddress, sizeof(flashRawData->pushDataWebAddress), flashData->pushDataWebAddress);
  for(int t=0; t<2; t++){
    for(int i=t; i<sizeof(t_flashRawData); i++){
      if(EEPROM.read(i) != rawData[i]){ EEPROM.write(p,rawData[i]); }
      p++;
    }
    setCheckSum((t_flashRawData*)rawData);
    if (!EEPROM.commit()){_D(DebugPrintln("EEPROM writing data set " + String(t) + " failed!", DEBUG_LEVEL_1);)}
  }
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

String readNewValue(const String prompt, bool* timeout){
  String NewValue = "";
  char c = 0;
  while(Serial.available() > 0){ Serial.read(); }
  Serial.println("set new " + prompt + ":");
  int t = millis();
  *timeout = false;
  while ((millis() -t < 60000) && (c != 10) && (c != 13)){
    while (Serial.available() > 0){
      c = Serial.read();
      if (c >= 32) {
        Serial.print(c);
        NewValue += c;
      }
    }
    delay(1);
  }
  Serial.println("");
  if ((millis() - t) >= 60000) *timeout = true;
  Serial.setTimeout(100);
  delay(1);
  while(Serial.available() > 0){ Serial.read(); delay(1);}
  NewValue.trim();
  return(NewValue);
}

char getSerialKey(void){
  while(Serial.available() > 0){ Serial.read(); delay(1);}
  while(Serial.available() == 0){
    delay(1);
  }
  char ch = (char)Serial.read();
  while(Serial.available() > 0){ Serial.read(); delay(1);}
  return ch;
}

bool checkData(t_flashRawData* flashRawData){
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t* buffer;
  buffer = (uint8_t*)flashRawData;
  int l = sizeof(t_flashRawData)-2;
  for(int i = 0; i < l; i++){
    a += buffer[i];
    b += a;
  }
  if(a == buffer[l]){
    if(b == buffer[l+1]){
      return true;
    }
  }
  return false;
}

void setCheckSum(t_flashRawData* flashRawData){
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t* buffer;
  buffer = (uint8_t*)flashRawData;
  int l = sizeof(t_flashRawData)-2;
  for(int i = 0; i < l; i++){
    a += buffer[i];
    b += a;
  }
  a = buffer[l];
  b = buffer[l+1];
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
