/*
 * heidi-lora.cpp
 *
 *  Created on: 31.01.2022
 *      Author: frank
 */
#include <Arduino.h>
#include <stdlib.h>
#include "RS-FEC.h"
#include "heidi-debug.h"
#include "heidi-defines.h"
#include "heidi-data.h"
#include "heidi-lora.h"
#include "heidi-sys.h"
#ifdef USE_OLED
#include "SSD1306.h"
#ifdef LORA_V1_3_OLED
#define DISPLAY_SDA 21
#define DISPLAY_SCL 22
#endif
#ifdef LORA_V1_1_OLED
#define DISPLAY_SDA 4
#define DISPLAY_SCL 15
#endif
#define DISPLAY_RST 16
#define DISPLAY_ADD 0x3c
SSD1306 display(DISPLAY_ADD, DISPLAY_SDA, DISPLAY_SCL);
#endif
#ifdef USE_LORA

TaskHandle_t LoRaTaskHandle = NULL;
bool stopLoraTask = false;
loraSetupResult_t setupLoraRc = LORA_SETUP_NOT_DONE;
uint32_t loraTaskWaterMark = 0;
uint32_t startTaskTime = 0;
RS::ReedSolomon<CONF_DATA_C_PAYLOAD_SIZE, LORA_ECC_LENGTH> rsc;
RS::ReedSolomon<DATA_SET_LEN, LORA_ECC_LENGTH> rsd;


#ifdef HEIDI_GATEWAY
void loraGatewayTask(void *pvParameters) {
  if (SetupLoRa()){
    int loops = 2;
    if (isFixPointCycle()){ loops = 5; }
    for(int k=0; k<loops; k++){ //maximum 5 loops
      if((k == 0) && (!isFixPointCycle()) && (heidiConfig->clients != 0)){
        loraLoadData();
      } else {
        if (!endLoraTask()) { loraGeneralCall(); }
        if ((heidiConfig->clientsNeedConf != 0) || !isFixPointCycle()){ loraBroadcastSettings(); }
        if((heidiConfig->clients == 0)){
          for(int i=0; i<100; i++){
            if (endLoraTask()) { break; }
            pause(10);
          }
        }
      }
      if (endLoraTask()) { break; }
    }
    CloseLoRa();
  } else { _D(DebugPrintln("LORA: device not available ",   DEBUG_LEVEL_1);) }
  _D(DebugPrintln("LORA: end task ",   DEBUG_LEVEL_1);)
  loraTaskWaterMark = uxTaskGetStackHighWaterMark(NULL);
  LoRaTaskHandle = NULL;
  vTaskDelete(NULL); //NULL = delete calling task
}
#else //HEIDI_GATEWAY
/*
 * LoRa Client task just listens and acts if there was something
 */
void loraClientTask(void *pvParameters) {
  uint8_t recBuffer[LORA_CLIENT_RECEIVE_PKG_MAX_LEN];
  if (SetupLoRa()){
    _D(DebugPrintln("LORA: listen for requests",   DEBUG_LEVEL_2);)
    while(!endLoraTask()){
      int recLen = loraWaitForDataPackage(LORA_PKG_ID_ALL_PKG, recBuffer, LORA_CLIENT_RECEIVE_PKG_MAX_LEN, 500);
      if(recLen <= 2) { continue; } //there should be more
      switch(recBuffer[0]) {
        case LORA_PKG_ID_GEN_CALL:{
          _D(
            uint32_t mask = _copyBufferToUInt32(recBuffer, 1);
            DebugPrintln("LORA: got general call: 0x" + String(mask, HEX),   DEBUG_LEVEL_2);
          )
          if (recLen >= LORA_PKG_GEN_CALL_LEN) { loraAnswerGenCall(recBuffer); }
          break;
        }
        case LORA_PKG_ID_CONFIG:{
          _D(DebugPrintln("LORA: got config", DEBUG_LEVEL_2);)
          if(loraDecodeConfigData(recBuffer)){
            _DD(_PrintHeidiConfig(DEBUG_LEVEL_3););
          }
          break;
        }
        case LORA_PKG_ID_DATA_REQ:{
          _D(DebugPrintln("LORA: got data request", DEBUG_LEVEL_2);)
          if (recBuffer[2] == recBuffer[3]){
            int maxSetsToSend = recBuffer[2];
            int setsToSend = packUpDataSets();
            _DD(DebugPrintln("LORA: " + String(maxSetsToSend) + " data sets allowed to send.", DEBUG_LEVEL_3);)
            loraSendDataSets((maxSetsToSend < setsToSend) ? maxSetsToSend : setsToSend);
          } _D( else {DebugPrintln("LORA: invalid data request message", DEBUG_LEVEL_1);} )
          break;
        }
      }
    }
    CloseLoRa();
  }
  _D(DebugPrintln("LORA: end task ", DEBUG_LEVEL_1);)
  loraTaskWaterMark = uxTaskGetStackHighWaterMark(NULL);
  LoRaTaskHandle = NULL;
  vTaskDelete(NULL); //NULL = delete calling task
}
#endif

/*
 * starts the LoRa handling task
 *  - no return code, because there is nothing to do for main task, if this fails
 */
void setupLoraTask(void){
  if (LoRaTaskHandle != NULL){
    _D(DebugPrintln("LORA: LoRa task already running. ", DEBUG_LEVEL_1);)
    return;
  }
  loraTaskWaterMark = 0;
  startTaskTime = millis();
  #ifdef HEIDI_GATEWAY
  BaseType_t rc = xTaskCreatePinnedToCore(loraGatewayTask, "HeidiLoraTask", LORA_TASK_HEAP_SIZE, NULL, 1, &LoRaTaskHandle, 0);
  #else
  BaseType_t rc = xTaskCreatePinnedToCore(loraClientTask, "HeidiLoraTask", LORA_TASK_HEAP_SIZE, NULL, 1, &LoRaTaskHandle, 0);
  #endif
  if (rc != pdPASS){
    _D(DebugPrintln("LORA: unable to create LoRa task. " + String(rc), DEBUG_LEVEL_1);)
    LoRaTaskHandle = NULL;
  } _D( else { DebugPrintln("LORA: Task up and running. ", DEBUG_LEVEL_2); })
}

/*
 * stops the LoRa handling task
 */
void closeLoraTask(void){
  if (LoRaTaskHandle != NULL){
    if (isFixPointCycle()){
      while ((LoRaTaskHandle != NULL) && ((millis() - startTaskTime) < FIX_POINT_MIN_MS)){ vTaskDelay(100); }
    }
    stopLoraTask = true;
    while (LoRaTaskHandle != NULL){ vTaskDelay(100); }
  } _DD(else { DebugPrintln("LORA: Task was not running (anymore) ", DEBUG_LEVEL_3); })
  _D(DebugPrintln("LORA: task closed, heap water mark: " + String(loraTaskWaterMark),DEBUG_LEVEL_2);)
  loraTaskWaterMark = 0;
}

/*
 * sets up LoRa hardware
 */
bool SetupLoRa(void){
  _D(DebugPrint("setup LoRa at " + String(BAND/1E6,2) + "MHz ", DEBUG_LEVEL_1);)
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND)) {
    _D(DebugPrintln(" failed!", DEBUG_LEVEL_1);)
    return false;
  }
  _DD(DebugPrintln("Configuring LoRa", DEBUG_LEVEL_3);
      DebugPrintln(" - spreading factor: " + String(LORA_SPERADING_F), DEBUG_LEVEL_3);)
  LoRa.setSpreadingFactor(LORA_SPERADING_F);
  _DD(DebugPrintln(" - signal bandwidth: " + String(LORA_BANDWIDTH), DEBUG_LEVEL_3);)
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  _DD(DebugPrintln(" - code rate: 4/" + String(LORA_CR_DENOMNTR), DEBUG_LEVEL_3);)
  LoRa.setCodingRate4(LORA_CR_DENOMNTR);
  _DD(DebugPrintln(" - preamble length: " + String(LORA_PREAMB_LEN), DEBUG_LEVEL_3);)
  LoRa.setPreambleLength(LORA_PREAMB_LEN);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.disableCrc();
  _D(DebugPrintln(".. done", DEBUG_LEVEL_1);)
  return true;
}

void CloseLoRa(void){
  LoRa.end();
  LoRa.sleep();
  SPI.end();
}

bool endLoraTask(){
  if ((millis() - startTaskTime) >= LORA_MAX_ONAIR_TIME_MS) { return true; }
  return stopLoraTask;
}
/*
 * waits for ack / nack by a fix timeout: LORA_ACK_TIMEOUT_MS
 *   *rssi: pointer for write RSSI to, defaults to NULL
 *   returns true, if a valid ACK was recognized otherwise false
 */
bool loraWaitForACK(int* rssi){ //add return of animal ID
  int t = millis();
  bool rc = false;
  uint8_t buffer[LORA_PKG_ACK_LEN];
  if (rssi != NULL) { *rssi = LORA_NO_RSSI; }
  #if LORA_SPERADING_F != LORA_SPERADING_F
  LoRa.setSpreadingFactor(LORA_ACK_SPREAD_F);
  #endif
  #if LORA_ACK_PREA_LEN != LORA_ACK_PREA_LEN
  LoRa.setPreambleLength(LORA_ACK_PREA_LEN);
  #endif
  while (millis() - t <= LORA_ACK_TIMEOUT_MS){
    int packetSize = 0;
    packetSize = loraWaitForDataPackage(LORA_PKG_ID_ACK, buffer, sizeof(buffer), LORA_ACK_TIMEOUT_MS, rssi); //oggo
    if (packetSize > 0) {
      if (rssi != NULL) { *rssi = LoRa.packetRssi(); }
      if (buffer[1] == animalID()){
        if(buffer[2] == LORA_ACK){
          rc = true;
          break;
        }
      }
    }
  }/*
  _DD(if (!rc) { DebugPrintln("got no ACK [" + String(millis()-t) + " ms]", DEBUG_LEVEL_3); }
      else  { DebugPrintln("got ACK [" + String(millis()-t) + " ms]", DEBUG_LEVEL_3); }
  )*/
  #if LORA_SPERADING_F != LORA_SPERADING_F
  LoRa.setSpreadingFactor(LORA_SPERADING_F);
  #endif
  #if LORA_ACK_PREA_LEN != LORA_ACK_PREA_LEN
  LoRa.setPreambleLength(LORA_PREAMB_LEN);
  #endif
  return rc;
}
/*
 * sends ack / nack
 *   recID: ID of client/gateway who needs it
 *   ack;   if true - ACK, if false - NACK
 *   returns true, if successful
 */

bool loraSendACK(uint8_t recID, bool ack){
  uint8_t buffer[LORA_PKG_ACK_LEN];
  bool rc = false;
  _DD(int t = millis();)
  #if LORA_ACK_TX_PWR  != LORA_TX_POWER
  LoRa.setTxPower(LORA_ACK_TX_PWR);
  #endif
  #if LORA_SPERADING_F != LORA_SPERADING_F
  LoRa.setSpreadingFactor(LORA_ACK_SPREAD_F);
  #endif
  #if LORA_ACK_CR_DEN != LORA_CR_DENOMNTR
  LoRa.setCodingRate4(LORA_ACK_CR_DEN);
  #endif
  #if LORA_ACK_PREA_LEN != LORA_ACK_PREA_LEN
  LoRa.setPreambleLength(LORA_ACK_PREA_LEN);
  #endif
  buffer[0] = LORA_PKG_ID_ACK;
  buffer[1] = recID;
  buffer[2] = ack ? LORA_ACK : LORA_NACK;
  rc = loraSendPackage(buffer, sizeof(buffer));
  #if LORA_ACK_TX_PWR  != LORA_TX_POWER
  LoRa.setTxPower(LORA_TX_POWER);
  #endif
  #if LORA_SPERADING_F != LORA_SPERADING_F
  LoRa.setSpreadingFactor(LORA_SPERADING_F);
  #endif
  #if LORA_ACK_CR_DEN != LORA_CR_DENOMNTR
  LoRa.setCodingRate4(LORA_CR_DENOMNTR);
  #endif
  #if LORA_ACK_PREA_LEN != LORA_ACK_PREA_LEN
  LoRa.setPreambleLength(LORA_PREAMB_LEN);
  #endif
  return rc;
}
/*
 * waits for a LoRa data transmission and copies the payload into buffer
 *   pkgID:      expected package ID
 *   buffer:     buffer where to write data
 *   bufferSize: size of buffer
 *   tmOutMs:    maximum wait time in milliseconds
 *   *rssi:      pointer for write RSSI to, defaults to NULL
 *
 *   returns bytes read if package was found, otherwise -1
 *   the buffer contains the whole message including msgID on buffer[0]
 */
int loraWaitForDataPackage(loraPackageID_t pkgID, uint8_t *buffer, int bufferSize, int tmOutMs, int *rssi, float *snr){
  int t = millis();
  if (rssi != NULL) { *rssi = LORA_NO_RSSI; }
  if (snr  != NULL) { *snr  = 0; }
  if (buffer == NULL) { return LORA_RC_OK; }
  while (millis() - t + 5 <= tmOutMs){
    int packetSize = 0;
    while(packetSize == 0){
      packetSize = LoRa.parsePacket();
      if(packetSize == 0) { pause(10); }
      if (millis() - t > LORA_ACK_TIMEOUT_MS) {
        //_DD(DebugPrintln("LORA package [" + String(pkgID, HEX) + "] timed out after " + String(millis()-t) + " ms", DEBUG_LEVEL_3);)
        break;
      }
    }
    if (packetSize > 0) {
      if (rssi != NULL) { *rssi = LoRa.packetRssi(); }
      if (snr != NULL)  { *snr = LoRa.packetSnr(); }
      if (LoRa.available() > 1){
        uint8_t readPgkID = LoRa.read();
        if((readPgkID == pkgID) || (pkgID == LORA_PKG_ID_ALL_PKG)){
          int i = 0;
          #ifndef HEIDI_GATEWAY //if we are client, we don't need to load data sets
          if((readPgkID != LORA_PKG_ID_ONE_SET) && (readPgkID != LORA_PKG_ID_TWO_SET))
          #endif
          {
            buffer[i++] = readPgkID;
            while((i<bufferSize) && (LoRa.available() > 0)){
              buffer[i++] = LoRa.read();
            }
          }
          //_DD(DebugPrintln("LORA got package [" + String(pkgID, HEX) + "] length " + String(i) + "; " + hexString2(buffer[0]) + ", " + hexString2(buffer[1]), DEBUG_LEVEL_1); )
          if (LoRa.available()>0){
            _D(
                if((readPgkID != LORA_PKG_ID_ONE_SET) && (readPgkID != LORA_PKG_ID_TWO_SET))
                {DebugPrintln("LORA package [" + String(readPgkID, HEX) + "] length " +String(LoRa.available() + i) + " exceeds buffer size " + String(bufferSize), DEBUG_LEVEL_1); }
              )
            while (LoRa.available()>0) { LoRa.read(); }
          }
          #ifndef HEIDI_GATEWAY
          if((buffer[1] == animalID()) || (readPgkID == LORA_PKG_ID_GEN_CALL) || (readPgkID == LORA_PKG_ID_CONFIG)) { return i; }
          #else
          return i;
          #endif
        }
      }
    }
  }
  return LORA_RC_TIME_OUT;
}

/*
 * sends a message (2 tries)
 *   buffer: where the message is stored
 *   size:   lenght of message
 *   returns true if successful, false is device was busy.
 */

bool loraSendPackage(uint8_t *buffer, int size){
#if 0
  _DD(
    DebugPrint("LORA send: 0x" + String(buffer[0], HEX), DEBUG_LEVEL_3);
    for(int i=1; i<size; i++){ DebugPrint(", 0x" + String(buffer[i], HEX), DEBUG_LEVEL_3); }
    DebugPrintln("", DEBUG_LEVEL_3);
  )
#endif
  for(int i=0; i<2; i++){
    if (LoRa.beginPacket()) {
      LoRa.write(buffer, size);
      LoRa.endPacket();
      return true;
    } else {_D(DebugPrintln("LORA: device busy", DEBUG_LEVEL_1);) pause(100); }
  }
  _D(DebugPrintln("LORA: device not available", DEBUG_LEVEL_1);)
  return false;
}
#ifdef HEIDI_GATEWAY
/*
 * sends a data request to client
 *   recID:   which client
 *   maxSets: maximum count of data sets may be sent
 *   minRSSI: minimum RSSI for transfer; usually -90 .. -120; 0 = no limit
 *
 *   Client will answer with a dataReady message which needs to be acknowledged.
 *   If all is fine, the function returns the amount of data sets the client intends to
 *   transfer, otherwise 0
 */
int loraRequestData(uint8_t recID, uint8_t maxSets, int minRSSI, float minSNR){
  uint8_t buffer[LORA_PKG_DATA_RDY_LEN];
  bool rc = false;
  int rssi = minRSSI-1;
  float snr = 0.0;
  buffer[0] = LORA_PKG_ID_DATA_REQ;
  buffer[1] = recID;
  buffer[2] = maxSets;
  buffer[3] = maxSets;
  if(loraSendPackage(buffer, LORA_PKG_DATA_REQ_LEN)){
    if (loraWaitForDataPackage(LORA_PKG_ID_DATA_RDY, buffer, LORA_PKG_DATA_RDY_LEN, 30, &rssi, &snr) != LORA_RC_TIME_OUT){
      _DD(DebugPrintln("LORA got pkg on request, RSSI: " + String(rssi)  + ", SNR: " + String(snr, 4), DEBUG_LEVEL_3);)
      if(buffer[1] == recID){ //check client ID
        if((rssi >= minRSSI) || (minRSSI == LORA_NO_RSSI)){
          if((snr >= minSNR) || (minSNR == LORA_NO_SNR)){
            if(buffer[2] == buffer[3]){
              if(loraSendACK(recID, true)) { return (int)buffer[2]; }
            }
          } _DD( else {DebugPrintln("LORA got too low SNR to request", DEBUG_LEVEL_3);} )
        } _DD( else {DebugPrintln("LORA got too low RSSI to request", DEBUG_LEVEL_3);} )
      } _DD( else {DebugPrintln("LORA got wrong answer to request", DEBUG_LEVEL_3);} )
    } _DD( else {DebugPrintln("LORA got no answer to request", DEBUG_LEVEL_3);} )
  }
  return 0;
}

/*
 * returns true if a general call should be performed
 */
bool loraDoGeneralCall(){
  if(heidiConfig->clients == 0){
    _DD( DebugPrintln("LORA: heidiConfig->clients == 0 -> general call", DEBUG_LEVEL_3);)
    return true;
  }
  if(isFixPointCycle()){
    _DD( DebugPrintln("LORA: isFixPointCycle() -> general call", DEBUG_LEVEL_3);)
    return true;
  }
  _DD( DebugPrintln("LORA: no general call", DEBUG_LEVEL_3);)
  return false;
}

/*
 * performs a general call, waits for answers and sets up the found new clients
 * in heidiConfig
 */
int loraGeneralCall(void){
  uint8_t buffer[LORA_PKG_GEN_CALL_LEN];
  int bytesAvailable;
  int unknownClients = 0;
  int newClients = 0;
  for(int i = 0; i<HEIDI_MAX_CLIENTS; i++) { if(((heidiConfig->clients >> i) & 0x01) == 0) { unknownClients++; }}
  buffer[0] = LORA_PKG_ID_GEN_CALL;
  _D( DebugPrintln("LORA: general call...", DEBUG_LEVEL_1);)
  _copyUint32toBuffer(buffer, 1, heidiConfig->clients);
  _copyUint32toBuffer(buffer, 5, heidiConfig->clients);
  if(loraSendPackage(buffer, LORA_PKG_GEN_CALL_LEN)){
    _DD(int t1 = millis();)
    for(int i=0; i<unknownClients; i++){
      int t = millis();
      if (loraWaitForDataPackage(LORA_PKG_ID_GEN_ANSW, buffer, LORA_PKG_GEN_ANSW_LEN, LORA_GEN_CALL_INTERVAL) != LORA_RC_TIME_OUT){
        _DD(DebugPrintln("LORA: got answer after " + String(millis() - t1) + "ms,", DEBUG_LEVEL_3);)
        if(buffer[1] == buffer[2]){
          uint32_t newClient = 0x01 << (buffer[1] - HEIDI_FIRST_CLIENT); //bit 0 represents HEIDI_FIRST_CLIENT - client address 0x01 may be the gateway
          heidiConfig->clients |= newClient;
          newClients++;
          uint16_t cCRC = _copyBufferToUInt16(buffer, 3);
          _D( DebugPrintln("LORA: new client: 0x" + LenTwo(String(buffer[1], HEX)), DEBUG_LEVEL_2); )
          if (cCRC != clientConfigCRC(&heidiConfig->c)){
            heidiConfig->clientsNeedConf |= newClient;
            _DD( DebugPrintln("LORA: new client needs new config.", DEBUG_LEVEL_3);)
          }
        }
        pause(LORA_GEN_CALL_INTERVAL - (millis()-t));
      }
      //_DD(else {_DD(DebugPrintln("LORA: nothing after " + String(millis() - t1) + "ms,", DEBUG_LEVEL_3);)})
    }
    _D( if(newClients > 0) { DebugPrintln("LORA: " + String(newClients) + " new clients.", DEBUG_LEVEL_1); })
  }
  return newClients;
}
/*
 * send the client settings with reed solomon protection
 *
 */
bool loraBroadcastSettings(void){
  uint8_t buffer[CONF_DATA_C_PAYLOAD_SIZE];
  uint8_t encoded[LORA_PKG_CONFIG_LEN];
  _D( DebugPrintln("LORA: broadcast settings...", DEBUG_LEVEL_1);)
  if (!pushSettingsToBuffer(buffer, sizeof(buffer), &heidiConfig->c)) { _D(DebugPrintln("LORA: push settings failed.", DEBUG_LEVEL_1);) return false; }
  encoded[0] = LORA_PKG_ID_CONFIG;
  rsc.Encode(buffer, &encoded[1]);
  return loraSendPackage(encoded, sizeof(encoded));
}

void loraLoadData(void){
  uint8_t lastClientTriggerd = heidiConfig->lastClientTrig;
  _D(int t = millis();)
  _D( DebugPrintln("LORA: load data...", DEBUG_LEVEL_1);)
  while((!endLoraTask()) && (freeDataSets() >= MIN_FREE_SETS_FOR_REQUEST)){
    lastClientTriggerd++;
    if(lastClientTriggerd >= HEIDI_MAX_CLIENTS) { lastClientTriggerd = 0; }
    if(((heidiConfig->clients >> lastClientTriggerd) & 0x01) == 0x01){
      uint8_t clientID = lastClientTriggerd + HEIDI_FIRST_CLIENT;
      _DD(DebugPrintln("LORA: request data from client " + String(clientID), DEBUG_LEVEL_3);)
      int dataSets = loraRequestData(clientID, LORA_MAX_PKGS_PER_TRANSMISSION, LORA_MIN_TRANS_RSSI, LORA_MIN_TRANS_SNR);
      if(dataSets > 0){
        loraLoadDataSets(clientID, dataSets);
      } else {
        //_DD(DebugPrintln("LORA: no data from client " + String(clientID) + " [" + String(millis()-t) + " ms]", DEBUG_LEVEL_3);)
        pause(20);
      }
    }
    if (lastClientTriggerd == heidiConfig->lastClientTrig) { break; } //we are through
  }
  heidiConfig->lastClientTrig = lastClientTriggerd;
}

void loraLoadDataSets(uint8_t clientID, int dataSets){
  uint8_t buffer[LORA_PKG_ONE_SET_LEN];
  int maxWaitTime = (LORA_DTA_TIMEOUT_MS * LORA_MAX_TRIES * LORA_MAX_PKG_LOSS);
  int packetSize;
  int lastSetNo = -1;
  _D(int t = millis(); int setsGotten = 0;)
  while((packetSize = loraWaitForDataPackage(LORA_PKG_ID_ONE_SET, buffer, sizeof(buffer), maxWaitTime)) != LORA_RC_TIME_OUT){
    bool ack = false;
    uint8_t setNo = buffer[1];
    t_SendData data;
    int rc = rsd.Decode(&buffer[2], (uint8_t*)&data);
    if (rc > 0) {_D(DebugPrintln("LORA: Decoding data set [" + String(setNo + 1) + "/" + String(dataSets) + "] failed: " + String(rc), DEBUG_LEVEL_1);)
    } else {
      ack = true;
      if(lastSetNo != setNo){
        _DD(DebugPrint("LORA: got data set [" + String(setNo + 1) + "/" + String(dataSets) + "] : ", DEBUG_LEVEL_3); _PrintShortSet(&data, -1, DEBUG_LEVEL_3);)
        addDataSet(&data);
        lastSetNo = setNo;
        _D(setsGotten++;)
      }
      _DD(else {DebugPrintln("LORA: got set " + String(setNo) + " twice", DEBUG_LEVEL_3);})
    }
    loraSendACK(clientID, ack);
    if((dataSets - setNo) < LORA_MAX_PKG_LOSS){
      //avoid endless or zero wait time when got a wrong setNo and (sets - setNo) turns to negative
      if((dataSets - setNo) >= 1) { maxWaitTime = (LORA_DTA_TIMEOUT_MS * LORA_MAX_TRIES * (dataSets - setNo)); }
    }
  }
  _D(DebugPrintln("LORA: got" + String(setsGotten) + " data sets from client " + String(clientID), DEBUG_LEVEL_2); setsGotten = 0;)
}

#else //HEIDI_GATEWAY
/*
 * send data sets
 *   - setsToSend = how many sets to send
 *     successfully sent data sets will be deleted from RTC memory
 *
 *   - returns successfully sent data sets
 */
int loraSendDataSets(int setsToSend){
  if ( setsToSend <= 0) { return 0; }
  int sccusessfullySent = 0;
  _D(int t = millis();) //oggo
  _DD(DebugPrintln("LORA: " + String(setsToSend) + " data sets ready to send.", DEBUG_LEVEL_3);)
  if(loraDataReady(setsToSend)) {
    _DD(DebugPrintln("LORA: start sending", DEBUG_LEVEL_3);)
    int bufferSize = sizeof(t_SendData) * setsToSend;
    t_SendData* buffer = (t_SendData*)malloc(bufferSize);
    if(buffer != NULL){
      setsToSend = getNextnDataSets(setsToSend, buffer, bufferSize);
      int failCnt = 0;
      for(int m = 0; m < setsToSend; m++){
        if(loraSendData(&buffer[m], m, LORA_MAX_TRIES)){
          _D(DebugPrint("LORA: package sent successfully [" + String(millis()-t) + " ms]: ", DEBUG_LEVEL_2);) //oggo
          _D(_PrintShortSet(&buffer[m], m, DEBUG_LEVEL_2);) //oggo
          eraseDataSet(&buffer[m]);
          failCnt = 0;
          sccusessfullySent++;
        } else { failCnt++; }
        if(failCnt == LORA_MAX_PKG_LOSS) { break; }
      }
      free(buffer);
    }
    _D(
      DebugPrint("LORA: " + String(sccusessfullySent) + " package", DEBUG_LEVEL_2);
      if(sccusessfullySent != 1){  DebugPrint("s", DEBUG_LEVEL_2); }
      DebugPrintln(" sent successfully", DEBUG_LEVEL_2);
     )
  }_D( else {DebugPrintln("LORA: send data ready message failed", DEBUG_LEVEL_1);} )
  return sccusessfullySent;
}
/*
 * send Data ready message
 *    dataSetCount - amount of data sets intended to send
 *    returns true if the message was acknowledged by gateway
 */
bool loraDataReady(uint8_t dataSetCount){
  uint8_t buffer[LORA_PKG_DATA_RDY_LEN];
  bool rc = false;
  buffer[0] = LORA_PKG_ID_DATA_RDY;
  buffer[1] = animalID();
  buffer[2] = dataSetCount;
  buffer[3] = dataSetCount;
  buffer[4] = 0; //currently not used
  if(loraSendPackage(buffer, LORA_PKG_DATA_RDY_LEN)){
    if(loraWaitForACK()){ return true; }
  }
  return false;
}
/*
 * answers a general call from gateway
 *   - recBuffer points to the (full) general call message (with message ID)
 */
void loraAnswerGenCall(uint8_t* recBuffer){
  uint8_t buffer[LORA_PKG_GEN_ANSW_LEN];
  uint32_t map1 = _copyBufferToUInt32(recBuffer, 1);
  uint32_t map2 = _copyBufferToUInt32(recBuffer, 5);
  _DD(int t1 = millis();)
  int zerosToMe = 0; //representing the wait states till answer
  bool needToAnswer = false;
  _D( DebugPrintln("LORA: check general call.", DEBUG_LEVEL_2);)
  if(map1 != map2) { _DD( DebugPrintln("LORA: client map invalid.", DEBUG_LEVEL_3);) return; }
  for(int i=0; i <= (animalID() - HEIDI_FIRST_CLIENT); i++){
    if(((map1 >> i) & 0x01) == 0){
      if(i == (animalID() - HEIDI_FIRST_CLIENT)){
        needToAnswer = true;
      } else { zerosToMe++; }
    }
  }
  if(needToAnswer){
    _D( DebugPrintln("LORA: answer to general call, wait " + String(zerosToMe) + " intervals.", DEBUG_LEVEL_2);)
    for(int i = 0; i < zerosToMe; i++){ pause(LORA_GEN_CALL_INTERVAL); }
    pause(25);
    buffer[0] = LORA_PKG_ID_GEN_ANSW;
    buffer[1] = animalID();
    buffer[2] = animalID();
    _copyUint16toBuffer(buffer, 3, clientConfigCRC(&heidiConfig->c));
    _DD(int t2 = millis();)
    loraSendPackage(buffer, LORA_PKG_GEN_ANSW_LEN);
    _DD(DebugPrintln("LORA: answer sent after " + String(t2-t1) + "ms, takes " + String(millis() - t2) + "ms,", DEBUG_LEVEL_3);)
  }
}
/*
 * decodes a LORA_PKG_ID_CONFIG message and sets new settings if needed
 *   - data points to the (full) message (with message ID)
 *   - returns true if heidi settings were changed
 */
bool loraDecodeConfigData(uint8_t* data){
  uint8_t buffer[CONF_DATA_C_PAYLOAD_SIZE];
  t_ConfigDataC confBuffer;
  if(rsc.Decode(&data[1], buffer) > 0){
    _D(DebugPrintln("LORA: Decoding config data failed.", DEBUG_LEVEL_1);)
    return false;
  }
  setSettingsFromBuffer(buffer, CONF_DATA_C_PAYLOAD_SIZE, &confBuffer);
  uint16_t newCRC = clientConfigCRC(&confBuffer);
  uint16_t oldCRC = clientConfigCRC(&heidiConfig->c);
  if(newCRC != oldCRC){
    _DD(DebugPrintln("LORA: set new config.[0x" + String(oldCRC, HEX) + " -> 0x" + String(newCRC, HEX) + "]", DEBUG_LEVEL_3);)
    if(setSettingsFromBuffer(buffer, CONF_DATA_C_PAYLOAD_SIZE, &heidiConfig->c)){
      _D(DebugPrintln("LORA: new config data successfully set.", DEBUG_LEVEL_2);)
      return true;
    } else {
      _D(DebugPrintln("LORA: set new config data failed.", DEBUG_LEVEL_1);)
      return false;
    }
  }
  _D(DebugPrintln("LORA: config keeps the same.", DEBUG_LEVEL_2);)
  return false;
}

/*
 * send the client data to gateway with reed solomon protection
 */
bool loraSendData(t_SendData* data, uint8_t pkgNo, int maxTries){
 bool ack = false;
  int tries = 0;
  uint8_t* buffer = (uint8_t*)data;
  uint8_t encoded[DATA_SET_LEN + LORA_ECC_LENGTH + 2]; //+2 = ID, pkgNo

  encoded[0] = LORA_PKG_ID_ONE_SET;
  encoded[1] = pkgNo;
  rsd.Encode(buffer, &encoded[2]);
  while((!ack) && (tries < maxTries)){
    _D(int t=millis();)
    if(loraSendPackage(encoded, sizeof(encoded))){
      _DD(DebugPrintln("LORA: data package send takes " + String(millis() - t) + "ms", DEBUG_LEVEL_3);)
      ack = loraWaitForACK(NULL);
      _DD(
        if (ack){ DebugPrintln("LORA: data package acknowledged.", DEBUG_LEVEL_3); }
        else { DebugPrintln("LORA: data package NOT acknowledged.", DEBUG_LEVEL_3); }
      )
      tries++;
    }
  }
  return ack;
}
#endif //else ifdef HEIDI_GATEWAY
#ifdef TEST_LORA
void TestLoRa(int maxTestDurationMS){
  int t = millis();
  if (LoRaTaskHandle != NULL){
    _D(DebugPrintln("LORA: LoRa task already running. ", DEBUG_LEVEL_1);)
    return;
  }
  stopLoraTask = false;
  setupLoraRc = LORA_SETUP_NOT_DONE;
  #ifdef HEIDI_GATEWAY
  BaseType_t rc = xTaskCreatePinnedToCore(loraTestGatewayTask, "loraTestGatewayTask", LORA_TASK_HEAP_SIZE, NULL, 1, &LoRaTaskHandle, 0);
  #else
  BaseType_t rc = xTaskCreatePinnedToCore(loraTestClientTask, "loraTestClientTask", LORA_TASK_HEAP_SIZE, NULL, 1, &LoRaTaskHandle, 0);
  #endif
  if (rc != pdPASS){
    _D(DebugPrintln("unable to create LoRa Task. " + String(rc), DEBUG_LEVEL_1); pause(50);)
    return;
  }
  while ((LoRaTaskHandle != NULL) && ((millis() - t) < maxTestDurationMS)){ vTaskDelay(100); }
  stopLoraTask = true;
  while (LoRaTaskHandle != NULL){ vTaskDelay(100); }
  _D(_PrintShortSummary(DEBUG_LEVEL_3);)
  _D(DebugPrintln("lora task heap water mark: " + String(loraTaskWaterMark),DEBUG_LEVEL_1);)
  pause(100);
}
#ifdef HEIDI_GATEWAY
void loraTestGatewayTask(void *pvParameters) {
  _D(DebugPrintln("LORA: start test gateway task ",   DEBUG_LEVEL_1);)
  #ifdef USE_OLED
  //reset OLED display via software
  pinMode(DISPLAY_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  pause(40);
  digitalWrite(DISPLAY_RST, HIGH);
  //initialize OLED
  display.init();
  //display.displayOn();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.clear();
  display.drawString(0, 0,  "Wait for data" );
  display.display();
  #endif
  _DD(_PrintHeidiConfig(DEBUG_LEVEL_3);)
  if (SetupLoRa()){
    setupLoraRc = LORA_SETUP_DONE_OK;
    while(!stopLoraTask){
      uint8_t clientID = HEIDI_FIRST_CLIENT;
      _DD(DebugPrintln("LORA: request data from client " + String(clientID), DEBUG_LEVEL_3);)
      int dataSets = loraRequestData(clientID, 1, LORA_MIN_TRANS_RSSI, LORA_MIN_TRANS_SNR);
      if(dataSets > 0){
        loraLoadDataSets(clientID, dataSets);
      }
      pause(2000);
      #if 0
      //test fix point
      //for( int i = 0; i < 2; i++){
        #ifdef USE_OLED
        display.clear();
        display.drawString(0, 0,  "Do a general call");
        display.display();
        #endif
        int newClients = 0;
        if (!stopLoraTask) { loraGeneralCall(); }
        #ifdef USE_OLED
        display.drawString(0, 20, String(newClients) + " new clients.");
        display.drawString(0, 40, "Send cconfig.");
        display.display();
        #endif
        if (!stopLoraTask) loraBroadcastSettings();
        pause(100);
        if((heidiConfig->clients != 0) && (!stopLoraTask)){ loraLoadData(); }
        if (!stopLoraTask) { pause(5000); }
      //}
      #endif
    }
    CloseLoRa();
  } else { setupLoraRc = LORA_SETUP_FAILED; }
  #ifdef USE_OLED
  display.clear();
  display.displayOff();
  display.end();
  #endif
  _D(DebugPrintln("LORA: end task ",   DEBUG_LEVEL_1);)
  loraTaskWaterMark = uxTaskGetStackHighWaterMark(NULL);
  LoRaTaskHandle = NULL;
  vTaskDelete(NULL);
}
#else //HEIDI_GATEWAY
void loraTestClientTask(void *pvParameters) {
  uint8_t recBuffer [LORA_CLIENT_RECEIVE_PKG_MAX_LEN];
  int rssi;
  int cnt = 0;
  int fail = 0;
  _D(DebugPrintln("LORA: start client task ",   DEBUG_LEVEL_1);)
  _D(DebugPrintln("LORA: taskhandle: " + String((uint32_t)xTaskGetCurrentTaskHandle(), HEX), DEBUG_LEVEL_1); pause(20);)
  #ifdef USE_OLED
  //reset OLED display via software
  pinMode(DISPLAY_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  pause(40);
  digitalWrite(DISPLAY_RST, HIGH);
  //initialize OLED
  display.init();
  //display.displayOn();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  #endif
  pause(1000);
  _DD(_PrintHeidiConfig(DEBUG_LEVEL_3);)
  if (SetupLoRa()){
    setupLoraRc = LORA_SETUP_DONE_OK;
    _DD(DebugPrintln("LORA: wait for request (client " + String(animalID()) + ")",   DEBUG_LEVEL_3);)
    while(!stopLoraTask){
      int recLen = loraWaitForDataPackage(LORA_PKG_ID_ALL_PKG, recBuffer, sizeof(recBuffer), 500, &rssi);
      if(recLen <= 2) { continue; } //there should be more
      switch(recBuffer[0]) {
        case LORA_PKG_ID_GEN_CALL:{
          _DD(
            uint32_t mask = _copyBufferToUInt32(recBuffer, 1);
            DebugPrintln("LORA: got general call: 0x" + String(mask, HEX) + " [rssi:" + String(rssi) + "]",   DEBUG_LEVEL_3);
          )
          if (recLen >= LORA_PKG_GEN_CALL_LEN) { loraAnswerGenCall(recBuffer); }
          break;
        }
        case LORA_PKG_ID_CONFIG:{
          _DD(DebugPrintln("LORA: got config [rssi:" + String(rssi) + "]", DEBUG_LEVEL_3);)
          if(loraDecodeConfigData(recBuffer)){
            _DD(_PrintHeidiConfig(DEBUG_LEVEL_3););
          }
          break;
        }
        case LORA_PKG_ID_DATA_REQ:{
          _DD(DebugPrintln("LORA: data request [rssi:" + String(rssi) + "]", DEBUG_LEVEL_3);)
          if (recBuffer[2] == recBuffer[3]){
            int maxSetsToSend = recBuffer[2];
            int setsToSend = packUpDataSets();
            if(setsToSend == 0){ stopLoraTask = true; }
            else {
              _DD(DebugPrintln("LORA: " + String(maxSetsToSend) + " data sets allowed to send.", DEBUG_LEVEL_3);)
              if (loraSendDataSets(1) == 1) { cnt++; } else { fail++; }
              #ifdef USE_OLED
              display.clear();
              display.drawString(0, 0, String(cnt) + " sent, " + String(fail) + " bad");
              display.drawString(0, 20, "RSSI: " + String(rssi));
              display.display();
              #endif
            }
          } _D( else {DebugPrintln("LORA: invalid data request message", DEBUG_LEVEL_1);} )
          break;
        }
      }
    } //while stop lora task
  } else { setupLoraRc = LORA_SETUP_FAILED; }
  _D(DebugPrintln("LORA: end task ",   DEBUG_LEVEL_1);)
  #ifdef USE_OLED
  display.clear();
  display.displayOff();
  display.end();
  #endif
  loraTaskWaterMark = uxTaskGetStackHighWaterMark(NULL);
  LoRaTaskHandle = NULL;
  vTaskDelete(NULL);
}
#endif //else HEIDI_GATEWAY

#if 0
void loraTestPushData(void){
  uint8_t buffer[LORA_PKG_DATA_REQ_LEN];
  int cnt = 0;
  int fail = 0;
  int rssi = LORA_NO_RSSI;
  int RSSIsum = 0;
  int t = millis();

  #ifdef USE_OLED
  display.clear();
  display.drawString(0, 0, "wait for request");
  display.display();
  #endif
  if(loraWaitForDataPackage(LORA_PKG_ID_DATA_REQ, buffer, sizeof(buffer), 50, &rssi) == LORA_PKG_DATA_REQ_LEN){
    _DD(DebugPrintln("LORA: got request",   DEBUG_LEVEL_3);)
    #ifdef USE_OLED
    display.drawString(0, 20, "got, request");
    display.display();
    #endif
    if (buffer[2] == buffer[3]){
      uint8_t setsToSend = buffer[2];
      if (TEST_DATA_COUNT < setsToSend) { setsToSend = TEST_DATA_COUNT; }
      if(loraDataReady(setsToSend)) {
        for(int m = TEST_DATA_COUNT-1; m >= (TEST_DATA_COUNT - setsToSend); m--){
          bool ack = false;
          uint8_t* data = (uint8_t*)(getTestData(m));
          for(int i = 0; i < msglen; i++) { message[i] = data[i]; } // Fill with the message
          rs.Encode(message, &encoded[2]);
          encoded[0] = LORA_PKG_ID_ONE_SET;
          encoded[1] = m;
          int repeats = 0;
          while((!ack) && (repeats < LORA_MAX_TRIES)){
            fail++;
            // send packet
            int dt = millis();
            for(int i=0; i<2; i++){
              if (LoRa.beginPacket()) {
                LoRa.write(encoded, sizeof(encoded));
                LoRa.endPacket();
                break;
              } else { pause(10); }
            }
            _D(DebugPrintln(" package [" + String(encoded[1]) + "] sent [" + String(millis()-dt) + " ms]", DEBUG_LEVEL_1);)
            dt = millis();
            ack = loraWaitForACK(&rssi);
            if (ack){
              fail--; cnt++; RSSIsum += rssi;
              _DD(DebugPrintln(String(m) + ": got ACK [rssi :" + String(rssi) + ", " + String(millis()-dt) + " ms]", DEBUG_LEVEL_3);)
            } else {
              _DD(DebugPrintln(String(m) + ": got no ACK [" + String(millis()-t) + " ms]", DEBUG_LEVEL_3);)
            }
            repeats++;
          }
          if(repeats >= LORA_MAX_TRIES) { _D(DebugPrintln("package sent failed [" + String(millis()-t) + " ms]", DEBUG_LEVEL_1);) }
          //delay(50);
        }
        _D(DebugPrintln(String(cnt) + " packages sent successfully [" + String(millis()-t) + " ms]", DEBUG_LEVEL_1);)
        if (cnt > 0) { RSSIsum /= cnt; }
        #ifdef USE_OLED
        display.clear();
        display.drawString(0, 0, String(cnt) + " good, " + String(fail) + " bad");
        display.drawString(0, 20, "rssi: "+ String(RSSIsum));
        display.drawString(0, 40, String((millis()-t) / 1000) + " sec");
        display.display();
        #endif
      } _DD( else {DebugPrintln("LORA send data ready message failed", DEBUG_LEVEL_3);} )
    } _DD( else {DebugPrintln("LORA invalis data request message", DEBUG_LEVEL_3);} )
  }
}

void TestLoRa(void){
  uint8_t* data = (uint8_t*)(&loRaTestData[0]);
  t_SendData dummy;
  uint8_t* buffer = (uint8_t*)(&dummy);
  int t= millis();
  #ifdef USE_OLED
  //reset OLED display via software
  pinMode(DISPLAY_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  pause(40);
  digitalWrite(DISPLAY_RST, HIGH);
  //initialize OLED
  display.init();
  //display.displayOn();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  #endif


  if (SetupLoRa()){
    #ifndef LORA_SENDER
    int packetSize = 0;
    int cnt = 0;
    int current = 0;
    int last    = 0;
    int twice   = 0;
    int fakeNACK = 4;
    int overAll = 0;
    int lost = 0;
    bool package = false;
    while(1){
      int x = 0;
      bool ack = false;
      while(packetSize == 0){
        packetSize = LoRa.parsePacket();
        if(packetSize == 0) { pause(5); } else { break; }
        x++;
        if (x > 1000) {
          if (package){
            DebugPrintln("", DEBUG_LEVEL_1);
            DebugPrintln(String(cnt) + " done in " + String(overAll) + "ms, " + String(lost) + " lost."  , DEBUG_LEVEL_1);
            cnt = 0;
            current = 0;
            last    = 0;
            twice = 0;
            lost = 0;
            package = false;
          }
          DebugPrint(".", DEBUG_LEVEL_1);
          x=0;
        }
      }
      package = true;
      if(cnt == 0){ DebugPrintln("", DEBUG_LEVEL_1); DebugPrintln("", DEBUG_LEVEL_1); DebugPrintln("", DEBUG_LEVEL_1); t= millis(); }
      //DebugPrintln("package received len: " + String(packetSize)+ " [" + String(millis()-t) + " ms]", DEBUG_LEVEL_1);
      if (LoRa.readBytes(encoded, packetSize) != packetSize){
        DebugPrintln("could not read " + String(packetSize) + " bytes from package", DEBUG_LEVEL_1);
      } else {
        int dt = millis();
        int rc = rs.Decode(encoded, repaired);
        dt = millis() - dt;
        fakeNACK--;
        if ((rc > 0) || (fakeNACK == 0)){
          DebugPrintln("Decoding failed: " + String(rc), DEBUG_LEVEL_1);
        } else {
          cnt++;
          ack = true;
          String corrected = "";
          for(int i = 0; i < msglen; i++) { if (encoded[i] != repaired[i]) { corrected = " - corrected!"; break; } }
          for(int i = 0; i < (msglen -1); i++) { buffer[i] = repaired[i+1]; }
          current = repaired[0]+1;
          if (current == last) { twice++; } else { lost += current-last-1; }
          last = current;
          //if(corrected != ""){
            DebugPrintln("No. " + String(cnt) + ", twice: " + String(twice) + ", lost: " + String(lost) + " -  pkg-No: " + String(current) + " Dec success [" + String(dt) + "] " + corrected, DEBUG_LEVEL_1);
          //}
          //_PrintDataSet(&dummy, DEBUG_LEVEL_1);
        }
      }
      sendACK(ack);
      packetSize = 0;
      overAll = millis()-t;
    }
    #else //LORA_SENDER
    int cnt = 0;
    int fail = 0;
    int RSSIsum = 0;
    for(int m = 0; m < (TEST_DATA_COUNT); m++){
      bool ack = false;
      uint8_t* data = (uint8_t*)(&loRaTestData[m]);
      message[0] = m; //LORA_PKG_ID_ONE_SET;
      for(int i = 0; i < msglen; i++) { message[i+1] = data[i]; } // Fill with the message
      rs.Encode(message, encoded);
      int repeats = 0;
      int rssi = LORA_NO_RSSI;
      while((!ack) && (repeats < LORA_MAX_TRIES)){
        fail++;
        // send packet
        int dt = millis();
        for(int i=0; i<2; i++){
          if (LoRa.beginPacket()) {
            LoRa.write(encoded, sizeof(encoded));
            LoRa.endPacket();
            break;
          } else { pause(10); }
        }
        DebugPrintln(" package sent [" + String(millis()-dt) + " ms]", DEBUG_LEVEL_1);
        dt = millis();
        ack = waitForACK(&rssi);
        if (ack){
          fail--; cnt++; RSSIsum += rssi;
          _DD(DebugPrintln(String(m) + ": got ACK [rssi :" + String(rssi) + ", " + String(millis()-dt) + " ms]", DEBUG_LEVEL_3);)
        } else {
          _DD(DebugPrintln(String(m) + ": got no ACK [" + String(millis()-t) + " ms]", DEBUG_LEVEL_3);)
        }
        repeats++;
      }
      if(repeats >= LORA_MAX_TRIES) { DebugPrintln("package sent failed [" + String(millis()-t) + " ms]", DEBUG_LEVEL_1); }
      #ifdef USE_OLED
      display.clear();
      display.drawString(0, 0, String(m+1) + " sent");
      display.display();
      #endif
      //delay(50);
    }
    DebugPrintln(String(cnt) + " packages sent successfully [" + String(millis()-t) + " ms]", DEBUG_LEVEL_1);
    if (cnt > 0) { RSSIsum /= cnt; }
    #ifdef USE_OLED
    display.clear();
    display.drawString(0, 0, String(cnt) + " good, " + String(fail) + " bad");
    display.drawString(0, 20, "rssi: "+ String(RSSIsum));
    display.drawString(0, 40, String((millis()-t) / 1000) + " sec");
    display.display();

    while(1);

    pause(4000);
    display.clear();
    display.displayOff();
    display.end();
    #endif
    #endif //LORA_SENDER
  }
  /*
  DebugPrintln("Original:  ", DEBUG_LEVEL_1);
  _PrintDataSet(&loRaTestData[0], DEBUG_LEVEL_1);

  for(int i = 0; i < msglen; i++) { message[i] = data[i]; } // Fill with the message
  rs.Encode(message, encoded);

  DebugPrintln("Encoded:  ", DEBUG_LEVEL_1);
  for(int i = 0; i < msglen; i++) { buffer[i] = encoded[i]; }
  _PrintDataSet(&dummy, DEBUG_LEVEL_1);


  dummy.date = (dummy.date & 0xff00);// | 0x23;
  dummy.latitude = (dummy.latitude & 0xff00ffff);// | 0x5b0000;
  DebugPrintln("Corrupted: ", DEBUG_LEVEL_1);
  _PrintDataSet(&dummy, DEBUG_LEVEL_1);
  for(int i = 0; i < msglen; i++) { encoded[i] = buffer[i]; } // Fill with the corrupted message

  int rc = rs.Decode(encoded, repaired);
  if (rc > 0){
    DebugPrintln("Decoding failed: " + String(rc), DEBUG_LEVEL_1);
  } else {
    DebugPrintln("Decoding success", DEBUG_LEVEL_1);
    for(int i = 0; i < msglen; i++) { buffer[i] = repaired[i]; }
    _PrintDataSet(&dummy, DEBUG_LEVEL_1);
  };
 */
  CloseLoRa();
}
/*
String LoRaTestStrings[] = {
  "1001010d560a030f97cb0026013e54605f190fd0080000070020000000000000000000000088fb",
  "100101c1550a031497cb0019013e549d5e190fc3080000070028000000000000000000000088fb",
  "10010174550a038a97cb0022013e54e05d1a0fbd08000006001e000000000000000000000088fb",
  "10010105560a03b897cb0026013e541d5d1a0fb108000007002e000000000000000000000088fb",
  "100101d4550a033c97cb0035013e545d5c1d0fa408000008002100ae002e000000009f00f588fb"
};

void transformTestDate(void){

 for(int i=0; i<TEST_STRINGS; i++){
   //DebugPrint("0x" + LoRaTestStrings[i].substring(0,2) + ", ", DEBUG_LEVEL_1);
   //DebugPrint("0x" + LoRaTestStrings[i].substring(2,4) + ", ", DEBUG_LEVEL_1);
   //DebugPrint("0x" + LoRaTestStrings[i].substring(4,6) + ", ", DEBUG_LEVEL_1);

   DebugPrint("{ 0x" + LoRaTestStrings[i].substring(12,14), DEBUG_LEVEL_1);
   DebugPrint(LoRaTestStrings[i].substring(10,12), DEBUG_LEVEL_1);
   DebugPrint(LoRaTestStrings[i].substring(8,10), DEBUG_LEVEL_1);
   DebugPrint(LoRaTestStrings[i].substring(6,8) + ", ", DEBUG_LEVEL_1);

   DebugPrint("0x" + LoRaTestStrings[i].substring(20,22), DEBUG_LEVEL_1);
   DebugPrint(LoRaTestStrings[i].substring(18,20), DEBUG_LEVEL_1);
   DebugPrint(LoRaTestStrings[i].substring(16,18), DEBUG_LEVEL_1);
   DebugPrint(LoRaTestStrings[i].substring(14,16) + ", ", DEBUG_LEVEL_1);
   for(int j=22; j<46; j+=4){
     DebugPrint("0x" + LoRaTestStrings[i].substring(j+2,j+4), DEBUG_LEVEL_1);
     DebugPrint(LoRaTestStrings[i].substring(j,j+2) + ", ", DEBUG_LEVEL_1);
   }
   DebugPrint("0x" + LoRaTestStrings[i].substring(46,48) + ", ", DEBUG_LEVEL_1);
   DebugPrint("0x" + LoRaTestStrings[i].substring(50,52) + ", ", DEBUG_LEVEL_1);
   for(int j=54; j<70; j+=4){
     DebugPrint("0x" + LoRaTestStrings[i].substring(j+2,j+4), DEBUG_LEVEL_1);
     DebugPrint(LoRaTestStrings[i].substring(j,j+2) + ", ", DEBUG_LEVEL_1);
   }
   DebugPrint("0x" + LoRaTestStrings[i].substring(72,74), DEBUG_LEVEL_1);
   DebugPrint(LoRaTestStrings[i].substring(70,72) + " },", DEBUG_LEVEL_1);

   DebugPrintln("", DEBUG_LEVEL_1);
 }
}
*/

#endif //0

#endif //TEST_LORA

#endif //USE_LORA


