/*
 * heidi-lora.cpp
 *
 *  Created on: 31.01.2022
 *      Author: frank
 */
#include <Arduino.h>
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

#ifdef HEIDI_GATEWAY
void loraGatewayTask(void *pvParameters) {
  if (SetupLoRa()){
    if(loraDoGeneralCall()){ loraGeneralCall(); }

    CloseLoRa();
  }
  _D(DebugPrintln("LORA: end task ",   DEBUG_LEVEL_1);)
  loraTaskWaterMark = uxTaskGetStackHighWaterMark(NULL);
  LoRaTaskHandle = NULL;
  vTaskDelete(NULL);
}
#else
/*
 * LoRa Client task just listens and acts if there was something
 */
void loraClientTask(void *pvParameters) {
  if (SetupLoRa()){
    while(!stopLoraTask){

    }
    CloseLoRa();
  }
  _D(DebugPrintln("LORA: end task ", DEBUG_LEVEL_1);)
  loraTaskWaterMark = uxTaskGetStackHighWaterMark(NULL);
  task_lora = NULL;
  vTaskDelete(NULL);
}
#endif

/*
 * starts the LoRa handling task
 *  - no return code, because there is nothing to do for main task, if this fails
 */
void setupLoraTask(void){
  loraTaskWaterMark = 0;
  #ifdef HEIDI_GATEWAY
  BaseType_t rc = xTaskCreatePinnedToCore(loraGatewayTask, "HeidiLoraTask", LORA_TASK_HEAP_SIZE, NULL, 1, &LoRaTaskHandle, 0);
  #else
  BaseType_t rc = xTaskCreatePinnedToCore(loraClientTask, "HeidiLoraTask", LORA_TASK_HEAP_SIZE, NULL, 1, &LoRaTaskHandle, 0);
  #endif
  if (rc != pdPASS){
    _D(DebugPrintln("LORA: unable to create LoRa Task. " + String(rc), DEBUG_LEVEL_1);)
    LoRaTaskHandle = NULL;
  } _D( else { DebugPrintln("LORA: Task up and running. ", DEBUG_LEVEL_2); })
}

/*
 * stops the LoRa handling task
 */
void closeLoraTask(void){
  if (LoRaTaskHandle != NULL){
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
  //LoRa.setSyncWord(herdeID());
  //LoRa.receive();
  _D(DebugPrintln(".. done", DEBUG_LEVEL_1);)
  return true;
}

void CloseLoRa(void){
  LoRa.end();
  LoRa.sleep();
  SPI.end();
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
  if (rssi != NULL) { *rssi = 0; }
  #if LORA_SPERADING_F != LORA_SPERADING_F
  LoRa.setSpreadingFactor(LORA_ACK_SPREAD_F);
  #endif
  #if LORA_ACK_PREA_LEN != LORA_ACK_PREA_LEN
  LoRa.setPreambleLength(LORA_ACK_PREA_LEN);
  #endif
  while (millis() - t <= LORA_ACK_TIMEOUT_MS){
    int packetSize = 0;
    packetSize = loraWaitForDataPackage(LORA_PKG_ID_ACK, buffer, sizeof(buffer), LORA_ACK_TIMEOUT_MS, rssi);
    if (packetSize > 0) {
      if (rssi != NULL) { *rssi = LoRa.packetRssi(); }
      if (buffer[1] == animalID()){
        if(buffer[2] == LORA_ACK){
          rc = true;
          break;
        }
      }
    }
  }
  _DD(if (!rc) { DebugPrintln("got no ACK [" + String(millis()-t) + " ms]", DEBUG_LEVEL_3); })
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
int loraWaitForDataPackage(loraPackageID_t pkgID, uint8_t *buffer, int bufferSize, int tmOutMs, int *rssi  = NULL){
  int t = millis();
  int rc = -1;
  if (rssi != NULL) { *rssi = 0; }
  if (buffer == NULL) { return rc; }
  while (millis() - t <= tmOutMs){
    int packetSize = 0;
    while(packetSize == 0){
      packetSize = LoRa.parsePacket();
      if(packetSize == 0) { pause(1); }
      if (millis() - t > LORA_ACK_TIMEOUT_MS) {
        //_DD(DebugPrintln("LORA package [" + String(pkgID, HEX) + "] timed out after " + String(millis()-t) + " ms", DEBUG_LEVEL_3);)
        break;
      }
    }
    if (packetSize > 0) {
      if (rssi != NULL) { *rssi = LoRa.packetRssi(); }
      if (LoRa.available() > 0){
        if(LoRa.read() == pkgID){
          int i = 0;
          buffer[i++] = pkgID;
          while((i<bufferSize) && (LoRa.available() > 0)){
            buffer[i++] = LoRa.read();
          }
          rc = i;
          //_DD(DebugPrintln("LORA got package [" + String(pkgID, HEX) + "] length " + String(i) + "; " + hexString2(buffer[0]) + ", " + hexString2(buffer[1]), DEBUG_LEVEL_1); )
          if (LoRa.available()>0){
            _D(DebugPrintln("LORA package [" + String(pkgID, HEX) + "] length " +String(LoRa.available()) + " exceeds buffer size " + String(bufferSize), DEBUG_LEVEL_1); )
            while (LoRa.available()>0) { LoRa.read(); }
          }
          return i;
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
  for(int i=0; i<2; i++){
    if (LoRa.beginPacket()) {
      LoRa.write(buffer, sizeof(buffer));
      LoRa.endPacket();
      return true;
    } else { pause(100); }
  }
  _D(DebugPrintln("LORA device busy", DEBUG_LEVEL_1);)
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
int loraRequestData(uint8_t recID, uint8_t maxSets, int minRSSI){
  uint8_t buffer[LORA_PKG_DATA_RDY_LEN];
  bool rc = false;
  int rssi = minRSSI-1;
  buffer[0] = LORA_PKG_ID_DATA_REQ;
  buffer[1] = recID;
  buffer[2] = maxSets;
  buffer[3] = maxSets;
  if(loraSendPackage(buffer, LORA_PKG_DATA_REQ_LEN)){
    if (loraWaitForDataPackage(LORA_PKG_ID_DATA_RDY, buffer, LORA_PKG_DATA_RDY_LEN, 30, &rssi) != LORA_RC_TIME_OUT){
      if(buffer[1] == recID){ //check client ID
        if((rssi >= minRSSI) || (minRSSI ==0)){
          if(buffer[2] == buffer[3]){
            if(loraSendACK(recID, true)) { return (int)buffer[2]; }
          }
        } _DD( else {DebugPrintln("LORA got too less RSSI to request", DEBUG_LEVEL_3);} )
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
void loraGeneralCall(void){
  uint8_t buffer[LORA_PKG_GEN_CALL_LEN];
  int bytesAvailable;
  buffer[0] = LORA_PKG_ID_GEN_CALL;
  _D( DebugPrintln("LORA: general call...", DEBUG_LEVEL_1);)
  _copyUint32toBuffer(buffer, 1, heidiConfig->clients);
  _copyUint32toBuffer(buffer, 5, heidiConfig->clients);
  if(loraSendPackage(buffer, LORA_PKG_GEN_CALL_LEN)){
    _D(bool newClientsFound = false;)
    while(loraWaitForDataPackage(LORA_PKG_ID_GEN_ANSW, buffer, LORA_PKG_GEN_ANSW_LEN, 200) != LORA_RC_TIME_OUT){
      if(buffer[1] == buffer[2]){
        uint32_t newClient = 1 << (buffer[1] - 1); //bit 0 represents client 1 - client address 0x00 is not allowed
        heidiConfig->clients |= newClient;
        uint16_t cCRC = _copyBufferToUInt16(buffer, 3);
        _D( DebugPrintln("LORA: new client: 0x" + LenTwo(String(buffer[1], HEX)), DEBUG_LEVEL_2); newClientsFound = true;)
        if (cCRC != clientConfigCRC(&heidiConfig->c)){
          heidiConfig->clientsNeedConf |= newClient;
          _DD( DebugPrintln("LORA: new client needs new config.", DEBUG_LEVEL_3);)
        }
      }
    }
    _D(if(!newClientsFound) {DebugPrintln("LORA: no new clients.", DEBUG_LEVEL_2);})
  }
}

#else //HEIDI_GATEWAY
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
#endif
#ifdef HEIDI_CONFIG_TEST
#define TEST_DATA_COUNT 96
t_SendData loRaTestData[TEST_DATA_COUNT] = {
 /* 1 */ { 0x030a560d, 0x00cb970f, 0x0126, 0x543e, 0x5f60, 0x0f19, 0x08d0, 0x0000, 0x07, 0x20, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55c1, 0x00cb9714, 0x0119, 0x543e, 0x5e9d, 0x0f19, 0x08c3, 0x0000, 0x07, 0x28, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5574, 0x00cb978a, 0x0122, 0x543e, 0x5de0, 0x0f1a, 0x08bd, 0x0000, 0x06, 0x1e, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5605, 0x00cb97b8, 0x0126, 0x543e, 0x5d1d, 0x0f1a, 0x08b1, 0x0000, 0x07, 0x2e, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55d4, 0x00cb973c, 0x0135, 0x543e, 0x5c5d, 0x0f1d, 0x08a4, 0x0000, 0x08, 0x21, 0x00ae, 0x002e, 0x0000, 0x9f00, 0xf500 },
         { 0x030a5521, 0x00cb96c6, 0x0139, 0x543e, 0x5b9d, 0x0f1d, 0x0891, 0x0000, 0x09, 0x20, 0x004f, 0x0021, 0x0000, 0xf070, 0xf000 },
         { 0x030a5655, 0x00cb9828, 0x0135, 0x543e, 0x5add, 0x0f23, 0x087f, 0x0000, 0x06, 0x2e, 0x0012, 0x0000, 0x0000, 0x00f0, 0x0000 },
         { 0x030a51f4, 0x00cb924a, 0x0079, 0x543e, 0x5a20, 0x0f22, 0x087f, 0x0000, 0x06, 0x28, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5620, 0x00cb963f, 0x00fa, 0x543e, 0x595d, 0x0f23, 0x087f, 0x0800, 0x09, 0x47, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 10 */ { 0x030a56a9, 0x00cb96ee, 0x011c, 0x543e, 0x589d, 0x0f23, 0x0878, 0x0000, 0x09, 0x3a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5490, 0x00cb96f3, 0x0179, 0x543e, 0x575d, 0x0f23, 0x0872, 0x0800, 0x06, 0x4b, 0x0000, 0x0000, 0x0008, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x569d, 0x0f25, 0x086c, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5cd1, 0x00cb99c6, 0x0238, 0x543e, 0x55e0, 0x0f26, 0x086c, 0x0800, 0x06, 0x55, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x5520, 0x0f26, 0x0866, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a54eb, 0x00cb9647, 0x0000, 0x543e, 0x545c, 0x0f2a, 0x0866, 0x0800, 0x03, 0x5d, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a54e0, 0x00cb96bd, 0x0000, 0x543e, 0x53a1, 0x0f28, 0x085f, 0x0000, 0x03, 0x20, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55a2, 0x00cb9743, 0x0142, 0x543e, 0x52dc, 0x0f2a, 0x085f, 0x0800, 0x07, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a557c, 0x00cb978d, 0x0000, 0x543e, 0x5220, 0x0f2b, 0x085f, 0x0000, 0x03, 0x27, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5755, 0x00cb9587, 0x0133, 0x543e, 0x515d, 0x0f2b, 0x085f, 0x0800, 0x08, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 20 */ { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x509d, 0x0f2b, 0x085f, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x4f60, 0x0f2d, 0x0859, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x4e9d, 0x0f2d, 0x084d, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a57b4, 0x00cb948b, 0x0166, 0x543e, 0x4ddd, 0x0f2d, 0x0846, 0x0800, 0x06, 0x43, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x4da1, 0x0f2d, 0x0840, 0x0000, 0x00, 0x03, 0x000e, 0x0000, 0x0000, 0x000f, 0x0000 },
         { 0x030a5742, 0x00cb94e8, 0x0120, 0x543e, 0x4ce1, 0x0f2e, 0x0840, 0x0800, 0x07, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x4c40, 0x0f30, 0x0834, 0x6858, 0x00, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a59b7, 0x00cba0c3, 0x0000, 0x543e, 0x4ba0, 0x0f31, 0x0821, 0x0840, 0x03, 0x79, 0x0000, 0x0000, 0x0074, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x4ae0, 0x0f30, 0x081b, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5661, 0x00cb957a, 0x014f, 0x543e, 0x4a1d, 0x0f33, 0x080e, 0x0800, 0x07, 0x62, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 30 */ { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x4960, 0x0f33, 0x0802, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a56b9, 0x00cb96b1, 0x00dc, 0x543e, 0x489d, 0x0f36, 0x07f5, 0x0800, 0x08, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55b5, 0x00cb9722, 0x0000, 0x543e, 0x4760, 0x0f34, 0x07dc, 0x0800, 0x03, 0x41, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a56db, 0x00cb95d3, 0x00dd, 0x543e, 0x46a0, 0x0f39, 0x07d6, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5624, 0x00cb94fc, 0x0000, 0x543e, 0x45dc, 0x0f39, 0x07d0, 0x0800, 0x03, 0x57, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5709, 0x00cb97d6, 0x007c, 0x543e, 0x4521, 0x0f37, 0x07c3, 0x0000, 0x05, 0x19, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a56c4, 0x00cb95c0, 0x00ac, 0x543e, 0x445c, 0x0f3c, 0x07d0, 0x0800, 0x06, 0xff, 0x0034, 0x000f, 0x0000, 0x00f0, 0x00f0 },
         { 0x030a5669, 0x00cb984b, 0x0000, 0x543e, 0x43a0, 0x0f3c, 0x07c9, 0x0000, 0x03, 0x35, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5742, 0x00cb94f2, 0x0012, 0x543e, 0x42dd, 0x0f3f, 0x07c9, 0x0800, 0x05, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5552, 0x00cb972f, 0x0000, 0x543e, 0x4220, 0x0f3f, 0x07b7, 0x0800, 0x03, 0x76, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 40 */ { 0x030a555a, 0x00cb96d8, 0x0125, 0x543e, 0x415d, 0x0f41, 0x07b7, 0x0800, 0x05, 0x79, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a558b, 0x00cb9763, 0x010e, 0x543e, 0x40a0, 0x0f42, 0x07c3, 0x0000, 0x05, 0x2b, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55c8, 0x00cb97b8, 0x0131, 0x543e, 0x3f5d, 0x0f44, 0x07c3, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5578, 0x00cb9777, 0x0000, 0x543e, 0x3e9d, 0x0f45, 0x07d0, 0x0800, 0x03, 0x43, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5412, 0x00cb9666, 0x0000, 0x543e, 0x3ddd, 0x0f44, 0x07d0, 0x0800, 0x03, 0x4a, 0x0000, 0x0000, 0x0007, 0x0000, 0x0000 },
         { 0x030a55ef, 0x00cb97c3, 0x0127, 0x543e, 0x3d20, 0x0f45, 0x07d0, 0x0800, 0x06, 0x61, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55db, 0x00cb9724, 0x0125, 0x543e, 0x3c60, 0x0f48, 0x07d6, 0x0800, 0x05, 0x58, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a556d, 0x00cb9734, 0x013b, 0x543e, 0x3b9d, 0x0f45, 0x07d6, 0x0800, 0x06, 0x3c, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55e7, 0x00cb9828, 0x0113, 0x543e, 0x3ae0, 0x0f48, 0x07d6, 0x0000, 0x07, 0x3a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a559b, 0x00cb9710, 0x013c, 0x543e, 0x3a1d, 0x0f4a, 0x07dc, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 50 */ { 0x030a56d7, 0x00cb96d6, 0x0000, 0x543e, 0x395d, 0x0f4a, 0x07e2, 0x0800, 0x03, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5552, 0x00cb9737, 0x012c, 0x543e, 0x38a0, 0x0f4a, 0x07dc, 0x0800, 0x05, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5569, 0x00cb96ec, 0x0000, 0x543e, 0x3760, 0x0f4e, 0x07e9, 0x0800, 0x03, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a58b0, 0x00cb95e3, 0x0000, 0x543e, 0x369d, 0x0f52, 0x07e9, 0x0800, 0x03, 0x62, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a56a2, 0x00cb963d, 0x0000, 0x543e, 0x35e0, 0x0f52, 0x07ef, 0x0800, 0x03, 0x5a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a559e, 0x00cb96e6, 0x012d, 0x543e, 0x3520, 0x0f56, 0x07ef, 0x0800, 0x05, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a56db, 0x00cb98ef, 0x0000, 0x543e, 0x345d, 0x0f58, 0x07e9, 0x0800, 0x03, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5637, 0x00cb9869, 0x0110, 0x543e, 0x33a0, 0x0f58, 0x07e9, 0x0000, 0x04, 0x24, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5552, 0x00cb96e3, 0x013d, 0x543e, 0x32dc, 0x0f58, 0x07f5, 0x0000, 0x06, 0x3c, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5578, 0x00cb978f, 0x0000, 0x543e, 0x321d, 0x0f56, 0x0802, 0x0000, 0x03, 0x1e, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 60 */ { 0x030a5569, 0x00cb9714, 0x0141, 0x543e, 0x3160, 0x0f5b, 0x0802, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5665, 0x00cb993f, 0x0000, 0x543e, 0x309d, 0x0f5c, 0x0808, 0x0800, 0x03, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55cc, 0x00cb97fa, 0x011e, 0x543e, 0x2f61, 0x0f5c, 0x080e, 0x0800, 0x04, 0x55, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a558f, 0x00cb97d8, 0x011a, 0x543e, 0x2e9c, 0x0f5e, 0x0808, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a54fa, 0x00cb960b, 0x0000, 0x543e, 0x2de1, 0x0f5e, 0x0808, 0x0000, 0x03, 0x33, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5543, 0x00cb96e3, 0x0137, 0x543e, 0x2d1c, 0x0f5f, 0x0808, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a54eb, 0x00cb95a3, 0x0000, 0x543e, 0x2c5d, 0x0f5f, 0x0814, 0x0000, 0x03, 0x2c, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5593, 0x00cb9766, 0x012f, 0x543e, 0x2ba0, 0x0f62, 0x080e, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a554e, 0x00cb961c, 0x0000, 0x543e, 0x2add, 0x0f67, 0x081b, 0x0800, 0x03, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x2a20, 0x0f67, 0x0814, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 70 */ { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x295d, 0x0f67, 0x081b, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a52d5, 0x00cb97fc, 0x01af, 0x543e, 0x28a0, 0x0f69, 0x0827, 0x0800, 0x05, 0x75, 0x0000, 0x0000, 0x0036, 0x0000, 0x0000 },
         { 0x030a55b9, 0x00cb9709, 0x0127, 0x543e, 0x275d, 0x0f6c, 0x081b, 0x0800, 0x05, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a05cb, 0x00caf5c0, 0x0000, 0x543e, 0x26a0, 0x0f6a, 0x0827, 0x0800, 0x03, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55b5, 0x00cb96ab, 0x0132, 0x543e, 0x25dd, 0x0f70, 0x0827, 0x0800, 0x06, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55c8, 0x00cb9734, 0x0126, 0x543e, 0x2520, 0x0f72, 0x0827, 0x0800, 0x04, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a573e, 0x00cb9806, 0x0000, 0x543e, 0x245d, 0x0f78, 0x082d, 0x0800, 0x03, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x239d, 0x0f76, 0x083a, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55e7, 0x00cb976a, 0x012f, 0x543e, 0x22e0, 0x0f7b, 0x083a, 0x0800, 0x04, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x221d, 0x0f81, 0x083a, 0x0818, 0x00, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 80 */ { 0x030a5701, 0x00cb9c2b, 0x0000, 0x543e, 0x2160, 0x0f80, 0x083a, 0x0800, 0x03, 0x79, 0x0000, 0x0000, 0x0021, 0x0000, 0x0000 },
         { 0x030a58e2, 0x00cb9863, 0x00f6, 0x543e, 0x20a0, 0x0f83, 0x083a, 0x0800, 0x05, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x2080, 0x0f83, 0x083a, 0x0000, 0x00, 0x02, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5602, 0x00cb9704, 0x0122, 0x543e, 0x1f40, 0x0f87, 0x0840, 0x0800, 0x05, 0xff, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030b4b16, 0x00c8a656, 0x0000, 0x543e, 0x1e9d, 0x0f87, 0x083a, 0x0840, 0x03, 0x79, 0x0000, 0x0000, 0x3718, 0x0000, 0x0000 },
         { 0x030a559b, 0x00cb984f, 0x0000, 0x543e, 0x1ddd, 0x0f89, 0x0840, 0x0800, 0x03, 0x64, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x1d20, 0x0f8a, 0x0846, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x1c5d, 0x0f8a, 0x0846, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x00000000, 0x00000000, 0x0000, 0x543e, 0x1ba0, 0x0f8e, 0x084d, 0x0818, 0x00, 0x78, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a56c8, 0x00cb9f08, 0x005d, 0x543e, 0x1add, 0x0f8f, 0x084d, 0x0800, 0x04, 0x79, 0x0000, 0x0000, 0x0054, 0x0000, 0x0000 },
/* 90 */ { 0x030a54ba, 0x00cb95a9, 0x0000, 0x543e, 0x1a1d, 0x0f8f, 0x084d, 0x0000, 0x03, 0x1b, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a555a, 0x00cb979b, 0x0123, 0x543e, 0x1960, 0x0f8e, 0x084d, 0x0000, 0x06, 0x22, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a5524, 0x00cb9718, 0x0129, 0x543e, 0x189d, 0x0f8e, 0x0859, 0x0800, 0x06, 0x3e, 0x0000, 0x0000, 0x0001, 0x0000, 0x0000 },
         { 0x030a554a, 0x00cb973f, 0x0000, 0x543e, 0x175d, 0x0f8f, 0x0859, 0x0000, 0x03, 0x21, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a55aa, 0x00cb97ac, 0x012b, 0x543e, 0x16a0, 0x0f8e, 0x0866, 0x0800, 0x07, 0x40, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
         { 0x030a555a, 0x00cb977a, 0x013c, 0x543e, 0x15dd, 0x0f8f, 0x0866, 0x0800, 0x06, 0x59, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
/* 96 */ { 0x030a55fa, 0x00cb9746, 0x0138, 0x543e, 0x151d, 0x0f8e, 0x0866, 0x0800, 0x06, 0x40, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 }
  };

uint8_t  message[sizeof(t_SendData)];
const int msglen = sizeof(message);
const uint8_t ECC_LENGTH_ONE_SET = 3;  //Max message lenght, and "guardian bytes", Max corrected bytes ECC_LENGTH/2
//uint8_t  message_frame[msglen];      //The message size would be different, so need a container
uint8_t  repaired[msglen];
uint8_t  encoded[msglen + ECC_LENGTH_ONE_SET + 2];
RS::ReedSolomon<msglen, ECC_LENGTH_ONE_SET> rs;

TaskHandle_t task_lora_test;

bool TestLoRa(void){
  stopLoraTask = false;
  setupLoraRc = LORA_SETUP_NOT_DONE;

  #ifdef HEIDI_GATEWAY
  BaseType_t rc = xTaskCreatePinnedToCore(loraTestGatewayTask, "loraGatewayTask", LORA_TASK_HEAP_SIZE, NULL, 1, &task_lora_test, 0);
  #else
  BaseType_t rc = xTaskCreatePinnedToCore(loraTestClientTask, "loraGatewayTask", LORA_TASK_HEAP_SIZE, NULL, 1, &task_lora_test, 0);
  #endif
  if (rc != pdPASS){
    _D(DebugPrintln("unable to create LoRa Task. " + String(rc), DEBUG_LEVEL_1); pause(50);)
    return false;
  }
  for(int i=0; i<60; i++) { pause(60000); }
  stopLoraTask = true;
  while (task_lora_test != NULL){ vTaskDelay(100); }
  _D(DebugPrintln("lora task heap water mark: " + String(loraTaskWaterMark),DEBUG_LEVEL_1);)
  pause(4000);
}

#ifdef HEIDI_GATEWAY
void loraTestGatewayTask(void *pvParameters) {
  uint8_t* data = (uint8_t*)(&loRaTestData[0]);
  t_SendData dummy;
  uint8_t* dataSetBuffer = (uint8_t*)(&dummy);

  _D(DebugPrintln("LORA: start gateway task ",   DEBUG_LEVEL_1);)
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
  if (SetupLoRa()){
    setupLoraRc = LORA_SETUP_DONE_OK;
    uint8_t buffer[LORA_PKG_ONE_SET_LEN];
    uint8_t clientID = 0x02;
    while(!stopLoraTask){
      int packetSize = 0;
      int cnt = 0;
      int current = 0;
      int last    = 0;
      int twice   = 0;
      int overAll = 0;
      int lost    = 0;
      int RSSIsum = 0;
      bool package = false;
      int t= millis();
       _D(DebugPrintln("LORA: SEND REQUEST ",   DEBUG_LEVEL_1);)
      int dataSets = loraRequestData(clientID, LORA_MIN_TRANS_RSSI);
      if (dataSets > 0){
        _D(DebugPrintln("LORA: load "+ String(dataSets) + " sets ",   DEBUG_LEVEL_1);)
        #ifdef USE_OLED
        display.drawString(0, 20, "receive data");
        display.display();
        #endif
        int x = 0;
        int rssi = 0;
        bool ack = false;
        while(current < dataSets){
          if ((packetSize = loraWaitForDataPackage(LORA_PKG_ID_ONE_SET, buffer, sizeof(buffer), 50, &rssi)) != LORA_RC_TIME_OUT){
            int dt = millis();
            package = true;
            if(cnt == 0){ DebugPrintln("", DEBUG_LEVEL_1); DebugPrintln("", DEBUG_LEVEL_1); DebugPrintln("", DEBUG_LEVEL_1); t= millis(); }
            //DebugPrintln("package received len: " + String(packetSize)+ " [" + hexString2(encoded[0]) + "]", DEBUG_LEVEL_1);
            current = buffer[1] + 1;
            int rc = rs.Decode(&buffer[2], repaired);
            if (rc > 0) {
              DebugPrintln("Decoding failed: " + String(rc), DEBUG_LEVEL_1);
            } else {
              cnt++;
              RSSIsum += rssi;
              ack = true;
              String corrected = "";
              for(int i = 0; i < msglen; i++) { if (buffer[i+2] != repaired[i]) { corrected = " - corrected!"; break; } }
              for(int i = 0; i < (msglen); i++) { dataSetBuffer[i] = repaired[i]; }
              if (current == last) { twice++; } else { lost += current-last-1; }
              last = current;
              //if(corrected != ""){
                DebugPrintln("No. " + String(cnt) + ", twice: " + String(twice) + ", lost: " + String(lost) + " -  pkg-No: " + String(current) + corrected, DEBUG_LEVEL_1);
              //}
              //_PrintDataSet(&dummy, DEBUG_LEVEL_1);
            }
            loraSendACK(clientID, ack);
            //DebugPrintln("LORA ack takes: " + String(millis() - dt), DEBUG_LEVEL_1);
            packetSize = 0;
            overAll = millis()-t;
          }
        }
        #ifdef USE_OLED
        display.clear();
        display.drawString(0, 0,  "Sets " + String(cnt) );
        display.drawString(0, 20, "lost/twice " + String(lost) + "/" + String(twice) );
        display.drawString(0, 40, "rssi " + String(int(RSSIsum / cnt)));
        display.display();
        #else
        _D(DebugPrintln("rssi: " + String(int(RSSIsum / cnt)), DEBUG_LEVEL_1);)
        #endif
      }
      pause(120000 - ( millis() - t ));
    }
    CloseLoRa();
  } else { setupLoraRc = LORA_SETUP_FAILED; }
  pause(10000);
  #ifdef USE_OLED
  display.clear();
  display.displayOff();
  display.end();
  #endif
  _D(DebugPrintln("LORA: end task ",   DEBUG_LEVEL_1);)
  loraTaskWaterMark = uxTaskGetStackHighWaterMark(NULL);
  task_lora_test = NULL;
  vTaskDelete(NULL);
}
#else
void loraTestClientTask(void *pvParameters) {
  uint8_t buffer[LORA_PKG_DATA_REQ_LEN];
  int cnt = 0;
  int fail = 0;
  int rssi = -255;
  int RSSIsum = 0;
  int t = millis();
  _D(DebugPrintln("LORA: start client task ",   DEBUG_LEVEL_1);)
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
    _D(DebugPrintln("LORA: data message length: " + String(sizeof(encoded)),   DEBUG_LEVEL_1);)
    setupLoraRc = LORA_SETUP_DONE_OK;
    _DD(DebugPrintln("LORA: wait for request",   DEBUG_LEVEL_3);)
    while(!stopLoraTask){
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
          uint8_t setsToSend = buffer[2]:
          if (TEST_DATA_COUNT < setsToSend) { setsToSend = TEST_DATA_COUNT; }
          if(loraDataReady(setsToSend)) {
            for(int m = TEST_DATA_COUNT-1; m >= (TEST_DATA_COUNT - setsToSend); m--){
              bool ack = false;
              uint8_t* data = (uint8_t*)(&loRaTestData[m]);
              for(int i = 0; i < msglen; i++) { message[i] = data[i]; } // Fill with the message
              rs.Encode(message, &encoded[2]);
              encoded[0] = LORA_PKG_ID_ONE_SET;
              encoded[1] = m;
              int repeats = 0;
              while((!ack) && (repeats < LORA_MAX_REPEATS)){
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
                DebugPrintln(" package [" + String(encoded[1]) + "] sent [" + String(millis()-dt) + " ms]", DEBUG_LEVEL_1);
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
              if(repeats >= LORA_MAX_REPEATS) { DebugPrintln("package sent failed [" + String(millis()-t) + " ms]", DEBUG_LEVEL_1); }
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
            #endif
          } _DD( else {DebugPrintln("LORA send data ready message failed", DEBUG_LEVEL_3);} )
        } _DD( else {DebugPrintln("LORA invalis data request message", DEBUG_LEVEL_3);} )
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
  task_lora_test = NULL;
  vTaskDelete(NULL);
}
#endif


#if 0
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
      int rssi = 0;
      while((!ack) && (repeats < LORA_MAX_REPEATS)){
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
      if(repeats >= LORA_MAX_REPEATS) { DebugPrintln("package sent failed [" + String(millis()-t) + " ms]", DEBUG_LEVEL_1); }
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

#endif // HEIDI_CONFIG_TEST

#endif //USE_LORA


