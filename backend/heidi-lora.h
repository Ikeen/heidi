/*
 * heidi-lora.h
 *
 *  Created on: 02.12.2021
 *      Author: frank
 */

#ifndef HEIDI_LORA_H_
#define HEIDI_LORA_H_
#include "heidi-defines.h"

#ifdef USE_LORA
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include "heidi-data.h"

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

/*
 * LoRa Spec
 *
 *Europe:
 *        433,05 MHz - 434,79 MHz (ISM-Band Region 1)
 *        863,00 MHz - 870,00 MHz (SRD-Band Europa)
 *North America:
 *        902,00 MHz - 928,00 MHz (ISM-Band Region 2)
 *
 *LoRa data rate = spreadingFactor * (SignalBandwidth / 2^spreadingFactor) * 4 / codingRateDenominator [bps]
 *               = 1098 bps
 */


#define BAND 866000000.00 //#define BAND    868E6
/*
#define spreadingFactor 7 //7..12 +1 ~ +2.5 dB SNR
#define SignalBandwidth 500E3 //250E3 // BW/2 ~ +3-4 dB SNR
#define preambleLength  6
#define codingRateDenominator 5 //5;8 -> 4/5; 4/8
*/
/*
 * see: https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md for settings
 */
#define LORA_BANDWIDTH   500E3 //250E3 // BW/2 ~ +3-4 dB SNR
#define LORA_SPERADING_F 9     //spreading factor7..12 -> +1 ~ +2.5 dB SNR
#define LORA_TX_POWER    14    //18
#define LORA_PREAMB_LEN  6     //preamble length
#define LORA_CR_DENOMNTR 5     //code rate denominator 5..8 -> 4/5 .. 4/8

#define LORA_ACK_SPREAD_F  9
#define LORA_ACK_TX_PWR   18
#define LORA_ACK_PREA_LEN  6
#define LORA_ACK_CR_DEN    8
/*
 * some specifications:
 * we use: BW=500kHz, SF=9, CR=4/5, preamble = 6, no CRC
 * the packages have fix lengths determined by package id
 * if necessary, the pay-load contains a reed solomon checksum with fix length determined by package id too
 *
 * the on air time of packages scales not linearly to package length but divided in steps, depending on the setup
 * use SemTechs SX1272CalculatorUI.exe (runs on wine too) to find the on air times
 *
 * for our setup we got:
 *
 *   1.. 3 bytes = 18.7 ms  (acknowledge  1 byte id = 63[ack] / 62[nack])
 *  33..37 bytes = 59.7 ms  (1 data set + 3 bytes EEC-length + id = 0xb1 + package number no)
 *  69..73 bytes = 100.6 ms (2 data sets + 8 bytes EEC-length + id = 0xb5 + 1st package number no)
 *
 */

/*
 * ACK package length = 3, byte1 = pkg-id, byte2 = animal-id, byte3 = ack/nack
 */
#define LORA_ACK_OFFSET_TIME_MS  15
#define LORA_ACK_PKG_DURATION_MS 30
#define LORA_ACK_TIMEOUT_MS  (LORA_ACK_OFFSET_TIME_MS +  LORA_ACK_PKG_DURATION_MS) //package on air time + 30 ms
#define LORA_MAX_REPEATS     3
/*
 * ACK  package length = 3,
 *      byte0 = pkg-id (receiver),
 *      byte1 = animal-id of ack-receiver,
 *      byte2 = 1: ack, all other: nak
 *
 * DATA package length = 37,
 *      byte0 = pkg-id,
 *      byte1 = set number,
 *      byte2 ..byte33 = 32 bytes payload
 *      byte34..byte37 =  3 bytes reed solomon checksum
 *
 * DATA REQEST package length = 4,
 *      byte0 = pkg-id (receiver),
 *      byte1 = animal-id of client
 *      byte2 = max data sets able to receive,
 *      byte3 = max data sets able to receive (double check),
 *
 * DATA READY package length = 5,
 *      byte0 = pkg-id (sender),
 *      byte1 = animal-id of client,
 *      byte2 = data sets intended to send,
 *      byte3 = data sets intended to send (double check),
 *      byte4 = 0 - one data set per package, 1 - two data sets per package - currently not used
 *
 * GENERAL CALL package length = 9,
 *      byte0 = pkg-id,
 *      byte1 ..byte4 = known client bit field,
 *      byte5 ..byte8 = known client bit field (double check),
 *
 * GENERAL ANSWER package length = 5
 *      byte0 = pkg-id,
 *      byte1 = animal-id of client
 *      byte2 = animal-id of client (double check)
 *      byte3 .. byte4 = clients configuration CRC
 *
 */

#define LORA_TASK_HEAP_SIZE 8192

typedef enum _loraSetupResult_t {
  LORA_SETUP_NOT_DONE,
  LORA_SETUP_DONE_OK,
  LORA_SETUP_FAILED
}loraSetupResult_t;

typedef enum _loraRC_t {
  LORA_RC_TIME_OUT = -1,
  LORA_RC_OK       = 0,
  LORA_RC_ERROR    = 1
}loraRC_t;

typedef enum _loraPackageID_t {
  LORA_PKG_ID_GEN_CALL = 0x11,
  LORA_PKG_ID_GEN_ANSW = 0x12,
  LORA_PKG_ID_ACK      = 0x63,
  LORA_PKG_ID_DATA_REQ = 0xa1,
  LORA_PKG_ID_DATA_RDY = 0xa2,
  LORA_PKG_ID_ONE_SET  = 0xb1,
  LORA_PKG_ID_TWO_SET  = 0xb2
}loraPackageID_t;

#define LORA_PKG_GEN_CALL_LEN 9
#define LORA_PKG_GEN_ANSW_LEN 5
#define LORA_PKG_ACK_LEN (3 + 1) //round up to 4
#define LORA_PKG_ONE_SET_LEN 37
#define LORA_PKG_DATA_REQ_LEN 4
#define LORA_PKG_DATA_RDY_LEN 5

#define LORA_MIN_TRANS_RSSI -110

#define LORA_ACK  1
#define LORA_NACK 0

void setupLoraTask(void);
void closeLoraTask(void);

bool SetupLoRa(void);
void CloseLoRa(void);
bool loraWaitForACK(int* rssi = NULL);
bool loraSendACK(uint8_t recID, bool ack);
#ifdef HEIDI_GATEWAY
void loraGatewayTask(void *pvParameters);
bool loraDoGeneralCall(void);
void loraGeneralCall(void);
int  loraRequestData(uint8_t animalID, uint8_t maxSets, int minRSSI = -255);
#else
void loraClientTask(void *pvParameters);
bool loraDataReady(uint8_t dataSetCount);
#endif
int  loraWaitForDataPackage(loraPackageID_t pkgID, uint8_t *buffer, int bufferSize, int tmOutMs, int *rssi);
bool loraSendPackage(uint8_t *buffer, int size);

#ifdef HEIDI_CONFIG_TEST
bool TestLoRa(void);
#ifdef HEIDI_GATEWAY
void loraTestGatewayTask(void *pvParameters);
#else
void loraTestClientTask(void *pvParameters);
#endif
#endif //HEIDI_CONFIG_TEST

#endif //USE_LORA


#endif /* HEIDI_LORA_H_ */
