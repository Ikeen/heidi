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
#define spreadingFactor 7 //7..12 +1 ~ +2.5 dB SNR
#define SignalBandwidth 500E3 //250E3 // BW/2 ~ +3-4 dB SNR
#define preambleLength  6
#define codingRateDenominator 5 //5;8 -> 4/5; 4/8
#define TX_POWER        18

#define LORA_ACK_PKG_CODE    0x63
#define LORA_ACK_ACK         0x80
#define LORA_ACK_NACK        0x81
#define LORA_ACK_TIMEOUT_MS  50
#define LORA_MAX_REPEATS     5

bool SetupLoRa(void);
void CloseLoRa(void);
bool waitForACK(uint8_t* code, int* rssi = NULL);
void sendACK(uint8_t coded);

#ifdef HEIDI_CONFIG_TEST
void TestLoRa(void);
#endif //HEIDI_CONFIG_TEST

#endif //USE_LORA


#endif /* HEIDI_LORA_H_ */
