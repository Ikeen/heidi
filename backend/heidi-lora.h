/*
 * heidi-lora.h
 *
 *  Created on: 02.12.2021
 *      Author: frank
 */

#ifndef HEIDI_LORA_H_
#define HEIDI_LORA_H_

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
 /*
 Europe:
         433,05 MHz - 434,79 MHz (ISM-Band Region 1)
         863,00 MHz - 870,00 MHz (SRD-Band Europa)
 North America:
         902,00 MHz - 928,00 MHz (ISM-Band Region 2)
*/
#define BAND  868E6




#endif /* HEIDI_LORA_H_ */
