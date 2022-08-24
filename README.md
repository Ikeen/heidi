# heidi

![heidi logo](Kuhvernetzer.png?raw=true "Heidi")

GPS tracking system for alpine pasture - or simply sheeps :-)

## Table of contents
* [Licence info](#licence-info)
* [Project](#project)
* [Milestones](#milestones)
* [Solutions](#solutions)
* [News](#news)

## Licence info

You may freely use or copy this project under the following conditions:

* non-commercial use
* no warranties
	
## Project

The project goal:

* creating a tracker system from scratch
* using free available cheap standard electronic components
* 3D printable housings
* with in situ solar power generation for continuous operation
* vitality monitoring
* local LoRa network with client tracker modules for individual animal monitoring
	
## Milestones

* functional prototype - done
* CAD work for housings - done 
* initial web service - done
* 1st gateway tracker module for testings - done
* endurance tests for housing, power supply and modul reliability - done
* 1st client tracker module for testings - done
* endurance tests for lokal LoRa netwok reliability (gateway tracker <-> client trackers) - done
* accelerometer for vitality monitoring - done
* geofencing functionality - done
* store configuration data into flash memory - tbd
* tool for programming configuration data into flash memory - tbd

## Solutions

some solutions to various problems

* sending data via GSM cause Arduino to crash: Try another orientation of GSM-antenna. Sometimes a piece of copper acts like a receiving antenna, what may have an impact on the Arduino board.
* sending / getting data via GSM is unreliable: Check power connection to battery, MOSFET, DC-DC-Converter - they should be strong. In my case I soldered wires directly to IC-pins.
* serial communication between components produces strange characters: internally pullup RX-wire with "pinMode(_RXD, INPUT_PULLUP);". The HardwareSerial library seems not to do that. Most time it works without dedicated pullup - sometimes it doesn't.
* never place GSM-antenna and DC-DC-converter near by together

## News

* after various problems with communication reliability - especially with low battery - we find out, that placing a additional big capacitor on the power input of the DC-DC-converter, solves a buck of problems. It seems we can even omit the backup battery when we do so - but the DC-DC-converter is still needed. The SIM800L module will not work properly with less then 3.8V - even if the manual tells: it will.
* the fixing via belt clip turned out to be not reliable. We went back to the first intended fixing by screwing the hard plastic parts directly to a belt. Description and pictures coming soon on this channel.
* the buffer battery on the GPS module empties within a few days due to the way we use it. Let me explain a little more: keep the GPS module in power save tracking mode will give us quick results (saving time saves power) but it still draws about 10mA, way too much - so we completely cut it off from power. Usually the buffer battery enables the module to backup some information about the last position lock. These information eases and speeds up the next position lock. Unfortunately the battery is weak and its charging to slow, so it will be discharged after a couple of days. Removing the Battery from module and connecting the backup-pin of the UBLOX to 3.3V from Arduino will solve that problem and we will significantly profit from the backup function. The backup draws about 15uA - no problem. Assembly instructions and sketches will be updated soon. (done)
* the battery solar charging modules in Heidi hardware version 1 and 2 have one or two small big problems: if they are, for what reason ever, shortly disconnected from battery, sometimes they may be unable to reconnect the load to the battery. Furthermore their thresholds of hysteresis (deep discharge protection) are too low, so if the voltage goes too low ESP32 will boot in a circle until the modules cut off from load due to under voltage. So far so good, but in case of recharged batteries they will reconnect the load too early, so the ESP32 will directly starts with booting in a circle until the voltage is too low again. We've decided to implement a own protection circuit which cuts off on 2.7V and reconnects on 3.0V. That will keep the ESP32 in save states. Additionally that means, we can use Modules without under voltage protection. BUT: Doing this we loose short circuit protection too. We strongly recommend batteries with included protection PCB. (no barbecue on pasture!)
