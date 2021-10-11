# heidi
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
* 1st client tracker module for testings - tbd
* endurance tests for lokal LoRa netwok reliability (gateway tracker <-> client trackers) - tbd
* accelerometer for vitality monitoring - done
* geofencing functionality - done

## Solutions

some solutions to various problems

* sending data via GSM cause Arduino to crash: Try another orientation of GSM-antenna. Sometimes a piece of copper acts like a receiving antenna, what may have an impact on the Arduino board.
* sending / getting data via GSM is unreliable: Check power connection to battery, MOSFET, DC-DC-Converter - they should be strong. In my case I soldered wires directly to IC-pins.
* serial communication between components produces strange characters: internally pullup RX-wire with "pinMode(_RXD, INPUT_PULLUP);". The HardwareSerial library seems not to do that. Most time it works without dedicated pullup - sometimes it doesn't.

## News

* the fixing via belt clip turned out to be not reliable. We went back to the first intended fixing by screwing the hard plastic parts directly to a belt. Descrition and pictures coming soon.
