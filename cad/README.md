# heidi - cad
Some CAD- and 3D-printing hints

## Table of contents
* [CAD](#cad)
* [3D-printing](#3d-printing)
  * [Printing direction](#printing-direction)
  * [Dealing with ASA-filament](#dealing-with-asa-filament)
  * [Warping](#warping)
  * [Solutions](#solutions)
  * [Notes](#notes)
* [Schmitt-Trigger](#schmitt-trigger)
  * [KiCad](#kicad)
  * [Get a PCB](#get-a-pcb)
  * [Electronic parts](#electronic-parts)
  

## CAD
All CAD was done with [FreeCad](https://www.freecad.org/). All CAD project files are uploaded too (*.fcstd).

## 3D-printing
For our purposes we decided to use 3D print for building up housings. This is flexible and cost efficient. For more reliability against weather and mechanical stress it would be better to use housings milled from the solid material. There are good offers for 3-axis CNC mills - maybe we will test this later on. (It seems we seriously need to talk to Santa regarding this. :-D )
### Printing direction
For mechanical reasons you need to consider printing direction. 3D printed objects do have less mechanical strength in vertical printing direction, so the orientation of objects during printing process matters. You may use the models or *.stl files with number 3 in the file name to get models already ready for printing. Most of them are doubled and connected by thin walls, to get a stable printing process.
### Dealing with ASA-filament
The outdoor usage of Heidi-Tracker requires a weather proofed housing material. We decided for ASA-filament from PolyMaker (for no special reasons). If you not own a industrial like 3D-printer, printing ASA-filament is not quite simple. The temperature for the material nozzle and the build plate (bed) needs to be adjusted depending on the geometry of the part you intend to print. (As far as we discovered due to many tries.)
### Warping
[(warping - see here)](https://support.ultimaker.com/hc/article_attachments/360009279100/How_to_fix_warping_Warping_model.jpg)<br />
Warping is the main issue while printing ASA filament. There are many hints you can find in the internet to fix warping. 

### Solutions
some solutions to various problems
* to minimize warping you may use a frame, like the one in the picture below, together with at least 20 brim lines. 
![Alt text](cura.jpg?raw=true "Cura")

### Notes
* There are slightly different sizes of GPS-antenna boards used. Currently GPSAntennaClip is for the bigger one (28mm edge length), GPSAntennaClip3 is for the smaller one (25mm edge length) 

## Schmitt-Trigger
As mentioned in the introducing README, the cut-off threshold of the battery protection circuits (2.4 volts) is too low and leads heidi into a dead lock if the voltage goes below 2.6 volts. Therefore we decided to create a own protection circuit, which cuts off at about 2.8 volts and brings heidi back to power at about 3.1 volts. That ensures, that the ESP32 works correctly (when empowered). (By the way: It's our first own PCB - it was a huge fun to research, learn to calculate a Schmitt-Trigger, to test, to explore new CAD-software...)

Keep in mind: You nevertheless need a battery protection circuit. The Schmitt-Trigger will continously draw a small current too. We decided to use LiPo batteries with [included protection](https://www.trustfire.com/collections/batteries/).

### KiCad
The software we're using for schematics an PCB layout is [KiCad](https://www.kicad.org). You find the project files unter /SchmittTrigger. 

### Get a PCB
The PCB has just one layer and no throughplatings - even for the power connections, what saves costs. The gerber files, needed for ordering PCBs online, are in the zip file. We've ordered the PCB [here]( https://aisler.net) -  good price, quick service. 

### Electronic parts
Be sure to use a dual channel rail to rail operational amplifier with a minimum operation voltage of 2.7 volt. Keep an eye on its supply current - it should be less as possible. The P-channel FET is a power-FET, but maybe another will do too, because there are just a few milliamps to manage. 
