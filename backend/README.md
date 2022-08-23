# heidi-backend
We're (still) working with Eclipse 2018-12 (4.10.0) and the (old 1.0.5) CDT-libraries by espressif. You can find the reason [here](https://github.com/frankheisig71/TestAccel).
So here you will find the instructions to setup Eclipse 2018 with Arduino-CDT libraries on a Linux machine to build the heidi firmware. You are free to use another dev environment or other versions and to adopt the installation steps.

## Table of contents
* [Install Eclipse](#install-eclipse)
* [Install ESP32 core libs](#esp32-core-libs)
* [Install tool chain](#toolchain)
* [Edit platform.txt](#platform.txt)
* [Change Java version](#java)
* [Add ESP32 libs](#esp32-libs)
* [Enable phyton-serial](#phyton-serial)


## Install Eclipse
* Get Eclipse 4.10 from
https://archive.eclipse.org/eclipse/downloads/drops4/R-4.10-201812060815/ and install it to a directory of your choice.
* Start the executable.
* go to Help -> Install New Software, press "Add" and install download.eclipse.org/mpc/photon as a new repository.
* after downloading, check the selection box -> Next -> Next -> accept -> Finish (installation may take some time)
* restart Eclipse
* go to Help -> Eclipse Marketplace, search for "espressif"
* install "Eclipse C++ IDE for Arduino" (be patient)

## Install ESP32 core libs
* get https://github.com/espressif/arduino-esp32/releases/download/1.0.5/esp32-1.0.5.zip
* copy zip content to ~/.arduinocdt/packages
* get https://github.com/espressif/arduino-esp32/releases/download/1.0.5/package_esp32_index.json and put it to ~/.arduinocdt

## Install tool chain
* check the needed toolchain by opening package_esp32_index.json and searching "platforms": -> "architecture": "esp32", "version": "1.0.5" -> "toolsDependencies" -> "name". In our case it is:  "xtensa-esp32-elf-gcc". Note the needed version. For our case it is: "1.22.0-97-gc752ad5-5.2.0".
* now find the link to the toolchain for your operating system by searching for '"tools": ['. Find the right toolchain ("name") with the right version ("version") -> find the right operating system. For our case it is: ""x86_64-pc-linux-gnu"
* get the package following the link under "url". For our case it is: "https://github.com/espressif/arduino-esp32/releases/download/1.0.5-rc5/xtensa-esp32-elf-linux64-1.22.0-97-gc752ad5-5.2.0.tar.gz"
* extract the tar archive - next we need to copy the content of the root folder from the extracted archive, WITHOUT the root folder
* copy the content to ~/.arduinocdt/packages/esp32/tools/%name%/%version%. For our case it is: ~/.arduinocdt/packages/esp32/tools/xtensa-esp32-elf-gcc/1.22.0-97-gc752ad5-5.2.0/
* do exact the same steps the other toolsDependencies under "platforms": (In our case: "esptool_py" and "mkspiffs") with similar path name structure
* check the symlinks under ~/.arduinocdt/packages/esp32/tools/%name%/%version%/xtensa-esp32-elf (usually they are okay)

## Edit platform.txt
* open ~/.arduinocdt/packages/esp32/esp32/hardware/esp32/1.0.5/platform.txt
* seach for: `## Create partitions.bin`
* comment out: `recipe.objcopy.partitions.bin.pattern={tools.gen_esp32part.cmd} -q "{build.path}/partitions.csv" "{build.path}/{build.project_name}.partitions.bin"`
* insert: `recipe.objcopy.eep.pattern={tools.gen_esp32part.cmd} -q "{runtime.platform.path}/tools/partitions/{build.partitions}.csv" "{build.path}/{build.project_name}.partitions.bin"`
* seach for: `runtime.tools.xtensa-esp32-elf-gcc.path =`
* if found, change it to: `runtime.tools.xtensa-esp32-elf-gcc.path = ~/.arduinocdt/packages/esp32/tools/xtensa-esp32-elf`
* seach for: `runtime.tools.xtensa-esp32-elf-gcc.path =`
* if found, change it to: `runtime.tools.xtensa-esp32-elf-gcc.path = ~/.arduinocdt/packages/esp32/tools/xtensa-esp32-elf`
* seach for: `runtime.tools.esptool.path =`
* if found, change it to: `runtime.tools.esptool.path = ~/.arduinocdt/packages/esp32/tools/esptool`

## Change Java version
* java version for Eclipse 2018 is 1.11
* open eclipse.ini search for: `-vmargs`
* add or change before `-vmargs`: <br>
  `-vm`<br>
  `/usr/lib/jvm/java-1.11.0-openjdk-amd64/bin/java`
* (... or whereever java is installed)

## Add ESP32 libs
* Open Eclipse and go to Help -> Arduino Downloads Manager -> Libraries

For Heidi we need:

* ESP8266 and ESP32 OLED driver for SSD1306 displays
* LoRa
* OneWire
* DallasTemperature
* FEC Reed-Solomon

## Enable phyton-serial
Linux users need to put themselves into the dialout group
* `sudo apt install python-serial`
* `sudo adduser [user] dialout`
