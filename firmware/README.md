# OpenPSX2AmigaPadAdapter Firmware

This is the firmware for the OpenPSX2AmigaPadAdapter project.

It it based on the [Arduino](https://www.arduino.cc) platform, since the OpenPSX2AmigaPadAdapter PCB is basically a customized *Arduino Uno* board. Hence you will need [the Arduino software](https://www.arduino.cc/en/Main/Software) to build and flash it to the board, so download and install it. Version 1.8.10 is the latest at the time of writing, and all testing has been done with this version, although any later version should work as well.

## Core
The OpenPSX2AmigaPadAdapter firmware was tested using the default Arduino core on ATmega328P microcontrollers. You can also use ATmega88/A/P/PA or ATmega168/P microcontrollers, they are pin-compatible and slightly cheaper. In this case you will need [MiniCore](https://github.com/MCUdude/MiniCore).

The ATmega328P**B** is also supported by MiniCore but it is NOT pin-compatible. Nevertheless, it MIGHT just work as well. It might also destroy whatever you connect the adapter to, so **do it at your own risk**.

If using MiniCore, take care to the following values under the *Tools* menu before compiling the firmware:
- Clock: 16 MHz external
- LTO: Enabled

## Libraries
You will need to install the following libraries:
- [Arduino-PS2X](https://github.com/SukkoPera/Arduino-PS2X): You need exactly this version, other versions won't do.
- [DigitalIO](https://github.com/SukkoPera/DigitalIO): You need this fork if your board has an ATmega88/A/P/PA or 328P**B** microcontroller, otherwise you can also use [the original version](https://github.com/greiman/DigitalIO).

Note that they are not available in the Arduino Library Manager, so you will need to install them manually. In order to do so, just download the ZIP files and unpack them under ```libraries``` into your sketches directory. Please refer to the [Arduino documentation](https://www.arduino.cc/en/Guide/Libraries#toc5) for more information.

## Bootloader
You can either use a bootloader or not. There's not much difference from the functional point of view, as you will need some specialized hardware anyway:
- If you don't want to use a bootloader, you will need an AVR I(C)SP programmer every time you want to update the firmware. There are cheap clones everywhere, just look for *usbasp* or *tinyisp*.
- If you want to use a bootloader, you will still need an AVR programmer to flash the bootloader (and set the fuses) the first time, unless someone else did it for you. From then on you can just use a USB to Serial adapter. These are cheap too, just search for them. Not that useful unless you want to do some development/debugging, anyway.
- If you want to help with the debugging, you will need the USB to Serial adapter, so the only bonus in using a bootloader is that you can debug and reflash using it alone.
- Note that you can use an Arduino Uno (or similar) board as an AVR ISP programmer, if you already have one: follow [these instructions](https://www.arduino.cc/en/Tutorial/ArduinoISP).

If you go for the bootloader, you can either use the stock Arduino/MiniCore ones or have a look at the [bootloaders](https://github.com/SukkoPera/OpenPSX2AmigaPadAdapter/tree/master/firmware/bootloaders) directory, where you will find some versions of Optiboot that have been customized to flash LD2 when they startup (since there is no led connected to pin 13 as on most Arduino boards). Each of them is accompanied by the command line it was built with and by an ```avrdude``` command that can be used to flash it to the board without using the Arduino software.

## Fuses
Even if you don't want to use a bootloader, you will need to burn it once in order to set some fuses that are internal to the microcontroller to the right values. Then, if you don't want to use a bootloader, just pretend it never happened.

In alternative, the ```bootloaders``` directory contains commands to set the fuses without using the Arduino software.

### ATmega328/P
Just select *Burn Bootloader* from the *Tools* menu.

Note that this will set the fuses so that the board actually expects a bootloader to be installed, but programs seem to run anyway even if you flash them later through a programmer. You will lose 256 bytes of flash but it won't be a problem in this case, since this firmware is much smaller. To fix this properly you will need to set the fuses manually and use value ```0xDF``` for the High fuse.

### Atmega88/A/P/PA or ATmega328PB
Set the following values under the *Tools* menu:
- Clock: 16 MHz external
- BOD: 2.7V
- Serial: UART0 (if applicable)
- Bootloader: Yes or No according to your choice

Finally select *Burn Bootloader* from the *Tools* menu.

## Compiling and Flashing
Just open the sketch located in the ```OpenPSX2AmigaPadAdapter``` in the Arduino software, connect the board and either use the ```Upload``` or ```Upload Using Programmer``` menu option, according to whether you are using the bootloader or not.

## Pin Mapping
Pins of the DB-9 connector are connected as follows on the OpenPSX2AmigaPadAdapter PCB:

|Pin#|Joystick  |CD32       |Mouse      |Arduino Pin#|Arduino Pin Mode (Joystick Mode)|Arduino Pin Mode (CD32 Mode)|Arduino Pin Mode (Mouse Mode)|
|----|----------|-----------|-----------|------------|--------------------------------|----------------------------|-----------------------------|
|1   |Up        |Up         |V-pulse    |4           |INPUT (HIGH)/OUTPUT (LOW)       |INPUT (HIGH)/OUTPUT (LOW)   |OUTPUT                       |
|2   |Down      |Down       |H-pulse    |5           |INPUT (HIGH)/OUTPUT (LOW)       |INPUT (HIGH)/OUTPUT (LOW)   |OUTPUT                       |
|3   |Left      |Left       |VQ-pulse   |6           |INPUT (HIGH)/OUTPUT (LOW)       |INPUT (HIGH)/OUTPUT (LOW)   |OUTPUT                       |
|4   |Right     |Right      |HQ-pulse   |7           |INPUT (HIGH)/OUTPUT (LOW)       |INPUT (HIGH)/OUTPUT (LOW)   |OUTPUT                       |
|5   |(Button 3)|Load/Shift |MMB        |2 (INT0)    |INPUT                           |INPUT                       |INPUT                        |
|6   |Button 1  |SR Clock   |LMB        |3 (INT1)    |INPUT (HIGH)/OUTPUT (LOW)       |INPUT                       |INPUT (HIGH)/OUTPUT (LOW)    |
|7   |+5V       |+5V        |+5V        |-           |-                               |-                           |-                            |
|8   |GND       |GND        |GND        |-           |-                               |-                           |-                            |
|9   |Button 2  |SR Output  |RMB        |8           |INPUT (HIGH)/OUTPUT (LOW)       |OUTPUT                      |INPUT (HIGH)/OUTPUT (LOW)    |

Notes:
- All signals are active-low.
- Direction signals are actually driven in an open-collector fashion, which means they are toggled between INPUT (to make them HIGH) and OUTPUT mode (to make them LOW).
- This way of working requires external pull-up resistors: Those for the direction pins are built-in into the computer, while those for the buttons are on the board.

## AVR I/O Control
This is just for reference, don't care about it if you don't know what it means:

|DDR \ PORT| L              | H                   |
|----------|----------------|---------------------|
| L        | INPUT (5V)     | INPUT + PULL-UP (5V)|
| H        | OUTPUT LOW (0V)| OUTPUT HIGH (5V)    |

## Logic Analyzer Plugin
The [LogicAnalyzer](https://github.com/SukkoPera/OpenPSX2AmigaPadAdapter/tree/master/firmware/LogicAnalyzer) directory contains a plug-in for the [Saleae Logic](https://www.saleae.com/downloads/) software that is capable of decoding the CD<sup>32</sup> controller protocol. It is not perfect but it's usable.

Please refer to the [Saleae documentation](https://github.com/saleae/SampleAnalyzer/tree/master/docs) for information on how to build the analyzer. Note that the directory contains a submodule, be sure to check it out as well.

## License
The OpenPSX2AmigaPadAdapter firmware is Copyright &copy; 2019 by SukkoPera.

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <http://www.gnu.org/licenses/>.
