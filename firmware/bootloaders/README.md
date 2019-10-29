# OpenPSX2AmigaPadAdapter Bootloaders

Here are some [Optiboot](https://github.com/Optiboot/optiboot) binaries compiled and customized for the OpenPSX2AmigaPadAdapter PCB.

They are compiled from the unmodified sources with the following commands:

```
export PATH=$PATH:/opt/arduino-1.8.9/hardware/tools/avr/bin
make atmega328 LED=C1
```

So the only difference versus the binaries included with Arduino and MiniCore are the LED pin, which is C1 on the OpenPSX2AmigaPadAdapter PCB.

## Flashing
You can flash these binaries with the Arduino software or simply with the avrduded tool as follows:

### ATmega328
#### Fuses
```
avrdude -C$HOME/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino14/etc/avrdude.conf -v -patmega328p -cusbasp -Pusb -e -Ulock:w:0x3F:m -Uefuse:w:0xFD:m -Uhfuse:w:0xDE:m -Ulfuse:w:0xFF:m
```

#### Bootloader
```
avrdude -C$HOME/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino14/etc/avrdude.conf -v -patmega328p -cusbasp -Pusb -Uflash:w:optiboot_atmega328.hex:i -Ulock:w:0x0f:m
```

### ATmega88/A:
Use `-patmega88p` and either `optiboot_flash_atmega88p_UART0_57600_16000000L.hex` or `optiboot_flash_atmega88p_UART0_115200_16000000L.hex` as bootloader for ATmega88P.

#### Fuses
```
avrdude -C$HOME/.arduino15/packages/MiniCore/hardware/avr/2.0.3/avrdude.conf -v -patmega88 -cusbasp -Pusb -e -Ulock:w:0x3f:m -Uefuse:w:0xfc:m -Uhfuse:w:0xd7:m -Ulfuse:w:0b11110111:m
```

#### Bootloader
```
avrdude -C$HOME/.arduino15/packages/MiniCore/hardware/avr/2.0.3/avrdude.conf -v -patmega88 -cusbasp -Pusb -Uflash:w:optiboot_atmega88.hex:i -Ulock:w:0x0f:m
```

#### Application
```
avrdude -C$HOME/.arduino15/packages/MiniCore/hardware/avr/2.0.3/avrdude.conf -v -V -patmega88 -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:OpenPSX2AmigaPadAdapter.ino.hex:i 
```
