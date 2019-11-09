# OpenPSX2AmigaPadAdapter Bootloaders
Here are some [Optiboot](https://github.com/Optiboot/optiboot) binaries customized for the OpenPSX2AmigaPadAdapter PCB.

They are compiled from the unmodified sources, the only difference versus the binaries included with Arduino and MiniCore are the LED pin, which is C1 on the OpenPSX2AmigaPadAdapter PCB.

For the commands below to work, you will need to add the AVR-GCC binaries directory to the PATH, i.e.:
```
export PATH=$PATH:/opt/arduino-1.8.10/hardware/tools/avr/bin
```

Note that your path might be different from the above. ```locate avrdude``` might help you find the correct directory.

## Flashing
You can flash these binaries with the Arduino software or simply with the ```avrdude``` tool as follows:

### ATmega328/P
#### Fuses
```
avrdude -C$HOME/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino14/etc/avrdude.conf -v -patmega328p -cusbasp -Pusb -e -Ulock:w:0x3F:m -Uefuse:w:0xFD:m -Uhfuse:w:0xDE:m -Ulfuse:w:0xFF:m
```

#### Bootloader Build
```
make atmega328 LED=C1
```

#### Bootloader Flash
```
avrdude -C$HOME/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino14/etc/avrdude.conf -v -patmega328p -cusbasp -Pusb -Uflash:w:optiboot_atmega328.hex:i -Ulock:w:0x0f:m
```

### ATmega328PB (Untested + TBD)
#### Fuses
avrdude -C$HOME/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino14/etc/avrdude.conf -v -patmega328pb -cusbasp -Pusb -e -Ulock:w:0x3f:m -Uefuse:w:0b11110101:m -Uhfuse:w:0xd6:m -Ulfuse:w:0b11111111:m 

#### Bootloader Build
```
make atmega328pb LED=C1 UART=0 BAUD_RATE=115200 AVR_FREQ=16000000
```


### ATmega88/A:
#### Fuses
```
avrdude -C$HOME/.arduino15/packages/MiniCore/hardware/avr/2.0.3/avrdude.conf -v -patmega88 -cusbasp -Pusb -e -Ulock:w:0x3f:m -Uefuse:w:0xfc:m -Uhfuse:w:0xd7:m -Ulfuse:w:0b11110111:m
```

#### Bootloader Build:
```
make atmega88 LED=C1 BAUD_RATE=115200 AVR_FREQ=16000000
```

#### Bootloader Flash
```
avrdude -C$HOME/.arduino15/packages/MiniCore/hardware/avr/2.0.3/avrdude.conf -v -patmega88 -cusbasp -Pusb -Uflash:w:optiboot_atmega88.hex:i -Ulock:w:0x0f:m
```

#### Application
```
avrdude -C$HOME/.arduino15/packages/MiniCore/hardware/avr/2.0.3/avrdude.conf -v -V -patmega88 -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:OpenPSX2AmigaPadAdapter.ino.hex:i 
```

### ATmega88P/A:
#### Fuses
```
avrdude -C$HOME/.arduino15/packages/MiniCore/hardware/avr/2.0.3/avrdude.conf -v -patmega88p -cusbasp -Pusb -e -Ulock:w:0x3f:m -Uefuse:w:0xfc:m -Uhfuse:w:0xd7:m -Ulfuse:w:0b11110111:m
```

#### Bootloader Build:
```
make atmega88pa LED=C1 UART=0 BAUD_RATE=115200 AVR_FREQ=16000000
```

#### Bootloader Flash
```
avrdude -C$HOME/.arduino15/packages/MiniCore/hardware/avr/2.0.3/avrdude.conf -v -patmega88p -cusbasp -Pusb -Uflash:w:optiboot_atmega88p_UART0_115200_16000000.hex:i -Ulock:w:0x0f:m
```

#### Application
```
avrdude -C$HOME/.arduino15/packages/MiniCore/hardware/avr/2.0.3/avrdude.conf -v -V -patmega88p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:OpenPSX2AmigaPadAdapter.ino.hex:i 
```
