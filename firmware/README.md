# Pin Mapping

|Pin#|Joystick  |CD32       |Mouse      |
|----|----------|-----------|-----      |
|1   |Up        |Up         |V-pulse    |
|2   |Down      |Down       |H-pulse    |
|3   |Left      |Left       |VQ-pulse   |
|4   |Right     |Right      |HQ-pulse   |
|5   |(Button 3)|Load/Shift*|MMB        |
|6   |Button 1  |SR Clock*  |LMB        |
|7   |+5V       |+5V        |+5V        |
|8   |GND       |GND        |GND        |
|9   |Button 2  |SR Output  |RMB        |

All pins are OUTPUTs for the peripherals and INPUTs for the computer/console
EXCEPT those marked with an asterisk.

All signals are active-low and should be drive in an open-collector fashion.
