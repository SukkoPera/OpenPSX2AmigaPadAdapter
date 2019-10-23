# Pin Mapping

|Pin#|Joystick  |CD32       |Mouse      |Arduino Pin#|Arduino Pin Mode (Joystick Mode)|Arduino Pin Mode (CD32 Mode)|Arduino Pin Mode (Mouse Mode)|
|----|----------|-----------|-----------|------------|--------------------------------|----------------------------|-----------------------------|
|1   |Up        |Up         |V-pulse    |4           |INPUT (HIGH)/OUTPUT (LOW)       |INPUT (HIGH)/OUTPUT (LOW)   |OUTPUT                       |
|2   |Down      |Down       |H-pulse    |5           |INPUT (HIGH)/OUTPUT (LOW)       |INPUT (HIGH)/OUTPUT (LOW)   |OUTPUT                       |
|3   |Left      |Left       |VQ-pulse   |6           |INPUT (HIGH)/OUTPUT (LOW)       |INPUT (HIGH)/OUTPUT (LOW)   |OUTPUT                       |
|4   |Right     |Right      |HQ-pulse   |7           |INPUT (HIGH)/OUTPUT (LOW)       |INPUT (HIGH)/OUTPUT (LOW)   |OUTPUT                       |
|5   |(Button 3)|Load/Shift*|MMB        |2 (INT0)    |INPUT                           |INPUT                       |INPUT                        |
|6   |Button 1  |SR Clock*  |LMB        |3 (INT1)    |INPUT (HIGH)/OUTPUT (LOW)       |INPUT                       |INPUT (HIGH)/OUTPUT (LOW)    |
|7   |+5V       |+5V        |+5V        |-           |-                               |-                           |-                            |
|8   |GND       |GND        |GND        |-           |-                               |-                           |-                            |
|9   |Button 2  |SR Output  |RMB        |8           |INPUT (HIGH)/OUTPUT (LOW)       |OUTPUT                      |INPUT (HIGH)/OUTPUT (LOW)    |

- All signals are active-low.
- Direction signals are actually driven in an open-collector fashion, which
  means they are toggled between INPUT (to make them HIGH) and OUTPUT mode (to
  make them LOW).
- This needs external pull-up resistors: Those for the direction pins are
  built-in into the computer, while those for the buttons are on the board.

# AVR I/O Control

|DDR \ PORT| L            | H                   |   |
|----------|--------------|---------------------|---|
| L        | INPUT (5V)   | INPUT + PULL-UP (5V)|   |
| H        | OUTPUT L (0V)| OUTPUT H (5V)       |   |
