# Pin Mapping

|Pin#|Joystick  |CD32       |Mouse      |Arduino Pin#|Arduino Pin Mode                         |
|----|----------|-----------|-----------|------------|-----------------------------------------|
|1   |Up        |Up         |V-pulse    |4           |OUTPUT                                   |
|2   |Down      |Down       |H-pulse    |5           |OUTPUT                                   |
|3   |Left      |Left       |VQ-pulse   |6           |OUTPUT                                   |
|4   |Right     |Right      |HQ-pulse   |7           |OUTPUT                                   |
|5   |(Button 3)|Load/Shift*|MMB        |2           |INPUT                                    |
|6   |Button 1  |SR Clock*  |LMB        |3           |INPUT (CD32 Mode)/OUTPUT(All other modes)|
|7   |+5V       |+5V        |+5V        |-           |-                                        |
|8   |GND       |GND        |GND        |-           |-                                        |
|9   |Button 2  |SR Output  |RMB        |8           |OUTPUT                                   |

- All signals are active-low.
- *OUTPUT* signals are actually driven in an open-collector fashion, which means
  they are toggled between INPUT (to make them HIGH) and OUTPUT mode (to make
  them LOW).
