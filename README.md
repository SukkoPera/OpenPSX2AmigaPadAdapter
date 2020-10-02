# OpenPSX2AmigaPadAdapter
OpenPSX2AmigaPadAdapter is an Open Hardware adapter that allows using a Sony PlayStation controller on Commodore computers, including the Commodore 64, Amiga and CD<sup>32</sup>.

![Board](https://raw.githubusercontent.com/SukkoPera/OpenPSX2AmigaPadAdapter/master/img/render-top.png)

## Summary
The Commodore Amiga CD<sup>32</sup> came with cumbersome and fragile controllers, most of which did not survive the challenge of time. While many third-party replacements have surfaced in the meantime, most of them are not as solid and comfortable as the ubiquitous Sony PlayStation controllers. These have more than enough buttons, are cheap and most people are familiar with them, as the same basic design has been around for nearly 25 years. This means they would be a very good replacement, if they just didn't use a different connector and communication protocol.

OpenPSX2AmigaPadAdapter is a simple [Arduino](https://www.arduino.cc)-based board that adapts the connector and translates the protocol, allowing the use of many PlayStation controllers on Commodore computers. It can either behave as a simple 1/2-button Atari-style joystick or as a full-fledged 7-button CD<sup>32</sup> gamepad. It can also appear as an Amiga mouse.

## Operating Modes
The adapter has two leds:
- LD1 lights up steadily when a supported controller is plugged in. It blinks when no controller - or an unsupported one - is connected.
- LD2 indicates the current operating mode, to be interpreted as follows.

### Two-Button Joystick Mode
When the adapter is powered on, it defaults to Atari-style Two-Button Mode, which is indicated by LD2 being off.

This mode has been throughly tested on several Amiga models, but it **should** work wherever an Atari-style joystick is supported, including the Commodore VIC-20, Commodore 16 (through an [adapter](https://github.com/SukkoPera/OpenC16JoyAdapter)), Commodore 64, etc. See [below](#computers-and-consoles) for a compatibility table.

While in this mode, the adapter supports different button mappings, which have been carefully designed and tailored to different game genres. The mappings can be switched by pressing <kbd>Select</kbd> in combination with other buttons. LD2 will blink quickly a few times to indicate what mapping has been activated.

#### Standard Mapping: <kbd>Select</kbd> + <kbd>&square;</kbd>
Standard Mapping is the simplest mapping possible: both the D-Pad and Left Analog work as direction buttons. <kbd>&square;</kbd> is <kbd>B1</kbd> and <kbd>&cross;</kbd> is <kbd>B2</kbd>. This is the default mapping as it should be usable just about in every game out there. It might not be the most natural these days, but it's the way the game was meant to be played by the developers, thus it should never fail you.

Note that very few games were originally made to take advantage of two buttons, as even fewer controllers had that many (!) those days. [Here is a list](http://eab.abime.net/showthread.php?t=57540) of Amiga games that somehow support two buttons, if it can be any useful.

LD2 will blink once when this mapping is activated.

#### Racing Mapping 1: <kbd>Select</kbd> + <kbd>&triangle;</kbd>
Racing Mapping 1 is useful for all those racing games that use <kbd>&uarr;</kbd> to accelerate and <kbd>&darr;</kbd> to brake. These have been mapped to <kbd>&square;</kbd> and <kbd>&cross;</kbd>, respectively, which should make them much more natural to play. When accelerating and braking at the same time, braking wins. Left Analog can be used to steer, but its vertical axis is ignored, to avoid accidental accelerating/braking. The D-Pad is fully functional and is handy when moving through menus. <kbd>B1</kbd> and <kbd>B2</kbd> can be found on <kbd>&triangle;</kbd> and <kbd>&cir;</kbd>.

This mode is probably best suited to games that do not involve shifting gears, as downshifting is usually performed through <kbd>&darr;</kbd> + <kbd>B1</kbd> which is pretty hard to achieve (<kbd>&triangle;</kbd> + <kbd>&cross;</kbd>).

LD2 will blink twice when this mapping is activated.

#### Racing Mapping 2: <kbd>Select</kbd> + <kbd>&cir;</kbd>
Racing Mapping 2 is an alternative mapping for racing games that was inspired by GTA V. It lets you use <kbd>R2</kbd> (or <kbd>R1</kbd>) to accelerate and <kbd>L2</kbd> (or <kbd>L1</kbd>) to brake (which means they map to <kbd>&uarr;</kbd> and <kbd>&darr;</kbd>, respectively). <kbd>B1</kbd> is mapped to its natural <kbd>&square;</kbd> position. Steering and the D-Pad work as in Racing Mode 1.

Accidentally, this control scheme was found out to be very comfortable with games that use <kbd>B1</kbd> to accelerate and <kbd>&uarr;</kbd> and <kbd>&darr;</kbd> to shift gears. Since <kbd>&darr;</kbd> is probably used for braking as well, it has also been mapped to <kbd>&cross;</kbd>, while <kbd>B2</kbd> has been moved to <kbd>&triangle;</kbd>.

LD2 will blink three times when this mapping is activated.

#### Platform Mapping: <kbd>Select</kbd> + <kbd>&cross;</kbd>
Platform Mapping is very similar to Standard Mapping, it just makes jumping way easier on a joypad and more natural to all the Mario players out there, by replicating <kbd>&uarr;</kbd> on <kbd>&cross;</kbd>. Consequently, <kbd>B2</kbd> has been moved to <kbd>&triangle;</kbd>.

LD2 will blink four times when this mapping is activated.

#### Custom Mappings: <kbd>Select</kbd> + <kbd>R1</kbd>/<kbd>R2</kbd>/<kbd>L1</kbd>/<kbd>L2</kbd>
What if the built-in mappings are not enough? OpenPSX2AmigaPadAdapter allows you to make your own! And you can have up to four different ones, which are stored internally so that they can be recalled at any time. By default they behave similarly to the Standard Mapping, but they can be customized so that any button produces either the press of a single button or even of a button combo!

The programming procedure is as follows:

1. Press and hold <kbd>Select</kbd>, then press and hold one of <kbd>R1</kbd>/<kbd>R2</kbd>/<kbd>L1</kbd>/<kbd>L2</kbd> until LD2 starts blinking, finally release both buttons. You are now in Programming Mode.
2. Press the button you want to configure. LD2 will flash quickly a few times.
3. Press and hold the single button or button combo you want to be assigned to the button you pressed before. At this stage the D-Pad directions have their obvious meaning, while <kbd>&square;</kbd> represents <kbd>B1</kbd> and <kbd>&cross;</kbd> represents <kbd>B2</kbd>. LD2 will again flash quickly a few times.
4. Release the button or combo you were holding.
5. Repeat steps 2-4 for every button you want to customize.
6. When you are done, press <kbd>Select</kbd> to store the mapping and leave Programming Mode. LD2 will stop blinking and you will be back to Two-Button Joystick Mode.

Note that a mapping you have just programmed is not activated automatically, so you will have to press <kbd>Select</kbd> and one of <kbd>R1</kbd>/<kbd>R2</kbd>/<kbd>L1</kbd>/<kbd>L2</kbd> (and release them quickly) to switch to it.

The Custom Mappings **cannot** be configured so that <kbd>&darr;</kbd> overrides <kbd>&uarr;</kbd> or so that the vertical axis of Left Analog is ignored, still they might be useful here and there. For instance, having <kbd>B1</kbd> + <kbd>&uarr;</kbd> on <kbd>&cross;</kbd> and <kbd>B1</kbd> + <kbd>&darr;</kbd> on <kbd>&triangle;</kbd> makes the Amiga version of *Golden Axe* much more playable.

#### Commodore 64 Mode
Button 2 on Commodore 64 usually behaves in the opposite way at the electrical level, with respect to the other buttons. So a tweak can be enabled to invert the behaviour of button 2, use it if you find that your game of choice always sees it pressed or if it triggers on release rather than on press.

Just hold <kbd>Select</kbd> and press <kbd>Start</kbd> briefly. LD2 will flash once when this tweak is enabled and twice when it is disabled.

### Mouse Mode
Whenever the right analog stick is moved, the adapter switches to Amiga Mouse Mode. In this mode, the right stick emulates the movements of a mouse. Movement speed is somewhat proportional to how far the stick is moved.

This mode can be useful as an emergency mouse, and it will be particularly handy if you have the adapter connected to port 1 of an Amiga computer and you need to use a mouse for short while (maybe to do some settings in a cracktro). Instead of unplugging the adapter, plugging in a mouse, and then the adapter again, you can take advantage of this feature.

Mouse mode is indicated by LD2 blinking. Press any direction on the D-Pad to go back to Joystick or CD32 Mode.

### CD<sup>32</sup> Controller Mode
When the adapter is connected to a CD<sup>32</sup> console, it will automatically switch into this mode, which will emulate all 7 buttons of the original CD<sup>32</sup> controller. LD2 will light up steadily.

By default, buttons are mapped as follows:
- <kbd>&square;</kbd>: Red
- <kbd>&cross;</kbd>: Blue
- <kbd>&cir;</kbd>: Yellow
- <kbd>&triangle;</kbd>: Green
- <kbd>L1</kbd>/<kbd>L2</kbd>/<kbd>L3</kbd>: L
- <kbd>R1</kbd>/<kbd>R2</kbd>/<kbd>R3</kbd>: R
- <kbd>Start</kbd>: Start/Pause

If you press <kbd>Select</kbd>, the 4 main buttons get "rotated":
- <kbd>&cross;</kbd>: Red
- <kbd>&cir;</kbd>: Blue
- <kbd>&triangle;</kbd>: Yellow
- <kbd>&square;</kbd>: Green

Both the D-Pad and Left Analog always work as direction buttons.

## Components and Assembly
The board is basically a customized Arduino Uno, this means it was designed to work with an ATmega328P microcontroller, but you can also use ATmega88/A/P/PA or ATmega168/P microcontrollers, as they are pin-compatible and slightly cheaper. The A/P/PA suffixes usually identify somewhat minor chip revisions, the board should work with all of them. If you can, use P or PA versions, which consume less power. Note that at the moment the firmware uses 99% of the flash space available on an ATmega88 and any possible future improvements and/or new features are likely to overflow that, so you'd better use an ATmega168 at least.

A noteworthy exception to this rule is the ATmega328P**B**, which is NOT 100% pin-compatible with the 328P. Nevertheless, it MIGHT just work as well. It might also destroy whatever you connect the adapter to, so do it **at your own risk**.

You are recommended to solder the microcontroller first. The TQFP-32 package is easier to solder than it looks, watch some videos on YouTube and develop your own technique. Just make sure to orient it correctly.

Solder the oscillator then. You can either use a through-hole crystal with its caps (these will depend on the crystal, but usually 18-22pF are a good bet) or a 3-pin resonator. Whichever you choose, it shall have a frequency of 16 MHz. If you choose the crystal, you might want to put some insulation tape under it to avoid it touching the pads for the resonator.

Solder all the remaining parts in the order you prefer, just keep the controller connectors last.

Note that the PlayStation controller is powered at 3.3V and the interface signals use the same voltage. Due to the particular circuit used for level shifting (the microcontroller works at 5V), the MOSFETs [should really be BSS138](https://electronics.stackexchange.com/questions/367052/replace-bss138-with-ao3400a-in-level-shifter-circuit).

The PlayStation controller connector can be found from many Chinese sellers as a spare part. Get one with 90° pins, otherwise the adapter will be pretty awkward to use. The pins will be pretty short, but you should manage to solder it in place. Make sure it sits level on the board, otherwise it might not fit in the [3D-printable case](#enclosure), which you are recommended to use in order to make the adapter more mechanically solid. In alternative, some hot glue behind the connector (where the pins are) seems to do the same job, but it won't look as pretty.

**Hint:** If you don't have the PCB, you can build a full adapter with an Arduino Uno/Nano/Whatever board. This is not supported though, so you are on your own, but all the information you need is in the schematics or in the firmware code :).

## Firmware
Before you can use the adapter, you will need to load some firmware (i.e.: an Arduino sketch) on it. This can be found under the [firmware](https://github.com/SukkoPera/OpenPSX2AmigaPadAdapter/tree/master/firmware) directory, along with instructions.

## Compatibility
### Computers and Consoles
|System                           |Compatible             |Notes                                                                                                                                                                                |
|---------------------------------|-----------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|Commodore Amiga (All models)     |![Yes](img/yes.png)    |Tested on A500+ and A1200 (1.D3), it is expected to be compatible with all models in both 2- and 7-button Joystick modes and in Mouse mode.                                          |
|Commodore Amiga CD<sup>32</sup>  |![Yes](img/yes.png)    |Both 2- and 7-button Joystick modes and Mouse mode. Games not using `lowlevel.library` might be a bit of hit-and-miss, please report any misbehaviours.                              |
|Commodore CDTV                   |![Maybe](img/maybe.png)|**Not tested yet**, but expected to work in 2-button Joystick mode and Mouse mode through a connector adapter.                                                                       |
|Commodore 64                     |![Yes](img/yes.png)    |Tested in 2-button Joystick mode, but second button untested. Mouse mode might damage CIA chips though, be careful not to turn it on!                                                |
|Commodore 16                     |![Maybe](img/maybe.png)|**Not tested yet**, but expected to work in 2-button Joystick mode through [OpenC16JoyAdapter](https://github.com/SukkoPera/OpenC16JoyAdapter). C16 only supports Button 1 though.   |
|Commodore VIC-20                 |![Maybe](img/maybe.png)|**Not tested yet**, but expected to work in 2-button Joystick mode. Be careful with Mouse mode.                                                                                      |
|Sega Master System               |![No](img/no.png)      |Would probably work in 2-button Joystick mode if power was routed from pin 5 on the SMS controller port to pin 7 of the adapter.                                                     |
|Sega Mega Drive/Genesis          |![No](img/no.png)      |Would probably work as an SMS controller with the same mod as above. Some more pin rerouting AND a custom firmware could make it appear as a Mega Drive 6-button controller.         |
|MSX                              |![No](img/no.png)      |Would probably work in 2-button Joystick mode by swapping a few pins on the controller port.                                                                                         |

### Controllers
The latest versions of the OpenPSX2AmigaPadAdapter firmware use [PsxNewLib](https://github.com/SukkoPera/PsxNewLib) to read the PlayStation controller. This makes it compatible with (almost) all controllers.

Please refer to the [PsxNewLib Compatibility List](https://github.com/SukkoPera/PsxNewLib#compatibility-list) for details.

## Current Consumption
Following are the results of some rough measurements:
- An ATmega88PA-based OpenPSX2AmigaPadAdapter with an original Sony Dual Shock/Dual Shock 2 controller connected draws about 20 mA.
- Consumption rises to 25 mA with the dongle of my EastVita wireless controller, when the controller is connected. It peaks to 35 mA while it is searching for the controller.
- Using an ATmega328P bumps all consumptions by 5 mA.

This means that the current absorbed by the adapter is "reasonable" and that it should be safe to use in all cases. It is well below the 100 mA maximum available from any Amiga controller ports, for instance.

## Enclosure
The [enclosure](https://github.com/SukkoPera/OpenPSX2AmigaPadAdapter/tree/master/enclosure) directory contains models for a 3D-printable enclosure/case. It was kindly contributed by Petros Kokotis, who has all my gratitude for his great work and support.

### Releases
If you want to get this board produced, you are recommended to get [the latest release](https://github.com/SukkoPera/OpenPSX2AmigaPadAdapter/releases) rather than the current git version, as the latter might be under development and is not guaranteed to be working.

Every release is accompanied by its Bill Of Materials (BOM) file and any relevant notes about it, which you are recommended to read carefully.

## License
The OpenPSX2AmigaPadAdapter documentation, including the design itself, is copyright &copy; SukkoPera 2019-2020.

OpenPSX2AmigaPadAdapter is Open Hardware licensed under the [CERN OHL v. 1.2](http://ohwr.org/cernohl).

You may redistribute and modify this documentation under the terms of the CERN OHL v.1.2. This documentation is distributed *as is* and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES whatsoever with respect to its functionality, operability or use, including, without limitation, any implied warranties OF MERCHANTABILITY, SATISFACTORY QUALITY, FITNESS FOR A PARTICULAR PURPOSE or infringement. We expressly disclaim any liability whatsoever for any direct, indirect, consequential, incidental or special damages, including, without limitation, lost revenues, lost profits, losses resulting from business interruption or loss of data, regardless of the form of action or legal theory under which the liability may be asserted, even if advised of the possibility or likelihood of such damages.

A copy of the full license is included in file [LICENSE.pdf](LICENSE.pdf), please refer to it for applicable conditions. In order to properly deal with its terms, please see file [LICENSE_HOWTO.pdf](LICENSE_HOWTO.pdf).

The contact points for information about manufactured Products (see section 4.2) are listed in file [PRODUCT.md](PRODUCT.md).

Any modifications made by Licensees (see section 3.4.b) shall be recorded in file [CHANGES.md](CHANGES.md).

The Documentation Location of the original project is https://github.com/SukkoPera/OpenPSX2AmigaPadAdapter/.

## Support the Project
Since the project is open you are free to get the PCBs made by your preferred manufacturer, however in case you want to support the development, you can order them from PCBWay through this link:

[![PCB from PCBWay](https://www.pcbway.com/project/img/images/frompcbway.png)](https://www.pcbway.com/project/shareproject/OpenPSX2AmigaPadAdapter_V3.html)

You get my gratitude and cheap, professionally-made and good quality PCBs, I get some credit that will help with this and [other projects](https://www.pcbway.com/project/member/shareproject/?bmbid=41100). You won't even have to worry about the various PCB options, it's all pre-configured for you!

Also, if you still have to register to that site, [you can use this link](https://www.pcbway.com/setinvite.aspx?inviteid=41100) to get some bonus initial credit (and yield me some more).

Again, if you want to use another manufacturer, feel free to, don't feel obligated :). But then you can buy me a coffee if you want:

<a href='https://ko-fi.com/L3L0U18L' target='_blank'><img height='36' style='border:0px;height:36px;' src='https://az743702.vo.msecnd.net/cdn/kofi2.png?v=2' border='0' alt='Buy Me a Coffee at ko-fi.com' /></a>

### Get Help
If you need help or have questions, you can join [the official Telegram group](https://t.me/joinchat/HUHdWBC9J9JnYIrvTYfZmg).

### Thanks
- Gerd Kautzmann for information about the [CD32 controller protocol](http://gerdkautzmann.de/cd32gamepad/cd32gamepad.html)
- CuriousInventor for informtation about the [PlayStation controller protocol](http://store.curiousinventor.com/guides/PS2)
- majinga/screwbreaker for testing and bugfixes
- Petros Kokotis for the 3D-printable enclosure
- Sarang Dumbre for the [3D model of the PlayStation connector](https://grabcad.com/library/ps-connector-female-1)
