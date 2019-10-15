# OpenPSX2AmigaPadAdapter
OpenPSX2AmigaPadAdapter is an Open Hardware adapter that allows using a Sony PlayStation controller on Commodore computers, including the Commodore 64, Amiga and CD<sup>32</sup>.

![Board](https://raw.githubusercontent.com/SukkoPera/OpenPSX2AmigaPadAdapter/master/doc/render-top.png)

## Summary
The Commodore Amiga CD<sup>32</sup> came with cumbersome and fragile controllers, most of which did not survive the challenge of time. While many third-party replacements have surfaced in the meantime, most of them are not as solid and comfortable as the ubiquitous Sony PlayStation controllers. These have more than enough buttons, are cheap and most people are familiar with them, as the same basic design has been around for nearly 25 years, so they would be a very good replacement, if they just didn't use a different connector and communication protocol.

OpenPSX2AmigaPadAdapter is a simple board that adapts the connector and translates the protocol, allowing the use of many PlayStation controllers on Commodore computers. It can either behave as a simple 1/2-button Atari-style joystick or as a full-fledged 7-button CD<sup>32</sup> gamepad. It can also appear as an Amiga mouse, which is handy when it is connected to port 1 of an Amiga computer.

## Operating Modes
The adapter has two leds:
- LD2 lights up steadily when a supported controller is plugged in. It blinks when no controller - or an unsupported one - is connected.
- LD1 indicates the current operating mode, to be interpreted as follows.

### Two-Button Joystick Mode
When the adapter is powered on, it defaults to Atari-style Two-Button Mode, which is indicated by LD1 being off.

This mode has been throughly tested on several Amiga models, but it **should** work wherever an Atari-style joystick is supported, including the Commodore VIC-20, Commodore 16 (through an [adapter](https://github.com/SukkoPera/OpenC16JoyAdapter)), Commodore 64, Sega Master System (but NOT MegaDrive/Genesis), etc. Although these platforms have NOT been tested yet, so use at your own risk.

[Here is a list](http://eab.abime.net/showthread.php?t=57540) of games that somehow support two buttons, if it can be any useful.

While in this mode, the adapter supports different button mappings, which have been carefully designed and tailored to different game genres. The mappings can be switched by pressing <kbd>Select</kbd> in combination with other buttons. LD1 will blink quickly a few times to indicate what mapping has been activated.

#### Standard Mapping: <kbd>Select</kbd> + <kbd>&square;</kbd>
Standard Mapping is the simplest mapping possible: both the D-Pad and Left Analog work as direction buttons. <kbd>&square;</kbd> is <kbd>B1</kbd> and <kbd>&cross;</kbd> is <kbd>B2</kbd>. This is the default mapping as it should be usable just about in every game out there. It might not be the most natural these days, but it's the way the game was meant to be played by the developers, thus it should never fail you. Note that very few games were originally made to take advantage of two buttons, as even fewer controllers had that many (!) those days.

LD1 will blink once when this mapping is activated.

#### Racing Mapping 1: <kbd>Select</kbd> + <kbd>&triangle;</kbd>
Racing Mapping 1 is useful for all those racing games that use <kbd>&uarr;</kbd> to accelerate and <kbd>&darr;</kbd> to brake. These have been mapped to <kbd>&square;</kbd> and <kbd>&cross;</kbd>, respectively, which should make them much more natural to play. When accelerating and braking at the same time, braking wins. Left Analog can be used to steer, but its vertical axis is ignored, to avoid accidental accelerating/braking. The D-Pad is fully functional and is handy when moving through menus. <kbd>B1</kbd> and <kbd>B2</kbd> can be found on <kbd>&triangle;</kbd> and <kbd>&cir;</kbd>.

This mode is probably best suited to games that do not involve shifting gears, as downshifting is usually performed through <kbd>&darr;</kbd> + <kbd>B1</kbd> which is pretty hard to achieve (<kbd>&triangle;</kbd> + <kbd>&cir;</kbd>).

LD1 will blink twice when this mapping is activated.

#### Racing Mapping 2: <kbd>Select</kbd> + <kbd>&cir;</kbd>
Racing Mapping 2 is an alternative mapping for racing games that was inspired by GTA V. It lets you use <kbd>R2</kbd> (or <kbd>R1</kbd>) to accelerate and <kbd>L2</kbd> (or <kbd>L1</kbd>) to brake (which means they map to <kbd>&uarr;</kbd> and <kbd>&darr;</kbd>, respectively). <kbd>B1</kbd> is mapped to its natural <kbd>&square;</kbd> position. Steering and the D-Pad work as in Racing Mode 1.

Accidentally, this control scheme was found out to be very comfortable with games that use <kbd>B1</kbd> to accelerate and <kbd>&uarr;</kbd> and <kbd>&darr;</kbd> to shift gears. Since <kbd>&darr;</kbd> is probably used for braking as well, it has also been mapped to <kbd>&cross;</kbd>, while <kbd>B2</kbd> has been moved to <kbd>&triangle;</kbd>.

LD1 will blink three times when this mapping is activated.

#### Platform Mapping: <kbd>Select</kbd> + <kbd>&cross;</kbd>
Platform Mapping is very similar to Standard Mapping, it just makes jumping way easier on a joypad and more natural to all the Mario players out there, by replicating <kbd>&uarr;</kbd> on <kbd>&cross;</kbd>. Consequently, <kbd>B2</kbd> has been moved to <kbd>&triangle;</kbd>.

LD1 will blink four times when this mapping is activated.

### Mouse Mode
If the right analog stick is moved, the adapter switches to Amiga Mouse Mode. This is indicated by LD1 blinking.

This mode is handy if you have the adapter connected to port 1 of an Amiga computer and you need to use a mouse for short while (maybe to do some settings in a cracktro). Instead of unplugging the adapter, plugging in a mouse, and then the adapter again, you can take advantage of this feature. It can also be useful as an emergency mouse. Movement is somewhat proportional to how far the stick is moved.

Press any direction on the D-Pad to go back to Joystick Mode.

### CD<sup>32</sup> Controller Mode
When the adapter is connected to a CD<sup>32</sup> console, it will automatically switch into this mode, which will emulate all 7 buttons of the original CD<sup>32</sup> controller. LD1 will light up steadily.


Buttons are mapped as follows:
- <kbd>&square;</kbd>: Red
- <kbd>&cross;</kbd>: Blue
- <kbd>&cir;</kbd>: Yellow
- <kbd>&triangle;</kbd>: Green
- <kbd>L1</kbd>/<kbd>L2</kbd>/<kbd>L3</kbd>: L
- <kbd>R1</kbd>/<kbd>R2</kbd>/<kbd>R3</kbd>: R
- <kbd>Start</kbd>: Start/Pause

Both the D-Pad and Left Analog work as direction buttons.

## Compatibility
OpenPSX2AmigaPadAdapter has currently been tested with the following Sony controllers:
- PlayStation Dual Shock Analog Controller (SCPH-1200)
- PlayStation 2 Dual Shock 2 Analog Controller (SCPH-10010)

Most controllers should work, please report (by [opening an issue](https://github.com/SukkoPera/OpenPSX2AmigaPadAdapter/issues/new)) if you find out working or non-working models.

## License
The OpenPSX2AmigaPadAdapter documentation, including the design itself, is copyright &copy; SukkoPera 2019.

OpenPSX2AmigaPadAdapter is Open Hardware licensed under the [CERN OHL v. 1.2](http://ohwr.org/cernohl).

You may redistribute and modify this documentation under the terms of the CERN OHL v.1.2. This documentation is distributed *as is* and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES whatsoever with respect to its functionality, operability or use, including, without limitation, any implied warranties OF MERCHANTABILITY, SATISFACTORY QUALITY, FITNESS FOR A PARTICULAR PURPOSE or infringement. We expressly disclaim any liability whatsoever for any direct, indirect, consequential, incidental or special damages, including, without limitation, lost revenues, lost profits, losses resulting from business interruption or loss of data, regardless of the form of action or legal theory under which the liability may be asserted, even if advised of the possibility or likelihood of such damages.

A copy of the full license is included in file [LICENSE.pdf](LICENSE.pdf), please refer to it for applicable conditions. In order to properly deal with its terms, please see file [LICENSE_HOWTO.pdf](LICENSE_HOWTO.pdf).

The contact points for information about manufactured Products (see section 4.2) are listed in file [PRODUCT.md](PRODUCT.md).

Any modifications made by Licensees (see section 3.4.b) shall be recorded in file [CHANGES.md](CHANGES.md).

The Documentation Location of the original project is https://github.com/SukkoPera/OpenPSX2AmigaPadAdapter/.

## Support the Project
Since the project is open you are free to get the PCBs made by your preferred manufacturer, however in case you want to support the development, you can order them from PCBWay through this link:

[![PCB from PCBWay](https://www.pcbway.com/project/img/images/frompcbway.png)](https://www.pcbway.com/project/shareproject/OpenPSX2AmigaPadAdapter_V1.html)

You get my gratitude and cheap, professionally-made and good quality PCBs, I get some credit that will help with this and [other projects](https://www.pcbway.com/project/member/shareproject/?bmbid=41100). You won't even have to worry about the various PCB options, it's all pre-configured for you!

Also, if you still have to register to that site, [you can use this link](https://www.pcbway.com/setinvite.aspx?inviteid=41100) to get some bonus initial credit (and yield me some more).

Again, if you want to use another manufacturer, feel free to, don't feel obligated :). But then you can buy me a coffee if you want:

<a href='https://ko-fi.com/L3L0U18L' target='_blank'><img height='36' style='border:0px;height:36px;' src='https://az743702.vo.msecnd.net/cdn/kofi2.png?v=2' border='0' alt='Buy Me a Coffee at ko-fi.com' /></a>

### Get Help
If you need help or have questions, you can join [the official Telegram group](https://t.me/joinchat/HUHdWBC9J9JnYIrvTYfZmg).

### Thanks
- Gerd Kautzmann for information about the [CD32 controller protocol](http://gerdkautzmann.de/cd32gamepad/cd32gamepad.html)
- CuriousInventor for informtation about the [PlayStation controller protocol](http://store.curiousinventor.com/guides/PS2)
