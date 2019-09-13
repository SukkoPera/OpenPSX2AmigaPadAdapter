# OpenPSX2AmigaPadAdapter
OpenPSX2AmigaPadAdapter is an Open Hardware adapter that allows using a Sony PlayStation controller on Commodore computers, including the Commodore 64, Amiga and CD<sup>32</sup>.

## Summary
The Commodore Amiga CD<sup>32</sup> came with cumbersome and fragile controllers, most of which did not survive the challenge of time. While many third-party replacements have surfaced in the meantime, most of them are not as solid and comfortable as the ubiquitous Sony PlayStation controllers. These have more than enough buttons, are cheap and most people are familiar with them, as the same basic design has been around for nearly 25 years, so they would be a very good replacement, if they just didn't speak a different connector and communication protocol.

OpenPSX2AmigaPadAdapter is a simple board that adapts the connector and translates the protocol, allowing the use of many PlayStation controllers on Commodore computers. It can either behave as a simple 1/2-button Atari-style joystick or as a full-fledged 7-button CD<sup>32</sup> gamepad. It can also appear as an Amiga mouse, which is handy when it is connected to port 1 of an Amiga computer.

### Operating Modes
When the adapter is powered on, it defaults to Atari-style Two-Button Mode. This has been throughly tested on an Amiga 500, but it should work wherever an Atari-style joystick is supported, including the Commodore VIC-20, Commodore 16 (through an [adapter](https://github.com/SukkoPera/OpenC16JoyAdapter)), Commodore 64, Sega Master System (but NOT MegaDrive/Genesis), etc. Although these platforms have NOT been tested yet, so use at your own risk.

If the right analog stick is moved, the adapter switches to Amiga Mouse Mode.

When in Two-Button Joystick mode, the adapter supports different button mappings, better suited to different game genres. The mappings can be switched by pressing <kbd>Select</kbd> in combination with other buttons.

#### Normal Mode: <kbd>Select</kbd> + <kbd>&square;</kbd>
Normal Mode is the simplest mapping possible: both the D-Pad and Left Analog work as direction buttons. <kbd>&square;</kbd> is <kbd>B1</kbd> and <kbd>&cross;</kbd> is <kbd>B2</kbd>. This is the default mapping as it should be usable just about in every game out there. It might not be the most natural these days, but it's the way the game was meant to be played by the developers, thus it should never fail you. Note that very few games were originally made to take advantage of two buttons, as even fewer controllers had that many (!) those days.

#### Racing Mode 1: <kbd>Select</kbd> + <kbd>&triangle;</kbd>
Racing Mode 1 is useful for all those racing games that use <kbd>&uarr;</kbd> to accelerate and <kbd>&darr;</kbd> to brake. These have been mapped to <kbd>&square;</kbd> and <kbd>&cross;</kbd>, respectively, which should make them much more natural to play. When accelerating and braking at the same time, braking wins. Left Analog can be used to steer, but its vertical axis is ignored, to avoid accidental accelerating/braking. The D-Pad is fully functional and is handy when moving through menus. <kbd>B1</kbd> and <kbd>B2</kbd> can be found on <kbd>&triangle;</kbd> and <kbd>&cir;</kbd>.

This mode is probably best suited to games that do not involve shifting gears, as downshifting is usually performed through <kbd>&darr;</kbd> + <kbd>B1</kbd> which is pretty hard to achieve (<kbd>&triangle;</kbd> + <kbd>&cir;</kbd>).

#### Racing Mode 2: <kbd>Select</kbd> + <kbd>&cir;</kbd>
Racing Mode 2 is an alternative mapping for racing games which was inspired by GTA V. It lets you use <kbd>R2</kbd> (or <kbd>R1</kbd>) to accelerate and <kbd>L2</kbd> (or <kbd>L1</kbd>) to brake (which means they map to <kbd>&uarr;</kbd> and <kbd>&darr;</kbd>, respectively). <kbd>B1</kbd> and <kbd>B2</kbd> are mapped to their natural <kbd>&square;</kbd> and <kbd>&cross;</kbd> positions. Steering and the D-Pad work as in Racing Mode 1.

Accidentally, this control scheme is very comfortable with games that use <kbd>B1</kbd> to accelerate and <kbd>&uarr;</kbd> and <kbd>&darr;</kbd> to shift gears, just try it!

#### Platform Mode: <kbd>Select</kbd> + <kbd>&cross;</kbd>
Platform Mode is very similar to Normal Mode, it just makes jumping way easier and more natural to all the Mario players out there, by replicating <kbd>&uarr;</kbd> on <kbd>&cross;</kbd>. Consequently, <kbd>B2</kbd> has been moved to <kbd>&triangle;</kbd>.

### License
OpenPSX2AmigaPadAdapter is Open Hardware released under the GNU General Public License (GPL) v3. If you make any modifications to the board, **you must** contribute them back.

### Disclaimer
OpenPSX2AmigaPadAdapter is provided to you ‘as is’ and without any express or implied warranties whatsoever with respect to its functionality, operability or use, including, without limitation, any implied warranties of merchantability, fitness for a particular purpose or infringement. We expressly disclaim any liability whatsoever for any direct, indirect, consequential, incidental or special damages, including, without limitation, lost revenues, lost profits, losses resulting from business interruption or loss of data, regardless of the form of action or legal theory under which the liability may be asserted, even if advised of the possibility or likelihood of such damages.

### Get Support
If you need help or have questions, you can join [the official Telegram group](https://t.me/joinchat/HUHdWBC9J9JnYIrvTYfZmg).

