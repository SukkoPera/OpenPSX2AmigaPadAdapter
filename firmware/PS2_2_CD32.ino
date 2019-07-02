/*******************************************************************************
 * This file is part of OpenPSX2AmigaPadAdapter.                               *
 *                                                                             *
 * Copyright (C) 2019 by SukkoPera <software@sukkology.net>                    *
 *                                                                             *
 * OpenPSX2AmigaPadAdapter is free software: you can redistribute it and/or    *
 * modify it under the terms of the GNU General Public License as published by *
 * the Free Software Foundation, either version 3 of the License, or           *
 * (at your option) any later version.                                         *
 *                                                                             *
 * OpenPSX2AmigaPadAdapter is distributed in the hope that it will be useful,*
 * but WITHOUT ANY WARRANTY; without even the implied warranty of              *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
 * GNU General Public License for more details.                                *
 *                                                                             *
 * You should have received a copy of the GNU General Public License           *
 * along with OpenPSX2AmigaPadAdapter. If not, see http://www.gnu.org/licenses.*
 *******************************************************************************
 *
 * OpenPSX2AmigaPadAdapter - Playstation/Playstation 2 to Commodore Amiga/CD32
 * controller adapter
 *
 * Please refer to the GitHub page and wiki for any information:
 * https://github.com/SukkoPera/OpenPSX2AmigaPadAdapter
 *
 * CD32 pad protocol information found at:
 * http://gerdkautzmann.de/cd32gamepad/cd32gamepad.html
 */

#include <PS2X_lib.h>

// INPUT pins, connected to PS2 controller
const byte PS2_DAT = 12;
const byte PS2_CLK = 11;
const byte PS2_CMD = 10;
const byte PS2_SEL = 9;

// PS2 Controller Class
PS2X ps2x;

// OUTPUT pins, connected to Amiga port
const byte PIN_UP = 4;    // Amiga Pin 1
const byte PIN_DOWN = 5;  // Amiga Pin 2
const byte PIN_LEFT = 6;  // Amiga Pin 3
const byte PIN_RIGHT = 7; // Amiga Pin 4
const byte PIN_BTN1 = 3;  // Amiga Pin 6
const byte PIN_BTN2 = 8;  // Amiga Pin 9

// This pin switches between Amiga (HIGH) and CD32 (LOW) mode
// It also triggers the loading of the button status shift register
const byte PIN_PADMODE = 2; // Amiga Pin 5

/* When in CD32 mode, button status is saved to an 8-bit register that gets
 * shifted out one bit at a time through this pin.
 */
const byte PIN_BTNREGOUT = PIN_BTN2;

// The shifting is clocked by rising edges on this pin
const byte PIN_BTNREGCLK = PIN_BTN1;

// Analog sticks idle value
const byte ANALOG_IDLE_VALUE = 127;

// Dead zone for analog sticks
const byte ANALOG_DEAD_ZONE = 60;

const byte MOUSE_SLOW_DELTA	= 60;
const byte MOUSE_FAST_DELTA = 1;

// Pin for led that lights up whenever the proper controller is detected
const byte PIN_LED_PAD_OK = A0;

// Pin for led that lights up whenever the adapter is in CD32 mode
const byte PIN_LED_MODE_CD32 = 13;


/*******************************************************************************
 * DEBUGGING SUPPORT
 ******************************************************************************/

// Send debug messages to serial port
#define ENABLE_SERIAL_DEBUG

// Print the controller status on serial. Useful for debugging.
//~ #define DEBUG_PAD

/*******************************************************************************
 * END OF SETTINGS
 ******************************************************************************/

enum PadMode {
	MODE_UNKNOWN = 0,
	MODE_JOYSTICK,
	MODE_MOUSE,
	MODE_CD32
};

enum PadError {
	PADERR_NONE = 0,		// AKA No error
	PADERR_NOTFOUND,
	PADERR_WRONGTYPE,
	PADERR_UNKNOWN
};


const word BTN_BLUE =		1U << 0U;
const word BTN_RED =		1U << 1U;
const word BTN_YELLOW =		1U << 2U;
const word BTN_GREEN =		1U << 3U;
const word BTN_FRONT_R =	1U << 4U;
const word BTN_FRONT_L =	1U << 5U;
const word BTN_START =		1U << 6U;
const word BTN_UP =			1U << 8U;
const word BTN_DOWN =		1U << 9U;
const word BTN_LEFT =		1U << 10U;
const word BTN_RIGHT =		1U << 11U;


#ifdef ENABLE_SERIAL_DEBUG
	#define dstart(spd) Serial.begin (spd)
	#define debug(...) Serial.print (__VA_ARGS__)
	#define debugln(...) Serial.println (__VA_ARGS__)
#else
	#define dstart(...)
	#define debug(...)
	#define debugln(...)
#endif

// Button register currently being shifted (buttons only!)
volatile /* register */ byte buttons = 0xFF;

// Button register being updated with ALL inputs (including directions)
/* volatile register */ word buttonsLive = 0xFF;

// Timestamp of last time the pad was switched to CD32 mode
unsigned long lastSwitchedTime = 0;

// True if mouse mode
boolean isMouse = false;


inline PadMode getMode () {
	PadMode padMode = MODE_UNKNOWN;
/*
//  byte sl = digitalRead (PIN_PADMODE);
	//if (sl == HIGH) {
	if (PIND & (1 << PD2)) {
		padMode = MODE_AMIGA;
	} else {
		padMode = MODE_CD32;
	}
*/

	if (lastSwitchedTime > 0 && millis () - lastSwitchedTime <= 200) {
		padMode = MODE_CD32;
	} else if (PIND & (1 << PD2)) {
		padMode = isMouse ? MODE_MOUSE : MODE_JOYSTICK;
	} else {
		padMode = MODE_UNKNOWN;
	}
		
	return padMode;
}

//~ PadMode padmode = MODE_UNKNOWN;

PadError initPad () {
	PadError ret = PADERR_NOTFOUND;

	// clock, command, attention, data, Pressures?, Rumble?
	int error = ps2x.config_gamepad (PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);

	switch (error) {
	case 0:
		// Controller is ready
		switch (ps2x.readType ()) {
		case 0:
			Serial.println (F("Unknown Controller type found"));
			// Well, it might work
			ret = PADERR_NONE;
			break;
		case 1:
			Serial.println (F("DualShock Controller found"));
			ret = PADERR_NONE;
			break;
		case 2:
			Serial.println (F("GuitarHero Controller found"));
			ret = PADERR_WRONGTYPE;
			break;
		case 3:
			Serial.println (F("Wireless Sony DualShock Controller found"));
			ret = PADERR_NONE;
			break;
		}
		break;
	case 1:
		Serial.println (F("No controller found, check wiring"));
		ret = PADERR_NOTFOUND;
		break;
	case 2:
		Serial.println (F("Controller found but not accepting commands"));
		ret = PADERR_UNKNOWN;
		break;
	}

	return ret;
}

word readButtons () {
	word pad_status = 0x00;

	ps2x.read_gamepad ();

	if (ps2x.Button (PSB_START))
		pad_status |= BTN_START;

	//~ if(ps2x.Button(PSB_SELECT))
		//~ Serial.println("Select is being held");

	if (ps2x.Button (PSB_PAD_UP))
		pad_status |= BTN_UP;

	if (ps2x.Button (PSB_PAD_RIGHT))
		pad_status |= BTN_RIGHT;

	if(ps2x.Button (PSB_PAD_LEFT))
		pad_status |= BTN_LEFT;

	if (ps2x.Button (PSB_PAD_DOWN))
		pad_status |= BTN_DOWN;

	if (ps2x.Button (PSB_TRIANGLE))
		pad_status |= BTN_GREEN;

	if (ps2x.Button (PSB_SQUARE))
		pad_status |= BTN_RED;

	if (ps2x.Button (PSB_CROSS))
		pad_status |= BTN_BLUE;

	if (ps2x.Button (PSB_CIRCLE))
		pad_status |= BTN_YELLOW;

	if (ps2x.Button (PSB_L1) || ps2x.Button (PSB_L2) || ps2x.Button (PSB_L3))
		pad_status |= BTN_FRONT_L;

	if (ps2x.Button (PSB_R1) || ps2x.Button (PSB_R2) || ps2x.Button (PSB_R3))
		pad_status |= BTN_FRONT_R;

	//~ if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
		//~ Serial.print("Stick Values:");
		//~ Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
		//~ Serial.print(",");
		//~ Serial.print(ps2x.Analog(PSS_LX), DEC);
		//~ Serial.print(",");
		//~ Serial.print(ps2x.Analog(PSS_RY), DEC);
		//~ Serial.print(",");
		//~ Serial.println(ps2x.Analog(PSS_RX), DEC);
	//~ }

#ifdef DEBUG_PAD
	static word last_pad_status = 0;

	if (pad_status != last_pad_status) {
		debug (F("Pressed: "));
		if (pad_status & BTN_UP)
			debug (F("Up "));
		if (pad_status & BTN_DOWN)
			debug (F("Down "));
		if (pad_status & BTN_LEFT)
			debug (F("Left "));
		if (pad_status & BTN_RIGHT)
			debug (F("Right "));
		if (pad_status & BTN_RED)
			debug (F("Red "));
		if (pad_status & BTN_BLUE)
			debug (F("Blue "));
		if (pad_status & BTN_YELLOW)
			debug (F("Yellow "));
		if (pad_status & BTN_GREEN)
			debug (F("Green "));
		if (pad_status & BTN_FRONT_L)
			debug (F("FrontL "));
		if (pad_status & BTN_FRONT_R)
			debug (F("FrontR "));
		if (pad_status & BTN_START)
			debug (F("Start "));
		debugln ();

		last_pad_status = pad_status;
	}
#endif

	return pad_status;
}

byte n;

// ISR
void onPadModeFalling () {
	// Switching to CD32 mode, pin 6 becomes an input for clock
	pinMode (PIN_BTNREGOUT, OUTPUT);
	//DDRD &= ~(1 << PD3);    // Input

	// Sample input values, they will be shifted out on subsequent clock inputs
	buttons = buttonsLive & 0x7F;   // Lowest 7 bits contains buttons only, already in correct order
	// Note that 8th bit must be 0 so that it will be reported as 1 for ID sequence

	digitalWrite (PIN_BTNREGOUT, !(buttons & 0x01));
/*  if (buttons & 0x01) {
		PORTB &= ~(1 << PB0);
	} else {
		PORTB |= (1 << PB0);
	}*/
	buttons >>= 1;
	buttons |= 1 << 7;  // This will report non-existing buttons 9 as pressed, for the ID sequence
	n = 1;

	lastSwitchedTime = millis ();

	pinMode (PIN_BTNREGCLK, INPUT);
	attachInterrupt (digitalPinToInterrupt (PIN_BTNREGCLK), onClockEdge, RISING);
}

// ISR
void onClockEdge () {
	digitalWrite (PIN_BTNREGOUT, !(buttons & 0x01));
	/*if (buttons & 0x01) {
		PORTB &= ~(1 << PB0);
	} else {
		PORTB |= (1 << PB0);
	}*/
/*  if (++n >= 8) {
		buttons = 0x01;
	} else {
		buttons >>= 1;
	}
	*/
	buttons >>= 1;
	buttons |= 1 << 7;  // This will report non-existing buttons 10 as pressed, for the ID sequence
}

void mypanic (int interval) {
	while (42) {
		digitalWrite (PIN_LED_PAD_OK, HIGH);
		delay (interval);
		digitalWrite (PIN_LED_PAD_OK, LOW);
		delay (interval);
	}
}

void setup () {
	dstart (115200);
	debugln (F("Starting up..."));

	pinMode (PIN_PADMODE, INPUT_PULLUP);
	pinMode (PIN_BTNREGOUT, OUTPUT);
	pinMode (PIN_BTNREGCLK, INPUT);

	pinMode (PIN_LED_PAD_OK, OUTPUT);
	pinMode (PIN_LED_MODE_CD32, OUTPUT);

	// Give wireless PS2 module some time to startup, before configuring it
	delay (300);

	PadError err = initPad ();
	if (err != PADERR_NONE) {
		int interval = 1000;
		switch (err) {
		case PADERR_NOTFOUND:
			debugln (F("No controller found"));
			interval = 200;
			break;
		case PADERR_WRONGTYPE:
			debugln (F("Unsupported controller"));
			interval = 333;
			break;
		case PADERR_UNKNOWN:
		default:
			debugln (F("Cannot initialize controller"));
			interval = 500;
			break;
		}

		mypanic (interval);
	}

	digitalWrite (PIN_LED_PAD_OK, HIGH);

	attachInterrupt (digitalPinToInterrupt (PIN_PADMODE), onPadModeFalling, FALLING);
}

inline void buttonPress (byte pin) {
	//~ digitalWrite (PIN_UP, LOW);
	pinMode (pin, OUTPUT);  // Low is implicit
}

inline void buttonRelease (byte pin) {
	pinMode (pin, INPUT); // Hi-Z
}

void loop () {
	buttonsLive = readButtons ();

	PadMode mode = getMode ();
	if (mode == MODE_JOYSTICK) {
		int rx = ps2x.Analog (PSS_RX);   // 0 ... 255
		int deltaRX = rx - ANALOG_IDLE_VALUE;
		int deltaRXabs = abs (deltaRX);

		int ry = ps2x.Analog (PSS_RY);
		int deltaRY = ry - ANALOG_IDLE_VALUE;
		int deltaRYabs = abs (deltaRY);
		if (deltaRXabs > ANALOG_DEAD_ZONE || deltaRYabs > ANALOG_DEAD_ZONE) {
			// Right analog stick moved, go to Mouse mode
			pinMode (PIN_UP, OUTPUT);
			pinMode (PIN_DOWN, OUTPUT);
			pinMode (PIN_LEFT, OUTPUT);
			pinMode (PIN_RIGHT, OUTPUT);

			isMouse = true;
		} else {
			// Handle directions
			int lx = ps2x.Analog (PSS_LX);   			// 0 ... 255
			int deltaLX = lx - ANALOG_IDLE_VALUE;		// --> -127 ... +128

			int ly = ps2x.Analog (PSS_LY);
			int deltaLY = ly - ANALOG_IDLE_VALUE;

			if ((buttonsLive & BTN_UP) || deltaLY < -ANALOG_DEAD_ZONE) {
				buttonPress (PIN_UP);
			} else {
				buttonRelease (PIN_UP);
			}

			if ((buttonsLive & BTN_DOWN) || deltaLY > +ANALOG_DEAD_ZONE) {
				buttonPress (PIN_DOWN);
			} else {
				buttonRelease (PIN_DOWN);
			}

			if ((buttonsLive & BTN_LEFT) || deltaLX < -ANALOG_DEAD_ZONE) {
				buttonPress (PIN_LEFT);
			} else {
				buttonRelease (PIN_LEFT);
			}

			if ((buttonsLive & BTN_RIGHT) || deltaLX > +ANALOG_DEAD_ZONE) {
				buttonPress (PIN_RIGHT);
			} else {
				buttonRelease (PIN_RIGHT);
			}

			// Fire Buttons
			noInterrupts ();

			if (buttonsLive & BTN_RED) {
				buttonPress (PIN_BTN1);
			} else {
				buttonRelease (PIN_BTN1);
			}

			if (buttonsLive & BTN_BLUE) {
				buttonPress (PIN_BTN2);
			} else {
				buttonRelease (PIN_BTN2);
			}

			interrupts ();
		}
	} else if (mode == MODE_MOUSE) {
		if (buttonsLive & (BTN_UP | BTN_DOWN | BTN_LEFT | BTN_RIGHT)) {
			// Directional button pressed, go back to joystick mode
			isMouse = false;
		} else {
			// Right analog stick works as a mouse - Horizontal axis
			int x = ps2x.Analog (PSS_RX);   // 0 ... 255
			int deltaX = x - ANALOG_IDLE_VALUE;
			int deltaXabs = abs (deltaX);
			if (deltaXabs > ANALOG_DEAD_ZONE) {
				unsigned int period = map (deltaXabs, ANALOG_DEAD_ZONE, ANALOG_IDLE_VALUE, MOUSE_SLOW_DELTA, MOUSE_FAST_DELTA);
				debug (F("x = "));
				debug (x);
				debug (F(" --> period = "));
				debugln (period);

				byte leadingPin;
				byte trailingPin;
				if (deltaX > 0) {
					// Right
					leadingPin = PIN_RIGHT;
					trailingPin = PIN_DOWN;
				} else {
					// Left
					leadingPin = PIN_DOWN;
					trailingPin = PIN_RIGHT;
				}

				static unsigned long tx = 0;

				if (millis () - tx >= period) {
					digitalWrite (leadingPin, !digitalRead (leadingPin));
					tx = millis ();
				}
				
				if (millis () - tx >= period / 2) {
					digitalWrite (trailingPin, !digitalRead (leadingPin));
				}
			}

			// Vertical axis
			int y = ps2x.Analog (PSS_RY);
			int deltaY = y - ANALOG_IDLE_VALUE;
			int deltaYabs = abs (deltaY);
			if (deltaYabs > ANALOG_DEAD_ZONE) {
				unsigned int period = map (deltaYabs, ANALOG_DEAD_ZONE, ANALOG_IDLE_VALUE, MOUSE_SLOW_DELTA, MOUSE_FAST_DELTA);
				debug (F("y = "));
				debug (y);
				debug (F(" --> period = "));
				debugln (period);

				byte leadingPin;
				byte trailingPin;
				if (deltaY > 0) {
					// Down
					leadingPin = PIN_LEFT;
					trailingPin = PIN_UP;
				} else {
					// Up
					leadingPin = PIN_UP;
					trailingPin = PIN_LEFT;
				}

				static unsigned long ty = 0;

				if (millis () - ty >= period) {
					digitalWrite (leadingPin, !digitalRead (leadingPin));
					ty = millis ();
				}
				
				if (millis () - ty >= period / 2) {
					digitalWrite (trailingPin, !digitalRead (leadingPin));
				} else {
					digitalWrite (trailingPin, digitalRead (leadingPin));
				}
			}

			// Buttons
			noInterrupts ();
			
			if (buttonsLive & BTN_FRONT_L) {
				buttonPress (PIN_BTN1);
			} else {
				buttonRelease (PIN_BTN1);
			}

			if (buttonsLive & BTN_FRONT_R) {
				buttonPress (PIN_BTN2);
			} else {
				buttonRelease (PIN_BTN2);
			}

			interrupts ();
		}
	} else if (mode == MODE_CD32) {
		// Directions still behave as in normal joystick mode
		int lx = ps2x.Analog (PSS_LX);   			// 0 ... 255
		int deltaLX = lx - ANALOG_IDLE_VALUE;		// --> -127 ... +128

		int ly = ps2x.Analog (PSS_LY);
		int deltaLY = ly - ANALOG_IDLE_VALUE;

		if ((buttonsLive & BTN_UP) || deltaLY < -ANALOG_DEAD_ZONE) {
			buttonPress (PIN_UP);
		} else {
			buttonRelease (PIN_UP);
		}

		if ((buttonsLive & BTN_DOWN) || deltaLY > +ANALOG_DEAD_ZONE) {
			buttonPress (PIN_DOWN);
		} else {
			buttonRelease (PIN_DOWN);
		}

		if ((buttonsLive & BTN_LEFT) || deltaLX < -ANALOG_DEAD_ZONE) {
			buttonPress (PIN_LEFT);
		} else {
			buttonRelease (PIN_LEFT);
		}

		if ((buttonsLive & BTN_RIGHT) || deltaLX > +ANALOG_DEAD_ZONE) {
			buttonPress (PIN_RIGHT);
		} else {
			buttonRelease (PIN_RIGHT);
		}
	}

	// Handle mode led
	if (mode == MODE_JOYSTICK || (mode == MODE_MOUSE && (millis () / 500) % 2 == 0)) {
		digitalWrite (PIN_LED_MODE_CD32, LOW);
	} else {
		digitalWrite (PIN_LED_MODE_CD32, HIGH);
	}
}
