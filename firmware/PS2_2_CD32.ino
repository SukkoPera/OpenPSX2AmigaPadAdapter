/** \file psx.ino
 * \author SukkoPera <software@sukkology.net>
 * \date 25 Sep 2019
 * \brief  Playstation/Playstation 2 to Commodore Amiga/CD32 controller adapter
 */

/*******************************************************************************
 * This file is part of OpenPSX2AmigaPadAdapter.                               *
 *                                                                             *
 * Copyright (C) 2019 by SukkoPera <software@sukkology.net>                    *
 *                                                                             *
 * OpenPSX2AmigaPadAdapter is free software: you can redistribute it and/or    *
 * modify it under the terms of the GNU General Public License as published by *
 * the Free Software Foundation, either version 3 of the License, or           *
 * (at your option) any later version.         Q                                *
 *                                                                             *
 * OpenPSX2AmigaPadAdapter is distributed in the hope that it will be useful,  *
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
 *
 */

#include <EEPROM.h>
#include <util/crc16.h>
#include <PS2X_lib.h>

//~ #define ENABLE_FACTORY_RESET

// INPUT pins, connected to PS2 controller
const byte PS2_CLK = 13;
const byte PS2_DAT = 12;
const byte PS2_CMD = 11;
const byte PS2_SEL = 10;

// PS2 Controller Class
PS2X ps2x;

// OUTPUT pins, connected to Amiga port
const byte PIN_UP = 4;    // Amiga Pin 1
const byte PIN_DOWN = 5;  // Amiga Pin 2
const byte PIN_LEFT = 6;  // Amiga Pin 3
const byte PIN_RIGHT = 7; // Amiga Pin 4
const byte PIN_BTN1 = 3;  // Amiga Pin 6
const byte PIN_BTN2 = 8;  // Amiga Pin 9

/** \brief Controller mode input pin
 *  
 * This pin switches between Amiga (HIGH) and CD32 (LOW) mode.
 * 
 * It also triggers the loading of the button status shift register.
 */
const byte PIN_PADMODE = 2; // Amiga Pin 5

/** \brief Shift register output pin for CD32 mode
 * 
 * When in CD32 mode, button status is saved to an 8-bit register that gets
 * shifted out one bit at a time through this pin.
 */
const byte PIN_BTNREGOUT = PIN_BTN2;

/** \brief Shift register clock input pin for CD32 mode
 * 
 * The shifting is clocked by rising edges on this pin.
 */
const byte PIN_BTNREGCLK = PIN_BTN1;

/** \brief Analog sticks idle value
 * 
 * Value reported when the analog stick is in the (ideal) center position.
 */
const byte ANALOG_IDLE_VALUE = 128;

/** \brief Dead zone for analog sticks
 *  
 * If the analog stick moves less than this value from the center position, it
 * is considered still.
 * 
 * \sa ANALOG_IDLE_VALUE
 */
const uint8_t ANALOG_DEAD_ZONE = 65;

/** \brief Delay of the quadrature square waves when mouse is moving at the
 * \a slowest speed
 */
const byte MOUSE_SLOW_DELTA	= 60;

/** \brief Delay of the quadrature square waves when mouse is moving at the
 * \a fastest speed.
 * 
 * Note that a 16 MHz Arduino Uno produces irregular signals if this is too
 * small and mouse movements will be affected. The smallest value producing
 * decently-shaped waves seems to be 6.
 */
const byte MOUSE_FAST_DELTA = 6;

/** \brief LED2 pin
 * 
 * Pin for led that lights up whenever the proper controller is detected.
 */
const byte PIN_LED_PAD_OK = A1;

/** \brief LED1 pin
 * 
 * Pin for led that lights up whenever the adapter is in CD32 mode.
 */
const byte PIN_LED_MODE = A0;

/** \brief CD32 mode timeout
 * 
 * Normal joystick mode will be entered if PIN_PADMODE is not toggled for this
 * amount of milliseconds.
 */
const byte TIMEOUT_CD32_MODE = 200;

/** \brief Single-button debounce time
 * 
 * A combo will be considered valid only after it has been stable for this
 * amount of milliseconds.
 * 
 * \sa debounceButtons()
 */
const unsigned long DEBOUNCE_TIME_BUTTON = 30;

/** \brief Combo debounce time
 * 
 * A combo will be considered valid only after it has been stable for this
 * amount of milliseconds
 * 
 * \sa debounceButtons()
 */
const unsigned long DEBOUNCE_TIME_COMBO = 150;


/*******************************************************************************
 * DEBUGGING SUPPORT
 ******************************************************************************/

// Send debug messages to serial port
//~ #define ENABLE_SERIAL_DEBUG

// Print the controller status on serial. Useful for debugging.
//~ #define DEBUG_PAD


/*******************************************************************************
 * END OF SETTINGS
 ******************************************************************************/

/** \brief State machine states
 * 
 * Possible states for the internal state machine that drives the whole thing.
 */
enum State {
	ST_NO_CONTROLLER,			//!< No controller connected
	ST_FIRST_READ,				//!< First time the controller is read
	
	// Main functioning modes
	ST_JOYSTICK,				//!< Two-button joystick mode
	ST_MOUSE,					//!< Mouse mode
	ST_CD32,					//!< CD32-controller mode
	
	// States to select mapping or go into programming mode
	ST_SELECT_HELD,				//!< Select being held
	ST_SELECT_AND_BTN_HELD,		//!< Select + mapping button being held
	ST_ENABLE_MAPPING,			//!< Select + mapping button released, enable mapping
	
	// States for programming mode
	ST_WAIT_SELECT_RELEASE,		//!< Select released, entering programming mode
	ST_WAIT_BUTTON_PRESS,		//!< Programmable button pressed
	ST_WAIT_BUTTON_RELEASE,		//!< Programmable button released
	ST_WAIT_COMBO_PRESS,		//!< Combo pressed
	ST_WAIT_COMBO_RELEASE,		//!< Combo released
	ST_WAIT_SELECT_RELEASE_FOR_EXIT,	//!< Wait for releact to be released to go back to joystick mode
	
	// States for factory reset
#ifdef ENABLE_FACTORY_RESET
	ST_FACTORY_RESET_WAIT_1,
	ST_FACTORY_RESET_WAIT_2,
	ST_FACTORY_RESET_PERFORM
#endif
};

/** \brief Current state of the internal state machine
 * 
 * We start out as a simple joystick.
 */
volatile State state = ST_NO_CONTROLLER;

//! \name Button bits for CD32 mode
//! @{
const word BTN_BLUE =		1U << 0U;	//!< \a Blue Button
const word BTN_RED =		1U << 1U;	//!< \a Red Button
const word BTN_YELLOW =		1U << 2U;	//!< \a Yellow Button
const word BTN_GREEN =		1U << 3U;	//!< \a Green Button
const word BTN_FRONT_R =	1U << 4U;	//!< \a Front \a Right Button
const word BTN_FRONT_L =	1U << 5U;	//!< \a Front \a Left Button
const word BTN_START =		1U << 6U;	//!< \a Start/Pause Button
//! @}

// This is only used for blinking the led when mapping is changed
enum JoyButtonMapping {
	JMAP_NORMAL = 1,
	JMAP_RACING1,
	JMAP_RACING2,
	JMAP_PLATFORM,
	JMAP_CUSTOM
};

/** \brief Structure representing a standard 2-button Atari-style joystick
 * 
 * This used for gathering button presses according to different button
 * mappings.
 * 
 * True means pressed.
 */
struct TwoButtonJoystick {
	boolean up: 1;			//!< Up/Forward direction
	boolean down: 1;		//!< Down/Backwards direction
	boolean left: 1;		//!< Left direction
	boolean right: 1;		//!< Right direction
	boolean b1: 1;			//!< Button 1
	boolean b2: 1;			//!< Button 2
};

/** \brief Joystick mapping function
 * 
 * This represents a function that should inspect the buttons currently being
 * pressed on the PSX controller and somehow map them to a #TwoButtonJoystick to
 * be sent to the DB-9 port.
 */
typedef void (*JoyMappingFunc) (TwoButtonJoystick& j);

// Default button mapping function prototype for initialization of the following
void mapJoystickNormal (TwoButtonJoystick& j);

//! \brief Joystick mapping function currently in effect
JoyMappingFunc joyMappingFunc = mapJoystickNormal;

/** \brief Number of buttons on a PSX controller
 *
 * Includes *everything*.
 */
const byte PSX_BUTTONS_NO = 16;

/** \brief Number of buttons on the PSX controller that can be mapped
 * 
 * Only 11 buttons can be mapped (X/O/^/[]/Lx/Rx/Start), but the way we store
 * these needs 2 extra slots.
 *
 * \sa button2position()
 */
const byte MAPPABLE_BUTTONS_NO = 13;

/** \brief Map a PSX button to a two-button-joystick combo
 * 
 * There's an entry for every mappable button. Use #button2position() to convert
 * the button to the index to use in the array.
 * 
 * \sa isButtonMappable()
 */
struct ControllerConfiguration {
	/** Two-button joystick combo to send out when the corresponding button is
	 * pressed
	 */
	TwoButtonJoystick buttonMappings[MAPPABLE_BUTTONS_NO];
};

/** \brief All possible controller configurations
 * 
 * Since these are activated with SELECT + a button, ideally there can be as
 * many different ones as other buttons we have (i.e.: #PSX_BUTTONS_NO). In
 * practice, we will start handling only a handful.
 */
ControllerConfiguration controllerConfigs[PSX_BUTTONS_NO];

//! \brief Custom controller configuration currently selected
ControllerConfiguration *currentCustomConfig = NULL;

/** \brief Button register for CD32 mode being updated
 * 
 * This shall be updated as often as possible, and is what gets sampled when we
 * get a falling edge on #PIN_PADMODE.
 * 
 * 0 means pressed, MSB must be 1 for the ID sequence
 */
byte buttonsLive = 0x80;

/** \brief Button register for CD32 mode currently being shifted out
 * 
 * This is where #buttonsLive gets copied when it is sampled.
 * 
 * 0 means pressed, etc.
 */
volatile byte buttons;

//! Timestamp of last time the pad was switched out of CD32 mode
unsigned long lastSwitchedTime = 0;

/** \brief Type that is used to report button presses
 *
 * This can be used with the PSB_* values from PS2X_lib, and cast from/to
 * values of that type.
 */
typedef unsigned int Buttons;

//! Value of #Buttons when it reports no buttons pressed
const Buttons NO_BUTTON = 0x00;

#ifdef ENABLE_SERIAL_DEBUG
	#include <avr/pgmspace.h>
	typedef const __FlashStringHelper * FlashStr;
	typedef const byte* PGM_BYTES_P;
	#define PSTR_TO_F(s) reinterpret_cast<const __FlashStringHelper *> (s)

	#define dstart(spd) Serial.begin (spd)
	#define debug(...) Serial.print (__VA_ARGS__)
	#define debugln(...) Serial.println (__VA_ARGS__)
#else
	#define dstart(...)
	#define debug(...)
	#define debugln(...)
#endif

boolean initPad () {
	boolean ret = false;

	// clock, command, attention, data, Pressures?, Rumble?
	int error = ps2x.config_gamepad (PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
	switch (error) {
	case 0:
		// Controller is ready
		switch (ps2x.readType ()) {
		case 0:
			/* The Dual Shock controller gets recognized as this sometimes, or
			 * anyway, whatever controller it is, it might work
			 */
			debugln (F("Unknown Controller type found, using anyway"));
			ret = true;
			break;
		case 1:
			debugln (F("DualShock Controller found"));
			ret = true;
			break;
		case 2:
			/* We used to refuse this, as it does not look suitable, but then we
			 * found out that the Analog Controller (SCPH-1200) gets detected as
			 * this... :/
			 */
			debugln (F("Analog/GuitarHero Controller found"));
			ret = true;
			break;
		case 3:
			debugln (F("Wireless Sony DualShock Controller found"));
			ret = true;
			break;
		}
		break;
	case 1:
		debugln (F("No controller found"));
		break;
	case 2:
	default:
		debugln (F("Cannot initialize controller"));
		break;
	}

	return ret;
}

const char buttonSelectName[] PROGMEM = "Select";
const char buttonL3Name[] PROGMEM = "L3";
const char buttonR3Name[] PROGMEM = "R3";
const char buttonStartName[] PROGMEM = "Start";
const char buttonUpName[] PROGMEM = "Up";
const char buttonRightName[] PROGMEM = "Right";
const char buttonDownName[] PROGMEM = "Down";
const char buttonLeftName[] PROGMEM = "Left";
const char buttonL2Name[] PROGMEM = "L2";
const char buttonR2Name[] PROGMEM = "R2";
const char buttonL1Name[] PROGMEM = "L1";
const char buttonR1Name[] PROGMEM = "R1";
const char buttonTriangleName[] PROGMEM = "Triangle";
const char buttonCircleName[] PROGMEM = "Circle";
const char buttonCrossName[] PROGMEM = "Cross";
const char buttonSquareName[] PROGMEM = "Square";

const char* const psxButtonNames[PSX_BUTTONS_NO] PROGMEM = {
	buttonSelectName,
	buttonL3Name,
	buttonR3Name,
	buttonStartName,
	buttonUpName,
	buttonRightName,
	buttonDownName,
	buttonLeftName,
	buttonL2Name,
	buttonR2Name,
	buttonL1Name,
	buttonR1Name,
	buttonTriangleName,
	buttonCircleName,
	buttonCrossName,
	buttonSquareName
};

/** \brief Convert a button on the PSX controller to a small integer
 * 
 * Similar to #button2position() but slower as this function is iterative. Might
 * do some measurements and unify them later on.
 * 
 * Output will always be in the range [0, #PSX_BUTTONS_NO - 1] and is not
 * guaranteed to be valid, so it should be checked to be < PSX_BUTTONS_NO before
 * use.
 */
byte psxButtonToIndex (Buttons psxButtons) {
	byte i;

	for (i = 0; i < PSX_BUTTONS_NO; ++i) {
		if (psxButtons & 0x01) {
			break;
		}

		psxButtons >>= 1;
	}

	return i;
}

#ifdef ENABLE_SERIAL_DEBUG
FlashStr getButtonName (Buttons psxButton) {
	FlashStr ret = F("");
	
	byte b = psxButtonToIndex (psxButton);
	if (b < PSX_BUTTONS_NO) {
		PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P> (pgm_read_ptr (&(psxButtonNames[b])));
		ret = PSTR_TO_F (bName);
	}

	return ret;
}
#endif

void dumpButtons (Buttons psxButtons) {
#ifdef DEBUG_PAD
	static Buttons lastB = 0;

	if (psxButtons != lastB) {
		lastB = psxButtons;			// Save it before we alter it
		
		debug (F("Pressed: "));

		for (byte i = 0; i < PSX_BUTTONS_NO; ++i) {
			byte b = psxButtonToIndex (psxButtons);
			if (b < PSX_BUTTONS_NO) {
				PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P> (pgm_read_ptr (&(psxButtonNames[b])));
				debug (PSTR_TO_F (bName));
			}

			psxButtons &= ~(1 << b);

			if (psxButtons != 0) {
				debug (F(", "));
			}
		}

		debugln ();
	}
#else
	(void) psxButtons;
#endif
}

// ISR
void onPadModeChange () {
	if (digitalRead (PIN_PADMODE) == LOW) {
		if (state != ST_CD32) {
			// Switch to CD32 mode
			toCD32 ();
		}
		
		// Sample input values, they will be shifted out on subsequent clock inputs
		//~ buttons = buttonsLive | 0x80;   // Make sure bit MSB is 1 for ID sequence
		buttons = buttonsLive;		/* The above is now handled elsewhere so that
									 * here we can run as fast as possible
									 */

		// Output first bit immediately
		digitalWrite (PIN_BTNREGOUT, buttons & 0x01);
		buttons >>= 1;	/* MSB will be zeroed during shifting, this will report
						 * non-existing button 9 as pressed for the ID sequence
						 */
	} else {
		/* Mark time pin went high so that we can see if it goes back high in a
		 * short while
		 */
		lastSwitchedTime = millis ();
	}
}

// ISR
void onClockEdge () {
	digitalWrite (PIN_BTNREGOUT, buttons & 0x01);
	buttons >>= 1;	/* Again, non-existing button 10 will be reported as pressed
	                 * for the ID sequence
	                 */
}

/** \brief Enable CD32 controller support
 * 
 * CD32 mode is entered automatically whenever a HIGH level is detected on
 * #PIN_PADMODE, after this function has been called.
 */
inline void enableCD32Trigger () {
	/* Avoid any pending interrupts, see
	 * https://github.com/arduino/ArduinoCore-avr/issues/244
	 */
	EIFR |= (1 << INTF1) | (1 << INTF0);

	// Call ISR on changes of the CD32 pad mode pin
	attachInterrupt (digitalPinToInterrupt (PIN_PADMODE), onPadModeChange, CHANGE);
}

/** \brief Disable CD32 controller support
 * 
 * CD32 mode will no longer be entered automatically, after this function has
 * been called.
 */
inline void disbleCD32Trigger () {
	detachInterrupt (digitalPinToInterrupt (PIN_PADMODE));
}

/** \brief Clear controller configurations
 * 
 * All controller configuration will be reset so that:
 * - Square is button 1
 * - Cross is button 2
 * - All other buttons are inactive
 * 
 * The programming function can then be used to map any button as desired.
 */
void clearConfigurations () {
	debugln (F("Clearing controllerConfigs"));
	//~ memset (controllerConfigs, 0x00, sizeof (controllerConfigs));
	for (byte i = 0; i < PSX_BUTTONS_NO; ++i) {
		ControllerConfiguration& config = controllerConfigs[i];
		memset (&config, 0x00, sizeof (TwoButtonJoystick));
		config.buttonMappings[button2position (PSB_SQUARE)].b1 = true;
		config.buttonMappings[button2position (PSB_CROSS)].b2 = true;
	}
}

uint16_t calculateConfigCrc () {
	uint16_t crc = 0x4242;
	uint8_t *data = (uint8_t *) controllerConfigs;
	for (word i = 0; i < sizeof (controllerConfigs); ++i) {
		crc = _crc16_update (crc, data[i]);
	}
	
	return crc;
}

/** \brief Load controller configurations from EEPROM
 * 
 * If the loaded configurations are not valid, they are cleared.
 * 
 * \return True if the loaded configurations are valid
 */
boolean loadConfigurations () {
	boolean ret = false;
	
	debug (F("Size of controllerConfigs is "));
	debugln (sizeof (controllerConfigs));
	
	EEPROM.get (4, controllerConfigs);
	
	// Validation
	uint16_t goodCrc;
	EEPROM.get (2, goodCrc);
	uint16_t crc = calculateConfigCrc ();
	if (crc == goodCrc) {
		debugln (F("CRCs match"));
		ret = true;
	} else {
		debugln (F("CRCs do not match"));
		clearConfigurations ();
	}
	
	return ret;
}

//! \brief Save controller configurations to EEPROM
void saveConfigurations () {
	debugln (F("Saving controllerConfigs"));
	
	EEPROM.put (4, controllerConfigs);
	
	// CRC
	uint16_t crc = calculateConfigCrc ();
	EEPROM.put (2, crc);
}

void setup () {
	dstart (115200);
	debugln (F("Starting up..."));

	pinMode (PIN_PADMODE, INPUT_PULLUP);
	//~ pinMode (PIN_BTNREGOUT, OUTPUT);
	//~ pinMode (PIN_BTNREGCLK, INPUT);

	// Prepare leds
	pinMode (PIN_LED_PAD_OK, OUTPUT);
	pinMode (PIN_LED_MODE, OUTPUT);

	// Give wireless PS2 module some time to startup, before configuring it
	delay (300);

	// This will also initialize the configurations if EEPROM is invalid
	loadConfigurations ();

	enableCD32Trigger ();
	
	// Start polling for controller
	state = ST_NO_CONTROLLER;
}

void toMouse () {
	debugln (F("To mouse mode"));
	
	// Direction pins must be outputs
	pinMode (PIN_UP, OUTPUT);
	pinMode (PIN_DOWN, OUTPUT);
	pinMode (PIN_LEFT, OUTPUT);
	pinMode (PIN_RIGHT, OUTPUT);

	// We're not going to care for clock pulses anymore
	detachInterrupt (digitalPinToInterrupt (PIN_BTNREGCLK));
	
	state = ST_MOUSE;
}

void toJoystick () {
	debugln (F("To joystick mode"));
	
	// All pins to Hi-Z without pull-up
	digitalWrite (PIN_UP, LOW);
	pinMode (PIN_UP, INPUT);
	digitalWrite (PIN_DOWN, LOW);
	pinMode (PIN_DOWN, INPUT);
	digitalWrite (PIN_LEFT, LOW);
	pinMode (PIN_LEFT, INPUT);
	digitalWrite (PIN_RIGHT, LOW);
	pinMode (PIN_RIGHT, INPUT);
	
	detachInterrupt (digitalPinToInterrupt (PIN_BTNREGCLK));

	state = ST_JOYSTICK;
}

void toCD32 () {
	debugln (F("To CD32 mode"));
		
	// Pin 9 becomes our data output
	pinMode (PIN_BTNREGOUT, OUTPUT);
	
	// Pin 6 becomes an input for clock
	pinMode (PIN_BTNREGCLK, INPUT);
	attachInterrupt (digitalPinToInterrupt (PIN_BTNREGCLK), onClockEdge, RISING);

	state = ST_CD32;
}

inline void buttonPress (byte pin) {
	/* Drive pins in open-collector style, so that we are compatible with the
	 * C64 too
	 */
	pinMode (pin, OUTPUT);  // Low is implicit
}

inline void buttonRelease (byte pin) {
	pinMode (pin, INPUT); // Hi-Z
}

void mapAnalogStickHorizontal (TwoButtonJoystick& j) {
	byte lx = ps2x.Analog (PSS_LX);   			// 0 ... 255
	int8_t deltaLX = lx - ANALOG_IDLE_VALUE;		// --> -127 ... +128
	j.left = deltaLX < -ANALOG_DEAD_ZONE;
	j.right = deltaLX > +ANALOG_DEAD_ZONE;

#ifdef ENABLE_SERIAL_DEBUG
	static int oldx = -1000;
	if (deltaLX != oldx) {
		debug (F("L Analog X = "));
		debugln (deltaLX);
		oldx = deltaLX;
	}
#endif
}

void mapAnalogStickVertical (TwoButtonJoystick& j) {
	byte ly = ps2x.Analog (PSS_LY);
	int8_t deltaLY = ly - ANALOG_IDLE_VALUE;
	j.up = deltaLY < -ANALOG_DEAD_ZONE;
	j.down = deltaLY > +ANALOG_DEAD_ZONE;

#ifdef ENABLE_SERIAL_DEBUG
	static int oldy = -1000;
	if (deltaLY != oldy) {
		debug (F("L Analog Y = "));
		debugln (deltaLY);
		oldy = deltaLY;
	}
#endif
}

void mapJoystickNormal (TwoButtonJoystick& j) {
	// Use both analog axes
	mapAnalogStickHorizontal (j);
	mapAnalogStickVertical (j);

	// D-Pad is fully functional as well
	j.up |= ps2x.Button (PSB_PAD_UP);
	j.down |= ps2x.Button (PSB_PAD_DOWN);
	j.left |= ps2x.Button (PSB_PAD_LEFT);
	j.right |= ps2x.Button (PSB_PAD_RIGHT);

	// Square/Rx are button 1
	j.b1 = ps2x.Button (PSB_SQUARE) || ps2x.Button (PSB_R1) || ps2x.Button (PSB_R2) || ps2x.Button (PSB_R3);

	// Cross/Lx are button 1
	j.b2 = ps2x.Button (PSB_CROSS) || ps2x.Button (PSB_L1) || ps2x.Button (PSB_L2) || ps2x.Button (PSB_L3);
}

void mapJoystickRacing1 (TwoButtonJoystick& j) {
	// Use analog's horizontal axis to steer, ignore vertical
	mapAnalogStickHorizontal (j);

	// D-Pad L/R can also be used
	j.left |= ps2x.Button (PSB_PAD_LEFT);
	j.right |= ps2x.Button (PSB_PAD_RIGHT);

	// Use D-Pad U/Square to accelerate and D/Cross to brake
	j.up = ps2x.Button (PSB_PAD_UP) || ps2x.Button (PSB_SQUARE);
	j.down = ps2x.Button (PSB_PAD_DOWN) || ps2x.Button (PSB_CROSS);
	
	/* Games probably did not expect up + down at the same time, so when
	 * braking, don't accelerate
	 */
	if (j.down)
		j.up = false;

	// Triangle/Rx are button 1
	j.b1 = ps2x.Button (PSB_TRIANGLE) || ps2x.Button (PSB_R1) || ps2x.Button (PSB_R2) || ps2x.Button (PSB_R3);

	// Circle/Lx are button 2
	j.b2 = ps2x.Button (PSB_CIRCLE) || ps2x.Button (PSB_L1) || ps2x.Button (PSB_L2) || ps2x.Button (PSB_L3);
}

void mapJoystickRacing2 (TwoButtonJoystick& j) {
	// Use analog's horizontal axis to steer, ignore vertical
	mapAnalogStickHorizontal (j);

	// D-Pad L/R can also be used
	j.left |= ps2x.Button (PSB_PAD_LEFT);
	j.right |= ps2x.Button (PSB_PAD_RIGHT);

	// Use D-Pad U/R1/R2 to accelerate and D/Cross/L1/L2 to brake
	j.up = ps2x.Button (PSB_PAD_UP) || ps2x.Button (PSB_R1) || ps2x.Button (PSB_R2);
	j.down = ps2x.Button (PSB_PAD_DOWN) || ps2x.Button (PSB_CROSS) || ps2x.Button (PSB_L1) || ps2x.Button (PSB_L2);

	/* Games probably did not expect up + down at the same time, so when
	 * braking, don't accelerate
	 */
	if (j.down)
		j.up = false;

	// Square/R3 are button 1
	j.b1 = ps2x.Button (PSB_SQUARE) || ps2x.Button (PSB_R3);

	// Triangle/L3 are button 2
	j.b2 = ps2x.Button (PSB_TRIANGLE) || ps2x.Button (PSB_L3);
}

void mapJoystickPlatform (TwoButtonJoystick& j) {
	// Use horizontal analog axis fully, but only down on vertical
	mapAnalogStickHorizontal (j);
	mapAnalogStickVertical (j);

	// D-Pad is fully functional
	j.up = ps2x.Button (PSB_PAD_UP);		// Note the '=', will override analog UP
	j.down |= ps2x.Button (PSB_PAD_DOWN);
	j.left |= ps2x.Button (PSB_PAD_LEFT);
	j.right |= ps2x.Button (PSB_PAD_RIGHT);

	// Cross is up/jump
	j.up |= ps2x.Button (PSB_CROSS);

	// Square/Rx are button 1
	j.b1 = ps2x.Button (PSB_SQUARE) || ps2x.Button (PSB_R1) || ps2x.Button (PSB_R2) || ps2x.Button (PSB_R3);

	// Triangle/Lx are button 2
	j.b2 = ps2x.Button (PSB_TRIANGLE) || ps2x.Button (PSB_L1) || ps2x.Button (PSB_L2) || ps2x.Button (PSB_L3);
}

/** \brief Map PSX controller buttons to two-button joystick according to Custom
 *         mapping 1
 */
void mapJoystickCustom (TwoButtonJoystick& j) {
	// Use horizontal analog axis fully, but only down on vertical
	mapAnalogStickHorizontal (j);
	mapAnalogStickVertical (j);

	// D-Pad is fully functional
	j.up |= ps2x.Button (PSB_PAD_UP);
	j.down |= ps2x.Button (PSB_PAD_DOWN);
	j.left |= ps2x.Button (PSB_PAD_LEFT);
	j.right |= ps2x.Button (PSB_PAD_RIGHT);

	for (byte i = 0; i < PSX_BUTTONS_NO; ++i) {
		Buttons button = 1 << i;
		if (isButtonMappable (button) && ps2x.Button (button)) {
			byte buttonIdx = button2position (button);
			mergeButtons (j, currentCustomConfig -> buttonMappings[buttonIdx]);
		}
	}
}

/** \brief Merge two #TwoButtonJoystick's
 * 
 * Every button that is pressed in either \a src or \a dest will end up pressed
 * in \a dest.
 * 
 * \param[inout] dest Destination
 * \param[in] src Source
 */
void mergeButtons (TwoButtonJoystick& dest, const TwoButtonJoystick& src) {
	dest.up |= src.up;
	dest.down |= src.down;
	dest.left |= src.left;
	dest.right |= src.right;
	dest.b1 |= src.b1;
	dest.b2 |= src.b2;
}

void flashLed (byte n) {
	for (byte i = 0; i < n; ++i) {
		digitalWrite (PIN_LED_MODE, HIGH);
		delay (40);
		digitalWrite (PIN_LED_MODE, LOW);
		delay (80);
	}
}

/** \brief Check if the right analog stick has been moved
 * 
 * The stick is not considered moved if it moves less than #ANALOG_DEAD_ZONE.
 * 
 * \param[out] x Movement on the horizontal axis [-127 ... 127]
 * \param[out] y Movement on the vertical axis [-127 ... 127]
 * \return True if the stick is not in the center position, false otherwise
 */
boolean rightAnalogMoved (int8_t& x, int8_t& y) {
	boolean ret = false;
	
	uint8_t rx = ps2x.Analog (PSS_RX);   		// [0 ... 255]
	int8_t deltaRX = rx - ANALOG_IDLE_VALUE;	// [-128 ... 127]
	uint8_t deltaRXabs = abs (deltaRX);
	if (deltaRXabs > ANALOG_DEAD_ZONE) {
		x = deltaRX;
		if (x == -128)
			x = -127;
		ret = true;
	} else {
		x = 0;
	}
	
	uint8_t ry = ps2x.Analog (PSS_RY);
	int8_t deltaRY = ry - ANALOG_IDLE_VALUE;
	uint8_t deltaRYabs = abs (deltaRY);
	if (deltaRYabs > ANALOG_DEAD_ZONE) {
		y = deltaRY;
		if (y == -128)
			y = -127;
		ret = true;
	} else {
		y = 0;
	}
	
#ifdef ENABLE_SERIAL_DEBUG
	if (ret) {
		static int oldx = -1000;
		if (x != oldx) {
			debug (F("R Analog X = "));
			debugln (x);
			oldx = x;
		}

		static int oldy = -1000;
		if (y != oldy) {
			debug (F("R Analog Y = "));
			debugln (y);
			oldy = y;
		}
	}
#endif
	
	return ret;
}

void handleJoystick () {
	// Call button mapping function
	TwoButtonJoystick j = {false, false, false, false, false, false};
	//~ if (!joyMappingFunc)
		//~ joyMappingFunc = mapJoystickNormal;			
	joyMappingFunc (j);

#ifdef ENABLE_SERIAL_DEBUG
	static TwoButtonJoystick oldJoy = {false, false, false, false, false, false};

	if (memcmp (&j, &oldJoy, sizeof (TwoButtonJoystick)) != 0) {
		debug (F("Sending to DB-9: "));
		dumpJoy (j);
		oldJoy = j;
	}
#endif

	// Make mapped buttons affect the actual pins
	if (j.up) {
		buttonPress (PIN_UP);
	} else {
		buttonRelease (PIN_UP);
	}

	if (j.down) {
		buttonPress (PIN_DOWN);
	} else {
		buttonRelease (PIN_DOWN);
	}

	if (j.left) {
		buttonPress (PIN_LEFT);
	} else {
		buttonRelease (PIN_LEFT);
	}

	if (j.right) {
		buttonPress (PIN_RIGHT);
	} else {
		buttonRelease (PIN_RIGHT);
	}

	/* If the interrupt that switches us to CD32 mode is
	 * triggered while we are here we might end up setting pin states after
	 * we should have relinquished control of the pins, so let's avoid this
	 * disabling interrupts, we will handle them in a few microseconds.
	 */
	noInterrupts ();

	if (j.b1) {
		buttonPress (PIN_BTN1);
	} else {
		buttonRelease (PIN_BTN1);
	}

	if (j.b2) {
		buttonPress (PIN_BTN2);
	} else {
		buttonRelease (PIN_BTN2);
	}

	interrupts ();
}

void handleMouse () {
	static unsigned long tx = 0, ty = 0;
	
	int8_t x, y;
	rightAnalogMoved (x, y);
	
	// Horizontal axis
	if (x != 0) {
		debug (F("x = "));
		debug (x);

		// Assume Right
		byte leadingPin = PIN_RIGHT;
		byte trailingPin = PIN_DOWN;
		if (x < 0) {
			// No, it was Left
			leadingPin = PIN_DOWN;
			trailingPin = PIN_RIGHT;

			x = -x;
		}

		unsigned int period = map (x, ANALOG_DEAD_ZONE, 127, MOUSE_SLOW_DELTA, MOUSE_FAST_DELTA);
		debug (F(" --> period = "));
		debugln (period);
		
		if (millis () - tx >= period) {
			digitalWrite (leadingPin, !digitalRead (leadingPin));
			tx = millis ();
		}
		
		if (millis () - tx >= period / 2) {
			digitalWrite (trailingPin, !digitalRead (leadingPin));
		}
	}

	// Vertical axis
	if (y != 0) {
		debug (F("y = "));
		debug (y);

		// Assume Down
		byte leadingPin = PIN_LEFT;
		byte trailingPin = PIN_UP;
		if (y < 0) {
			// No, it was Up
			leadingPin = PIN_UP;
			trailingPin = PIN_LEFT;

			y = -y;
		}

		unsigned int period = map (y, ANALOG_DEAD_ZONE, 127, MOUSE_SLOW_DELTA, MOUSE_FAST_DELTA);
		debug (F(" --> period = "));
		debugln (period);
		
		if (millis () - ty >= period) {
			digitalWrite (leadingPin, !digitalRead (leadingPin));
			ty = millis ();
		}
		
		if (millis () - ty >= period / 2) {
			digitalWrite (trailingPin, !digitalRead (leadingPin));
		}
	}

	// Buttons
	noInterrupts ();
	
	if (ps2x.Button (PSB_L1) || ps2x.Button (PSB_L2) || ps2x.Button (PSB_L3)) {
		buttonPress (PIN_BTN1);
	} else {
		buttonRelease (PIN_BTN1);
	}

	if (ps2x.Button (PSB_R1) || ps2x.Button (PSB_R2) || ps2x.Button (PSB_R3)) {
		buttonPress (PIN_BTN2);
	} else {
		buttonRelease (PIN_BTN2);
	}

	interrupts ();
}

void handleCD32Pad () {
	// Directions still behave as in normal joystick mode, so abuse those functions
	TwoButtonJoystick analog;
	mapAnalogStickHorizontal (analog);
	mapAnalogStickVertical (analog);

	// D-Pad is fully functional as well, keep it in mind
	if (analog.up || ps2x.Button (PSB_PAD_UP)) {
		buttonPress (PIN_UP);
	} else {
		buttonRelease (PIN_UP);
	}

	if (analog.down || ps2x.Button (PSB_PAD_DOWN)) {
		buttonPress (PIN_DOWN);
	} else {
		buttonRelease (PIN_DOWN);
	}

	if (analog.left || ps2x.Button (PSB_PAD_LEFT)) {
		buttonPress (PIN_LEFT);
	} else {
		buttonRelease (PIN_LEFT);
	}

	if (analog.right || ps2x.Button (PSB_PAD_RIGHT)) {
		buttonPress (PIN_RIGHT);
	} else {
		buttonRelease (PIN_RIGHT);
	}

	/* Map buttons, working on a temporary variable to avoid the sampling
	 * interrupt to happen while we are filling in button statuses and catch a
	 * value that has not yet been fully populated.
	 *
	 * Note that 0 means pressed and that MSB must be 1 for the ID
	 * sequence.
	 */
	byte buttonsTmp = 0xFF;

	if (ps2x.Button (PSB_START))
		buttonsTmp &= ~BTN_START;

	if (ps2x.Button (PSB_TRIANGLE))
		buttonsTmp &= ~BTN_GREEN;

	if (ps2x.Button (PSB_SQUARE))
		buttonsTmp &= ~BTN_RED;

	if (ps2x.Button (PSB_CROSS))
		buttonsTmp &= ~BTN_BLUE;

	if (ps2x.Button (PSB_CIRCLE))
		buttonsTmp &= ~BTN_YELLOW;

	if (ps2x.Button (PSB_L1) || ps2x.Button (PSB_L2) || ps2x.Button (PSB_L3))
		buttonsTmp &= ~BTN_FRONT_L;

	if (ps2x.Button (PSB_R1) || ps2x.Button (PSB_R2) || ps2x.Button (PSB_R3))
		buttonsTmp &= ~BTN_FRONT_R;

	// Atomic operation, interrupt either happens before or after this
	buttonsLive = buttonsTmp;
}

/** \brief Debounce button/combo presses
 * 
 * Makes sure that the same button/combo has been pressed steadily for some
 * time.
 * 
 * \sa DEBOUNCE_TIME_BUTTON
 * \sa DEBOUNCE_TIME_COMBO
 * 
 * \param[in] holdTime Time the button/combo must be stable for
 */
Buttons debounceButtons (unsigned long holdTime) {
	static Buttons oldButtons = NO_BUTTON;
	static unsigned long pressedOn = 0;

	Buttons ret = NO_BUTTON;

	Buttons buttons = (Buttons) ps2x.ButtonDataByte ();
	if (buttons == oldButtons) {
		if (millis () - pressedOn > holdTime) {
			// Same combo held long enough
			ret = buttons;
		} else {
			// Combo held not long enough (yet)
		}
	} else {
		// Buttons bouncing
		oldButtons = buttons;
		pressedOn = millis ();
	}

	return ret;
}

/** \brief Translate a button combo to what would be sent to the DB-9 port
 * 
 * This is used during programming mode to enter the combo that should be sent
 * whenever a mapped button is pressed.
 * 
 * \param[in] psxButtons PSX controller button combo
 * \param[out] j Two-button joystick configuration corresponding to input
 * \return True if the output contains at least one pressed button
 */
boolean psxButton2Amiga (Buttons psxButtons, TwoButtonJoystick& j) {
	memset (&j, 0x00, sizeof (j));
	
	j.up = ps2x.Button (psxButtons, PSB_PAD_UP);
	j.down = ps2x.Button (psxButtons, PSB_PAD_DOWN);
	j.left = ps2x.Button (psxButtons, PSB_PAD_LEFT);
	j.right = ps2x.Button (psxButtons, PSB_PAD_RIGHT);
	j.b1 = ps2x.Button (psxButtons, PSB_SQUARE);
	j.b2 = ps2x.Button (psxButtons, PSB_CROSS);

	return *reinterpret_cast<byte *> (&j);
}

void dumpJoy (TwoButtonJoystick& j) {
	if (j.up) {
		debug (F("Up "));
	}
	if (j.down) {
		debug (F("Down "));
	}
	if (j.left) {
		debug (F("Left "));
	}
	if (j.right) {
		debug (F("Right "));
	}
	if (j.b1) {
		debug (F("B1 "));
	}
	if (j.b2) {
		debug (F("B2"));
	}
	debugln (F(""));
}

/** \brief Get number of set bits in binary representation of passed number
 * 
 * All hail to Brian Kernighan.
 */
unsigned int countSetBits (int n) { 
	unsigned int count = 0; 

	while (n) { 
		n &= n - 1;
		++count; 
	} 

	return count; 
}

/** \brief Check whether a button report contains a mappable button
 * 
 * That means it contains a single button which is not \a SELECT neither one
 * from the D-Pad.
 */
boolean isButtonMappable (Buttons b) {
	return countSetBits (b) == 1 &&
	       !ps2x.Button (b, PSB_SELECT) &&
	       !ps2x.Button (b, PSB_PAD_UP) &&
	       !ps2x.Button (b, PSB_PAD_DOWN) &&
	       !ps2x.Button (b, PSB_PAD_LEFT) &&
	       !ps2x.Button (b, PSB_PAD_RIGHT);
}

boolean isButtonProgrammable (Buttons b) {
	return ps2x.Button (b, PSB_L1) || ps2x.Button (b, PSB_L2) ||
	       ps2x.Button (b, PSB_R1) || ps2x.Button (b, PSB_R2);
}

/** \brief Convert a mappable button to a small integer
 * 
 * A mappable button is converted to an integer in the
 * [0 - #MAPPABLE_BUTTONS_NO - 1) range as detailed in the table below.
 * 
 * This allows to quickly look up a button in an array and is used both to map a
 * button press to a programmed combo and to map a button to its corresponding
 * #ControllerConfiguration.
 * 
 * <pre>
 *                                      >> 2		>> 9		>> 13
 * #define PSB_L3          0x0002       0
 * #define PSB_R3          0x0004       1
 * #define PSB_START       0x0008       2
 * #define PSB_L2          0x0100					0
 * #define PSB_R2          0x0200					1
 * #define PSB_L1          0x0400					2
 * #define PSB_R1          0x0800					4
 * #define PSB_TRIANGLE    0x1000								0
 * #define PSB_CIRCLE      0x2000								1
 * #define PSB_CROSS       0x4000								2
 * #define PSB_SQUARE      0x8000								4
 * </pre>
 *
 * Positions 6 and 11 are unused, but this should be fast
 * 
 * \sa mapJoystickCustom1()
 */ 
byte button2position (Buttons b) {
	byte pos = 0xFF;
	
	if (b < 0x100) {
		pos = b >> 2;			// 0, 1, 2
	} else if (b < 0x1000) {
		pos = (b >> 9) + 3;		// 0, 1, 2, 4 + 3 = 3, 4, 5, 7
	} else {
		pos = (b >> 13) + 8;	// 0, 1, 2, 4 + 8 = 8, 9, 10, 12
	}
	
	return pos;
}

void stateMachine () {
	static Buttons selectComboButton = NO_BUTTON;
	static Buttons programmedButton = NO_BUTTON;
	Buttons buttons = NO_BUTTON;
	static unsigned long stateEnteredTime = 0;

	/* This is done first since ALL states except NO_CONTROLLER need to poll the
	 * controller first and switch to NO_CONTROLLER if polling failed
	 */
	if (state != ST_NO_CONTROLLER) {
		// We have a controller and we can poll it
		if (ps2x.read_gamepad ()) {
			dumpButtons (ps2x.ButtonDataByte ());
		} else {
			// Polling failed
			debugln (F("Controller lost"));
			state = ST_NO_CONTROLLER;
			buttonsLive = 0x7F;		// No ID sequence, all buttons released
		}
	}

	switch (state) {
		case ST_NO_CONTROLLER:
			/* There's no need to only poll every so often, the function is slow
			 * and will basically time itself just fine
			 */
			if (initPad ()) {
				// Got a controller
				state = ST_FIRST_READ;
			}
			break;
		case ST_FIRST_READ:
			if (ps2x.Button (PSB_SELECT)) {
#ifdef ENABLE_FACTORY_RESET
				/* The controller was plugged in (or the adapter was powered on)
				 * with SELECT held, so the user wants to do a factory reset
				 */
				debugln (F("SELECT pressed at power-up, starting factory reset"));
				state = ST_FACTORY_RESET_WAIT_1;
#endif
			} else {
				// Default to joystick mode
				toJoystick ();
			}
			break;
				
		/**********************************************************************
		 * MAIN MODES
		 **********************************************************************/
		case ST_JOYSTICK: {
			int8_t x, y;
			
			if (rightAnalogMoved (x, y)) {
				// Right analog stick moved, switch to Mouse mode
				toMouse ();
			} else if (ps2x.Button (PSB_SELECT)) {
				state = ST_SELECT_HELD;
			} else {
				// Handle normal joystick movements
				handleJoystick ();
			}
			break;
		} case ST_MOUSE:
			if (ps2x.Button (PSB_PAD_UP) || ps2x.Button (PSB_PAD_DOWN) ||
				ps2x.Button (PSB_PAD_LEFT) || ps2x.Button (PSB_PAD_RIGHT)) {
				// D-Pad pressed, go back to joystick mode
				toJoystick ();
			} else {
				handleMouse ();
			}
			break;
		case ST_CD32:
			handleCD32Pad ();
			break;
				
		/**********************************************************************
		 * SELECT MAPPING/SWITCH TO PROGRAMMING MODE
		 **********************************************************************/
		case ST_SELECT_HELD:
			if (!ps2x.Button (PSB_SELECT)) {
				// Select was released
				state = ST_JOYSTICK;
			} else if (ps2x.Button (PSB_SQUARE)) {
				selectComboButton = PSB_SQUARE;
				state = ST_SELECT_AND_BTN_HELD;
			} else if (ps2x.Button (PSB_TRIANGLE)) {
				selectComboButton = PSB_TRIANGLE;
				state = ST_SELECT_AND_BTN_HELD;
			} else if (ps2x.Button (PSB_CIRCLE)) {
				selectComboButton = PSB_CIRCLE;
				state = ST_SELECT_AND_BTN_HELD;
			} else if (ps2x.Button (PSB_CROSS)) {
				selectComboButton = PSB_CROSS;
				state = ST_SELECT_AND_BTN_HELD;
			} else if (ps2x.Button (PSB_L1)) {
				selectComboButton = PSB_L1;
				state = ST_SELECT_AND_BTN_HELD;
			} else if (ps2x.Button (PSB_R1)) {
				selectComboButton = PSB_R1;
				state = ST_SELECT_AND_BTN_HELD;
			} else if (ps2x.Button (PSB_L2)) {
				selectComboButton = PSB_L2;
				state = ST_SELECT_AND_BTN_HELD;
			} else if (ps2x.Button (PSB_R2)) {
				selectComboButton = PSB_R2;
				state = ST_SELECT_AND_BTN_HELD;
			}
			break;
		case ST_SELECT_AND_BTN_HELD:
			if (stateEnteredTime == 0) {
				// State was just entered
				stateEnteredTime = millis ();
			} else if (isButtonProgrammable (selectComboButton) && millis () - stateEnteredTime > 1000) {
				// Combo kept pressed, enter programming mode
				debug (F("Entering programming mode for "));
				debugln (getButtonName (selectComboButton));
				stateEnteredTime = 0;
				state = ST_WAIT_SELECT_RELEASE;
			} else if (!ps2x.Button (PSB_SELECT) || !ps2x.Button (selectComboButton)) {
				// Combo released, switch to desired mapping
				stateEnteredTime = 0;
				state = ST_ENABLE_MAPPING;
			}
			break;
		case ST_ENABLE_MAPPING:
			// Change button mapping
			switch (selectComboButton) {
				case PSB_SQUARE:
					debugln (F("Setting normal mapping"));
					joyMappingFunc = mapJoystickNormal;
					flashLed (JMAP_NORMAL);
					break;
				case PSB_TRIANGLE:
					debugln (F("Setting Racing1 mapping"));
					joyMappingFunc = mapJoystickRacing1;
					flashLed (JMAP_RACING1);
					break;
				case PSB_CIRCLE:
					debugln (F("Setting Racing2 mapping"));
					joyMappingFunc = mapJoystickRacing2;
					flashLed (JMAP_RACING2);
					break;
				case PSB_CROSS:
					debugln (F("Setting Platform mapping"));
					joyMappingFunc = mapJoystickPlatform;
					flashLed (JMAP_PLATFORM);
					break;
				case PSB_L1:
				case PSB_R1:
				case PSB_L2:
				case PSB_R2: {
					byte configIdx = psxButtonToIndex (selectComboButton);
					if (configIdx < PSX_BUTTONS_NO) {
						debug (F("Setting Custom mapping for controllerConfig "));
						debugln (configIdx);
						currentCustomConfig = &controllerConfigs[configIdx];
						joyMappingFunc = mapJoystickCustom;
						flashLed (JMAP_CUSTOM);
					} else {
						// Something went wrong, just pretend it never happened
						state = ST_JOYSTICK;
					}
					break;
				} default:
					// Shouldn't be reached
					break;
			}
			state = ST_JOYSTICK;		// Exit immediately
			break;
		
		/**********************************************************************
		 * PROGRAMMING
		 **********************************************************************/
		case ST_WAIT_SELECT_RELEASE:
			if (!ps2x.Button (PSB_SELECT)) {
				state = ST_WAIT_BUTTON_PRESS;
			}
			break;
		case ST_WAIT_BUTTON_PRESS:
			if (ps2x.Button (PSB_SELECT)) {
				// Exit programming mode
				debugln (F("Leaving programming mode"));
				saveConfigurations ();	// No need to check for changes as this uses EEPROM.update()
				state = ST_WAIT_SELECT_RELEASE_FOR_EXIT;
			} else {
				buttons = debounceButtons (DEBOUNCE_TIME_BUTTON);
				if (isButtonMappable (buttons)) {
					// Exactly one key pressed, go on
					programmedButton = buttons;
					debug (F("Programming button "));
					debugln (getButtonName (buttons));
					flashLed (3);
					state = ST_WAIT_BUTTON_RELEASE;
				}
			}
			break;
		case ST_WAIT_BUTTON_RELEASE:
			buttons = debounceButtons (DEBOUNCE_TIME_BUTTON);
			if (buttons == NO_BUTTON) {
				state = ST_WAIT_COMBO_PRESS;
			}
			break;
		case ST_WAIT_COMBO_PRESS:
			buttons = debounceButtons (DEBOUNCE_TIME_COMBO);
			TwoButtonJoystick j;
			if (buttons != NO_BUTTON && psxButton2Amiga (buttons, j)) {
				debug (F("Programmed to "));
				dumpJoy (j);

				// First look up the config the mapping shall be saved to
				byte configIdx = psxButtonToIndex (selectComboButton);
				if (configIdx < PSX_BUTTONS_NO) {
					debug (F("Storing to controllerConfig "));
					debugln (configIdx);
					
					ControllerConfiguration *config = &controllerConfigs[configIdx];

					// Then look up the mapping according to the programmed button
					byte buttonIdx = button2position (programmedButton);
					config -> buttonMappings[buttonIdx] = j;
				}
				
				programmedButton = NO_BUTTON;
				flashLed (5);
				state = ST_WAIT_COMBO_RELEASE;
			}
			break;
		case ST_WAIT_COMBO_RELEASE:
			buttons = debounceButtons (DEBOUNCE_TIME_BUTTON);
			if (buttons == NO_BUTTON) {
				state = ST_WAIT_BUTTON_PRESS;
			}
			break;
		case ST_WAIT_SELECT_RELEASE_FOR_EXIT:
			if (!ps2x.Button (PSB_SELECT)) {
				state = ST_JOYSTICK;
			}
			break;
			
		/**********************************************************************
		 * FACTORY_RESET
		 **********************************************************************/
#ifdef ENABLE_FACTORY_RESET
		case ST_FACTORY_RESET_WAIT_1:
			if (stateEnteredTime == 0) {
				stateEnteredTime = millis ();
			} else if (millis () - stateEnteredTime > 2000) {
				stateEnteredTime = 0;
				state = ST_FACTORY_RESET_WAIT_2;
			} else if (!ps2x.Button (PSB_SELECT)) {
				stateEnteredTime = 0;
				state = ST_JOYSTICK;
			}
			break;
		case ST_FACTORY_RESET_WAIT_2:
			if (stateEnteredTime == 0) {
				stateEnteredTime = millis ();
			} else if (millis () - stateEnteredTime > 2000) {
				stateEnteredTime = 0;
				state = ST_FACTORY_RESET_PERFORM;
			} else if (!ps2x.Button (PSB_SELECT)) {
				stateEnteredTime = 0;
				state = ST_JOYSTICK;
			}
			break;
		case ST_FACTORY_RESET_PERFORM:
			// OK, user has convinced us to actually perform the reset
			//~ for (byte i = 0; i < 2; ++i) {
				//~ digitalWrite (PIN_LED_MODE, HIGH);
				//~ delay (500);
				//~ digitalWrite (PIN_LED_MODE, LOW);
				//~ delay (500);
			//~ }
			//~ digitalWrite (PIN_LED_MODE, HIGH);
			//~ delay (2000);
			//~ digitalWrite (PIN_LED_MODE, LOW);
			clearConfigurations ();
			saveConfigurations ();
			state = ST_JOYSTICK;
			break;
#endif
	}
}

/** \brief Update leds
 *
 * We have a separate function for this as several machine states share the same led state.
 */
void updateLeds () {
	// Pad OK led
	if (state == ST_NO_CONTROLLER) {
		// Blink
		digitalWrite (PIN_LED_PAD_OK, (millis () / 500) % 2 == 0);
	} else {
		// Steadily lit
		digitalWrite (PIN_LED_PAD_OK, HIGH);
	}

	// Mode led
	switch (state) {
		case ST_NO_CONTROLLER:
		case ST_FIRST_READ:
#ifdef ENABLE_FACTORY_RESET
		case ST_FACTORY_RESET_PERFORM:	// Led for this state is handled in SM
#endif
			digitalWrite (PIN_LED_MODE, LOW);
			break;		
		case ST_JOYSTICK:
		case ST_SELECT_HELD:
		case ST_SELECT_AND_BTN_HELD:
		case ST_ENABLE_MAPPING:
			// Led off
			digitalWrite (PIN_LED_MODE, LOW);
			break;
		case ST_MOUSE:
			// Blink slowly
			digitalWrite (PIN_LED_MODE, (millis () / 500) % 2 == 0);
			break;
		case ST_CD32:
			// Led lit up steadily
			digitalWrite (PIN_LED_MODE, HIGH);
			break;
		case ST_WAIT_SELECT_RELEASE:
		case ST_WAIT_BUTTON_PRESS:
		case ST_WAIT_BUTTON_RELEASE:
		case ST_WAIT_COMBO_PRESS:
		case ST_WAIT_COMBO_RELEASE:
		case ST_WAIT_SELECT_RELEASE_FOR_EXIT:
			// Programming mode, blink fast
			digitalWrite (PIN_LED_MODE, (millis () / 250) % 2 == 0);
			break;
#ifdef ENABLE_FACTORY_RESET
		case ST_FACTORY_RESET_WAIT_1:
			digitalWrite (PIN_LED_MODE, (millis () / 333) % 2 == 0);
			break;
		case ST_FACTORY_RESET_WAIT_2:
			digitalWrite (PIN_LED_MODE, (millis () / 80) % 2 == 0);
			break;
#endif
		default:
			// WTF?! Blink fast... er!
			digitalWrite (PIN_LED_MODE, (millis () / 100) % 2 == 0);
			break;
	}
}

void loop () {
	stateMachine ();
	updateLeds ();

	if (lastSwitchedTime > 0 && millis () - lastSwitchedTime > TIMEOUT_CD32_MODE) {
		// Pad Mode pin has been high for a while, disable CD32 mode
		toJoystick ();
		lastSwitchedTime = 0;
	}
}
