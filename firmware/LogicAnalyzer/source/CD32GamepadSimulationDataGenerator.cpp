#include "CD32GamepadSimulationDataGenerator.h"
#include "CD32GamepadAnalyzerSettings.h"

#include <AnalyzerHelpers.h>

const U8 BTN_BLUE =		0;	//!< \a Blue Button
const U8 BTN_RED =		1;	//!< \a Red Button
const U8 BTN_YELLOW =	2;	//!< \a Yellow Button
const U8 BTN_GREEN =	3;	//!< \a Green Button
const U8 BTN_FRONT_R =	4;	//!< \a Front \a Right Button
const U8 BTN_FRONT_L =	5;	//!< \a Front \a Left Button
const U8 BTN_START =	6;	//!< \a Start/Pause Button

const U8 N_BUTTONS = 7;

CD32GamepadSimulationDataGenerator::CD32GamepadSimulationDataGenerator()
	:   mSerialText ("My first analyzer, woo hoo!"),
	    mStringIndex (0) {
}

CD32GamepadSimulationDataGenerator::~CD32GamepadSimulationDataGenerator() {
}

void CD32GamepadSimulationDataGenerator::Initialize (U32 simulation_sample_rate,
        CD32GamepadAnalyzerSettings *settings) {
			
	mSimulationSampleRateHz = simulation_sample_rate;
	mSettings = settings;

	currentBtn = BTN_BLUE;

	//~ clockgen.Init (xxx, simulation_sample_rate);
	
	loadShift = chanGroup.Add (settings -> loadShiftChannel, mSimulationSampleRateHz, BIT_HIGH);
	srClock = chanGroup.Add (settings -> clockChannel, mSimulationSampleRateHz, BIT_HIGH);
	srOutput = chanGroup.Add (settings -> dataChannel, mSimulationSampleRateHz, BIT_HIGH);
}

U32 CD32GamepadSimulationDataGenerator::GenerateSimulationData (
    U64 largest_sample_requested, U32 sample_rate,
    SimulationChannelDescriptor **simulation_channels) {
		
	U64 adjusted_largest_sample_requested =
	    AnalyzerHelpers::AdjustSimulationTargetSample (largest_sample_requested,
		sample_rate, mSimulationSampleRateHz);

	while (loadShift -> GetCurrentSampleNumber() < adjusted_largest_sample_requested) {
		CreateSerialByte();
	}

	*simulation_channels = chanGroup.GetArray ();
	return chanGroup.GetCount ();
}

void CD32GamepadSimulationDataGenerator::CreateSerialByte() {
	//~ U32 samples_per_bit = mSimulationSampleRateHz / mSettings->mBitRate;

	//~ U8 byte = mSerialText[ mStringIndex ];
	//~ mStringIndex++;
	//~ if( mStringIndex == mSerialText.size() )
	//~ mStringIndex = 0;

	//~ //we're currenty high
	//~ //let's move forward a little
	//~ mSerialSimulationData.Advance( samples_per_bit * 10 );

	//~ mSerialSimulationData.Transition();  //low-going edge for start bit
	//~ mSerialSimulationData.Advance( samples_per_bit );  //add start bit time

	//~ U8 mask = 0x1 << 7;
	//~ for( U32 i=0; i<8; i++ )
	//~ {
	//~ if( ( byte & mask ) != 0 )
	//~ mSerialSimulationData.TransitionIfNeeded( BIT_HIGH );
	//~ else
	//~ mSerialSimulationData.TransitionIfNeeded( BIT_LOW );

	//~ mSerialSimulationData.Advance( samples_per_bit );
	//~ mask = mask >> 1;
	//~ }

	//~ mSerialSimulationData.TransitionIfNeeded( BIT_HIGH ); //we need to end high

	//~ //lets pad the end a bit for the stop bit:
	//~ mSerialSimulationData.Advance( samples_per_bit );

	const U32 samples_per_us = mSimulationSampleRateHz / 1000000UL;
	U8 i;
	
	// 40 us idle
	chanGroup.AdvanceAll (samples_per_us * 40);
	
	// Start CD32 sampling and stay low
	loadShift -> Transition ();		// -> Low
	srClock -> TransitionIfNeeded (BIT_LOW);

	chanGroup.AdvanceAll (samples_per_us * 25);
	
	for (i = 0; i < 9; ++i) {
		srClock -> Transition ();		// -> High		
		srClock -> Advance (samples_per_us * 6);
		srClock -> Transition ();		// -> Low
		srClock -> Advance (samples_per_us * 35);

		srOutput -> Advance (samples_per_us * 4);
		if (i == currentBtn - 1 || i == 7 || i == 8) {
			srOutput -> TransitionIfNeeded (BIT_LOW);
		} else {
			srOutput -> TransitionIfNeeded (BIT_HIGH);
		}
		srOutput -> Advance (samples_per_us * 37);

		loadShift -> Advance (samples_per_us * 41);
	}

	// Last bit
	srClock -> Transition ();		// -> High		
	srClock -> Advance (samples_per_us * 16);
	//~ srClock -> Transition ();		// -> Low
	//~ srClock -> Advance (samples_per_us * 10);
	srOutput -> Advance (samples_per_us * 16);
	loadShift -> Advance (samples_per_us * 16);

	loadShift -> Transition ();		// -> High

	currentBtn = (currentBtn + 1) % N_BUTTONS;
}
