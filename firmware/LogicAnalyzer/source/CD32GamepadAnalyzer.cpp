#include "CD32GamepadAnalyzer.h"
#include "CD32GamepadAnalyzerSettings.h"
#include <AnalyzerChannelData.h>

CD32GamepadAnalyzer::CD32GamepadAnalyzer()
	:   Analyzer2(),
	    mSettings (new CD32GamepadAnalyzerSettings()),
	    mSimulationInitilized (false) {
	SetAnalyzerSettings (mSettings.get());
}

CD32GamepadAnalyzer::~CD32GamepadAnalyzer() {
	KillThread();
}

void CD32GamepadAnalyzer::SetupResults() {
	mResults.reset (new CD32GamepadAnalyzerResults (this, mSettings.get()));
	SetAnalyzerResults (mResults.get());
	mResults->AddChannelBubblesWillAppearOn (mSettings->dataChannel);
}

void CD32GamepadAnalyzer::WorkerThread() {
	loadShiftData = GetAnalyzerChannelData (mSettings->loadShiftChannel);
	clockData = GetAnalyzerChannelData (mSettings->clockChannel);
	dataData = GetAnalyzerChannelData (mSettings->dataChannel);

	for (;;) {
		// Look for next falling edge on load/shift
		if (loadShiftData->GetBitState() == BIT_HIGH) {
			loadShiftData->AdvanceToNextEdge();
		} else {
			loadShiftData->AdvanceToNextEdge();
			loadShiftData->AdvanceToNextEdge();
		}

		// Frame starts here
		U64 value = ~0;		// All ones
		bool incomplete = false;
		bool noclock = false;
		U64 starting_sample = loadShiftData->GetSampleNumber();
		mResults->AddMarker (starting_sample, AnalyzerResults::Start,
							 mSettings->loadShiftChannel);

		// Not sure why we need this if, sometimes we will already be past the correct position?!?
		clockData->AdvanceToAbsPosition (starting_sample);
		for (int i = 0; !noclock && !incomplete && i < 10; i++) {
			// Find next rising edge of clock
			U64 nextRisingClock;
			nextRisingClock = clockData->GetSampleOfNextEdge ();
			if (loadShiftData->WouldAdvancingToAbsPositionCauseTransition (nextRisingClock)) {
				// No more cock pulses in current low period of loadShiftData
				mResults->AddMarker (loadShiftData->GetSampleNumber(), AnalyzerResults::X,
								 mSettings->clockChannel);
				if (i == 0) {
					noclock = true;
				} else {
					incomplete = true;
				}
			} else {
				// Clock edge found, make sure it's a rising one
				clockData->AdvanceToAbsPosition (nextRisingClock);
				if (clockData->GetBitState() == BIT_LOW) {
					// We are at a FALLING edge, go to next edge
					nextRisingClock = clockData->GetSampleOfNextEdge ();
					if (loadShiftData->WouldAdvancingToAbsPositionCauseTransition (nextRisingClock)) {
						// No more cock pulses in current low period of loadShiftData
						//~ mResults->AddMarker (loadShiftData->GetSampleNumber(), AnalyzerResults::X,
										 //~ mSettings->dataChannel);
						if (i == 0) {
							noclock = true;
						} else {
							incomplete = true;
						}
					} else {
						clockData->AdvanceToAbsPosition (nextRisingClock);
					}
				}
			}

			if (!noclock && !incomplete) {
				// Rising clock pulse found, we are already positioned on it

				// Let's put a dot exactly where we sample this bit:
				mResults->AddMarker (clockData->GetSampleNumber(), AnalyzerResults::UpArrow,
									 mSettings->clockChannel);

				dataData->AdvanceToAbsPosition (clockData->GetSampleNumber());
				if (dataData->GetBitState() == BIT_HIGH) {
					mResults->AddMarker (clockData->GetSampleNumber(), AnalyzerResults::One,
										 mSettings->dataChannel);
				} else {
					value &= ~(1 << (9 - i));

					mResults->AddMarker (clockData->GetSampleNumber(), AnalyzerResults::Zero,
										 mSettings->dataChannel);
				}
			}
		}

		loadShiftData->AdvanceToNextEdge();     // END OF FRAME
		if (noclock || incomplete) {
			mResults->AddMarker (loadShiftData->GetSampleNumber(), AnalyzerResults::X,
				 mSettings->clockChannel);
		}
		U64 ending_sample = loadShiftData->GetSampleNumber();
		mResults->AddMarker (ending_sample, AnalyzerResults::Stop,
							 mSettings->loadShiftChannel);

		//we have a byte to save.
		Frame frame;
		frame.mStartingSampleInclusive = starting_sample;
		frame.mEndingSampleInclusive = ending_sample;
		frame.mData1 = value;
		frame.mData2 = 0x00;

		frame.mFlags = 0;

		if ((value & (1U << 2U)) == 0 ||
			(value & (1U << 1U)) != 0 ||
			(value & (1U << 0U)) != 0) {
			frame.mFlags |= CD32_FLAG_BADSYNC;
		}

		if (incomplete) {
			frame.mFlags |= CD32_FLAG_INCOMPLETE;
		}

		if (noclock) {
			frame.mFlags |= CD32_FLAG_NOCLOCK;
		}

		if (frame.mFlags != 0) {
			frame.mFlags |= DISPLAY_AS_ERROR_FLAG;
		}

		mResults->AddFrame (frame);
		mResults->CommitResults ();
		ReportProgress (frame.mEndingSampleInclusive);
		
		CheckIfThreadShouldExit ();
	}
}

bool CD32GamepadAnalyzer::NeedsRerun() {
	return false;
}

U32 CD32GamepadAnalyzer::GenerateSimulationData (U64 minimum_sample_index,
        U32 device_sample_rate, SimulationChannelDescriptor **simulation_channels) {
	if (mSimulationInitilized == false) {
		mSimulationDataGenerator.Initialize (GetSimulationSampleRate(),
		                                     mSettings.get());
		mSimulationInitilized = true;
	}

	return mSimulationDataGenerator.GenerateSimulationData (minimum_sample_index,
	        device_sample_rate, simulation_channels);
}

U32 CD32GamepadAnalyzer::GetMinimumSampleRateHz() {
	return 25000;      // Smallest sample rate
}

const char *CD32GamepadAnalyzer::GetAnalyzerName() const {
	return "CD32 Gamepad";
}

const char *GetAnalyzerName() {
	return "Commodore Amiga CD32 Gamepad";
}

Analyzer *CreateAnalyzer() {
	return new CD32GamepadAnalyzer();
}

void DestroyAnalyzer (Analyzer *analyzer) {
	delete analyzer;
}
