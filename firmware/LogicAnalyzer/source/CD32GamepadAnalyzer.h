#ifndef CD32GAMEPAD_ANALYZER_H
#define CD32GAMEPAD_ANALYZER_H

#include <Analyzer.h>
#include "CD32GamepadAnalyzerResults.h"
#include "CD32GamepadSimulationDataGenerator.h"

class CD32GamepadAnalyzerSettings;
class ANALYZER_EXPORT CD32GamepadAnalyzer : public Analyzer2 {
public:
	CD32GamepadAnalyzer();
	virtual ~CD32GamepadAnalyzer();

	virtual void SetupResults();
	virtual void WorkerThread();

	virtual U32 GenerateSimulationData (U64 newest_sample_requested,
	                                    U32 sample_rate, SimulationChannelDescriptor **simulation_channels);
	virtual U32 GetMinimumSampleRateHz();

	virtual const char *GetAnalyzerName() const;
	virtual bool NeedsRerun();

protected: //vars
	std::auto_ptr< CD32GamepadAnalyzerSettings > mSettings;
	std::auto_ptr< CD32GamepadAnalyzerResults > mResults;
	AnalyzerChannelData *loadShiftData;
	AnalyzerChannelData *clockData;
	AnalyzerChannelData *dataData;	// ;)

	CD32GamepadSimulationDataGenerator mSimulationDataGenerator;
	bool mSimulationInitilized;

	//Serial analysis vars:
	//~ U32 mSampleRateHz;
	//~ U32 mStartOfStopBitOffset;
	//~ U32 mEndOfStopBitOffset;
};

extern "C" ANALYZER_EXPORT const char *__cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer *__cdecl CreateAnalyzer();
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer (Analyzer *analyzer);

#endif //CD32GAMEPAD_ANALYZER_H
