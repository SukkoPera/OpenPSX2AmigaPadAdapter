#ifndef CD32GAMEPAD_SIMULATION_DATA_GENERATOR
#define CD32GAMEPAD_SIMULATION_DATA_GENERATOR

#include <SimulationChannelDescriptor.h>
#include <AnalyzerHelpers.h>
#include <string>

class CD32GamepadAnalyzerSettings;

class CD32GamepadSimulationDataGenerator {
public:
	CD32GamepadSimulationDataGenerator();
	~CD32GamepadSimulationDataGenerator();

	void Initialize (U32 simulation_sample_rate,
	                 CD32GamepadAnalyzerSettings *settings);
	U32 GenerateSimulationData (U64 newest_sample_requested, U32 sample_rate,
	                            SimulationChannelDescriptor **simulation_channel);

protected:
	CD32GamepadAnalyzerSettings *mSettings;
	U32 mSimulationSampleRateHz;

private:
	void CreateSerialByte();
	std::string mSerialText;
	U32 mStringIndex;
	U8 currentBtn;

	ClockGenerator clockgen;

	SimulationChannelDescriptorGroup chanGroup;
	SimulationChannelDescriptor* loadShift;
	SimulationChannelDescriptor* srClock;
	SimulationChannelDescriptor* srOutput;

};
#endif //CD32GAMEPAD_SIMULATION_DATA_GENERATOR
