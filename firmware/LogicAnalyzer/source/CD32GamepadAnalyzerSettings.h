#ifndef CD32GAMEPAD_ANALYZER_SETTINGS
#define CD32GAMEPAD_ANALYZER_SETTINGS

#include <AnalyzerSettings.h>
#include <AnalyzerTypes.h>

class CD32GamepadAnalyzerSettings : public AnalyzerSettings {
public:
	CD32GamepadAnalyzerSettings();
	virtual ~CD32GamepadAnalyzerSettings();

	virtual bool SetSettingsFromInterfaces();
	void UpdateInterfacesFromSettings();
	virtual void LoadSettings (const char *settings);
	virtual const char *SaveSettings();


	Channel loadShiftChannel;
	Channel clockChannel;
	Channel dataChannel;
	//~ U32 mBitRate;

protected:
	std::auto_ptr< AnalyzerSettingInterfaceChannel >    loadShiftChannelInterface;
	std::auto_ptr< AnalyzerSettingInterfaceChannel >    clockChannelInterface;
	std::auto_ptr< AnalyzerSettingInterfaceChannel >    dataChannelInterface;
	//~ std::auto_ptr< AnalyzerSettingInterfaceInteger >    mBitRateInterface;
};

#endif //CD32GAMEPAD_ANALYZER_SETTINGS
