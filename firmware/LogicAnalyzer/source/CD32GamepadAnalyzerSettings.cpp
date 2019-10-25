#include "CD32GamepadAnalyzerSettings.h"
#include <AnalyzerHelpers.h>


CD32GamepadAnalyzerSettings::CD32GamepadAnalyzerSettings()
	:   loadShiftChannel (UNDEFINED_CHANNEL),
	    clockChannel (UNDEFINED_CHANNEL),
	    dataChannel (UNDEFINED_CHANNEL)
	    //~ mBitRate( 9600 )
{
	loadShiftChannelInterface.reset (new AnalyzerSettingInterfaceChannel());
	loadShiftChannelInterface->SetTitleAndTooltip ("Load/Shift Pin (Pin 5)",
	        "As long as shift/load on the shift register is switched to shift it will shift out the state of the buttons that where loaded before.");
	loadShiftChannelInterface->SetChannel (loadShiftChannel);

	clockChannelInterface.reset (new AnalyzerSettingInterfaceChannel());
	clockChannelInterface->SetTitleAndTooltip ("Clock Pin (Pin 6)",
	        "With every rising edge on (P6) the state of the Gamepad buttons will be shifted from the inputs of the shift register over the 74LS125 driver to Pot-Y (P9).");
	clockChannelInterface->SetChannel (clockChannel);

	dataChannelInterface.reset (new AnalyzerSettingInterfaceChannel());
	dataChannelInterface->SetTitleAndTooltip ("Data Pin (Pin 9)",
	        "Standard Commodore Amiga CD32 Gamepad");
	dataChannelInterface->SetChannel (dataChannel);

	//~ mBitRateInterface.reset( new AnalyzerSettingInterfaceInteger() );
	//~ mBitRateInterface->SetTitleAndTooltip( "Bit Rate (Bits/S)",  "Specify the bit rate in bits per second." );
	//~ mBitRateInterface->SetMax( 6000000 );
	//~ mBitRateInterface->SetMin( 1 );
	//~ mBitRateInterface->SetInteger( mBitRate );

	AddInterface (loadShiftChannelInterface.get());
	AddInterface (clockChannelInterface.get());
	AddInterface (dataChannelInterface.get());
	//~ AddInterface( mBitRateInterface.get() );

	//~ AddExportOption( 0, "Export as text/csv file" );
	//~ AddExportExtension( 0, "text", "txt" );
	//~ AddExportExtension( 0, "csv", "csv" );

	ClearChannels();
	//~ AddChannel( mInputChannel, "Serial", false );
}

CD32GamepadAnalyzerSettings::~CD32GamepadAnalyzerSettings() {
}

bool CD32GamepadAnalyzerSettings::SetSettingsFromInterfaces() {
	loadShiftChannel = loadShiftChannelInterface->GetChannel();
	clockChannel = clockChannelInterface->GetChannel();
	dataChannel = dataChannelInterface->GetChannel();
	//~ mBitRate = mBitRateInterface->GetInteger();

	ClearChannels();
	AddChannel (loadShiftChannel, "CD32 Load/Shift Pin", true);
	AddChannel (clockChannel, "CD32 Clock Pin", true);
	AddChannel (dataChannel, "CD32 Data Pin", true);

	return true;
}

void CD32GamepadAnalyzerSettings::UpdateInterfacesFromSettings() {
	loadShiftChannelInterface->SetChannel (loadShiftChannel);
	clockChannelInterface->SetChannel (clockChannel);
	dataChannelInterface->SetChannel (dataChannel);
	//~ mBitRateInterface->SetInteger( mBitRate );
}

void CD32GamepadAnalyzerSettings::LoadSettings (const char *settings) {
	SimpleArchive text_archive;
	text_archive.SetString (settings);

	text_archive >> loadShiftChannel;
	text_archive >> clockChannel;
	text_archive >> dataChannel;
	//~ text_archive >> mBitRate;

	ClearChannels();
	AddChannel (loadShiftChannel, "Commodore Amiga CD32 Gamepad", true);
	AddChannel (clockChannel, "Commodore Amiga CD32 Gamepad", true);
	AddChannel (dataChannel, "Commodore Amiga CD32 Gamepad", true);

	UpdateInterfacesFromSettings();
}

const char *CD32GamepadAnalyzerSettings::SaveSettings() {
	SimpleArchive text_archive;

	text_archive << loadShiftChannel;
	text_archive << clockChannel;
	text_archive << dataChannel;
	//~ text_archive << mBitRate;

	return SetReturnString (text_archive.GetString());
}
