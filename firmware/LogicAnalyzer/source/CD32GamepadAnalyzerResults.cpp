#include "CD32GamepadAnalyzerResults.h"
#include <AnalyzerHelpers.h>
#include "CD32GamepadAnalyzer.h"
#include "CD32GamepadAnalyzerSettings.h"
#include <iostream>
#include <fstream>

CD32GamepadAnalyzerResults::CD32GamepadAnalyzerResults (
    CD32GamepadAnalyzer *analyzer, CD32GamepadAnalyzerSettings *settings)
	:   AnalyzerResults(),
	    mSettings (settings),
	    mAnalyzer (analyzer) {
}

CD32GamepadAnalyzerResults::~CD32GamepadAnalyzerResults() {
}

void CD32GamepadAnalyzerResults::GenerateBubbleText (U64 frame_index,
        Channel &channel, DisplayBase display_base) {
	ClearResultStrings();
	Frame frame = GetFrame (frame_index);
	char str[256];
	U16 buttons = frame.mData1 & 0xFFFF;

	// Numeric value
	//~ AnalyzerHelpers::GetNumberString (buttons, display_base, 8, str,
	                                  //~ 128);
	//~ AddResultString (str);

	// Short button names
	str[0] = '\0';
	if ((buttons & (1U << 9)) == 0)
		strcat (str, "B, ");
	if ((buttons & (1U << 8)) == 0)
		strcat (str, "R, ");
	if ((buttons & (1U << 7)) == 0)
		strcat (str, "Y, ");
	if ((buttons & (1U << 6)) == 0)
		strcat (str, "G, ");
	if ((buttons & (1U << 5)) == 0)
		strcat (str, "FR, ");
	if ((buttons & (1U << 4)) == 0)
		strcat (str, "FL, ");
	if ((buttons & (1U << 3)) == 0)
		strcat (str, "P, ");

	if (strlen (str) >= 2)
		str[strlen (str) - 2] = '\0';
	else
		strcpy (str, "None");

	if (frame.mFlags != 0) {
		strcat (str, " (Bad)");
	}
	AddResultString (str);

	// Long names
	strcpy (str, "Pressed: ");
	if ((buttons & (1U << 9)) == 0)
		strcat (str, "Blue, ");
	if ((buttons & (1U << 8)) == 0)
		strcat (str, "Red, ");
	if ((buttons & (1U << 7)) == 0)
		strcat (str, "Yellow, ");
	if ((buttons & (1U << 6)) == 0)
		strcat (str, "Green, ");
	if ((buttons & (1U << 5)) == 0)
		strcat (str, "Front Right, ");
	if ((buttons & (1U << 4)) == 0)
		strcat (str, "Front Left, ");
	if ((buttons & (1U << 3)) == 0)
		strcat (str, "Pause, ");

	if (strlen (str) > 9)
		str[strlen (str) - 2] = '\0';
	else
		strcat (str, "No buttons");

	if (frame.mFlags != 0) {
		char tmp[64];
		tmp[0] = '\0';

		if (frame.mFlags & CD32_FLAG_BADSYNC) {
			strcat (tmp, ", Bad Sync");
		}

		if (frame.mFlags & CD32_FLAG_INCOMPLETE) {
			strcat (tmp, ", Incomplete");
		}

		if (frame.mFlags & CD32_FLAG_NOCLOCK) {
			strcat (tmp, ", No Clock");
		}

		strcat (str, " (");
		strcat (str, tmp + 2);
		strcat (str, ")");
	}
	                                  
	AddResultString (str);
}

void CD32GamepadAnalyzerResults::GenerateExportFile (const char *file,
        DisplayBase display_base, U32 export_type_user_id) {
	std::ofstream file_stream (file, std::ios::out);

	U64 trigger_sample = mAnalyzer->GetTriggerSample();
	U32 sample_rate = mAnalyzer->GetSampleRate();

	file_stream << "Time [s],Value" << std::endl;

	U64 num_frames = GetNumFrames();

	for (U32 i = 0; i < num_frames; i++) {
		Frame frame = GetFrame (i);

		char time_str[128];
		AnalyzerHelpers::GetTimeString (frame.mStartingSampleInclusive, trigger_sample,
		                                sample_rate, time_str, 128);

		char number_str[128];
		AnalyzerHelpers::GetNumberString (frame.mData1, display_base, 8, number_str,
		                                  128);

		file_stream << time_str << "," << number_str << std::endl;

		if (UpdateExportProgressAndCheckForCancel (i, num_frames) == true) {
			file_stream.close();
			return;
		}
	}

	file_stream.close();
}

void CD32GamepadAnalyzerResults::GenerateFrameTabularText (U64 frame_index,
        DisplayBase display_base) {
			
//~ #ifdef SUPPORTS_PROTOCOL_SEARCH
	//~ Frame frame = GetFrame (frame_index);
	//~ ClearTabularText();

	//~ char number_str[128];
	//~ AnalyzerHelpers::GetNumberString (frame.mData1, display_base, 8, number_str,
	                                  //~ 128);
	//~ AddTabularText (number_str);
//~ #endif

	Frame frame = GetFrame( frame_index );
	char str[256];
	U16 buttons = frame.mData1 & 0xFFFF;

	ClearTabularText();

	// Short button names
	str[0] = '\0';
	if ((buttons & (1U << 9)) == 0)
		strcat (str, "B, ");
	if ((buttons & (1U << 8)) == 0)
		strcat (str, "R, ");
	if ((buttons & (1U << 7)) == 0)
		strcat (str, "Y, ");
	if ((buttons & (1U << 6)) == 0)
		strcat (str, "G, ");
	if ((buttons & (1U << 5)) == 0)
		strcat (str, "FR, ");
	if ((buttons & (1U << 4)) == 0)
		strcat (str, "FL, ");
	if ((buttons & (1U << 3)) == 0)
		strcat (str, "P, ");

	if (strlen (str) >= 2)
		str[strlen (str) - 2] = '\0';
	else
		strcpy (str, "None");

	if (frame.mFlags != 0) {
		strcat (str, " (Bad)");
	}
	AddTabularText (str);

	// Long names
	//~ strcpy (str, "Pressed: ");
	//~ if ((buttons & (1U << 9)) == 0)
		//~ strcat (str, "Blue, ");
	//~ if ((buttons & (1U << 8)) == 0)
		//~ strcat (str, "Red, ");
	//~ if ((buttons & (1U << 7)) == 0)
		//~ strcat (str, "Yellow, ");
	//~ if ((buttons & (1U << 6)) == 0)
		//~ strcat (str, "Green, ");
	//~ if ((buttons & (1U << 5)) == 0)
		//~ strcat (str, "Front Right, ");
	//~ if ((buttons & (1U << 4)) == 0)
		//~ strcat (str, "Front Left, ");
	//~ if ((buttons & (1U << 3)) == 0)
		//~ strcat (str, "Pause, ");

	//~ if (strlen (str) > 9)
		//~ str[strlen (str) - 2] = '\0';
	//~ else
		//~ strcat (str, "No buttons");

	//~ if (frame.mFlags != 0) {
		//~ char tmp[64];
		//~ tmp[0] = '\0';

		//~ if (frame.mFlags & CD32_FLAG_BADSYNC) {
			//~ strcat (tmp, ", Bad Sync");
		//~ }

		//~ if (frame.mFlags & CD32_FLAG_INCOMPLETE) {
			//~ strcat (tmp, ", Incomplete");
		//~ }

		//~ if (frame.mFlags & CD32_FLAG_NOCLOCK) {
			//~ strcat (tmp, ", No Clock");
		//~ }

		//~ strcat (str, " (");
		//~ strcat (str, tmp + 2);
		//~ strcat (str, ")");
	//~ }
	                                  
	//~ AddTabularText (str);
}

void CD32GamepadAnalyzerResults::GeneratePacketTabularText (U64 packet_id,
        DisplayBase display_base) {

	ClearResultStrings();
	AddResultString ("Not supported");
}

void CD32GamepadAnalyzerResults::GenerateTransactionTabularText (
    U64 transaction_id, DisplayBase display_base) {

	ClearResultStrings();
	AddResultString ("Not supported");
}
