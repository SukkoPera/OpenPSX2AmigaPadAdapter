#ifndef CD32GAMEPAD_ANALYZER_RESULTS
#define CD32GAMEPAD_ANALYZER_RESULTS

#include <AnalyzerResults.h>

#define CD32_FLAG_BADSYNC		(1 << 0)
#define CD32_FLAG_INCOMPLETE	(1 << 1)
#define CD32_FLAG_NOCLOCK		(1 << 2)

class CD32GamepadAnalyzer;
class CD32GamepadAnalyzerSettings;

class CD32GamepadAnalyzerResults : public AnalyzerResults {
public:
	CD32GamepadAnalyzerResults (CD32GamepadAnalyzer *analyzer,
	                            CD32GamepadAnalyzerSettings *settings);
	virtual ~CD32GamepadAnalyzerResults();

	virtual void GenerateBubbleText (U64 frame_index, Channel &channel,
	                                 DisplayBase display_base);
	virtual void GenerateExportFile (const char *file, DisplayBase display_base,
	                                 U32 export_type_user_id);

	virtual void GenerateFrameTabularText (U64 frame_index,
	                                       DisplayBase display_base);
	virtual void GeneratePacketTabularText (U64 packet_id,
	                                        DisplayBase display_base);
	virtual void GenerateTransactionTabularText (U64 transaction_id,
	        DisplayBase display_base);

protected: //functions

protected:  //vars
	CD32GamepadAnalyzerSettings *mSettings;
	CD32GamepadAnalyzer *mAnalyzer;
};

#endif //CD32GAMEPAD_ANALYZER_RESULTS
