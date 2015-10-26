#define SUPPORT_LISBON
inline void IO_InitLisbon(void)
{
	VALUE_SET(id                 , GOTYPE_LISBON);
	VALUE_SET(caseid             , GOCASE_LISBON);
	VALUE_SET(btclass            , 0x200408);
	VALUE_SET(handsfree          , 1);
	VALUE_SET(headsetgw          , 1);
	VALUE_SET(batterytype	     , GOBATTERY_ICR18650_2200);
	VALUE_SET(chargertype        , GOCHARGER_LTC1733);
	VALUE_SET(chargerresistor    , 1300);
	VALUE_SET(gpsephemeris       , 1);	

	STRING_SET(name,"TomTom RIDER");
	STRING_SET(btname,"TomTom RIDER");
	STRING_SET(usbname,"RIDER");
	STRING_SET(familyname,"TomTom RIDER");
	STRING_SET(projectname,"LISBON");
	STRING_SET(requiredbootloader,"5.15");

	// Pins that are only applicable for Lisbon
	PIN_SET(DOCK_MOTOR_SENSE , PIN_GPJ10 | PIN_INVERTED); // For motor dock detection
	PIN_SET(DOCK_SENSE       , PIN_GPJ8 | PIN_INVERTED);  // For windscreen dock detection
	PIN_SET(IGNITION         , PIN_GPF3 | PIN_INVERTED); // DOCK_GP1 is used as IGNITION
}
