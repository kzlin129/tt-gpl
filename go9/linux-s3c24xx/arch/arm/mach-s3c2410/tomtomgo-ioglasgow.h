#define SUPPORT_GLASGOW
inline void IO_InitGlasgow(void)
{
	VALUE_SET(id                 , GOTYPE_GLASGOW);
	VALUE_SET(caseid             , GOCASE_GLASGOW);
	VALUE_SET(btclass            , 0x001f00);
	VALUE_SET(batterytype	     , GOBATTERY_ICP803443_1350);
	VALUE_SET(chargertype        , GOCHARGER_LTC1733);
	VALUE_SET(chargerresistor    , 3080);

	STRING_SET(name,"TomTom ONE");
	STRING_SET(btname,"TomTom ONE");
	STRING_SET(usbname,"ONE");
	STRING_SET(familyname,"TomTom ONE");
	STRING_SET(projectname,"GLASGOW");
	STRING_SET(requiredbootloader,"3.30");

	// Pins that are only applicable for Glasgow
	PIN_SET(IGNITION         , PIN_GPF3 | PIN_INVERTED); // DOCK_GP1 is used as IGNITION
	PIN_SET(HEADPHONE_DETECT , PIN_GPF5);                // DOCK_GP3 is used as HEADPHONE_DETECT
	PIN_SET(AMP_ON           , PIN_GPF6);                // DOCK_GP4 is used as AMP_ON
#if 0
	PIN_SET(HW_IIC_SDA       , PIN_GPJ10);
	PIN_SET(HW_IIC_SCL       , PIN_GPJ8);
	PIN_SET(TOUCHPAD_SW      , PIN_GPF4 | PIN_INVERTED);
#endif
}
