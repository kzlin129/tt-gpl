#define SUPPORT_SMDK2440

inline void IO_InitSMDK2440(void)
{
	VALUE_SET(id             , GOTYPE_SMDK2440);
	VALUE_SET(cputype        , GOCPU_S3C2440);
	VALUE_SET(sdcard         , 1);
	VALUE_SET(usbslavetype   , GOUSB_S3C24XX);

	STRING_SET(name,"SMDK2440 Development Board");
	STRING_SET(usbname,"SMDK2440");
	STRING_SET(familyname,"SMDK2440");
	STRING_SET(dockdev,"ttySAC0");

	PIN_SET(CTS_DOCK        , PIN_GPH0);
	PIN_SET(RTS_DOCK        , PIN_GPH1);
	PIN_SET(TXD_DOCK        , PIN_GPH2);
	PIN_SET(RXD_DOCK        , PIN_GPH3);
	PIN_SET(ON_OFF          , PIN_GPF0 | PIN_INVERTED);
}
