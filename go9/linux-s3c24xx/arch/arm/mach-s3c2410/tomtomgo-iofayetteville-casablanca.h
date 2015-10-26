#define SUPPORT_FAYETTEVILLE_CASABLANCA
//#define SUPPORT_CASABLANCA_PR0
inline void IO_InitFayettevilleCasablanca(void)
{
	VALUE_SET(id                 , GOTYPE_FAYETTEVILLE_CASABLANCA);
	VALUE_SET(caseid             , GOCASE_EDINBURGH);
	VALUE_SET(cputype            , GOCPU_S3C2412);
	VALUE_SET(batterytype	     , GOBATTERY_1100);
	VALUE_SET(chargertype        , GOCHARGER_LTC3455);
	VALUE_SET(chargerresistor    , 2490);
	VALUE_SET(sdcard             , 1);
        VALUE_SET(sdslot             , 0);
	VALUE_SET(internalflash      , 1);
	VALUE_SET(sdisharedbus       , 0);
	VALUE_SET(tfttype            , GOTFT_SAMSUNG_LMS350GF);
	VALUE_SET(backlighttype	     , GOBACKLIGHT_CH0_AT1312_350);
	VALUE_SET(tsxchannel         , 5);
	VALUE_SET(tsychannel         , 7);
	VALUE_SET(gpstype            , GOGPS_GL_INT_LNA);
	VALUE_SET(gldetected         , 1);
	VALUE_SET(codectype          , GOCODEC_WM8711);
	VALUE_SET(usbslavetype       , GOUSB_S3C24XX);
	VALUE_SET(pnp                , 1);
	VALUE_SET(usbdevicehostcapable,1);
	VALUE_SET(ohciports          , 0x01);
	VALUE_SET(tftsoftstart       , 1);
        VALUE_SET(pnp                , 1);
	VALUE_SET(gpsephemeris       , 1);

	STRING_SET(name,"TomTom ONE");
	STRING_SET(usbname,"ONE (v6)");
	STRING_SET(familyname,"TomTom ONE");
	STRING_SET(projectname,"Fayetteville C");
	STRING_SET(requiredbootloader,"5.2910");
	STRING_SET(dockdev,"ttySAC0");
	STRING_SET(gpsdev,"ttySAC1");

	PIN_SET(SD_PWR_ON          , PIN_GPC6);
	PIN_SET(WP_SD              , PIN_GPH8);
	PIN_SET(CD_SD              , PIN_GPF2 | PIN_INVERTED);
	PIN_SET(SW_SD              , PIN_GPG11);
	PIN_SET(EN_SD              , PIN_GPB1 | PIN_INVERTED);
#ifdef SUPPORT_CASABLANCA_PR0
	PIN_SET(MOVI_PWR_ON        , PIN_GPA20 | PIN_INVERTED);
#else
	PIN_SET(MOVI_PWR_ON        , PIN_GPH0 | PIN_INVERTED);
#endif
	PIN_SET(PULLUP_SD          , PIN_GPB9);
	PIN_SET(GPS_RESET          , PIN_GPC5 | PIN_INVERTED);
	PIN_SET(GPS_ON             , PIN_GPF4);
	PIN_SET(GPS_POWERON_OUT    , PIN_GPF3);
	PIN_SET(BATT_TEMP_OVER     , PIN_GPG5 | PIN_INVERTED);
	PIN_SET(CHARGING           , PIN_GPE14 | PIN_INVERTED);
	PIN_SET(AIN4_PWR           , PIN_GPA12);
	PIN_SET(DAC_PWR_ON         , PIN_GPE13);
	PIN_SET(AMP_ON             , PIN_GPF6);
	PIN_SET(I2SSDO             , PIN_GPE4); 
	PIN_SET(CDCLK              , PIN_GPE2); 
	PIN_SET(I2SSCLK            , PIN_GPE1); 
	PIN_SET(I2SLRCK            , PIN_GPE0); 
	PIN_SET(L3CLOCK            , PIN_GPB4); 
	PIN_SET(L3MODE             , PIN_GPB3); 
	PIN_SET(L3DATA             , PIN_GPB2); 
	PIN_SET(TXD_DOCK           , PIN_GPH2);
	PIN_SET(RXD_DOCK           , PIN_GPH3);
	PIN_SET(RXD_DOCK_INT       , PIN_GPG1);	
	PIN_SET(LINEIN_DETECT      , PIN_GPG0 | PIN_INVERTED);		
#ifdef SUPPORT_CASABLANCA_PR0
	PIN_SET(DOCK_PWREN         , PIN_GPA18);
#else
	PIN_SET(DOCK_PWREN         , PIN_GPH1 );
#endif
	PIN_SET(TXD_GPS            , PIN_GPH4);
	PIN_SET(RXD_GPS            , PIN_GPH5);
	PIN_SET(CTS_GPS            , PIN_GPG9);
	PIN_SET(RTS_GPS            , PIN_GPG10);
	PIN_SET(GPS_ON             , PIN_GPF4);
	PIN_SET(LCD_VCC_PWREN      , PIN_GPG4 | PIN_INVERTED);
	PIN_SET(LCD_BIAS_PWREN     , PIN_GPC7);
	PIN_SET(BACKLIGHT_PWM      , PIN_GPB0);
	PIN_SET(LCD_CS             , PIN_GPG3 | PIN_INVERTED);
	PIN_SET(LCD_SCL            , PIN_GPG7);
	PIN_SET(LCD_SDI            , PIN_GPG6);
	PIN_SET(LCD_RESET          , PIN_GPF5 | PIN_INVERTED);
	PIN_SET(USB_HOST_DETECT    , PIN_GPF1 | PIN_INVERTED); 			/* shared with ACPWR and IGNITION */
	PIN_SET(USB_PULL_EN        , PIN_GPF7);
	PIN_SET(USB_SUSPEND_OUT    , PIN_GPE12);
	PIN_SET(USB_HP             , PIN_GPE11);
	PIN_SET(USB_PWR_BYPASS     , PIN_GPG8);
	PIN_SET(PWR_RST            , PIN_GPE3);
	PIN_SET(LCD_ID             , PIN_GPE15);

	PIN_SET(ON_OFF             , PIN_GPF0 | PIN_INVERTED);
	PIN_SET(ACPWR              , PIN_GPF1 | PIN_INVERTED);
	PIN_SET(IGNITION           , PIN_GPF1 | PIN_INVERTED);
	PIN_SET(XMON               , PIN_GPG13 | PIN_OPEN_COLLECTOR);
	PIN_SET(XPON               , PIN_GPG12 | PIN_OPEN_EMITTER);
	PIN_SET(YMON               , PIN_GPG15 | PIN_OPEN_COLLECTOR);
	PIN_SET(YPON               , PIN_GPG14 | PIN_OPEN_EMITTER);
	PIN_SET(TSDOWN             , PIN_GPG15 | PIN_OPEN_COLLECTOR);
	PIN_SET(FACTORY_TEST_POINT , PIN_GPG2);

	PIN_SET(TYPE_SUB_ID0       , PIN_GPB8 );
}

