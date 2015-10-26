#define SUPPORT_ABERDEEN
inline void IO_InitAberdeen(void)
{
	VALUE_SET(id                 , GOTYPE_ABERDEEN);
	VALUE_SET(caseid             , GOCASE_GLASGOW);
	VALUE_SET(cputype            , GOCPU_S3C2440);
	VALUE_SET(btchip             , GOBT_BC3);
	VALUE_SET(btspeed            , 921600);
	VALUE_SET(btclock            , 16000000);
	VALUE_SET(btclass            , 0x001f00);
	VALUE_SET(lowbutton          , 1);
	VALUE_SET(batterytype	     , GOBATTERY_ICP803443_1350);
	VALUE_SET(chargertype        , GOCHARGER_LTC3455);
	VALUE_SET(chargerresistor    , 1540);
	VALUE_SET(sdcard             , 1);
	VALUE_SET(sdslot             , 1);
	VALUE_SET(tfttype            , GOTFT_SAMSUNG_LTV350);
	VALUE_SET(backlighttype	     , GOBACKLIGHT_CH0_TPS61042_350);
	VALUE_SET(tsxchannel         , 5);
	VALUE_SET(tsychannel         , 7);
	VALUE_SET(gpstype            , GOGPS_SIRF3);
	VALUE_SET(codectype          , GOCODEC_WM8711);
	VALUE_SET(usbslavetype       , GOUSB_S3C24XX);
	VALUE_SET(pnp                , 1);
	VALUE_SET(usbdevicehostcapable,1);
	VALUE_SET(ohciports          , 0x01);
	VALUE_SET(tftsoftstart       , 1);
	VALUE_SET(gpsephemeris       , 1);

	STRING_SET(name,"TomTom ONE");
	STRING_SET(btname,"TomTom ONE");
	STRING_SET(usbname,"ONE (v2)");
	STRING_SET(familyname,"TomTom ONE");
	STRING_SET(projectname,"ABERDEEN");
	STRING_SET(requiredbootloader,"4.00");
	STRING_SET(dockdev,"ttySAC0");
	STRING_SET(btdev,"ttySAC1");
	STRING_SET(gpsdev,"ttySAC2");

	// Mainboard
	PIN_SET(SD_PWR_ON       , PIN_GPA17);
	PIN_SET(WP_SD           , PIN_GPH8);
	PIN_SET(CD_SD           , PIN_GPF2 | PIN_INVERTED);
	PIN_SET(GPS_RESET       , PIN_GPC5 | PIN_INVERTED);
	PIN_SET(GPS_ON          , PIN_GPJ6);
	PIN_SET(GPS_1PPS        , PIN_GPG0);
	PIN_SET(GPS_REPRO       , PIN_GPG1);
	PIN_SET(BATT_TEMP_OVER  , PIN_GPJ1 | PIN_INVERTED);
	PIN_SET(CHARGE_OUT      , PIN_GPJ7 | PIN_OPEN_EMITTER);
	PIN_SET(CHARGING        , PIN_GPJ12 | PIN_INVERTED);
	PIN_SET(PWR_RST         , PIN_GPJ4); // | PIN_OPEN_COLLECTOR); // ); // |
	PIN_SET(AIN4_PWR        , PIN_GPA18);
	PIN_SET(DAC_PWR_ON      , PIN_GPA22);
	PIN_SET(I2SSDO          , PIN_GPE4); 
	PIN_SET(CDCLK           , PIN_GPE2); 
	PIN_SET(I2SSCLK         , PIN_GPE1); 
	PIN_SET(I2SLRCK         , PIN_GPE0); 
	PIN_SET(I2SSDI          , PIN_GPE3); 
	PIN_SET(L3CLOCK         , PIN_GPB4); 
	PIN_SET(L3MODE          , PIN_GPB3); 
	PIN_SET(L3DATA          , PIN_GPB2); 
	PIN_SET(CTS_DOCK        , PIN_GPH0);
	PIN_SET(RTS_DOCK        , PIN_GPH1);
	PIN_SET(TXD_DOCK        , PIN_GPH2);
	PIN_SET(RXD_DOCK        , PIN_GPH3);
	PIN_SET(TXD_GPS         , PIN_GPH6);
	PIN_SET(RXD_GPS         , PIN_GPH7);
	PIN_SET(TXD_BT          , PIN_GPH4);
	PIN_SET(RXD_BT          , PIN_GPH5);
	PIN_SET(RTS_BT          , PIN_GPG9);
	PIN_SET(CTS_BT          , PIN_GPG10);
	PIN_SET(LCD_VCC_PWREN   , PIN_GPG4 | PIN_INVERTED);
	PIN_SET(LCD_BIAS_PWREN  , PIN_GPC7);
	PIN_SET(BACKLIGHT_EN    , PIN_GPB1 | PIN_INVERTED);
	PIN_SET(BACKLIGHT_PWM   , PIN_GPB0);
	PIN_SET(LCD_CS          , PIN_GPG3 | PIN_INVERTED);
	PIN_SET(LCD_SCL         , PIN_GPG7);
	PIN_SET(LCD_SDI         , PIN_GPG6);
	PIN_SET(LCD_RESET       , PIN_GPJ0 | PIN_INVERTED);
	PIN_SET(USB_HOST_DETECT , PIN_GPF1 | PIN_INVERTED);
	PIN_SET(USB_PULL_EN     , PIN_GPF7 | PIN_INVERTED);
	// USB_SUSPEND_OUT PIN_GPE12 and USB_HP PIN_GPE11 not defined
	// since USB charging is not supported
	PIN_SET(ON_OFF          , PIN_GPF0 | PIN_INVERTED);
	PIN_SET(ACPWR           , PIN_GPJ11 | PIN_INVERTED);
	PIN_SET(XMON            , PIN_GPG13 | PIN_OPEN_COLLECTOR);
	PIN_SET(XPON            , PIN_GPG12 | PIN_OPEN_EMITTER);
	PIN_SET(YMON            , PIN_GPG15 | PIN_OPEN_COLLECTOR);
	PIN_SET(YPON            , PIN_GPG14 | PIN_OPEN_EMITTER);
	PIN_SET(TSDOWN          , PIN_GPG15 | PIN_OPEN_COLLECTOR);
	PIN_SET(FACTORY_TEST_POINT , PIN_GPC0);

	// Powerboard
	PIN_SET(IGNITION         , PIN_GPF3 | PIN_INVERTED); // DOCK_GP1 is used as IGNITION
	PIN_SET(HEADPHONE_DETECT , PIN_GPF5);                // DOCK_GP3 is used as HEADPHONE_DETECT
	PIN_SET(AMP_ON           , PIN_GPF6);                // DOCK_GP4 is used as AMP_ON
}
