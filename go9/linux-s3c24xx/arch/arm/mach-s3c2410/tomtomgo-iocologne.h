#define SUPPORT_COLOGNE
inline void IO_InitCologne(void)
{
	VALUE_SET(id                 , GOTYPE_COLOGNE);
	VALUE_SET(caseid             , GOCASE_EDINBURGH);
	VALUE_SET(cputype            , GOCPU_S3C2412);
	VALUE_SET(btchip             , GOBT_BC4);
	VALUE_SET(btspeed            , 921600);
	VALUE_SET(btclock            , 16000000);
	VALUE_SET(btclass            , 0x001f00);
	VALUE_SET(lowbutton          , 1);
	VALUE_SET(batterytype	     , GOBATTERY_UNDEFINED);
	VALUE_SET(chargertype        , GOCHARGER_LTC3455);
	VALUE_SET(chargerresistor    , 1820);
	VALUE_SET(sdcard             , 1);
	VALUE_SET(sdslot             , 1);
	VALUE_SET(tfttype            , GOTFT_SAMSUNG_LTE246QV);
	VALUE_SET(backlighttype	     , GOBACKLIGHT_CH0_AT1312_350);
	VALUE_SET(gpstype            , GOGPS_SIRF3);
	VALUE_SET(codectype          , GOCODEC_WM8711);
	VALUE_SET(usbslavetype       , GOUSB_S3C24XX);
	VALUE_SET(usbdevicehostcapable,1);
	VALUE_SET(ohciports          , 0x01);
	VALUE_SET(compass            , 1);
	VALUE_SET(hw_i2c             , 1);
	VALUE_SET(mp3                , 1);
	VALUE_SET(codecmaster        , 1);
	VALUE_SET(unusedpinlevel     , 1);

	STRING_SET(name,"TomTom HIKE");
	STRING_SET(btname,"TomTom HIKE");
	STRING_SET(usbname,"HIKE");
	STRING_SET(familyname,"TomTom HIKE");
	STRING_SET(projectname,"COLOGNE");
	STRING_SET(requiredbootloader,"4.83");
	STRING_SET(dockdev,"ttySAC0");
	STRING_SET(btdev,"ttySAC1");
	STRING_SET(gpsdev,"ttySAC2");

	PIN_SET(SD_PWR_ON       , PIN_GPC6);
	PIN_SET(WP_SD           , PIN_GPH8);
	PIN_SET(CD_SD           , PIN_GPF2 | PIN_INVERTED);
	PIN_SET(GPS_RESET       , PIN_GPC5 | PIN_INVERTED);
	PIN_SET(GPS_ON          , PIN_GPF4);
	PIN_SET(GPS_1PPS        , PIN_GPG0);
	PIN_SET(GPS_REPRO       , PIN_GPG1);
	PIN_SET(BATT_TEMP_OVER  , PIN_GPF5 | PIN_INVERTED);
	PIN_SET(CHARGING        , PIN_GPG4 | PIN_INVERTED);
	PIN_SET(AIN4_PWR        , PIN_GPF3);
	PIN_SET(DAC_PWR_ON      , PIN_GPE13);
	PIN_SET(I2SSDO          , PIN_GPE4); 
	PIN_SET(CDCLK           , PIN_GPE2); 
	PIN_SET(I2SSCLK         , PIN_GPE1); 
	PIN_SET(I2SLRCK         , PIN_GPE0); 
	PIN_SET(L3CLOCK         , PIN_GPB4); 
	PIN_SET(L3MODE          , PIN_GPB3); 
	PIN_SET(L3DATA          , PIN_GPB1); 
	PIN_SET(TXD_DOCK        , PIN_GPH2);
	PIN_SET(RXD_DOCK        , PIN_GPH3);
	PIN_SET(TXD_GPS         , PIN_GPH6);
	PIN_SET(RXD_GPS         , PIN_GPH7);
	PIN_SET(TXD_BT          , PIN_GPH4);
	PIN_SET(RXD_BT          , PIN_GPH5);
	PIN_SET(RTS_BT          , PIN_GPG9);
	PIN_SET(CTS_BT          , PIN_GPG10);
	/* WARNING: BT_RESET is GPS_ON inverted!!! */
//	PIN_SET(BT_RESET           , PIN_GPF4 | PIN_INVERTED);
	PIN_SET(BT_RESET_GPS_OFF   , PIN_GPF4);
	PIN_SET(LCD_VCC_PWREN   , PIN_GPB9 | PIN_INVERTED);
	PIN_SET(BACKLIGHT_PWM   , PIN_GPB0);
	PIN_SET(LCD_CS          , PIN_GPG3 | PIN_INVERTED);
	PIN_SET(LCD_SCL         , PIN_GPG7);
	PIN_SET(LCD_SDI         , PIN_GPG6);
	PIN_SET(LCD_SDO         , PIN_GPG5);
	PIN_SET(LCD_RESET       , PIN_GPC7 | PIN_INVERTED);
	PIN_SET(USB_HOST_DETECT , PIN_GPF1 | PIN_INVERTED);
	PIN_SET(USB_PULL_EN     , PIN_GPF7);
	PIN_SET(USB_SUSPEND_OUT , PIN_GPE12);
	PIN_SET(USB_HP          , PIN_GPE11);
	PIN_SET(USB_PWR_BYPASS  , PIN_GPG8);
	PIN_SET(PWR_RST         , PIN_GPE3);
	PIN_SET(ON_OFF          , PIN_GPF0 | PIN_INVERTED);
	PIN_SET(ACPWR           , PIN_GPF1 | PIN_INVERTED);
	PIN_SET(IGNITION        , PIN_GPF1 | PIN_INVERTED);

	PIN_SET(AIN_DOCK_EINT   , PIN_GPG11);
	PIN_SET(MEP_DAT         , PIN_GPG12);
	PIN_SET(MEP_ACK         , PIN_GPG13);
	PIN_SET(TOUCHPAD_SW     , PIN_GPG14 | PIN_INVERTED);
	PIN_SET(MEP_CLK         , PIN_GPG15);
	PIN_SET(BUZZER_EN       , PIN_GPB2);
#ifdef COLOGNE_PCB_REV10
	PIN_SET(CAM_DPWDN       , PIN_GPA18);
	PIN_SET(DOCK_PWREN      , PIN_GPA17);
#else
	PIN_SET(CAM_DPWDN       , PIN_GPC8);
	PIN_SET(DOCK2_PWREN     , PIN_GPC9);
#endif
	
	PIN_SET(HW_IIC_SDA      , PIN_GPE15);
	PIN_SET(HW_IIC_SCL      , PIN_GPE14);

	PIN_SET(CAMRESET        , PIN_GPJ12_2413);
	PIN_SET(CAMCLKOUT       , PIN_GPJ11_2413);
	PIN_SET(CAMPCLK         , PIN_GPJ8_2413);
	PIN_SET(CAMVSYNC        , PIN_GPJ9_2413);
	PIN_SET(CAMHREF         , PIN_GPJ10_2413);
	PIN_SET(CAMDATA0        , PIN_GPJ0_2413);
	PIN_SET(CAMDATA1        , PIN_GPJ1_2413);
	PIN_SET(CAMDATA2        , PIN_GPJ2_2413);
	PIN_SET(CAMDATA3        , PIN_GPJ3_2413);
	PIN_SET(CAMDATA4        , PIN_GPJ4_2413);
	PIN_SET(CAMDATA5        , PIN_GPJ5_2413);
	PIN_SET(CAMDATA6        , PIN_GPJ6_2413);
	PIN_SET(CAMDATA7        , PIN_GPJ7_2413);

	PIN_SET(FACTORY_TEST_POINT , PIN_GPG2);

#ifdef TS_ON_COLOGNE
	PIN_SET(XMON          , PIN_GPB10 | PIN_OPEN_COLLECTOR);
	PIN_SET(XPON          , PIN_GPF6 | PIN_OPEN_EMITTER);
	PIN_SET(YMON          , PIN_GPH0 | PIN_OPEN_COLLECTOR);
	PIN_SET(YPON          , PIN_GPH1 | PIN_OPEN_EMITTER);
	PIN_SET(TSDOWN        , PIN_GPH0 | PIN_OPEN_COLLECTOR);
	tsxchannel = 5;
	tsychannel = 7;
#endif
}
