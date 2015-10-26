#define SUPPORT_CAIRNS
inline void IO_InitCairns(void)
{
	VALUE_SET(id                 , GOTYPE_CAIRNS);
	VALUE_SET(caseid             , GOCASE_MILAN);
	VALUE_SET(cputype            , GOCPU_S3C2443);

	VALUE_SET(btchip             , GOBT_BC4);
	VALUE_SET(btspeed            , 921600);
	VALUE_SET(btclock            , 16000000);
	VALUE_SET(btclass            , 0x280408);
	VALUE_SET(handsfree          , 1);
	VALUE_SET(headsetgw          , 1);
	VALUE_SET(a2dp               , 1);
	VALUE_SET(leveled_on_off     , 1);

	VALUE_SET(batterytype	     , GOBATTERY_UNDEFINED);
	VALUE_SET(chargertype        , GOCHARGER_LTC3455);
	VALUE_SET(chargerresistor    , 1780);
	VALUE_SET(sdcard             , 1);
	VALUE_SET(sdslot             , 1);
	VALUE_SET(tfttype            , GOTFT_SHARP_LQ043T1);
	VALUE_SET(backlighttype	     , GOBACKLIGHT_FB_CH0_20K_CAT3238TD_430);

	VALUE_SET(gpstype            , GOGPS_SIRF3);
	VALUE_SET(codectype          , GOCODEC_WM8971);
	VALUE_SET(codecmaster        , GOCODECCFG_SLAVE);
	VALUE_SET(usbslavetype       , GOUSB_S3C2443);
	VALUE_SET(ohciports          , 0x01);
	VALUE_SET(usbdevicehostcapable,1);
	VALUE_SET(regulatorforcodec  , 1);
	VALUE_SET(keeprtcpowered     , 1);
	VALUE_SET(canrecordaudio     , 1);
	VALUE_SET(hw_i2c             , 1);
	VALUE_SET(pnp                , 1);
	VALUE_SET(codecamptype       , GOCODECAMP_DIFFERENTIAL_HW);
	VALUE_SET(aectype            , GOAEC_FM);
	VALUE_SET(loquendo           , 1);
	VALUE_SET(gpsephemeris       , 1);

	VALUE_SET(tsxchannel         , 5);
	VALUE_SET(tsychannel         , 9);

	STRING_SET(familyname,"TomTom GO");
	STRING_SET(projectname,"CAIRNS");
	STRING_SET(requiredbootloader,"5.07");
	STRING_SET(btdev,"ttySAC1");
	STRING_SET(gpsdev,"ttySAC2");
	STRING_SET(dockdev,"ttySAC3");

	PIN_SET(SD_PWR_ON           , PIN_GPJ13);
	PIN_SET(WP_SD               , PIN_GPF7);
	PIN_SET(CD_SD               , PIN_GPF1 | PIN_INVERTED);
	PIN_SET(GPS_RESET           , PIN_GPE13 | PIN_INVERTED);
	PIN_SET(GPS_ON              , PIN_GPE12);
	PIN_SET(GPS_1PPS            , PIN_GPG0);
	PIN_SET(GPS_REPRO           , PIN_GPE11);
	PIN_SET(AIN4_PWR            , PIN_GPA11);
	PIN_SET(DAC_PWR_ON          , PIN_GPJ15);
	PIN_SET(I2SSDO              , PIN_GPE4);
	PIN_SET(I2SSDI              , PIN_GPE3);
	PIN_SET(CDCLK               , PIN_GPE2);
	PIN_SET(CDCLK_12MHZ         , PIN_GPH13);
	PIN_SET(I2SSCLK             , PIN_GPE1);
	PIN_SET(I2SLRCK             , PIN_GPE0);
	PIN_SET(L3CLOCK             , PIN_GPB4);
	PIN_SET(L3MODE              , PIN_GPB3);
	PIN_SET(L3DATA              , PIN_GPB2);
	PIN_SET(TXD_GPS             , PIN_GPH4);/* 1. Used as in Milan code */
	PIN_SET(RXD_GPS             , PIN_GPH5);/* 1. Used as in Milan code */
	PIN_SET(TXD_BT              , PIN_GPH2);/* 2. Used as in Milan code */
	PIN_SET(RXD_BT              , PIN_GPH3);/* 2. Used as in Milan code */
	PIN_SET(RTS_BT              , PIN_GPH11);/* 3. Used as in Milan code */
	PIN_SET(CTS_BT              , PIN_GPH10);/* 3. Used as in Milan code */
	PIN_SET(BT_RESET            , PIN_GPG9 | PIN_INVERTED);
	PIN_SET(BT_MODE             , PIN_GPB1);
	PIN_SET(LCD_VCC_PWREN       , PIN_GPC6 | PIN_INVERTED);
	PIN_SET(LCD_RESET           , PIN_GPC0 | PIN_INVERTED);/* Called DISP_ON in GPIO pin document*/
	PIN_SET(LCD_ID              , PIN_GPC7); /* Called LCM_ID in GPIO pin document */
	PIN_SET(BACKLIGHT_PWM       , PIN_GPB0);
	PIN_SET(ON_OFF              , PIN_GPF0 | PIN_INVERTED);
	PIN_SET(USB_PHY_PWR_EN      , PIN_GPG8);
	PIN_SET(PWR_MODE            , PIN_GPG6);
	PIN_SET(PWR_RST             , PIN_GPA12);
	PIN_SET(HW_IIC_SDA          , PIN_GPE15);
	PIN_SET(HW_IIC_SCL          , PIN_GPE14);
	PIN_SET(FACTORY_TEST_POINT  , PIN_GPL14); /* Called PRO_CON in GPIO pin doc */
	PIN_SET(XMON                , PIN_GPG13 | PIN_OPEN_COLLECTOR);/* 4. Used as in Milan code */
	PIN_SET(XPON                , PIN_GPG12 | PIN_OPEN_EMITTER);/* 4. Used as in Milan code */
	PIN_SET(YMON                , PIN_GPG15 | PIN_OPEN_COLLECTOR);/* 5. Used as in Milan code */
	PIN_SET(YPON                , PIN_GPG14 | PIN_OPEN_EMITTER);/* 5. Used as in Milan code */
	PIN_SET(TSDOWN              , PIN_GPG15 | PIN_OPEN_COLLECTOR);/* Didn't find this one in the GPIO pin doc. Milan code has this one doubly defined with YMON (iPIN_GPG15) */
	PIN_SET(TYPE_SUB_ID0        , PIN_GPB7);
	PIN_SET(TYPE_SUB_ID1        , PIN_GPB8);

  /* New additions, compared to Milan */
	PIN_SET(RADIO_RX            , PIN_GPH6);/* 0. Switched in Milan code (TXD3)*/
	PIN_SET(RADIO_TX            , PIN_GPH7);/* 0. Switched in Milan code (RXD3)*/
	PIN_SET(PWR_5V_ON           , PIN_GPG4);
	PIN_SET(GPS_ANT_OPEN        , PIN_GPG5);
	PIN_SET(GPS_ANT_SHORT       , PIN_GPG1);
	PIN_SET(RDS_SEN             , PIN_GPB9 | PIN_INVERTED);
	PIN_SET(EN_RDS_CLK          , PIN_GPL8 | PIN_INVERTED);
	PIN_SET(RDS_PWR_ON          , PIN_GPL9);
	PIN_SET(USB_PWR_ERROR       , PIN_GPG2 | PIN_INVERTED);
	PIN_SET(NAVI_MUTE           , PIN_GPL11);
	PIN_SET(GPIO_DIAG0          , PIN_GPL2);
	PIN_SET(GPIO_DIAG1          , PIN_GPL3);
	PIN_SET(GPIO_DIAG2          , PIN_GPL4);
	PIN_SET(GPIO_DIAG_EN        , PIN_GPL5);
	PIN_SET(BACKLIGHT_EN        , PIN_GPL6);
	PIN_SET(FAN_EN              , PIN_GPL7);
}

void IO_InitCairns_C1(void)
{
	STRING_SET(name,"TomTom GO Cairns");
	STRING_SET(btname,"TomTom GO Cairns");
	STRING_SET(usbname,"GO Cairns");
	/*VALUE_SET(deadreckoning		, 1);*/
}
