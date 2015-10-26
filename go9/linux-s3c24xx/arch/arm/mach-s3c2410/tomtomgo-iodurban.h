#define SUPPORT_DURBAN

static backlight_mapping_t durban_bl_mapping = 
	{60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 52, 54, 57, 59, 62, 64, 67, 70, 72};
//	{17, 22, 27, 32, 34, 36, 38, 40, 42, 45, 47, 49, 52, 54, 57, 59, 62, 64, 67, 70, 72};

#ifdef __BOOTLOADER__
void IO_DetectFeature(void);
#endif

inline void IO_InitDurban(void)
{
	VALUE_SET(id                 , GOTYPE_DURBAN);
	VALUE_SET(caseid             , GOCASE_CAGLIARI);
	VALUE_SET(cputype            , GOCPU_S3C2450);

	VALUE_SET(btchip             , GOBT_BC4);
	VALUE_SET(btspeed            , 921600);
	VALUE_SET(btclock            , 26000000);
	VALUE_SET(btclass            , 0x280408);
	VALUE_SET(handsfree          , 1);
	VALUE_SET(headsetgw          , 1);
	VALUE_SET(a2dp               , 1);

	VALUE_SET(batterytype	     , GOBATTERY_VARTA_1460_100);
	VALUE_SET(chargertype        , GOCHARGER_LTC3555);
	VALUE_SET(adc_ref_value		 , 3220); /* GOCHARGER_LTC3555 */
	VALUE_SET(chargerresistor    , 2000); /* 1430 for Cagliari */
	VALUE_SET(hs_sdcard          , 1); /* high speed sd controller */
	VALUE_SET(sdslot             , 1); /* for the time being */
	VALUE_SET(internalflash      , 1);
	VALUE_SET(hsmmcinterface     , 1);
	VALUE_SET(hsmovinandsoftpoweron, 1);
	VALUE_SET(tfttype            , GOTFT_SAMSUNG_LMS430HF12);
	VALUE_SET(backlightfreq      , 20000);
	VALUE_SET(backlighttype	     , GOBACKLIGHT_FB_CH0_20K_CAT3238TD_430);

	VALUE_SET(gpstype            , GOGPS_UNDEFINED);
	VALUE_SET(fmtransmittertype  , GOFM_SI4711);
	VALUE_SET(codectype          , GOCODEC_WM8750);
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
	VALUE_SET(backlight_inverted , 1);
	VALUE_SET(backlight_mapping  , &durban_bl_mapping);
	VALUE_SET(lightsensor        , 1);
	VALUE_SET(tsxchannel         , 5);
	VALUE_SET(tsychannel         , 9);
	VALUE_SET(tsdownchannel      , 9);
	VALUE_SET(gyroxchannel       , 1);
	VALUE_SET(gyroychannel       , 3);

	VALUE_SET(gldetected         , 1);

	STRING_SET(familyname,"TomTom GO "); 
	STRING_SET(projectname,"Durban");
	STRING_SET(requiredbootloader,"5.4200");
	STRING_SET(gpsdev,"ttySAC0");
	STRING_SET(gprsmodemdev,"ttySAC1");
	STRING_SET(btdev,"ttySAC2");
	STRING_SET(dockdev,"ttySAC3");

	STRING_SET(name,"TomTom GO"); /* important: leave " \0" after GO for IO_GetModelVariant() checking in blit.c */
	STRING_SET(btname,"TomTom GO");
	STRING_SET(usbname,"GO");

	VALUE_SET(deadreckoning	     , 1);
	VALUE_SET(acctype	     , GOACC_KXR94);
	VALUE_SET(barotype	     , GOBARO_SCP1000);

	PIN_SET(SD_PWR_ON          , PIN_GPJ11);
	PIN_SET(HS_MOVI_PWR_ON     , PIN_GPJ14 | PIN_INVERTED);
	PIN_SET(CD_SD              , PIN_GPF1 | PIN_INVERTED);
	PIN_SET(GPS_RESET          , PIN_GPJ7 | PIN_INVERTED);
	PIN_SET(GPS_1PPS           , PIN_GPG0);
	PIN_SET(GPS_STANDBY        , PIN_GPJ8 | PIN_INVERTED);
	PIN_SET(AIN4_PWR           , PIN_GPL14);
	PIN_SET(SPICLK             , PIN_GPL10);
	PIN_SET(SPIMSI             , PIN_GPL11);
	PIN_SET(SPIMSO             , PIN_GPL12);
	PIN_SET(AMP_ON             , PIN_GPH0);
	PIN_SET(DAC_PWR_ON         , PIN_GPJ15);
	PIN_SET(I2SSDO             , PIN_GPE4);
	PIN_SET(CDCLK              , PIN_GPE2);
	PIN_SET(I2SSCLK            , PIN_GPE1);
	PIN_SET(I2SLRCK            , PIN_GPE0);
	PIN_SET(I2SSDI             , PIN_GPE3);
	PIN_SET(L3CLOCK            , PIN_GPB4);
	PIN_SET(L3MODE             , PIN_GPB3);
	PIN_SET(L3DATA             , PIN_GPB2);
	PIN_SET(DOCK_PWREN         , PIN_GPJ12);
	PIN_SET(DOCK_DET_PWREN     , PIN_GPJ6);
	PIN_SET(DOCK_I2CEN         , PIN_GPJ1 | PIN_INVERTED);
	PIN_SET(DOCK_SENSE         , PIN_GPF4);
	PIN_SET(DOCK_SENSE1        , PIN_GPF3);
//	PIN_SET(DOCK_RESET         , PIN_GPE13 | PIN_INVERTED); /* PIN_GPG4 */
	PIN_SET(TXD_DOCK           , PIN_GPH6);
	PIN_SET(RXD_DOCK           , PIN_GPH7);
	PIN_SET(RXD_DOCK_INT       , PIN_GPG11); /* PIN_GPG2 */

//	PIN_SET(TXD_GPS            , PIN_GPH0);
//	PIN_SET(RXD_GPS            , PIN_GPH1);
//	PIN_SET(RTS_GPS            , PIN_GPH9);
//	PIN_SET(CTS_GPS            , PIN_GPH8);

	PIN_SET(TXD_BT             , PIN_GPH4);
	PIN_SET(RXD_BT             , PIN_GPH5);
	PIN_SET(BT_RESET           , PIN_GPJ9 | PIN_INVERTED);

	PIN_SET(LCD_VCC_PWREN      , PIN_GPE12| PIN_INVERTED);
	PIN_SET(LCD_ID             , PIN_GPE11| PIN_PULL_DOWN);
	PIN_SET(LCD_RESET          , PIN_GPC0 | PIN_INVERTED);
	PIN_SET(BACKLIGHT_PWM      , PIN_GPB0); 
	PIN_SET(BACKLIGHT_EN       , PIN_GPB1);
	PIN_SET(ON_OFF             , PIN_GPF0 | PIN_INVERTED);
	PIN_SET(ACPWR              , PIN_GPF2 | PIN_INVERTED);
        PIN_SET(IGNITION           , PIN_GPF2 | PIN_INVERTED);

	PIN_SET(HEADPHONE_DETECT   , PIN_GPG3 | PIN_INVERTED);
	PIN_SET(USB_HOST_DETECT    , PIN_GPF2 | PIN_INVERTED);
	PIN_SET(CHARGING	   	   , PIN_GPG8 | PIN_INVERTED); /* PIN_GPG1 */
	PIN_SET(USB_PWR_BYPASS     , PIN_GPH9); /* PIN_GPG5 */
	PIN_SET(USB_PHY_PWR_EN     , PIN_GPL13); /* was PIN_GPG8 */
	PIN_SET(PWR_RST            , PIN_GPJ10);
	PIN_SET(HW_IIC_SDA         , PIN_GPE15);
	PIN_SET(HW_IIC_SCL         , PIN_GPE14);
//	PIN_SET(I2C_SWPWR          , PIN_GPH8); bootloader
	PIN_SET(I2C_SWPWR          , PIN_GPH8 | PIN_PULL_UP);

	PIN_SET(ACC_SPI_CSB        , PIN_GPJ4 | PIN_INVERTED);
	PIN_SET(BARO_SPI_CSB       , PIN_GPG9 | PIN_INVERTED);
	PIN_SET(BARO_SPI_IRQ	   , PIN_GPF5);
	PIN_SET(LOW_DC_VCC         , PIN_GPF6 | PIN_INVERTED);
	PIN_SET(EN_DR_PWR          , PIN_GPJ0);
	PIN_SET(DRDY_INT           , PIN_GPF5);

	PIN_SET(FM_RST             , PIN_GPJ5 | PIN_INVERTED);
	PIN_SET(FM_RDS_INT         , PIN_GPG10);
	PIN_SET(FM_TX_INT	   , PIN_GPF7);

	PIN_SET(GSM_RXD            , PIN_GPH3 /*| PIN_PULL_DOWN*/);
	PIN_SET(GSM_TXD            , PIN_GPH2 /*| PIN_PULL_DOWN*/);
	PIN_SET(GSM_RTS            , PIN_GPH11);
	PIN_SET(GSM_CTS            , PIN_GPH10);
	PIN_SET(GSM_SYS_RST        , PIN_GPJ2);
        PIN_SET(GSM_SYS_EN         , PIN_GPJ3);

	PIN_SET(XPON               , PIN_GPG12 | PIN_OPEN_EMITTER);
	PIN_SET(XMON               , PIN_GPG13 | PIN_OPEN_COLLECTOR);
	PIN_SET(YPON               , PIN_GPG14 | PIN_OPEN_EMITTER);
	PIN_SET(YMON               , PIN_GPG15 | PIN_OPEN_COLLECTOR);
	PIN_SET(TSDOWN             , PIN_GPG15 | PIN_OPEN_COLLECTOR);

	PIN_SET(UART_CLK_OUT       , PIN_GPH13);
	PIN_SET(UART_CLK_IN	   , PIN_GPH12 | PIN_INPUT_ON_SUSPEND);
}

void IO_InitDurban540(int connected)
{
	if (connected) {
		STRING_SET(name,"TomTom GO 540 LIVE");
		STRING_SET(btname,"TomTom GO 540 LIVE");
		STRING_SET(usbname,"GO 540 LIVE");
	} else {
		STRING_SET(name,"TomTom GO 540");
		STRING_SET(btname,"TomTom GO 540");
		STRING_SET(usbname,"GO 540");
	}

	VALUE_SET(deadreckoning	     , 0);
}

void IO_InitDurban740(int connected)
{
	if (connected) {
		STRING_SET(name,"TomTom GO 740 LIVE");
		STRING_SET(btname,"TomTom GO 740 LIVE");
		STRING_SET(usbname,"GO 740 LIVE");
	} else {
		STRING_SET(name,"TomTom GO 740");
		STRING_SET(btname,"TomTom GO 740");
		STRING_SET(usbname,"GO 740");
	}

	VALUE_SET(deadreckoning	     , 1);
	VALUE_SET(acctype	     , GOACC_KXR94);
	VALUE_SET(barotype	     , GOBARO_SCP1000);
}

void IO_InitDurban940(int connected)
{
	if (connected) {
		STRING_SET(name,"TomTom GO 940 LIVE");
		STRING_SET(btname,"TomTom GO 940 LIVE");
		STRING_SET(usbname,"GO 940 LIVE");
	} else {
		STRING_SET(name,"TomTom GO 940");
		STRING_SET(btname,"TomTom GO 940");
		STRING_SET(usbname,"GO 940");
	}

	VALUE_SET(deadreckoning	     , 1);
	VALUE_SET(acctype	     , GOACC_KXR94);
	VALUE_SET(barotype	     , GOBARO_SCP1000);
}
