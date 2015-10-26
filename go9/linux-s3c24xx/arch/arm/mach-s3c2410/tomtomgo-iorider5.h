#define SUPPORT_RIDER5

// #TODO - for the time being
static backlight_mapping_t rider5_bl_mapping = 
	{60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 52, 54, 57, 59, 62, 64, 67, 70, 72};

#ifdef __BOOTLOADER__
void IO_DetectFeature(void);
#endif

inline void IO_InitRider5(void)
{
	VALUE_SET(id                 , GOTYPE_RIDER5);
	VALUE_SET(caseid             , GOCASE_ROME);
	VALUE_SET(cputype            , GOCPU_S3C2450);
	VALUE_SET(btchip             , GOBT_CSR8811);
	VALUE_SET(btspeed            , 921600);
	VALUE_SET(btclock            , 26000000);
	VALUE_SET(btclass            , 0x280408);
	VALUE_SET(handsfree          , 1);
	VALUE_SET(headsetgw          , 1);
	VALUE_SET(a2dp               , 1);

	VALUE_SET(batterytype	     , GOBATTERY_VARTA_1460_100);
	VALUE_SET(chargertype        , GOCHARGER_LTC3577);
	VALUE_SET(adc_ref_value      , 3318); // 0.8V x (1 + 1.02M / 324K) = 3.3185V = 3318.5mV
	VALUE_SET(chargerresistor    , 953);
	VALUE_SET(internalflash      , 1);	
#ifdef __BOOTLOADER__
	VALUE_SET(hsmmcinterface     , 1);
	VALUE_SET(hsmmc_4bit         , 1);
	VALUE_SET(hs_sdcard          , 1);
#else
	VALUE_SET(hsmmcinterface0_4bit , 1);
	VALUE_SET(hsmmcclocktype     , GOHSMMC_USB48CLK);
#endif
#if 0
	VALUE_SET(speaker, 0);
#endif
#ifdef __BOOTLOADER__
	VALUE_SET(movi_hsmmc_port    , 0);
	VALUE_SET(alt_lcd_controller , 1); /* use alt. lcd controller */
#endif
	VALUE_SET(tfttype            , GOTFT_SAMSUNG_LMS430HF29);
	VALUE_SET(needsvcclcmforiddetect,0);
	VALUE_SET(backlighttype	     , GOBACKLIGHT_LTC3577_CC_MODE);
	VALUE_SET(backlight_mapping  , &rider5_bl_mapping);

	VALUE_SET(gpstype            , GOGPS_ATH_AR1520);
	VALUE_SET(codectype          , GOCODEC_ALC5628);
	VALUE_SET(codecmaster        , GOCODECCFG_SLAVE);
	VALUE_SET(usbslavetype       , GOUSB_S3C2443);
	VALUE_SET(ohciports          , 0x01);
	VALUE_SET(dualusbphyctrl     , 1); 
	VALUE_SET(usbdevicehostcapable,1);
	VALUE_SET(regulatorforcodec  , 1);
	VALUE_SET(keeprtcpowered     , 1);
	VALUE_SET(canrecordaudio     , 1);
	VALUE_SET(hw_i2c             , 1);
	VALUE_SET(pnp                , 1);
	VALUE_SET(loquendo           , 0);
	VALUE_SET(gpsephemeris       , 1);
#ifdef __BOOTLOADER__
	VALUE_SET(altchargechannel   , 5);
#else
	VALUE_SET(batchargecurrentchannel, 5);
#endif
#if !defined ( __BOOTLOADER__ )
        VALUE_SET(battvoltnum        , 7);
        VALUE_SET(battvoltdenom      , 5);
#endif		
	VALUE_SET(tsxchannel         , 7);
	VALUE_SET(tsychannel         , 9);
	VALUE_SET(tsdownchannel      , 9);

	VALUE_SET(gldetected         , 0);

	STRING_SET(familyname,"TomTom Rider "); 
	STRING_SET(projectname,"Rider5");
	STRING_SET(requiredbootloader,"5.4247");
	STRING_SET(gpsdev,"ttySAC0");
	STRING_SET(btdev,"ttySAC1");
	STRING_SET(dockdev,"ttySAC3");

	STRING_SET(name,"TomTom Rider "); /* important: leave " \0" after GO for IO_GetModelVariant() checking in blit.c */
	STRING_SET(btname,"TomTom Rider");
	STRING_SET(usbname,"Rider");

	PIN_SET(GPS_RESET          , PIN_GPB0 | PIN_INVERTED);
//	PIN_SET(GPS_1PPS           , PIN_GPG0);
//	PIN_SET(GPS_STANDBY        , PIN_GPH13 | PIN_INVERTED);
	PIN_SET(I2SSDO             , PIN_GPE4);
	PIN_SET(CDCLK              , PIN_GPE2);
	PIN_SET(I2SSCLK            , PIN_GPE1);
	PIN_SET(I2SLRCK            , PIN_GPE0);
//	PIN_SET(I2SSDI             , PIN_GPE3);
  //dock to be rename and revised
	PIN_SET(DOCK_MOTOR_SENSE         , PIN_GPF6 | PIN_INVERTED);
	PIN_SET(DOCK_SENSE        , PIN_GPF3 |PIN_INVERTED);

	PIN_SET(TXD_DOCK           , PIN_GPH6);
	PIN_SET(RXD_DOCK           , PIN_GPH7);
	PIN_SET(RXD_DOCK_INT       , PIN_GPG2);

	PIN_SET(TXD_GPS            , PIN_GPH0);
	PIN_SET(RXD_GPS            , PIN_GPH1);		

	PIN_SET(TXD_BT             , PIN_GPH2);
	PIN_SET(RXD_BT             , PIN_GPH3);
	PIN_SET(CTS_BT             , PIN_GPH10);
	PIN_SET(RTS_BT             , PIN_GPH11);	
	PIN_SET(BT_RESET           , PIN_GPF5 | PIN_INVERTED);

	PIN_SET(LCD_VCC_PWREN      , PIN_GPA19| PIN_INVERTED);
	PIN_SET(LCD_RESET          , PIN_GPF4 | PIN_INVERTED);
	PIN_SET(LCD_ID             , PIN_GPB9);
	PIN_SET(LCD_CS             , PIN_GPC0 | PIN_INVERTED);
	PIN_SET(LCD_SCL            , PIN_GPE13);
	PIN_SET(LCD_SDI            , PIN_GPE12);		

	PIN_SET(ON_OFF             , PIN_GPF0 | PIN_INVERTED);
	PIN_SET(ACPWR              , PIN_GPF1 | PIN_INVERTED);
	PIN_SET(IGNITION           , PIN_GPF1 | PIN_INVERTED);
	PIN_SET(USB_HOST_DETECT    , PIN_GPF2 | PIN_INVERTED);
	PIN_SET(CHARGING           , PIN_GPG1 | PIN_INVERTED);
	PIN_SET(USB_PWR_BYPASS     , PIN_GPG4);

	PIN_SET(USB_PHY_1V2_PWR_EN , PIN_GPA9);
	PIN_SET(USB_PHY_PWR_EN	   , PIN_GPA8);
	PIN_SET(PWR_RST            , PIN_GPG5 | PIN_INVERTED);	// Suicide is done by pull-down LTC3577 PWR_ON

	PIN_SET(HW_IIC_SDA         , PIN_GPE15);
	PIN_SET(HW_IIC_SCL         , PIN_GPE14);
#ifdef __BOOTLOADER__
	PIN_SET(SW_SCL             , PIN_GPE14); 	
	PIN_SET(SW_SDA             , PIN_GPE15); 		
#endif

	PIN_SET(I2C_SWPWR          , PIN_GPL0);


}
