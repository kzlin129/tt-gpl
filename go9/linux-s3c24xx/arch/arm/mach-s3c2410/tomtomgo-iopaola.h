#define SUPPORT_PAOLA
static backlight_mapping_t PaolaMapTable = {17,22,26,31,34,36,38,40,42,45,47,
	                49,52,54,57,59,62,64,67,70,72};
inline void IO_InitPaola(void)
{
	VALUE_SET(id                 , GOTYPE_PAOLA);//New for Paola
	VALUE_SET(caseid             , GOCASE_PALERMO);//Same as Palermo
	VALUE_SET(cputype            , GOCPU_S3C2412);//Same as Palermo
	VALUE_SET(batterytype	     , GOBATTERY_920);
	VALUE_SET(chargertype        , GOCHARGER_LTC3455);//Same as Palermo
	VALUE_SET(chargerresistor    , 2490);//Same as Palermo
	VALUE_SET(sdcard             , 1);//Same as Palermo
	VALUE_SET(sdslot             , 0);//Same as Palermo
	VALUE_SET(internalflash      , 1);//Same as Palermo
	VALUE_SET(sdisharedbus       , 0);//Same as Palermo
	VALUE_SET(movinandsoftpoweron, 1);//Same as Palermo
	VALUE_SET(tfttype            , GOTFT_SAMSUNG_LMS350GF);//New for Paola
	VALUE_SET(backlighttype	     , GOBACKLIGHT_CH0_EUP2584_350); /* as Trapani */
	VALUE_SET(backlighttimer     , 1);

	
	VALUE_SET(tsxchannel         , 5);
	VALUE_SET(tsychannel         , 7);
	VALUE_SET(gpstype            , GOGPS_GL_BCM4750);//Barracuda
	VALUE_SET(gldetected         , 1);
	VALUE_SET(codectype          , GOCODEC_WM8711);//Same as Palermo
	VALUE_SET(usbslavetype       , GOUSB_S3C24XX);//Same as Palermo
	VALUE_SET(usbdevicehostcapable,1);
	VALUE_SET(ohciports          , 0x01);
	VALUE_SET(tftsoftstart       , 1);
	VALUE_SET(pnp                , 1);
	VALUE_SET(gpsephemeris       , 1);
	VALUE_SET(hw_i2c             , 1);
	VALUE_SET(loquendo           , 0 );
	VALUE_SET(picsecshutdown     , 1);
	//VALUE_SET(battvoltnum        , 2);
	//VALUE_SET(battvoltdenom      , 1);
	VALUE_SET(backlight_inverted , 1);
	VALUE_SET(backlightfreq      ,20000);//backlight frequence = 20kHz
	VALUE_SET(backlight_mapping  , &PaolaMapTable);
	
	STRING_SET(name,"TomTom ONE IQ Routes Edition");
	STRING_SET(usbname,"ONE IQ Routes Edition");
	STRING_SET(familyname,"TomTom ONE");
	STRING_SET(projectname,"Paola");
	STRING_SET(requiredbootloader,"5.4218");
	STRING_SET(gpsdev,"ttySAC1");//GPS use UART1
	STRING_SET(dockdev,"ttySAC0");//JTAG use UART0

// A Group
	PIN_SET(AIN4_PWR           , PIN_GPA12);//ADC reference voltage

// B Group
	PIN_SET(BACKLIGHT_PWM      , PIN_GPB1);//Backlight
	PIN_SET(BACKLIGHT_EN       , PIN_GPB0);//Backlight
	PIN_SET(L3DATA             , PIN_GPB2);//CODEC
	PIN_SET(L3MODE             , PIN_GPB3);//CODEC 
	PIN_SET(L3CLOCK            , PIN_GPB4);//CODEC 
// C Group
	PIN_SET(GPS_RESET          , PIN_GPC5 | PIN_INVERTED);//Same as Palermo
	PIN_SET(CHARGING           , PIN_GPC7 | PIN_INVERTED);//Same as Palermo

// D Group

// E Group
	PIN_SET(I2SLRCK            , PIN_GPE0);//Same as Palermo 
	PIN_SET(I2SSCLK            , PIN_GPE1);//Same as Palermo 
	PIN_SET(CDCLK              , PIN_GPE2);//Same as Palermo 
	PIN_SET(PWR_RST            , PIN_GPE3);//Same as Palermo 
	PIN_SET(I2SSDO             , PIN_GPE4);//Same as Palermo  
	PIN_SET(USB_HP             , PIN_GPE11);//Same as Palermo  
	PIN_SET(USB_SUSPEND_OUT    , PIN_GPE12);//Same as Palermo
	PIN_SET(HW_IIC_SCL         , PIN_GPE14);//Same as Palermo
	PIN_SET(HW_IIC_SDA         , PIN_GPE15);//Same as Palermo

// F Group
	PIN_SET(ON_OFF             , PIN_GPF0 | PIN_INVERTED);//Same as Palermo
	PIN_SET(USB_HOST_DETECT    , PIN_GPF1 | PIN_INVERTED);//Same as Palermo 	  /* shared with ACPWR and IGNITION */
	PIN_SET(ACPWR              , PIN_GPF1 | PIN_INVERTED);//Same as Palermo      /* shared with USB_HOST_DETECT and IGNITION */
	PIN_SET(IGNITION           , PIN_GPF1 | PIN_INVERTED);//Same as Palermo      /* shared with ACPWR and USB_HOST_DETECT */
	//PIN_SET(GPS_POWERON_OUT    , PIN_GPF3);//New for Paola
	PIN_SET(GPS_STANDBY        , PIN_GPF4| PIN_INVERTED);//New for Paola
	PIN_SET(LCD_RESET          , PIN_GPF5 | PIN_INVERTED);//Same as Palermo
	PIN_SET(AMP_ON             , PIN_GPF6);//Same as Palermo
	PIN_SET(USB_PULL_EN        , PIN_GPF7);//Same as Palermo

// G Group
	//PIN_SET(LINEIN_DETECT      , PIN_GPG0 | PIN_INVERTED);		
	PIN_SET(LCD_ID             , PIN_GPG0);//Same as Palermo

	PIN_SET(LCD_CS             , PIN_GPG3 | PIN_INVERTED);//Same as Palermo
	PIN_SET(LCD_VCC_PWREN      , PIN_GPG4 | PIN_INVERTED);//Same as Palermo
	PIN_SET(BATT_TEMP_OVER     , PIN_GPG5 | PIN_INVERTED);//Same as Palermo
	PIN_SET(LCD_SDI            , PIN_GPG6);//Same as Palermo
	PIN_SET(LCD_SCL            , PIN_GPG7);//Same as Palermo
	PIN_SET(USB_PWR_BYPASS     , PIN_GPG8);// set to high on CLA detect
	PIN_SET(CTS_GPS            , PIN_GPG9);//Same as Palermo
	PIN_SET(RTS_GPS            , PIN_GPG10);//Same as Palermo
	PIN_SET(XPON               , PIN_GPG12 | PIN_OPEN_EMITTER);//Same as Palermo
	PIN_SET(XMON               , PIN_GPG13 | PIN_OPEN_COLLECTOR);//Same as Palermo
	PIN_SET(YPON               , PIN_GPG14 | PIN_OPEN_EMITTER);//Same as Palermo
	PIN_SET(YMON               , PIN_GPG15 | PIN_OPEN_COLLECTOR);//Same as Palermo
	PIN_SET(TSDOWN             , PIN_GPG15 | PIN_OPEN_COLLECTOR);//Same as Palermo		

// H Group
	PIN_SET(MOVI_PWR_ON        , PIN_GPH0 | PIN_INVERTED);//New for Paola
	PIN_SET(TXD_DOCK           , PIN_GPH2);//New for Paola
	PIN_SET(RXD_DOCK           , PIN_GPH3);//New for Paola
	PIN_SET(TXD_GPS            , PIN_GPH4);
	PIN_SET(RXD_GPS            , PIN_GPH5);
	PIN_SET(CDCLK_12MHZ        , PIN_GPH10); 	
}


