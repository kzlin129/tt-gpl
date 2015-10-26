#define SUPPORT_LIVORNO
static backlight_mapping_t LivornoMapTable = {17,22,26,31,34,36,38,40,42,45,47,
	                49,52,54,57,59,62,64,67,70,72};
inline void IO_InitLivorno(void)
{
	VALUE_SET(id                 , GOTYPE_LIVORNO);//New for Livorno
	VALUE_SET(caseid             , GOCASE_ROME);//Same as Rome
	VALUE_SET(cputype            , GOCPU_S3C2412);//Same as Rome
	VALUE_SET(batterytype	     , GOBATTERY_1100);
	VALUE_SET(chargertype        , GOCHARGER_LTC3455);//Same as Rome
	VALUE_SET(chargerresistor    , 2260);//Same as Rome
	VALUE_SET(sdcard             , 1);//Same as Rome
	VALUE_SET(sdslot             , 0);//Same as Rome
	VALUE_SET(internalflash      , 1);//Same as Rome
	VALUE_SET(sdisharedbus       , 0);//Same as Rome
	VALUE_SET(movinandsoftpoweron, 1);//Same as Rome
	VALUE_SET(tfttype            , GOTFT_SAMSUNG_LMS430HF12);//Same as Rome
	VALUE_SET(backlighttype	     , GOBACKLIGHT_FB_CH1_CAT4238TD_430); /* as Trapani */
	VALUE_SET(backlighttimer     , 1);

	
	VALUE_SET(tsxchannel         , 5);
	VALUE_SET(tsychannel         , 7);
	VALUE_SET(gpstype            , GOGPS_GL_BCM4750);//Barracuda
	VALUE_SET(gldetected         , 1);
	VALUE_SET(codectype          , GOCODEC_WM8711);//Same as Rome
	VALUE_SET(usbslavetype       , GOUSB_S3C24XX);//Same as Rome
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
	VALUE_SET(backlight_mapping  , &LivornoMapTable);
	
	STRING_SET(name,"TomTom XL IQ Routes Edition");
	STRING_SET(usbname,"XL IQ Routes Edition");
	STRING_SET(familyname,"TomTom ONE");
	STRING_SET(projectname,"Livorno");
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
	PIN_SET(GPS_RESET          , PIN_GPC5 | PIN_INVERTED);//Same as Rome
	PIN_SET(CHARGING           , PIN_GPC7 | PIN_INVERTED);//Same as Rome

// D Group

// E Group
	PIN_SET(I2SLRCK            , PIN_GPE0);//Same as Rome 
	PIN_SET(I2SSCLK            , PIN_GPE1);//Same as Rome 
	PIN_SET(CDCLK              , PIN_GPE2);//Same as Rome 
	PIN_SET(PWR_RST            , PIN_GPE3);//Same as Rome 
	PIN_SET(I2SSDO             , PIN_GPE4);//Same as Rome  
	PIN_SET(USB_HP             , PIN_GPE11);//Same as Rome  
	PIN_SET(USB_SUSPEND_OUT    , PIN_GPE12);//Same as Rome
	PIN_SET(HW_IIC_SCL         , PIN_GPE14);//Same as Rome
	PIN_SET(HW_IIC_SDA         , PIN_GPE15);//Same as Rome

// F Group
	PIN_SET(ON_OFF             , PIN_GPF0 | PIN_INVERTED);//Same as Rome
	PIN_SET(USB_HOST_DETECT    , PIN_GPF1 | PIN_INVERTED);//Same as Rome 	  /* shared with ACPWR and IGNITION */
	PIN_SET(ACPWR              , PIN_GPF1 | PIN_INVERTED);//Same as Rome      /* shared with USB_HOST_DETECT and IGNITION */
	PIN_SET(IGNITION           , PIN_GPF1 | PIN_INVERTED);//Same as Rome      /* shared with ACPWR and USB_HOST_DETECT */
	//PIN_SET(GPS_POWERON_OUT    , PIN_GPF3);//New for Livorno
	PIN_SET(GPS_STANDBY        , PIN_GPF4| PIN_INVERTED);//New for Livorno
	PIN_SET(LCD_RESET          , PIN_GPF5 | PIN_INVERTED);//Same as Rome
	PIN_SET(AMP_ON             , PIN_GPF6);//Same as Rome
	PIN_SET(USB_PULL_EN        , PIN_GPF7);//Same as Rome

// G Group
	//PIN_SET(LINEIN_DETECT      , PIN_GPG0 | PIN_INVERTED);		
	PIN_SET(LCD_ID             , PIN_GPG0);//Same as Rome

	PIN_SET(LCD_CS             , PIN_GPG3 | PIN_INVERTED);//Same as Rome
	PIN_SET(LCD_VCC_PWREN      , PIN_GPG4 | PIN_INVERTED);//Same as Rome
	PIN_SET(BATT_TEMP_OVER     , PIN_GPG5 | PIN_INVERTED);//Same as Rome
	PIN_SET(LCD_SDI            , PIN_GPG6);//Same as Rome
	PIN_SET(LCD_SCL            , PIN_GPG7);//Same as Rome
	PIN_SET(USB_PWR_BYPASS     , PIN_GPG8);// set to high on CLA detect
	PIN_SET(CTS_GPS            , PIN_GPG9);//Same as Rome
	PIN_SET(RTS_GPS            , PIN_GPG10);//Same as Rome
	PIN_SET(XPON               , PIN_GPG12 | PIN_OPEN_EMITTER);//Same as Rome
	PIN_SET(XMON               , PIN_GPG13 | PIN_OPEN_COLLECTOR);//Same as Rome
	PIN_SET(YPON               , PIN_GPG14 | PIN_OPEN_EMITTER);//Same as Rome
	PIN_SET(YMON               , PIN_GPG15 | PIN_OPEN_COLLECTOR);//Same as Rome
	PIN_SET(TSDOWN             , PIN_GPG15 | PIN_OPEN_COLLECTOR);//Same as Rome		

// H Group
	PIN_SET(MOVI_PWR_ON        , PIN_GPH0 | PIN_INVERTED);//New for Livorno
	PIN_SET(TXD_DOCK           , PIN_GPH2);//New for Livorno
	PIN_SET(RXD_DOCK           , PIN_GPH3);//New for Livorno
	PIN_SET(TXD_GPS            , PIN_GPH4);
	PIN_SET(RXD_GPS            , PIN_GPH5);
	PIN_SET(CDCLK_12MHZ        , PIN_GPH10); 	
}


