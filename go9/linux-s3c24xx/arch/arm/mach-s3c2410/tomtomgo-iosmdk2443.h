#define SUPPORT_SMDK2443

inline void IO_InitSMDK2443(void)
{
	VALUE_SET(id             , GOTYPE_SMDK2443);
	VALUE_SET(cputype        , GOCPU_S3C2443);
	VALUE_SET(sdcard         , 1);
	VALUE_SET(sdslot         , 1);
	VALUE_SET(tfttype        , GOTFT_SAMSUNG_LTV350);
	VALUE_SET(usbslavetype   , GOUSB_S3C24XX);
	VALUE_SET(hw_i2c         , 1);
	VALUE_SET(videodecoder   , GOVD_ADV7180);
	
	STRING_SET(name,"SMDK2443 Development Board");
	STRING_SET(usbname,"SMDK2443");
	STRING_SET(familyname,"SMDK2443");
	STRING_SET(dockdev,"ttySAC0");

	PIN_SET(CTS_DOCK        , PIN_GPH0);
	PIN_SET(RTS_DOCK        , PIN_GPH1);
	PIN_SET(TXD_DOCK        , PIN_GPH2);
	PIN_SET(RXD_DOCK        , PIN_GPH3);

	PIN_SET(LCD_CS          , PIN_GPL14 | PIN_INVERTED);
	PIN_SET(LCD_SCL         , PIN_GPL10);
	PIN_SET(LCD_SDI         , PIN_GPL11);
	PIN_SET(LCD_RESET       , PIN_GPB1 | PIN_INVERTED);

	PIN_SET(ON_OFF          , PIN_GPF0 | PIN_INVERTED);

	PIN_SET(HW_IIC_SDA      , PIN_GPE15);
	PIN_SET(HW_IIC_SCL      , PIN_GPE14);
	
	PIN_SET(CAMRESET        , PIN_GPJ12);
	PIN_SET(CAMCLKOUT       , PIN_GPJ11);
	PIN_SET(CAMPCLK         , PIN_GPJ8);
	PIN_SET(CAMVSYNC        , PIN_GPJ9);
	PIN_SET(CAMHREF         , PIN_GPJ10);
	PIN_SET(CAMDATA0        , PIN_GPJ0);
	PIN_SET(CAMDATA1        , PIN_GPJ1);
	PIN_SET(CAMDATA2        , PIN_GPJ2);
	PIN_SET(CAMDATA3        , PIN_GPJ3);
	PIN_SET(CAMDATA4        , PIN_GPJ4);
	PIN_SET(CAMDATA5        , PIN_GPJ5);
	PIN_SET(CAMDATA6        , PIN_GPJ6);
	PIN_SET(CAMDATA7        , PIN_GPJ7);
	
	PIN_SET(LED0            , PIN_GPF4); 
	PIN_SET(LED1            , PIN_GPF5); 
	PIN_SET(LED2            , PIN_GPF6); 
	PIN_SET(LED3            , PIN_GPF7); 
}
