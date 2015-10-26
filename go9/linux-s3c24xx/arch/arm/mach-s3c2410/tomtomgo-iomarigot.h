#define SUPPORT_MARIGOT
inline void IO_InitMarigot(void)
{
	VALUE_SET(id                 , GOTYPE_MARIGOT);
	VALUE_SET(caseid             , GOCASE_MILAN);
	VALUE_SET(cputype            , GOCPU_S3C2443);

	VALUE_SET(btchip             , GOBT_BC4);
	VALUE_SET(btspeed            , 921600);
	VALUE_SET(btclock            , 16000000);
	VALUE_SET(btclass            , 0x280408);
	VALUE_SET(handsfree          , 1);
	VALUE_SET(headsetgw          , 1);
	VALUE_SET(a2dp               , 1);

	VALUE_SET(sdcard             , 1);
        VALUE_SET(sdslot             , 1);
	VALUE_SET(internalflash      , 1);
	VALUE_SET(hsmmcinterface     , 1);
	VALUE_SET(tfttype            , GOTFT_SAMSUNG_LTE430WQ);
	VALUE_SET(backlighttype      , GOBACKLIGHT_CH0_1839_CAT3238TD_430);
	VALUE_SET(gpstype            , GOGPS_SIRF3);
	VALUE_SET(codectype          , GOCODEC_WM8750);
	VALUE_SET(codecmaster        , GOCODECCFG_SLAVE);
	VALUE_SET(regulatorforcodec  , 1);
	VALUE_SET(keeprtcpowered     , 1);
	VALUE_SET(canrecordaudio     , 1);
	VALUE_SET(hw_i2c             , 1);
	VALUE_SET(pnp                , 1);
	VALUE_SET(codecamptype       , GOCODECAMP_DIFFERENTIAL_HW);
	VALUE_SET(aectype            , GOAEC_FM);
	VALUE_SET(loquendo           , 1);
        VALUE_SET(gpsephemeris       , 1);
        VALUE_SET(leveled_on_off     , 1);

	STRING_SET(familyname,"TomTom GO");
	STRING_SET(projectname,"MARIGOT");
	STRING_SET(requiredbootloader,"5.07");
	STRING_SET(btdev,"ttySAC1");
	STRING_SET(gpsdev,"ttySAC2");
	STRING_SET(dockdev,"ttySAC0");
	STRING_SET(mcudev,"ttySAC3");

        PIN_SET(CAMRESET           , PIN_GPJ12);
        PIN_SET(CAMCLKOUT          , PIN_GPJ11);
        PIN_SET(CAMHREF            , PIN_GPJ10);
        PIN_SET(CAMVSYNC           , PIN_GPJ9);
        PIN_SET(CAMPCLK            , PIN_GPJ8);
        PIN_SET(CAMDATA0           , PIN_GPJ0);
        PIN_SET(CAMDATA1           , PIN_GPJ1);
        PIN_SET(CAMDATA2           , PIN_GPJ2);
        PIN_SET(CAMDATA3           , PIN_GPJ3);
        PIN_SET(CAMDATA4           , PIN_GPJ4);
        PIN_SET(CAMDATA5           , PIN_GPJ5);
        PIN_SET(CAMDATA6           , PIN_GPJ6);
        PIN_SET(CAMDATA7           , PIN_GPJ7);
        PIN_SET(CAM_DIRQ           , PIN_GPG3 | PIN_INVERTED);

	PIN_SET(ON_OFF             , PIN_GPF0 | PIN_INVERTED);
	PIN_SET(CD_SD              , PIN_GPF1 | PIN_INVERTED);
	PIN_SET(RDS_GPIO2	   , PIN_GPF4);
	PIN_SET(RDS_GPIO1	   , PIN_GPF5);
	PIN_SET(RXTRIGGER	   , PIN_GPF6);
	PIN_SET(WP_SD              , PIN_GPF7);

	PIN_SET(GPS_1PPS           , PIN_GPG0);
	PIN_SET(GPS_ANT_SHORT      , PIN_GPG1);
	PIN_SET(PWR_5V_ON_MCU      , PIN_GPG4);
	PIN_SET(GPS_ANT_OPEN       , PIN_GPG5);
	PIN_SET(FAN_EN             , PIN_GPG6);
	PIN_SET(BACKLIGHT_EN       , PIN_GPG7);
        PIN_SET(BT_RESET           , PIN_GPG9 | PIN_INVERTED);
        PIN_SET(CAN_RESETIN        , PIN_GPG10 | PIN_INVERTED);

	PIN_SET(I2SLRCK            , PIN_GPE0);
	PIN_SET(I2SSCLK            , PIN_GPE1);
	PIN_SET(CDCLK              , PIN_GPE2);
	PIN_SET(I2SSDI             , PIN_GPE3);
	PIN_SET(I2SSDO             , PIN_GPE4);
	PIN_SET(HW_IIC_SCL         , PIN_GPE14);
	PIN_SET(HW_IIC_SDA         , PIN_GPE15);

	PIN_SET(BACKLIGHT_PWM      , PIN_GPB0);
	PIN_SET(BT_MODE            , PIN_GPB1);
	PIN_SET(L3DATA             , PIN_GPB2);
	PIN_SET(L3MODE             , PIN_GPB3);
	PIN_SET(L3CLOCK            , PIN_GPB4);

        PIN_SET(TXD_DOCK	   , PIN_GPH0);
        PIN_SET(RXD_DOCK   	   , PIN_GPH1);
	PIN_SET(TXD_BT             , PIN_GPH2);
	PIN_SET(RXD_BT             , PIN_GPH3);
	PIN_SET(TXD_GPS            , PIN_GPH4);
	PIN_SET(RXD_GPS            , PIN_GPH5);
	PIN_SET(TXD_MCU            , PIN_GPH6);
	PIN_SET(RXD_MCU            , PIN_GPH7);
	PIN_SET(CTS_BT             , PIN_GPH10);
	PIN_SET(RTS_BT             , PIN_GPH11);

	PIN_SET(GPS_REPRO          , PIN_GPE11);
	PIN_SET(GPS_ON             , PIN_GPE12);
	PIN_SET(GPS_RESET          , PIN_GPE13 | PIN_INVERTED);

        PIN_SET(NAVI_MUTE    	   , PIN_GPL11);
        PIN_SET(PWR_HOLD	   , PIN_GPL12);
	PIN_SET(FACTORY_TEST_POINT , PIN_GPL14);

	PIN_SET(SD_PWR_ON          , PIN_GPJ13 | PIN_INVERTED);
	PIN_SET(HS_MOVI_PWR_ON     , PIN_GPJ14 | PIN_INVERTED);
	PIN_SET(DAC_PWR_ON         , PIN_GPJ15);

	PIN_SET(LCD_RESET          , PIN_GPC0 | PIN_INVERTED);
	PIN_SET(V_PWRDWN           , PIN_GPC5 | PIN_INVERTED);
	PIN_SET(LCD_VCC_PWREN      , PIN_GPC6);

        PIN_SET(CAN_RESETOUT       , PIN_GPB5 | PIN_INVERTED);
        PIN_SET(CAN_BT_MODE        , PIN_GPB6);
        PIN_SET(CAN_RESERV         , PIN_GPB7);
        PIN_SET(CAN_SYNC           , PIN_GPB8);
        PIN_SET(RDS_SEN            , PIN_GPB9 | PIN_INVERTED);
        PIN_SET(RDS_RST            , PIN_GPB10 | PIN_INVERTED);

        PIN_SET(EN_RDS_RCLK        , PIN_GPH13 | PIN_INVERTED);
        PIN_SET(RDS_PWR_ON         , PIN_GPH14);

	PIN_SET(GPIO_DIAG0         , PIN_GPA9);
        PIN_SET(AIN4_PWR           , PIN_GPA11);	// ADC_REF in schematics
	PIN_SET(PWR_RST            , PIN_GPA12);
	PIN_SET(GPIO_DIAG1         , PIN_GPA13);
	PIN_SET(GPIO_DIAG2         , PIN_GPA14);
        PIN_SET(MCU_PWR_ON         , PIN_GPA15 | PIN_INVERTED);
}

void IO_InitMarigotM1(void)
{
	STRING_SET(name,"TomTom GO MARIGOT");
	STRING_SET(btname,"TomTom GO MARIGOT");
	STRING_SET(usbname,"GO MARIGOT");
}
