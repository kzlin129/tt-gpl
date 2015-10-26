/* include/barcelona/gopins.h
 *
 * one line to give the program's name and an idea of what it does.
 *
 * Copyright (C) 2005,2006,2007,2008 TomTom BV <http://www.tomtom.com/>
 * Authors: Dimitry Andric <dimitry.andric@tomtom.com>
 *          Jeroen Taverne <jeroen.taverne@tomtom.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __INCLUDE_BARCELONA_GOPINS_H
#define __INCLUDE_BARCELONA_GOPINS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#if defined (__KERNEL__) || (__BOOTLOADER__)

typedef unsigned short gopin_t;

struct gopins {
	/* Just to find the address of the first pin */
	gopin_t FIRST;

	/* Type detection */
	gopin_t TYPE_MAIN_ID0;
	gopin_t TYPE_MAIN_ID1;
	gopin_t TYPE_MAIN_ID2;
	gopin_t TYPE_NEC_LCD;
	gopin_t TYPE_SUB_ID0;
	gopin_t TYPE_SUB_ID1;

	/* SD Card Interface */
	gopin_t SD_PWR_ON;
	gopin_t SDCLK;
	gopin_t SDCMD;
	gopin_t SDDATA0;
	gopin_t SDDATA1;
	gopin_t SDDATA2;
	gopin_t SDDATA3;
	gopin_t WP_SD;
	gopin_t CD_SD;
	gopin_t SW_SD;
	gopin_t EN_SD;
	gopin_t MOVI_PWR_ON;
	gopin_t PULLUP_SD;
	
	/* HSMMC Interface */
	gopin_t HS_SDCLK;
	gopin_t	HS_SDCMD;
	gopin_t	HS_SDDATA0;
	gopin_t	HS_SDDATA1;
	gopin_t	HS_SDDATA2;
	gopin_t	HS_SDDATA3;
	gopin_t	HS_SDDATA4;
	gopin_t	HS_SDDATA5;
	gopin_t	HS_SDDATA6;
	gopin_t	HS_SDDATA7;
	gopin_t HS_MOVI_PWR_ON;
	gopin_t HS0_MOVI_PWR_ON;
	gopin_t HS1_MOVI_PWR_ON;

	/* HSMMC0 4bit Interface */
	gopin_t HS0_SDCLK;
	gopin_t	HS0_SDCMD;
	gopin_t	HS0_SDDATA0;
	gopin_t	HS0_SDDATA1;
	gopin_t	HS0_SDDATA2;
	gopin_t	HS0_SDDATA3;

	/* HSMMC1 4bit Interface */
	gopin_t HS1_SDCLK;
	gopin_t	HS1_SDCMD;
	gopin_t	HS1_SDDATA0;
	gopin_t	HS1_SDDATA1;
	gopin_t	HS1_SDDATA2;
	gopin_t	HS1_SDDATA3;

	/* GPS interface */
	gopin_t GPS_RESET;
	gopin_t GPS_ON;
	gopin_t GPS_1PPS;
	gopin_t GPS_REPRO;
	gopin_t TXD_GPS;
	gopin_t RXD_GPS;
	gopin_t RTS_GPS;
	gopin_t CTS_GPS;
	gopin_t GPS_POWERON_OUT;
	gopin_t GPS_STANDBY;
	gopin_t GPS_ANT_OPEN;
	gopin_t GPS_ANT_SHORT;

	/* Harddisk interface */
	gopin_t HDD_LED;
	gopin_t HDD_PWR_ON;
	gopin_t HDD_RST;
	gopin_t HDD_BUF_EN;
	gopin_t HDD_IRQ;
	gopin_t HDD_DRQ;
	gopin_t HDD_DACK;
	gopin_t HDD_CS;

	/* Power management */
	gopin_t ON_OFF;
	gopin_t IGNITION;
	gopin_t ACPWR;
	gopin_t PWR_RST;
	gopin_t CHARGEFAULT;
	gopin_t CHARGE_OUT;
	gopin_t CHARGING;
	gopin_t AIN4_PWR;
	gopin_t BATT_TEMP_OVER;
	gopin_t USB_HP;
	gopin_t USB_SUSPEND_OUT;
	gopin_t USB_PWR_BYPASS;
	gopin_t USB_PWR_ERROR;
	gopin_t PWR_MODE;
	gopin_t LOW_DC_VCC;
	gopin_t PWR_HOLD;
	gopin_t PWR_5V_ON;
	gopin_t FAN_EN;
	gopin_t RDS_ON;
	
	/* Docking */
	gopin_t SW_SCL;
	gopin_t SW_SDA;
	gopin_t DOCK_INT;
	gopin_t DOCK_PWREN;
	gopin_t DOCK_DET_PWREN;
	gopin_t DOCK_SENSE;
	gopin_t DOCK_SENSE1;
	gopin_t DOCK_DESK_SENSE;
	gopin_t DOCK_CRIB_SENSE;
	gopin_t DOCK_VIB_SENSE;
	gopin_t DOCK_MOTOR_SENSE;
	gopin_t DOCK_RADIO_SENSE;
	gopin_t LIGHTS_DETECT;
	gopin_t CTS_DOCK;
	gopin_t RTS_DOCK;
	gopin_t TXD_DOCK;
	gopin_t RXD_DOCK;
	gopin_t RXD_DOCK_INT;
	gopin_t LINEIN_DETECT;
	gopin_t AIN_DOCK_EINT;
	gopin_t DOCK2_PWREN; /* cologne dockpower lights up USB_HOST_DETECT too, valencia USB detect doesn't */
	gopin_t TMC_POWER;
	gopin_t DOCK_I2CEN;

	/* Remote */
	gopin_t FSK_FFS;
	gopin_t FSK_FFE;
	gopin_t FSK_IRQ;
	gopin_t SPICLK;
	gopin_t SPIMSI;
	gopin_t SPIMSO;
	gopin_t FSK_EN;
	gopin_t FSK_CLKOUT;

	/* Accelerometer */
	gopin_t ACC_PWR_ON;
	gopin_t GYRO_EN;

	/* Sound */
	gopin_t MIC_SW;
	gopin_t AMP_ON;
	gopin_t DAC_PWR_ON;
	gopin_t MUTE_EXT;
	gopin_t NAVI_MUTE;
	gopin_t TEL_MUTE;
	gopin_t MUTE_INT;
	gopin_t HEADPHONE_DETECT;
	gopin_t EXTMIC_DETECT;
	gopin_t I2SSDO;
	gopin_t CDCLK;
	gopin_t CDCLK_12MHZ;
	gopin_t I2SSCLK;
	gopin_t I2SLRCK;
	gopin_t I2SSDI;
	gopin_t L3CLOCK;
	gopin_t L3MODE;
	gopin_t L3DATA;

	/* Buzzer */
	gopin_t BUZZER_EN;

	/* Bluetooth */
	gopin_t TXD_BT;
	gopin_t RXD_BT;
	gopin_t RTS_BT;
	gopin_t CTS_BT;
	gopin_t BT_RESET;
	gopin_t BT_MODE;
	gopin_t BT_CLKOUT;
	gopin_t UART_CLK_IN;
	gopin_t UART_CLK_OUT;

	/* GPS_ON + BT_RESET */
	gopin_t BT_RESET_GPS_OFF;

	/* LCD */
	gopin_t VD0;
	gopin_t VD1;
	gopin_t VD2;
	gopin_t VD3;
	gopin_t VD4;
	gopin_t VD5;
	gopin_t VD6;
	gopin_t VD7;
	gopin_t VD8;
	gopin_t VD9;
	gopin_t VD10;
	gopin_t VD11;
	gopin_t VD12;
	gopin_t VD13;
	gopin_t VD14;
	gopin_t VD15;
	gopin_t VD16;
	gopin_t VD17;
	gopin_t VD18;
	gopin_t VD19;
	gopin_t VD20;
	gopin_t VD21;
	gopin_t VD22;
	gopin_t VD23;
	gopin_t VDEN;
	gopin_t VCLK;
	gopin_t LCD_VCC_PWREN;
	gopin_t LCD_BIAS_PWREN;
	gopin_t LCD_OEN;
	gopin_t HSYNC;
	gopin_t VSYNC;
	gopin_t BACKLIGHT_EN;
	gopin_t BACKLIGHT_PWM;
	gopin_t LCD_CS;
	gopin_t LCD_SCL;
	gopin_t LCD_SDI;
	gopin_t LCD_SDO;
	gopin_t LCD_RESET;
	gopin_t LCD_ID;			/* LCM flex mounted ID */
	gopin_t V_PWRDWN;

	/* USB */
	gopin_t USB_HOST_DETECT;
	gopin_t USB_PULL_EN;
	gopin_t USB_PWR_EN;
	gopin_t USB_RST;
	gopin_t USB_SUSPEND;
	gopin_t USB_EJECT;
	gopin_t USB_CLKOUT;
	gopin_t USB_DACK;
	gopin_t USB_DREQ;
	gopin_t USB_IRQ;
	gopin_t USB_VBUS;
	gopin_t USB_RESET;
	gopin_t USB_PHY_PWR_EN;
	gopin_t USB_PHY_1V2_PWR_EN;

	/* IIC */
	gopin_t HW_IIC_SDA;
	gopin_t HW_IIC_SCL;
	gopin_t I2C_SWPWR;

	/* Touchscreen */
	gopin_t XMON;
	gopin_t XPON;
	gopin_t YMON;
	gopin_t YPON;
	gopin_t TSDOWN;

	/* Touchpad */
	gopin_t MEP_DAT;
	gopin_t MEP_ACK;
	gopin_t MEP_CLK;
	gopin_t TOUCHPAD_SW;

	/* CPU */
	gopin_t LOW_CORE;

	/* Flex cable */
	gopin_t FLEX_ID1;
	gopin_t FLEX_ID2;

	/* Light sensor */
	gopin_t LX_EN;

	/* Camera */
	gopin_t	CAM_DPWDN;
	gopin_t CAM_DIRQ;
	gopin_t	CAMRESET;
	gopin_t	CAMCLKOUT;
	gopin_t	CAMPCLK;
	gopin_t	CAMVSYNC;
	gopin_t	CAMHREF;
	gopin_t	CAMDATA0;
	gopin_t	CAMDATA1;
	gopin_t	CAMDATA2;
	gopin_t	CAMDATA3;
	gopin_t	CAMDATA4;
	gopin_t	CAMDATA5;
	gopin_t	CAMDATA6;
	gopin_t	CAMDATA7;

	/* Head Unit communication */
	gopin_t HU_PWR_ON;
	gopin_t TXD_HU;
	gopin_t RXD_HU;	

	/* Factory test points */
	gopin_t FACTORY_TEST_POINT;

	/* GSM */
	gopin_t GSM_SYNC;
	gopin_t GSM_WAKEUP;
	gopin_t GSM_ON;
	gopin_t GSM_OFF;
	gopin_t GSM_DL_EN;
	gopin_t GSM_PORT_SEL;
	gopin_t GSM_RING;
	gopin_t GSM_RESET;
	gopin_t GSM_RXD;
	gopin_t GSM_TXD;
	gopin_t GSM_RTS;
	gopin_t GSM_CTS;
	gopin_t GSM_DSR;
	gopin_t GSM_DTR;
	gopin_t GSM_DCD;
	gopin_t GSM_SYS_RST;
	gopin_t GSM_SYS_EN;

	/* Extra UART(s) */
	gopin_t UART_PWRSAVE;
	gopin_t UART_INTA;
	gopin_t	UART_CSA;
	gopin_t	UART_RXD_IPOD;
	gopin_t UART_INTB;
	gopin_t	UART_CSB;
	gopin_t	UART_RXD_TMC;
	gopin_t UART_RESET;
	gopin_t	UART_CLK;

	/* Dead Reckoning stuff	*/
	gopin_t	ACC_SPI_CSB;	/* Accelerometer SPI cs			*/
	gopin_t ACC_SPI_IRQ;	/* ACC irq pin	*/
	gopin_t	CMP_SPI_CSB;	/* Electronic compass SPI cs	*/
	gopin_t DR_GYRO_HPS;	/* ING300 thingie	*/
	gopin_t DR_CMP_RST;
	gopin_t EN_DR_PWR;
	gopin_t DRDY_INT;
	gopin_t SEN;
	gopin_t BARO_SPI_CSB;	/* Barometer SPI CS */
	gopin_t BARO_SPI_IRQ;	/* Barometer SPI IRQ */
	
	/* PIC workaround */
	gopin_t PIC_DETECT;

	/* SI4710 FM Transmitter */
	gopin_t EN_FM_PWR;
	gopin_t FM_RCLK;
	gopin_t FM_RST;
	gopin_t EN_FM_RCLK;
	gopin_t FM_RTC;
	gopin_t FM_TX_INT;
	
	/* RDS receiver */
	gopin_t FM_RDS_INT;
	gopin_t EN_RDS_RCLK;
	gopin_t RDS_GPIO2;
	gopin_t RDS_GPIO1;
	gopin_t RDS_RST;
	gopin_t RDS_SEN;
	gopin_t EN_RDS_CLK;
	gopin_t RDS_PWR_ON;

	/* Radio */
	gopin_t RADIO_RX;
	gopin_t RADIO_TX;

	/* Diagnostics */
	gopin_t DEBUG_RX;
	gopin_t DEBUG_TX;
	gopin_t GPIO_DIAG0;
	gopin_t GPIO_DIAG1;
	gopin_t GPIO_DIAG2;
	gopin_t GPIO_DIAG_EN;
	gopin_t RXTRIGGER;
	
	/* Debug LED's on SMDK boards */
	gopin_t LED0;
	gopin_t LED1;
	gopin_t LED2;
	gopin_t LED3;

	/* CAM controller */
	gopin_t TXD_MCU;
	gopin_t RXD_MCU;
	gopin_t CAN_RESETIN;
	gopin_t CAN_RESETOUT;
	gopin_t CAN_RESERV;
	gopin_t CAN_BT_MODE;
	gopin_t CAN_SYNC;
	gopin_t MCU_PWR_ON;
	gopin_t PWR_5V_ON_MCU;

	/* Just to find the address of the last pin */
	gopin_t LAST;
};

/* Backlight value mapping, assigned by 0%, 5%, 10%,... 100% -> */
typedef unsigned char backlight_mapping_t[21];

struct gotype {
	unsigned id;
	unsigned short caseid;
	unsigned char backlighttype;
	unsigned int  backlightfreq;
	backlight_mapping_t *backlight_mapping;
	unsigned char backlightccmode;
	unsigned int  battvoltnum:4;
	unsigned int  battvoltdenom:4;
	unsigned char battcalibration;
	unsigned char batvoltagechannel;
	unsigned char batchargecurrentchannel;
	unsigned char refvoltagechannel;
	unsigned char fastchargingduringsuspend;
	unsigned char tsxchannel;
	unsigned char tsychannel;
	unsigned char tsdownchannel;
	unsigned char gyroxchannel;
	unsigned char gyroychannel;
	unsigned char cputype;
	unsigned char btchip;
	unsigned char btusb;
	unsigned int  btspeed;
	unsigned int  btclock;
	unsigned int  btclass;
	unsigned char handsfree;
	unsigned char headsetgw;
	unsigned char a2dp;
	unsigned char lowbutton;
	unsigned char chargertype;
	unsigned char tmcreceivertype;
	unsigned char batterytype;
	unsigned char fmtransmittertype;
	unsigned char sdcard;
	unsigned char hs_sdcard;
	unsigned char hsmmc_4bit;
	unsigned short chargerresistor;
	unsigned char sdslot;
	unsigned char harddisk;
	unsigned char internalflash;
	unsigned char sdisharedbus;
	unsigned char movinandsoftpoweron;
	unsigned char hsmovinandsoftpoweron;
	unsigned char hsmmcinterface0_4bit;
	unsigned char hsmmcinterface1_4bit;
	unsigned char hsmmcinterface;
	unsigned char hsmmcclocktype;
	unsigned char tsfets;
	unsigned char needsvcclcmforiddetect;
	unsigned char tfttype;
	unsigned char gpstype;
	unsigned char gpsephemeris;
	unsigned char codectype;
	unsigned char codecmaster;
	unsigned char canrecordaudio;
	unsigned char acctype;
	unsigned char barotype;
	unsigned char usbslavetype;
	unsigned char usbdevicehostcapable;
	unsigned char dualusbphyctrl;
	unsigned char gpio_autoshutdown;
	unsigned char picsecshutdown;
	unsigned char ohciports;
	unsigned char harddisktiming;
	unsigned char loquendo;
	unsigned char mp3;
	unsigned char regulatorforcodec;
	unsigned char tftsoftstart;
	unsigned char compass;
	unsigned char keeprtcpowered;
	unsigned char hw_i2c;
	unsigned char glautodetect;
	unsigned char gldetected;
	unsigned char picdetected;
	unsigned char pnp;
	unsigned char usbhost;
	unsigned char unusedpinlevel;
	unsigned char codecamptype;
	unsigned char aectype;
	unsigned char deadreckoning;
	unsigned char detected64mb;
	unsigned char videodecoder;
	unsigned char lightsensor;
	unsigned char lightsensorchannel;
	unsigned char backlight_inverted;
	unsigned char tomtom_work;
	unsigned char advancedlaneguidance;
	unsigned char rdstmc;
	unsigned short i2c_suspend_level;
	unsigned char leveled_on_off;
	unsigned char fmtransmitter;
        unsigned char backlighttimer;  
	unsigned char sixbuttonui;
	unsigned short adc_ref_value;
	char name[64];
	char usbname[64];
	char familyname[32];
	char projectname[32];
	char btname[64];
	char requiredbootloader[8];
	char dockdev[8];
	char btdev[8];
	char gpsdev[8];
	char hudev[8];
	char mcudev[8];
	char gprsmodemdev[8];
	struct gopins pins;
};

int IO_Init(void);

extern struct gotype gotype_current;

void IO_InitDockUART(void);
void IO_ExitDockUART(void);
void IO_PowerOff(void);
void IOP_Activate(gopin_t pin);
void IOP_Deactivate(gopin_t pin);
int  IOP_IsInverted(gopin_t pin);
void IOP_GeneratePWM(gopin_t pin);
void IOP_GenerateSPI(gopin_t clockpin, gopin_t datapin, unsigned short* c, int len);
void IOP_Suspend(gopin_t pin);
void IOP_SetFunc(gopin_t pin, unsigned functionNr);
void IOP_SetFunction(gopin_t pin);
void IOP_SetFunction2(gopin_t pin);
void IOP_DisablePullResistor(unsigned port, unsigned pin);
void IOP_SetPullResistor(gopin_t pin_t, unsigned port, unsigned pin);
void IO_Update(void);
int IO_GetDockState(void);
int IO_CarDocked(void);
signed IOP_SetInterruptOnActivation(gopin_t pin);
signed IOP_SetInterruptOnDeactivation(gopin_t pin);
signed IOP_SetInterruptOnActivated(gopin_t pin);
signed IOP_SetInterruptOnDeactivated(gopin_t pin);
signed IOP_SetInterruptOnToggle(gopin_t pin);
signed IOP_GetGCSNumber(gopin_t pin);
signed IO_GetClkOut(gopin_t pin);
void IOP_SetInput(gopin_t pin);
int IOP_GetInput(gopin_t pin);

#ifdef __KERNEL__
signed IOP_GetInterruptNumber(gopin_t pin);
#define IO_GetGpsDevice()		(gotype_current.gpsdev)
#define IO_GetGpsUartNr()		(gotype_current.gpsdev[6] - '0')
#define IO_GetBluetoothUartNr()		(gotype_current.btdev[6] - '0')
#define IO_GetGprsUartNr()		(gotype_current.gprsmodemdev[6] - '0')
#define IO_GetBluetoothDevice()		(gotype_current.btdev)
#define IO_GetDockDevice()		(gotype_current.dockdev)
#define IO_GetMcuDevice()		(gotype_current.mcudev)
#endif
#ifdef __BOOTLOADER__
#define IO_GetGpsDevice()		(gotype_current.gpsdev[6] - '0')
#define IO_GetBluetoothDevice()		(gotype_current.btdev[6] - '0')
#define IO_GetDockDevice()		(gotype_current.dockdev[6] - '0')
#define IO_GetMcuDevice()		(gotype_current.mcudev[6] - '0')
#define IO_GetDockDeviceString()	(gotype_current.dockdev)
#endif

#define IO_GetModelId()			(gotype_current.id)
#define IO_GetCaseId()			(gotype_current.caseid)
#define IO_GetModelName()		(gotype_current.name)
#define IO_GetFamilyName()		(gotype_current.familyname)
#define IO_GetProjectName()		(gotype_current.projectname)
#define IO_GetUsbName()			(gotype_current.usbname)
#define IO_GetBtName()			(gotype_current.btname)
#define IO_GetCpuType()			(gotype_current.cputype)
#define IO_GetSocType()			(IO_GetCpuType())
#define IO_GetBluetoothChip()		(gotype_current.btchip)
#define IO_HaveBluetoothUsb()		(gotype_current.btusb)
#define IO_GetBluetoothSpeed()		(gotype_current.btspeed)
#define IO_GetBluetoothClock()		(gotype_current.btclock)
#define IO_NeedsBluetoothCalibration()	(gotype_current.btclock == 16000000)
#define IO_GetBluetoothClass()		(gotype_current.btclass)
#define IO_HaveHandsfree()		(gotype_current.handsfree)
#define IO_HaveHeadsetGw()		(gotype_current.headsetgw)
#define IO_HaveA2DP()			(gotype_current.a2dp)
#define IO_HaveLoweredButton()		(gotype_current.lowbutton)
#define IO_GetChargerType()		(gotype_current.chargertype)
#define IO_GetBatteryType()		(gotype_current.batterytype)
#define IO_HaveFastChargingDuringSuspend()	(gotype_current.fastchargingduringsuspend)
#define IO_HaveGpioLowBattDetect()	(IO_HasPin(LOW_DC_VCC))
#define IO_HaveAmpShutdownOnLowDcVCC()	(IO_HasPin(LOW_DC_VCC) && (IO_GetChargerType() == GOCHARGER_LTC3455))
#define IO_HavePowerBurstMode()		(IO_HasPin(PWR_MODE))
#define IO_HaveDynamicVoltageScaling()	(IO_HasPin(LOW_CORE) || (IO_GetChargerType() == GOCHARGER_LTC3555))
#define IO_HaveBatteryCalibration()	(gotype_current.battcalibration)
//#define IO_GetADCREFVoltage()		((IO_GetChargerType() == GOCHARGER_LTC3555)?3220:3300)
#define IO_GetADCREFVoltage()		(gotype_current.adc_ref_value)
#define IO_GetCalibrationREFVoltage() 	(3300)
#define IO_GetFMTransmitterType()	(gotype_current.fmtransmittertype)
#define IO_GetTMCReceiverType()		(gotype_current.tmcreceivertype)
#define IO_GetChargerResistor()		(gotype_current.chargerresistor)
#define IO_HaveSdCardInterface()	(gotype_current.sdcard) 	// SD interface used (movinand or SD card)
#define IO_HaveHsSdCardInterface()	(gotype_current.hs_sdcard) 	// HS SD interface
#define IO_IsHsMMC4Bit()		(gotype_current.hsmmc_4bit)
#define IO_HaveSdSlot()			(gotype_current.sdslot)
#define IO_HaveHarddisk()		(gotype_current.harddisk)
#define IO_HaveHsMmcInterface()		(gotype_current.hsmmcinterface)
#define IO_HaveHsMmcInterface0_4bit()	(gotype_current.hsmmcinterface0_4bit)
#define IO_HaveHsMmcInterface1_4bit()	(gotype_current.hsmmcinterface1_4bit)
#define IO_GetHsMmcClockType()		(gotype_current.hsmmcclocktype)
#define IO_HaveTwoConcurrentDisks()     (IO_HaveSdSlot() && IO_HaveHsMmcInterface())
#define IO_HaveAutoformat()		(IO_HaveHarddisk())
#define IO_HaveTsFets()			(gotype_current.tsfets)
#define IO_HaveGpioAutoShutdown()	(gotype_current.gpio_autoshutdown)
#define IO_NeedsVccLcmForLCMIDDetect()	(gotype_current.needsvcclcmforiddetect)
#define IO_UseAltLcdController()	(IO_GetSocType() == GOSOC_S3C2450)
#define IO_GetTftType()			(gotype_current.tfttype)
#define IO_GetTsXChannel()		(gotype_current.tsxchannel)
#define IO_GetTsYChannel()		(gotype_current.tsychannel)
#define IO_GetTsDownChannel()		(gotype_current.tsdownchannel)
#define IO_GetBatteryChannel()		(gotype_current.batterychannel)
#define IO_GetGyroXChannel()		(gotype_current.gyroxchannel)
#define IO_GetGyroYChannel()		(gotype_current.gyroychannel)
#define IO_GetGpsType()			(gotype_current.gpstype)
#define IO_Location025EWrong()		(IO_GetModelId() == GOTYPE_CAGLIARI)
#define IO_GpsFlashTypeIllegal()	(IO_Location025EWrong())
#define IO_HaveGpsEphemeris()		(gotype_current.gpsephemeris)
#define IO_GetCodecType()		(gotype_current.codectype)
#define IO_GetCodecMaster()		(gotype_current.codecmaster)
#define IO_GetCodecClkOut()		(IO_GetClkOut(IO_Pin(CDCLK_12MHZ) & PIN_MASK ))
#define IO_CanRecordAudio()		(gotype_current.canrecordaudio)
#define IO_GetAccType()			(gotype_current.acctype)
#define IO_GetBaroType()		(gotype_current.barotype)
#define IO_HaveDualUSBPhyCtrl()	(gotype_current.dualusbphyctrl)
#define IO_HasBaro()			(IO_HasPin(BARO_SPI_CSB))
#define IO_GetUSBSlaveType()		(gotype_current.usbslavetype)
#define IO_GetUSBProductID()		(gotype_current.usbproductid)
#define IO_UsbOhciPortMask()		(gotype_current.ohciports)
#define IO_GetHarddiskTiming()		(gotype_current.harddisktiming)
#define IO_HaveCompass()		(gotype_current.compass)
#define IO_HaveLoquendo()		(gotype_current.loquendo)
#define IO_HaveMp3()			(gotype_current.mp3)
#define IO_HaveRegulatorForCodec()	(gotype_current.regulatorforcodec)
#define IO_HaveRxdDetect()		(IO_HasPin(RXD_DOCK_INT))
#define IO_GetRequiredBootloader()	(gotype_current.requiredbootloader)
#define IO_HaveTftSoftStart()		(gotype_current.tftsoftstart)
#define IO_GetBacklightType()		(gotype_current.backlighttype)
#define IO_GetBacklightFreq()		(gotype_current.backlightfreq)
#define IO_GetBacklightCCMode()		(gotype_current.backlightccmode)
#define IO_HaveKeepRtcPowered()		(gotype_current.keeprtcpowered)
#define IO_HaveHardwareI2C()		(gotype_current.hw_i2c)
#define IO_HaveSoftwareI2C()		(IO_HasPin(SW_SCL) && IO_HasPin(SW_SDA))
#define IO_GetGprsModemDevice()		(gotype_current.gprsmodemdev)
#define IO_HaveGlAutoDetect()		(gotype_current.glautodetect)
#define IO_HaveGlDetected()		(gotype_current.gldetected)
#define IO_HavePicDetected()		(gotype_current.picdetected)
#define IO_HavePNP()			(gotype_current.pnp)
#define IO_HaveUsbHost()		(gotype_current.usbhost)
#define IO_HaveUsbDeviceHostCapable()   (gotype_current.usbdevicehostcapable)
#define IO_GetUnusedPinLevel()		(gotype_current.unusedpinlevel)
#define IO_GetCodecAmpType()		(gotype_current.codecamptype)
#define IO_GetAecType()			(gotype_current.aectype)
#define IO_HaveDeadReckoning()		(gotype_current.deadreckoning)
#define IO_HaveDetected64MB()		(gotype_current.detected64mb)
#define IO_GetBacklightInverted()	(gotype_current.backlight_inverted)
#define IO_GetBacklightTimer()          (gotype_current.backlighttimer)
#define IO_GetBacklightMapping()	(gotype_current.backlight_mapping)
#define IO_GetUsbChargeBacklightLevel()	(1)
#define IO_GetUsbBootBacklightLevel()	(2)

#define IO_HaveAsyncBatteryDivider()	(IO_HaveBatteryCalibration())
/* keep num/denom within 5 bits */
#define IO_GetBattVoltNumerator()	(gotype_current.battvoltnum ?: (IO_HaveAsyncBatteryDivider()?(3000+1200)/200:(3000+3000)/200))
#define IO_GetBattVoltDenomenator()	(gotype_current.battvoltdenom ?: (IO_HaveAsyncBatteryDivider()?(3000)/200:(3000)/200))
#define IO_GetADCRange()		(1 << 10)
#define IO_GetAIN4RefVoltage()		(1224)
#define IO_GetAIN4RefRawValue()		(((IO_GetAIN4RefVoltage()) * IO_GetADCRange()) / IO_GetADCREFVoltage())
#define IO_GetBatVoltageChannel()	(gotype_current.batvoltagechannel)
#define IO_GetBatChargeCurrentChannel()	(gotype_current.batchargecurrentchannel)
#define IO_GetRefVoltageChannel()	(gotype_current.refvoltagechannel)

#define IO_HaveBluetooth()		(IO_HasPin(TXD_BT))
#define IO_HaveHeadphoneConnector()	(IO_HasPin(HEADPHONE_DETECT))
#define IO_HaveSpeaker()		(IO_HasPin(AMP_ON))
#define IO_HaveLightSensor()		(IO_HasPin(LX_EN) || gotype_current.lightsensor)
#define IO_GetLightSensorChannel()	(gotype_current.lightsensorchannel)
#define IO_HaveTS()			(1)
#define IO_HaveTP()			(IO_HasPin(TOUCHPAD_SW))
#define IO_HaveCamera()			(IO_HasPin(CAMDATA0))
#define IO_HaveBuzzer()			(IO_HasPin(BUZZER_EN))
#define IO_HaveIoExpander()		(IO_HasPin(SW_SDA))
#define IO_HaveGprsModem()		(IO_HasPin(GSM_ON) || IO_HasPin(GSM_SYS_EN))
#define IO_HaveExternalUart()		(IO_HasPin(UART_INTA))
#define IO_HaveDoubleExternalUart()	(IO_HasPin(UART_INTB))
#define IO_HaveSingleExternalUart()	(IO_HaveExternalUart() && (!IO_HaveDoubleExternalUart()))
#define IO_GetExternalUartClkOut()	(IO_GetClkOut (IO_Pin(UART_CLK)&PIN_MASK))
#define IO_HaveSPI()			(IO_HasPin(SPIMSO))
#define IO_HaveRemote()			(IO_HasPin(FSK_EN))
#define IO_HaveUsbBusPowered()		(IO_HasPin(USB_PWR_BYPASS))
#define IO_HaveRealShutdown()		(IO_HasPin(PWR_RST))
#define IO_AllowSuicideException()	(IO_HasPin(USB_HOST_DETECT) && IO_GetInput(USB_HOST_DETECT))
#define IO_AllowSuicide()		(IO_HaveRealShutdown() && (!IO_HavePicDetected()) && (!IO_AllowSuicideException()))
#define IO_HavePicSecShutdown()		(gotype_current.picsecshutdown)
#define IO_HaveRiderDock()		(IO_HasPin(DOCK_MOTOR_SENSE))
#define IO_HaveDocking()		(IO_HasPin(DOCK_SENSE) || IO_HaveRiderDock())
#define IO_HaveDockPower()		(IO_HasPin(DOCK_PWREN))
#define IO_HaveGpioHeadlights()		(IO_HasPin(LIGHTS_DETECT))
#define IO_HaveAlkaline()		(IO_HasPin(CAM_DPWDN))
#define IO_HaveAnalogDockInput()	(IO_HasPin(CAM_DPWDN))
#define IO_HaveNewcastleDock()		(IO_HasPin(DOCK_RADIO_SENSE))
#define IO_HaveCagliariDock()		(IO_HasPin(DOCK_SENSE1))
#define IO_HaveInternalFlash()		(gotype_current.internalflash)
#define IO_HaveSdMovinandShared()	(gotype_current.sdisharedbus)
#define IO_HaveMovinandSoftPoweron()	(gotype_current.movinandsoftpoweron)
#define IO_HaveHsMovinandSoftPoweron()	(gotype_current.hsmovinandsoftpoweron)
#define IO_PlayStartupSound()		(!IO_HaveBuzzer())
#define IO_HaveFMTransmitter()		(gotype_current.fmtransmitter)
#define IO_HaveTMCReceiver()		(IO_HasPin(RDS_RST))
#define IO_HaveForteMediaDSP()		(IO_HasPin(VP_PWRDN))
#define IO_HaveLcmIdWorkaroundFeature()	((IO_GetModelId() == GOTYPE_CASABLANCA) && (IO_GetGpsType() == GOGPS_SIRF3))
#define IO_NeedsATAGForLCMIDDetect()	(IO_HaveLcmIdWorkaroundFeature())
#define IO_GetVideoDecoder()		(gotype_current.videodecoder)
#define IO_HaveLeveledOnOff()		(gotype_current.leveled_on_off)
#define IO_HaveUartClockLoopback()	(IO_HasPin(UART_CLK_IN))
#define IO_GetUartLoopbackClkOut()	(IO_GetClkOut(IO_Pin(UART_CLK_OUT)&PIN_MASK))
#define IO_HasTomTomWorkFeatures()	(gotype_current.tomtom_work)
#define IO_HaveAdvancedLaneGuidanceFeature()	(gotype_current.advancedlaneguidance)
#define IO_HaveRDSTMCFeature()		(gotype_current.rdstmc)
#define IO_HaveSixButtonUiFeature()	(gotype_current.sixbuttonui)
#define IO_HaveADCAIN4Ref()		(IO_HasPin(AIN4_PWR))

#define IO_Pin(x)			(gotype_current.pins.x)

#define IO_Activate(x)				IOP_Activate(IO_Pin(x))
#define IO_Deactivate(x)			IOP_Deactivate(IO_Pin(x))
#define IO_IsInverted(x)			IOP_IsInverted(IO_Pin(x))
#define IO_GeneratePWM(x)			IOP_GeneratePWM(IO_Pin(x))
#define IO_GenerateSPI(a,b,c,d)			IOP_GenerateSPI(IO_Pin(a),IO_Pin(b),(c),(d))
#define IO_Suspend(x)				IOP_Suspend(IO_Pin(x))
#define IO_SetFunction(x)			IOP_SetFunction(IO_Pin(x))
#define IO_SetFunction2(x)			IOP_SetFunction2(IO_Pin(x))
#define IO_SetInterruptOnActivation(x)		IOP_SetInterruptOnActivation(IO_Pin(x))
#define IO_SetInterruptOnDeactivation(x)	IOP_SetInterruptOnDeactivation(IO_Pin(x))
#define IO_SetInterruptOnActivated(x)		IOP_SetInterruptOnActivated(IO_Pin(x))
#define IO_SetInterruptOnDeactivated(x)		IOP_SetInterruptOnDeactivated(IO_Pin(x))
#define IO_SetInterruptOnToggle(x)		IOP_SetInterruptOnToggle(IO_Pin(x))
#define IO_SetInput(x)				IOP_SetInput(IO_Pin(x))
#define IO_GetInput(x)				IOP_GetInput(IO_Pin(x))
#define IO_GetPinState(x)			IOP_GetInput(IO_Pin(x))
#define IO_DisablePullResistor(x)		IOP_DisablePullResistor(GET_PORTNR(IO_Pin(x)), GET_PINNR(IO_Pin(x)))
#define IO_SetPullResistor(x)			IOP_SetPullResistor(IO_Pin(x), GET_PORTNR(IO_Pin(x)), GET_PINNR(IO_Pin(x)))
#define IO_GetInterruptNumber(x)		IOP_GetInterruptNumber(IO_Pin(x))
#define IO_GetInterruptCapable(x)		(IO_GetInterruptNumber(x) >= 0)
#define IO_GetInterruptPendingMask(x)		(1 << (IO_GetInterruptNumber(x) - S3C2410_CPUIRQ_OFFSET))
#define IO_GetGCSNumber(x)			IOP_GetGCSNumber(IO_Pin(x))
#define IO_GetPAForGCSNumber(x)			((unsigned long) (x) * 0x8000000)
#define IO_HasPin(x)				(IO_Pin(x) != 0)
#define IO_GetPinLocation(x)			((x) & 0x01ff)
#define IO_CheckPinLocation(x,y)		(IO_GetPinLocation(x) == (y))
#define IO_MakeHigh(x)				(IO_IsInverted(x)?IO_Deactivate(x):IO_Activate(x))
#define IO_MakeLow(x)				(IO_IsInverted(x)?IO_Activate(x):IO_Deactivate(x))

#define GO_FLASH_SHORT_UNDEFINED		0xffff

/* GO CPU types */
#define GOCPU_UNDEFINED		0	/* Undefined */
/* Samsung */
#define GOCPU_S3C2410		1	/* Samsung S3C2410 */
#define GOCPU_S3C2440		2	/* Samsung S3C2440 */
#define GOCPU_S3C2442		3	/* Samsung S3C2442 */
#define GOCPU_S3C2412		4	/* Samsung S3C2412 or S3C2413 */
#define GOCPU_S3C2443		5	/* Samsung S3C2443 */
#define GOCPU_S3C2416		6	/* Samsung S3C2416 */
#define GOCPU_S3C2450		7	/* Samsung S3C2450 */

/* Soc types */
#define GOSOC_UNDEFINED		0	/* Undefined */
#define GOSOC_S3C2443		5	/* Samsung S3C2443 */
#define GOSOC_S3C2450		7	/* Samsung S3C2450 */

/* GO TFT types */
#define GOTFT_HWDETECT 0xffff

#define GOTFT_UNDEFINED			0	/* Undefined */
#define GOTFT_NEC_NL2432HC22		2	/* NEC NL2432HC22-22B */
#define GOTFT_SAMSUNG_LTV350		3	/* Samsung LTV350QV */
#define GOTFT_SAMSUNG_LTP400		4	/* Samsung LTP400WQ */
#define GOTFT_SAMSUNG_LTE246QV		5	/* Samsung LTE246QV */
#define GOTFT_SAMSUNG_LTE430WQ		6	/* Samsung LTE430WQ */
#define GOTFT_SHARP_LQ043T1		7	/* Sharp LQ043T1DG01 */
#define GOTFT_SAMSUNG_LMS350GF		8	/* Samsung LMS350GF12 */
#define GOTFT_SHARP_LQ035Q1DG   	9       /* Sharp LQ035Q1DG */
#define GOTFT_SAMSUNG_LMS430HF12 	10      /* Samsung LMS430HF12 */
#define GOTFT_TOSHIBA_LTA058B3LOF	11      /* Toshiba LTA058B3LOF */
#define GOTFT_SHARP_LQ035Q1DG04		12      /* Sharp LQ035Q1DG04 */
#define GOTFT_AUO_A035QN02   		13      /* AUO A035QN02 */
#define GOTFT_SHARP_LQ043T3DW01 	14      /* Sharp LQ043T3DW01 */
#define GOTFT_SHARP_LQ043T3DW02 	15      /* Sharp LQ043T3DW02 */
#define GOTFT_SAMSUNG_LMS350GF20	16	/* Samsung LMS350GF20 */
#define GOTFT_SAMSUNG_LMS430HF19 	17      /* Samsung VE LMS430HF19 */
#define GOTFT_AUO_A043FW03V0  		18      /* AUO A043FW03V0; Treviso */
#define GOTFT_AUO_A043FW03V1  		19      /* AUO A043FW03V1; Livorno/Florenc/Nanchang */
#define GOTFT_ATLAS_A_CLD			20      /* Atlas3 LCD */
#define GOTFT_AUO_A050FW02V2  		21      /* AUO A050FW02V2 */
#define GOTFT_SAMSUNG_LMS500HF01	22		/* Samsung LMS500HF01 */
#define GOTFT_SAMSUNG_LMS430HF17    23      /* Samsung LMS430HF17 */
#define GOTFT_SAMSUNG_LMS430HF11    24      /* Samsung LMS430HF11 */
#define GOTFT_LG_LB043WQ3           25      /* LG LB043WQ3 */
#define GOTFT_SAMSUNG_LMS430HF29	26		/* Samsung LMS430HF29 */
#define GOTFT_AUO_A043FW05V1		27		/* AUO A043FW05V1: Bergamo */
#define GOTFT_AUO_A050FW03V2		28      /* AUO A050FW03V2: Austin */
#define GOTFT_LG_LD050WQ1			29		/* LG LD050WQ1: Austin/Acton */
#define GOTFT_SAMSUNG_LMS500HF05	30      /* Samsung LMS500HF05 */
#define GOTFT_WISTRON_T35QTA530		31      /* Wistron T35QTA530 */

/* GO backlight variants */
#define GOBACKLIGHT_UNDEFINED				0	/* Undefined */
#define GOBACKLIGHT_CH0_TPS61042_350			1	/* Barcelona, Malaga series */
#define GOBACKLIGHT_CH0_30K_TPS61042_350		2	/* Atlanta, Glasgow, Bilbao, Abderdeen */
#define GOBACKLIGHT_CH0_30K_TPS61042DRBR_400		3	/* Valencia, Murcia */
#define GOBACKLIGHT_CH0_AT1312_350			4	/* Edinburgh, Cork, Casablanca, Newcastle, Cologne */
#define GOBACKLIGHT_CH0_1000_CAT3238TD_430		5	/* 1000 Hz Milan/Modena Sharp, Limerick, Knock, Rome */
#define GOBACKLIGHT_CH0_1839_CAT3238TD_430		6	/* 1839 Hz Marigot, Milan/Modena Samsung */
#define GOBACKLIGHT_CH0_EUP2584_350			7	/* PalermoS */
#define GOBACKLIGHT_FB_CH1_EUP2584_350			10	/* Experimental PalermoS */
#define GOBACKLIGHT_FB_CH1_CAT4238TD_430		11	/* Experimental Rome */
#define GOBACKLIGHT_FB_CH0_20K_CAT3238TD_430		12	/* Cagliari, Cairns */
#define GOBACKLIGHT_LTC3577_CC_MODE			13	/* Treviso */
#define GOBACKLIGHT_FB_CH1_TB62752AFUG_500		14	/* Austin */
#define GOBACKLIGHT_CH0_APW7209					15	/* Austin */
#define GOBACKLIGHT_FB_CH1_APW7209				16	/* Austin */

/* GO GPS types */
#define GOGPS_UNDEFINED		0	/* Undefined */
#define GOGPS_SIRF1		1	/* Sirf 1 */
#define GOGPS_SIRF2		2	/* Sirf 2 */
#define GOGPS_SIRF3		3	/* Sirf 3 */
#define GOGPS_SIRF_ATLAS3	4	/* Sirf Atlas3 */
#define GOGPS_ATH_AR1520	65	/* Atheros AR1520 */
#define GOGPS_GL		128	/* Global Locate Hammerhead */
#define GOGPS_GL_INT_LNA	129	/* Global Locate Hammerhead using internal LNA */
#define GOGPS_GL_BCM4750	130	/* Global Locate Barracuda */

/* GO BT chip types */
#define GOBT_NONE		0	/* No bluetooth chip */
#define GOBT_BC3		3	/* BlueCore 3 */
#define GOBT_BC4		4	/* BlueCore 4 */
#define GOBT_CSR8811		5	/* CSR8811 */

/* GO Codec types */
#define GOCODEC_NONE		0	/* No codec */
#define GOCODEC_WM8711		1	/* Wolfson WM8711 */
#define GOCODEC_WM8971		2	/* Wolfson WM8971 */
#define GOCODEC_WM8750		3	/* Wolfson WM8750, Marigot */
#define GOCODEC_WM8972		4	/* Wolfson WM8971 atlas3evb */
#define GOCODEC_CS42L52		5	/* Cirrus Logic CS42L52 */
#define GOCODEC_ALC5628		6	/* Realtek ALC5628 */

/* GO Codec clock configuration */
#define GOCODECCFG_SLAVE		0	/* Codec is slave */
#define GOCODECCFG_EXTERNAL_MASTER	1	/* Codec is master and is fed with external 12 MHZ clock */
#define GOCODECCFG_INTERNAL_MASTER	2	/* Codec is master but fed from S3C24XX internal iis clock */

/* GO Acc types */
#define GOACC_NONE		0
#define GOACC_MXR2312		1
#define GOACC_MXR3999		2
#define GOACC_MXM9301		3
#define GOACC_MXR9500		4
#define GOACC_SMB365		5
#define GOACC_KXP74		6
#define GOACC_KXR94		7

/* GO Baro types */
#define GOBARO_NONE		0
#define GOBARO_SCP1000		1

/* GO Gyro types */
#define GOGYRO_NONE		0
#define GOGYRO_EWTS98PA21	1

/* GO SDRAM/DDR types for NOR CONFIG values*/

#define GO_RAM_UNDEFINED			0xffff	/* Undefined */
#define GO_SAMSUNG_SDRAM_32MB_30V		1	/* K4M561633 */
#define	GO_SAMSUNG_SDRAM_32MB_18V		2 
#define GO_MICRON_SDRAM_32MB_18V		128

/* GO FM Transmitter types */
#define GOFM_NONE		0
#define GOFM_SI4710		1
#define GOFM_SI4711		2

/* GO TMC receiver types */
#define GOTMC_NONE 		0
#define GOTMC_SI4703		1
#define GOTMC_SI4705		2
#define GOTMC_SI4706		3
#define GOTMC_SI4749		4

/* GO Charger types */
#define GOCHARGER_LTC1733	0
#define GOCHARGER_LTC3455	1
#define GOCHARGER_LTC3555	2
#define GOCHARGER_LTC3577	3

/* GO Battery types */
#define GOBATTERY_UNDEFINED		0	/* undefined */
#define GOBATTERY_ICR18650_2200		1	/* GO Classic - Valencia, Rider */
#define GOBATTERY_1100			2	/* various */
#define GOBATTERY_ICP803443_1350	3	/* Aberdeen, Knock */
#define GOBATTERY_1320			4	/* Milan / Modena */
#define	GOBATTERY_920			5	/* Palermo */
#define GOBATTERY_VARTA_1460_100	6	/* Eldorado */
#define GOBATTERY_MAXELL_1100_130	7	/* Cagliari/Treviso */
#define GOBATTERY_LISHEN_900		8	/* PalermoS */
#define GOBATTERY_LISHEN_1100		9	/* Treviso */
#define GOBATTERY_LISHEN_650		10	/* Xiamen */
#define GOBATTERY_SAMSUNG_463446_820 	11 /* Bergamo */
#define GOBATTERY_MAXELL_820		12 /* Bergamo */
#define GOBATTERY_LISHEN_820		13 /* Bergamo */

/* GO GPRS modem types */
#define GOGPRS_NONE		0
#define GOGPRS_MC55		1
#define GOGPRS_ADI6720		2
#define GOGPRS_FARO		3

/* GO GPRS antenna types */
#define GOGPRSANT_NONE		0
#define GOGPRSANT_EU		1
#define GOGPRSANT_US		2
#define GOGPRSANT_QUAD		3

/* Go HSMMC clock types */
#define GOHSMMC_HCLK		0	/* Default clock */
#define GOHSMMC_EPLL		1
#define GOHSMMC_USB48CLK	2

/* Video Decoder */
#define GOVD_OV9655  		0
#define GOVD_ADV7180 		1

/* GO dock types */
#define GODOCK_NONE		0
#define GODOCK_WINDSCREEN	1
#define GODOCK_CRIB		2
#define GODOCK_DESK		3
#define GODOCK_VIB		4
#define GODOCK_MOTOR		5
#define GODOCK_RADIO		6
#define GODOCK_WINDSCR_WO_FM	7

/* GO case types */
#define GOCASE_MALAGA		0	/* Barcelona, M100,300,500 */
#define GOCASE_BILBAO		1	/* Bilbao */
#define GOCASE_GLASGOW		2	/* Glasgow and Aberdeen */
#define GOCASE_VALENCIA		3	/* V510,710 */
#define GOCASE_EDINBURGH	4	/* Edinburgh */
#define GOCASE_NEWCASTLE	5	/* Newcastle */
#define GOCASE_LIMERICK		6	/* Limerick */
#define GOCASE_MILAN		7	/* Milan */
#define GOCASE_LISBON		8	/* Lisbon */
#define GOCASE_PALERMO		9	/* Palermo */
#define GOCASE_ROME		10	/* Rome */
#define GOCASE_CAGLIARI		11	/* Cagliari */
#define GOCASE_DUBLIN		12	/* Dublin */
#define GOCASE_MARIGOT		13	/* Marigot */
#define GOCASE_REDWOOD		14	/* Centrality Atlas 3 EVB */
#define GOCASE_TTDEV		15	/* TT Evaluation Board */
#define GOCASE_MODENA		16	/* Modena */
#define GOCASE_CAIRNS		17	/* Cairns */
#define GOCASE_PAOLA		18	/* Paola */
#define GOCASE_LIVORNO		19	/* Livorno / Florence */
#define GOCASE_ALICANTE		20	/* Alicante ~= Parma 4GB + TomTom Work */

/* Differential Amp Type */ 
#define GOCODECAMP_SINGLEENDED		0	/* Codec and Amp are used in single ended mode */
#define GOCODECAMP_DIFFERENTIAL_HW	1	/* Codec and Amp are used in differential mode */
#define GOCODECAMP_DIFFERENTIAL_SW	2	/* Amp is used in differential mode, codec output is made differential in software */

/* AEC type */
#define GOAEC_ACOUSTIC		0
#define GOAEC_FM		1

/* GO USB slave types */
#define GOUSB_S3C24XX		0
#define GOUSB_TUSB6250		1
#define GOUSB_NET2272		2
#define GOUSB_S3C2443		3

/* GO SDRAM/DDR types for NOR CONFIG values*/
#define GODDR_HWDETECT		0xffff
#define GODDR_32MB		32
#define GODDR_64MB		64
#define GODDR_128MB		128

/* Set if device is having a USB port that is device-capable only (no prague over USB support) */
#define IO_HaveUSBDeviceOnly()		((IO_GetUSBSlaveType() == GOUSB_TUSB6250) || (IO_GetUSBSlaveType() == GOUSB_NET2272))

/* Software feature flags */
#define	GOFEATURE_TTS			0 /* Bit 0, text to speech */
#define	GOFEATURE_TTW			1 /* Bit 1, TomTom Work */
#define	GOFEATURE_ALG			2 /* Bit 2, Advanced Lane Guidance */
#define	GOFEATURE_RDSTMC		3 /* Bit 3, RDSTMC */
#define GOFEATURE_CARLINK		4 /* bit 4, CarLink */
#define GOFEATURE_SIXBUTTONUI		5 /* bit 5, Use two button UI */

#else /* (__KERNEL__) || (__BOOTLOADER__) */
#error "Never include this header from user space."
#endif /* (__KERNEL__) || (__BOOTLOADER__) */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCLUDE_BARCELONA_GOPINS_H */

/* EOF */
