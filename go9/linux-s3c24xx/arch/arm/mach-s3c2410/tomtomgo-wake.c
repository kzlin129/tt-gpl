/* arch/arm/mach-s3c2410/tomtomgo-wake.c
 *
 * Definitions of TomTom GO wakeup check functions.
 *
 * Copyright (C) 2005 TomTom BV <http://www.tomtom.com/>
 * Authors: Dimitry Andric <dimitry.andric@tomtom.com>
 *          Jeroen Taverne <jeroen.taverne@tomtom.com>
 *          Mark-Jan Bastian <mark-jan.bastian@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <asm/io.h>
#include <asm/hardware/clock.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-adc.h>
#include <asm/arch/regs-dyn.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-mem.h>
#include <asm/arch/regs-irq.h>
#include <asm/arch/system.h>
#include <asm/arch/tomtomgo-wake.h>
#include <barcelona/Barc_Battery.h>
#include <barcelona/gotype.h>
#include <barcelona/gopins.h>
#include <asm/arch/irqs.h>
#include "tomtomgo-iopins.h"

/* external defs of barcelona bat driver */
extern unsigned int ain4_refraw_calc;
extern unsigned int adc_factor;
extern unsigned int adc_offset;
#ifdef BASIC_BATTERY_ADC_CALIBRATION
extern int use_basic_battery_adc_calibration;
extern unsigned int adc_cal_diff;
extern unsigned int adc_cal_offset;
#endif

#define BOOTLOADER_START_ADDRESS 0x8024
#define SIMULATED_WAKE 0xa5a5a5a5

#ifdef CONFIG_S3C2410_PM_DEBUG
#define DBG(fmt, arg...) \
	do { \
		char buf[256]; \
		extern void printascii(const char *); \
		snprintf(buf, sizeof buf, "tomtomgo-wake: " fmt "\n" ,##arg); \
		printascii(buf); \
	} while (0)
#else /* CONFIG_S3C2410_PM_DEBUG */
#define DBG(fmt, arg...) do {} while (0)
#endif /* CONFIG_S3C2410_PM_DEBUG */

typedef enum {eUNKNOWN_DOCK, eWINDSCREEN_DOCK, eDESK_DOCK} eDockType ;

#ifdef CONFIG_BARCELONA_GPIO
extern int gpio_suicide(void);
#endif

#ifndef CONFIG_SMDK2413_BOARD
static int tomtomgo_decide_battery(void);
#endif /* CONFIG_SMDK2413_BOARD */

/* reset state defined in arch/arm/kernel/setup.c */
extern unsigned int reset_state;

/* 
 * This variable is used to keep track of the SD status at suspend. This is
 * needed to determine if we need to return to the bootloader or not on resume
 * on hardware with builtin flash.
 * (i.e. If we booted from SD, and SD is removed, return to bootloader).
 */

static int sd_status = 0;

/*
 * The var barcelona_reboot_on_sd_removal is to determine the behaviour on wake up. 
 * Depending on if we booted from the SD card, we must sometimes suspend to bootloader 
 * and sometimes just wake linux up.
 * This var is declared extern in /drivers/barcelona/hw/hwdrv.c for proc/barceloa/rebootonsdremoval
 * It must be initialized to '1' because default behaviour must be to suspend to bootloader.
*/ 
int barcelona_reboot_on_sd_removal = 1;

/* 
 * The var suicide_on_rtc_wakeup is set through the gpio driver ioctl IOW_RTCALARM_SUICIDE
 * it will set the RTC alarm and this flag. During suspend, cpu wake on RTC IRQ is set.
 * When waking, and no ACPWR/ONOFF/USB/CDSD is the source, it is assumed to be RTC (srcpnd bug) 
 */
int suicide_on_rtc_wakeup = 0;

/* 
 * The var wake_on_vbus is set through the gpio driver ioctl IOW_USB_VBUS_WAKEUP
 */
int wake_on_vbus = 0;

/* 
 * The var wake_on_rtc is set through the gpio driver ioctl IOW_RTC_WAKEUP
 */
int wake_on_rtc = 0;

/* 
 * The var app_suicide_on_rtc is set through the gpio driver ioctl IOW_RTC_APP_SUICIDE
 */
int app_suicide_on_rtc = 0;

static unsigned int const app_suicide_fallback_wait = 20*HZ; /* 20 sec */
static wait_queue_head_t app_suicide_queue;

/**
 * This kernel thread is started right after we wake up to do an application suicide,
 * and makes sure we do power down even if the application fails to call gpio_suicide()
 * in time.
 */
static int app_suicide_fallback( void *arg )
{
	init_waitqueue_head( &app_suicide_queue );

	if (!wait_event_timeout(
			app_suicide_queue, kthread_should_stop(), app_suicide_fallback_wait))
 
		gpio_suicide();

	return 0;
}

void tomtomgo_before_sleep(int simulateWake, int linuxSuspend)
{
	DBG("tomtomgo_before_sleep");
	if (suicide_on_rtc_wakeup) {
		DBG("enable rtc_wakeup");
#ifdef CONFIG_CPU_S3C2412
		if (IO_GetCpuType() == GOCPU_S3C2412) {
			/* note: also done in sleep.S */
			DBG("enable rtc_wakeup 2412");
			__raw_writel(__raw_readl(S3C2412_PWRCFG) & ~S3C2412_PWRCFG_RTC_CFG, S3C2412_PWRCFG);
		}
#endif /* CONFIG_CPU_S3C2412 */
#ifdef CONFIG_CPU_S3C2443
		if (IO_GetCpuType() == GOCPU_S3C2443) {
			DBG("enable rtc_wakeup 2443");
			__raw_writel(__raw_readl(S3C2443_PWRCFG) & ~S3C2443_PWRCFG_RTC_CFG, S3C2443_PWRCFG);
		}
#endif /* CONFIG_CPU_S3C2443 */
#ifdef CONFIG_CPU_S3C2450
		if (IO_GetCpuType() == GOCPU_S3C2450) {
			DBG("enable rtc_wakeup 2450");
			__raw_writel(__raw_readl(S3C2450_PWRCFG) & ~S3C2450_PWRCFG_RTC_CFG, S3C2450_PWRCFG);
		}
#endif /* CONFIG_CPU_S3C2450 */
#if defined CONFIG_CPU_S3C2440 | defined CONFIG_CPU_S3C2410
		/* and CPU_S3C2442 */
		/* not supported for now, RTC wakeup signal mask not fully documented */
		/* also, no suicide circuit implemented on tomtom boards carrying these SoCs */
#endif
	}

#ifdef CONFIG_S3C2410_PM_DEBUG
	/* Wait a little while to flush debug messages */
	mdelay(10);
#endif /* CONFIG_S3C2410_PM_DEBUG */

	// Disable dock power
	IO_Deactivate(DOCK_PWREN);
	// Wait a while so dockint gets stable
	mdelay(100);
	// Disable all IO to be sure to enter low power mode
	IO_PowerOff();
	// Set power button as wakeup source (just in case)
	IO_SetInterruptOnActivation(ON_OFF);

	if (IO_HavePowerBurstMode()) {
		IO_Activate(PWR_MODE); // Set low power efficiency mode on power supply
	}

	if (simulateWake)
	{
#ifdef CONFIG_CPU_S3C2412
		if (IO_GetCpuType() == GOCPU_S3C2412) {
			tomtomgo_reboot_2412(S3C2412_INFORM0_SFTRST);
		}
		else
#endif /* CONFIG_CPU_S3C2412 */
#ifdef CONFIG_CPU_S3C2443
		if (IO_GetCpuType() == GOCPU_S3C2443) {
			tomtomgo_reboot_2443(S3C2412_INFORM0_SFTRST);
		}
		else
#endif /* CONFIG_CPU_S3C2443 */
#ifdef CONFIG_CPU_S3C2450
		if (IO_GetCpuType() == GOCPU_S3C2450) {
			tomtomgo_reboot_2450(S3C2412_INFORM0_SFTRST);
		}
                else
#endif /* CONFIG_CPU_S3C2450 */
		{
			// S3C2410/40/42
			// Simulate direct wakeup by using non pressed power button
			IO_SetInterruptOnDeactivated(ON_OFF);
			// Set resume address to bootloader
			__raw_writel(BOOTLOADER_START_ADDRESS, s3c24xx_gstatus3);
			// Bootloader checks GSTATUS4 For simulated wake
			__raw_writel(SIMULATED_WAKE, s3c24xx_gstatus4);
		}
	}
	else
	{
		/* suspend */
#ifdef CONFIG_CPU_S3C2412
		if (IO_GetCpuType() == GOCPU_S3C2412) {
			__raw_writel(S3C2412_INFORM0_OFFRST, S3C2412_INFORM0);
		}
#endif /* CONFIG_CPU_S3C2412 */
#ifdef CONFIG_CPU_S3C2443
		if (IO_GetCpuType() == GOCPU_S3C2443) {
			__raw_writel(S3C2412_INFORM0_OFFRST, S3C2443_INFORM0);
		}
#endif /* CONFIG_CPU_S3C2443 */
#ifdef CONFIG_CPU_S3C2450
		if (IO_GetCpuType() == GOCPU_S3C2450) {
			__raw_writel(S3C2412_INFORM0_OFFRST, S3C2450_INFORM0);
		}
#endif /* CONFIG_CPU_S3C2450 */

		if (IO_HaveSdCardInterface() || IO_HaveHsSdCardInterface())
		{
			IO_SetInput(CD_SD);
			mdelay(1);

			/* If no SD, and no internal flash, force bootloader wakeup */
			if (!IO_GetInput(CD_SD) && !IO_HaveInternalFlash())
				linuxSuspend = 0;

			/* Store SD card status for wakup logic */
			sd_status = IO_GetInput(CD_SD);
		}

		// Only set other wakeup sources when staying in Linux
		if (linuxSuspend)
		{
			// Set ignition as wakeup source
			if (IO_HaveNewcastleDock())
			{
				if (IO_CarDocked()) 
				{
					IO_SetInterruptOnActivated(IGNITION);
				}
				else 
				{
					IO_SetInterruptOnActivated(DOCK_RADIO_SENSE);
				}
			}
			else 
			{
				if (IO_HaveDocking()) 
				{
					switch(IO_GetModelId()) 
					{
                                                case GOTYPE_DURBAN:
						case GOTYPE_CAGLIARI:
						case GOTYPE_TREVISO:
							if( wake_on_vbus )
							{
								/* Cagliary ignition wakeup */
								IO_SetInterruptOnActivation(IGNITION);
							}
							break;
						case GOTYPE_RIDER3:
						case GOTYPE_RIDER5:
							IO_SetInterruptOnToggle(IGNITION);
							break;
						default:
							/* malaga / bilbao / valencia / murcia ignition wakeup */
							IO_SetInterruptOnActivation(IGNITION);
							break;
					}
				} 
				else 
				{
					if (IO_HaveUsbBusPowered())
					{
						if (wake_on_vbus) 
						{
							/* palermoS, milan, many others */
							IO_SetInterruptOnActivation(IGNITION);
						}
					}
				}
			}
			// Set SD Card removal as wakeup source
			if (!IO_HavePicDetected()) 
			// except if having PIC, since on PIC units CD_SD 
			// signal is not routed through PIC, and not delaying 
			// CD_SD with PIC before enable PWREN -> crash
			// Non-pic units either have EVT4 without bug, or
			// have EVT3 and keep PWREN always enabled 
			// (and are screened for standby current)
			{
				IO_SetInterruptOnToggle(CD_SD);
			}
			// Resume address of Linux is/was set by pm.c
		}
		else
		{
			// Set ignition as wakeup source on Newcastle
			if (IO_HaveNewcastleDock())
			{
				if (IO_CarDocked()) 
				{
					IO_SetInterruptOnActivated(IGNITION);
				}
				else 
				{
					IO_SetInterruptOnActivated(DOCK_RADIO_SENSE);
				}
			}
			// Suspend to bootloader, so set resume address to bootloader
#ifdef CONFIG_CPU_S3C2412
			if (IO_GetCpuType() == GOCPU_S3C2412)
			{
				__raw_writel(BOOTLOADER_START_ADDRESS, S3C2412_INFORM1);
			}
			else
#endif /* CONFIG_CPU_S3C2412 */
#if defined(CONFIG_CPU_S3C2443) || defined(CONFIG_CPU_S3C2450)
			if ((IO_GetCpuType() == GOCPU_S3C2443) ||
			    (IO_GetCpuType() == GOCPU_S3C2450) )
			{
				__raw_writel(BOOTLOADER_START_ADDRESS, S3C2443_INFORM1);
			}
			else
#endif /* CONFIG_CPU_S3C2443 || CONFIG_CPU_S3C2450 */
			{
				__raw_writel(BOOTLOADER_START_ADDRESS, s3c24xx_gstatus3);
				// Bootloader checks GSTATUS4 for simulated wake
				__raw_writel(0, s3c24xx_gstatus4);
			}
		}
	}
}

void tomtomgo_after_sleep(void)
{
	if (IO_HavePowerBurstMode()) {
		IO_Deactivate(PWR_MODE); 	// Set powersupply in low-ripple mode
	}
}

eDockType get_dock_type (void)
{
	eDockType e_dock = eUNKNOWN_DOCK ;

	IO_SetInput(SW_SCL) ;
	if (IO_GetInput(SW_SCL)) {
		IO_SetInput(SW_SDA) ;
		if (IO_GetInput(SW_SDA)) 
			e_dock = eWINDSCREEN_DOCK ; 
		else 
			e_dock = eDESK_DOCK ;
	}
	return e_dock ;
}

int tomtomgo_decide_wakeup(void)
{
	unsigned long srcpnd;

	// Disable high speed VBUS charging, VBUS removal could have woken up the unit
	IO_Deactivate(USB_PWR_BYPASS);

#ifdef CONFIG_S3C2410_PM_DEBUG
	IO_Activate(DOCK_PWREN);
#endif

#ifndef CONFIG_SMDK2413_BOARD
	// Let inputs stabilize
	mdelay(10);

	// set INFORM0 watchdog flag, if needed
#ifdef CONFIG_CPU_S3C2412
	if (IO_GetCpuType() == GOCPU_S3C2412)
		__raw_writel(S3C2412_INFORM0_WDTRST, S3C2412_INFORM0);
	else
#endif /* CONFIG_CPU_S3C2412 */
#if defined(CONFIG_CPU_S3C2443) || defined(CONFIG_CPU_S3C2450)
	if ((IO_GetCpuType() == GOCPU_S3C2443) ||
	    (IO_GetCpuType() == GOCPU_S3C2450) )
        {
		__raw_writel(S3C2412_INFORM0_WDTRST, S3C2443_INFORM0);
        }
#endif /* CONFIG_CPU_S3C2443 || CONFIG_CPU_S3C2450 */

	IO_SetInput(CD_SD);
	IO_SetInput(ON_OFF);
	IO_SetInput(IGNITION);

	if (IO_HaveSdCardInterface() || IO_HaveHsSdCardInterface())
	{
		// Goto sleep again with resume to bootloader when SD card removed
		DBG("checking SD card removal (or insertion)");
		IO_SetInput(CD_SD);
		mdelay(10);
		if (IO_GetInput(CD_SD) != sd_status)
		{
			if( barcelona_reboot_on_sd_removal ){
#ifdef CONFIG_CPU_S3C2412
				if (IO_HavePicDetected() && IO_GetInput(ON_OFF))
				{
					DBG("Pic detect, SD card removed, rebooting to bootloader");
					tomtomgo_reboot_2412(S3C2412_INFORM0_NEUTRAL);
				}
				else
#endif /* CONFIG_CPU_S3C2412 */
				{
					DBG("SD card removed, suspending to bootloader");
					goto bootloader_suspend;
				}
			}
			/* else simply resume linux */
		}
	}
#if 0
	// Reboot to bootloader when HDD temperature is out of range
	DBG("checking HDD temp");
	if (IO_HaveHarddisk() && tomtom_hddtemp_check() < 0) 
	{
		DBG("HDD temp bad, suspending to bootloader");
		goto bootloader_suspend;
	}
#endif

	// Goto sleep again if battery is too low
	DBG("checking battery");
	if (!tomtomgo_decide_battery()) 
	{
		DBG("battery too low, suspending to linux");
		goto linux_suspend;
	}

	// Enable dock power so IO status can be read
	IO_Activate(DOCK_PWREN);
	// Wait a while for stable IO
	mdelay(50);

	srcpnd=__raw_readl( S3C2410_SRCPND );
	if (srcpnd & IO_GetInterruptPendingMask(CD_SD)) {
		DBG( "Wakeup because of SDCard Insert/Remove" );
		goto linux_suspend;
	}

	// Check if we have woken up because of the RTC. On 2412 the SRCPND register flags are not set to indicate
	// a received interrupt on the RTC, so we check if the external interrupt for the ON_OFF button is the cause.
	// If not, it has to be the RTC.
	if( srcpnd & IO_GetInterruptPendingMask(ON_OFF) ) {
		// Check docking state
		DBG("checking dock Sense");
		if (!IO_CarDocked()) {
			// Not docked so check power button
			int i = IO_HaveLoweredButton() ? 1500 : 440;
			// Power button should be pressed longer for earlier models
			DBG("checking power button %d ", i);
			while (i--) 
			{
				mdelay(1);
				if (!IO_GetInput(ON_OFF)) {
					DBG("power button bounce, suspending to linux");
					goto linux_suspend;
				}
			}
			DBG("power button pressed");
		} else {
			// Docked so check power button and ignition
			DBG("docked");
			if ((!IO_GetInput(IGNITION)) && (!IO_GetInput(ON_OFF))) goto linux_suspend;
			DBG("docked, power button pressed, no wait");
		}
	} else {
		DBG("checking: IGNITION wakeup\n");
		if (IO_GetInterruptCapable(IGNITION)) {
			if (srcpnd & IO_GetInterruptPendingMask(IGNITION)) {
				DBG("IGNITION wakeup\n");
				IO_SetInput(IGNITION);
				if (IO_GetInput(IGNITION)) {
					/*
					 * AC PWR is waking the device up AND tha AC power
					 * cable is plugged in.
					 */
					DBG("IGNITION cable plugged in");
					switch (get_dock_type()) {
						case eWINDSCREEN_DOCK :    
							DBG("Windscreen Dock detected") ;
							break ;
						case eDESK_DOCK :
							DBG("Desk Dock detected -> "
							    "Go back to suspend mode") ;
							goto linux_suspend;
							break ;
						default         :
							DBG("No Dock detected, could be a Malaga or Barcelona device. Waking up") ;
							break ;
					}
				} else {
					DBG("IGNITION loss");
					goto linux_suspend;
				}
			} else {
				DBG("RTC wakeup: not ON_OFF, not CD_SD, not ACPWR, assume source is RTC\n");
				// TODO: double check on all platforms if SRCPND really does not list RTC as source
				if (wake_on_rtc) {
					goto wakeup;
				} else if (app_suicide_on_rtc) {
					app_suicide_on_rtc = 0;

					if (!IO_AllowSuicide())
						goto linux_suspend;

					/* signal to user space that this resume is for suicide only */
					reset_state = 2;

					/* start the kernel thread that will do a gpio_suicide() after 20 secs */
					kthread_run( app_suicide_fallback, NULL, "app_suicide_fallback" );

					goto wakeup;
				} else {
					goto suicide;
				}
					}
				}
	}
#endif // CONFIG_SMDK2413_BOARD
wakeup:
	wake_on_rtc = 0;	/* do not wake again on next sleep */

	DBG("going to wake up");
	return 1;

#ifndef CONFIG_SMDK2413_BOARD
bootloader_suspend:
	tomtomgo_before_sleep(0, 0);
	return 0;

suicide:
	if (suicide_on_rtc_wakeup) {
	  DBG("Performing suicide if possible");
	  if (IO_AllowSuicide()) {
	    gpio_suicide();
	  }
	  DBG("suicide not possible (VBUS/2412EVT3+PIC?), sleeping");
	  suicide_on_rtc_wakeup = 0; // do not retry
	}
	DBG("Suspending to linux");
	// Fall through

linux_suspend:
	tomtomgo_before_sleep(0, 1);
	return 0;
#endif // CONFIG_SMDK2413_BOARD
}


/********************************************************************
 * Battery check
 ********************************************************************/

#ifndef CONFIG_SMDK2413_BOARD
/* adc setup */
static unsigned long const my_adctsc = (1 << 3) | (1 << 4) | (1 << 6), my_adccon = (0xff << 6) | (1 << 14);

static unsigned long get_adc_sample(int channel)
{
	if (IO_GetCpuType() == GOCPU_S3C2443)
		__raw_writel(channel, S3C2443_ADCMUX);
	else if (IO_GetCpuType() == GOCPU_S3C2450)
		__raw_writel(channel, S3C2450_ADCMUX);
	else
		__raw_writel(my_adccon | 0 | (channel << 3), S3C2410_ADCCON);

	mdelay(5);

	/* Start conversion */
	if (IO_GetCpuType() == GOCPU_S3C2443 || IO_GetCpuType() == GOCPU_S3C2450)
		__raw_writel(my_adccon | 1, S3C2410_ADCCON);
	else
		__raw_writel(my_adccon | 1 | (channel << 3), S3C2410_ADCCON);

	/* Wait for start of and end of conversion */
	/* TODO: Timeout? */
	while (__raw_readl(S3C2410_ADCCON) & 1)
		;
	while (!(__raw_readl(S3C2410_ADCCON) & (1<<15)))
		;
	/* Read result; */
	return __raw_readl(S3C2410_ADCDAT0) & 0x3FF;
}

#define NUM_VOLTAGE_SAMPLES	(10)

static int tomtomgo_decide_battery(void)
{
	struct clk *clk;
	int i;
	unsigned long adctsc, adccon;
	unsigned long battery, reference, voltage;

	/* cannot check battery when charging... */
	if (IO_GetInput(ACPWR)) return 1;

	clk = clk_get(NULL, "adc");
	if (!clk)
		return 1;

	/* Save touch screen & ADC registers */
	adctsc = __raw_readl(S3C2410_ADCTSC);
	adccon = __raw_readl(S3C2410_ADCCON);

	/* Activate reference voltage source */
	if (IO_HaveADCAIN4Ref())
	IO_Activate(AIN4_PWR);

	/* Enable clock and let it settle a bit */
	clk_enable(clk);
	mdelay(10);

	/* Write my values */
	__raw_writel(my_adctsc, S3C2410_ADCTSC);
	__raw_writel(my_adccon, S3C2410_ADCCON);

	/* Battery and reference voltages are averaged over 10 samples. */
	battery = reference = 0;

	for (i = 0; i < NUM_VOLTAGE_SAMPLES; i++) 
	{
		battery += get_adc_sample(0);

		if (IO_HaveADCAIN4Ref())
			reference += get_adc_sample(4);
	}

	/* Calculate real battery voltage */
	// 10 + 12 = 22 bits, within range
	if (IO_HaveADCAIN4Ref())
		battery = (battery * ain4_refraw_calc) / (reference);
	else
		battery = battery / NUM_VOLTAGE_SAMPLES;

#ifdef BASIC_BATTERY_ADC_CALIBRATION
	if ( use_basic_battery_adc_calibration ) {
		voltage = CALIBRATE_SAMPLE(battery, adc_cal_offset, BATT_ADC_CAL_DIFF_VOLTAGE, adc_cal_diff, BATT_ADC_CAL_LOW_VOLTAGE);
	}
	else {
		// 12 + 12 = 24 bits, in range
		voltage = (battery * IO_GetADCREFVoltage()) / IO_GetADCRange(); // fast divide by 2^ number
		// 12 + 14 + 5 = 31 bits, in range
		voltage = (voltage * adc_factor * IO_GetBattVoltNumerator()) / (4000 * IO_GetBattVoltDenomenator());
		voltage += adc_offset;
	}
	//printk("%s: battery=%lu, reference=%lu, voltage=%lu", __func__, battery, reference, voltage);
#else
	// 12 + 12 = 24 bits, in range
	voltage = (battery * IO_GetADCREFVoltage()) / IO_GetADCRange(); // fast divide by 2^ number
	// 12 + 14 + 5 = 31 bits, in range
	voltage = (voltage * adc_factor * IO_GetBattVoltNumerator()) / (4000 * IO_GetBattVoltDenomenator());
	voltage += adc_offset;
#endif

	DBG("battery=%lu, reference=%lu, voltage=%lu", battery, reference, voltage);

	/* disable clock and restore original register contents */
	__raw_writel(adccon, S3C2410_ADCCON);
	__raw_writel(adctsc, S3C2410_ADCTSC);

	clk_disable(clk);
	clk_put(clk);

	if (IO_HaveADCAIN4Ref())
	IO_Deactivate(AIN4_PWR);

	if (IO_HaveBatteryCalibration()) {
		/* factory battery calibration allows a lower margin */
		return (voltage >= (3500 + 50)) ? 1 : 0;
	} else {
		if ( IO_GetModelId() == GOTYPE_RIDER5 ) {
			return (voltage >= (3500 + 50)) ? 1 : 0;
		}
		/* older devices have a wider measurement margin */
		return (voltage >= (3500 + 150)) ? 1 : 0;
	}
}
#endif // CONFIG_SMDK2413_BOARD

/********************************************************************
 * IDE check
 ********************************************************************/

static unsigned long ideBaseAddress;

// IDE interface
#define rIDEDATAREAD     (*(volatile unsigned short*)(ideBaseAddress+(0*4)))
#define rIDEDATAWRITE    (*(volatile unsigned short*)(ideBaseAddress+(0*4)))
#define rIDEERROR        (*(volatile unsigned char*) (ideBaseAddress+(1*4)))
#define rIDEFEATURES     (*(volatile unsigned char*) (ideBaseAddress+(1*4)))
#define rIDESECTORCOUNT  (*(volatile unsigned char*) (ideBaseAddress+(2*4)))
#define rIDESECTORNUMBER (*(volatile unsigned char*) (ideBaseAddress+(3*4)))
#define rIDECYLINDERLOW  (*(volatile unsigned char*) (ideBaseAddress+(4*4)))
#define rIDECYLINDERHIGH (*(volatile unsigned char*) (ideBaseAddress+(5*4)))
#define rIDEDEVICEHEAD   (*(volatile unsigned char*) (ideBaseAddress+(6*4)))
#define rIDESTATUS       (*(volatile unsigned char*) (ideBaseAddress+(7*4)))
#define rIDECOMMAND      (*(volatile unsigned char*) (ideBaseAddress+(7*4)))

/* bits in Status register: */
#define BSY 0x0080

static void IDE_WaitForBusyCleared(void)
{
	while ((rIDESTATUS & BSY) > 0);
}

static void IDE_DoCommand(unsigned short cCommand)
{
	IDE_WaitForBusyCleared();
	rIDECOMMAND = cCommand;
	IDE_WaitForBusyCleared();
}

static void IDE_InitDisk(void)
{
	IO_SetInput(HDD_DRQ);
	IO_SetInput(HDD_DACK);
	IO_Activate(HDD_PWR_ON);
	IO_SetInput(HDD_IRQ);
	IO_SetInput(HDD_LED);
	IO_Activate(HDD_RST);
	mdelay(100);
	IO_Deactivate(HDD_RST);
	IO_Activate(HDD_BUF_EN);
	mdelay(100);
	IDE_WaitForBusyCleared();
}

static void IDE_Off(void)
{
	IO_Deactivate(HDD_BUF_EN);
	IO_Activate(HDD_RST);
	IO_Deactivate(HDD_PWR_ON);
	IO_Activate(HDD_LED);
	IO_Deactivate(HDD_IRQ);
	IO_Deactivate(HDD_DRQ);
	IO_Deactivate(HDD_DACK);
}

static int IDE_GetTemperature(void)
{
	rIDEDEVICEHEAD = 0xA0;
	rIDEFEATURES = 0xDF;
	IDE_DoCommand(0xEF);
	return (rIDESECTORNUMBER);
}

void tomtom_hddtemp_setaddress(unsigned long address)
{
	ideBaseAddress = address;
}

int tomtom_hddtemp_check(void)
{
	int temperature;

	IDE_InitDisk();
	temperature = IDE_GetTemperature();
	IDE_Off();
	if ((temperature < 1) || (temperature > 64)) return -1; else return 0;
}

/* EOF */
