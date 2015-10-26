/* drivers/barcelona/gpio/gpio.c
 *
 * Implementation of the TomTom Dock (GPIO) driver.
 *
 * Copyright (C) 2004,2005,2006 TomTom BV <http://www.tomtom.com/>
 * Authors: Dimitry Andric <dimitry.andric@tomtom.com>
 *          Koen Martens <kmartens@sonologic.nl>
 *          Jeroen Taverne <jeroen.taverne@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* Includes */
#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/stat.h>
#include <linux/workqueue.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/hardware/clock.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-mem.h>
#include <asm/system.h>
#include <barcelona/Barc_Gpio.h>
#include <barcelona/gopins.h>
#include <barcelona/debug.h>
#include <barcelona/Barc_buspower.h>

#include "../sound/codec.h"

#define GREEN   0x07e0
void tomtomgo_lcdclearcolor(unsigned short color);

int rds_tmc_hack_enabled = 0; // global shared with buspower.c

#include "ns73.h"

/* Defines */
#define PFX "gpio: "
#define PK_DBG PK_DBG_FUNC

#define GPIO_POLL_DELAY (HZ / 5)     /* 5 polls / sec */
#define GPIO_PREPIC_TIMEOUT (10 * 5) /* 10 seconds */
#define GPIO_SHUTDOWN_TIMEOUT (2)    /* 400 ms. Actual event is sent between 400-600 ms*/

/* Forward declarations */
static void gpio_status_poll( void *data );
static void gpio_set_membus_speed(void);

/* Globals */
HARDWARE_STATUS gpio_hw_status;
EXPORT_SYMBOL( gpio_hw_status );
static unsigned int diskAccessStarted = 0;
static unsigned int btModeState;
static int power_button_timer = -1;
static int power_button_picreset_timer = -1;
static int ignition_timer = -1;
static int cycleDockPowerTimer = 0;
static int diskAccess = 0;

atomic_t low_dc_vcc_event_count = ATOMIC_INIT(0);
EXPORT_SYMBOL(low_dc_vcc_event_count);

#define UART_AMOUNT	4

static atomic_t uart_break_cnt[UART_AMOUNT];
static int uart_low_cnt[UART_AMOUNT] = {0,0,0,0};
static int uart_high_cnt[UART_AMOUNT] = {0,0,0,0};
static int dockdev_uartno = 0;

extern int suicide_on_rtc_wakeup;
extern int wake_on_vbus;
extern int wake_on_rtc;
extern int app_suicide_on_rtc;

/* global shared with s3cmci.c sdcard driver */
extern struct s3c_sdi_host *s3c_sdi_host_poweroff;
void s3c_sdi_poweroff(struct s3c_sdi_host *host);

/* ext func from hsmmc movinand driver */
extern void hsmmc_hw_exit(void);
extern void hsmmc0_hw_exit(void);
extern void hsmmc1_hw_exit(void);
extern void gpio_prepare_shutdown(void);
extern void s3c_sdi_emergency_poweroff(void);

/* reset state defined in arch/arm/kernel/setup.c */
extern unsigned int reset_state;

extern int bt_failed;
static wait_queue_head_t gpio_wait;
static int statusChanged = 0;
static unsigned int oldFrequency = 0;
static unsigned int membus_speed = 133;
static unsigned int old_membus_speed = 0;
static unsigned int factory_test_point_state = 0;

static int gpio_prepic_suicide(void);
static void gpio_workqueue(unsigned long data);
static struct work_struct gpio_workqueue_handle;
static atomic_t disable_gpio_wq;
DECLARE_WAIT_QUEUE_HEAD( gpio_wq_busy );

/* Functions */
void gpio_handle_break(int uart)
{
	/* Keep track of BREAKs on all UARTs */
	if (uart < UART_AMOUNT) 
		atomic_inc(&uart_break_cnt[uart]);
}

static void gpio_wq_disable( void )
{
	flush_scheduled_work( );
	atomic_set( &disable_gpio_wq, 1 );
	return;
}

static void gpio_wq_enable( void )
{
	atomic_set( &disable_gpio_wq, 0 );
	wake_up( &gpio_wq_busy );
	return;
}

static void gpio_workqueue(unsigned long data)
{
	static unsigned int oldInputStatus = 0;

	// Only do dock power cycle if dock power is available
	if (IO_HaveDockPower())
	{
		// Check if power cycle is in progress
		if (cycleDockPowerTimer > 0)
		{
			// Deactivate TMC power and UART signals so all signals on TMC connector are low
			IO_Deactivate(TMC_POWER);
			IO_Deactivate(RXD_DOCK);
			IO_Deactivate(TXD_DOCK);
		}
		else
		if (cycleDockPowerTimer == 0)
		{
			cycleDockPowerTimer = -1;
			// Enable TMC power and UART signals again and wait a while
			IO_Activate(TMC_POWER);
			IO_SetFunction(RXD_DOCK);
			IO_SetFunction(TXD_DOCK);
			mdelay(10);
		}
	}
		
	gpio_hw_status.u8DockStatus = IO_GetDockState();
	if(buspower_get_status()) gpio_hw_status.u8InputStatus |= USB_DETECT_MASK; else gpio_hw_status.u8InputStatus &= ~USB_DETECT_MASK;
	if (IO_GetInput(LIGHTS_DETECT)) gpio_hw_status.u8InputStatus |= DOCK_LIGHTS_MASK; else gpio_hw_status.u8InputStatus &= ~DOCK_LIGHTS_MASK;
	if (IO_GetInput(IGNITION)) gpio_hw_status.u8InputStatus |= DOCK_IGNITION_MASK; else gpio_hw_status.u8InputStatus &= ~DOCK_IGNITION_MASK;
	if (IO_GetInput(EXTMIC_DETECT)) gpio_hw_status.u8InputStatus |= DOCK_EXTMIC_MASK; else gpio_hw_status.u8InputStatus &= ~DOCK_EXTMIC_MASK;
	if (IO_GetInput(HEADPHONE_DETECT)) gpio_hw_status.u8InputStatus |= DOCK_HEADPHONE_MASK; else gpio_hw_status.u8InputStatus &= ~DOCK_HEADPHONE_MASK;
	if (IO_GetInput(LINEIN_DETECT)) gpio_hw_status.u8InputStatus |= DOCK_LINEIN_MASK; else gpio_hw_status.u8InputStatus &= ~DOCK_LINEIN_MASK;

	if (atomic_read(&low_dc_vcc_event_count)) {
		gpio_hw_status.u8InputStatus |= LOW_DC_VCC_PEAK_DETECT;
		// reset count to 0
		atomic_set(&low_dc_vcc_event_count, 0);
	} else {
		gpio_hw_status.u8InputStatus &= ~LOW_DC_VCC_PEAK_DETECT;
	}

	/* warning: dead time for LOW_DC_VCC peak detect */
	IO_SetInput(LOW_DC_VCC);
	if (IO_GetInput(LOW_DC_VCC)) {
		gpio_hw_status.u8InputStatus |= LOW_DC_VCC_DETECT;
	} else {
		gpio_hw_status.u8InputStatus &= ~LOW_DC_VCC_DETECT;
	}
	IO_SetInterruptOnDeactivation(LOW_DC_VCC);
	/* end dead time */

	if (IO_HasPin(UART_RXD_IPOD))
	{
		if (gpio_hw_status.u8InputStatus & DOCK_IPOD_DETECT)
		{
			int break_cnt = 0;
			/* rxd was previously detected (high) */
			/* check if any breaks occurred in UART0 driver */
			while(atomic_sub_and_test(1, &uart_break_cnt[0])) 
			{
				break_cnt++;
			}
			if (break_cnt) 
			{
				if (IO_GetInput(UART_RXD_IPOD) == 0) 
				{
					/* uart still low, good */
					gpio_hw_status.u8InputStatus &= ~DOCK_IPOD_DETECT;
				} 
			} 
			else 
			{
				/* no break counted, line assumed to be busy, most of the time high */
				if (IO_GetInput(UART_RXD_IPOD) == 0) 
				{
					uart_low_cnt[0]++;
					if (uart_low_cnt[0] > 5) 
					{
						// ca 1 sec low, disconnected
						gpio_hw_status.u8InputStatus &= ~DOCK_IPOD_DETECT;
					}
				} 
				else 
				{
					uart_low_cnt[0] = 0;
				}
			}
		}
		else
		{
			/* rxd was low (disconnected), check if came up */
			if (IO_GetInput(UART_RXD_IPOD) == 0) 
			{
				/* still low, good */
				uart_high_cnt[0] = 0;
			} 
			else 
			{
				/* line assumed to be busy or still high */
				if (uart_high_cnt[0] < 5) 
				{
					uart_high_cnt[0]++;
				} 
				else 
				{
					gpio_hw_status.u8InputStatus |= DOCK_IPOD_DETECT;
				}
			}
		}
	}

	if (IO_HasPin(UART_RXD_TMC))
	{
		if (gpio_hw_status.u8InputStatus & DOCK_TMC_DETECT)
		{
			int break_cnt = 0;
			/* rxd was previously detected (high) */
			/* check if any breaks occurred in UART0 driver */
			while(atomic_sub_and_test(1, &uart_break_cnt[1])) 
			{
				break_cnt++;
			}
			if (break_cnt) 
			{
				if (IO_GetInput(UART_RXD_TMC) == 0) 
				{
					/* uart still low, good */
					gpio_hw_status.u8InputStatus &= ~DOCK_TMC_DETECT;
				} 
			} 
			else 
			{
				/* no break counted, line assumed to be busy, most of the time high */
				if (IO_GetInput(UART_RXD_TMC) == 0) 
				{
					uart_low_cnt[1]++;
					if (uart_low_cnt[1] > 5) 
					{
						// ca 1 sec low, disconnected
						gpio_hw_status.u8InputStatus &= ~DOCK_TMC_DETECT;
					}
				} 
				else 
				{
					uart_low_cnt[1] = 0;
				}
			}
		}
		else
		{
			/* rxd was low (disconnected), check if came up */
			if (IO_GetInput(UART_RXD_TMC) == 0) 
			{
				/* still low, good */
				uart_high_cnt[1] = 0;
			} 
			else 
			{
				/* line assumed to be busy or still high */
				if (uart_high_cnt[1] < 5) 
				{
					uart_high_cnt[1]++;
				} 
				else 
				{
					gpio_hw_status.u8InputStatus |= DOCK_TMC_DETECT;
				}
			}
		}
	}
	
	/* UART RXD line test for valencia and other extended dock users - NEEDS PULLDOWN (2442) to work */
	/* optionally, for interupt-driven detection, a parallel interrupt input RXD_DOCK_INT to be there */
	if (IO_HaveRxdDetect()) 
	{
		if (gpio_hw_status.u8InputStatus & DOCK_RXD_DETECT_MASK) 
		{
			int break_cnt = 0;
			/* rxd was previously detected (high) */
			/* check if any breaks occurred in UART0 driver */
			while(atomic_sub_and_test(1, &uart_break_cnt[dockdev_uartno])) 
			{
				break_cnt++;
			}
			if (break_cnt) 
			{
				if (IO_GetInput(RXD_DOCK_INT) == 0) 
				{
					/* uart still low, good */
					gpio_hw_status.u8InputStatus &= ~DOCK_RXD_DETECT_MASK;
				} 
				else 
				{
					/* break detected, but line still high - let it debounce, uart will count another break */
				}
			} 
			else 
			{
				/* no break counted, line assumed to be busy, most of the time high */
				if (IO_GetInput(RXD_DOCK_INT) == 0) 
				{
					uart_low_cnt[dockdev_uartno]++;
					if (uart_low_cnt[dockdev_uartno] > 5) 
					{
						// ca 1 sec low, disconnected
						gpio_hw_status.u8InputStatus &= ~DOCK_RXD_DETECT_MASK;
					}
				} 
				else 
				{
					uart_low_cnt[dockdev_uartno] = 0;
				}
			}
		} 
		else 
		{
			/* rxd was low (disconnected), check if came up */
			if (IO_GetInput(RXD_DOCK_INT) == 0) 
			{
				/* still low, good */
				uart_high_cnt[dockdev_uartno] = 0;
			} 
			else 
			{
				/* line assumed to be busy or still high */
				if (uart_high_cnt[dockdev_uartno] < 5) 
				{
					uart_high_cnt[dockdev_uartno]++;
				} 
				else 
				{
					gpio_hw_status.u8InputStatus |= DOCK_RXD_DETECT_MASK;
				}
			}
		}
	} 
	else 
	{
		/* no serial detect dock */
		/* serial always active */
		gpio_hw_status.u8InputStatus |= DOCK_RXD_DETECT_MASK;
	}

	// Auto-switch off if in dock and power has been off for more than five seconds
	// Check if docked in car
	if (IO_CarDocked())
	{
		// External power?
		if (IO_GetInput(ACPWR)) 
		{
			// Enable running timer when external power is removed (wait 5 seconds)
			ignition_timer = 25;
			gpio_hw_status.u8InputStatus |= DOCK_MAINPOWER_ONOFF;
		}
		else
		{
      			gpio_hw_status.u8InputStatus &= ~DOCK_MAINPOWER_ONOFF;
			// Count down timer when external power was active while docked
			if (ignition_timer > 0)
			{
				ignition_timer--;
			}
			else
			if (ignition_timer == 0)
			{
				// Disable timer to wait for connection of external power
				ignition_timer = -1;
				// Send message to application to shut down
				if (IO_HaveGpioAutoShutdown())
					gpio_hw_status.u8InputStatus |= ONOFF_MASK;
				else
					gpio_hw_status.u8InputStatus |= DOCK_ONOFF_MASK;
			}
		}
	}
	else 
	{
		// Not docked, so disable timer
		ignition_timer = -1;
		gpio_hw_status.u8InputStatus &= ~DOCK_MAINPOWER_ONOFF;
	}
	
	// Button pressed?
	if (IO_HavePicSecShutdown()) {
		if (IO_GetInput(ON_OFF)) {
			if (power_button_picreset_timer > 0) {
				power_button_picreset_timer--;
			} else {
				/* disable timer */
				power_button_picreset_timer = -1;
				/* 2 seconds before hardware PIC reset, shutdown movinand etc */
				gpio_prepic_suicide();
				/* should never return unless connected to external power */
			}
		} else {
			power_button_picreset_timer = GPIO_PREPIC_TIMEOUT;
		}
	}

	// Button pressed?
	if (IO_GetInput(ON_OFF)) {
		// Count down timer when button was released
		if (power_button_timer > 0)
		{
			power_button_timer--;
		}
		else
		if (power_button_timer == 0)
		{
			// Disable timer to wait for button release
			power_button_timer = -1;
			// Send message to application to shut down
			gpio_hw_status.u8InputStatus |= ONOFF_MASK;
		}
	} else {
		// Enable running timer when button is released (wait 1/5 second)
		power_button_timer = GPIO_SHUTDOWN_TIMEOUT;
	}

	// HDD access?
	if (IO_GetInput(HDD_LED))
	{
		// Transition to disk access started?
		if (diskAccess == 0)
		{
			diskAccess = 1;
			diskAccessStarted = 1;
		}
	} else diskAccess = 0;
	
	if (oldInputStatus != gpio_hw_status.u8InputStatus)
	{
		oldInputStatus = gpio_hw_status.u8InputStatus;
		statusChanged = 1;
		wake_up_interruptible(&gpio_wait);
	}
}

static void gpio_status_poll( void *data )
{
	wait_event( gpio_wq_busy, (atomic_read( &disable_gpio_wq ) == 0) );
	gpio_workqueue( (unsigned long) data );
	if (cycleDockPowerTimer > 0) cycleDockPowerTimer--;
	schedule_delayed_work( &gpio_workqueue_handle, GPIO_POLL_DELAY );
}

void gpio_force_update(void)
{
	flush_scheduled_work( );
	cancel_delayed_work( &gpio_workqueue_handle );
	flush_scheduled_work( );
	schedule_work( &gpio_workqueue_handle );
}

void gpio_prepare_shutdown(void)
{
	unsigned long dummy_flags;

	/* wait 500 ms without any (sd cmd starting) int handlers */
	local_irq_save(dummy_flags);
	mdelay(500);

	/* turn off SD card / normal movinand */
	if (IO_HaveSdCardInterface()) {
		s3c_sdi_emergency_poweroff();
	}
	
	/* turn off HSMMC movinand */
	if (IO_HaveHsMmcInterface()) {
		hsmmc_hw_exit();
	}

	/* turn off HSMMC movinand channel 0 */
	if (IO_HaveHsMmcInterface0_4bit()) {
		hsmmc0_hw_exit();
	}

	/* turn off HSMMC movinand channel 1 */
	if (IO_HaveHsMmcInterface1_4bit()) {
		hsmmc1_hw_exit();
	}
}

void gpio_suicide(void)
{
	unsigned long dummy_flags;

//  To avoid a "power-drop pulse" during sucide, we do not remove the movinnand power.
//  We just wait 500ms and let the power drops smoothly after activating the PWR_RST pin.
//
//	/* note: not allowed on LTC3455 and having VBUS/extpower */
//	gpio_prepare_shutdown();

	/* on some devices, pin reset produces an audible pop throught the speaker if the amp is
	   not turned off */
	codec_suspend();

	/* wait 500 ms without any (sd cmd starting) int handlers */
	local_irq_save(dummy_flags);
	mdelay(500);

	while (1) {
		IO_Activate(PWR_RST);
	}
}

static int gpio_prepic_suicide(void)
{
	/* always do (try) the suicide, secure managed NAND flash before PIC kicks in */
	unsigned long dummy_flags;
	local_irq_save(dummy_flags);
	tomtomgo_lcdclearcolor(GREEN);
	gpio_suicide();
	return -EFAULT;
}


static int gpio_setusbvbuswakeup(int enable)
{
	if (IO_HaveUsbBusPowered()) {
		wake_on_vbus = enable ? 1 : 0;
		return 0;
	}
	return -EFAULT;
}

static int gpio_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned tmp = 0;
	int ret = 0;
	
	switch (cmd) 
	{
	case IOW_POKE_RESET_BUTTON:
		/* cannot do suicide on LTC3455 with extpower plugged in, edinburgh with PIC, or older units */
		if (IO_AllowSuicide()) {
			gpio_suicide();
			/* will not return */
		}
		ret = -EFAULT;
		break;

	case IOW_RTCALARM_SUICIDE:
		if (IO_HaveRealShutdown() && !IO_HavePicDetected()) {
			// RTC driver should have gotten the time
			// suspend should be initiated using some other call /sys/mem/state
			suicide_on_rtc_wakeup = arg ? 1 : 0;
		} else {
			printk(KERN_INFO "IOW_RTCALARM_SUICIDE not supported\n");
		}
		ret = 0;
		break;

	case IOW_RTCALARM_WAKEUP:
		wake_on_rtc = 1;
		printk(KERN_INFO "IOW_RTCALARM_WAKEUP set\n");
		break;
		
	case IOW_RTCALARM_APP_SUICIDE:
		app_suicide_on_rtc = 1;
		printk(KERN_INFO "IOW_RTCALARM_WAKEUP set\n");
		break;
		
	case IOR_HWSTATUS:
		gpio_wq_disable( );
		ret = copy_to_user((void __user *) arg, &gpio_hw_status, sizeof gpio_hw_status) ? -EFAULT : 0;
		statusChanged = 0;
		gpio_wq_enable( );
		break;

	case IOR_DISK_ACCESS:
		gpio_wq_disable( );
		ret = copy_to_user((void __user *) arg, &diskAccessStarted, sizeof diskAccessStarted) ? -EFAULT: 0;
		diskAccessStarted = 0;
		gpio_wq_enable( );
		break;
	
	case IOR_BT_MODE:
		gpio_wq_disable( );
		ret = copy_from_user(&btModeState, (void __user *) arg, sizeof btModeState) ? -EFAULT: 0;
		gpio_wq_enable( );
		/* set BT mode */
		if (btModeState & 0x01)
			IO_Activate(BT_MODE);
		else
			IO_Deactivate(BT_MODE);
		
		/* issue BT reset */
		IO_Activate(BT_RESET);
		msleep_interruptible(40); /* wait at least 5ms according to BC4ROM datasheet */
		IO_Deactivate(BT_RESET);
		msleep_interruptible(40);
		IO_Activate(BT_RESET);
		msleep_interruptible(40);
		IO_Deactivate(BT_RESET);
		msleep_interruptible(100);
		break;

	case IOR_GET_BT_ERROR:
		ret = copy_to_user((void __user *) arg, &bt_failed, sizeof bt_failed) ? -EFAULT : 0;
		break;

	case IOR_SET_BT_ERROR:
		ret = copy_from_user(&bt_failed, (void __user *) arg, sizeof bt_failed) ? -EFAULT: 0;
		break;

	case IOW_RESET_ONOFF_STATE:
		gpio_wq_disable( );
		gpio_hw_status.u8InputStatus &= (~(ONOFF_MASK));
		gpio_wq_enable( );
		break;

	case IOW_ENABLE_DOCK_UART:
		{
			static unsigned int uartEnableState;
			gpio_wq_disable( );
			ret = copy_from_user(&uartEnableState, (void __user *) arg, sizeof uartEnableState) ? -EFAULT: 0;
			gpio_wq_enable( );
			if (uartEnableState) 
			{
				IO_SetFunction(TXD_DOCK);
				IO_SetFunction(RXD_DOCK);
			}
			else
			{
				IO_SetInput(TXD_DOCK);
				IO_SetInput(RXD_DOCK);
			}
		}
		break;

	case OBSOLETE_IOW_SET_FM_FREQUENCY:
			return -ENOSYS;
			//ret = copy_from_user(&frequency, (void __user *) arg, sizeof frequency) ? -EFAULT: 0;
		break;

	case IOW_SET_MEMBUS_SPEED:
			ret = copy_from_user(&membus_speed, (void __user *) arg, sizeof membus_speed) ? -EFAULT: 0;
			gpio_set_membus_speed();
		break;
		
	case IOW_CYCLE_DOCK_POWER:
			gpio_wq_disable( );
			cycleDockPowerTimer = 10; // Cycle power for 2 seconds
			gpio_wq_enable( );
		break;
		
	case IOW_USB_VBUS_WAKEUP:
		ret = copy_from_user(&tmp, (void __user *) arg, sizeof tmp) ? -EFAULT: 0;
		if (ret == 0) ret = gpio_setusbvbuswakeup(tmp);
		break;

	case IOW_KRAKOW_RDSTMC_HACK_VBUSOFF:
		/* IO_Deactivate(USB_HOST_DETECT); */
		rds_tmc_hack_enabled = 1;
		break;

	case IOW_KRAKOW_RDSTMC_HACK_VBUSINP:
		/* IO_SetInput(USB_HOST_DETECT); */
		rds_tmc_hack_enabled = 0;
		break;
	case IOW_FORCE_LOW_DC_VCC_HIGH:
		/* turn amplifier on on milan/modena/parma */
		if (IO_HaveAmpShutdownOnLowDcVCC()) {
			IO_MakeHigh(LOW_DC_VCC);
		}
		break;
	case IOW_FORCE_LOW_DC_VCC_IRQ:
		/* Let battery driver use the pin again as IRQ source */
		IO_SetInterruptOnDeactivation(LOW_DC_VCC);
		break;
	default:
		PK_WARN("Invalid ioctl command %u\n", cmd);
		ret = -EINVAL;
		break;
		
	case IOW_FACTORY_TEST_POINT:
		ret = copy_from_user(&factory_test_point_state, (void __user *) arg, sizeof factory_test_point_state) ? -EFAULT: 0;
		if (factory_test_point_state) {
			IO_Activate(FACTORY_TEST_POINT);
		} else {
			IO_Deactivate(FACTORY_TEST_POINT);
		}
		break;
	}

	return ret;
}

/* normal: 266/133/66 */
#define S3C2412_HDIVN	1
#define S3C2412_PDIVN	1
#define S3C2412_MDIV	125
#define S3C2412_PDIV	4
#define S3C2412_SDIV	1
#define S3C2412_HCLKDIV	2
#define S3C2412_PCLKDIV	4
#define S3C2412_ARMDIV  0
/* variant: 266/66/66 */
#define S3C2412_ARMDIV_SLOW 0
#define S3C2412_HDIVN_SLOW  3
#define S3C2412_PDIVN_SLOW	0

#define S3C2412_REFRESH_133 1037
#define S3C2412_REFRESH_66  514

static void gpio_set_membus_speed(void)
{
	unsigned int clkdivn, refresh;
	
	clkdivn =__raw_readl(S3C2410_CLKDIVN);
	clkdivn &= ~(0xf); // clear ARMDIV/HDIVN/PDIVN bits
	
	switch (membus_speed) 
	{
		default:
			PK_INFO("%d: invalid memory speed\n", membus_speed);
			return;
		case 133:
			// 266/133/66
			clkdivn |= (S3C2412_ARMDIV << 3) | (S3C2412_PDIVN<<2) | S3C2412_HDIVN;
			refresh = S3C2412_REFRESH_133;
			PK_INFO("%d Mhz memory speed\n", membus_speed);
			// 200 mA
			break;
		case 66:
			// 266/66/66
			clkdivn |= (S3C2412_ARMDIV_SLOW << 3) | (S3C2412_PDIVN_SLOW <<2) | S3C2412_HDIVN_SLOW;
			refresh = S3C2412_REFRESH_66;
			PK_INFO("%d Mhz memory speed\n", membus_speed);
			break;
	}

	if (membus_speed > old_membus_speed) 
	{
		// going faster, set new refresh first
		__raw_writel(refresh, S3C2412_REFRESH); /* Clock divider */
		__raw_writel(clkdivn, S3C2410_CLKDIVN); /* Clock divider */
	} 
	else 
	{
		// going slower, set new refresh first
		__raw_writel(clkdivn, S3C2410_CLKDIVN); /* Clock divider */
		__raw_writel(refresh, S3C2412_REFRESH); /* Clock divider */
	}
 
	old_membus_speed = membus_speed;
}

static int gpio_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int gpio_release(struct inode *inode, struct file *file)
{
	return 0;
}

static unsigned int gpio_poll(struct file *file, struct poll_table_struct *wait)
{
	unsigned int ret;

	poll_wait(file, &gpio_wait, wait);

	gpio_wq_disable( );
	ret = (statusChanged == 0) ? 0 : (POLLIN | POLLRDNORM);
	gpio_wq_enable( );

	return ret;
}

/* Kernel interface */
static struct file_operations gpio_fops = 
{
	.owner		= THIS_MODULE,
	.ioctl		= gpio_ioctl,
	.open		= gpio_open,
	.release	= gpio_release,
	.poll		= gpio_poll,
};

static void gpio_hw_init(void)
{
	IO_SetInput(ON_OFF);
	IO_SetInput(IGNITION);
	IO_SetInput(ACPWR);
	IO_SetInput(CHARGEFAULT);
	IO_SetInput(CHARGING);
	IO_SetInput(LIGHTS_DETECT);
	IO_SetInput(HEADPHONE_DETECT);
	IO_SetInput(EXTMIC_DETECT);
	IO_SetInput(LINEIN_DETECT);
	IO_SetInput(DOCK_SENSE);
	IO_SetInput(DOCK_DESK_SENSE);
	IO_SetInput(USB_HOST_DETECT);
	IO_SetInput(RXD_DOCK_INT);
	IO_SetInput(UART_RXD_IPOD);
	IO_SetInput(UART_RXD_TMC);
	IO_SetInput(DOCK_MOTOR_SENSE);
	IO_SetInput(DOCK_RADIO_SENSE);
	IO_SetInput(DOCK_CRIB_SENSE);
	IO_SetInput(DOCK_VIB_SENSE);

	// Stop timers
	power_button_timer = -1;
	ignition_timer = -1;
	// Clear state from before suspend
	gpio_hw_status.u8InputStatus = 0;
	// Force update of transmitter if docked
	oldFrequency = 0;
	atomic_set(&uart_break_cnt[0], 0);
	atomic_set(&uart_break_cnt[1], 0);
	// Force TMC/dock power to be enabled
	cycleDockPowerTimer = 0;
	
	/* Determine UART # for dock device */
	if (strncmp(IO_GetDockDevice(), "ttySAC", 6) == 0) {
		/* FIXME: Break detection should be moved to TTPnPD! */
		dockdev_uartno = IO_GetDockDevice()[6] - '0';

		printk("%s: UART '%s' => %d\n", __func__, IO_GetDockDevice(), dockdev_uartno);
	}
}

static void gpio_hw_exit(void)
{
	flush_scheduled_work( );
	cancel_delayed_work( &gpio_workqueue_handle );
	flush_scheduled_work( );
}

static int gpio_probe(struct device *dev)
{
	int ret;

	PK_DBG("Initializing wait queue\n");
	init_waitqueue_head(&gpio_wait);

	gpio_hw_init();
		
	// Get new state
	INIT_WORK(&gpio_workqueue_handle, gpio_status_poll, NULL);
	schedule_work( &gpio_workqueue_handle );
	atomic_set( &disable_gpio_wq, 0 );
	
	PK_DBG("Registering chardev\n");
	ret = register_chrdev(GPIO_MAJOR, GPIO_DEVNAME, &gpio_fops);
	if (ret < 0) 
	{
		printk("Unable to register chardev on major=%d (%d)\n", GPIO_MAJOR, ret);
		return ret;
	}
		
	PK_DBG("Done\n");
	return 0;
}

static int gpio_remove(struct device *dev)
{
	PK_DBG("Unregistering chardev\n");
	unregister_chrdev(GPIO_MAJOR, GPIO_DEVNAME);
	gpio_hw_exit();

	PK_DBG("Done\n");
	return 0;
}

static void gpio_shutdown(struct device *dev)
{
	PK_DBG("Shutting down\n");
	gpio_hw_exit();
}

#ifdef CONFIG_PM

static int gpio_suspend(struct device *dev, u32 state, u32 level)
{
	PK_DBG("dev = %p, state = %u, level = %u\n", dev, state, level);	
	if (level == SUSPEND_POWER_DOWN) 
	{
		gpio_hw_exit( );
	}
	return 0;
}

static int gpio_resume(struct device *dev, u32 level)
{
	PK_DBG("dev = %p, level = %u\n", dev, level);	
	if (level == RESUME_POWER_ON) 
	{ 
		gpio_hw_init( );
		atomic_set(&low_dc_vcc_event_count, 0);
		schedule_work( &gpio_workqueue_handle );
	}
	return 0;
}

#else /* CONFIG_PM */
#define gpio_suspend NULL
#define gpio_resume  NULL
#endif /* CONFIG_PM */

static struct device_driver gpio_driver = 
{
	.name		= "tomtomgo-gpio",
	.bus		= &platform_bus_type,
	.probe		= gpio_probe,
	.remove		= gpio_remove,
	.shutdown	= gpio_shutdown,
	.suspend	= gpio_suspend,
	.resume		= gpio_resume,
};

static int __init gpio_mod_init(void)
{
	int ret;

	printk(KERN_INFO "TomTom GO GPIO Driver, (C) 2004,2005 TomTom BV\n");
	PK_DBG("Registering driver\n");
	ret = driver_register(&gpio_driver);
	if (ret) 
	{
		PK_ERR("Unable to register driver (%d)\n", ret);
		return ret;
	}
	PK_DBG("Done\n");
	return 0;
}

static void __exit gpio_mod_exit(void)
{
	PK_DBG("Unregistering driver\n");
	driver_unregister(&gpio_driver);
	PK_DBG("Done\n");
}

module_init(gpio_mod_init);
module_exit(gpio_mod_exit);

MODULE_AUTHOR("Jeroen Taverne <jeroen.taverne@tomtom.com> and Dimitry Andric <dimitry.andric@tomtom.com> and Koen Martens <kmartens@sonologic.nl>");
MODULE_DESCRIPTION("TomTom GO GPIO Driver");
MODULE_LICENSE("GPL");

/* EOF */
