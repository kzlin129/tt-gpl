/* arch/arm/mach-s3c2410/tomtomgo-ioexp.c
 *
 * Implementation of the IO expander driver.
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Author: Jeroen Taverne <jeroen.taverne@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef __KERNEL__
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
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/hardware/clock.h>
#include <asm/arch/regs-gpio.h>
#include <barcelona/Barc_Gpio.h>
#include <barcelona/gopins.h>
#include <barcelona/debug.h>
#include <barcelona/dock-iic.h>
#endif /* __KERNEL__ */

#ifdef __BOOTLOADER__
#include "compiler.h"
#include "timer.h"
#include "gopins.h"
#include "dock-iic.h"
#define EXPORT_SYMBOL(x)
#endif  /* __BOOTLOADER__ */

#define IOEXP_ADDRESS 	0x40
#define IOEXP_IODIR 	0x00
#define IOEXP_IPOL  	0x01
#define IOEXP_GPINTEN 	0x02
#define IOEXP_DEFVAL 	0x03
#define IOEXP_INTCON 	0x04
#define IOEXP_IOCON 	0x05
#define IOEXP_GPPU 		0x06
#define IOEXP_INTF 		0x07
#define IOEXP_INTCAP 	0x08
#define IOEXP_GPIO 		0x09
#define IOEXP_OLAT 		0x0a

#define IOEXP_DOCK_CRIB_SENSE	(1 << 0)
#define IOEXP_MUTE_EXT			(1 << 2)
#define IOEXP_HEADPHONE_DETECT	(1 << 3)
#define IOEXP_LINEIN_DETECT		(1 << 4)
#define IOEXP_EXTMIC_DETECT		(1 << 5)
#define IOEXP_LIGHTS_DETECT		(1 << 6)
#define IOEXP_TMC_POWER			(1 << 7)
#define IOEXP_USB_HOST_DETECT	(1 << 8)
#define IOEXP_DOCK_SENSE		(1 << 9)
#define IOEXP_DOCK_DESK_SENSE	(1 << 10)
#define IOEXP_DOCK_VIB_SENSE	(1 << 11)

#define IOEXP_INPUT_MASK ((unsigned char) ~(IOEXP_MUTE_EXT | IOEXP_TMC_POWER))
#define IOEXP_PULLUP_MASK ((unsigned char) ~(IOEXP_EXTMIC_DETECT | IOEXP_MUTE_EXT | IOEXP_TMC_POWER))

unsigned int IOEXP_InputOutput(unsigned reqOutputValue)
{
	unsigned int result = 0xff;

	IO_SetInput(SW_SCL);
	IO_SetInput(DOCK_INT);

	if (IO_GetInput(SW_SCL))
	{
		IO_SetInput(SW_SDA);
		if (IO_GetInput(SW_SDA))
		{
			#ifndef DONT_USE_IOEXPANDER
			if (IO_GetInput(DOCK_INT))
			{
				int expanderDetected = 0;
				IOIIC_Start();
				if (IOIIC_Transmit(IOEXP_ADDRESS)) 
				{
					expanderDetected = 1; // any of &= will reset expanderDetected
					expanderDetected &= IOIIC_Transmit(IOEXP_IODIR);		// Start with first register
					expanderDetected &= IOIIC_Transmit(IOEXP_INPUT_MASK);	// IODIR	Set input pins
					expanderDetected &= IOIIC_Transmit(0x00);				// IPOL		No pin inversion
					expanderDetected &= IOIIC_Transmit(0x00);				// GPINTEN	Set interrupt on inputs
					expanderDetected &= IOIIC_Transmit(0x00);				// DEFVAL	Reset default value
					expanderDetected &= IOIIC_Transmit(0x00);				// INTCON	Compare with last pin value
					expanderDetected &= IOIIC_Transmit(0x00);				// IOCON	Active low interrupt (DOCK_INT will be low when no change is detected)
					expanderDetected &= IOIIC_Transmit(IOEXP_PULLUP_MASK);	// GPPU		Enable pullups for input pins
					expanderDetected &= IOIIC_Transmit(0x00);				// INTF		Read only register
					expanderDetected &= IOIIC_Transmit(0x00);				// INTCAP	Read only register
					expanderDetected &= IOIIC_Transmit(reqOutputValue);		// GPIO		Send output data to GPIO register
					IOIIC_Stop();
					IOIIC_Start();
					expanderDetected &= IOIIC_Transmit(IOEXP_ADDRESS);		// Send chip address
					expanderDetected &= IOIIC_Transmit(IOEXP_GPIO);			// GPIO	Select GPIO register
					IOIIC_Stop();
					IOIIC_Start();
					expanderDetected &= IOIIC_Transmit(IOEXP_ADDRESS+1);	// Send chip address
					
					result = IOIIC_Receive(0);			// Receive GPIO data
					result |= (1 << 9);					// Add virtual DOCK_SENSE pin signal
				}
				IOIIC_Stop();
				if (!expanderDetected) result = 0xff;
			}
			#endif //DONT_USE_IOEXPANDER
		}
		else
		{
			// Desk dock detected
			result |= IOEXP_DOCK_DESK_SENSE; // add virtual DOCK_DESK_SENSE
			if (IO_GetInput(DOCK_INT)) result |= IOEXP_USB_HOST_DETECT; // add virtual USB_HOST_DETECT
		}
	}
	else
	if (!IO_GetInput(SW_SCL))
	{
		// VIB detected
		result |= IOEXP_DOCK_VIB_SENSE; // add virtual DOCK_VIB_SENSE
		if (IO_GetInput(DOCK_INT)) result &= ~IOEXP_LIGHTS_DETECT; // add virtual LIGHTS_DETECT
		if (reqOutputValue & IOEXP_MUTE_EXT) IO_Deactivate(SW_SDA); else IO_SetInput(SW_SDA);
	}
	
	return result;
}
