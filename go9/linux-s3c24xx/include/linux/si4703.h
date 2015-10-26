/*
 * Driver for the si4703 FM/RDS/TMC Receiver
 *
 *	This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 *
 *  Author:      Xander.Hover, <Xander.Hover@tomtom.com>
 */

#ifndef __INCLUDE_LINUX_SI4703_H
#    define __INCLUDE_LINUX_SI4703_H       __FILE__

#    include <linux/ioctl.h>
#    include <linux/major.h>
#    include <linux/fmreceiver.h>

#    define SI4703_DEVNAME  		"SI4703"

#    define SI4703_MAJOR    		FMRECEIVER_MAJOR

/*#    define SI4703_MINOR    		(FMRECEIVER_MINOR + 1) */
#    define SI4703_MINOR    		FMRECEIVER_MINOR

#    define SI4703_I2C_SLAVE_ADDR         (0x10)

#endif				/* __INCLUDE_LINUX_SI4703_H */
