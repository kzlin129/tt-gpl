/* OmniVision* camera chip driver API
 *
 * Copyright (c) 1999-2006 Mark McClelland <mark@ovcam.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version. NO WARRANTY OF ANY KIND is expressed or implied.
 *
 * * OmniVision is a trademark of OmniVision Technologies, Inc. This driver
 * is not sponsored or developed by them.
 */

#ifndef __LINUX_OVCAMCHIP_H
#define __LINUX_OVCAMCHIP_H

#include <linux/videodev.h>
#include <linux/i2c.h>

/* --------------------------------- */
/*           ENUMERATIONS            */
/* --------------------------------- */

/* Controls */
enum {
	OVCAMCHIP_CID_CONT,		/* Contrast */
	OVCAMCHIP_CID_BRIGHT,		/* Brightness */
	OVCAMCHIP_CID_SAT,		/* Saturation */
	OVCAMCHIP_CID_HUE,		/* Hue */
	OVCAMCHIP_CID_EXP,		/* Exposure */
	OVCAMCHIP_CID_FREQ,		/* Light frequency */
	OVCAMCHIP_CID_BANDFILT,		/* Banding filter */
	OVCAMCHIP_CID_AUTOBRIGHT,	/* Auto brightness */
	OVCAMCHIP_CID_AUTOEXP,		/* Auto exposure */
	OVCAMCHIP_CID_BACKLIGHT,	/* Back light compensation */
	OVCAMCHIP_CID_MIRROR,		/* Mirror horizontally */
};

#if defined(HAVE_V4L2)
/* Don't use these values from userspace!! They are translated by the
 * upper-layer driver, and are not guaranteed to remain stable anyway. Instead,
 * find private controls by name. */
#define OVCAMCHIP_V4L2_CID_AEC     (V4L2_CID_PRIVATE_BASE + 0)
#define OVCAMCHIP_V4L2_CID_LASTP1  (V4L2_CID_PRIVATE_BASE + 1) /* Last + 1 */
#endif

/* Chip types */
#define NUM_CC_TYPES	11
enum {
	CC_UNKNOWN,
	CC_OV76BE,
	CC_OV7610,
	CC_OV7620,
	CC_OV7620AE,
	CC_OV6620,
	CC_OV6630,
	CC_OV6630AE,
	CC_OV6630AF,
	CC_OV9655_REV4,
	CC_OV9655_REV5,
};

/* --------------------------------- */
/*           I2C ADDRESSES           */
/* --------------------------------- */

#define OV6xx0_SID   (0xC0 >> 1)
#define OV7xx0_SID   (0x42 >> 1)
#define OV8xx0_SID   (0xA0 >> 1)
#define OV_HIRES_SID (0x60 >> 1)	/* OV9xxx / OV2xxx / OV3xxx */

/* --------------------------------- */
/*                API                */
/* --------------------------------- */

struct ovcamchip_control {
	__u32 id;
	__s32 value;
};

struct ovcamchip_window {
	int x;
	int y;
	int width;
	int height;
	int format;
	int quarter;		/* Scale width and height down 2x */

	/* This stuff will be removed eventually */
	int clockdiv;		/* Clock divisor setting */
};

struct ovcamchip_reg
{
	__u8	reg;
	__u8	val;
	__u8	mask;
};

/* Commands */
#define OVCAMCHIP_CMD_Q_SUBTYPE     _IOR  (0x88, 0x00, int)
#define OVCAMCHIP_CMD_INITIALIZE    _IOW  (0x88, 0x01, int)
/* You must call OVCAMCHIP_CMD_INITIALIZE before any of commands below! */
#define OVCAMCHIP_CMD_S_CTRL        _IOW  (0x88, 0x02, struct ovcamchip_control)
#define OVCAMCHIP_CMD_G_CTRL        _IOWR (0x88, 0x03, struct ovcamchip_control)
#define OVCAMCHIP_CMD_S_MODE        _IOW  (0x88, 0x04, struct ovcamchip_window)
#define OVCAMCHIP_CAM_SLEEP         _IO   (0x88, 0x05)
#define OVCAMCHIP_CAM_WAKE          _IO   (0x88, 0x06)
#define OVCAMCHIP_CAM_SUSPEND       _IO   (0x88, 0x07)
#define OVCAMCHIP_CAM_RESUME        _IO   (0x88, 0x08)
#define OVCAMCHIP_CAM_READREG       _IOWR (0x88, 0x09, struct ovcamchip_reg)
#define OVCAMCHIP_CAM_WRITEREG      _IOWR (0x88, 0x0A, struct ovcamchip_reg)
#define OVCAMCHIP_MAX_CMD           _IO   (0x88, 0x3f)

#endif
