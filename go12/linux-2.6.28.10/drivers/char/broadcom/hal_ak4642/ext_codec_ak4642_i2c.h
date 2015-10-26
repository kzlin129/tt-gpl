/*****************************************************************************
* Copyright 2005 - 2008 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/




/*
*
*****************************************************************************
*
*  external_codec_ak4642_ic.h
*
*  PURPOSE:
*
*     This file contains definitions for the I2C Driver for the AK4642.
*
*  NOTES:
*
*****************************************************************************/


#ifndef EXTERNAL_CODEC_AK4642_I2C_H    /* support nested includes */
#define EXTERNAL_CODEC_AK4642_I2C_H

/* ---- Include Files ---------------------------------------- */
/* ---- Constants and Types ---------------------------------- */
/* ---- Variable Externs ------------------------------------- */
/* ---- Function Prototypes ---------------------------------- */

int ext_codec_ak4642_ic_init(void);
int ext_codec_ak4642_ic_release(void);
int ext_codec_ak4642_i2c_read(unsigned char reg);
int ext_codec_ak4642_i2c_write(unsigned char reg, unsigned char value);

#endif   /* EXTERNAL_CODEC_AK4642_I2C_H */

