/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
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
/**
*
*  @file    halaudio_ioctl.h
*
*  @brief   HAL Audio User IOCTL API definitions
*
*****************************************************************************/
#if !defined( HALAUDIO_IOCTL_H )
#define HALAUDIO_IOCTL_H

/* ---- Include Files ---------------------------------------------------- */

#if defined( __KERNEL__ )
#include <linux/types.h>            /* Needed for standard types */
#else
#include <stdint.h>
#endif

#include <linux/ioctl.h>
#include <linux/broadcom/halaudio_new.h>

/* ---- Constants and Types ---------------------------------------------- */

/* Type define used to create unique IOCTL number */
#define HALAUDIO_MAGIC_TYPE                  'A'

/* IOCTL commands */
enum halaudio_cmd_e
{
   HALAUDIO_CMD_LOCK_CODEC = 0x30,     /* Start at 0x30 arbitrarily */
   HALAUDIO_CMD_UNLOCK_CODEC,
   HALAUDIO_CMD_GET_GAIN_INFO,
   HALAUDIO_CMD_SET_GAIN,
   HALAUDIO_CMD_ALTER_GAIN,
   HALAUDIO_CMD_SET_POWER,
   HALAUDIO_CMD_GET_POWER,
   HALAUDIO_CMD_READ_PARMS,
   HALAUDIO_CMD_WRITE_PARMS,
   HALAUDIO_CMD_SET_SUPERUSER,
   HALAUDIO_CMD_SET_FREQ,
   HALAUDIO_CMD_GET_FREQ,
   HALAUDIO_CMD_SET_EQU_PARMS,
   HALAUDIO_CMD_GET_EQU_PARMS,
   HALAUDIO_CMD_GET_HARDWARE_INFO,
   HALAUDIO_CMD_GET_INTERFACE_INFO,
   HALAUDIO_CMD_GET_CODEC_INFO,
   HALAUDIO_CMD_ENABLE_INTERFACE,
   HALAUDIO_CMD_LAST                   /* Do no delete */
};

/* IOCTL Data structures */
struct halaudio_ioctl_getgaininfo
{
   HALAUDIO_BLOCK      block;          /*<< (i) Block id for digital or analog gain */
   HALAUDIO_GAIN_INFO *info;           /*<< (o) Pointer to returned gain information */
};

struct halaudio_ioctl_setgain
{
   HALAUDIO_BLOCK  block;              /*<< (i) Block id for digital or analog gain */
   int             db;                 /*<< (i) Gain in dB to set */
};

struct halaudio_ioctl_altergain
{
   HALAUDIO_BLOCK  block;              /*<< (i) Block id for digital or analog gain */
   int             delta;              /*<< (i) Number of notches to adjust gain by in unitless notches */
};   

struct halaudio_ioctl_rw_parms
{
   HALAUDIO_CODEC cid;                 /*<< (i) Codec channel */
   HALAUDIO_FMT   format;              /*<< (i) Format of samples */
};

struct halaudio_ioctl_setfreq
{
   HALAUDIO_CODEC cid;                 /*<< (i) Codec channel to write to */
   int            freqhz;              /*<< (i) Sampling frequency in Hz to set */
};

struct halaudio_ioctl_getfreq
{
   HALAUDIO_CODEC cid;                 /*<< (i) Codec channel to write to */
   int           *freqhz;              /*<< (o) Ptr to store sampling freq */
};

struct halaudio_ioctl_setequparms
{
   HALAUDIO_CODEC       cid;           /*<< (i) Codec to write to */
   HALAUDIO_DIR         dir;           /*<< (i) Select equalizer direction */
   const HALAUDIO_EQU  *equ;           /*<< (i) Pointer to equalizer parameters */
};

struct halaudio_ioctl_getequparms
{
   HALAUDIO_CODEC       cid;           /*<< (i) Codec channel to write to */
   HALAUDIO_DIR         dir;           /*<< (i) Select equalizer direction */
   HALAUDIO_EQU        *equ;           /*<< (o) Ptr to store equalizer parameters */
};

struct halaudio_ioctl_getinterfaceinfo
{
   HALAUDIO_IF          interface;     /*<< (i) Interface number */
   HALAUDIO_IF_INFO    *info;          /*<< (o) Ptr to store interface info */
};

struct halaudio_ioctl_getcodecinfo
{
   HALAUDIO_CODEC       cid;           /*<< (i) Codec channel to write to */
   HALAUDIO_CODEC_INFO *info;          /*<< (o) Ptr to store codec info */
};

struct halaudio_ioctl_enableinterface
{
   HALAUDIO_IF          interface;     /*<< (i) Interface number */
   int                  enable;        /*<< (i) 1 to enable, 0 to disable */
};

/* IOCTL numbers */
#define HALAUDIO_IOCTL_LOCK_CODEC         _IOW( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_LOCK_CODEC, int )
#define HALAUDIO_IOCTL_UNLOCK_CODEC       _IOW( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_UNLOCK_CODEC, int )
#define HALAUDIO_IOCTL_GET_GAIN_INFO      _IOR( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_GET_GAIN_INFO, struct halaudio_ioctl_getgaininfo * )
#define HALAUDIO_IOCTL_SET_GAIN           _IOW( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_SET_GAIN, struct halaudio_ioctl_setgain * )
#define HALAUDIO_IOCTL_ALTER_GAIN         _IOW( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_ALTER_GAIN, struct halaudio_ioctl_altergain * )
#define HALAUDIO_IOCTL_SET_POWER          _IOW( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_SET_POWER, int )
#define HALAUDIO_IOCTL_GET_POWER          _IOR( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_GET_POWER, int * )
#define HALAUDIO_IOCTL_READ_PARMS         _IOW( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_READ_PARMS, struct halaudio_ioctl_rw_parms * )
#define HALAUDIO_IOCTL_WRITE_PARMS        _IOW( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_WRITE_PARMS, struct halaudio_ioctl_rw_parms * )
#define HALAUDIO_IOCTL_SET_SUPERUSER      _IOW( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_SET_SUPERUSER, int )
#define HALAUDIO_IOCTL_SET_FREQ           _IOW( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_SET_FREQ, struct halaudio_ioctl_setfreq * )
#define HALAUDIO_IOCTL_GET_FREQ           _IOR( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_GET_FREQ, struct halaudio_ioctl_getfreq * )
#define HALAUDIO_IOCTL_SET_EQU_PARMS      _IOW( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_SET_EQU_PARMS, struct halaudio_ioctl_setequparms * )
#define HALAUDIO_IOCTL_GET_EQU_PARMS      _IOR( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_GET_EQU_PARMS, struct halaudio_ioctl_getequparms * )
#define HALAUDIO_IOCTL_GET_HARDWARE_INFO  _IOR( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_GET_HARDWARE_INFO, HALAUDIO_HW_INFO * )
#define HALAUDIO_IOCTL_GET_INTERFACE_INFO _IOR( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_GET_INTERFACE_INFO, struct halaudio_ioctl_getinterfaceinfo * )
#define HALAUDIO_IOCTL_GET_CODEC_INFO     _IOR( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_GET_CODEC_INFO, struct halaudio_ioctl_getcodecinfo * )
#define HALAUDIO_IOCTL_ENABLE_INTERFACE   _IOW( HALAUDIO_MAGIC_TYPE, HALAUDIO_CMD_ENABLE_INTERFACE, struct halaudio_ioctl_enableinterface * )

/* ---- Variable Externs ------------------------------------------ */
/* ---- Function Prototypes --------------------------------------- */


#endif /* HALAUDIO_IOCTL_H */
