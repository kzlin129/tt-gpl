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
*  @file    amixer_ioctl.h
*
*  @brief   Audio Mixer User IOCTL API definitions
*
*****************************************************************************/
#if !defined( AMIXER_IOCTL_H )
#define AMIXER_IOCTL_H

/* ---- Include Files ---------------------------------------------------- */

#if defined( __KERNEL__ )
#include <linux/types.h>            /* Needed for standard types */
#else
#include <stdint.h>
#endif

#include <linux/ioctl.h>
#include <linux/broadcom/amixer.h>

/* ---- Constants and Types ---------------------------------------------- */

/* Type define used to create unique IOCTL number */
#define AMIXER_MAGIC_TYPE              'M'

/* IOCTL commands */
enum amixer_cmd_e
{
   AMIXER_CMD_QUERY_PORT = 0x30,    /* Start at 0x30 arbitrarily */
   AMIXER_CMD_GET_PORT_INFO,
   AMIXER_CMD_GET_INFO,
   AMIXER_CMD_SET_CNX_LOSS,
   AMIXER_CMD_GET_CNX_LOSS,
   AMIXER_CMD_CONNECT,
   AMIXER_CMD_DISCONNECT,
   AMIXER_CMD_LAST                  /* Do no delete */
};

/* IOCTL Data structures */
struct amixer_ioctl_queryport
{
   char              name[32];      /*<< (i) User name to query with */
   AMIXER_PORT_ID   *id;            /*<< (o) Ptr to store found port ID */
};

struct amixer_ioctl_queryportinfo
{
   AMIXER_PORT_ID    port;          /*<< (i) Port id */
   AMIXER_PORT_INFO *info;          /*<< (o) Ptr to port info structure */
};

struct amixer_ioctl_setcnxloss
{
   AMIXER_PORT_ID    src_port;      /*<< (i) source port id */
   AMIXER_PORT_ID    dst_port;      /*<< (i) destination port id */
   unsigned int      db;            /*<< (i) Loss amount in dB */
};

struct amixer_ioctl_getcnxloss
{
   AMIXER_PORT_ID    src_port;      /*<< (i) source port id */
   AMIXER_PORT_ID    dst_port;      /*<< (i) destination port id */
   unsigned int     *db;            /*<< (i) Loss amount in dB */
};

struct amixer_ioctl_connect
{
   AMIXER_PORT_ID       src_port;   /*<< (i) source port id */
   AMIXER_PORT_ID       dst_port;   /*<< (i) destination port id */
   AMIXER_CONNECT_DESC  desc;       /*<< (i) Connection descriptor */
};

struct amixer_ioctl_disconnect
{
   AMIXER_PORT_ID       src_port;   /*<< (i) source port id */
   AMIXER_PORT_ID       dst_port;   /*<< (i) destination port id */
};

/* IOCTL numbers */
#define AMIXER_IOCTL_QUERY_PORT        _IOR( AMIXER_MAGIC_TYPE, AMIXER_CMD_QUERY_PORT, struct amixer_ioctl_queryport * )
#define AMIXER_IOCTL_GET_PORT_INFO     _IOR( AMIXER_MAGIC_TYPE, AMIXER_CMD_GET_PORT_INFO, struct amixer_ioctl_queryportinfo * )
#define AMIXER_IOCTL_GET_INFO          _IOR( AMIXER_MAGIC_TYPE, AMIXER_CMD_GET_INFO, AMIXER_INFO * )
#define AMIXER_IOCTL_SET_CNX_LOSS      _IOW( AMIXER_MAGIC_TYPE, AMIXER_CMD_SET_CNX_LOSS, struct amixer_ioctl_setcnxloss * )
#define AMIXER_IOCTL_GET_CNX_LOSS      _IOR( AMIXER_MAGIC_TYPE, AMIXER_CMD_GET_CNX_LOSS, struct amixer_ioctl_getcnxloss * )
#define AMIXER_IOCTL_CONNECT           _IOW( AMIXER_MAGIC_TYPE, AMIXER_CMD_CONNECT, struct amixer_ioctl_connect * )
#define AMIXER_IOCTL_DISCONNECT        _IOW( AMIXER_MAGIC_TYPE, AMIXER_CMD_DISCONNECT, struct amixer_ioctl_disconnect * )

/* ---- Variable Externs ------------------------------------------ */
/* ---- Function Prototypes --------------------------------------- */


#endif /* AMIXER_IOCTL_H */
