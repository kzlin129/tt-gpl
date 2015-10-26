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

#ifndef LINUX_OTP_H
#define LINUX_OTP_H

#include <linux/ioctl.h>

#define OTP_MAGIC    'o'

#define OTP_NOT_SUPPORTED  0xFF

#define OTP_MAX_APM_CH     3
#define OTP_MAX_ETH_PHY    2
#define OTP_MAX_EHSS       2
#define OTP_MAX_USB        2
#define OTP_MAX_SDIO       2

typedef enum
{
   OTP_CMD_INFO_GET             = 0x80,
   /* insert new ioctls here */
} OTP_CMD;

typedef struct
{
   unsigned char chip_id;
   unsigned char max_vpm_clk;
   unsigned char max_cpu_clk;
   unsigned char lcd_res;
   unsigned char apm_ch[OTP_MAX_APM_CH];
   unsigned char eth_phy[OTP_MAX_ETH_PHY];
   unsigned char eth_gmii[OTP_MAX_ETH_PHY];
   unsigned char eth_sgmii[OTP_MAX_ETH_PHY];
   unsigned char ext_rgmii;
   unsigned char vpm;
   unsigned char ehss[OTP_MAX_EHSS];
   unsigned char uart;
   unsigned char touch_screen;
   unsigned char keypad;
   unsigned char led_matrix;
   unsigned char usb[OTP_MAX_USB];
   unsigned char sdio[OTP_MAX_SDIO];
   unsigned char crypto;
   unsigned char hvg_rfg_apm;
   unsigned char lcd;
   unsigned char ge;
   unsigned char vdec;
   unsigned char pif;
   unsigned char bbl;
} OTP_INFO;

#define OTP_IOCTL_INFO_GET   _IOR(OTP_MAGIC, OTP_CMD_INFO_GET, OTP_INFO)

#endif  /* LINUX_OTP_H */
