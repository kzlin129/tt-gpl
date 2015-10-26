/*****************************************************************************
* Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
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
*  vco2_drv.c
*
*  PURPOSE:
*
*     This implements the driver for the VCO2.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/types.h>
#include <linux/string.h>
#include <linux/module.h>

#include <linux/broadcom/vc.h>

/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */
/* ---- Private Function Prototypes -------------------------------------- */

/* ---- Functions -------------------------------------------------------- */
int vc_gencmd(char *response, int maxlen, const char *format, ...)
{
   BUG();
   return -1;
}
int vc_gencmd_number_property(char *text, char *property, int *number)
{
   BUG();
   return -1;
}
void vc_host_get_clock ( VC_Clock_t *clock)
{
   BUG();
}
void vc_host_set_clock ( VC_Clock_t *clock)
{
   BUG();
}
void vc_sendframe ( int devhdl )
{
   BUG();
}
void vc_host_read_VC02_STC_reg ( void)
{
   BUG();
}

VC_HOSTCALLBACK_T *vc_host_set_cmd_callback( VC_HOSTCALLBACK_T *callback )
{
   BUG();
   return NULL;
}

/****************************************************************************/
EXPORT_SYMBOL (vc_gencmd);
EXPORT_SYMBOL (vc_gencmd_number_property);
EXPORT_SYMBOL (vc_host_get_clock);
EXPORT_SYMBOL (vc_host_set_clock);
EXPORT_SYMBOL (vc_sendframe);
EXPORT_SYMBOL (vc_host_read_VC02_STC_reg);
EXPORT_SYMBOL (vc_host_set_cmd_callback);


