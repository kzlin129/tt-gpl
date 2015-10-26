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
*  vco2_gpio.c
*
*  PURPOSE:
*
*     This implements the driver for the VCO2 gpio function.
*
*  NOTES:
*
*****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */
#include <linux/broadcom/gpio.h>
#include <linux/broadcom/vc_gpio.h>
#include <linux/broadcom/vc.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/arch/reg_gpio.h>
#include "vcgencmd.h"

#define CMD_RESP_LEN  100

typedef enum
{
    VC_GPIO_PIN_TYPE_INPUT                = 0x00,
    VC_GPIO_PIN_TYPE_OUTPUT               = 0x01,
    VC_GPIO_PIN_TYPE_ALTERNATIVE_FUNC0    = 0x02,
    VC_GPIO_PIN_TYPE_ALTERNATIVE_FUNC1    = 0x03,

} VC_GPIO_PIN_TYPE;


int  vc_gpio_get_pin_type(int pin)
{
   char gencmd[CMD_RESP_LEN];
   char response[CMD_RESP_LEN];
   int ret=-1;
   int rspNum;

   snprintf(&gencmd[0], sizeof(gencmd), "gpio_control get_pin_type %d 0 ", (pin - GPIO_VC02_OFFSET));
   vc_gencmd( &response[0], sizeof( response ), "%s", &gencmd[0] );

   rspNum = response[2] - '0';

   if(rspNum == VC_GPIO_PIN_TYPE_INPUT )
   {
      ret = GPIO_PIN_TYPE_INPUT;
   }
   else if(rspNum == VC_GPIO_PIN_TYPE_OUTPUT)
   {
      ret = GPIO_PIN_TYPE_OUTPUT;
   }
   else if(rspNum == VC_GPIO_PIN_TYPE_ALTERNATIVE_FUNC0)
   {
      ret = GPIO_PIN_TYPE_ALTERNATIVE_FUNC0;
   }
   else if(rspNum == VC_GPIO_PIN_TYPE_ALTERNATIVE_FUNC1)
   {
      ret = GPIO_PIN_TYPE_ALTERNATIVE_FUNC1;
   }
   return ret; 
}


void vc_gpio_set_pin_type(int pin, int pinType)
{
   char gencmd[CMD_RESP_LEN];
   char response[CMD_RESP_LEN];

   VC_GPIO_PIN_TYPE type = VC_GPIO_PIN_TYPE_OUTPUT;

   if((pinType == GPIO_PIN_TYPE_INPUT_WITH_INTERRUPT) || (pinType ==GPIO_PIN_TYPE_INPUT))
   {
      type = VC_GPIO_PIN_TYPE_INPUT;
   }
   else if(pinType == GPIO_PIN_TYPE_OUTPUT)
   {
      type = VC_GPIO_PIN_TYPE_OUTPUT;
   }
   else if(pinType == GPIO_PIN_TYPE_ALTERNATIVE_FUNC0)
   {
      type = VC_GPIO_PIN_TYPE_ALTERNATIVE_FUNC0;
   }
   else if(pinType == GPIO_PIN_TYPE_ALTERNATIVE_FUNC1)
   {
      type = VC_GPIO_PIN_TYPE_ALTERNATIVE_FUNC1;
   }

   snprintf(&gencmd[0], sizeof(gencmd), "gpio_control set_pin_type %d %d ", (pin - GPIO_VC02_OFFSET), type);
   vc_gencmd( &response[0], sizeof( response ), "%s", &gencmd[0] );
  
}

int  vc_gpio_get_pin_val(int pin)
{
   char gencmd[CMD_RESP_LEN];
   char response[CMD_RESP_LEN];
   int ret=-1;
   int rspNum;

   snprintf(&gencmd[0], sizeof(gencmd), "gpio_control get_pin_val %d 0 ", (pin- GPIO_VC02_OFFSET));
   vc_gencmd( &response[0], sizeof( response ), "%s", &gencmd[0] );

   rspNum = response[2] - '0';

   if(rspNum == 1)
   {
      ret = 1;
   }
   else if(rspNum == 0)
   {
      ret = 0;
   }

   return ret; 
}

void vc_gpio_set_pin_val(int pin, int val)
{
   char gencmd[CMD_RESP_LEN];
   char response[CMD_RESP_LEN];

   snprintf(&gencmd[0], sizeof(gencmd), "gpio_control set_pin_val %d %d ", (pin- GPIO_VC02_OFFSET), val);
   vc_gencmd( &response[0], sizeof( response ), "%s", &gencmd[0] );

}

EXPORT_SYMBOL (vc_gpio_get_pin_type);
EXPORT_SYMBOL (vc_gpio_set_pin_type);
EXPORT_SYMBOL (vc_gpio_get_pin_val);
EXPORT_SYMBOL (vc_gpio_set_pin_val);

