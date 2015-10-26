/*****************************************************************************
* Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
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
 * Description: The Display Director LCD sub module
 */

#include <linux/interrupt.h>
#include <asm/semaphore.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/broadcom/gpio.h>
#include <linux/broadcom/dd/dd.h>
#include <linux/broadcom/lcd_backlight.h>
#include <linux/broadcom/lcd_settings.h>
#include <asm/arch/csp/chipcHw_inline.h>
#include <asm/arch/csp/gpiomux.h>
#include <csp/clcdHw.h>
#include "dd_lcd.h"

#define LCD_UPDATE_TIMEOUT    (1 * HZ) /* LCD update timeout */

typedef struct lcd_cfg {
   struct semaphore lock; /* lock to protect the LCD operations */
   struct completion complete; /* signals the completion of LCD update */
   DD_LCD_PANEL_T panel;
} LCD_CFG_T;

static LCD_CFG_T g_lcd; 

/*
 * LCD panel information
 */
static DD_LCD_PANEL_T g_panel[DD_CLD_MAX_NUM_PANEL] = {
   {
      .model = DD_LCD_MODEL_SEIKO_RA167Z, /* panel model */
      .name = "Seiko RA167Z 480 x 272", /* panel name */
   	.refresh = 60, /* refresh rate in Hz */
   	.xres = 480, /* x resolution in pixels */
   	.yres = 272, /* y resolution in pixels */
   	.pixel_clk = 111000, /* pixel clock, 9.009 MHz ~= 111000 ps */
   	.left_margin = 2, /* horizontal back porch */
   	.right_margin = 2, /* horizontal front porch */
   	.upper_margin = 2, /* vertical back porch */
   	.lower_margin = 2, /* vertical front porch */
   	.hsync_len = 41, /* horizontal pulse width */
   	.vsync_len = 10, /* vertical pulse width */
#ifdef CONFIG_ARCH_FPGA11107
      .pixel_clk_div = 210, /* pixel clock divider */
#else
      .pixel_clk_div = 14, /* pixel clock divider */
#endif
      .inv_out_enable = 0, /* invert output enable */
      .inv_pixel_clk = 0, /* invert pixel clock */
      .inv_hsync = 1, /* invert horizontal sync */
      .inv_vsync = 1, /* invert certical sync */
      .type = DD_LCD_TYPE_TFT, /* panel type */
      .bpp = 32, /* bits per pixel */
   },

   {
      .model = DD_LCD_MODEL_SEIKO_RA169Z, /* panel model */
      .name = "Seiko RA169Z 800 x 480", /* panel name */
   	.refresh = 60, /* refresh rate in Hz */
   	.xres = 800, /* x resolution in pixels */
   	.yres = 480, /* y resolution in pixels */
   	.pixel_clk = 29850, /* pixel clock, 33.5 MHz ~= 29850 ps */
   	.left_margin = 89, /* horizontal back porch */
   	.right_margin = 164, /* horizontal front porch */
   	.upper_margin = 23, /* vertical back porch */
   	.lower_margin = 10, /* vertical front porch */
   	.hsync_len = 10, /* horizontal pulse width */
   	.vsync_len = 10, /* vertical pulse width */
#ifdef CONFIG_ARCH_FPGA11107
      .pixel_clk_div = 65, /* pixel clock divider */
#else
      .pixel_clk_div = 2, /* pixel clock divider */
#endif
      .inv_out_enable = 0, /* invert output enable */
      .inv_pixel_clk = 0, /* invert pixel clock */
      .inv_hsync = 1, /* invert horizontal sync */
      .inv_vsync = 1, /* invert certical sync */
      .type = DD_LCD_TYPE_TFT, /* panel type */
      .bpp = 32, /* bits per pixel */
   },

   {
      .model = DD_LCD_MODEL_SAMSUNG_LMS700KF01, /* panel model */
      .name = "Samsung LMS700KF01 800 x 480", /* panel name */
   	.refresh = 60, /* refresh rate in Hz */
   	.xres = 800, /* x resolution in pixels */
   	.yres = 480, /* y resolution in pixels */
   	.pixel_clk = 21830, /* pixel clock, 24.5 MHz ~= 21830 ps */
   	.left_margin = 13, /* horizontal back porch */
   	.right_margin = 8, /* horizontal front porch */
   	.upper_margin = 7, /* vertical back porch */
   	.lower_margin = 5, /* vertical front porch */
   	.hsync_len = 3, /* horizontal pulse width */
   	.vsync_len = 1, /* vertical pulse width */
#ifdef CONFIG_ARCH_FPGA11107
      .pixel_clk_div = 90, /* pixel clock divider */
#else
      .pixel_clk_div = 4, /* pixel clock divider */
#endif
      .inv_out_enable = 0, /* invert output enable */
      .inv_pixel_clk = 0, /* invert pixel clock */
      .inv_hsync = 1, /* invert horizontal sync */
      .inv_vsync = 1, /* invert certical sync */
      .type = DD_LCD_TYPE_TFT, /* panel type */
      .bpp = 32, /* bits per pixel */
   },
};

/*
 * LCD IRQ hanlder
 */
static irqreturn_t lcd_isr(int irq, void *data)
{
   LCD_CFG_T *lcd = (LCD_CFG_T *)data;

   /* LCD update */
   if (ClcdHw_GetMaskIntBaseAddrUpd()) {
      /* clear and disable this interrupt */
      ClcdHw_ClearIntBaseAddrUpd();
      ClcdHw_EnIntBaseAddrUpd(0);

      /* signal the completion of LCD update */
      complete(&lcd->complete);
   }

   /* LCD FIFO underrun */
   if (ClcdHw_GetMaskIntFifo()) {
      ClcdHw_ClearIntFifo();
   }

   /* AHB master bus error */
   if (ClcdHw_GetMaskIntMBERR()) {
      ClcdHw_ClearIntMBERR();
      printk(KERN_ERR "DD_LCD: AHB Bus Error\n");
   }

   if (ClcdHw_GetMaskIntVcomp()) {
      ClcdHw_ClearIntVcomp();
   }

   return IRQ_HANDLED;
}

DD_STATUS_T dd_lcd_init(DD_LCD_MODEL_T model)
{
   gpiomux_rc_e rc;
   LCD_CFG_T *lcd = &g_lcd;

   if (model >= DD_LCD_MODEL_INVALID) {
      printk(KERN_ERR "DD_LCD: Unknown LCD model=%u\n", model);
      return DD_FAIL;
   }

   printk(KERN_NOTICE "DD_LCD: LCD (model=%u) initializing...\n", model);

   init_MUTEX(&lcd->lock); /* unlocked */
   init_completion(&lcd->complete); /* incomplete */

   /* copy the static LCD panel parameters over */
   memcpy(&lcd->panel, &g_panel[model], sizeof(DD_LCD_PANEL_T));

   /* enable the clock */
   chipcHw_setClockFrequency(chipcHw_CLOCK_LCD, 150000000); // FIXME where should this come from?
   chipcHw_setClockEnable(chipcHw_CLOCK_LCD);

   switch (model) {
      case DD_LCD_MODEL_SEIKO_RA167Z:
      case DD_LCD_MODEL_SEIKO_RA169Z:
         rc = gpiomux_request(HW_CFG_LCD_CLPOWER, chipcHw_GPIO_FUNCTION_GPIO,
               "LCD Power");
         if (rc != gpiomux_rc_SUCCESS) {
            printk(KERN_ERR "DD_LCD: Unable to request GPIO pin %u\n",
                  HW_CFG_LCD_CLPOWER);
            return DD_FAIL;
         }
         rc = gpiomux_request(HW_CFG_LCD_SPI_CLK, chipcHw_GPIO_FUNCTION_GPIO,
               "LCD 3V");
         if (rc != gpiomux_rc_SUCCESS) {
            printk(KERN_ERR "DD_LCD: Unable to request GPIO pin %u\n",
                  HW_CFG_LCD_SPI_CLK);
            return DD_FAIL;
         }
         rc = gpiomux_request(HW_CFG_LCD_SPI_DATA, chipcHw_GPIO_FUNCTION_GPIO,
               "LCD 5V");
         if (rc != gpiomux_rc_SUCCESS) {
            printk(KERN_ERR "DD_LCD: Unable to request GPIO pin %u\n",
                  HW_CFG_LCD_SPI_DATA);
            return DD_FAIL;
         }
         rc = gpiomux_request(HW_CFG_LCD_BUF_ENABLE,
               chipcHw_GPIO_FUNCTION_GPIO, "LCD Backlight");
         if (rc != gpiomux_rc_SUCCESS) {
            printk(KERN_ERR "DD_LCD: Unable to request GPIO pin %u\n",
                  HW_CFG_LCD_BUF_ENABLE);
            return DD_FAIL;
         }

         /* set GPIO pins to output mode */
         gpio_direction_output(HW_CFG_LCD_CLPOWER, 0);
         gpio_direction_output(HW_CFG_LCD_SPI_CLK, 0);
         gpio_direction_output(HW_CFG_LCD_SPI_DATA, 0);
         gpio_direction_output(HW_CFG_LCD_BUF_ENABLE, 0);
 
         break;

      case DD_LCD_MODEL_SAMSUNG_LMS700KF01:
         rc = gpiomux_request(HW_CFG_LCD_BUF_ENABLE,
               chipcHw_GPIO_FUNCTION_GPIO, "LCD Buffer");
         if (rc != gpiomux_rc_SUCCESS) {
            printk(KERN_ERR "DD_LCD: Unable to request GPIO pin %u\n",
                  HW_CFG_LCD_CLPOWER);
            return DD_FAIL;
         }
         rc = gpiomux_request(HW_CFG_LCD_CLPOWER, chipcHw_GPIO_FUNCTION_GPIO,
               "LCD Power");
         if (rc != gpiomux_rc_SUCCESS) {
            printk(KERN_ERR "DD_LCD: Unable to request GPIO pin %u\n",
                  HW_CFG_LCD_SPI_CLK);
            return DD_FAIL;
         }
         rc = gpiomux_request(HW_CFG_LCD_RESET, chipcHw_GPIO_FUNCTION_GPIO,
               "LCD Reset");
         if (rc != gpiomux_rc_SUCCESS) {
            printk(KERN_ERR "DD_LCD: Unable to request GPIO pin %u\n",
                  HW_CFG_LCD_SPI_DATA);
            return DD_FAIL;
         }

         /* set GPIO pins to output mode */
         gpio_direction_output(HW_CFG_LCD_BUF_ENABLE, 0);
         gpio_direction_output(HW_CFG_LCD_CLPOWER, 0);
         gpio_direction_output(HW_CFG_LCD_RESET, 0);

         break;
         
      default:
         printk(KERN_ERR "DD_LCD: Unknown LCD model=%u\n", model);
         return DD_FAIL;
   }

   ClcdHw_EnIntFifo(1);
   ClcdHw_EnIntBaseAddrUpd(0);
   ClcdHw_EnIntVertComp(0);
   ClcdHw_EnIntMBERR(1);

   if (request_irq(IRQ_CLCD, lcd_isr, IRQF_SHARED, "DD_LCD", lcd) < 0) {
      printk(KERN_ERR "DD_LCD: request_irq %u failed\n", IRQ_CLCD);
      goto lcd_err_term;
   }

   return DD_SUCCESS;

lcd_err_term:
   dd_lcd_term();
   return DD_FAIL;
}

DD_STATUS_T dd_lcd_term(void)
{
   LCD_CFG_T *lcd = &g_lcd;

   free_irq(IRQ_CLCD, 0);
   dd_lcd_panel_disable();

   switch (lcd->panel.model) {
      case DD_LCD_MODEL_SEIKO_RA167Z:
      case DD_LCD_MODEL_SEIKO_RA169Z:
         gpiomux_free(HW_CFG_LCD_CLPOWER);
         gpiomux_free(HW_CFG_LCD_SPI_CLK);
         gpiomux_free(HW_CFG_LCD_SPI_DATA);
         gpiomux_free(HW_CFG_LCD_BUF_ENABLE);
         break;

      case DD_LCD_MODEL_SAMSUNG_LMS700KF01:
         gpiomux_free(HW_CFG_LCD_BUF_ENABLE);
         gpiomux_free(HW_CFG_LCD_CLPOWER);
         gpiomux_free(HW_CFG_LCD_RESET);
         break;
         
      default:
         printk(KERN_ERR "DD_LCD: Unknown LCD model=%u\n", lcd->panel.model);
         return DD_FAIL;
   }
   
   return DD_SUCCESS;
}

DD_STATUS_T dd_lcd_panel_enable(void)
{
   LCD_CFG_T *lcd = &g_lcd;

   if (down_interruptible(&lcd->lock) != 0) {
      printk(KERN_ERR "DD_LCD: Lock acquire interrupted or failed\n");
      return DD_FAIL;
   }

   /* activate the LCD interface in CHIPC */
   chipcHw_activateLcdInterface();

   switch (lcd->panel.model) {
      case DD_LCD_MODEL_SEIKO_RA167Z:
         gpio_set_value(HW_CFG_LCD_SPI_CLK, 1);
         mdelay(50);
         ClcdHw_SetLcdEnable(1);
         mdelay(50);
         gpio_set_value(HW_CFG_LCD_SPI_DATA, 1);
         mdelay(50);
         /* enable output data pins */
         ClcdHw_SetLcdPwr(1);
         /* apply power */
         gpio_set_value(HW_CFG_LCD_CLPOWER, 1);
         mdelay(50);
         gpio_set_value(HW_CFG_LCD_BUF_ENABLE, 1);
         break;

      case DD_LCD_MODEL_SEIKO_RA169Z:
         gpio_set_value(HW_CFG_LCD_SPI_CLK, 1);
         mdelay(10);
         ClcdHw_SetLcdEnable(1);
         mdelay(10);
         gpio_set_value(HW_CFG_LCD_SPI_DATA, 1);
         mdelay(10);
         /* enable output data pins */
         ClcdHw_SetLcdPwr(1);
         /* apply power */
         gpio_set_value(HW_CFG_LCD_CLPOWER, 1);
         mdelay(200);
         gpio_set_value(HW_CFG_LCD_BUF_ENABLE, 1);
         break;

      case DD_LCD_MODEL_SAMSUNG_LMS700KF01:
         gpio_set_value(HW_CFG_LCD_BUF_ENABLE, 0);
         /* reset the LCD panel */
         gpio_set_value(HW_CFG_LCD_RESET, 0);
         mdelay(6);
         gpio_set_value(HW_CFG_LCD_RESET, 1);
         /* apply power */
         gpio_set_value(HW_CFG_LCD_CLPOWER, 0);
         mdelay(10);
         ClcdHw_SetLcdEnable(1);
         mdelay(17);
         ClcdHw_SetLcdPwr(1);
         mdelay(50);
         lcd_backlight_enable(LCD_BACKLIGHT_FULL_ON);
         break;
         
      default:
         printk(KERN_ERR "DD_LCD: Unknown LCD model=%u\n", lcd->panel.model);
         up(&lcd->lock);
         return DD_FAIL;
   }
   
   up(&lcd->lock);
   return DD_SUCCESS;
}

DD_STATUS_T dd_lcd_panel_disable(void)
{
   LCD_CFG_T *lcd = &g_lcd;

   if (down_interruptible(&lcd->lock) != 0) {
      printk(KERN_ERR "DD_LCD: Lock acquire interrupted or failed\n");
      return DD_FAIL;
   }

   switch (lcd->panel.model) {
      case DD_LCD_MODEL_SEIKO_RA167Z:
         gpio_set_value(HW_CFG_LCD_BUF_ENABLE, 0);
         gpio_set_value(HW_CFG_LCD_CLPOWER, 0);
         ClcdHw_SetLcdPwr(0);
         mdelay(50);
         gpio_set_value(HW_CFG_LCD_SPI_DATA, 0);
         mdelay(50);
         ClcdHw_SetLcdEnable( 0 );
         mdelay(50);
         gpio_set_value(HW_CFG_LCD_SPI_CLK, 0);
         break;

      case DD_LCD_MODEL_SEIKO_RA169Z:
         gpio_set_value(HW_CFG_LCD_BUF_ENABLE, 0);
         mdelay(5);
         gpio_set_value(HW_CFG_LCD_CLPOWER, 0);
         ClcdHw_SetLcdPwr(0);
         mdelay(200);
         gpio_set_value(HW_CFG_LCD_SPI_DATA, 0);
         mdelay(5);
         ClcdHw_SetLcdEnable(0);
         mdelay(5);
         gpio_set_value(HW_CFG_LCD_SPI_CLK, 0);
         break;

      case DD_LCD_MODEL_SAMSUNG_LMS700KF01:
         lcd_backlight_enable(LCD_BACKLIGHT_OFF);
         mdelay(50);
         ClcdHw_SetLcdPwr(0);
         mdelay(17);
         ClcdHw_SetLcdEnable(0);
         mdelay(10);
         gpio_set_value(HW_CFG_LCD_CLPOWER, 1);
         gpio_set_value(HW_CFG_LCD_BUF_ENABLE, 1);
         break;
         
      default:
         printk(KERN_ERR "DD_LCD: Unknown LCD model=%u\n", lcd->panel.model);
         up(&lcd->lock);
         return DD_FAIL;
   }
   
   /* deactivate the LCD interface */
   chipcHw_deactivatePifLcdInterface();
   up(&lcd->lock);
   return DD_SUCCESS;
}

DD_STATUS_T dd_lcd_panel_reset(void)
{
   LCD_CFG_T *lcd = &g_lcd;
   DD_LCD_PANEL_T *panel = &lcd->panel;

   if (down_interruptible(&lcd->lock) != 0) {
      printk(KERN_ERR "DD_LCD: Lock acquire interrupted or failed\n");
      return DD_FAIL;
   }

   ClcdHw_SetPPL(panel->xres);
   /* dual panel NOT supported */
   ClcdHw_SetLPP(panel->yres, 0);

   ClcdHw_SetHrzBackPorch(panel->left_margin);
   ClcdHw_SetHrzFrontPorch(panel->right_margin);
   ClcdHw_SetVertBackPorch(panel->upper_margin);
   ClcdHw_SetVertFrontPorch(panel->lower_margin);

   ClcdHw_SetHrzSynchWidth(panel->hsync_len);
   ClcdHw_SetVertSynchWidth(panel->vsync_len);

   ClcdHw_SetPCD(panel->pixel_clk_div);
   /* do NOT bypass the clock divider */
   ClcdHw_SetBCD(0);
   ClcdHw_SetCPL(panel->xres, panel->type);

   ClcdHw_SetIOE(panel->inv_out_enable);
   ClcdHw_SetIPC(panel->inv_pixel_clk);
   ClcdHw_SetIHS(panel->inv_hsync);
   ClcdHw_SetIVS(panel->inv_vsync);

   /* NO AC frequency bias */
   ClcdHw_SetACB(0);
   /* NO external clock */
   ClcdHw_SetClkSel(0);
   ClcdHw_SetLineEndEn(0);
   ClcdHw_SetSigDelay(0);
   ClcdHw_SetWatermarkSig(1);
   ClcdHw_SetLcdVComp(0);
   ClcdHw_SetBEPO(0);
   ClcdHw_SetBEBO(0);
   ClcdHw_SetBGR(1);
   ClcdHw_SetDualLcd(0);
   ClcdHw_SetMonoLcdInterface(0);
   if (panel->type == DD_LCD_TYPE_TFT)
      ClcdHw_SetLcdTFT(1);
   else
      ClcdHw_SetLcdTFT(0);
   ClcdHw_SetLcdBW(0);
   /* TODO: hardcoded to 24 (32 in Linux) for now */
   ClcdHw_SetLcdBpp(CLCDHW_BPP24);

   up(&lcd->lock);
   return DD_SUCCESS;
}

DD_STATUS_T dd_lcd_update(uint32_t addr)
{
   LCD_CFG_T *lcd = &g_lcd;

   if (down_interruptible(&lcd->lock) != 0) {
      printk(KERN_ERR "DD_LCD: Lock acquire interrupted or failed\n");
      return DD_FAIL;
   }

   /* mark as incomplete before the LCD update */
   INIT_COMPLETION(lcd->complete);

   /*
    * Enable the address update interrupt so we get signaled when the update
    * finishes
    */
   ClcdHw_EnIntBaseAddrUpd(1);

   /* update the LCD */
   ClcdHw_SetUpperBaseAddr(addr);

   /* block wait until the LCD finishes updating */
   if (wait_for_completion_interruptible_timeout(&lcd->complete,
            LCD_UPDATE_TIMEOUT) < 0) {
      printk(KERN_ERR "DD_LCD: LCD update interrupted or timeout\n");
      return DD_FAIL;
   }

   up(&lcd->lock);
   return DD_SUCCESS;
}

void dd_lcd_info_get(DD_LCD_INFO_T *info)
{
   LCD_CFG_T *lcd = &g_lcd;
   DD_LCD_PANEL_T *panel = &lcd->panel;

   if (down_interruptible(&lcd->lock) != 0) {
      printk(KERN_ERR "DD_LCD: Lock acquire interrupted or failed\n");
      return;
   }

   info->width = panel->xres;
   info->height = panel->yres;
   info->bpp = panel->bpp;
   info->pitch = panel->xres * panel->bpp / 8;
   info->size = panel->xres * panel->yres * panel->bpp / 8;

   info->pixel_clk = panel->pixel_clk;
	info->left_margin = panel->left_margin;
   info->right_margin = panel->right_margin;
   info->upper_margin = panel->upper_margin;
   info->lower_margin = panel->lower_margin;
   info->hsync_len = panel->hsync_len;
   info->vsync_len = panel->vsync_len;

   up(&lcd->lock);
}
