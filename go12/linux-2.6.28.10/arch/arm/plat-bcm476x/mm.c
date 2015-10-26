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

#include <linux/init.h>
#include <linux/device.h>
#include <mach/hw_cfg.h>
#include <asm/io.h>
#include <asm/mach/map.h>

#ifdef CONFIG_USB_GADGET_DWC_OTG
 #include <asm/arch/lm.h>
#endif

#define	IO_DESC(pa, sz) { .virtual = IO_ADDRESS(pa), 	\
                          .pfn = __phys_to_pfn(pa), 	\
                          .length = (sz), 		\
                          .type = MT_DEVICE }

static struct map_desc bcm476x_io_desc[] __initdata =
{
	IO_DESC(BCM47XX_SRAM_ADDRBASE,  SZ_64K),
	IO_DESC(BCM47XX_NAND_ADDRBASE0, SZ_64M),
	IO_DESC(BCM47XX_NAND_ADDRBASE1, SZ_64M),

	IO_DESC(VIC0_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(VIC1_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(DMA_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(DDR_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(LCD_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(GFX_REG_BASE_ADDR, 	4*SZ_4K),
	IO_DESC(HPI_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(USB_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(SDM0_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(SDM1_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(SDM2_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(FLS_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(ETM_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(TIM1_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(RPC_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(GIO1_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(URT2_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(PWM_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(TSC_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(AUD0_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(AUD1_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(I2S_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(URT0_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(URT1_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(URTG_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(I2C0_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(I2C1_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(SPI0_REG_BASE_ADDR,	SZ_4K),
	IO_DESC(SPI1_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(OTP_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(TIM0_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(GIO0_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(RTC_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(CMU_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(WDT_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(AXA_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(SPM_AHB_REG_BASE_ADDR, 	SZ_64K),
	IO_DESC(SPM_APB_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(PKA_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(RNG_REG_BASE_ADDR, 	SZ_4K),
	IO_DESC(PML_REG_BASE_ADDR, 	SZ_4K),
};

//TODO: check if it's better to put in arch.c
#ifdef CONFIG_USB_GADGET_DWC_OTG

int __init usb_devices_init( void )
{
	struct lm_device *lmdev;
	uint32_t pll_ctl;
	int rc = -1;

	pll_ctl = readl( IO_ADDRESS( CMU_R_PLA_PLL_CTL0_MEMADDR ) );

	// @KP: 090316: setup for usb device mode
	// bypass PMU for usb device testing; reg 0xb00a8
	// writel(0x6a3, IO_ADDRESS(CMU_R_MODE_CTL_MEMADDR));

	// config usb broadcom phy (8-bit) for otg mode (default to device mode); reset and unreset pair; reg 0xb0244
	writel(0x004069d5, IO_ADDRESS(CMU_R_USB_PHY0_MEMADDR));
	writel(0x0041a9d5, IO_ADDRESS(CMU_R_USB_PHY0_MEMADDR));

	// enable differential pll for usb; reg 0xb0040 bit 9
	pll_ctl |= CMU_F_PLA_EN_CMLBUF6_MASK;
	writel(pll_ctl, IO_ADDRESS(CMU_R_PLA_PLL_CTL0_MEMADDR));

	lmdev = kmalloc(sizeof(struct lm_device), GFP_KERNEL);
	if (lmdev) {
		memset( lmdev, 0 , sizeof(struct lm_device));

		lmdev->resource.start = USB_REG_BASE_ADDR;
		lmdev->resource.end   = lmdev->resource.start + SZ_4K -1;
		lmdev->resource.flags = IORESOURCE_MEM ;

		lmdev->irq = BCM4760_INTR_USB;
		lmdev->id  = -2;

		rc = lm_device_register(lmdev);
		if (rc) {
			printk(KERN_ERR "can't register BCM4760 OTG core device\n");
		}

	}

	return rc;
}

subsys_initcall( usb_devices_init );

#endif // CONFIG_USB_GADGET_DWC_OTG

void __init bcm476x_map_io( void )
{
	iotable_init( bcm476x_io_desc, ARRAY_SIZE( bcm476x_io_desc ));
}

