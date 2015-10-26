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

#include <linux/module.h>
#include <linux/kernel.h>

#include <mach/hardware.h>
#include <asm/delay.h>
#include <asm/io.h>

/*
 * Below are the "safe" reset masks for the system reset function.
 * These are termed safe because we can use them to reset the critical
 * components but resetting them doesn't mess up our execution environment
 * needed to continue the system reset process.
 */
#define CMU_BLK_SAFE_RESET0_MASK	(		  \
	/*CMU_F_CMU_ARM_HRESETN_MASK			|*/ \
	/*CMU_F_CMU_ARM_NPORESETIN_MASK			|*/ \
	/*CMU_F_CMU_ETM_NRESETN_MASK			|*/ \
	/*CMU_F_CMU_ETM_PRESETIN_MASK			|*/ \
	/*CMU_F_CMU_BM0_HRESETN_MASK			|*/ \
	/*CMU_F_CMU_BM1_HRESETN_MASK			|*/ \
	/*CMU_F_CMU_BM2_HRESETN_MASK			|*/ \
	/*CMU_F_CMU_BM3_HRESETN_MASK			|*/ \
	/*CMU_F_CMU_BM4_HRESETN_MASK			|*/ \
	/*CMU_F_CMU_BM5_HRESETN_MASK			|*/ \
	CMU_F_CMU_DMA_HRESETN_MASK			| \
	/*CMU_F_CMU_DDR_HRESETN_MASK			|*/ \
	/*CMU_F_CMU_DDR_RESETN_MASK			|*/ \
	CMU_F_CMU_FLS_HRESETN_MASK			| \
	CMU_F_CMU_GFX_HRESETN_MASK			| \
	CMU_F_CMU_HPI_HRESETN_MASK			| \
	CMU_F_CMU_LCD_HRESETN_MASK			| \
	/*CMU_F_CMU_ROM_HRESETN_MASK			|*/ \
	/*CMU_F_CMU_SCR_HRESETN_MASK			|*/ \
	CMU_F_CMU_SDM_0_HRESETN_MASK			| \
	CMU_F_CMU_SDM_1_HRESETN_MASK			| \
	CMU_F_CMU_SDM_2_HRESETN_MASK			| \
	CMU_F_CMU_USB_HRESETN_MASK			| \
	CMU_F_CMU_VIC_HRESETN_MASK			  \
	)

#define CMU_BLK_SAFE_RESET1_MASK	(		  \
	CMU_F_CMU_URT_G_PRESETN_MASK			| \
	CMU_F_CMU_URT_0_PRESETN_MASK			| \
	/*CMU_F_CMU_URT_1_PRESETN_MASK			|*/ \
	/*CMU_F_CMU_URT_2_PRESETN_MASK			|*/ \
	CMU_F_CMU_SPI_PRESETN_MASK			| \
	CMU_F_CMU_PWM_PRESETN_MASK			| \
	CMU_F_CMU_I2S_PRESETN_MASK			| \
	CMU_F_CMU_I2C_0_RESETN_MASK			| \
	CMU_F_CMU_I2C_0_PRESETN_MASK			| \
	CMU_F_CMU_I2C_1_RESETN_MASK			| \
	CMU_F_CMU_I2C_1_PRESETN_MASK			| \
	CMU_F_CMU_TSC_PRESETN_MASK			| \
	CMU_F_CMU_PKA_PRESETN_MASK			| \
	CMU_F_CMU_RNG_PRESETN_MASK			| \
	CMU_F_CMU_SPM_HRESETN_MASK			| \
	CMU_F_CMU_SPM_RESETN_MASK			| \
	CMU_F_CMU_AXA_PRESETN_MASK			| \
	CMU_F_CMU_GPS_RESETN_MASK			| \
	CMU_F_CMU_AUD_APB_SRSTN_MASK			| \
	CMU_F_CMU_AUD_156MHZ_SRSTN_MASK			| \
	CMU_F_OTP_PRESETN_MASK				  \
	)

#define CMU_BLK_SAFE_RESET2_MASK	(		  \
	CMU_F_CMU_RPC_PRESETN_MASK			| \
	CMU_F_CMU_RPC_32KRESETN_MASK			| \
	CMU_F_CMU_TIM_0_32KRESETN_MASK			| \
	CMU_F_CMU_TIM_0_24MRESETN_MASK			| \
	CMU_F_CMU_TIM_1_32KRESETN_MASK			| \
	CMU_F_CMU_TIM_1_24MRESETN_MASK			| \
	CMU_F_CMU_WDT_32KRESETN_MASK			| \
	/*CMU_F_CMU_GIO_0_32KRESETN_MASK		|*/ \
	/*CMU_F_CMU_GIO_1_32KRESETN_MASK		|*/ \
	/*CMU_F_CMU_GIO_0_PRESETN_MASK			|*/ \
	/*CMU_F_CMU_GIO_1_PRESETN_MASK			|*/ \
	CMU_F_CMU_PML_PRESETN_MASK			| \
	CMU_F_CMU_RTC_PRESETN_MASK			| \
	CMU_F_CMU_TIM_2_PRESETN_MASK			| \
	CMU_F_CMU_DFC_NPORESETIN_MASK			| \
	CMU_F_CMU_TIM_1_PRESETN_MASK			| \
	CMU_F_CMU_TIM_0_PRESETN_MASK			| \
	CMU_F_CMU_WDT_PRESETN_MASK			  \
	)

void bcm476x_reset_system(void)
{
	printk(KERN_INFO "Resetting the board...\n");

	asm("cpsid aif");	/* disable FAULT/IRQ/FIQ */

	/*
	* Unlock CMU so we can use BLOCK_RESETn registers. This is safer
	* as we're putting pretty much all hardware blocks into reset
	* state prior to watchdog reset getting fired after which we'll
	* re-enter Boot ROM.
	*/
	writel(0xbcbc4760, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR));

		writel(readl(IO_ADDRESS(CMU_R_GLOBAL_RESET_CTL_MEMADDR)) &
			~(CMU_F_SELECT_TRIGGER_RESETS_MASK | CMU_F_TRIGGER_RESETS_MASK),
			IO_ADDRESS(CMU_R_GLOBAL_RESET_CTL_MEMADDR));

		/*
		 * Most things in BLOCK_RESET2 are safe except for GPIO
		 * blocks. Resetting those is not ok for A0 because it
		 * resets the drive strength settings to the defaults which
		 * are too weak.
		 */
		writel(readl(IO_ADDRESS(CMU_R_BLOCK_RESET2_MEMADDR)) &
			~(CMU_BLK_SAFE_RESET2_MASK),
			IO_ADDRESS(CMU_R_BLOCK_RESET2_MEMADDR));
		udelay(5);
		writel(readl(IO_ADDRESS(CMU_R_BLOCK_RESET2_MEMADDR)) |
			(CMU_BLK_SAFE_RESET2_MASK),
			IO_ADDRESS(CMU_R_BLOCK_RESET2_MEMADDR));

		/*
		 * This method uses the watchdog timer to generate the
		 * reset. It still uses the BLOCK_RESETn registers to
		 * reset most peripheral devices to a known state.
		 */

		/*
		 * Need to be careful what we reset in BLOCK_RESET0. Some of the
		 * peripheral resets in here will cause the system to fail.
		 */
		writel(readl(IO_ADDRESS(CMU_R_BLOCK_RESET0_MEMADDR)) &
			~(CMU_BLK_SAFE_RESET0_MASK),
			IO_ADDRESS(CMU_R_BLOCK_RESET0_MEMADDR));
		udelay(5);
		writel(readl(IO_ADDRESS(CMU_R_BLOCK_RESET0_MEMADDR)) |
			(CMU_BLK_SAFE_RESET0_MASK),
			IO_ADDRESS(CMU_R_BLOCK_RESET0_MEMADDR));

		/*
		* Generally everything in BLOCK_RESET1 is safe.
		*/
		writel(readl(IO_ADDRESS(CMU_R_BLOCK_RESET1_MEMADDR)) &
			~(CMU_BLK_SAFE_RESET1_MASK),
			IO_ADDRESS(CMU_R_BLOCK_RESET1_MEMADDR));
		udelay(5);
		writel(readl(IO_ADDRESS(CMU_R_BLOCK_RESET1_MEMADDR)) |
			(CMU_BLK_SAFE_RESET1_MASK),
			IO_ADDRESS(CMU_R_BLOCK_RESET1_MEMADDR));
		udelay(100);

	/*
	* Program watchdog timer to reset us in six 32Khz clocks.
	*/
	writel(0x1ACCE551, IO_ADDRESS(WDT_R_WDOGLOCK_MEMADDR));
	udelay(1000);
	writel(readl(IO_ADDRESS(WDT_R_WDOGCONTROL_MEMADDR)) &
		~(WDT_F_RESEN_MASK | WDT_F_PREEN_MASK | WDT_F_INTEN_MASK),
		IO_ADDRESS(WDT_R_WDOGCONTROL_MEMADDR));
	udelay(1000);
	writel(3, IO_ADDRESS(WDT_R_WDOGLOAD_MEMADDR));
	udelay(1000);
	writel(WDT_F_RESEN_MASK | WDT_F_INTEN_MASK, IO_ADDRESS(WDT_R_WDOGCONTROL_MEMADDR));

	/* Loop here forever, until watchdog reset occurs. */
	__asm("1: nop; b 1b");
    
	/* We'll never get here because the reset will occur but... */
	writel(0, IO_ADDRESS(CMU_R_TRIGGER_UNLOCK_MEMADDR));
}

void bcm476x_reset_device(int dev_reset_id)
{
}
EXPORT_SYMBOL(bcm476x_reset_system);
EXPORT_SYMBOL(bcm476x_reset_device);
