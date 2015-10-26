/*****************************************************************************
*  Copyright 2006 - 2008 Broadcom Corporation.  All rights reserved.
*
*  Unless you and Broadcom execute a separate written software license
*  agreement governing use of this software, this software is licensed to you
*  under the terms of the GNU General Public License version 2, available at
*  http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
*  Notwithstanding the above, under no circumstances may you combine this
*  software in any way with any other Broadcom software provided under a
*  license other than the GPL, without Broadcom's express prior written
*  consent.
*
*****************************************************************************/
#ifndef __HW_CFG
#define __HW_CFG	

/*-----------------------------------------------------------------------------
 *  Include files
 *---------------------------------------------------------------------------*/
#include <asm/arch/hardware.h>
//#include <asm/arch/spi.h>

/*-----------------------------------------------------------------------------
 *  pci_emu specific defines
 *-----------------------------------------------------------------------------*/
#define HW_PRIMARY_STORAGE_TYPE  "SD"  /* possible value can be SD */
#define HW_SD_B_S1 "primary_sd"
#define HW_SD_B_S2 "secondary_sd"
#define HW_SD_B_S3 "tertiary_sd"

#define BCM_SD0		0
#define BCM_SD1		1
#define BCM_SD2		2
#define BCM_SD_DISABLE	0

// GPIO Assignments - pins not showing up here are either preallocated (e.g. keyboard 0-3, 8-14),
// or used by the hardware internally (e.g. BT, codec), or are unassigned (e.g. 31).

#define HW_GPIO_PMU_IRQ_PIN               1
#define HW_GPIO_PMU_DATA_TO_GPIO(data,idx)		(((struct resource *) data.private_data)[idx].start)

typedef enum {
	HW_GPIO_PMU_DATA_IDX_IRQ_DEFAULT,
	HW_GPIO_PMU_DATA_IDX_IRQ_A0_REWORK
} HW_GPIO_PMU_DATA_IDX_t;

/*-----------------------------------------------------------------------------
 *SD/MMC specific defines
 *-----------------------------------------------------------------------------*/
#ifndef CONFIG_ARCH_BCM476X_FPGA
#define BCM47XX_DEFAULT_BASE_CLOCK      48000000
#define BCM47XX_DEFAULT_TIMEOUT_CLOCK   48000000
#else
#define BCM47XX_DEFAULT_BASE_CLOCK      4500000
#define BCM47XX_DEFAULT_TIMEOUT_CLOCK   4500000
#endif

#define BCM476X_ENABLE_DAFCA_TRACE 	0	// set this to 1 to enable DAFCA trace and output pin muxing



#if 0

#include <asm-arm/arch-bcm4760/spi.h>

/*-----------------------------------------------------------------------------
 *  System specific defines
 *---------------------------------------------------------------------------*/
//#define BCM47XX_ARM_FREQ                ((unsigned long)24000000)
#define BCM47XX_ARM_FREQ                ((unsigned long) 12000000)

#define INVALID_DVFS_REQ_HANDLE         ((unsigned int)-1)
#define SD_DVFS_REQ_ARM_FREQ            BCM47XX_ARM_FREQ

#define SDHCI_CTRL_8BITMODE (0x1<<5)
#define SDHCI_BLOCK_REQ_MAX_NUM_SECTORS     128
#define SDHCI_BLOCK_REQ_MAX_NUM_SEGMENTS    128
#define SDHCI_BLOCK_REQ_SEG_BOUNDARY        0x80000        /* which is (512 * 1024)  */
#define SDHCI_BLOCK_REQ_SEG_BOUNDARY_MASK   (0x80000 - 1)  /* which is (512 * 1024 - 1) */

#define SDHCI_HOST_REQUEST_BUF_SIZE_IN_PAGE_ORDER  3


/*Somehow these reg's weren't defined in mach
specific header's.So we go ahead and define them
ourselves*/
#define CMSTOR  ((unsigned long )(0x0c0d1000 + 0x0070))
#define CMSDIO1 ((unsigned long )(0x0c0d1000 + 0x0064))
#define CMPLL1  ((unsigned long )(0x0c0d1000 + 0x0004))
/*This is ugly.We use this as we do not have acess to unified clock interface*/
#define  set_ceata_clk() do{                                          \
        u32 regval=0;                                                 \
        /*regval|=(1<<8)|(1<<1);*/                                    \
        regval=readl(ioremap(CMSTOR,sizeof(u32)));                    \
        printk("Initialized CEATA clock ...\n");                      \
        /*printk("CMSTOR=0x%08x\n",regval);*/                         \
        regval^=(1<<3);                                               \
        /*printk("CMPLL1=0x%08x\n",(int)readl(ioremap(CMPLL1,sizeof(u32))));*/\
        /*printk("CMSDIO1=0x%08x\n",(int)readl(ioremap(CMSDIO1,sizeof(u32))));*/\
        writel(regval,ioremap(CMSTOR,sizeof(u32)));                   \
        printk("CMSTOR=0x%08x\n",(int)readl(ioremap(CMSTOR,sizeof(u32))));\
        }while(0)

/* VC3 reset GPIO pin    */
#define HW_VC3_RUN_GPIO                                     2
#define HW_GPIO_VID_INT_PIN                                 1
#define HW_HOSTPORT_BUSCFG                                  0x40000803

/* used by vc03_hostport_umi.c */
#define HW_VC3_ADR_SHIFT                                    22

/* LCD display size */
#define HW_LCD_WIDTH										800
#define HW_LCD_HEIGHT										480

/*-----------------------------------------------------------------------------
 *  Serial flash/SPI specific defines
 *---------------------------------------------------------------------------*/
#define HW_SPI_PRESCALE_DIVISOR         (2&SPI_SSPCPSR_MASK)
#define HW_SPI_SERIAL_CLOCK_DIVISOR     (2&SPI_SSPCR0_SCR_MASK)

#endif

#endif /* __HW_CFG */
