/* arch/arm/mach-s3c2410/tomtomgo-type.c
 *
 * Definitions for different TomTom GO types.
 *
 * Copyright (C) 2004,2005,2006,2007,2008 TomTom BV <http://www.tomtom.com/>
 * Authors:
 * Jeroen Taverne <jeroen.taverne@tomtom.com>
 * Dimitry Andric <dimitry.andric@tomtom.com>
 * Mark-Jan Bastian <mark-jan.bastian@tomtom.com>
 * Kwok Wong <kwok.wong@tomtom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef __KERNEL__
#include <linux/kernel.h>		/* for general stuff (e.g. printk) */
#include <linux/module.h>		/* for EXPORT_SYMBOL */
#include <linux/delay.h>		/* for mdelay */
#include <asm/system.h>			/* for local_irq_save/restore */
#include <asm/arch/map.h>		/* for S3C2410_VA_GPIO */
#include <barcelona/gopins.h>
#include <barcelona/goboard.h>
#include <barcelona/gotype.h>
#include <asm/arch/pm.h>
#include <asm/arch/regs-gpio.h>
#ifdef CONFIG_BARCELONA_DOCK
#include <barcelona/dock.h>
#endif
#define GP_BASE		S3C24XX_VA_GPIO
#define DBG(fmt, arg...) printk(KERN_DEBUG "%s: " fmt, __func__ ,##arg)
#define INF(fmt, arg...) printk(KERN_INFO fmt ,##arg)
#endif /* __KERNEL__ */

#ifdef CONFIG_SMDK2410_BOARD
#define CONFIG_SMDK
#endif
#ifdef CONFIG_SMDK2413_BOARD
#define CONFIG_SMDK
#endif
#ifdef CONFIG_SMDK2440_BOARD
#define CONFIG_SMDK
#endif
#ifdef CONFIG_SMDK2443_BOARD
#define CONFIG_SMDK
#endif

#ifdef __BOOTLOADER__
#define GP_BASE		0x56000000
#define DBG(x)
#define INF(x,y,z)
#include "option.h"
#include "compiler.h"
#include "irqsave.h"
#include "timer.h"
#include "gopins.h"
#include "gotype.h"
#include "goboard.h"
#include "poweroff.h"
#include "cpu.h"
#include "flash.h"
#include "2443addr.h"
#include <string.h>
#define EXPORT_SYMBOL(x)
#endif  /* __BOOTLOADER__ */

#include "tomtomgo-ioexp.h"

#define ID_S3C2410 0x32410000
#define ID_S3C2412 0x32412000
#define ID_S3C2440 0x32440000
#define ID_S3C2442 0x32440aa0
#define ID_S3C2443 0x32443000
#define ID_S3C2450 0x32450000

#define rS3C2443_DATAPDEN  (*(volatile unsigned *)(GP_BASE + 0x000000e8))
#define rS3C2450_PDDMCON   (*(volatile unsigned *)(GP_BASE + 0x00000114))
#define rS3C2450_PDSMCON   (*(volatile unsigned *)(GP_BASE + 0x00000118))
#define rS3C2412_MSLCON    (*(volatile unsigned *)(GP_BASE + 0x000000d8))

#define rS3C24XX_GSTATUS0  (*(volatile unsigned *)(GP_BASE + 0x000000ac))
#define rS3C24XX_GSTATUS1  (*(volatile unsigned *)(GP_BASE + 0x000000b0))
#define rS3C24XX_GSTATUS2  (*(volatile unsigned *)(GP_BASE + 0x000000b4))
#define rS3C24XX_GSTATUS3  (*(volatile unsigned *)(GP_BASE + 0x000000b8))
#define rS3C24XX_GSTATUS4  (*(volatile unsigned *)(GP_BASE + 0x000000bc))

#define rS3C24XX_MISCCR    (*(volatile unsigned *)(GP_BASE + 0x00000080))

#define rGP_CON(p)         (*(volatile unsigned*)(GP_BASE + ((p) << 4) + 0x0))
#define rGP_DAT(p)         (*(volatile unsigned*)(GP_BASE + ((p) << 4) + 0x4))
#define rGP_PULL(p)        (*(volatile unsigned*)(GP_BASE + ((p) << 4) + 0x8))
#define rGP_SLPCON(p)      (*(volatile unsigned*)(GP_BASE + ((p) << 4) + 0xc))
//rGP_SEL on 2416, 2450
#define rGP_SEL(p)         (*(volatile unsigned*)(GP_BASE + ((p) << 4) + 0xc))
#define rS3C24XX_EXTINT(p) (*(volatile unsigned *)(GP_BASE + 0x88 + ((p) << 2)))
#define rS3C2412_EXTINT(p) (*(volatile unsigned *)(GP_BASE + 0x98 + ((p) << 2)))
#define rS3C2450_EXTINT_PULL(p) (*(volatile unsigned *)(GP_BASE + 0x58 + ((p) << 4)))

/* Get pin definition macros */
#include "tomtomgo-iopins.h"

/* s3c2443-bugs.h */
extern unsigned s3c2443_read_extint1(void);

#ifdef __BOOTLOADER__
size_t strlcpy(char *destination, char *source, size_t length)
{
	while (length > 1)
	{
		if (*source == 0) break;
		*(destination++) = *(source++);
		length--;
	}
	while (length)
	{
		*(destination++) = 0;
		length--;
	}
	return 0;
}
#endif

#if defined CONFIG_SMDK
#include "tomtomgo-iosmdk2413.h"
#include "tomtomgo-iosmdk2440.h"
#include "tomtomgo-iosmdk2443.h"
#else
#include "tomtomgo-ioclassic.h"
#include "tomtomgo-iom100.h"
#include "tomtomgo-iom300.h"
#include "tomtomgo-iom500.h"
#include "tomtomgo-ioatlanta.h"
#include "tomtomgo-iobilbao.h"
#include "tomtomgo-iolisbon.h"
#include "tomtomgo-ioglasgow.h"
#include "tomtomgo-iovalencia.h"
#include "tomtomgo-iomurcia.h"
#include "tomtomgo-ioaberdeen.h"
#include "tomtomgo-ioedinburgh.h"
#include "tomtomgo-iocologne.h"
#include "tomtomgo-ionewcastle.h"
#include "tomtomgo-iocork.h"
#include "tomtomgo-iomilan.h"
#include "tomtomgo-iocairns.h"
#include "tomtomgo-iolimerick.h"
#include "tomtomgo-iocasablanca-sirf.h"
#include "tomtomgo-iocasablanca-gl.h"
#include "tomtomgo-iocasablanca-cn.h"
#include "tomtomgo-ioknock.h"
#include "tomtomgo-iodublin.h"
#include "tomtomgo-iopalermo.h"
#include "tomtomgo-iocagliari.h"
#include "tomtomgo-iomodena.h"
#include "tomtomgo-iorome.h"
#include "tomtomgo-iofayetteville-casablanca.h"
#include "tomtomgo-iofayetteville-limerick.h"
#include "tomtomgo-iopaola.h"
#include "tomtomgo-ioflorence.h"
#include "tomtomgo-iolivorno.h"
#include "tomtomgo-ioaustin.h"
#include "tomtomgo-iotreviso.h"
#include "tomtomgo-iodurban.h"
#include "tomtomgo-ioxiamen.h"
#include "tomtomgo-iobergamo.h"
#include "tomtomgo-iorider3.h"
#include "tomtomgo-ioboston.h"
#include "tomtomgo-iorider5.h"
#include "tomtomgo-ioacton.h"
#include "tomtomgo-ioanqing.h"
#endif
#include "tomtomgo-iopullresistors.h"
#include "tomtomgo-iosuspend.h"

#ifdef __KERNEL__
#define IO_SetInterruptType set_irq_type
#endif /*	__KERNEL__	*/

#ifdef __BOOTLOADER__
#define IRQ_EINT0  0
#define IRQ_EINT1  1
#define IRQ_EINT2  2
#define IRQ_EINT3  3
#define IRQ_EINT4  4
#define IRQ_EINT5  5
#define IRQ_EINT6  6
#define IRQ_EINT7  7
#define IRQ_EINT8  8
#define IRQ_EINT9  9
#define IRQ_EINT10 10
#define IRQ_EINT11 11
#define IRQ_EINT12 12
#define IRQ_EINT13 13
#define IRQ_EINT14 14
#define IRQ_EINT15 15
#define IRQ_EINT16 16
#define IRQ_EINT17 17
#define IRQ_EINT18 18
#define IRQ_EINT19 19
#define IRQ_EINT20 20
#define IRQ_EINT21 21
#define IRQ_EINT22 22
#define IRQ_EINT23 23

#define IRQT_LOW      (0 | 8)
#define IRQT_HIGH     (1 | 8) 
#define IRQT_FALLING  (2 | 8)
#define IRQT_RISING   (4 | 8)
#define IRQT_BOTHEDGE (6 | 8)
#endif /* __BOOTLOADER__ */

#define CONNECTED	1
#define NOTCONNECTED	0

// Use an array so the defines can change without problems
static const signed irqList[] =
{
	IRQ_EINT0,
	IRQ_EINT1,
	IRQ_EINT2,
	IRQ_EINT3,
	IRQ_EINT4,
	IRQ_EINT5,
	IRQ_EINT6,
	IRQ_EINT7,
	IRQ_EINT8,
	IRQ_EINT9,
	IRQ_EINT10,
	IRQ_EINT11,
	IRQ_EINT12,
	IRQ_EINT13,
	IRQ_EINT14,
	IRQ_EINT15,
	IRQ_EINT16,
	IRQ_EINT17,
	IRQ_EINT18,
	IRQ_EINT19,
	IRQ_EINT20,
	IRQ_EINT21,
	IRQ_EINT22,
	IRQ_EINT23
};

/* Currently detected GO type */
struct gotype gotype_current;
EXPORT_SYMBOL(gotype_current);

/* Data from and to extended dock */
static unsigned int dockInputData = 0xff;
static unsigned int dockOutputData = 0x00;

/* Hardware detection */
static unsigned id;
static char s3c2412detected = 0;
static char s3c2443detected = 0;
static char s3c2442detected = 0;
static char s3c2440detected = 0;
static char s3c2410detected = 0;
static char s3c2450detected = 0;

#ifdef __KERNEL__
#include <asm/mach/irq.h>
extern spinlock_t irq_controller_lock;

/* Since this function is specifically needed in this code we add it here. */
void clear_irq_nosync( unsigned int irq )
{
	struct irqdesc          *desc=irq_desc+irq;
	unsigned long int       flags;

	spin_lock_irqsave( &irq_controller_lock, flags );
	desc->chip->ack( irq );
	spin_unlock_irqrestore( &irq_controller_lock, flags );
	return;
}
#endif /* __KERNEL__ */

#ifdef __BOOTLOADER__
void clear_irq_nosync( unsigned int irq )
{
	unsigned long int       flags;
	unsigned long bitval = 1UL << (irq - IRQ_EINT0);

	local_irq_save(flags);
	rSRCPND=bitval;
	rINTPND=bitval;
	local_irq_restore(flags);
	return;
}
#endif /* __BOOTLOADER__ */

#ifdef __BOOTLOADER__
static void IO_SetInterruptType(signed intNr,unsigned int intType)
{
	unsigned long flags;
	static unsigned extint[3] = {0xffffffff,0xffffffff,0xffffffff};

	local_irq_save(flags);

	if ( s3c2443detected || s3c2450detected ) //assume 2450 has the same bug
	{
		/* S3C2443 has bug reading EXTINT registers, so use array */
		extint[intNr >> 3] &= ~(0x7 << ((intNr & 7) << 2));
		extint[intNr >> 3] |= (intType << ((intNr & 7) << 2));
		rS3C24XX_EXTINT(intNr >> 3) = extint[intNr >> 3];
	}
	else
	if (s3c2412detected)
	{
		rS3C2412_EXTINT(intNr >> 3) &= ~(0xf << ((intNr & 7) << 2));
		rS3C2412_EXTINT(intNr >> 3) |= (intType << ((intNr & 7) << 2));
	}
	else
	{
		rS3C24XX_EXTINT(intNr >> 3) &= ~(0xf << ((intNr & 7) << 2));
		rS3C24XX_EXTINT(intNr >> 3) |= (intType << ((intNr & 7) << 2));
	}

	local_irq_restore(flags);
}
#endif /* __BOOTLOADER__	*/

static void IO_NoHarddiskDetected(void)
{
	switch (id)
	{
#ifdef SUPPORT_VALENCIA
	case GOBOARD_VALENCIA:
		IO_InitValencia500();
		break;
#endif
#ifdef SUPPORT_MURCIA
	case GOBOARD_MURCIA:
		IO_InitMurcia500();
		break;
#endif
	}
}

static void IO_HarddiskDetected(void)
{
	switch (id)
	{
#ifdef SUPPORT_VALENCIA
	case GOBOARD_VALENCIA:
		IO_InitValencia700();
		break;
#endif
#ifdef SUPPORT_MURCIA
	case GOBOARD_MURCIA:
		IO_InitMurcia700();
		break;
#endif
	}
}

/*
 * Initialize the default gopin structures
 */
void IO_InitDefaults(void)
{
	/* default for 'older' devices */
	VALUE_SET(tsdownchannel, 7);
	VALUE_SET(adc_ref_value, 3300);
	VALUE_SET(lightsensorchannel, 2);
	VALUE_SET(batvoltagechannel, 0);
	VALUE_SET(batchargecurrentchannel, 6);
	VALUE_SET(refvoltagechannel, 4);
}

void IO_InitMainType(void)
{
	IO_InitDefaults();
	switch (id) 
	{
#ifdef SUPPORT_CLASSIC
	case GOBOARD_CLASSIC:
		IO_InitClassic();
		break;
#endif
#ifdef SUPPORT_M100
	case GOBOARD_M100:
		IO_InitM100();
		break;
#endif
#ifdef SUPPORT_M300
	case GOBOARD_M300:
		IO_InitM300();
		break;
#endif
#ifdef SUPPORT_M500
	case GOBOARD_M500:
		IO_InitM500();
		break;
#endif
#ifdef SUPPORT_ATLANTA
	case GOBOARD_ATLANTA:
		IO_InitAtlanta();
		break;
#endif
#ifdef SUPPORT_VALENCIA
	case GOBOARD_VALENCIA:
		IO_InitValencia();
		break;
#endif
#ifdef SUPPORT_MURCIA
	case GOBOARD_MURCIA:
		IO_InitMurcia();
		break;
#endif
#ifdef SUPPORT_ABERDEEN
	case GOBOARD_ABERDEEN:
		IO_InitAberdeen();
		break;
#endif
#ifdef SUPPORT_EDINBURGH
	case GOBOARD_EDINBURGH:
		IO_InitEdinburgh();
		break;
#endif
#ifdef SUPPORT_CORK_FYRESTORM
	case GOBOARD_CORK_FYRESTORM:
		IO_InitCorkFyrestorm();
		break;
#endif
#ifdef SUPPORT_CORK
	case GOBOARD_CORK:
		IO_InitCork();
		break;
#endif
#ifdef SUPPORT_COLOGNE
	case GOBOARD_COLOGNE:
		IO_InitCologne();
		break;
#endif
#ifdef SUPPORT_NEWCASTLE
	case GOBOARD_NEWCASTLE:
		IO_InitNewcastle();
		break;
#endif
#ifdef SUPPORT_MILAN
	case GOBOARD_MILAN:
		IO_InitMilan();
		break;
#endif
#ifdef SUPPORT_CAIRNS
	case GOBOARD_CAIRNS:
		IO_InitCairns();
		break;
#endif
#ifdef SUPPORT_LIMERICK
	case GOBOARD_LIMERICK:
		IO_InitLimerick();
		break;
#endif
#ifdef SUPPORT_CASABLANCA_SIRF
	case GOBOARD_CASABLANCA_SIRF:
		IO_InitCasablancaSirf();
		break;
#endif
#ifdef SUPPORT_CASABLANCA_GL
	case GOBOARD_CASABLANCA_GL:
		IO_InitCasablancaGl();
		break;
#endif
#ifdef SUPPORT_CASABLANCA_CN
	case GOBOARD_CASABLANCA_CN:
		IO_InitCasablancaCn();
		break;
#endif
#ifdef SUPPORT_PALERMO
	case GOBOARD_PALERMO:
		IO_InitPalermo();
		break;
#endif
#ifdef SUPPORT_KNOCK
	case GOBOARD_KNOCK:
		IO_InitKnock();
		break;
#endif
#ifdef SUPPORT_DUBLIN
	case GOBOARD_DUBLIN:
		IO_InitDublin();
		break;
#endif
#ifdef SUPPORT_CAGLIARI
	case GOBOARD_CAGLIARI:
		IO_InitCagliari();
		break;
#endif
#ifdef SUPPORT_RIDER5
	case GOBOARD_RIDER5:
		IO_InitRider5();
		break;
#endif
#ifdef SUPPORT_MODENA
	case GOBOARD_MODENA:
		IO_InitModena();
		break;
#endif
#ifdef SUPPORT_ROME
	case GOBOARD_ROME:
		IO_InitRome();
		break;
#endif
#ifdef SUPPORT_MARIGOT
	case GOBOARD_MARIGOT:
		IO_InitMarigot();
		break;
#endif
#ifdef SUPPORT_FAYETTEVILLE_CASABLANCA
	case GOBOARD_FAYETTEVILLE_CASABLANCA:
		IO_InitFayettevilleCasablanca();
		break;
#endif
#ifdef SUPPORT_FAYETTEVILLE_LIMERICK
	case GOBOARD_FAYETTEVILLE_LIMERICK:
		IO_InitFayettevilleLimerick();
		break;
#endif
#ifdef SUPPORT_SMDK2440
	case GOBOARD_S3C2440:
		IO_InitSMDK2440();
		break;
#endif
#ifdef SUPPORT_SMDK2413
	case GOBOARD_S3C2412:
		IO_InitSMDK2413();
		break;
#endif
#ifdef SUPPORT_SMDK2443
	case GOBOARD_S3C2443:
		IO_InitSMDK2443();
		break;
#endif
#ifdef SUPPORT_PAOLA
	case GOBOARD_PAOLA:
		IO_InitPaola();
		break;
#endif
#ifdef SUPPORT_LIVORNO
	case GOBOARD_LIVORNO:
		IO_InitLivorno();
		break;
#endif
#ifdef SUPPORT_AUSTIN
	case GOBOARD_AUSTIN:
		IO_InitAustin();
		break;
#endif
#ifdef SUPPORT_ACTON
	case GOBOARD_ACTON:
		IO_InitActon();
		break;
#endif
#ifdef SUPPORT_ANQING
    case GOBOARD_ANQING:
        IO_InitAnqing();
        break;
#endif
#ifdef SUPPORT_FLORENCE
	case GOBOARD_FLORENCE:
		IO_InitFlorence();
		break;
#endif
#ifdef SUPPORT_DURBAN
	case GOBOARD_DURBAN:
		IO_InitDurban();
		break;
#endif
#ifdef SUPPORT_TREVISO
	case GOBOARD_TREVISO:
		IO_InitTreviso();
		break;
#endif
#ifdef SUPPORT_XIAMEN
	case GOBOARD_XIAMEN:
		IO_InitXiamen();
		break;
#endif
#ifdef SUPPORT_BERGAMO
	case GOBOARD_BERGAMO:
		IO_InitBergamo();
		break;
#endif
#ifdef SUPPORT_BOSTON
	case GOBOARD_BOSTON:
		IO_InitBoston();
		break;
#endif
#ifdef SUPPORT_RIDER3
	case GOBOARD_RIDER3:
		IO_InitRider3();
		break;
#endif
	default:
		// Flash the backlight to show ID error
		IOP_Activate(PIN_GPB1 | PIN_INVERTED);
		while (1)
		{
			IOP_Activate(PIN_GPB0);
			mdelay(200);
			IOP_Deactivate(PIN_GPB0);
			mdelay(200);
		}
	}
	
	// Setup IO pins for HSMMC
	if (IO_HaveHsMmcInterface())
	{
		PIN_SET(HS_SDCLK   , PIN_GPL9);
		PIN_SET(HS_SDCMD   , PIN_GPL8);
		PIN_SET(HS_SDDATA0 , PIN_GPL0);
		PIN_SET(HS_SDDATA1 , PIN_GPL1);
		PIN_SET(HS_SDDATA2 , PIN_GPL2);
		PIN_SET(HS_SDDATA3 , PIN_GPL3);
		PIN_SET(HS_SDDATA4 , PIN_GPL4);
		PIN_SET(HS_SDDATA5 , PIN_GPL5);
		PIN_SET(HS_SDDATA6 , PIN_GPL6);
		PIN_SET(HS_SDDATA7 , PIN_GPL7);
	}

	if (IO_HaveHsMmcInterface0_4bit())
	{
		PIN_SET(HS0_SDCLK   , PIN_GPE5);
		PIN_SET(HS0_SDCMD   , PIN_GPE6);
		PIN_SET(HS0_SDDATA0 , PIN_GPE7);
		PIN_SET(HS0_SDDATA1 , PIN_GPE8);
		PIN_SET(HS0_SDDATA2 , PIN_GPE9);
		PIN_SET(HS0_SDDATA3 , PIN_GPE10);
	}

	if (IO_HaveHsMmcInterface1_4bit())
	{
		PIN_SET(HS1_SDCLK   , PIN_GPL9);
		PIN_SET(HS1_SDCMD   , PIN_GPL8);
		PIN_SET(HS1_SDDATA0 , PIN_GPL0);
		PIN_SET(HS1_SDDATA1 , PIN_GPL1);
		PIN_SET(HS1_SDDATA2 , PIN_GPL2);
		PIN_SET(HS1_SDDATA3 , PIN_GPL3);
	}		
	
	if (IO_HaveSdCardInterface())
	{
		PIN_SET(SDCLK   , PIN_GPE5);
		PIN_SET(SDCMD   , PIN_GPE6);
		PIN_SET(SDDATA0 , PIN_GPE7);
		PIN_SET(SDDATA1 , PIN_GPE8);
		PIN_SET(SDDATA2 , PIN_GPE9);
		PIN_SET(SDDATA3 , PIN_GPE10);
	}
	
	// Setup pull resistors
	IO_InitPullResistors();

	// Setup IO pins for TFT
	{
		int bits=0, sync=0;
		switch (IO_GetTftType())
		{
			case GOTFT_NEC_NL2432HC22:
				bits = 18;
				sync = 0;
				break;
			case GOTFT_SAMSUNG_LTV350:
			case GOTFT_SAMSUNG_LTP400:
			case GOTFT_SAMSUNG_LTE430WQ:
			case GOTFT_SHARP_LQ043T1:
			case GOTFT_SAMSUNG_LMS350GF:
			case GOTFT_SHARP_LQ035Q1DG:
			case GOTFT_SHARP_LQ035Q1DG04:				
			case GOTFT_SAMSUNG_LMS430HF12:	
			case GOTFT_SAMSUNG_LMS430HF19:	
			case GOTFT_AUO_A035QN02:    	
			case GOTFT_SHARP_LQ043T3DW01:	
			case GOTFT_SAMSUNG_LMS350GF20:
			case GOTFT_AUO_A043FW03V1:
			case GOTFT_AUO_A050FW02V2:
			case GOTFT_SAMSUNG_LMS500HF01:
			case GOTFT_LG_LB043WQ3:
				bits = 24;
				sync = 1;
				break;
			case GOTFT_SAMSUNG_LTE246QV:
				bits = 18;
				sync = 1;
				break;
			default:
		        bits = 24;
		        sync = 1;
		        break;				
		}
			
		if (bits == 24)
		{
			PIN_SET(VD0  , PIN_GPC8);
			PIN_SET(VD1  , PIN_GPC9);
			PIN_SET(VD8  , PIN_GPD0);
			PIN_SET(VD9  , PIN_GPD1);
			PIN_SET(VD16 , PIN_GPD8);
			PIN_SET(VD17 , PIN_GPD9);
		}
		PIN_SET(VD2  , PIN_GPC10);
		PIN_SET(VD3  , PIN_GPC11);
		PIN_SET(VD4  , PIN_GPC12);
		PIN_SET(VD5  , PIN_GPC13);
		PIN_SET(VD6  , PIN_GPC14);
		PIN_SET(VD7  , PIN_GPC15);
		PIN_SET(VD10 , PIN_GPD2);
		PIN_SET(VD11 , PIN_GPD3);
		PIN_SET(VD12 , PIN_GPD4);
		PIN_SET(VD13 , PIN_GPD5);
		PIN_SET(VD14 , PIN_GPD6);
		PIN_SET(VD15 , PIN_GPD7);
		PIN_SET(VD18 , PIN_GPD10);
		PIN_SET(VD19 , PIN_GPD11);
		PIN_SET(VD20 , PIN_GPD12);
		PIN_SET(VD21 , PIN_GPD13);
		PIN_SET(VD22 , PIN_GPD14);
		PIN_SET(VD23 , PIN_GPD15);

		PIN_SET(VDEN , PIN_GPC4);
		PIN_SET(VCLK , PIN_GPC1);
		
		if (sync)
		{
			PIN_SET(VSYNC , PIN_GPC3);
			PIN_SET(HSYNC , PIN_GPC2);
		}
	}
}

void IO_InitSubType(void)
{
	switch (id) 
	{
	/* atlanta based units */
#ifdef SUPPORT_BILBAO
	case GOBOARD_BILBAO:
		IO_InitBilbao();
		break;
#endif
#ifdef SUPPORT_LISBON
	case GOBOARD_LISBON:
		IO_InitLisbon();
		break;
#endif
#ifdef SUPPORT_GLASGOW
	case GOBOARD_GLASGOW:
		IO_InitGlasgow();
		break;
#endif
	/* subid pins used for fast movinand availability / size detection */
#ifdef SUPPORT_CORK
	case GOBOARD_CORK_Z1Z2:
		IO_InitCorkZ1Z2();
		break;
	case GOBOARD_CORK_Z3:
		IO_InitCorkZ3();
		break;
#endif
#ifdef SUPPORT_LIMERICK
	case GOBOARD_LIMERICK_L1L2:
		IO_InitLimerickL1L2();
		break;
	case GOBOARD_LIMERICK_L3:
		IO_InitLimerickL3();
		break;
	case GOBOARD_LIMERICK_L4:
		IO_InitLimerickL4();
		break;
	case GOBOARD_LIMERICK_L5:
		IO_InitLimerickL5();
		break;
#endif
#ifdef SUPPORT_MILAN
	case GOBOARD_MILAN_M2:
		IO_InitMilanM2();
		break;
	case GOBOARD_MILAN_M4:
		IO_InitMilanM4();
		break;
	case GOBOARD_MILAN_M6:
		IO_InitMilanM6();
		break;
	case GOBOARD_MILAN_M8:
		IO_InitMilanM8();
		break;
#endif
#ifdef SUPPORT_CAIRNS
	case GOBOARD_CAIRNS_C1:
		IO_InitCairns_C1();
		break;
#endif
#ifdef SUPPORT_CASABLANCA_SIRF
	case GOBOARD_CASABLANCA_SIRF_Y1Y3:
		IO_InitCasablancaSirfY1Y3();
		break;
	case GOBOARD_CASABLANCA_SIRF_UNKNOWN:
		/* GPB8 hi */
		break;
#endif
#ifdef SUPPORT_CASABLANCA_GL
	case GOBOARD_CASABLANCA_GL_Y2Y4:
		IO_InitCasablancaGlY2Y4();
		break;
	case GOBOARD_CASABLANCA_GL_UNKNOWN:
		/* GPB8 hi */
		break;
#endif
#ifdef SUPPORT_KNOCK
        case GOBOARD_KNOCK_MOVINAND:
                IO_InitKnockMovinand();
                break;
        case GOBOARD_KNOCK_NO_MOVINAND:
                IO_InitKnockNoMovinand();
                break;
#endif
#ifdef SUPPORT_MARIGOT
	case GOBOARD_MARIGOT_M1:
		IO_InitMarigotM1();
		break;
#endif
#ifdef SUPPORT_MODENA
	case GOBOARD_MODENA_J1:
		IO_InitModenaJ1();
		break;
	case GOBOARD_MODENA_J2:
		IO_InitModenaJ2();
		break;
	case GOBOARD_MODENA_J3:
		IO_InitModenaJ3();
		break;
	case GOBOARD_MODENA_J4:
		IO_InitModenaJ4();
		break;
	case GOBOARD_MODENA_J5:
		IO_InitModenaJ5();
		break;
	case GOBOARD_MODENA_J6:
		IO_InitModenaJ6();
		break;
	case GOBOARD_MODENA_JA:
	case GOBOARD_MODENA_JB:
	case GOBOARD_MODENA_JC:
		IO_InitModenaJA_B_C(id);
		break;
	case GOBOARD_MODENA_J81:
		IO_InitModenaJ81();
		break;
	case GOBOARD_MODENA_J82:
		IO_InitModenaJ82();
		break;
	case GOBOARD_MODENA_J83:
		IO_InitModenaJ83();
		break;
	case GOBOARD_MODENA_J84:
		IO_InitModenaJ84();
		break;
	case GOBOARD_MODENA_J85:
		IO_InitModenaJ85();
		break;
	case GOBOARD_MODENA_J86:
		IO_InitModenaJ86();
		break;
	case GOBOARD_MODENA_J8A:
	case GOBOARD_MODENA_J8B:
	case GOBOARD_MODENA_J8C:
		IO_InitModenaJ8A_B_C(id);
		break;
#endif
#ifdef SUPPORT_PALERMO
	case GOBOARD_PALERMO_P1:
		IO_InitPalermoP1();
		break;
#endif
#ifdef SUPPORT_ROME
	case GOBOARD_ROME_R1:
		IO_InitRomeR1();
		break;
#endif
#ifdef SUPPORT_CAGLIARI
	case GOBOARD_CAGLIARI_540_NC:
		IO_InitCagliari540(NOTCONNECTED);
		break;
	case GOBOARD_CAGLIARI_540_C_WEU:
	case GOBOARD_CAGLIARI_540_C_US:
		IO_InitCagliari540(CONNECTED);
		break;
	case GOBOARD_CAGLIARI_740_NC:
		IO_InitCagliari740(NOTCONNECTED);
		break;
	case GOBOARD_CAGLIARI_740_C_WEU:
	case GOBOARD_CAGLIARI_740_C_US:
		IO_InitCagliari740(CONNECTED);
		break;
	case GOBOARD_CAGLIARI_940_NC:
		IO_InitCagliari940(NOTCONNECTED);
		break;
	case GOBOARD_CAGLIARI_940_C_WEU:
	case GOBOARD_CAGLIARI_940_C_US:
		IO_InitCagliari940(CONNECTED);
		break;
#endif
#ifdef SUPPORT_TREVISO
	/* ugly, use the same subids as for Cagliari */
	case GOBOARD_TREVISO_540_NC:
		IO_InitTreviso540(NOTCONNECTED);
		break;
	case GOBOARD_TREVISO_540_C_WEU:
	case GOBOARD_TREVISO_540_C_US:
		IO_InitTreviso540(CONNECTED);
		break;
	case GOBOARD_TREVISO_740_NC:
		IO_InitTreviso740(NOTCONNECTED);
		break;
	case GOBOARD_TREVISO_740_C_WEU:
	case GOBOARD_TREVISO_740_C_US:
		IO_InitTreviso740(CONNECTED);
		break;
	case GOBOARD_TREVISO_940_NC:
		IO_InitTreviso940(NOTCONNECTED);
		break;
	case GOBOARD_TREVISO_940_C_WEU:
	case GOBOARD_TREVISO_940_C_US:
		IO_InitTreviso940(CONNECTED);
		break;
#endif
#ifdef SUPPORT_XIAMEN
	case GOBOARD_XIAMEN_US:
		IO_InitXiamenUS();
		break;
#endif
#ifdef SUPPORT_BERGAMO
	case GOBOARD_BERGAMO_US:
		IO_InitBergamoUS();
		break;

	case GOBOARD_CHELSEA4:
		IO_InitChelsea4( );
		break;
#endif
#ifdef SUPPORT_ANQING
	case GOBOARD_CHELSEA5:
		IO_InitChelsea5( );
		break;
#endif

#ifdef SUPPORT_FLORENCE
	case GOBOARD_FLORENCE_FLORIN:
		IO_InitFlorenceFlorin();
		break;
#endif
	}
	// Setup pull resistors
	IO_InitPullResistors();
}

void IO_DetectCpu(void)
{
	switch (rS3C24XX_GSTATUS1 & 0xfffffff0)
	{
		case ID_S3C2442:
			s3c2442detected = 1;
			break;
			
		case ID_S3C2440:
			s3c2440detected = 1;
			break;
			
		case ID_S3C2410:
			s3c2410detected = 1;
			break;

		case ID_S3C2450:
			// Reset EXTINT registers to make sure pulldowns on interrupt pins are disabeled at startup
			// also need to disable pull-down ??? /
			rS3C24XX_EXTINT(0) = 0x00000000; // enable filters */
			rS3C24XX_EXTINT(1) = 0x00000000; // enable filters */
			rS3C24XX_EXTINT(2) = 0x00000000; // enable filters */
			rS3C2450_EXTINT_PULL(0) = 0; // disable pull-down */
			rS3C2450_EXTINT_PULL(1) = 0; // disable pull-down */
			s3c2450detected = 1;
			break;

		case ID_S3C2443:
			// Reset EXTINT registers to make sure pulldowns on interrupt pins are disabeled at startup
			rS3C24XX_EXTINT(0) = 0x88888888; /* disable pull-down */
			rS3C24XX_EXTINT(1) = 0x88888888; /* disable pull-down */
			rS3C24XX_EXTINT(2) = 0x88888888; /* disable filter */
			s3c2443detected = 1;
			break;

		default:
			s3c2412detected = 1;
			break;
	}
}

/* for legacy devices without mainID in NOR flash */
void IO_DetectMainType(void)
{
#ifdef CONFIG_SMDK
	if (s3c2410detected) id |= GOBOARD_S3C2410; else
	if (s3c2440detected) id |= GOBOARD_S3C2440; else
	if (s3c2442detected) id |= GOBOARD_S3C2442; else
	if (s3c2443detected) id |= GOBOARD_S3C2443; else
	if (s3c2412detected) id |= GOBOARD_S3C2412;
#else
	id |= GOBOARD_GPIO;
	
	if (s3c2442detected)
	{
		id |= GOBOARD_S3C2442;
		PIN_SET(TYPE_MAIN_ID0 , PIN_GPB5 | PIN_PULL_DOWN);
		PIN_SET(TYPE_MAIN_ID1 , PIN_GPB6 | PIN_PULL_DOWN);
	}
	else
	if (s3c2440detected)
	{
		id |= GOBOARD_S3C2440;
		PIN_SET(TYPE_NEC_LCD , PIN_GPB8 | PIN_PULL_UP);
		IO_SetInput(TYPE_NEC_LCD);
		mdelay(1);
		if (IO_GetInput(TYPE_NEC_LCD)) 
		{
			PIN_SET(TYPE_MAIN_ID0 , PIN_GPC2 | PIN_PULL_UP);
			PIN_SET(TYPE_MAIN_ID1 , PIN_GPC3 | PIN_PULL_UP);
		} 
		else 
		{
			PIN_SET(TYPE_MAIN_ID0 , PIN_GPB5 | PIN_PULL_UP);
			PIN_SET(TYPE_MAIN_ID1 , PIN_GPB6 | PIN_PULL_UP);
			PIN_SET(TYPE_MAIN_ID2 , PIN_GPB7 | PIN_PULL_UP);
		}
	}
	else
	if (s3c2410detected)
	{
		id |= GOBOARD_S3C2410;
		PIN_SET(TYPE_NEC_LCD  , PIN_GPB8 | PIN_PULL_UP);
		PIN_SET(TYPE_MAIN_ID0 , PIN_GPC2 | PIN_PULL_UP);
		PIN_SET(TYPE_MAIN_ID1 , PIN_GPC3 | PIN_PULL_UP);
	}
	else
	if (s3c2443detected)
	{
		// Reset EXTINT registers to make sure pulldowns on interrupt pins are disabeled at startup
		rS3C24XX_EXTINT(0) = 0x88888888;
		rS3C24XX_EXTINT(1) = 0x88888888;
		rS3C24XX_EXTINT(2) = 0x88888888;

		id |= GOBOARD_S3C2443;
		PIN_SET(TYPE_MAIN_ID0 , PIN_GPB5 | PIN_PULL_DOWN);
		PIN_SET(TYPE_MAIN_ID1 , PIN_GPB6 | PIN_PULL_DOWN);
	}
	else
	if (s3c2412detected)
	{
		id |= GOBOARD_S3C2412;
		PIN_SET(TYPE_MAIN_ID0 , PIN_GPB5 | PIN_PULL_DOWN);
		PIN_SET(TYPE_MAIN_ID1 , PIN_GPB6 | PIN_PULL_DOWN);
		PIN_SET(TYPE_MAIN_ID2 , PIN_GPB7 | PIN_PULL_DOWN);
	}
	
	/* Read main id */
	IO_SetInput(TYPE_MAIN_ID0);
	IO_SetInput(TYPE_MAIN_ID1);
	IO_SetInput(TYPE_MAIN_ID2);
	IO_SetInput(TYPE_NEC_LCD);
	mdelay(1);
	id |= (IO_GetInput(TYPE_MAIN_ID2) * GOBOARD_ID2_HI) | (IO_GetInput(TYPE_MAIN_ID1) * GOBOARD_ID1_HI) | (IO_GetInput(TYPE_MAIN_ID0) * GOBOARD_ID0_HI);
	if (IO_GetInput(TYPE_NEC_LCD)) id |= GOBOARD_NEC_TFT; else id |= GOBOARD_SAMSUNG_TFT;
#endif
}	

void IO_DetectHardwareSubType(void)
{
	/* Make sure pins are low before reading them (pulldown on GPE15 is missing inside S3C2412) */
	IO_Deactivate(TYPE_SUB_ID0);
	IO_Deactivate(TYPE_SUB_ID1);
	IO_SetInput(TYPE_SUB_ID0);
	IO_SetInput(TYPE_SUB_ID1);
	mdelay(1);
	id |= (IO_GetInput(TYPE_SUB_ID1) * GOBOARD_ID4_HI) | (IO_GetInput(TYPE_SUB_ID0) * GOBOARD_ID3_HI);
}

void IO_DetectLCMType(void)
{
	int lcm_alt = 0;
	int lcdon = 0;
#ifdef __KERNEL__
	extern unsigned int atag_tfttype;
	if (atag_tfttype == GOTFT_UNDEFINED) {
		atag_tfttype = GOTFT_HWDETECT;
	}
#else
	unsigned short flash_tfttype;	

	flash_tfttype = FLASH_GetLCMType();
	if (flash_tfttype == GOTFT_UNDEFINED) {
		flash_tfttype = GOTFT_HWDETECT;
	}
#endif
	/* limerick needs power before ID detection, pullup connected to 'wrong' power rail */
	if (IO_NeedsVccLcmForLCMIDDetect()) {
		lcdon = IO_GetInput(LCD_VCC_PWREN);
		if (!lcdon) {
			IO_Activate(LCD_VCC_PWREN);
			mdelay(10);
		}
	}

	// Detect TFT type
	if (IO_HasPin(LCD_ID)) {
		IO_SetInput(LCD_ID);
		mdelay(1);
		lcm_alt = IO_GetInput(LCD_ID);
	}

	
#ifdef __KERNEL__
// if kernel needs to support more than  2 LCM per model, it will need the
// atag_tfttype.
	if( lcm_alt && atag_tfttype!=GOTFT_HWDETECT ) {
	    VALUE_SET(tfttype , (unsigned char) atag_tfttype);
	} else
#else
	// use flash_tfttype only if lcm_alt is high && flash_tfttype is valid
	if( lcm_alt && flash_tfttype != GOTFT_HWDETECT) {
	    VALUE_SET(tfttype , (unsigned char) flash_tfttype);
	} else
#endif	
	{
		// Configure type for multi-LCM capable units
		switch (IO_GetModelId()) {
			/* 4.3" LCM options */
			case GOTYPE_TREVISO:
				if (lcm_alt) {
					VALUE_SET(tfttype, GOTFT_SHARP_LQ043T3DW02);
				} else {
					VALUE_SET(tfttype, GOTFT_SAMSUNG_LTE430WQ);
				}
				break;
			case GOTYPE_CAGLIARI:
				if (lcm_alt) {
					VALUE_SET(tfttype, GOTFT_SHARP_LQ043T3DW02);
				} else {
					VALUE_SET(tfttype , GOTFT_SAMSUNG_LTE430WQ);
					if (IO_GetBacklightType() == GOBACKLIGHT_FB_CH0_20K_CAT3238TD_430) {
						/* don't touch the frequency */
					} else {
						VALUE_SET(backlightfreq, 1839);
						VALUE_SET(backlighttype, GOBACKLIGHT_CH0_1839_CAT3238TD_430);
					}
				}
				break;
			case GOTYPE_MILAN:
			case GOTYPE_MODENA:
			case GOTYPE_CAIRNS:
			case GOTYPE_LIMERICK:
			case GOTYPE_KNOCK:
			case GOTYPE_FAYETTEVILLE_LIMERICK:
				if (lcm_alt) {
					VALUE_SET(tfttype , GOTFT_SHARP_LQ043T1);
				} else {
					VALUE_SET(tfttype , GOTFT_SAMSUNG_LTE430WQ);
					if (IO_GetBacklightType() == GOBACKLIGHT_FB_CH0_20K_CAT3238TD_430) {
						/* don't touch the frequency */
					} else {
						VALUE_SET(backlightfreq, 1839);
						VALUE_SET(backlighttype, GOBACKLIGHT_CH0_1839_CAT3238TD_430);
					}
				}
				break;
			/* 3.5" LCM options */
			case GOTYPE_CASABLANCA:
				if ( (lcm_alt) || (atag_tfttype == GOTFT_SHARP_LQ035Q1DG) ) {
					VALUE_SET(tfttype, GOTFT_SHARP_LQ035Q1DG);
				} else {
					VALUE_SET(tfttype, GOTFT_SAMSUNG_LMS350GF);
				}
				break;

			case GOTYPE_PALERMO:
			case GOTYPE_PAOLA:
				if (lcm_alt)
					VALUE_SET(tfttype, GOTFT_SHARP_LQ035Q1DG04);
				else if ( atag_tfttype == GOTFT_HWDETECT ) 
					VALUE_SET(tfttype, GOTFT_SAMSUNG_LMS350GF);
				else
					VALUE_SET(tfttype, atag_tfttype);
  			break;			
			/* 4.3" LCM options */
			case GOTYPE_ROME:
			case GOTYPE_LIVORNO:
			case GOTYPE_FLORENCE:
		 		if (lcm_alt)
					VALUE_SET(tfttype, GOTFT_SHARP_LQ043T3DW01);
				else if ( atag_tfttype == GOTFT_HWDETECT ) 
			        	VALUE_SET(tfttype, GOTFT_SAMSUNG_LMS430HF12);
				else
					VALUE_SET(tfttype, atag_tfttype);
  			break;			
			case GOTYPE_BOSTON:
			case GOTYPE_BERGAMO:
			case GOTYPE_CHELSEA4:
		 		if (lcm_alt)
					VALUE_SET(tfttype, GOTFT_AUO_A043FW03V1);
				else if ( atag_tfttype == GOTFT_HWDETECT ) 
			       	VALUE_SET(tfttype, GOTFT_SAMSUNG_LMS430HF19);
				else
					VALUE_SET(tfttype, atag_tfttype);
  			break;			
		     /* 5" LCM options */
			case GOTYPE_ACTON:
			case GOTYPE_AUSTIN:
			case GOTYPE_ANQING:
			case GOTYPE_CHELSEA5:
		 		if (lcm_alt) {
			    	VALUE_SET(tfttype, GOTFT_AUO_A050FW02V2);
				} else {
			    	VALUE_SET(tfttype, GOTFT_SAMSUNG_LMS500HF01);
			  	}
  			break;			
		} // switch
	
		// Turn off pullup power Limerick
		if (IO_NeedsVccLcmForLCMIDDetect()) {
			if (!lcdon) {
				IO_Deactivate(LCD_VCC_PWREN);
			}
		}
	}
}

void IO_DetectFeature(void) {
	unsigned int feature[2];
	unsigned mainId;

#ifdef __KERNEL__
	extern unsigned system_rev;
	extern unsigned int atag_feature[];

	mainId = system_rev & 0x0000ffff;
	
	feature[0] = atag_feature[0];
	feature[1] = atag_feature[1];
#else
	mainId = FLASH_GetMainId();
	memcpy(feature,FLASH_GetFeature(),2*sizeof(unsigned int));
#endif
	if (mainId && ((~feature[0]) & (1 << GOFEATURE_TTS) )) {
		VALUE_SET(loquendo, 1 );   
	}
	if (mainId && ((~feature[0]) & (1 << GOFEATURE_ALG) )) {
		VALUE_SET(advancedlaneguidance, 1 );   
	}
	if (mainId && ((~feature[0]) & (1 << GOFEATURE_RDSTMC) )) {
		VALUE_SET(rdstmc, 1 );   
	}
	if (mainId && ((~feature[0]) & (1 << GOFEATURE_TTW) )) {
		VALUE_SET(tomtom_work, 1 );   
	}
	if (mainId && ((~feature[0]) & (1 << GOFEATURE_SIXBUTTONUI) )) {
		VALUE_SET(sixbuttonui, 1 );   
	}
}

void IO_InitDevices(void)
{
	IO_DetectLCMType();

	if (IO_HasPin(FLEX_ID1))
	{
		// Use SD insertion signal to check if SD is inserted
		IO_SetInput(CD_SD);
		mdelay(1);
		if (IO_GetInput(CD_SD))
		{
			IO_NoHarddiskDetected();
		}
		else
		{
			// Use flexcable ID to check if SD slot is available
			IO_SetInput(FLEX_ID1);
			IO_SetInput(FLEX_ID2);
			mdelay(1);
			if ((IO_GetInput(FLEX_ID1)) && (!IO_GetInput(FLEX_ID2))) IO_NoHarddiskDetected(); else IO_HarddiskDetected();
		}
	}
	
	if (IO_HaveGlAutoDetect())
	{
		IO_SetInput(GPS_REPRO);
		mdelay(1);
		if (IO_GetInput(GPS_REPRO))
		{
			VALUE_SET(gldetected , 1);
			VALUE_SET(gpstype , GOGPS_GL);
		}
		else
		{
			VALUE_SET(gldetected , 0);
		}
	}

	/* Check GPS type */
	{
#ifdef __KERNEL__
//		Note: Do not use atag_gpstype
//		extern unsigned int atag_gpstype;
//		
//		if ( atag_gpstype!=GOGPS_UNDEFINED && atag_gpstype!=GO_FLASH_SHORT_UNDEFINED )
//		{
//			VALUE_SET(gpstype, atag_gpstype); /* override default */
//		}
		if ( 0 ) { /* test only, use sub id to specify gps type */
			extern unsigned system_rev;
			unsigned gpsId = system_rev >> 16;

			if ( gpsId == GOGPS_ATH_AR1520 )
			{
				VALUE_SET(gpstype, GOGPS_ATH_AR1520); /* override default */
			}
		}
#else
		unsigned short flash_gpstype;	
	
		flash_gpstype = FLASH_GetGPSType();
		
		if (!IO_GpsFlashTypeIllegal()) {
			if ( flash_gpstype!=GO_FLASH_SHORT_UNDEFINED )
			{
				VALUE_SET(gpstype, flash_gpstype); /* override default */
			}
		}
#endif
	}
	if (IO_HasPin(PIC_DETECT))
	{
		int i;
		// Assume PIC is detected
		VALUE_SET(picdetected , 1);
		for (i=0;i<10;i++)
		{
			IO_Deactivate(PIC_DETECT);
			mdelay(1);
			IO_SetInput(PIC_DETECT);
			// Check if PIC is detected
			if (!IO_GetInput(PIC_DETECT)) VALUE_SET(picdetected , 0);
		}
	}
	
	IO_InitSuspend();
	IO_InitDockUART();
	IO_SuspendDetectionPins();
}

int IO_Init(void)
{
	unsigned mainId;
	unsigned subId;

#ifdef __KERNEL__
	extern unsigned system_rev;
	mainId = system_rev & 0x0000ffff;
	subId = system_rev >> 16;
#else
	mainId = FLASH_GetMainId();
	subId = FLASH_GetSubId();
#endif
	
	id = 0;

#ifdef __BOOTLOADER__
	/* get IO out of sleep mode to enable correct ID detection */
	if ( CPU_2443Detected() )
	{
		/* PWROFF_SLP, max. duration */
		rS3C2443_RSTCON = 0x1fefe; /* the value FF is forbidden */
	}
	else if ( CPU_2450Detected() ) {
		rS3C2443_RSTCON = 0x7fefe; /* the value FF is forbidden */
	}
#endif	

	/* Clear structure before init */
	memset(&gotype_current, 0, sizeof gotype_current);

	/* Detect CPU */
	IO_DetectCpu();
	
	/* Detect main type */
	if (mainId)
	{
		/* Get main type from tag */
		id |= mainId;
	}
	else
	{
		/* Get  main type from GPIO */
		IO_DetectMainType();
	}

	/* Init default values for features */
	VALUE_SET(sixbuttonui, 1);

	/* Init fixed members according to main hardware type */
	IO_InitMainType();

	/* Detect sub type */
	if (subId)
	{
		/* Get subtype from tag */
		id |= (subId << 16);
	}
	else
	{
		/* Get sub type from GPIO */
		IO_DetectHardwareSubType();
	}

	/* Init fixed members according to sub hardware type */
	IO_InitSubType();
	
	/* Init devices by detection */
	IO_InitDevices();

	/* Init. features */
	IO_DetectFeature();

	INF("Detected type %d, name %s\n", IO_GetModelId(), IO_GetModelName());
	return 0;
}
EXPORT_SYMBOL(IO_Init);


void IO_InitDockUART(void)
{
	IO_Activate(DOCK_PWREN);
	IO_SetFunction(CTS_DOCK);
	IO_SetFunction(RTS_DOCK);
	IO_SetFunction(TXD_DOCK);
	IO_SetFunction(RXD_DOCK);
}
EXPORT_SYMBOL(IO_InitDockUART);


void IO_ExitDockUART(void)
{
	IO_Suspend(CTS_DOCK);
	IO_Suspend(RTS_DOCK);
	IO_Suspend(TXD_DOCK);
	IO_Suspend(RXD_DOCK);
	IO_Suspend(DOCK_PWREN);
}
EXPORT_SYMBOL(IO_ExitDockUART);


void IOP_DisablePullResistor(unsigned portNr, unsigned pinNr)
{
	if (portNr == PORT_GPIIC)
	{
#ifdef CONFIG_BARCELONA_DOCK
		DOCK_DisablePullResistor( pinNr );
#endif
		return;
	}
	if (s3c2450detected)
	{
		pinNr <<= 1;
		rGP_PULL(portNr) &= ~(0x3 << pinNr);
	}
	else if (s3c2443detected)
	{
//		pinNr <<= 1;
//		rGP_PULL(portNr) |= (1 << pinNr);
		if ((portNr == PORT_GPG) && (pinNr <= 7)) {
			/* need to change EXTINT1 instead */
			/* note: pullup not supported */
			rS3C24XX_EXTINT(1) = s3c2443_read_extint1() | (1 << (3 + (4 * pinNr)));
		} else {
			pinNr <<= 1;
			rGP_PULL(portNr) |= (1 << pinNr);
		}
	}
	else
	{
		/* non-2443 */
		rGP_PULL(portNr) |= (1 << pinNr);
	}
}
EXPORT_SYMBOL(IOP_DisablePullResistor);

void IOP_SetPullResistor(gopin_t pin, unsigned portNr, unsigned pinNr)
{
	if (portNr == PORT_GPIIC)
	{
#ifdef CONFIG_BARCELONA_DOCK
		DOCK_SetPullResistor( pin );
#endif
		return;
	}

	if ((s3c2412detected) || (s3c2442detected))
	{
		if (pin & PIN_PULL_DOWN) 
			rGP_PULL(portNr) &= ~(1 << pinNr);
		else 
			rGP_PULL(portNr) |= (1 << pinNr);
	}
	else if (s3c2450detected)
	{
		pinNr <<= 1;
		if (pin & PIN_PULL_DOWN) /* enable pull-up */
			rGP_PULL(portNr) = (rGP_PULL(portNr) & ~(0x3 << pinNr)) | 0x01 << pinNr;
		else
		if (pin & PIN_PULL_UP) /* enable pull-down */
			rGP_PULL(portNr) = (rGP_PULL(portNr) & ~(0x3 << pinNr)) | 0x02 << pinNr;
		else 
			rGP_PULL(portNr) &= ~(3 << pinNr); /* disable */
	}
	else if (s3c2443detected)
	{
#if 0
		pinNr <<= 1;
		if (pin & PIN_PULL_DOWN) 
			rGP_PULL(portNr) = (rGP_PULL(portNr) | (2 << pinNr)) & ~(1 << pinNr);
		else
		if (pin & PIN_PULL_UP)
			rGP_PULL(portNr) &= ~(3 << pinNr);
		else 
			rGP_PULL(portNr) |= (1 << pinNr);
#endif
		if ((portNr == PORT_GPG) && (pinNr <= 7)) {
			/* need to change EXTINT1 instead */
			if (pin & PIN_PULL_DOWN) {
				rS3C24XX_EXTINT(1) = s3c2443_read_extint1() & ~(1 << (3 + (4 * pinNr)));
			} else {
				/* pullup not supported */
			}
		} else {
			pinNr <<= 1;
			if (pin & PIN_PULL_DOWN) 
				rGP_PULL(portNr) = (rGP_PULL(portNr) | (2 << pinNr)) & ~(1 << pinNr);
			else
			if (pin & PIN_PULL_UP)
				rGP_PULL(portNr) &= ~(3 << pinNr);
			else 
				rGP_PULL(portNr) |= (1 << pinNr);
		}
	}
	else
	{
		if (pin & PIN_PULL_UP) 
			rGP_PULL(portNr) &= ~(1 << pinNr);
		else 
			rGP_PULL(portNr) |= (1 << pinNr);
	}
}
EXPORT_SYMBOL(IOP_SetPullResistor);

inline static void IOP_SetBit(gopin_t pin, unsigned value)
{
	register unsigned pinNr;
	register unsigned portNr;

	portNr = GET_PORTNR(pin);
	pinNr = GET_PINNR(pin);

	if (portNr == PORT_GPIIC)
	{
#ifndef __KERNEL__
		if (value)
			dockOutputData |= (1 << pinNr);
		else
			dockOutputData &= ~(1 << pinNr);
#else
#ifdef CONFIG_BARCELONA_DOCK
		DOCK_SetBit( pin, value );
#endif
#endif
	}
	else
	if (portNr == PORT_GPA)
	{
		if ( !s3c2443detected )
		{
			// Set bit
			if (value)
				rGP_DAT(portNr) |= (1 << pinNr);
			else
				rGP_DAT(portNr) &= ~(1 << pinNr);
			// Set config to output
			rGP_CON(portNr) &= ~(1 << pinNr);
		}
	}
	else
	{
		// Set bit
		if (value)
		{
			rGP_DAT(portNr) |= (1 << pinNr);
			if ( s3c2412detected ) { /* only 2412 has SLPCON */
				rGP_SLPCON(portNr) = (rGP_SLPCON(portNr) & ~(3 << (pinNr*2))) | (1 << (pinNr*2));
			}
		}
		else
		{
			rGP_DAT(portNr) &= ~(1 << pinNr);
			if ( s3c2412detected ) {
				rGP_SLPCON(portNr) = (rGP_SLPCON(portNr) & ~(3 << (pinNr*2))) | (0 << (pinNr*2));
			}
		}
		// Set config to output
		if(s3c2450detected)
		{	
			if(portNr == PORT_GPB)
			{	
		  	if(pinNr==6)
					rGP_SEL(portNr)&=(~(1<<0));
				else if(pinNr==9)
					rGP_SEL(portNr)&=(~(1<<3));			
				else if(pinNr==10)
					rGP_SEL(portNr)&=(~(1<<4));							
			}
			else if(portNr == PORT_GPE)		
			{
				rGP_SEL(portNr)&=(~(1<<pinNr));
			}	
		}
		// Set config to output
		rGP_CON(portNr) = (rGP_CON(portNr) & ~(3 << (pinNr*2))) | (1 << (pinNr*2));
		IOP_DisablePullResistor(portNr,pinNr);		
	}
}


void IO_PowerOff(void)
{
	gopin_t *pin;
	unsigned char used[PIN_AMOUNT];
#ifndef CONFIG_SMDK
	int i;
#endif
	
	memset(used,0,PIN_AMOUNT);

	for (pin = &gotype_current.pins.FIRST; pin <= &gotype_current.pins.LAST; ++pin)
	{
		IOP_Suspend(*pin);
		if (*pin) used[*pin & PIN_MASK] = 1;
	}

#ifndef CONFIG_SMDK
	for (i = PIN_GPB0 ;i <= PIN_GPB15;i++) if (!used[i]) IOP_SetBit(i,IO_GetUnusedPinLevel());
	for (i = PIN_GPC0 ;i <= PIN_GPC15;i++) if (!used[i]) IOP_SetBit(i,IO_GetUnusedPinLevel());
	for (i = PIN_GPD0 ;i <= PIN_GPD15;i++) if (!used[i]) IOP_SetBit(i,IO_GetUnusedPinLevel());
	for (i = PIN_GPE0 ;i <= PIN_GPE13;i++) if (!used[i]) IOP_SetBit(i,IO_GetUnusedPinLevel());
	for (i = PIN_GPE14;i <= PIN_GPE15;i++) if (!used[i]) IOP_SetBit(i,0);
	for (i = PIN_GPF0 ;i <= PIN_GPF15;i++) if (!used[i]) IOP_SetBit(i,IO_GetUnusedPinLevel());
	for (i = PIN_GPG0 ;i <= PIN_GPG15;i++) if (!used[i]) IOP_SetBit(i,IO_GetUnusedPinLevel());
	for (i = PIN_GPH0 ;i <= PIN_GPH15;i++) if (!used[i]) IOP_SetBit(i,IO_GetUnusedPinLevel());

	if (s3c2412detected)
	{
		for (i = PIN_GPJ0_2413;i <= PIN_GPJ15_2413;i++) if (!used[i]) IOP_SetBit(i,IO_GetUnusedPinLevel());
#ifdef SUPPORT_CASABLANCA_PR0
		rS3C2412_MSLCON = (1 << 16) | (1 << 14) | (1 << 12) | (1 << 10) | (1 << 6) | (3 << 0) | (1 << 8) | (1 << 21);
#else
		rS3C2412_MSLCON = (1 << 14) | (1 << 12) | (1 << 10) | (1 << 6) | (3 << 0) | (1 << 8) | (1 << 21);
#endif
	}
	
	if (s3c2443detected) 
	{
		rS3C2443_DATAPDEN  = (0x3 << 4);
	}

	if (s3c2450detected) 
	{
		rS3C2450_PDDMCON = (1 << 22) | (0 << 20) | (0 << 18) | (3 << 16) | (0 << 14) | (1 << 12) | (1 << 10) | (1 << 8) | (1 << 6) | (0 << 4) | (0 << 2) | (0 << 0);
		rS3C2450_PDSMCON = (1 << 22) | (0 << 20) | (1 << 18) | (1 << 16) | (0 << 14) | (1 << 12) | (0 << 10) | (1 << 8) | (0 << 6) | (0 << 4) | (0 << 2) | (0 << 0);
	}
	
	if ( s3c2440detected || s3c2442detected || s3c2443detected  || s3c2450detected )
	{
		for (i = PIN_GPJ0;i <= PIN_GPJ15;i++) if (!used[i]) IOP_SetBit(i,IO_GetUnusedPinLevel());
	}

	if ( s3c2443detected || s3c2450detected )
	{
		for (i = PIN_GPL0;i <= PIN_GPL15;i++) if (!used[i]) IOP_SetBit(i,IO_GetUnusedPinLevel());
	}
#endif
}
EXPORT_SYMBOL(IO_PowerOff);

void IO_Update(void)
{
	if (IO_HaveIoExpander())
	{
		dockInputData = IOEXP_InputOutput(dockOutputData);
	}
}
EXPORT_SYMBOL(IO_Update);

void IOP_GeneratePWM(gopin_t pin)
{
	register int a, b, c;
	unsigned long flags;

	// Don't do anything if pin isn't used
	if (PIN_IS_NOT_USED(pin))
		return;

	local_irq_save(flags);

	if (PIN_IS_INVERTED(pin))
	{
		for (a=0;a<10;a++)
		{
			for (b=0;b<40;b++)
			{
				for (c=a;c<10;c++) IOP_SetBit(pin,1);
				for (c=0;c<a;c++) IOP_SetBit(pin,0);
			}
		}
	}
	else
	{
		for (a=0;a<10;a++)
		{
			for (b=0;b<40;b++)
			{
				for (c=a;c<10;c++) IOP_SetBit(pin,0);
				for (c=0;c<a;c++) IOP_SetBit(pin,1);
			}
		}
	}

	local_irq_restore(flags);
}
EXPORT_SYMBOL(IOP_GeneratePWM);

void IOP_Activate(gopin_t pin)
{
	unsigned long flags;

	// Don't do anything if pin isn't used
	if (PIN_IS_NOT_USED(pin))
		return;

	if (PIN_IS_INVERTED(pin))
	{
		local_irq_save(flags);
		IOP_SetBit(pin, 0);
		local_irq_restore(flags);
	}
	else
	{
		local_irq_save(flags);
		IOP_SetBit(pin, 1);
		local_irq_restore(flags);
	}
}
EXPORT_SYMBOL(IOP_Activate);

void IOP_Deactivate(gopin_t pin)
{
	unsigned long flags;

	// Don't do anything if pin isn't used
	if (PIN_IS_NOT_USED(pin))
		return;

	if (PIN_IS_FLOATING(pin))
	{
		IOP_SetInput(pin);
	}
	else if (PIN_IS_INVERTED(pin))
	{
		local_irq_save(flags);
		IOP_SetBit(pin, 1);
		local_irq_restore(flags);
	}
	else
	{
		local_irq_save(flags);
		IOP_SetBit(pin, 0);
		local_irq_restore(flags);
	}
}
EXPORT_SYMBOL(IOP_Deactivate);

int IOP_IsInverted(gopin_t pin)
{
	return pin & PIN_INVERTED;
}
EXPORT_SYMBOL(IOP_IsInverted);

void IOP_Suspend(gopin_t pin)
{
	if (pin & PIN_IGNORE_ON_SUSPEND) {
		return;
	} else {
		if (pin & PIN_INPUT_ON_SUSPEND)
		{
			IOP_SetInput(pin);
		} else {
	 		if (pin & PIN_ACTIVATE_ON_SUSPEND)
			{
				IOP_Activate(pin);
			}
			else
			{
				IOP_Deactivate(pin);
			}
		}
	}
}
EXPORT_SYMBOL(IOP_Suspend);

void IOP_SetFunc(gopin_t pin, unsigned functionNr)
{
	unsigned long flags;
	unsigned portNr, pinNr;

	portNr = GET_PORTNR(pin);
	pinNr = GET_PINNR(pin);

	local_irq_save(flags);

	if (portNr == PORT_GPA)
	{
		if ( !s3c2443detected )
		{
			// Set config to function
			rGP_CON(portNr) |= (1 << pinNr);
		}
	}
	else
	{ /* 2443, 2450 and others */
		// Set config to function
		if(s3c2450detected)
		{	
			if( portNr == PORT_GPB )
			{	
			  	if ( pinNr == 6 )
						rGP_SEL(portNr)&=(~(1<<0));
					else if(pinNr==9)
						rGP_SEL(portNr)&=(~(1<<3));			
					else if(pinNr==10)
						rGP_SEL(portNr)&=(~(1<<4));							
			}
			else if(portNr == PORT_GPE)		
			{
				rGP_SEL(portNr)&=(~(1<<pinNr));
			}	
		}
		rGP_CON(portNr) = (rGP_CON(portNr) & ~(3 << (pinNr*2))) | ((functionNr ? 3 :2) << (pinNr*2));
		IOP_SetPullResistor(pin,portNr,pinNr);
	}
	local_irq_restore(flags);
}

void IOP_SetFunction(gopin_t pin)
{
	// Don't do anything if pin isn't used
	if (PIN_IS_NOT_USED(pin))
		return;

	if (GET_PORTNR(pin) == PORT_GPIIC) {
		return;
	}
	IOP_SetFunc(pin, 0);
}
EXPORT_SYMBOL(IOP_SetFunction);

void IOP_SetFunction2(gopin_t pin)
{
	// Don't do anything if pin isn't used
	if (PIN_IS_NOT_USED(pin))
		return;

	if (GET_PORTNR(pin) == PORT_GPIIC) {
		return;
	}
	IOP_SetFunc(pin, 1);
}
EXPORT_SYMBOL(IOP_SetFunction2);

void IOP_SetFunction3(gopin_t pin)
{
	unsigned pinNr;
	unsigned portNr;
	unsigned long flags;

	// Don't do anything if pin isn't used
	if (PIN_IS_NOT_USED(pin))
		return;

	portNr = GET_PORTNR(pin);
	pinNr = GET_PINNR(pin);

	if (portNr == PORT_GPIIC)
	{
		return;
	}

	local_irq_save(flags);

	if (portNr == PORT_GPA)
	{
		if (!s3c2443detected)
		{
			// Set config to function
			rGP_CON(portNr) |= (1 << pinNr);
		}
	}
	else
	{
		// Set config to function
		if ( s3c2450detected )
		{	
			if(portNr == PORT_GPB)
			{	
		  	if(pinNr==6)
					rGP_SEL(portNr)|=(1<<0);
				else if(pinNr==9)
					rGP_SEL(portNr)|=(1<<3);			
				else if(pinNr==10)
					rGP_SEL(portNr)|=(1<<4);							
			}
			else if(portNr == PORT_GPE)		
			{
				rGP_SEL(portNr)|=(1<<pinNr);
			}	
		}			
		rGP_CON(portNr) = (rGP_CON(portNr) & ~(3 << (pinNr*2))) | (3 << (pinNr*2));
		IOP_SetPullResistor(pin,portNr,pinNr);
	}

	local_irq_restore(flags);
}
EXPORT_SYMBOL(IOP_SetFunction3);

signed IOP_GetInterruptNumber(gopin_t pin)
{
	unsigned pinNr;
	unsigned portNr;

	// Don't do anything if pin isn't used
	if (PIN_IS_NOT_USED(pin))
		return -1;

	portNr = GET_PORTNR(pin);
	pinNr = GET_PINNR(pin);

	switch (portNr)
	{
	case PORT_GPF:
		return irqList[pinNr];
	case PORT_GPG:
		return irqList[pinNr + 8];
	case PORT_GPIIC:
#ifdef CONFIG_BARCELONA_DOCK
		return DOCK_GetInterruptNumber( pin );
#endif
	default:
		return -1;
	}
}
EXPORT_SYMBOL(IOP_GetInterruptNumber);

static signed IOP_SetInterrupt(gopin_t pin)
{
	unsigned pinNr;
	unsigned portNr;
	unsigned long flags;
	signed intNr;

	intNr = IOP_GetInterruptNumber(pin);
	if (intNr < 0) return -1;

	portNr = GET_PORTNR(pin);
	pinNr = GET_PINNR(pin);

	switch (portNr)
	{
	case PORT_GPF:
		intNr = irqList[pinNr];
		break;
	case PORT_GPG:
		intNr = irqList[pinNr + 8];
		break;
	case PORT_GPIIC:
#ifdef CONFIG_BARCELONA_DOCK
		return DOCK_SetInterrupt( pin );
#endif
	default:
		return -1;
	}

	local_irq_save(flags);

	// Set config to interrupt
	if ( s3c2450detected )
	{	
		if(portNr == PORT_GPB)
		{	
	  	if(pinNr==6)
				rGP_SEL(portNr)&=(~(1<<0));
			else if(pinNr==9)
				rGP_SEL(portNr)&=(~(1<<3));			
			else if(pinNr==10)
				rGP_SEL(portNr)&=(~(1<<4));							
		}
		else if(portNr == PORT_GPE)		
		{
			rGP_SEL(portNr)&=(~(1<<pinNr));
		}	
	}		
	rGP_CON(portNr) = (rGP_CON(portNr) & ~(3 << (pinNr*2))) | (2 << (pinNr*2));
	IOP_SetPullResistor(pin,portNr,pinNr);

	local_irq_restore(flags);

	return intNr;
}

signed IOP_SetInterruptOnActivation(gopin_t pin)
{
	signed intNr;
	unsigned long int flags;

#ifdef CONFIG_BARCELONA_DOCK
	if( GET_PORTNR(pin) == PORT_GPIIC )
		return DOCK_SetInterruptOnActivation( pin );
#endif

	local_irq_save(flags);
	intNr = IOP_SetInterrupt(pin);
	if (intNr >= 0)
	{
		IO_SetInterruptType(intNr, PIN_IS_INVERTED(pin) ? IRQT_FALLING : IRQT_RISING);
		clear_irq_nosync(intNr);
	}
	local_irq_restore(flags);

	return intNr;
}
EXPORT_SYMBOL(IOP_SetInterruptOnActivation);

signed IOP_SetInterruptOnDeactivation(gopin_t pin)
{
	signed intNr;
	unsigned long int flags;

#ifdef CONFIG_BARCELONA_DOCK
	if( GET_PORTNR(pin) == PORT_GPIIC )
		return DOCK_SetInterruptOnDeactivation( pin );
#endif

	local_irq_save(flags);
	intNr = IOP_SetInterrupt(pin);
	if (intNr >= 0)
	{
		IO_SetInterruptType(intNr, PIN_IS_INVERTED(pin) ? IRQT_RISING : IRQT_FALLING);
		clear_irq_nosync(intNr);
	}
	local_irq_restore(flags);

	return intNr;
}
EXPORT_SYMBOL(IOP_SetInterruptOnDeactivation);

signed IOP_SetInterruptOnActivated(gopin_t pin)
{
	signed intNr;
	unsigned long int flags;

	local_irq_save(flags);
	intNr = IOP_SetInterrupt(pin);
	if (intNr >= 0)
	{
		IO_SetInterruptType(intNr, PIN_IS_INVERTED(pin) ? IRQT_LOW : IRQT_HIGH);
		clear_irq_nosync(intNr);
	}
	local_irq_restore(flags);

	return intNr;
}
EXPORT_SYMBOL(IOP_SetInterruptOnActivated);

signed IOP_SetInterruptOnDeactivated(gopin_t pin)
{
	signed intNr;
	unsigned long int flags;

	local_irq_save(flags);
	intNr = IOP_SetInterrupt(pin);
	if (intNr >= 0)
	{
		IO_SetInterruptType(intNr, PIN_IS_INVERTED(pin) ? IRQT_HIGH : IRQT_LOW);
		clear_irq_nosync(intNr);
	}
	local_irq_restore(flags);

	return intNr;
}
EXPORT_SYMBOL(IOP_SetInterruptOnDeactivated);

signed IOP_SetInterruptOnToggle(gopin_t pin)
{
	signed intNr;
	unsigned long int flags;

#ifdef CONFIG_BARCELONA_DOCK
	if( GET_PORTNR(pin) == PORT_GPIIC )
		return DOCK_SetInterruptOnToggle( pin );
#endif

	local_irq_save(flags);
	intNr = IOP_SetInterrupt(pin);
	if (intNr >= 0)
	{
		IO_SetInterruptType(intNr, IRQT_BOTHEDGE);
		clear_irq_nosync(intNr);
	}
	local_irq_restore(flags);

	return intNr;
}
EXPORT_SYMBOL(IOP_SetInterruptOnToggle);

void IOP_SetInput(gopin_t pin)
{
	unsigned pinNr = pin & PIN_MASK;
	unsigned portNr;
	unsigned long flags;

	// Don't do anything if pin isn't used
	if (PIN_IS_NOT_USED(pin))
		return;

	portNr = GET_PORTNR(pin);
	pinNr = GET_PINNR(pin);

	local_irq_save(flags);
	if (portNr == PORT_GPIIC)
	{
#ifdef CONFIG_BARCELONA_DOCK
		DOCK_SetInput( pin );
#endif
		local_irq_restore(flags);
		return;
	}

	if (portNr == PORT_GPA)
	{
		local_irq_restore(flags);
		return;
	}
	else
	{
		// Set config to input
		if ( s3c2450detected )
		{	
			if(portNr == PORT_GPB)
			{	
		  	if(pinNr==6)
					rGP_SEL(portNr)&=(~(1<<0));
				else if(pinNr==9)
					rGP_SEL(portNr)&=(~(1<<3));			
				else if(pinNr==10)
					rGP_SEL(portNr)&=(~(1<<4));							
			}
			else if(portNr == PORT_GPE)		
			{
				rGP_SEL(portNr)&=(~(1<<pinNr));
			}	
		}			
		rGP_CON(portNr) = (rGP_CON(portNr) & ~(3 << (pinNr*2))) | (0 << (pinNr*2));
		IOP_SetPullResistor(pin,portNr,pinNr);
		// Enable pullup or pull down for input pin
		if ( s3c2412detected ) { /* only 2412 has rGP_SLPCON */
			if (pin & PIN_PULL_DOWN)
				rGP_SLPCON(portNr) = (rGP_SLPCON(portNr) & ~(3 << (pinNr*2))) | (3 << (pinNr*2));
			else
				rGP_SLPCON(portNr) = (rGP_SLPCON(portNr) & ~(3 << (pinNr*2))) | (2 << (pinNr*2));
		}
	}

	local_irq_restore(flags);
}
EXPORT_SYMBOL(IOP_SetInput);

int IOP_GetInput(gopin_t pin)
{
	unsigned pinNr;
	unsigned portNr;

	// Don't do anything if pin isn't used
	if (PIN_IS_NOT_USED(pin))
		return 0;

	portNr = GET_PORTNR(pin);
	pinNr = GET_PINNR(pin);

	if (portNr == PORT_GPIIC)
	{
#ifndef __KERNEL__
		if (dockInputData & (1 << pinNr))
		{
			return PIN_IS_INVERTED(pin) ? 0 : 1;
		}
		else
		{
			return PIN_IS_INVERTED(pin) ? 1 : 0;
		}
#else
#ifdef CONFIG_BARCELONA_DOCK
		return DOCK_GetInput( pin );
#endif
#endif
	}
	else
	if (rGP_DAT(portNr) & (1 << pinNr))
	{
		return PIN_IS_INVERTED(pin) ? 0 : 1;
	}
	else
	{
		return PIN_IS_INVERTED(pin) ? 1 : 0;
	}
}
EXPORT_SYMBOL(IOP_GetInput);


int IO_GetDockState(void)
{
#ifdef CONFIG_BARCELONA_DOCK
	/* One day, everything should go in the function below I guess. */
	int retval;
	retval = DOCK_GetDockState();
	if (retval != -1)
		return retval;
#endif
	if (IO_GetInput(DOCK_RADIO_SENSE)) return GODOCK_RADIO;
	if (IO_GetInput(DOCK_MOTOR_SENSE)) return GODOCK_MOTOR;
	if (IO_GetInput(DOCK_VIB_SENSE)) return GODOCK_VIB;
	if (IO_GetInput(DOCK_CRIB_SENSE)) return GODOCK_CRIB;
	if (IO_GetInput(DOCK_SENSE)) return GODOCK_WINDSCR_WO_FM;
	if (IO_GetInput(DOCK_DESK_SENSE)) return GODOCK_DESK;
	return GODOCK_NONE;
}
EXPORT_SYMBOL(IO_GetDockState);

int IO_CarDocked(void)
{
	switch (IO_GetDockState())
	{
		case GODOCK_VIB:
		case GODOCK_CRIB:
		case GODOCK_WINDSCREEN:
		case GODOCK_MOTOR:
		case GODOCK_RADIO:
			return 1;
		default:
			return 0;
	}
}
EXPORT_SYMBOL(IO_CarDocked);

/**
 * Return the Chip Select (NGCS) number that this pin maps to.
 */ 
signed IOP_GetGCSNumber(gopin_t pin)
{
	pin &= PIN_MASK; /* filter out info about inverted lines etc	*/
	// Don't do anything if pin isn't used
	if (PIN_IS_NOT_USED(pin))
		return -1;

	switch( pin )
	{
		case PIN_GPA12:
			return 1;
		case PIN_GPA13:
			return 2;
		case PIN_GPA14:
			return 3;
		case PIN_GPA15:
			return 4;
		case PIN_GPA16:
			return 5;
		/* The other Chip Selects are not attached to GPx pins	*/
		default:
			return -1;
	}
}
EXPORT_SYMBOL(IOP_GetGCSNumber);

signed IO_GetClkOut(gopin_t pin)	
{	
	gopin_t clkout0pin = PIN_GPH9;
	if( s3c2443detected || s3c2450detected ) {
		clkout0pin = PIN_GPH13;
	}

	/* CLKOUT1 is returned if pin is not connected.	*/	
	if( (pin & PIN_MASK) == clkout0pin )
		return 0;
	else
		return 1;
}
EXPORT_SYMBOL(IO_GetClkOut);

/* EOF */

