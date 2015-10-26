/* linux/arch/arm/mach-s3c2410/s3c2443-irq.c
 *
 * Copyright (C) 2006 TomTom BV <http://www.tomtom.com/>
 * Author 0: Dimitry Andric <dimitry.andric@tomtom.com>
 *
 * S3C2443 IRQ support.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ptrace.h>
#include <linux/sysdev.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <asm/mach/irq.h>

#include <asm/arch/regs-irq.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/pm.h>

#include "cpu.h"
#include "irq.h"

/* WDT/AC97 */

#define SUBSHF_WDT_AC97 27
#define SUBMSK_WDT_AC97 3

static void
s3c_irq_demux_wdtac97(unsigned int irq,
		      struct irqdesc *desc,
		      struct pt_regs *regs)
{
	unsigned int subsrc, submsk;
	struct irqdesc *mydesc;

	/* read the current pending interrupts, and the mask
	 * for what it is available */

	subsrc = __raw_readl(S3C2410_SUBSRCPND);
	submsk = __raw_readl(S3C2410_INTSUBMSK);

	subsrc &= ~submsk;
	subsrc >>= SUBSHF_WDT_AC97;
	subsrc &= SUBMSK_WDT_AC97;

	if (subsrc == 0)
		return;

	if (subsrc & 1) {
		mydesc = irq_desc + IRQ_S3C2443_WDT;
		mydesc->handle(IRQ_S3C2443_WDT, mydesc, regs);
	}
	if (subsrc & 2) {
		mydesc = irq_desc + IRQ_S3C2443_AC97;
		mydesc->handle(IRQ_S3C2443_AC97, mydesc, regs);
	}
}


#define INTMSK_WDT_AC97 (1UL << (IRQ_WDT_AC97 - IRQ_EINT0))

static void
s3c_irq_wdtac97_mask(unsigned int irqno)
{
	s3c_irqsub_mask(irqno, INTMSK_WDT_AC97, SUBMSK_WDT_AC97 << SUBSHF_WDT_AC97);
}

static void
s3c_irq_wdtac97_unmask(unsigned int irqno)
{
	s3c_irqsub_unmask(irqno, INTMSK_WDT_AC97);
}

static void
s3c_irq_wdtac97_ack(unsigned int irqno)
{
	s3c_irqsub_maskack(irqno, INTMSK_WDT_AC97, SUBMSK_WDT_AC97 << SUBSHF_WDT_AC97);
}

static struct irqchip s3c_irq_wdtac97 = {
	.mask	    = s3c_irq_wdtac97_mask,
	.unmask	    = s3c_irq_wdtac97_unmask,
	.ack	    = s3c_irq_wdtac97_ack,
};

/* camera irq */

#define SUBSHF_CAM 11
#define SUBMSK_CAM 3

static void
s3c_irq_demux_cam(unsigned int irq,
		  struct irqdesc *desc,
		  struct pt_regs *regs)
{
	unsigned int subsrc, submsk;
	struct irqdesc *mydesc;

	/* read the current pending interrupts, and the mask
	 * for what it is available */

	subsrc = __raw_readl(S3C2410_SUBSRCPND);
	submsk = __raw_readl(S3C2410_INTSUBMSK);

	subsrc &= ~submsk;
	subsrc >>= SUBSHF_CAM;
	subsrc &= SUBMSK_CAM;

	if (subsrc == 0)
		return;

	if (subsrc & 1) {
		mydesc = irq_desc + IRQ_S3C2443_CAM_C;
		mydesc->handle(IRQ_S3C2443_CAM_C, mydesc, regs);
	}
	if (subsrc & 2) {
		mydesc = irq_desc + IRQ_S3C2443_CAM_P;
		mydesc->handle(IRQ_S3C2443_CAM_P, mydesc, regs);
	}
}


#define INTMSK_CAM (1UL << (IRQ_CAM - IRQ_EINT0))

static void
s3c_irq_cam_mask(unsigned int irqno)
{
	s3c_irqsub_mask(irqno, INTMSK_CAM, SUBMSK_CAM << SUBSHF_CAM);
}

static void
s3c_irq_cam_unmask(unsigned int irqno)
{
	s3c_irqsub_unmask(irqno, INTMSK_CAM);
}

static void
s3c_irq_cam_ack(unsigned int irqno)
{
	s3c_irqsub_maskack(irqno, INTMSK_CAM, SUBMSK_CAM << SUBSHF_CAM);
}

static struct irqchip s3c_irq_cam = {
	.mask	    = s3c_irq_cam_mask,
	.unmask	    = s3c_irq_cam_unmask,
	.ack	    = s3c_irq_cam_ack,
};

/* dma irq */

#define SUBSHF_DMA 18
#define SUBMSK_DMA 0x3f

static void
s3c_irq_demux_dma(unsigned int irq,
		  struct irqdesc *desc,
		  struct pt_regs *regs)
{
	unsigned int subsrc, submsk;
	struct irqdesc *mydesc;

	/* read the current pending interrupts, and the mask
	 * for what it is available */

	subsrc = __raw_readl(S3C2410_SUBSRCPND);
	submsk = __raw_readl(S3C2410_INTSUBMSK);

	subsrc &= ~submsk;
	subsrc >>= SUBSHF_DMA;
	subsrc &= SUBMSK_DMA;

	if (subsrc == 0)
		return;

	if (subsrc & 1) {
		mydesc = irq_desc + IRQ_S3C2443_DMA0;
		mydesc->handle(IRQ_S3C2443_DMA0, mydesc, regs);
	}
	if (subsrc & 2) {
		mydesc = irq_desc + IRQ_S3C2443_DMA1;
		mydesc->handle(IRQ_S3C2443_DMA1, mydesc, regs);
	}
	if (subsrc & 4) {
		mydesc = irq_desc + IRQ_S3C2443_DMA2;
		mydesc->handle(IRQ_S3C2443_DMA2, mydesc, regs);
	}
	if (subsrc & 8) {
		mydesc = irq_desc + IRQ_S3C2443_DMA3;
		mydesc->handle(IRQ_S3C2443_DMA3, mydesc, regs);
	}
	if (subsrc & 16) {
		mydesc = irq_desc + IRQ_S3C2443_DMA4;
		mydesc->handle(IRQ_S3C2443_DMA4, mydesc, regs);
	}
	if (subsrc & 32) {
		mydesc = irq_desc + IRQ_S3C2443_DMA5;
		mydesc->handle(IRQ_S3C2443_DMA5, mydesc, regs);
	}
}

#define INTMSK_DMAPARENT (1UL << (IRQ_DMAPARENT - IRQ_EINT0))

static void
s3c_irq_dma_mask(unsigned int irqno)
{
	s3c_irqsub_mask(irqno, INTMSK_DMAPARENT, SUBMSK_DMA << SUBSHF_DMA);
}

static void
s3c_irq_dma_unmask(unsigned int irqno)
{
	s3c_irqsub_unmask(irqno, INTMSK_DMAPARENT);
}

static void
s3c_irq_dma_ack(unsigned int irqno)
{
	s3c_irqsub_maskack(irqno, INTMSK_DMAPARENT, SUBMSK_DMA << SUBSHF_DMA);
}

static struct irqchip s3c_irq_dma = {
	.mask	    = s3c_irq_dma_mask,
	.unmask	    = s3c_irq_dma_unmask,
	.ack	    = s3c_irq_dma_ack,
};

/* lcd irq */

#define SUBSHF_LCD 14
#define SUBMSK_LCD 0xf

static void
s3c_irq_demux_lcd(unsigned int irq,
		  struct irqdesc *desc,
		  struct pt_regs *regs)
{
	unsigned int subsrc, submsk;
	struct irqdesc *mydesc;

	/* read the current pending interrupts, and the mask
	 * for what it is available */

	subsrc = __raw_readl(S3C2410_SUBSRCPND);
	submsk = __raw_readl(S3C2410_INTSUBMSK);

	subsrc &= ~submsk;
	subsrc >>= SUBSHF_LCD;
	subsrc &= SUBMSK_LCD;

	if (subsrc == 0)
		return;

	if (subsrc & 1) {
		mydesc = irq_desc + IRQ_S3C2443_LCD1;
		mydesc->handle(IRQ_S3C2443_LCD1, mydesc, regs);
	}
	if (subsrc & 2) {
		mydesc = irq_desc + IRQ_S3C2443_LCD2;
		mydesc->handle(IRQ_S3C2443_LCD2, mydesc, regs);
	}
	if (subsrc & 4) {
		mydesc = irq_desc + IRQ_S3C2443_LCD3;
		mydesc->handle(IRQ_S3C2443_LCD3, mydesc, regs);
	}
	if (subsrc & 8) {
		mydesc = irq_desc + IRQ_S3C2443_LCD4;
		mydesc->handle(IRQ_S3C2443_LCD4, mydesc, regs);
	}
}


#define INTMSK_LCDPARENT (1UL << (IRQ_LCDPARENT - IRQ_EINT0))

static void
s3c_irq_lcd_mask(unsigned int irqno)
{
	s3c_irqsub_mask(irqno, INTMSK_LCDPARENT, SUBMSK_LCD << SUBSHF_LCD);
}

static void
s3c_irq_lcd_unmask(unsigned int irqno)
{
	s3c_irqsub_unmask(irqno, INTMSK_LCDPARENT);
}

static void
s3c_irq_lcd_ack(unsigned int irqno)
{
	s3c_irqsub_maskack(irqno, INTMSK_LCDPARENT, SUBMSK_LCD << SUBSHF_LCD);
}

static struct irqchip s3c_irq_lcd = {
	.mask	    = s3c_irq_lcd_mask,
	.unmask	    = s3c_irq_lcd_unmask,
	.ack	    = s3c_irq_lcd_ack,
};

/* UART 3 */

#define SUBSHF_UART3 24
#define SUBMSK_UART3 7

static void
s3c_irq_demux_uart3(unsigned int irq,
		    struct irqdesc *desc,
		    struct pt_regs *regs)
{
	unsigned int subsrc, submsk;
	struct irqdesc *mydesc;

	/* read the current pending interrupts, and the mask
	 * for what it is available */

	subsrc = __raw_readl(S3C2410_SUBSRCPND);
	submsk = __raw_readl(S3C2410_INTSUBMSK);

	subsrc &= ~submsk;
	subsrc >>= SUBSHF_UART3;
	subsrc &= SUBMSK_UART3;

	if (subsrc == 0)
		return;

	if (subsrc & 1) {
		mydesc = irq_desc + IRQ_S3C2443_RX3;
		mydesc->handle(IRQ_S3C2443_RX3, mydesc, regs);
	}
	if (subsrc & 2) {
		mydesc = irq_desc + IRQ_S3C2443_TX3;
		mydesc->handle(IRQ_S3C2443_TX3, mydesc, regs);
	}
	if (subsrc & 4) {
		mydesc = irq_desc + IRQ_S3C2443_ERR3;
		mydesc->handle(IRQ_S3C2443_ERR3, mydesc, regs);
	}
}


#define INTMSK_UART3 (1UL << (IRQ_UART3 - IRQ_EINT0))

static void
s3c_irq_uart3_mask(unsigned int irqno)
{
	s3c_irqsub_mask(irqno, INTMSK_UART3, SUBMSK_UART3 << SUBSHF_UART3);
}

static void
s3c_irq_uart3_unmask(unsigned int irqno)
{
	s3c_irqsub_unmask(irqno, INTMSK_UART3);
}

static void
s3c_irq_uart3_ack(unsigned int irqno)
{
	s3c_irqsub_maskack(irqno, INTMSK_UART3, SUBMSK_UART3 << SUBSHF_UART3);
}

static struct irqchip s3c_irq_uart3 = {
	.mask	    = s3c_irq_uart3_mask,
	.unmask	    = s3c_irq_uart3_unmask,
	.ack	    = s3c_irq_uart3_ack,
};

static int s3c2443_irq_add(struct sys_device *sysdev)
{
	unsigned int irqno;

	printk("S3C2443: IRQ Support\n");

	set_irq_chip(IRQ_NFCON, &s3c_irq_level_chip);
	set_irq_handler(IRQ_NFCON, do_level_IRQ);
	set_irq_flags(IRQ_NFCON, IRQF_VALID);

	set_irq_chip(IRQ_CAM, &s3c_irq_level_chip);
	set_irq_handler(IRQ_CAM, do_level_IRQ);
	set_irq_flags(IRQ_CAM, IRQF_VALID);

	/* add new chained handler for wdt, ac7 */

	set_irq_chip(IRQ_WDT, &s3c_irq_level_chip);
	set_irq_handler(IRQ_WDT, do_level_IRQ);
	set_irq_chained_handler(IRQ_WDT, s3c_irq_demux_wdtac97);

	for (irqno = IRQ_S3C2443_WDT; irqno <= IRQ_S3C2443_AC97; irqno++) {
		set_irq_chip(irqno, &s3c_irq_wdtac97);
		set_irq_handler(irqno, do_level_IRQ);
		set_irq_flags(irqno, IRQF_VALID);
	}

	/* add chained handler for camera */

	set_irq_chip(IRQ_CAM, &s3c_irq_level_chip);
	set_irq_handler(IRQ_CAM, do_level_IRQ);
	set_irq_chained_handler(IRQ_CAM, s3c_irq_demux_cam);

	for (irqno = IRQ_S3C2443_CAM_C; irqno <= IRQ_S3C2443_CAM_P; irqno++) {
		set_irq_chip(irqno, &s3c_irq_cam);
		set_irq_handler(irqno, do_level_IRQ);
		set_irq_flags(irqno, IRQF_VALID);
	}

	/* add chained handler for dma */

	set_irq_chip(IRQ_DMAPARENT, &s3c_irq_level_chip);
	set_irq_handler(IRQ_DMAPARENT, do_level_IRQ);
	set_irq_chained_handler(IRQ_DMAPARENT, &s3c_irq_demux_dma);

	for (irqno = IRQ_S3C2443_DMA0; irqno <= IRQ_S3C2443_DMA5; irqno++) {
		set_irq_chip(irqno, &s3c_irq_dma);
		set_irq_handler(irqno, do_level_IRQ);
		set_irq_flags(irqno, IRQF_VALID);
	}

	/* add chained handler for lcd */

	set_irq_chip(IRQ_LCDPARENT, &s3c_irq_level_chip);
	set_irq_handler(IRQ_LCDPARENT, do_level_IRQ);
	set_irq_chained_handler(IRQ_LCDPARENT, &s3c_irq_demux_lcd);

	for (irqno = IRQ_S3C2443_LCD1; irqno <= IRQ_S3C2443_LCD4; irqno++) {
		set_irq_chip(irqno, &s3c_irq_lcd);
		set_irq_handler(irqno, do_level_IRQ);
		set_irq_flags(irqno, IRQF_VALID);
	}

	/* add chained handler for uart3 */

	set_irq_chip(IRQ_UART3, &s3c_irq_level_chip);
	set_irq_handler(IRQ_UART3, do_level_IRQ);
	set_irq_chained_handler(IRQ_UART3, &s3c_irq_demux_uart3);

	for (irqno = IRQ_S3C2443_RX3; irqno <= IRQ_S3C2443_ERR3; irqno++) {
		set_irq_chip(irqno, &s3c_irq_uart3);
		set_irq_handler(irqno, do_level_IRQ);
		set_irq_flags(irqno, IRQF_VALID);
	}

	return 0;
}

static struct sysdev_driver s3c2443_irq_driver = {
	.add	= s3c2443_irq_add,
};

static int s3c24xx_irq_driver(void)
{
	return sysdev_driver_register(&s3c2443_sysclass, &s3c2443_irq_driver);
}

arch_initcall(s3c24xx_irq_driver);

