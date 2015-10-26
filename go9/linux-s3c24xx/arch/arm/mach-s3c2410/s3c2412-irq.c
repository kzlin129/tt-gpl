/* linux/arch/arm/mach-s3c2410/s3c2412-irq.c
 *
 * Copyright (c) 2006 TomTom BV
 *	Thomas Kleffel <tk@maintech.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
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

/* SDI/CF */

#define SUBSHF_SDI_CF 13
#define SUBMSK_SDI_CF 3

static void
s3c_irq_demux_sdicf(unsigned int irq,
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
	subsrc >>= SUBSHF_SDI_CF;
	subsrc &= SUBMSK_SDI_CF;

	if (subsrc == 0)
		return;

	if (subsrc & 1) {
		mydesc = irq_desc + IRQ_S3C2412_SDI;
		mydesc->handle(IRQ_S3C2412_SDI, mydesc, regs);
	}
	if (subsrc & 2) {
		mydesc = irq_desc + IRQ_S3C2412_CF;
		mydesc->handle(IRQ_S3C2412_CF, mydesc, regs);
	}
}


#define INTMSK_SDI_CF (1UL << (IRQ_SDI_CF - IRQ_EINT0))

static void
s3c_irq_sdicf_mask(unsigned int irqno)
{
	s3c_irqsub_mask(irqno, INTMSK_SDI_CF, SUBMSK_SDI_CF << SUBSHF_SDI_CF);
}

static void
s3c_irq_sdicf_unmask(unsigned int irqno)
{
	s3c_irqsub_unmask(irqno, INTMSK_SDI_CF);
}

static void
s3c_irq_sdicf_ack(unsigned int irqno)
{
	s3c_irqsub_maskack(irqno, INTMSK_SDI_CF, SUBMSK_SDI_CF << SUBSHF_SDI_CF);
}

static struct irqchip s3c_irq_sdicf = {
	.mask	    = s3c_irq_sdicf_mask,
	.unmask	    = s3c_irq_sdicf_unmask,
	.ack	    = s3c_irq_sdicf_ack,
};

/* SPI */

#define SUBSHF_SPI1_SPITO 11
#define SUBMSK_SPI1_SPITO 0x19	/* 11001 */

static void
s3c_irq_demux_spi(unsigned int irq,
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
	subsrc >>= SUBSHF_SPI1_SPITO;
	subsrc &= SUBMSK_SPI1_SPITO;

	if (subsrc == 0)
		return;

	if (subsrc & 1) {
		mydesc = irq_desc + IRQ_S3C2412_SPI0_TO;
		mydesc->handle(IRQ_S3C2412_SPI0_TO, mydesc, regs);
	}
	if (subsrc & 2) {
		mydesc = irq_desc + IRQ_S3C2412_SPI1_TO;
		mydesc->handle(IRQ_S3C2412_SPI1_TO, mydesc, regs);
	}
	if (subsrc & 16) {
		mydesc = irq_desc + IRQ_S3C2412_SPI1;
		mydesc->handle(IRQ_S3C2412_SPI1, mydesc, regs);
	}
}


#define INTMSK_SPI1_SPITO (1UL << (IRQ_SPI1_SPITO - IRQ_EINT0))

static void
s3c_irq_spi_mask(unsigned int irqno)
{
	s3c_irqsub_mask(irqno, INTMSK_SPI1_SPITO, SUBMSK_SPI1_SPITO << SUBSHF_SPI1_SPITO);
}

static void
s3c_irq_spi_unmask(unsigned int irqno)
{
	s3c_irqsub_unmask(irqno, INTMSK_SPI1_SPITO);
}

static void
s3c_irq_spi_ack(unsigned int irqno)
{
	s3c_irqsub_maskack(irqno, INTMSK_SPI1_SPITO, SUBMSK_SPI1_SPITO << SUBSHF_SPI1_SPITO);
}

static struct irqchip s3c_irq_spi = {
	.mask	    = s3c_irq_spi_mask,
	.unmask	    = s3c_irq_spi_unmask,
	.ack	    = s3c_irq_spi_ack,
};

static int s3c2412_irq_add(struct sys_device *sysdev)
{
	unsigned int irqno;

	printk("S3C2412: IRQ Support\n");

	set_irq_chip(IRQ_NFCON, &s3c_irq_level_chip);
	set_irq_handler(IRQ_NFCON, do_level_IRQ);
	set_irq_flags(IRQ_NFCON, IRQF_VALID);

	set_irq_chip(IRQ_CAM, &s3c_irq_level_chip);
	set_irq_handler(IRQ_CAM, do_level_IRQ);
	set_irq_flags(IRQ_CAM, IRQF_VALID);
	
	/* add new chained handler for sdi/cf */

	set_irq_chip(IRQ_SDI, &s3c_irq_level_chip);
	set_irq_handler(IRQ_SDI, do_level_IRQ);
	set_irq_chained_handler(IRQ_SDI, s3c_irq_demux_sdicf);

	for (irqno = IRQ_S3C2412_SDI; irqno <= IRQ_S3C2412_CF; irqno++) {
		set_irq_chip(irqno, &s3c_irq_sdicf);
		set_irq_handler(irqno, do_level_IRQ);
		set_irq_flags(irqno, IRQF_VALID);
	}

	/* add new chained handler for spi */

	/* add new chained handler for spi */

	set_irq_chip(IRQ_SPI1, &s3c_irq_level_chip);
	set_irq_handler(IRQ_SPI1, do_level_IRQ);
	set_irq_chained_handler(IRQ_SPI1, s3c_irq_demux_spi);

	set_irq_chip(IRQ_S3C2412_SPI0_TO, &s3c_irq_spi);
	set_irq_handler(IRQ_S3C2412_SPI0_TO, do_level_IRQ);
	set_irq_flags(IRQ_S3C2412_SPI0_TO, IRQF_VALID);

	set_irq_chip(IRQ_S3C2412_SPI1_TO, &s3c_irq_spi);
	set_irq_handler(IRQ_S3C2412_SPI1_TO, do_level_IRQ);
	set_irq_flags(IRQ_S3C2412_SPI1_TO, IRQF_VALID);

	set_irq_chip(IRQ_S3C2412_SPI1, &s3c_irq_spi);
	set_irq_handler(IRQ_S3C2412_SPI1, do_level_IRQ);
	set_irq_flags(IRQ_S3C2412_SPI1, IRQF_VALID);

	return 0;
}

static struct sysdev_driver s3c2412_irq_driver = {
	.add	= s3c2412_irq_add,
};

static int s3c24xx_irq_driver(void)
{
	return sysdev_driver_register(&s3c2412_sysclass, &s3c2412_irq_driver);
}

arch_initcall(s3c24xx_irq_driver);

