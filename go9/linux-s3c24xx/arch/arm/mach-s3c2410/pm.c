/* linux/arch/arm/mach-s3c2410/pm.c
 *
 * Copyright (c) 2004 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C2410 Power Manager (Suspend-To-RAM) support
 *
 * See Documentation/arm/Samsung-S3C24XX/Suspend.txt for more information
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
 * Parts based on arch/arm/mach-pxa/pm.c
 *
 * Thanks to Dimitry Andric for debugging
 *
 * Modifications:
 *     10-Mar-2005 LCVR  Changed S3C2410_VA_UART to S3C24XX_VA_UART
*/

#include <linux/config.h>
#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/crc32.h>
#include <linux/ioport.h>
#include <linux/delay.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/cacheflush.h>

#include <asm/arch/regs-serial.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-gpio.h>
#ifdef CONFIG_CPU_S3C2440
#include <asm/arch/regs-gpioj.h>
#endif /* CONFIG_CPU_S3C2440 */
#if defined(CONFIG_CPU_S3C2443) || defined(CONFIG_CPU_S3C2450)
#include <asm/arch/regs-hsudc.h>
#endif /* CONFIG_CPU_S3C2443 || CONFIG_CPU_S3C2450 */
#include <asm/arch/regs-mem.h>
#include <asm/arch/regs-irq.h>
#include <asm/arch/regs-dyn.h>

#include <asm/mach/time.h>
#include <asm/arch/pm.h>

#include <asm/arch/tomtomgo-wake.h>
#include <barcelona/gopins.h>

/* for external use */

unsigned long s3c_pm_flags;

#define PFX "s3c24xx-pm: "

static struct sleep_save s3c2410_core_save[] = {
	SAVE_ITEM(S3C2410_LOCKTIME),
	SAVE_ITEM(S3C2410_CLKCON),

	/* we restore the timings here, with the proviso that the board
	 * brings the system up in an slower, or equal frequency setting
	 * to the original system.
	 *
	 * if we cannot guarantee this, then things are going to go very
	 * wrong here, as we modify the refresh and both pll settings.
	 */

	SAVE_ITEM(S3C2410_BWSCON),
	SAVE_ITEM(S3C2410_BANKCON0),
	SAVE_ITEM(S3C2410_BANKCON1),
	SAVE_ITEM(S3C2410_BANKCON2),
	SAVE_ITEM(S3C2410_BANKCON3),
	SAVE_ITEM(S3C2410_BANKCON4),
	SAVE_ITEM(S3C2410_BANKCON5),

	SAVE_ITEM(S3C2410_CLKDIVN),
	SAVE_ITEM(S3C2410_MPLLCON),
	SAVE_ITEM(S3C2410_UPLLCON),
	SAVE_ITEM(S3C2410_CLKSLOW),
	SAVE_ITEM(S3C2410_REFRESH),

#ifdef CONFIG_CPU_S3C2440
	SAVE_ITEM(S3C2440_CAMDIVN),
#endif /* CONFIG_CPU_S3C2440 */
};

#ifdef CONFIG_CPU_S3C2412
static struct sleep_save s3c2412_core_save[] = {
	SAVE_ITEM(S3C2410_LOCKTIME),
	SAVE_ITEM(S3C2410_CLKCON),

	SAVE_ITEM(S3C2410_CLKDIVN),
	SAVE_ITEM(S3C2410_MPLLCON),
	SAVE_ITEM(S3C2410_UPLLCON),
	SAVE_ITEM(S3C2412_CLKSRC),

	SAVE_ITEM(S3C2440_CAMDIVN),
};
#endif /* CONFIG_CPU_S3C2412 */


#if defined(CONFIG_CPU_S3C2443) || defined(CONFIG_CPU_S3C2450)
static struct sleep_save s3c2443_core_save[] = {
	SAVE_ITEM(S3C2443_LOCKCON0),
	SAVE_ITEM(S3C2443_LOCKCON1),
	SAVE_ITEM(S3C2443_OSCSET),
	SAVE_ITEM(S3C2443_MPLLCON),
	SAVE_ITEM(S3C2443_EPLLCON),

	SAVE_ITEM(S3C2443_HCLKCON),
	SAVE_ITEM(S3C2443_SCLKCON),
	SAVE_ITEM(S3C2443_PCLKCON),

	SAVE_ITEM(S3C2443_CLKDIV0),
	SAVE_ITEM(S3C2443_CLKDIV1),

	SAVE_ITEM(S3C2443_CLKSRC),

	SAVE_ITEM(S3C2443_PHYCTRL),
	SAVE_ITEM(S3C2443_PHYPWR),
	SAVE_ITEM(S3C2443_URSTCON),
	SAVE_ITEM(S3C2443_UCLKCON),
	SAVE_ITEM(S3C2443_PWRCFG),
	SAVE_ITEM(S3C_HSUDC_EP_INT_EN_REG),
	SAVE_ITEM(S3C_HSUDC_TEST_REG),
	SAVE_ITEM(S3C_HSUDC_SYS_CON_REG),
	SAVE_ITEM(S3C_HSUDC_EP0_CON_REG),
};
#endif /* CONFIG_CPU_S3C2443 || defined(CONFIG_CPU_S3C2450) */


/* this lot should be really saved by the IRQ code */
static struct sleep_save s3c2410_irq_save[] = {
	SAVE_ITEM(S3C2410_EXTINT0),
	SAVE_ITEM(S3C2410_EXTINT1),
	SAVE_ITEM(S3C2410_EXTINT2),
	SAVE_ITEM(S3C2410_EINFLT0),
	SAVE_ITEM(S3C2410_EINFLT1),
	SAVE_ITEM(S3C2410_EINFLT2),
	SAVE_ITEM(S3C2410_EINFLT3),
	SAVE_ITEM(S3C2410_EINTMASK),
	SAVE_ITEM(S3C2410_INTMSK),
	SAVE_ITEM(S3C2410_PRIORITY),
	SAVE_ITEM(S3C2410_INTSUBMSK ),
	SAVE_ITEM(S3C2410_INTMOD),
};

#ifdef CONFIG_CPU_S3C2412
static struct sleep_save s3c2412_irq_save[] = {
	SAVE_ITEM(S3C2412_EXTINT0),
	SAVE_ITEM(S3C2412_EXTINT1),
	SAVE_ITEM(S3C2412_EXTINT2),
	SAVE_ITEM(S3C2412_EINFLT0),
	SAVE_ITEM(S3C2412_EINFLT1),
	SAVE_ITEM(S3C2412_EINFLT2),
	SAVE_ITEM(S3C2412_EINFLT3),
	SAVE_ITEM(S3C2412_EINTMASK),
	SAVE_ITEM(S3C2410_INTMSK),
	SAVE_ITEM(S3C2410_PRIORITY),
	SAVE_ITEM(S3C2410_INTSUBMSK ),
	SAVE_ITEM(S3C2410_INTMOD),
};
#endif /* CONFIG_CPU_S3C2412 */

static struct sleep_save s3c2410_gpio_save[] = {
	SAVE_ITEM(S3C2410_GPADAT),
	SAVE_ITEM(S3C2410_GPACON),

	SAVE_ITEM(S3C2410_GPBDAT),
	SAVE_ITEM(S3C2410_GPBCON),
	SAVE_ITEM(S3C2410_GPBUP),

	SAVE_ITEM(S3C2410_GPCDAT),
	SAVE_ITEM(S3C2410_GPCCON),
	SAVE_ITEM(S3C2410_GPCUP),

	SAVE_ITEM(S3C2410_GPDDAT),
	SAVE_ITEM(S3C2410_GPDCON),
	SAVE_ITEM(S3C2410_GPDUP),

	SAVE_ITEM(S3C2410_GPEDAT),
	SAVE_ITEM(S3C2410_GPECON),
	SAVE_ITEM(S3C2410_GPEUP),

	SAVE_ITEM(S3C2410_GPFDAT),
	SAVE_ITEM(S3C2410_GPFCON),
	SAVE_ITEM(S3C2410_GPFUP),

	SAVE_ITEM(S3C2410_GPGDAT),
	SAVE_ITEM(S3C2410_GPGCON),
	SAVE_ITEM(S3C2410_GPGUP),

	SAVE_ITEM(S3C2410_GPHDAT),
	SAVE_ITEM(S3C2410_GPHCON),
	SAVE_ITEM(S3C2410_GPHUP),

#ifdef CONFIG_CPU_S3C2440
	SAVE_ITEM(S3C2440_GPJDAT),
	SAVE_ITEM(S3C2440_GPJCON),
	SAVE_ITEM(S3C2440_GPJUP),
#endif /* CONFIG_CPU_S3C2440 */

	SAVE_ITEM(S3C2410_DCLKCON),
};

#ifdef CONFIG_CPU_S3C2412
static struct sleep_save s3c2412_gpio_save[] = {
	SAVE_ITEM(S3C2410_GPADAT),
	SAVE_ITEM(S3C2410_GPACON),

	SAVE_ITEM(S3C2410_GPBDAT),
	SAVE_ITEM(S3C2410_GPBCON),
	SAVE_ITEM(S3C2410_GPBUP),

	SAVE_ITEM(S3C2410_GPCDAT),
	SAVE_ITEM(S3C2410_GPCCON),
	SAVE_ITEM(S3C2410_GPCUP),

	SAVE_ITEM(S3C2410_GPDDAT),
	SAVE_ITEM(S3C2410_GPDCON),
	SAVE_ITEM(S3C2410_GPDUP),

	SAVE_ITEM(S3C2410_GPEDAT),
	SAVE_ITEM(S3C2410_GPECON),
	SAVE_ITEM(S3C2410_GPEUP),

	SAVE_ITEM(S3C2410_GPFDAT),
	SAVE_ITEM(S3C2410_GPFCON),
	SAVE_ITEM(S3C2410_GPFUP),

	SAVE_ITEM(S3C2410_GPGDAT),
	SAVE_ITEM(S3C2410_GPGCON),
	SAVE_ITEM(S3C2410_GPGUP),

	SAVE_ITEM(S3C2410_GPHDAT),
	SAVE_ITEM(S3C2410_GPHCON),
	SAVE_ITEM(S3C2410_GPHUP),

	SAVE_ITEM(S3C2412_DCLKCON),
};
#endif /* CONFIG_CPU_S3C2412 */

#if defined(CONFIG_CPU_S3C2443) || defined(CONFIG_CPU_S3C2450)
static struct sleep_save s3c2443_gpio_save[] = {
	SAVE_ITEM(S3C2410_GPADAT),
	SAVE_ITEM(S3C2410_GPACON),

	SAVE_ITEM(S3C2410_GPBDAT),
	SAVE_ITEM(S3C2410_GPBCON),
	SAVE_ITEM(S3C2410_GPBUP),

	SAVE_ITEM(S3C2410_GPCDAT),
	SAVE_ITEM(S3C2410_GPCCON),
	SAVE_ITEM(S3C2410_GPCUP),

	SAVE_ITEM(S3C2410_GPDDAT),
	SAVE_ITEM(S3C2410_GPDCON),
	SAVE_ITEM(S3C2410_GPDUP),

	SAVE_ITEM(S3C2410_GPEDAT),
	SAVE_ITEM(S3C2410_GPECON),
	SAVE_ITEM(S3C2410_GPEUP),

	SAVE_ITEM(S3C2410_GPFDAT),
	SAVE_ITEM(S3C2410_GPFCON),
	SAVE_ITEM(S3C2410_GPFUP),

	SAVE_ITEM(S3C2410_GPGDAT),
	SAVE_ITEM(S3C2410_GPGCON),
	SAVE_ITEM(S3C2410_GPGUP),

	SAVE_ITEM(S3C2410_GPHDAT),
	SAVE_ITEM(S3C2410_GPHCON),
	SAVE_ITEM(S3C2410_GPHUP),

	SAVE_ITEM(S3C2443_GPJCON),
	SAVE_ITEM(S3C2443_GPJDAT),
	SAVE_ITEM(S3C2443_GPJUDP),

	SAVE_ITEM(S3C2443_GPLCON),
	SAVE_ITEM(S3C2443_GPLDAT),
	SAVE_ITEM(S3C2443_GPLUDP),

	SAVE_ITEM(S3C2443_GPMCON),
	SAVE_ITEM(S3C2443_GPMDAT),
	SAVE_ITEM(S3C2443_GPMUDP),

	SAVE_ITEM(S3C2410_DCLKCON),
};
#endif /* CONFIG_CPU_S3C2443 || CONFIG_CPU_S3C2450 */


#ifdef CONFIG_S3C2410_PM_DEBUG

#define SAVE_UART(va) \
	SAVE_ITEM((va) + S3C2410_ULCON), \
	SAVE_ITEM((va) + S3C2410_UCON), \
	SAVE_ITEM((va) + S3C2410_UFCON), \
	SAVE_ITEM((va) + S3C2410_UMCON), \
	SAVE_ITEM((va) + S3C2410_UBRDIV)

static struct sleep_save uart_save[] = {
	SAVE_UART(S3C24XX_VA_UART0),
	SAVE_UART(S3C24XX_VA_UART1),
#ifndef CONFIG_CPU_S3C2400
	SAVE_UART(S3C24XX_VA_UART2),
#endif /* CONFIG_CPU_S3C2400 */
#if defined(CONFIG_CPU_S3C2443) || defined(CONFIG_CPU_S3C2450)
	SAVE_UART(S3C2443_VA_UART3),
#endif /* CONFIG_CPU_S3C2443 || CONFIG_CPU_S3C2450 */
};

/* debug
 *
 * we send the debug to printascii() to allow it to be seen if the
 * system never wakes up from the sleep
*/

extern void printascii(const char *);

static void pm_dbg(const char *fmt, ...)
{
	va_list va;
	char buff[256];

	va_start(va, fmt);
	vsprintf(buff, fmt, va);
	va_end(va);

	printascii(buff);
}

static void s3c_pm_debug_init(void)
{
	unsigned long tmp;

	/* re-start uart clocks */
#ifdef CONFIG_CPU_S3C2412
	if (IO_GetCpuType() == GOCPU_S3C2412) {
		tmp = __raw_readl(S3C2410_CLKCON);

		tmp |= S3C2412_CLKCON_UART0;
		tmp |= S3C2412_CLKCON_UART1;
		tmp |= S3C2412_CLKCON_UART2;

		__raw_writel(tmp, S3C2410_CLKCON);
	} else
#endif /* CONFIG_CPU_S3C2412 */
#ifdef CONFIG_CPU_S3C2443
	if (IO_GetCpuType() == GOCPU_S3C2443) {
		tmp = __raw_readl(S3C2443_PCLKCON);

		tmp |= S3C2443_PCLKCON_UART0;
		tmp |= S3C2443_PCLKCON_UART1;
		tmp |= S3C2443_PCLKCON_UART2;
		tmp |= S3C2443_PCLKCON_UART3;

		__raw_writel(tmp, S3C2410_CLKCON);
	} else
#endif /* CONFIG_CPU_S3C2443 */
#ifdef CONFIG_CPU_S3C2450
	if (IO_GetCpuType() == GOCPU_S3C2450) {
		tmp = __raw_readl(S3C2450_PCLKCON);

		tmp |= S3C2450_PCLKCON_UART0;
		tmp |= S3C2450_PCLKCON_UART1;
		tmp |= S3C2450_PCLKCON_UART2;
		tmp |= S3C2450_PCLKCON_UART3;

		__raw_writel(tmp, S3C2410_CLKCON);
	} else
#endif /* CONFIG_CPU_S3C2450 */
	{
		tmp = __raw_readl(S3C2410_CLKCON);

		tmp |= S3C2410_CLKCON_UART0;
		tmp |= S3C2410_CLKCON_UART1;
		tmp |= S3C2410_CLKCON_UART2;

		__raw_writel(tmp, S3C2410_CLKCON);
	}

	udelay(10);
}

#define DBG(fmt...) pm_dbg(fmt)
#else /* CONFIG_S3C2410_PM_DEBUG */
#define DBG(fmt...) printk(KERN_DEBUG fmt)

#define s3c_pm_debug_init() do { } while(0)

static struct sleep_save uart_save[] = {};
#endif /* CONFIG_S3C2410_PM_DEBUG */

#if defined(CONFIG_S3C_PM_CHECK) && CONFIG_S3C_PM_CHECK_CHUNKSIZE != 0

/* suspend checking code...
 *
 * this next area does a set of crc checks over all the installed
 * memory, so the system can verify if the resume was ok.
 *
 * CONFIG_S3C2410_PM_CHECK_CHUNKSIZE defines the block-size for the CRC,
 * increasing it will mean that the area corrupted will be less easy to spot,
 * and reducing the size will cause the CRC save area to grow
*/

#define CHECK_CHUNKSIZE (CONFIG_S3C_PM_CHECK_CHUNKSIZE * 1024)

static u32 crc_size;	/* size needed for the crc block */
static u32 *crcs;	/* allocated over suspend/resume */

typedef u32 *(run_fn_t)(struct resource *ptr, u32 *arg);

/* s3c_pm_run_res
 *
 * go thorugh the given resource list, and look for system ram
*/

static void s3c_pm_run_res(struct resource *ptr, run_fn_t fn, u32 *arg)
{
	while (ptr != NULL) {
		if (ptr->child != NULL)
			s3c2410_pm_run_res(ptr->child, fn, arg);

		if ((ptr->flags & IORESOURCE_MEM) &&
		    strcmp(ptr->name, "System RAM") == 0) {
			DBG("Found system RAM at %08lx..%08lx\n",
			    ptr->start, ptr->end);
			arg = (fn)(ptr, arg);
		}

		ptr = ptr->sibling;
	}
}

static void s3c_pm_run_sysram(run_fn_t fn, u32 *arg)
{
	s3c_pm_run_res(&iomem_resource, fn, arg);
}

static u32 *s3c_pm_countram(struct resource *res, u32 *val)
{
	u32 size = (u32)(res->end - res->start)+1;

	size += CHECK_CHUNKSIZE-1;
	size /= CHECK_CHUNKSIZE;

	DBG("Area %08lx..%08lx, %d blocks\n", res->start, res->end, size);

	*val += size * sizeof(u32);
	return val;
}

/* s3c_pm_prepare_check
 *
 * prepare the necessary information for creating the CRCs. This
 * must be done before the final save, as it will require memory
 * allocating, and thus touching bits of the kernel we do not
 * know about.
*/

static void s3c_pm_check_prepare(void)
{
	crc_size = 0;

	s3c_pm_run_sysram(s3c_pm_countram, &crc_size);

	DBG("s3c_pm_prepare_check: %u checks needed\n", crc_size);

	crcs = kmalloc(crc_size+4, GFP_KERNEL);
	if (crcs == NULL)
		printk(KERN_ERR "Cannot allocated CRC save area\n");
}

static u32 *s3c_pm_makecheck(struct resource *res, u32 *val)
{
	unsigned long addr, left;

	for (addr = res->start; addr < res->end;
	     addr += CHECK_CHUNKSIZE) {
		left = res->end - addr;

		if (left > CHECK_CHUNKSIZE)
			left = CHECK_CHUNKSIZE;

		*val = crc32_le(~0, phys_to_virt(addr), left);
		val++;
	}

	return val;
}

/* s3c_pm_check_store
 *
 * compute the CRC values for the memory blocks before the final
 * sleep.
*/

static void s3c_pm_check_store(void)
{
	if (crcs != NULL)
		s3c_pm_run_sysram(s3c_pm_makecheck, crcs);
}

/* in_region
 *
 * return TRUE if the area defined by ptr..ptr+size contatins the
 * what..what+whatsz
*/

static inline int in_region(void *ptr, int size, void *what, size_t whatsz)
{
	if ((what+whatsz) < ptr)
		return 0;

	if (what > (ptr+size))
		return 0;

	return 1;
}

static u32 *s3c_pm_runcheck(struct resource *res, u32 *val)
{
	void *save_at = phys_to_virt(s3c_sleep_save_phys);
	unsigned long addr;
	unsigned long left;
	void *ptr;
	u32 calc;

	for (addr = res->start; addr < res->end;
	     addr += CHECK_CHUNKSIZE) {
		left = res->end - addr;

		if (left > CHECK_CHUNKSIZE)
			left = CHECK_CHUNKSIZE;

		ptr = phys_to_virt(addr);

		if (in_region(ptr, left, crcs, crc_size)) {
			DBG("skipping %08lx, has crc block in\n", addr);
			goto skip_check;
		}

		if (in_region(ptr, left, save_at, 32*4 )) {
			DBG("skipping %08lx, has save block in\n", addr);
			goto skip_check;
		}

		/* calculate and check the checksum */

		calc = crc32_le(~0, ptr, left);
		if (calc != *val) {
			printk(KERN_ERR PFX "Restore CRC error at "
			       "%08lx (%08x vs %08x)\n", addr, calc, *val);

			DBG("Restore CRC error at %08lx (%08x vs %08x)\n",
			    addr, calc, *val);
		}

	skip_check:
		val++;
	}

	return val;
}

/* s3c_pm_check_restore
 *
 * check the CRCs after the restore event and free the memory used
 * to hold them
*/

static void s3c_pm_check_restore(void)
{
	if (crcs != NULL) {
		s3c_pm_run_sysram(s3c_pm_runcheck, crcs);
		kfree(crcs);
		crcs = NULL;
	}
}

#else

#define s3c_pm_check_prepare() do { } while(0)
#define s3c_pm_check_restore() do { } while(0)
#define s3c_pm_check_store()   do { } while(0)
#endif

/* helper functions to save and restore register state */

void s3c_pm_do_save(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		if (IO_GetCpuType() == GOCPU_S3C2443) {
			if (ptr->reg == S3C2410_EXTINT0) {
				ptr->val = s3c2443_read_extint0();
			} else if (ptr->reg == S3C2410_EXTINT1) {
				ptr->val = s3c2443_read_extint1();
			} else if (ptr->reg == S3C2410_EXTINT2) {
				ptr->val = s3c2443_read_extint2();
			} else if (ptr->reg == S3C2410_EINFLT2) {
				ptr->val = s3c2443_read_einflt2();
			} else if (ptr->reg == S3C2410_EINFLT3) {
				ptr->val = s3c2443_read_einflt3();
			} else if (ptr->reg == S3C2410_GPACON) {
				ptr->val = s3c2443_read_gpacon();
			} else if (ptr->reg == S3C2410_GPADAT) {
				ptr->val = s3c2443_read_gpadat();
			} else {
				ptr->val = __raw_readl(ptr->reg);
			}
		} else {
			ptr->val = __raw_readl(ptr->reg);
		}
		DBG("saved %p value %08lx\n", ptr->reg, ptr->val);
	}
}

/* s3c_pm_do_restore
 *
 * restore the system from the given list of saved registers
 *
 * Note, we do not use DBG() in here, as the system may not have
 * restore the UARTs state yet
*/

void s3c_pm_do_restore(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		printk(KERN_DEBUG "restore %p (restore %08lx, was %08x)\n",
		       ptr->reg, ptr->val, __raw_readl(ptr->reg));

		__raw_writel(ptr->val, ptr->reg);
	}
}

/* s3c_pm_do_restore_core
 *
 * similar to s3c_pm_do_restore_core
 *
 * WARNING: Do not put any debug in here that may effect memory or use
 * peripherals, as things may be changing!
*/

static void s3c_pm_do_restore_core(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		__raw_writel(ptr->val, ptr->reg);
	}
}

/* 
 *	On S3C2412 EVT3 (fixed in EVT4), software suspend/RTC wakeup is needed due to unreliable hardware watchdog
 *	Called from IRQ context
 */
void s3c2410_pm_watchdog_reboot(void *data)
{
	IO_Deactivate(BACKLIGHT_PWM);
	s3c_pm_sleep(1, 0); // sleep and wakeup using RTC
}

#define any_allowed(mask, allow) (((mask) & (allow)) != (allow))

void s3c_pm_sleep(int simulateWake, int linuxSuspend)
{
	unsigned long regs_save[16];

	/* prepare check area if configured */

	s3c_pm_check_prepare();

	/* store the physical address of the register recovery block */

	s3c_sleep_save_phys = virt_to_phys(regs_save);

	DBG("s3c_sleep_save_phys=0x%08lx\n", s3c_sleep_save_phys);

	/* ensure at least GSTATUS3 has the resume address */

	if (linuxSuspend) {
#ifdef CONFIG_CPU_S3C2412
		if (IO_GetCpuType() == GOCPU_S3C2412) {
			__raw_writel(virt_to_phys(s3c2410_cpu_resume), S3C2412_INFORM1);
		} else
#endif /* CONFIG_CPU_S3C2412 */
#ifdef CONFIG_CPU_S3C2443
		if (IO_GetCpuType() == GOCPU_S3C2443) {
			__raw_writel(virt_to_phys(s3c2410_cpu_resume), S3C2443_INFORM1);
		} else
#endif /* CONFIG_CPU_S3C2443 */
#ifdef CONFIG_CPU_S3C2450
		if (IO_GetCpuType() == GOCPU_S3C2450) {
			__raw_writel(virt_to_phys(s3c2410_cpu_resume), S3C2450_INFORM1);
		} else
#endif /* CONFIG_CPU_S3C2450 */
		{
			__raw_writel(virt_to_phys(s3c2410_cpu_resume), s3c24xx_gstatus3);	
		}
	}

#ifdef CONFIG_CPU_S3C2412
	if (IO_GetCpuType() == GOCPU_S3C2412) {
		DBG("INFORM0 0x%08x\n", __raw_readl(S3C2412_INFORM0));
		DBG("INFORM1 0x%08x\n", __raw_readl(S3C2412_INFORM1));
	} else
#endif /* CONFIG_CPU_S3C2412 */
#ifdef CONFIG_CPU_S3C2443
	if (IO_GetCpuType() == GOCPU_S3C2443) {
		DBG("INFORM0 0x%08x\n", __raw_readl(S3C2443_INFORM0));
		DBG("INFORM1 0x%08x\n", __raw_readl(S3C2443_INFORM1));
	} else
#endif /* CONFIG_CPU_S3C2443 */
#ifdef CONFIG_CPU_S3C2450
	if (IO_GetCpuType() == GOCPU_S3C2450) {
		DBG("INFORM0 0x%08x\n", __raw_readl(S3C2450_INFORM0));
		DBG("INFORM1 0x%08x\n", __raw_readl(S3C2450_INFORM1));
	} else
#endif /* CONFIG_CPU_S3C2450 */
	{
		DBG("GSTATUS3 0x%08x\n", __raw_readl(s3c24xx_gstatus3));
		DBG("GSTATUS4 0x%08x\n", __raw_readl(s3c24xx_gstatus4));
	}

	/* save all necessary core registers not covered by the drivers */
#ifdef CONFIG_CPU_S3C2412
	if (IO_GetCpuType() == GOCPU_S3C2412) {
		s3c_pm_do_save(s3c2412_gpio_save, ARRAY_SIZE(s3c2412_gpio_save));
		s3c_pm_do_save(s3c2412_irq_save,  ARRAY_SIZE(s3c2412_irq_save));
		s3c_pm_do_save(s3c2412_core_save, ARRAY_SIZE(s3c2412_core_save));
	} else
#endif /* CONFIG_CPU_S3C2412 */
#ifdef CONFIG_CPU_S3C2443
	if (IO_GetCpuType() == GOCPU_S3C2443) {
		s3c_pm_do_save(s3c2443_gpio_save, ARRAY_SIZE(s3c2443_gpio_save));
		s3c_pm_do_save(s3c2410_irq_save,  ARRAY_SIZE(s3c2410_irq_save));
		s3c_pm_do_save(s3c2443_core_save, ARRAY_SIZE(s3c2443_core_save));
	} else
#endif /* CONFIG_CPU_S3C2443 */
#ifdef CONFIG_CPU_S3C2450
	if (IO_GetCpuType() == GOCPU_S3C2450) {
		s3c_pm_do_save(s3c2443_gpio_save, ARRAY_SIZE(s3c2443_gpio_save));
		s3c_pm_do_save(s3c2410_irq_save,  ARRAY_SIZE(s3c2410_irq_save));
		s3c_pm_do_save(s3c2443_core_save, ARRAY_SIZE(s3c2443_core_save));
	} else
#endif /* CONFIG_CPU_S3C2450 */
	{
		s3c_pm_do_save(s3c2410_gpio_save, ARRAY_SIZE(s3c2410_gpio_save));
		s3c_pm_do_save(s3c2410_irq_save,  ARRAY_SIZE(s3c2410_irq_save));
		s3c_pm_do_save(s3c2410_core_save, ARRAY_SIZE(s3c2410_core_save));
	}

	s3c_pm_do_save(uart_save, ARRAY_SIZE(uart_save));

	tomtomgo_before_sleep(simulateWake, linuxSuspend);
	__raw_writel(0xffffffff, S3C2410_INTMSK);
	__raw_writel(0xffffffff, s3c24xx_eintmask);

	/* leave databus floating (default is floating) during sleep */
	/*__raw_writel(__raw_readl(s3c24xx_mslcon) | S3C2410_MSLCON_PSC_DATA, s3c24xx_mslcon);*/

	/* ack any outstanding external interrupts before we go to sleep */
	__raw_writel(__raw_readl(s3c24xx_eintpend), s3c24xx_eintpend);

	/* set USB pads to idle and enable pullups on databus */
	__raw_writel((__raw_readl(s3c24xx_misccr) & ~S3C2410_MISCCR_SPUCR_HDIS & ~S3C2410_MISCCR_SPUCR_LDIS) | S3C2410_MISCCR_USBSUSPND0 | S3C2410_MISCCR_USBSUSPND1, s3c24xx_misccr);

	/* flush cache back to ram */

	__cpuc_flush_kern_all();

	__raw_writel(__raw_readl(s3c24xx_misccr), s3c24xx_misccr);

	s3c_pm_check_store();

sleep_again:

#if defined(CONFIG_CPU_S3C2412) || \
    defined(CONFIG_CPU_S3C2440) || \
    defined(CONFIG_CPU_S3C2443) || \
    defined(CONFIG_CPU_S3C2450)
	if ( !simulateWake || linuxSuspend )
	{
		/* debounce EINT0 pin GPF0 */
		IO_SetInput(ON_OFF);
		while (IO_GetInput(ON_OFF))
			;
		IO_SetInterruptOnActivation(ON_OFF);
	}
#endif /* CONFIG_CPU_S3C2412 / 2440 / 2443 / 2450 */

#if defined(CONFIG_CPU_S3C2410) || defined(CONFIG_CPU_S3C2440) || defined(CONFIG_CPU_S3C2442)
	if (IO_GetCpuType() == GOCPU_S3C2410 || 
			IO_GetCpuType() == GOCPU_S3C2440 || 
			IO_GetCpuType() == GOCPU_S3C2442) {
		/* turn off clocks over sleep */
		__raw_writel(0x00, S3C2410_CLKCON);
	}
#endif /* CONFIG_CPU_S3C2410 / 2440 / 2442 */

	/* ack any outstanding interrupts before going to sleep */
	while ((__raw_readl(S3C2410_INTPND) != 0) || (__raw_readl(s3c24xx_eintpend) != 0))
	{
		__raw_writel(__raw_readl(s3c24xx_eintpend), s3c24xx_eintpend);
		__raw_writel(__raw_readl(S3C2410_SRCPND), S3C2410_SRCPND);
		__raw_writel(__raw_readl(S3C2410_INTPND), S3C2410_INTPND);
	}

	/* send the cpu to sleep... */
	s3c2410_cpu_suspend(regs_save);

	/* restore the cpu state */
	cpu_init();

	/* reset all flags to be sure */
	__raw_writel(__raw_readl(s3c24xx_gstatus2), s3c24xx_gstatus2);

	/* restore the system state */
#ifdef CONFIG_CPU_S3C2412
	if (IO_GetCpuType() == GOCPU_S3C2412) {
		s3c_pm_do_restore_core(s3c2412_core_save, ARRAY_SIZE(s3c2412_core_save));
		s3c_pm_do_restore(s3c2412_gpio_save, ARRAY_SIZE(s3c2412_gpio_save));
		s3c_pm_do_restore(s3c2412_irq_save, ARRAY_SIZE(s3c2412_irq_save));
	} else
#endif /* CONFIG_CPU_S3C2412 */
#ifdef CONFIG_CPU_S3C2443
	if (IO_GetCpuType() == GOCPU_S3C2443) {
		s3c_pm_do_restore_core(s3c2443_core_save, ARRAY_SIZE(s3c2443_core_save));
		s3c_pm_do_restore(s3c2443_gpio_save, ARRAY_SIZE(s3c2443_gpio_save));
		/* magic poke to power on retention I/O */
		__raw_writel(0x1fffe, S3C2443_RSTCON);
		s3c_pm_do_restore(s3c2410_irq_save, ARRAY_SIZE(s3c2410_irq_save));
	} else
#endif /* CONFIG_CPU_S3C2443 */
#ifdef CONFIG_CPU_S3C2450
	if (IO_GetCpuType() == GOCPU_S3C2450) {
		s3c_pm_do_restore_core(s3c2443_core_save, ARRAY_SIZE(s3c2443_core_save));
		s3c_pm_do_restore(s3c2443_gpio_save, ARRAY_SIZE(s3c2443_gpio_save));
		/* magic poke to power on retention I/O */
		__raw_writel(0x1fffe, S3C2443_RSTCON);
		s3c_pm_do_restore(s3c2410_irq_save, ARRAY_SIZE(s3c2410_irq_save));
	} else
#endif /* CONFIG_CPU_S3C2450 */
	{
		s3c_pm_do_restore_core(s3c2410_core_save, ARRAY_SIZE(s3c2410_core_save));
		s3c_pm_do_restore(s3c2410_gpio_save, ARRAY_SIZE(s3c2410_gpio_save));
		s3c_pm_do_restore(s3c2410_irq_save, ARRAY_SIZE(s3c2410_irq_save));
	}

	s3c_pm_do_restore(uart_save, ARRAY_SIZE(uart_save));

	s3c_pm_debug_init();

	DBG("going to decide to wakeup\n");

	if (!tomtomgo_decide_wakeup()) {
		__raw_writel(0xffffffff, S3C2410_INTMSK);
		__raw_writel(0xffffffff, s3c24xx_eintmask);
		goto sleep_again;
	}

	/* check what irq (if any) restored the system */

	DBG("post sleep: IRQs 0x%08x, 0x%08x\n",
	    __raw_readl(S3C2410_SRCPND),
	    __raw_readl(s3c24xx_eintpend));

	DBG("post sleep, preparing to return\n");

	s3c_pm_check_restore();

	/* ok, let's return from sleep */
	DBG("S3C2410 PM Resume (post-restore)\n");
	
	/* restore powersupply settings not done by drivers */
	tomtomgo_after_sleep();
}

/* s3c_pm_enter
 *
 * central control for sleep/resume process
*/

static int s3c_pm_enter(suspend_state_t state)
{
	/* ensure the debug is initialised (if enabled) */

	s3c_pm_debug_init();

	DBG("s3c2410_pm_enter(%d)\n", state);

	switch (state) {
	case PM_SUSPEND_MEM:
		/* Sleep without simulated wake and with resume to Linux */
		s3c_pm_sleep(0, 1);
		break;

	case PM_SUSPEND_STANDBY:
		/* Sleep with simulated wake and without resume to Linux */
		s3c_pm_sleep(1, 0);
		break;

	default:
		printk(KERN_ERR PFX "error: only PM_SUSPEND_MEM and PM_SUSPEND_STANDBY supported\n");
		return -EINVAL;
	}
	
	return 0;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int s3c_pm_prepare(suspend_state_t state)
{
	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static int s3c_pm_finish(suspend_state_t state)
{
	return 0;
}

/*
 * Set to PM_DISK_FIRMWARE so we can quickly veto suspend-to-disk.
 */
static struct pm_ops s3c_pm_ops = {
	.pm_disk_mode	= PM_DISK_FIRMWARE,
	.prepare	= s3c_pm_prepare,
	.enter		= s3c_pm_enter,
	.finish		= s3c_pm_finish,
};

/* s3c2410_pm_init
 *
 * Attach the power management functions. This should be called
 * from the board specific initialisation if the board supports
 * it.
*/

int __init s3c_pm_init(void)
{
	printk("S3C Power Management, (c) 2004 Simtec Electronics\n");

	pm_set_ops(&s3c_pm_ops);
	return 0;
}
