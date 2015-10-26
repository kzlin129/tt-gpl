/* linux/arch/arm/plat-s3c24xx/pm.c
 *
 * Copyright (c) 2004,2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C24XX Power Manager (Suspend-To-RAM) support
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
*/

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/crc32.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/serial_core.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include <asm/cacheflush.h>
#include <mach/hardware.h>

#include <plat/map-base.h>
#include <plat/regs-serial.h>
#include <plat/regs-clock.h>
#include <plat/regs-gpio.h>
#include <plat/regs-timer.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-mem.h>
#include <mach/regs-irq.h>
#include <asm/gpio.h>

#include <asm/mach/time.h>

#include <plat/pm.h>

/* for external use */

unsigned long s3c_pm_flags;

#define PFX "s3c64xx-pm: "
static struct sleep_save core_save[] = {
	SAVE_ITEM(S3C_APLL_LOCK),
	SAVE_ITEM(S3C_MPLL_LOCK),
	SAVE_ITEM(S3C_EPLL_LOCK),
	SAVE_ITEM(S3C_CLK_GATE_HCLK0),
	SAVE_ITEM(S3C_CLK_GATE_PCLK),
	SAVE_ITEM(S3C_CLK_GATE_SCLK0),
	SAVE_ITEM(S3C_CLK_GATE_MEM0),
	SAVE_ITEM(S3C_CLK_GATE_HCLK1),
	SAVE_ITEM(S3C_CLK_GATE_SCLK1),
	SAVE_ITEM(S3C_CLK_SRC0),
	SAVE_ITEM(S3C_CLK_SRC1),
	SAVE_ITEM(S3C_CLK_DIV0),
	SAVE_ITEM(S3C_CLK_DIV1),
	SAVE_ITEM(S3C_CLK_DIV2),
	SAVE_ITEM(S3C_CLK_DIV3),
	SAVE_ITEM(S3C_CLK_OUT),
	SAVE_ITEM(S3C_APLL_CON),
	SAVE_ITEM(S3C_MPLL_CON),
	SAVE_ITEM(S3C_EPLL_CON),
	SAVE_ITEM(S3C_EPLL_CON_K),
	SAVE_ITEM(S3C_AHB_CON0),
	SAVE_ITEM(S3C_PWR_CFG),
	SAVE_ITEM(S3C_NORMAL_CFG),
	SAVE_ITEM(S3C_STOP_CFG),
	SAVE_ITEM(S3C_SLEEP_CFG),
	SAVE_ITEM(S3C_OSC_FREQ),
	SAVE_ITEM(S3C_OSC_STABLE),
	SAVE_ITEM(S3C_PWR_STABLE),
	SAVE_ITEM(S3C_MTC_STABLE),
	SAVE_ITEM(S3C_SYS_OTHERS),
	SAVE_ITEM(S3C_OTHERS),
	SAVE_ITEM(S3C64XX_TINT_CSTAT),

        SAVE_ITEM(S5P64XX_EINT0CON0),
        SAVE_ITEM(S5P64XX_EINT0FLTCON0),
        SAVE_ITEM(S5P64XX_EINT0FLTCON1),
        SAVE_ITEM(S5P64XX_EINT0MASK),
};

static struct sleep_save gpio_save[] = {
	SAVE_ITEM(S5P64XX_GPACON),
	SAVE_ITEM(S5P64XX_GPADAT),
	SAVE_ITEM(S5P64XX_GPAPUD),
	SAVE_ITEM(S5P64XX_GPACONSLP),
	SAVE_ITEM(S5P64XX_GPAPUDSLP),
	SAVE_ITEM(S5P64XX_GPBCON),
	SAVE_ITEM(S5P64XX_GPBDAT),
	SAVE_ITEM(S5P64XX_GPBPUD),
	SAVE_ITEM(S5P64XX_GPBCONSLP),
	SAVE_ITEM(S5P64XX_GPBPUDSLP),
	SAVE_ITEM(S5P64XX_GPCCON),
	SAVE_ITEM(S5P64XX_GPCDAT),
	SAVE_ITEM(S5P64XX_GPCPUD),
	SAVE_ITEM(S5P64XX_GPCCONSLP),
	SAVE_ITEM(S5P64XX_GPCPUDSLP),
	SAVE_ITEM(S5P64XX_GPFCON),
	SAVE_ITEM(S5P64XX_GPFDAT),
	SAVE_ITEM(S5P64XX_GPFPUD),
	SAVE_ITEM(S5P64XX_GPFCONSLP),
	SAVE_ITEM(S5P64XX_GPFPUDSLP),
	SAVE_ITEM(S5P64XX_GPGCON),
	SAVE_ITEM(S5P64XX_GPGDAT),
	SAVE_ITEM(S5P64XX_GPGPUD),
	SAVE_ITEM(S5P64XX_GPGCONSLP),
	SAVE_ITEM(S5P64XX_GPGPUDSLP),
	SAVE_ITEM(S5P64XX_GPHCON0),
	SAVE_ITEM(S5P64XX_GPHCON1),
	SAVE_ITEM(S5P64XX_GPHDAT),
	SAVE_ITEM(S5P64XX_GPHPUD),
	SAVE_ITEM(S5P64XX_GPHCONSLP),
	SAVE_ITEM(S5P64XX_GPHPUDSLP),
	SAVE_ITEM(S5P64XX_GPICON),
	SAVE_ITEM(S5P64XX_GPIDAT),
	SAVE_ITEM(S5P64XX_GPIPUD),
	SAVE_ITEM(S5P64XX_GPICONSLP),
	SAVE_ITEM(S5P64XX_GPIPUDSLP),
	SAVE_ITEM(S5P64XX_GPJCON),
	SAVE_ITEM(S5P64XX_GPJDAT),
	SAVE_ITEM(S5P64XX_GPJPUD),
	SAVE_ITEM(S5P64XX_GPJCONSLP),
	SAVE_ITEM(S5P64XX_GPJPUDSLP),
	SAVE_ITEM(S5P64XX_GPNCON),
	SAVE_ITEM(S5P64XX_GPNDAT),
	SAVE_ITEM(S5P64XX_GPNPUD),
	SAVE_ITEM(S5P64XX_GPPCON),
	SAVE_ITEM(S5P64XX_GPPDAT),
	SAVE_ITEM(S5P64XX_GPPPUD),
	SAVE_ITEM(S5P64XX_GPPCONSLP),
	SAVE_ITEM(S5P64XX_GPPPUDSLP),
	SAVE_ITEM(S5P64XX_GPRCON0),
	SAVE_ITEM(S5P64XX_GPRCON1),
	SAVE_ITEM(S5P64XX_GPRDAT),
	SAVE_ITEM(S5P64XX_GPRPUD),
	SAVE_ITEM(S5P64XX_GPRCONSLP),
	SAVE_ITEM(S5P64XX_GPRPUDSLP),

	/* Special register */
	SAVE_ITEM(S5P64XX_SPC_BASE),
	SAVE_ITEM(S5P64XX_SPC1_BASE),
};

/* this lot should be really saved by the IRQ code */
/* VICXADDRESSXX initilaization to be needed */
static struct sleep_save irq_save[] = {
	SAVE_ITEM(S5P64XX_VIC0INTSELECT),
	SAVE_ITEM(S5P64XX_VIC1INTSELECT),
	SAVE_ITEM(S5P64XX_VIC0INTENABLE),
	SAVE_ITEM(S5P64XX_VIC1INTENABLE),
	SAVE_ITEM(S5P64XX_VIC0SOFTINT),
	SAVE_ITEM(S5P64XX_VIC1SOFTINT),
};

static struct sleep_save sromc_save[] = {
	SAVE_ITEM(S5P64XX_SROM_BW),
	SAVE_ITEM(S5P64XX_SROM_BC0),
	SAVE_ITEM(S5P64XX_SROM_BC1),
	SAVE_ITEM(S5P64XX_SROM_BC2),
	SAVE_ITEM(S5P64XX_SROM_BC3),
	SAVE_ITEM(S5P64XX_SROM_BC4),
	SAVE_ITEM(S5P64XX_SROM_BC5),
};

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
#endif
};

/* debug
 *
 * we send the debug to printascii() to allow it to be seen if the
 * system never wakes up from the sleep
*/

extern void printascii(const char *);

void pm_dbg(const char *fmt, ...)
{
	va_list va;
	char buff[256];

	va_start(va, fmt);
	vsprintf(buff, fmt, va);
	va_end(va);

	printascii(buff);
}

static void s3c2410_pm_debug_init(void)
{
	unsigned long tmp = __raw_readl(S3C_CLK_GATE_PCLK);

	/* re-start uart clocks */
	tmp |= S3C_CLKCON_PCLK_UART0;
	tmp |= S3C_CLKCON_PCLK_UART1;
	tmp |= S3C_CLKCON_PCLK_UART2;

	__raw_writel(tmp, S3C_CLK_GATE_PCLK);
	udelay(10);
}

#define DBG(fmt...) pm_dbg(fmt)
#else
#define DBG(fmt...)

#define s5p6440_pm_debug_init() do { } while(0)
#endif

#if defined(CONFIG_S3C2410_PM_CHECK) && CONFIG_S3C2410_PM_CHECK_CHUNKSIZE != 0

/* suspend checking code...
 *
 * this next area does a set of crc checks over all the installed
 * memory, so the system can verify if the resume was ok.
 *
 * CONFIG_S3C6410_PM_CHECK_CHUNKSIZE defines the block-size for the CRC,
 * increasing it will mean that the area corrupted will be less easy to spot,
 * and reducing the size will cause the CRC save area to grow
*/

#define CHECK_CHUNKSIZE (CONFIG_S3C2410_PM_CHECK_CHUNKSIZE * 1024)

static u32 crc_size;	/* size needed for the crc block */
static u32 *crcs;	/* allocated over suspend/resume */

typedef u32 *(run_fn_t)(struct resource *ptr, u32 *arg);

/* s5p6440_pm_run_res
 *
 * go thorugh the given resource list, and look for system ram
*/

static void s5p6440_pm_run_res(struct resource *ptr, run_fn_t fn, u32 *arg)
{
	while (ptr != NULL) {
		if (ptr->child != NULL)
			s5p6440_pm_run_res(ptr->child, fn, arg);

		if ((ptr->flags & IORESOURCE_MEM) &&
		    strcmp(ptr->name, "System RAM") == 0) {
			DBG("Found system RAM at %08lx..%08lx\n",
			    ptr->start, ptr->end);
			arg = (fn)(ptr, arg);
		}

		ptr = ptr->sibling;
	}
}

static void s5p6440_pm_run_sysram(run_fn_t fn, u32 *arg)
{
	s5p6440_pm_run_res(&iomem_resource, fn, arg);
}

static u32 *s5p6440_pm_countram(struct resource *res, u32 *val)
{
	u32 size = (u32)(res->end - res->start)+1;

	size += CHECK_CHUNKSIZE-1;
	size /= CHECK_CHUNKSIZE;

	DBG("Area %08lx..%08lx, %d blocks\n", res->start, res->end, size);

	*val += size * sizeof(u32);
	return val;
}

/* s5p6440_pm_prepare_check
 *
 * prepare the necessary information for creating the CRCs. This
 * must be done before the final save, as it will require memory
 * allocating, and thus touching bits of the kernel we do not
 * know about.
*/

static void s5p6440_pm_check_prepare(void)
{
	crc_size = 0;

	s5p6440_pm_run_sysram(s5p6440_pm_countram, &crc_size);

	DBG("s5p6440_pm_prepare_check: %u checks needed\n", crc_size);

	crcs = kmalloc(crc_size+4, GFP_KERNEL);
	if (crcs == NULL)
		printk(KERN_ERR "Cannot allocated CRC save area\n");
}

static u32 *s5p6440_pm_makecheck(struct resource *res, u32 *val)
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

/* s5p6440_pm_check_store
 *
 * compute the CRC values for the memory blocks before the final
 * sleep.
*/

static void s5p6440_pm_check_store(void)
{
	if (crcs != NULL)
		s5p6440_pm_run_sysram(s5p6440_pm_makecheck, crcs);
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

static u32 *s5p6440_pm_runcheck(struct resource *res, u32 *val)
{
	void *save_at = phys_to_virt(s5p6440_sleep_save_phys);
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

/* s5p6440_pm_check_restore
 *
 * check the CRCs after the restore event and free the memory used
 * to hold them
*/

static void s5p6440_pm_check_restore(void)
{
	if (crcs != NULL) {
		s5p6440_pm_run_sysram(s5p6440_pm_runcheck, crcs);
		kfree(crcs);
		crcs = NULL;
	}
}

#else

#define s5p6440_pm_check_prepare() do { } while(0)
#define s5p6440_pm_check_restore() do { } while(0)
#define s5p6440_pm_check_store()   do { } while(0)
#endif

/* helper functions to save and restore register state */

void s5p6440_pm_do_save(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		ptr->val = __raw_readl(ptr->reg);
		//DBG("saved %p value %08lx\n", ptr->reg, ptr->val);
	}
}

/* s5p6440_pm_do_restore
 *
 * restore the system from the given list of saved registers
 *
 * Note, we do not use DBG() in here, as the system may not have
 * restore the UARTs state yet
*/

void s5p6440_pm_do_restore(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		//printk(KERN_DEBUG "restore %p (restore %08lx, was %08x)\n",
		       //ptr->reg, ptr->val, __raw_readl(ptr->reg));

		__raw_writel(ptr->val, ptr->reg);
	}
}

/* s5p6440_pm_do_restore_core
 *
 * similar to s36410_pm_do_restore_core
 *
 * WARNING: Do not put any debug in here that may effect memory or use
 * peripherals, as things may be changing!
*/

/* s5p6440_pm_do_save_phy
 *
 * save register of system
 *
 * Note, I made this function to support driver with ioremap.
 * If you want to use this function, you should to input as first parameter
 * struct sleep_save_phy type
*/

int s3c2410_pm_do_save_phy(struct sleep_save_phy *ptr, struct platform_device *pdev, int count)
{
	void __iomem *target_reg;
	struct resource *res;
	u32 reg_size;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(res == NULL){
		printk(KERN_ERR "%s resource get error\n",__FUNCTION__);
		return 0;
	}
	reg_size = res->end - res->start + 1;
	target_reg = ioremap(res->start,reg_size);

	for (; count > 0; count--, ptr++) {
		ptr->val = readl(target_reg + (ptr->reg));
	}

	return 0;
}

/* s5p6440_pm_do_restore_phy
 *
 * restore register of system
 *
 * Note, I made this function to support driver with ioremap.
 * If you want to use this function, you should to input as first parameter
 * struct sleep_save_phy type
*/

int s3c2410_pm_do_restore_phy(struct sleep_save_phy *ptr, struct platform_device *pdev, int count)
{
	void __iomem *target_reg;
	struct resource *res;
	u32 reg_size;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(res == NULL){
		printk(KERN_ERR "%s resource get error\n",__FUNCTION__);
		return 0;
	}
	reg_size = res->end - res->start + 1;
	target_reg = ioremap(res->start,reg_size);

	for (; count > 0; count--, ptr++) {
		writel(ptr->val, (target_reg + ptr->reg));
	}

	return 0;
}

static void s5p6440_pm_do_restore_core(struct sleep_save *ptr, int count)
{
	for (; count > 0; count--, ptr++) {
		__raw_writel(ptr->val, ptr->reg);
	}
}

static void s5p6440_pm_configure_extint(void)
{
	int i;

	/* For each external interrupt configured as a wakeup source:
	 * - Clear the (possibly) pending interrupt.
	 * - Make sure it is unmasked.
	 * Then program the wakeup register, and pray.
	 */

	DBG("Suspending with wakeup mask = %08lx\n", s3c_irqwake_eintmask);

	for (i = 0; i < 16; i++) {
		if (!(s3c_irqwake_eintmask & (1 << i))) {
			__raw_writel((1UL << i), S5P64XX_EINT0PEND);
			__raw_writel(__raw_readl(S5P64XX_EINT0MASK) & ~(1UL << i), S5P64XX_EINT0MASK);
		}
	}

	__raw_writel(s3c_irqwake_eintmask & 0x0000ffff, S3C_EINT_WAKEUP_MASK);
}

static const int internal_wakeup_irq[32] = {
	[1]  = IRQ_RTC_ALARM,
	[2]  = IRQ_RTC_TIC,
	[3]  = IRQ_TSI,
	[9]  = IRQ_HSMMC0,
	[10] = IRQ_HSMMC1,
	[11] = IRQ_HSMMC2,
};

static int s5p6440_wakeup_cause_to_irq(u32 wakeup_stat, u32 eint_pend)
{
	int wakeup_level = __ffs(wakeup_stat);
	int irq;

	switch(wakeup_level) {
		case 0:
		return IRQ_EINT(__ffs(eint_pend));

		default:
		if ((irq = internal_wakeup_irq[wakeup_level]))
			return irq;
	}

	// Should never be reached...
	return -1;
}

void (*pm_cpu_prep)(void);
void (*pm_cpu_sleep)(void);
void (*pm_cpu_wakeup_irq)(int);

#define any_allowed(mask, allow) (((mask) & (allow)) != (allow))

/* s5p6440_pm_enter
 *
 * central control for sleep/resume process
*/

static int s5p6440_pm_enter(suspend_state_t state)
{
	unsigned long regs_save[16];
	unsigned int tmp;
	u32 wakeup_stat;
	u32 eint_pend;

	/* ensure the debug is initialised (if enabled) */

	DBG("s5p6440_pm_enter(%d)\n", state);

	if (pm_cpu_prep == NULL || pm_cpu_sleep == NULL) {
		printk(KERN_ERR PFX "error: no cpu sleep functions set\n");
		return -EINVAL;
	}

	/* prepare check area if configured */
	s5p6440_pm_check_prepare();

	/* store the physical address of the register recovery block */
	s5p6440_sleep_save_phys = virt_to_phys(regs_save);

	printk("s5p6440_sleep_save_phys=0x%08lx\n", s5p6440_sleep_save_phys);

	/* save all necessary core registers not covered by the drivers */

	s5p6440_pm_do_save(gpio_save, ARRAY_SIZE(gpio_save));
	s5p6440_pm_do_save(irq_save, ARRAY_SIZE(irq_save));
	s5p6440_pm_do_save(core_save, ARRAY_SIZE(core_save));
	s5p6440_pm_do_save(sromc_save, ARRAY_SIZE(sromc_save));

	/* ensure INF_REG0  has the resume address */
	__raw_writel(virt_to_phys(s5p6440_cpu_resume), S3C_INFORM0);

	/* set the irq configuration for wake */
	s5p6440_pm_configure_extint();

	/* call cpu specific preperation */

	pm_cpu_prep();

	/* flush cache back to ram */

	flush_cache_all();

	s5p6440_pm_check_store();

	/* send the cpu to sleep... */

	__raw_writel(0xffffffff, S5P64XX_VIC0INTENCLEAR);
	__raw_writel(0xffffffff, S5P64XX_VIC1INTENCLEAR);
	__raw_writel(0xffffffff, S5P64XX_VIC0SOFTINTCLEAR);
	__raw_writel(0xffffffff, S5P64XX_VIC1SOFTINTCLEAR);

	__raw_writel(1, S3C_OSC_STABLE);
	__raw_writel(1, S3C_PWR_STABLE);

	/* Set WFI instruction to SLEEP mode */

	tmp = __raw_readl(S3C_CLK_GATE_SCLK1);
	tmp |= (0x1<<2);
	__raw_writel(tmp, S3C_CLK_GATE_SCLK1);

	tmp = __raw_readl(S3C_CLK_GATE_HCLK0);
	tmp = tmp & ~(0x120);
	__raw_writel(tmp, S3C_CLK_GATE_HCLK0);
	tmp = __raw_readl(S3C_CLK_GATE_HCLK1);
	tmp = tmp & ~(0x06);
	__raw_writel(tmp, S3C_CLK_GATE_HCLK1);

	tmp = __raw_readl(S3C_PWR_CFG);
	tmp |= (0x3<<5);
	tmp &= ~(0x1f);
	__raw_writel(tmp, S3C_PWR_CFG);

	tmp = __raw_readl(S3C_SLEEP_CFG);
	tmp &= ~(0x1<<0);
	__raw_writel(tmp, S3C_SLEEP_CFG);

	/* Clear WAKEUP_STAT register for next wakeup -jc.lee */
	/* If this register do not be cleared, Wakeup will be failed */
	tmp = __raw_readl(S3C_WAKEUP_STAT);
	__raw_writel(tmp, S3C_WAKEUP_STAT);

	/* Hand external pads over to SLEEP control, GPIOs are now disconnected */
	__raw_writel(0x2, S5P64XX_SLPEN);

	/* s5p6440_cpu_save will also act as our return point from when
	 * we resume as it saves its own register state, so use the return
	 * code to differentiate return from save and return from sleep */

	if (s5p6440_cpu_save(regs_save) == 0) {
		flush_cache_all();
		pm_cpu_sleep();
	}

	/* restore the cpu state */
	cpu_init();

	/* restore the system state */
	s5p6440_pm_do_restore_core(core_save, ARRAY_SIZE(core_save));
	s5p6440_pm_do_restore(sromc_save, ARRAY_SIZE(sromc_save));
	s5p6440_pm_do_restore(gpio_save, ARRAY_SIZE(gpio_save));
	s5p6440_pm_do_restore(irq_save, ARRAY_SIZE(irq_save));

	/* Reconnect GPIO block */
	__raw_writel(0, S5P64XX_SLPEN);

	wakeup_stat = __raw_readl(S3C_WAKEUP_STAT);
	eint_pend = __raw_readl(S5P64XX_EINT0PEND);
	
	__raw_writel(0, S3C_EINT_WAKEUP_MASK);
	__raw_writel(eint_pend, S5P64XX_EINT0PEND);

	DBG("post sleep, preparing to return\n");

	s5p6440_pm_check_restore();

	if (pm_cpu_wakeup_irq)
		pm_cpu_wakeup_irq(s5p6440_wakeup_cause_to_irq(wakeup_stat, eint_pend));

	/* ok, let's return from sleep */
	DBG("S3C6410 PM Resume (post-restore)\n");
	return 0;
}

static struct platform_suspend_ops s5p6440_pm_ops = {
	.enter		= s5p6440_pm_enter,
	.valid		= suspend_valid_only_mem,
};

/* s5p6440_pm_init
 *
 * Attach the power management functions. This should be called
 * from the board specific initialisation if the board supports
 * it.
*/

int __init s5p6440_pm_init(void)
{
	printk("S5P6440 Power Management, (c) 2008 Samsung Electronics\n");

	suspend_set_ops(&s5p6440_pm_ops);
	return 0;
}
