/*
 *  linux/arch/arm/plat-s3c64xx/s3c64xx-cpufreq.c
 *
 *  CPU frequency scaling for S3C64XX
 *
 *  Copyright (C) 2008 Samsung Electronics
 *
 *  Based on cpu-sa1110.c, Copyright (C) 2001 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>

//#include <mach/hardware.h>
#include <asm/system.h>
#include <linux/gpio.h>

#include <mach/map.h>
#include <plat/regs-clock.h>
#include <plat/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/gpio-bank-f.h>
#include <plat/cpu-freq.h>
#include <mach/regs-dmc.h>

#if defined(CONFIG_S5P6440_S5M8751)
#include <linux/mfd/s5m8751/s5m8751_pmic.h>
#endif


/* definition for power setting function */
#if defined(CONFIG_S5P64XX_LTC3714)
extern int set_power(unsigned int freq);
extern void ltc3714_init(void);

#elif defined(CONFIG_S5P6440_S5M8751)

extern int s5m8751_buck_set_voltage(int regulator, int mV);
/* frequency voltage matching table */
static const unsigned int dvfs_table[][3] = {
/* This table should match to "s5p6440_freq_table" */
/* frequency, Mathced VDD ARM voltage , Matched VDD INT */
	{L0, 1100, 1100},	// 532Mhz @1100mv
	{L1, 1000, 1000},	// 266Mhz @1000mv
//	{L2, 950, 950},		// 133Mhz @950mv
};

#else

#endif

//#define CLK_PROBING

/* frequency */
static struct cpufreq_frequency_table s5p6440_freq_table[] = {
	{L0, L0},
	{L1, L1},
#if defined(CONFIG_S5P64XX_LTC3714)		
	{L2, L2},
#endif		
	{0, CPUFREQ_TABLE_END},
};

/* TODO: Add support for SDRAM timing changes */

int s5p6440_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu)
		return -EINVAL;
	return cpufreq_frequency_table_verify(policy, s5p6440_freq_table);
}

unsigned int s5p6440_getspeed(unsigned int cpu)
{
	struct clk * mpu_clk;
	unsigned long rate;

	if (cpu)
		return 0;

	mpu_clk = clk_get(NULL, MPU_CLK);
	if (IS_ERR(mpu_clk))
		return 0;
	rate = clk_get_rate(mpu_clk) / KHZ_T;

	clk_put(mpu_clk);

	return rate;
}

static int s5p6440_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	struct clk * mpu_clk;
	struct cpufreq_freqs freqs;
	int ret = 0;
	unsigned long arm_clk;
	unsigned int index;

	mpu_clk = clk_get(NULL, MPU_CLK);
	if (IS_ERR(mpu_clk))
		return PTR_ERR(mpu_clk);

	freqs.old = s5p6440_getspeed(0);
	if (cpufreq_frequency_table_target(policy, s5p6440_freq_table, target_freq, relation, &index))
		return -EINVAL;

	arm_clk = s5p6440_freq_table[index].frequency;

	freqs.new = arm_clk;
	freqs.cpu = 0;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	if(freqs.new < freqs.old){
		/* frequency scaling */
		ret = clk_set_rate(mpu_clk, target_freq * KHZ_T);
		if(ret != 0)
			printk("frequency scaling error\n");
		/* voltage scaling */
#if defined(CONFIG_S5P64XX_LTC3714)		
		set_power(freqs.new);
#elif defined(CONFIG_S5P6440_S5M8751)

		/* GPN12 -> High */
		gpio_set_value(S5P64XX_GPN(12), 1);
#endif
	}else{
		/* voltage scaling */
#if defined(CONFIG_S5P64XX_LTC3714)		
		set_power(freqs.new);
#elif defined(CONFIG_S5P6440_S5M8751)

		/* GPN12 -> Low */
		gpio_set_value(S5P64XX_GPN(12), 0);
#endif
		/* frequency scaling */
		ret = clk_set_rate(mpu_clk, target_freq * KHZ_T);
		if(ret != 0)
			printk("frequency scaling error\n");
	}

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	clk_put(mpu_clk);
	return ret;
}

static int __init s5p6440_cpu_init(struct cpufreq_policy *policy)
{
	struct clk * mpu_clk;
	unsigned int stat_reg, con_reg;
#if defined(CONFIG_S5P6440_S5M8751)
	unsigned int gpio_req;
#endif
	
	/* Turn off DLL of DMC and set lock value for DLL off manually */
	/* U-boot turns on DLL of DMC by default */
	con_reg = __raw_readl(S5P64XX_PHYCONTROL0);
	__raw_writel(con_reg |(1<<1), S5P64XX_PHYCONTROL0); // DLL on
	__raw_writel(con_reg |(1<<0), S5P64XX_PHYCONTROL0); // DLL start

	do {
		stat_reg = __raw_readl(S5P64XX_PHYSTATUS);
	} while(!((stat_reg>>2)&0x1));	//Wait until DLL lock

	stat_reg = (stat_reg>>6) & (0xff); // get locked value of DLL
	con_reg = __raw_readl(S5P64XX_PHYCONTROL0);
	con_reg &= ~(0xff<<24);
	con_reg |=(stat_reg<<24);
	__raw_writel(con_reg, S5P64XX_PHYCONTROL0); // force lock value for DLL off
	__raw_writel(con_reg & ~(1<<1), S5P64XX_PHYCONTROL0); // DLL off

#if defined(CONFIG_S5P64XX_LTC3714)
	ltc3714_init();
#endif

#if defined(CONFIG_S5P6440_S5M8751)
	/* GPIO configuration */
	/* GPN12 Low:1.1v, High:1.0v*/

	gpio_req = gpio_request(S5P64XX_GPN(12), "GPN");
	if (gpio_req) {
		printk(KERN_ERR "failed to request GPN for PMIC control\n");

		return gpio_req;
	}
	gpio_direction_output(S5P64XX_GPN(12), 0);
	
	s3c_gpio_setpull(S5P64XX_GPN(12), S3C_GPIO_PULL_UP);
#endif


#ifdef CLK_PROBING
	/* GPF14 pin is configured CLKOUT pin */
	s3c_gpio_cfgpin(S5P64XX_GPF(14), S3C_GPIO_SFN(3));
	__raw_writel((__raw_readl(S3C_CLK_OUT)&~(0xf<<12)), S3C_CLK_OUT);

#endif
	mpu_clk = clk_get(NULL, MPU_CLK);
	if (IS_ERR(mpu_clk))
		return PTR_ERR(mpu_clk);

	if (policy->cpu != 0)
		return -EINVAL;
	policy->cur = policy->min = policy->max = s5p6440_getspeed(0);
	cpufreq_frequency_table_get_attr(s5p6440_freq_table, policy->cpu);
	policy->cpuinfo.transition_latency = CPUFREQ_ETERNAL;

	clk_put(mpu_clk);
	return cpufreq_frequency_table_cpuinfo(policy, s5p6440_freq_table);
}

static struct cpufreq_driver s5p6440_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= s5p6440_verify_speed,
	.target		= s5p6440_target,
	.get		= s5p6440_getspeed,
	.init		= s5p6440_cpu_init,
	.name		= "s5p6440",
};

static int __init s5p6440_cpufreq_init(void)
{
	return cpufreq_register_driver(&s5p6440_driver);
}

arch_initcall(s5p6440_cpufreq_init);
