#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/stat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <plat/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/cpu-freq.h>

#define ARM_LE		0
#define INT_LE		1

enum PMIC_VOLTAGE {
	VOUT_0_90,
	VOUT_0_95,
	VOUT_1_00, 
	VOUT_1_05, 
	VOUT_1_10, 
	VOUT_1_15, 
	VOUT_1_20, 
	VOUT_1_25, 
	VOUT_1_30, 
	VOUT_1_35, 
	VOUT_1_40, 
	VOUT_1_45, 
	VOUT_1_50, 	
};

/* ltc3714 voltage table */
static const unsigned int voltage_table[13] = {
	0x13, 0x11, 0xf, 0xe, 0xd, 0xc, 0xb, 0xa, 0x9,
	0x8, 0x7, 0x6, 0x5,
};

/* frequency voltage matching table */
static const unsigned int frequency_match[][3] = {
/* frequency, Mathced VDD ARM voltage , Matched VDD INT*/
	{L0, VOUT_1_10, VOUT_1_10},	// 532Mhz @1100mv
	{L1, VOUT_1_00, VOUT_1_00},	// 266Mhz @1000mv
	//{L2, VOUT_0_95, VOUT_0_95},	// 133Mhz @950mv
	{L2, VOUT_1_00, VOUT_1_00},	// 133Mhz @1000mv
};

/* LTC3714 Setting Routine */
static int ltc3714_gpio_setting(void)
{
	gpio_direction_output(S5P64XX_GPN(11), 0);
	gpio_direction_output(S5P64XX_GPN(12), 0);
	gpio_direction_output(S5P64XX_GPN(13), 0);
	gpio_direction_output(S5P64XX_GPN(14), 0);
	gpio_direction_output(S5P64XX_GPN(15), 0);
	gpio_direction_output(S5P64XX_GPR(0), 0);
	gpio_direction_output(S5P64XX_GPR(1), 0);
	gpio_direction_output(S5P64XX_GPR(2), 0);

	s3c_gpio_setpull(S5P64XX_GPN(11), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5P64XX_GPN(12), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5P64XX_GPN(13), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5P64XX_GPN(14), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5P64XX_GPN(15), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5P64XX_GPR(0), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5P64XX_GPR(1), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5P64XX_GPR(2), S3C_GPIO_PULL_NONE);

	return 0;
}

static int set_ltc3714(unsigned int pwr, unsigned int index)
{
	int gpio_val;
	int voltage = frequency_match[index][pwr + 1];
	
	gpio_val = voltage_table[voltage];

	gpio_val &=0x1f;
	
	gpio_set_value(S5P64XX_GPN(12),(gpio_val >> 1)&0x1);
	gpio_set_value(S5P64XX_GPN(13),(gpio_val >> 2)&0x1);
	gpio_set_value(S5P64XX_GPN(14),(gpio_val >> 3)&0x1);
	gpio_set_value(S5P64XX_GPN(15),(gpio_val >> 4)&0x1);
	gpio_set_value(S5P64XX_GPN(11),(gpio_val >> 0)&0x1);

	if(pwr == ARM_LE) {
		gpio_set_value(S5P64XX_GPR(0), 1);
		udelay(10);
		gpio_set_value(S5P64XX_GPR(0), 0);
	} else if(pwr == INT_LE) {
		gpio_set_value(S5P64XX_GPR(2), 1);
		udelay(10);
		gpio_set_value(S5P64XX_GPR(2), 0);
	} else {
		printk("[error]: set_power, check mode [pwr] value\n");
		return -EINVAL;
	}

	return 0;
}

static int find_voltage(int freq)
{
	int index = 0;

	if(freq > frequency_match[0][0]){
		printk(KERN_ERR "frequecy is over then support frequency\n");
		return 0;
	}

	for(index = 0 ; index < ARRAY_SIZE(frequency_match) ; index++){
		if(freq >= frequency_match[index][0])
			return index;
	}

	printk("Cannot find matched voltage on table\n");

	return 0;
}

int set_power(unsigned int freq)
{
	int index;

	index = find_voltage(freq);

	set_ltc3714(ARM_LE, index);

	set_ltc3714(INT_LE, index);

	return 0;
}

EXPORT_SYMBOL(set_power);

void ltc3714_init(void)
{
	ltc3714_gpio_setting();
	set_power(L0);
	gpio_set_value(S5P64XX_GPR(1), 1);
}

EXPORT_SYMBOL(ltc3714_init);
