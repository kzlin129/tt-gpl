/* arch/arm/plat-s3c/include/adc.h
 *
 * Copyright (c) 2008 Simtec Electronics
 *	http://armlinux.simnte.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C24XX ADC driver information
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_PLAT_ADC_H__
#define __ASM_PLAT_ADC_H__ 

struct s3c_adc_client;

typedef enum {
	AIN0,
	AIN1,
	AIN2,
	AIN3,
#ifdef CONFIG_CPU_S5P6440
	AIN4,
	AIN5,
	/* AIN6-AIN9 are used for TS, beware! */
	AIN6,
	AIN7,
	AIN8,
	AIN9,
	AIN10,
	AIN11,
#endif
} adc_channel;

typedef enum {
	INIT,
	SELECTED,
	FINISHED
} adc_select;

struct s3c_adc_mach_info 
{
	u16 delay;
	u8 presc;
	u8 resolution;
};

typedef void (* select_func )(unsigned int selected, void *data);
typedef void (* convert_func)(unsigned int d0, unsigned int d1, void *data);

extern int s3c_adc_start(struct s3c_adc_client *client,
			 unsigned int channel, unsigned int nr_samples);

extern struct s3c_adc_client *s3c_adc_register(struct platform_device *pdev,
					       select_func select, convert_func conv, void *data,
					       unsigned int is_ts);

extern void s3c_adc_release(struct s3c_adc_client *client);

extern int s3c_adc_set_ts_control (struct s3c_adc_client *client, unsigned int port, unsigned int value);
extern unsigned int s3c_adc_get_ts_control (struct s3c_adc_client *client, unsigned int port);

void __init s3c_adc_set_platdata(struct s3c_adc_mach_info * pd);/* actual function in devs.c */

#endif /* __ASM_PLAT_ADC_H */

