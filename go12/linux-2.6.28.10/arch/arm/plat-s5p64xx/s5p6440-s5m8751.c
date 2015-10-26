/*
 * s5p6440-s5m8751.c  --  S5M8751 Power-Audio IC ALSA Soc Audio driver
 *
 * Copyright 2009 Samsung Electronics.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/i2c.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include <linux/stat.h>
#include <linux/proc_fs.h>

#include <linux/delay.h>

#include <linux/mfd/s5m8751/s5m8751_core.h>
#include <linux/mfd/s5m8751/s5m8751_audio.h>
#include <linux/mfd/s5m8751/s5m8751_pmic.h>
#include <linux/mfd/s5m8751/s5m8751_regulator.h>

#include "s5p6440-s5m8751.h"

struct proc_dir_entry *proc_root_fp	= NULL;
struct proc_dir_entry *proc_voltage_fp	= NULL;

char proc_ldo_audio_voltage_str[PAGE_SIZE-80]	= { 0,};
char proc_ldo1_voltage_str[PAGE_SIZE-80]	= { 0,};
char proc_ldo2_voltage_str[PAGE_SIZE-80]	= { 0,};
char proc_ldo3_voltage_str[PAGE_SIZE-80]	= { 0,};
char proc_ldo4_voltage_str[PAGE_SIZE-80]	= { 0,};
char proc_ldo_mem_voltage_str[PAGE_SIZE-80]	= { 0,};
char proc_buck1_1_voltage_str[PAGE_SIZE-80]	= { 0,};
char proc_buck1_2_voltage_str[PAGE_SIZE-80]	= { 0,};
char proc_buck2_1_voltage_str[PAGE_SIZE-80]	= { 0,};
char proc_buck2_2_voltage_str[PAGE_SIZE-80]	= { 0,};

static s5p6440_s5m8751_info s5m8751_info;

extern struct s5m8751 *s5m8751;

static const unsigned int ldo_voltage_table[16] = {
	1800, 1900, 2000, 2100, 2200, 2300, 2400, 2500,
	2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300,
};


static const unsigned int ldo3_4_voltage_table[16] = {
	800, 900, 1000, 1100, 1200, 1300, 1400, 1500,
	1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300,
};

static const unsigned int buck1_voltage_table[61] = {
	1800, 1825, 1850, 1875, 1900, 1925, 1950, 1975,
	2000, 2025, 2050, 2075, 2100, 2125, 2150, 2175,
	2200, 2225, 2250, 2275, 2300, 2325, 2350, 2375,
	2400, 2425, 2450, 2475, 2500, 2525, 2550, 2575,
	2600, 2625, 2650, 2675, 2700, 2725, 2750, 2775,
	2800, 2825, 2850, 2875, 2900, 2925, 2950, 2975,
	3000, 3025, 3050, 3075, 3100, 3125, 3150, 3175,
	3200, 3225, 3250, 3275, 3300,
};

static const unsigned int buck2_voltage_table[47] = {
	500, 525, 550, 575, 600, 625, 650, 675, 
	700, 725, 750, 775, 800, 825, 850, 875,
	900, 925, 950, 975, 1000, 1025, 1050, 1075,
	1100, 1125, 1150, 1175, 1200, 1225, 1250, 1275,
	1300, 1325, 1350, 1375, 1400, 1425, 1450, 1475,
	1500, 1525, 1550, 1575, 1600, 1625, 1650,
};

int read_ldo_audio_proc_voltage(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	char *buf;
	char *realdata;
	int mV;

	realdata = (char *) data_unused;
	buf = page;
	//buf += sprintf(buf, "S5M8751 LDO audio voltage = [%s] mV \n", realdata);
	
	mV = s5m8751_ldo_get_voltage(S5M8751_LDO_AUDIO);
	printk("\nS5M8751 LDO audio current voltage = [%d] mV\n", mV);
	
	*eof = 1;

	return buf - page;
}


int read_ldo1_proc_voltage(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	char *buf;
	char *realdata;
	int mV;

	realdata = (char *) data_unused;
	buf = page;
//	buf += sprintf(buf, "S5M8751 LDO1 voltage = [%s] mV\n", realdata);
	
	mV = s5m8751_ldo_get_voltage(S5M8751_LDO1);
	printk("\nS5M8751 LDO1 current voltage = [%d] mV\n", mV);

	*eof = 1;

	return buf - page;
}

int read_ldo2_proc_voltage(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	char *buf;
	char *realdata;
	int mV;

	realdata = (char *) data_unused;
	buf = page;
//	buf += sprintf(buf, "S5M8751 LDO2 voltage = [%s] mV\n", realdata);

	mV = s5m8751_ldo_get_voltage(S5M8751_LDO2);
	printk("\nS5M8751 LDO2 current voltage = [%d] mV\n", mV);

	*eof = 1;

	return buf - page;
}

int read_ldo3_proc_voltage(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	char *buf;
	char *realdata;
	int mV;

	realdata = (char *) data_unused;
	buf = page;
//	buf += sprintf(buf, "S5M8751 LDO3 voltage = [%s] mV\n", realdata);

	mV = s5m8751_ldo_get_voltage(S5M8751_LDO3);
	printk("\nS5M8751 LDO3 current voltage = [%d] mV\n", mV);

	*eof = 1;

	return buf - page;
}

int read_ldo4_proc_voltage(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	char *buf;
	char *realdata;
	int mV;

	realdata = (char *) data_unused;
	buf = page;
//	buf += sprintf(buf, "S5M8751 LDO4 voltage = [%s] mV\n", realdata);

	mV = s5m8751_ldo_get_voltage(S5M8751_LDO4);
	printk("\nS5M8751 LDO4 current voltage = [%d] mV\n", mV);

	*eof = 1;

	return buf - page;
}

int read_ldo_mem_proc_voltage(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	char *buf;
	char *realdata;
	int mV;

	realdata = (char *) data_unused;
	buf = page;
//	buf += sprintf(buf, "S5M8751 LDO memory voltage = [%s] mV\n", realdata);

	mV = s5m8751_ldo_get_voltage(S5M8751_LDO_MEMORY);
	printk("\nS5M8751 LDO MEMORY current voltage = [%d] mV\n", mV);

	*eof = 1;

	return buf - page;
}

int read_buck1_1_proc_voltage(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	char *buf;
	char *realdata;
	int mV;

	realdata = (char *) data_unused;
	buf = page;
//	buf += sprintf(buf, "S5M8751 BUCK1 V1 voltage = [%s] mV\n", realdata);

	mV = s5m8751_buck_get_voltage(S5M8751_BUCK1_1);
	printk("\nS5M8751 BUCK1 V1 current voltage = [%d] mV\n", mV);

	*eof = 1;

	return buf - page;
}

int read_buck1_2_proc_voltage(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	char *buf;
	char *realdata;
	int mV;

	realdata = (char *) data_unused;
	buf = page;
//	buf += sprintf(buf, "S5M8751 BUCK1 V2 voltage = [%s] mV\n", realdata);

	mV = s5m8751_buck_get_voltage(S5M8751_BUCK1_2);
	printk("\nS5M8751 BUCK1 V2 current voltage = [%d] mV\n", mV);

	*eof = 1;

	return buf - page;
}

int read_buck2_1_proc_voltage(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	char *buf;
	char *realdata;
	int mV;

	realdata = (char *) data_unused;
	buf = page;
//	buf += sprintf(buf, "S5M8751 BUCK2 V1 voltage = [%s] mV\n", realdata);

	mV = s5m8751_buck_get_voltage(S5M8751_BUCK2_1);
	printk("\nS5M8751 BUCK2 V1 current voltage = [%d] mV\n", mV);

	*eof = 1;

	return buf - page;
}

int read_buck2_2_proc_voltage(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
	char *buf;
	char *realdata;
	int mV;

	realdata = (char *) data_unused;
	buf = page;
//	buf += sprintf(buf, "S5M8751 BUCK2 V2 voltage = [%s] mV\n", realdata);

	mV = s5m8751_buck_get_voltage(S5M8751_BUCK2_2);
	printk("\nS5M8751 BUCK2 V2 current voltage = [%d] mV\n", mV);

	*eof = 1;

	return buf - page;
}

int write_ldo_audio_proc_voltage(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int len;
	char *realdata;

	realdata = (char *) data;

	printk("\ns5m8751 LDO audio write proc voltage\n");
	if (copy_from_user(realdata, buffer, count))
		return -EFAULT;

	realdata[count] = '\0';
	len = strlen(realdata);
	if (realdata[len-1] == '\n')
		realdata[--len] = 0;
	
	s5m8751_info.new_voltage = simple_strtoul(realdata, NULL, 10);

	printk("\nnew voltage = %d\n", s5m8751_info.new_voltage);	

	if(s5m8751_info.new_voltage < ldo_voltage_table[0] || s5m8751_info.new_voltage > ldo_voltage_table[15]) 
	{
		printk("\n[ERROR] : LDO audio voltage value over limits~!!\n");
		return -EINVAL;
	}

	s5m8751_ldo_set_voltage(S5M8751_LDO_AUDIO, s5m8751_info.new_voltage);
	s5m8751_info.curr_voltage_ldo_audio = s5m8751_info.new_voltage;
	return count;
}



int write_ldo1_proc_voltage(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int len;
	char *realdata;

	realdata = (char *) data;

	printk("\ns5m8751 LDO1 write proc voltage\n");
	if(copy_from_user(realdata, buffer, count))
		return -EFAULT;

	realdata[count] = '\0';
	len = strlen(realdata);
	if(realdata[len-1] == '\n')
		realdata[--len] = 0;
	
	s5m8751_info.new_voltage = simple_strtoul(realdata, NULL, 10);

	printk("\nnew voltage = %d mV\n", s5m8751_info.new_voltage);	

	if(s5m8751_info.new_voltage < ldo_voltage_table[0] || s5m8751_info.new_voltage > ldo_voltage_table[15]) 
	{
		printk("\n[ERROR] : LDO1 voltage value over limits~!!\n");
		return -EINVAL;
	}

	s5m8751_ldo_set_voltage(S5M8751_LDO1, s5m8751_info.new_voltage);
	s5m8751_info.curr_voltage_ldo1 = s5m8751_info.new_voltage;
	return count;
}

int write_ldo2_proc_voltage(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int len;
	char *realdata;

	realdata = (char *) data;
	printk("\ns5m8751 LDO2 write proc voltage\n");

	if(copy_from_user(realdata, buffer, count))
		return -EFAULT;

	realdata[count] = '\0';
	len = strlen(realdata);
	if(realdata[len-1] == '\n')
		realdata[--len] = 0;
	
	s5m8751_info.new_voltage = simple_strtoul(realdata, NULL, 10);

	printk("\nnew voltage = %d\n", s5m8751_info.new_voltage);	

	if(s5m8751_info.new_voltage < ldo_voltage_table[0] || s5m8751_info.new_voltage > ldo_voltage_table[15]) 
	{
		printk("\n[ERROR] : LDO2 voltage value over limits~!!\n");
		return -EINVAL;
	}

	s5m8751_ldo_set_voltage(S5M8751_LDO2, s5m8751_info.new_voltage);
	s5m8751_info.curr_voltage_ldo2 = s5m8751_info.new_voltage;
	return count;
}

int write_ldo3_proc_voltage(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int len;
	char *realdata;

	realdata = (char *) data;
	printk("\ns5m8751 LDO3 write proc voltage\n");

	if(copy_from_user(realdata, buffer, count))
		return -EFAULT;

	realdata[count] = '\0';
	len = strlen(realdata);
	if(realdata[len-1] == '\n')
		realdata[--len] = 0;
	
	s5m8751_info.new_voltage = simple_strtoul(realdata, NULL, 10);

	printk("\nnew voltage = %d\n", s5m8751_info.new_voltage);	

	if(s5m8751_info.new_voltage < ldo3_4_voltage_table[0] || s5m8751_info.new_voltage > ldo3_4_voltage_table[15]) 
	{
		printk("\n[ERROR] : LDO3 voltage value over limits~!!\n");
		return -EINVAL;
	}

	s5m8751_ldo_set_voltage(S5M8751_LDO3, s5m8751_info.new_voltage);
	s5m8751_info.curr_voltage_ldo3 = s5m8751_info.new_voltage;
	return count;
}

int write_ldo4_proc_voltage(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int len;
	char *realdata;

	realdata = (char *) data;
	printk("\ns5m8751 LDO4 write proc voltage\n");

	if(copy_from_user(realdata, buffer, count))
		return -EFAULT;

	realdata[count] = '\0';
	len = strlen(realdata);
	if(realdata[len-1] == '\n')
		realdata[--len] = 0;
	
	s5m8751_info.new_voltage = simple_strtoul(realdata, NULL, 10);

	printk("\nnew voltage = %d\n", s5m8751_info.new_voltage);	

	if(s5m8751_info.new_voltage < ldo3_4_voltage_table[0] || s5m8751_info.new_voltage > ldo3_4_voltage_table[15]) 
	{
		printk("\n[ERROR] : LDO4 voltage value over limits~!!\n");
		return -EINVAL;
	}

	s5m8751_ldo_set_voltage(S5M8751_LDO4, s5m8751_info.new_voltage);
	s5m8751_info.curr_voltage_ldo4 = s5m8751_info.new_voltage;
	
	return count;
}

int write_ldo_mem_proc_voltage(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int len;
	char *realdata;

	realdata = (char *) data;
	printk("\ns5m8751 LDO memory write proc voltage\n");

	if(copy_from_user(realdata, buffer, count))
		return -EFAULT;

	realdata[count] = '\0';
	len = strlen(realdata);
	if(realdata[len-1] == '\n')
		realdata[--len] = 0;
	
	s5m8751_info.new_voltage = simple_strtoul(realdata, NULL, 10);

	printk("\nnew voltage = %d\n", s5m8751_info.new_voltage);	

	if(s5m8751_info.new_voltage < ldo_voltage_table[0] || s5m8751_info.new_voltage > ldo_voltage_table[15]) 
	{
		printk("\n[ERROR] : LDO memory voltage value over limits~!!\n");
		return -EINVAL;
	}

	s5m8751_ldo_set_voltage(S5M8751_LDO_MEMORY, s5m8751_info.new_voltage);
	s5m8751_info.curr_voltage_ldo_memory = s5m8751_info.new_voltage;
	
	return count;
}

int write_buck1_1_proc_voltage(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int len;
	char *realdata;

	realdata = (char *) data;
	printk("\ns5m8751 BUCK1 V1 write proc voltage\n");

	if(copy_from_user(realdata, buffer, count))
		return -EFAULT;

	realdata[count] = '\0';
	len = strlen(realdata);
	if(realdata[len-1] == '\n')
		realdata[--len] = 0;
	
	s5m8751_info.new_voltage = simple_strtoul(realdata, NULL, 10);

	printk("\nnew voltage = %d\n", s5m8751_info.new_voltage);	

	if(s5m8751_info.new_voltage < buck1_voltage_table[0] || s5m8751_info.new_voltage > buck1_voltage_table[15]) 
	{
		printk("\n[ERROR] : BUCK1 voltage value over limits~!!\n");
		return -EINVAL;
	}

	s5m8751_buck_set_voltage(S5M8751_BUCK1_1, s5m8751_info.new_voltage);
	s5m8751_info.curr_voltage_buck1 = s5m8751_info.new_voltage;
	
	return count;
}

int write_buck1_2_proc_voltage(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int len;
	char *realdata;

	realdata = (char *) data;
	printk("\ns5m8751 BUCK1 V2 write proc voltage\n");

	if(copy_from_user(realdata, buffer, count))
		return -EFAULT;

	realdata[count] = '\0';
	len = strlen(realdata);
	if(realdata[len-1] == '\n')
		realdata[--len] = 0;
	
	s5m8751_info.new_voltage = simple_strtoul(realdata, NULL, 10);

	printk("\nnew voltage = %d\n", s5m8751_info.new_voltage);	

	if(s5m8751_info.new_voltage < buck1_voltage_table[0] || s5m8751_info.new_voltage > buck1_voltage_table[15]) 
	{
		printk("\n[ERROR] : BUCK1 voltage value over limits~!!\n");
		return -EINVAL;
	}

	s5m8751_buck_set_voltage(S5M8751_BUCK1_2, s5m8751_info.new_voltage);
	s5m8751_info.curr_voltage_buck1 = s5m8751_info.new_voltage;
	
	return count;
}

int write_buck2_1_proc_voltage(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int len;
	char *realdata;

	realdata = (char *) data;
	printk("\ns5m8751 BUCK2 V1 write proc voltage\n");

	if(copy_from_user(realdata, buffer, count))
		return -EFAULT;

	realdata[count] = '\0';
	len = strlen(realdata);
	if(realdata[len-1] == '\n')
		realdata[--len] = 0;
	
	s5m8751_info.new_voltage = simple_strtoul(realdata, NULL, 10);

	printk("\nnew voltage = %d\n", s5m8751_info.new_voltage);	

	if(s5m8751_info.new_voltage < buck2_voltage_table[0] || s5m8751_info.new_voltage > buck2_voltage_table[15]) 
	{
		printk("\n[ERROR] : BUCK2 voltage value over limits~!!\n");
		return -EINVAL;
	}
	
	s5m8751_buck_set_voltage(S5M8751_BUCK2_1, s5m8751_info.new_voltage);
	s5m8751_info.curr_voltage_buck2 = s5m8751_info.new_voltage;
	
	return count;
}

int write_buck2_2_proc_voltage(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	int len;
	char *realdata;

	realdata = (char *) data;
	printk("\ns5m8751 BUCK2 V2 write proc voltage\n");

	if(copy_from_user(realdata, buffer, count))
		return -EFAULT;

	realdata[count] = '\0';
	len = strlen(realdata);
	if(realdata[len-1] == '\n')
		realdata[--len] = 0;
	
	s5m8751_info.new_voltage = simple_strtoul(realdata, NULL, 10);

	printk("\nnew voltage = %d\n", s5m8751_info.new_voltage);	

	if(s5m8751_info.new_voltage < buck2_voltage_table[0] || s5m8751_info.new_voltage > buck2_voltage_table[15]) 
	{
		printk("\n[ERROR] : BUCK2 voltage value over limits~!!\n");
		return -EINVAL;
	}

	s5m8751_buck_set_voltage(S5M8751_BUCK2_2, s5m8751_info.new_voltage);
	s5m8751_info.curr_voltage_buck2 = s5m8751_info.new_voltage;
	
	return count;
}

int s5m8751_proc_init(void)
{
	proc_root_fp = proc_mkdir("s5m8751", 0 );

	proc_voltage_fp = create_proc_entry("ldo_audio_voltage", S_IFREG | S_IRWXU, proc_root_fp);
	if(proc_voltage_fp) {
		proc_voltage_fp->data = proc_ldo_audio_voltage_str;
		proc_voltage_fp->read_proc = read_ldo_audio_proc_voltage;
		proc_voltage_fp->write_proc = write_ldo_audio_proc_voltage;
	}
	
	proc_voltage_fp = create_proc_entry("ldo1_voltage", S_IFREG | S_IRWXU, proc_root_fp);
	if(proc_voltage_fp) {
		proc_voltage_fp->data = proc_ldo1_voltage_str;
		proc_voltage_fp->read_proc = read_ldo1_proc_voltage;
		proc_voltage_fp->write_proc = write_ldo1_proc_voltage;
	}
	
	proc_voltage_fp = create_proc_entry("ldo2_voltage", S_IFREG | S_IRWXU, proc_root_fp);
	if(proc_voltage_fp) {
		proc_voltage_fp->data = proc_ldo2_voltage_str;
		proc_voltage_fp->read_proc = read_ldo2_proc_voltage;
		proc_voltage_fp->write_proc = write_ldo2_proc_voltage;
	}
	
	proc_voltage_fp = create_proc_entry("ldo3_voltage", S_IFREG | S_IRWXU, proc_root_fp);
	if(proc_voltage_fp) {
		proc_voltage_fp->data = proc_ldo3_voltage_str;
		proc_voltage_fp->read_proc = read_ldo3_proc_voltage;
		proc_voltage_fp->write_proc = write_ldo3_proc_voltage;
	}
	
	proc_voltage_fp = create_proc_entry("ldo4_voltage", S_IFREG | S_IRWXU, proc_root_fp);
	if(proc_voltage_fp) {
		proc_voltage_fp->data = proc_ldo4_voltage_str;
		proc_voltage_fp->read_proc = read_ldo4_proc_voltage;
		proc_voltage_fp->write_proc = write_ldo4_proc_voltage;
	}
	
	proc_voltage_fp = create_proc_entry("ldo_mem_voltage", S_IFREG | S_IRWXU, proc_root_fp);
	if(proc_voltage_fp) {
		proc_voltage_fp->data = proc_ldo_mem_voltage_str;
		proc_voltage_fp->read_proc = read_ldo_mem_proc_voltage;
		proc_voltage_fp->write_proc = write_ldo_mem_proc_voltage;
	}
	
	proc_voltage_fp = create_proc_entry("buck1_1_voltage", S_IFREG | S_IRWXU, proc_root_fp);
	if(proc_voltage_fp) {
		proc_voltage_fp->data = proc_buck1_1_voltage_str;
		proc_voltage_fp->read_proc = read_buck1_1_proc_voltage;
		proc_voltage_fp->write_proc = write_buck1_1_proc_voltage;
	}

	proc_voltage_fp = create_proc_entry("buck1_2_voltage", S_IFREG | S_IRWXU, proc_root_fp);
	if(proc_voltage_fp) {
		proc_voltage_fp->data = proc_buck1_2_voltage_str;
		proc_voltage_fp->read_proc = read_buck1_2_proc_voltage;
		proc_voltage_fp->write_proc = write_buck1_2_proc_voltage;
	}

	proc_voltage_fp = create_proc_entry("buck2_1_voltage", S_IFREG | S_IRWXU, proc_root_fp);
	if(proc_voltage_fp) {
		proc_voltage_fp->data = proc_buck2_1_voltage_str;
		proc_voltage_fp->read_proc = read_buck2_1_proc_voltage;
		proc_voltage_fp->write_proc = write_buck2_1_proc_voltage;
	}

	proc_voltage_fp = create_proc_entry("buck2_2_voltage", S_IFREG | S_IRWXU, proc_root_fp);
	if(proc_voltage_fp) {
		proc_voltage_fp->data = proc_buck2_2_voltage_str;
		proc_voltage_fp->read_proc = read_buck2_2_proc_voltage;
		proc_voltage_fp->write_proc = write_buck2_2_proc_voltage;
	}
	return 0;
}

void s5m8751_proc_exit(void)
{
	remove_proc_entry("ldo_audio_voltage", proc_root_fp);

	remove_proc_entry("ldo1_voltage", proc_root_fp);
	remove_proc_entry("ldo2_voltage", proc_root_fp);
	remove_proc_entry("ldo3_voltage", proc_root_fp);
	remove_proc_entry("ldo4_voltage", proc_root_fp);
	remove_proc_entry("ldo_mem_voltage", proc_root_fp);
	remove_proc_entry("buck1_1_voltage", proc_root_fp);
	remove_proc_entry("buck1_2_voltage", proc_root_fp);
	remove_proc_entry("buck2_1_voltage", proc_root_fp);
	remove_proc_entry("buck2_2_voltage", proc_root_fp);

	remove_proc_entry("s5m8751", 0);
}

module_init(s5m8751_proc_init);
module_exit(s5m8751_proc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("S5P6440-S5M8751 pmic test system");
