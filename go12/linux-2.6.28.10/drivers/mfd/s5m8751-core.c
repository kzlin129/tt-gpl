/*
 * s5m8751-core.c  --  S5M8751 Power-Audio IC ALSA Soc Audio driver
 *
 * Copyright 2009 Samsung Electronics.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/bug.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/mfd/s5m8751/s5m8751_core.h>
#include <linux/mfd/s5m8751/s5m8751_audio.h>
#include <linux/mfd/s5m8751/s5m8751_pmic.h>
#include <linux/mfd/s5m8751/s5m8751_backlight.h>

static DEFINE_MUTEX(io_mutex);
static DEFINE_MUTEX(reg_lock_mutex);

static int s5m8751_read(struct s5m8751 *s5m8751, u8 reg, int num_regs, u8 *dest)
{

	if (s5m8751->read_dev == NULL)
		return -ENODEV;

	if ((reg + num_regs - 1) > S5M8751_MAX_REGISTER) {
		dev_err(s5m8751->dev, "invalid reg %x\n", reg + num_regs - 1);
		return -EINVAL;
	}
	s5m8751->read_dev(s5m8751, reg, sizeof(reg), (char *)dest);
//	dev_dbg(s5m8751->dev, "%s R%d(0x%2.2x) %d regs\n", __func__, reg, reg, num_regs);

	return 0;	
}

static int s5m8751_write(struct s5m8751 *s5m8751, u8 reg, int num_regs, u8 *src)
{
	int byte = num_regs;
	
	if (s5m8751->write_dev == NULL)
		return -ENODEV;

	if ((reg + num_regs - 1) > S5M8751_MAX_REGISTER) {
		dev_err(s5m8751->dev, "invalid reg %x\n", reg + num_regs - 1);
		return -EINVAL;
	}

	return s5m8751->write_dev(s5m8751, reg, byte, (char *)src);
}

int s5m8751_clear_bits(struct s5m8751 *s5m8751, u8 reg, u8 mask)
{
	u8 data;
	int err;

	err = s5m8751_read(s5m8751, reg, 1, &data);
	if (err) {
		dev_err(s5m8751->dev, "read from reg R%d failed\n", reg);
		goto out;
	}

	data &= ~mask;
	err = s5m8751_write(s5m8751, reg, 1, &data);
	if (err)
		dev_err(s5m8751->dev, "write to reg R%d failed\n", reg);

out:
	return err;
}
EXPORT_SYMBOL_GPL(s5m8751_clear_bits);

int s5m8751_set_bits(struct s5m8751 *s5m8751, u8 reg, u8 mask)
{
	u8 data;
	int err;

	err = s5m8751_read(s5m8751, reg, 1, &data);
	if (err) {
		dev_err(s5m8751->dev, "read from reg R%d failed\n", reg);
		goto out;
	}

	data |= mask;
	err = s5m8751_write(s5m8751, reg, 1, &data);
	if (err)
		dev_err(s5m8751->dev, "write to reg R%d failed\n", reg);

out:
	return err;
}
EXPORT_SYMBOL_GPL(s5m8751_set_bits);

u8 s5m8751_reg_read(struct s5m8751 *s5m8751, int reg)
{
	u8 data;
	int err;

	mutex_lock(&io_mutex);
	err = s5m8751_read(s5m8751, reg, 1, &data);
	if (err)
		dev_err(s5m8751->dev, "read from reg R%d failed\n", reg);
	mutex_unlock(&io_mutex);
	return data;
}
EXPORT_SYMBOL_GPL(s5m8751_reg_read);

int s5m8751_reg_write(struct s5m8751 *s5m8751, int reg, u8 val)
{
	int ret;
	u8 data = val;

	mutex_lock(&io_mutex);
	ret = s5m8751_write(s5m8751, reg, 1, &data);
	if (ret)
		dev_err(s5m8751->dev, "write to reg R%d failed\n", reg);
	mutex_unlock(&io_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(s5m8751_reg_write);


int s5m8751_mask_irq(struct s5m8751 *s5m8751, int irq)
{
	switch (irq) {
		case S5M8751_IRQ_PWRKEY1B:
			return s5m8751_clear_bits(s5m8751, S5M8751_IRQB_MASK1, S5M8751_MASK_PWRKEY1B);
		
		case S5M8751_IRQ_PWRKEY2B:
			return s5m8751_clear_bits(s5m8751, S5M8751_IRQB_MASK1, S5M8751_MASK_PWRKEY2B);
 
		case S5M8751_IRQ_PWRKEY3:
			return s5m8751_clear_bits(s5m8751, S5M8751_IRQB_MASK1, S5M8751_MASK_PWRKEY3);
		
		case S5M8751_IRQ_PWRKEY4:
			return s5m8751_clear_bits(s5m8751, S5M8751_IRQB_MASK1, S5M8751_MASK_PWRKEY4);

		case S5M8751_IRQ_CHARGER_REMOVAL:
			return s5m8751_clear_bits(s5m8751, S5M8751_IRQB_MASK2, S5M8751_MASK_VCHG_DET);
		
		case S5M8751_IRQ_USB_DEVICE_REMOVAL:
			return s5m8751_clear_bits(s5m8751, S5M8751_IRQB_MASK2, S5M8751_MASK_VCHG_REM);

		case S5M8751_IRQ_CHARGER_TIMEOUT:
			return s5m8751_clear_bits(s5m8751, S5M8751_IRQB_MASK2, S5M8751_MASK_CHG_T_OUT);
		
		case S5M8751_IRQ_BATTERY_DETECTION:
			return s5m8751_clear_bits(s5m8751, S5M8751_IRQB_MASK2, S5M8751_MASK_CHG_BATT_DET);

		case S5M8751_IRQ_CHARGE_COMPLETION:
			return s5m8751_clear_bits(s5m8751, S5M8751_IRQB_MASK2, S5M8751_CHG_EOC);

		default:
			dev_warn(s5m8751->dev, "Attempting to mask unknown IRQ %d\n", irq);
			return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(s5m8751_mask_irq);


int s5m8751_unmask_irq(struct s5m8751 *s5m8751, int irq)
{
	switch (irq) {
		case S5M8751_IRQ_PWRKEY1B:
			return s5m8751_set_bits(s5m8751, S5M8751_IRQB_MASK1, S5M8751_MASK_PWRKEY1B);
		
		case S5M8751_IRQ_PWRKEY2B:
			return s5m8751_set_bits(s5m8751, S5M8751_IRQB_MASK1, S5M8751_MASK_PWRKEY2B);
 
		case S5M8751_IRQ_PWRKEY3:
			return s5m8751_set_bits(s5m8751, S5M8751_IRQB_MASK1, S5M8751_MASK_PWRKEY3);
		
		case S5M8751_IRQ_PWRKEY4:
			return s5m8751_set_bits(s5m8751, S5M8751_IRQB_MASK1, S5M8751_MASK_PWRKEY4);

		case S5M8751_IRQ_CHARGER_REMOVAL:
			return s5m8751_set_bits(s5m8751, S5M8751_IRQB_MASK2, S5M8751_MASK_VCHG_DET);
		
		case S5M8751_IRQ_USB_DEVICE_REMOVAL:
			return s5m8751_set_bits(s5m8751, S5M8751_IRQB_MASK2, S5M8751_MASK_VCHG_REM);

		case S5M8751_IRQ_CHARGER_TIMEOUT:
			return s5m8751_set_bits(s5m8751, S5M8751_IRQB_MASK2, S5M8751_MASK_CHG_T_OUT);
		
		case S5M8751_IRQ_BATTERY_DETECTION:
			return s5m8751_set_bits(s5m8751, S5M8751_IRQB_MASK2, S5M8751_MASK_CHG_BATT_DET);

		case S5M8751_IRQ_CHARGE_COMPLETION:
			return s5m8751_set_bits(s5m8751, S5M8751_IRQB_MASK2, S5M8751_CHG_EOC);

		default:
			dev_warn(s5m8751->dev, "Attempting to unmask unknown IRQ %d\n", irq);
			return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(s5m8751_unmask_irq);

int s5m8751_client_register(struct s5m8751 *s5m8751, struct platform_device *pdev)
{
	int ret;

	if (!pdev)
		return -ENODEV;

	pdev->dev.parent = s5m8751->dev;

	if ((ret = platform_device_register(pdev)))
		dev_err(s5m8751->dev, "Failed to register %s: %d\n", pdev->name, ret);

	return ret;
}
EXPORT_SYMBOL(s5m8751_client_register);


void s5m8751_device_exit(struct s5m8751 *s5m8751)
{
	free_irq(s5m8751->chip_irq, s5m8751);
	kfree(s5m8751->reg_cache);
}
EXPORT_SYMBOL_GPL(s5m8751_device_exit);

MODULE_DESCRIPTION("S5M8751 Power-Audio IC driver");
MODULE_LICENSE("GPL");	

