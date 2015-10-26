/*
 *  Copyright (C) 2008 by TomTom International BV. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation.
 *
 *
 * ** See arch/arm/plat-tomtom/include/plat/dock.h for documentation **
 *
 */

#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/vgpio.h>
#include <linux/gpio.h>

#include <plat/tt_setup_handler.h>
#include <plat/irvine.h>
#include <plat/dock.h>

#include <mach/pinmux.h>

/* All BCM4760 plats have these in common so far */
#define BCM4760_DOCK_REGULATOR	"LDO1"
#define BCM4760_DOCK_I2C_BUS	    1
#define dock_det_state()	    1 /* Dock det pullup in Keep Alive domain */

struct bcm4760_dock_drv_data 
{
	struct regulator *reg;
	int reg_enabled;
};

/* Be careful, non-atomic */
static inline int _enable_reg(struct bcm4760_dock_drv_data *d)
{
	int r=0;
	if (!d->reg_enabled && (r = regulator_enable(d->reg)) == 0) {
		d->reg_enabled = 1;
	}
	return r;
}

/* Be careful, non-atomic */
static inline int _disable_reg(struct bcm4760_dock_drv_data *d)
{
	int r=0;
	if (d->reg_enabled && (r = regulator_disable(d->reg)) == 0) {
		d->reg_enabled = 0;
	}
	return r;
}

static int bcm4760_dock_mach_init(struct dock_platform_data *pd)
{
	int err = 0;
	struct bcm4760_dock_drv_data *d = NULL;

	pd->priv = kzalloc(sizeof(struct bcm4760_dock_drv_data), 0);
	if (pd->priv == NULL) {
		return -ENOMEM;
	}
	d = (struct bcm4760_dock_drv_data *) pd->priv;

	d->reg = regulator_get(NULL, BCM4760_DOCK_REGULATOR);
	if (IS_ERR(d->reg)) {
		return PTR_ERR(d->reg);
	}

	if (gpio_is_valid(TT_VGPIO_DOCK_RESET)) {
		if ((err = gpio_request(TT_VGPIO_DOCK_RESET, "TT_VGPIO_DOCK_RESET"))) {
			printk("%s: Can't request TT_VGPIO_DOCK_RESET\n", __func__);
			goto err_free1;
		}
	}

	if (gpio_is_valid(TT_VGPIO_DOCK_DET0)) {
		if ((err = gpio_request(TT_VGPIO_DOCK_DET0, "TT_VGPIO_DOCK_DET0"))) {
			printk("%s: Can't request TT_VGPIO_DOCK_DET0\n", __func__);
			goto err_free4;
		}
	}

	gpio_direction_input (TT_VGPIO_DOCK_DET0);

	if (gpio_is_valid(TT_VGPIO_DOCK_DET1)) {
		if ((err = gpio_request(TT_VGPIO_DOCK_DET1, "TT_VGPIO_DOCK_DET1"))) {
			printk("%s: Can't request TT_VGPIO_DOCK_DET1\n", __func__);
			goto err_free5;
		}
	}

	gpio_direction_input (TT_VGPIO_DOCK_DET1);

	if (gpio_is_valid(TT_VGPIO_DOCK_MUTE)) {
		if ((err = gpio_request(TT_VGPIO_DOCK_MUTE, "TT_VGPIO_DOCK_MUTE"))) {
			printk("%s: Can't request TT_VGPIO_DOCK_MUTE\n", __func__);
			goto err_free6;
		}
	}

	gpio_direction_output (TT_VGPIO_DOCK_MUTE,0);

	return 0;

err_free6:
	gpio_free(TT_VGPIO_DOCK_MUTE);
err_free5:
	gpio_free(TT_VGPIO_DOCK_DET0);
err_free4:
	gpio_free(TT_VGPIO_DOCK_RESET);
err_free1:
	return err;
}

static void bcm4760_dock_mach_release(struct dock_platform_data *pd)
{
	struct bcm4760_dock_drv_data *d =
		(struct bcm4760_dock_drv_data *) pd->priv;

	_disable_reg(d); /* Should already be disabled here */
	regulator_put(d->reg);
	kfree(pd->priv);

	gpio_free(TT_VGPIO_DOCK_DET1);
	gpio_free(TT_VGPIO_DOCK_DET0);
	gpio_free(TT_VGPIO_DOCK_RESET);
	gpio_free(TT_VGPIO_DOCK_MUTE);
}

static int bcm4760_dock_line_config(struct dock_platform_data *pd, dock_connector_line_t line, int default_value, dock_line_flag_t flags)
{
	int ret = 0;

	switch(line) {
	case RDS_nRST:
		BUG_ON(flags & (DOCK_LINE_INPUT | DOCK_LINE_INTERRUPT));
		ret = gpio_direction_output(TT_VGPIO_DOCK_RESET, default_value);
		break;
	case DOCK_DET0:
	case DOCK_DET1:
		BUG_ON(flags & DOCK_LINE_OUTPUT);
		ret = gpio_direction_input((line == DOCK_DET1) ? TT_VGPIO_DOCK_DET1 : TT_VGPIO_DOCK_DET0);
		break;
	case DOCK_I2C_PWR:
		WARN_ON(flags & (DOCK_LINE_INPUT | DOCK_LINE_INTERRUPT));
		ret = gpio_direction_output(TT_VGPIO_DOCK_I2C_PWR, default_value);
		break;
	case FM_INT:
		WARN_ON(flags & DOCK_LINE_OUTPUT);
		ret = gpio_direction_input(TT_VGPIO_DOCK_FM_INT);
//		ret = gpio_direction_output(TT_VGPIO_DOCK_FM_INT, 0);
		break;
	default:
		/* Ignore most lines */
		break;
	}

	return ret;
}

static int bcm4760_dock_line_get(struct dock_platform_data *pd, dock_connector_line_t line)
{
	int ret = 0;

	switch(line) {
	case DOCK_DET0:
		ret = gpio_get_value(TT_VGPIO_DOCK_DET0);
		break;
	case DOCK_DET1:
		ret = gpio_get_value(TT_VGPIO_DOCK_DET1);
		break;
	case DOCK_PWR_DET:
		ret = dock_det_state();
		break;
	case DOCK_I2C_PWR:
		ret = gpio_get_value(TT_VGPIO_DOCK_I2C_PWR);
		break;
	default:
		/* The driver logic asked for something that is not supported. WARN, because you don't want */
		/* stall the boot process for something as menial as the dock.                              */
		WARN_ON(1);
		break;
	}

	return ret;	
}

static int bcm4760_dock_line_set(struct dock_platform_data *pd, dock_connector_line_t line, int value)
{
	int ret = 0;
	struct bcm4760_dock_drv_data *d =
		(struct bcm4760_dock_drv_data *) pd->priv;

	BUG_ON(d == NULL);

	switch(line) {
	case RDS_nRST:
		gpio_set_value(TT_VGPIO_DOCK_RESET, value);
		break;
	case DC_OUT:
		if (value == DOCK_LINE_ASSERT) {
			ret = _enable_reg(d);
		} else {
			ret = _disable_reg(d);
		}
		break;
	case DOCK_PWR_DET:
		/* Ignore */
		break;
	case DOCK_I2C_PWR:
		gpio_set_value(TT_VGPIO_DOCK_I2C_PWR, value);
		break;
	default:
		printk("Pin number %d has no set implementation\n", line);
		WARN_ON(1);
		break;
	}

	return ret;
}

static int bcm4760_dock_set_mute(struct dock_platform_data *pd, int value)
{
	gpio_set_value(TT_VGPIO_DOCK_MUTE, value);
	return 0;
}

static int bcm4760_dock_get_mute(struct dock_platform_data *pd)
{
	return gpio_get_value(TT_VGPIO_DOCK_MUTE);
}

static int bcm4760_dock_line_get_irq(struct dock_platform_data *pd, dock_connector_line_t line)
{
	int ret = -1;

	switch(line) {
	case DOCK_DET0:
		ret	= gpio_to_irq(TT_VGPIO_DOCK_DET0);
		break;
	case DOCK_DET1:
		ret	= gpio_to_irq(TT_VGPIO_DOCK_DET1);
		break;
	case FM_INT:
		ret	= gpio_to_irq(TT_VGPIO_DOCK_FM_INT);
		break;
	default:
		break;
	}

	return ret;
}

/* List of all docks recognized by the platform */
static const char*supported_dock_list[] = {
	"DOCK_NONE",
	"DOCK_WINDSCR_NO_RDS",
	"DOCK_WINDSCR_RDS",
	"DOCK_INVALID"
};

static int bcm4760_present(const char *name, int id)
{
 int i;

	for (i=0; i<sizeof(supported_dock_list)/sizeof(const char*); i++) {
		if (!strcmp(supported_dock_list[i], name))
			return 1;
	}
	return 0;
}

static inline void dock_suspend (void)
{
	printk(KERN_INFO "TT_Dock: Suspend\n");
}

static inline void dock_resume (void)
{
	gpio_set_value(TT_VGPIO_DOCK_MUTE, 0);
	printk(KERN_INFO "TT_Dock: Resume\n");
}

struct resource dock_resources[] = 
{
	{
		.name = "dock_sense0",
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "dock_sense1",
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "dock_i2c",
		.flags = IORESOURCE_IRQ,
	},
};

static struct dock_platform_data dock_pdata = 
{
	.i2cbusnumber		= BCM4760_DOCK_I2C_BUS,

	.dock_machine_init	= bcm4760_dock_mach_init,
	.dock_machine_release	= bcm4760_dock_mach_release,

	.dock_line_config	= bcm4760_dock_line_config,
	.dock_line_set		= bcm4760_dock_line_set,
	.dock_set_mute		= bcm4760_dock_set_mute,
	.dock_get_mute		= bcm4760_dock_get_mute,
	.dock_line_get		= bcm4760_dock_line_get,
	.dock_line_get_irq	= bcm4760_dock_line_get_irq,
	.present		= bcm4760_present,

	.suspend		= dock_suspend,
	.resume			= dock_resume,

	.priv			= NULL,
};

static struct platform_device dock_pdev = 
{
	.name 		= "dock",
	.id 		= -1,
	.num_resources 	= ARRAY_SIZE(dock_resources),
	.resource 	= dock_resources,
	.dev = 
	{
		.platform_data = (void *) &dock_pdata,
	},
};

static int __init tt_dock_setup (char *str)
{
	printk ("Initializing Dock\n");

	BUG_ON(TT_VGPIO_DOCK_DET0 == 0);
	dock_resources[DOCK_IRQ_SENSE0].start = gpio_to_irq(TT_VGPIO_DOCK_DET0);
	dock_resources[DOCK_IRQ_SENSE0].end   = dock_resources[DOCK_IRQ_SENSE0].start;

	BUG_ON(TT_VGPIO_DOCK_DET1 == 0);
	dock_resources[DOCK_IRQ_SENSE1].start = gpio_to_irq(TT_VGPIO_DOCK_DET1);
	dock_resources[DOCK_IRQ_SENSE1].end   = dock_resources[DOCK_IRQ_SENSE1].start;

	/* Mux the GPIO for RDS_nRST */
	bcm4760_set_pin_mux (vgpio_to_gpio (TT_VGPIO_DOCK_RESET), BCM4760_PIN_MUX_GPIO);

	/* Mux the GPIO for DOCK_I2C_PWR */
	bcm4760_set_pin_mux (vgpio_to_gpio (TT_VGPIO_DOCK_I2C_PWR), BCM4760_PIN_MUX_GPIO);

	/* Mux the GPIO for FM_INT */
	bcm4760_set_pin_mux (vgpio_to_gpio (TT_VGPIO_DOCK_FM_INT), BCM4760_PIN_MUX_GPIO);
	bcm4760_set_pin_mux (vgpio_to_gpio (TT_VGPIO_DOCK_DET0), BCM4760_PIN_MUX_GPIO);
	bcm4760_set_pin_mux (vgpio_to_gpio (TT_VGPIO_DOCK_DET1), BCM4760_PIN_MUX_GPIO);
	bcm4760_set_pin_mux (vgpio_to_gpio (TT_VGPIO_DOCK_MUTE), BCM4760_PIN_MUX_GPIO);

	return platform_device_register(&dock_pdev);
};

TT_SETUP_CB(tt_dock_setup, "tomtom-bcm-dock");

