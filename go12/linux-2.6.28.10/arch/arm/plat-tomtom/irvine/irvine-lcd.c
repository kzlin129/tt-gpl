#include <linux/device.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/vgpio.h>

#include <linux/amba/bus.h>
#include <linux/amba/bcmring_clcd.h>

#include <plat/fdt.h>
#include <plat/clcd.h>
#include <plat/irvine.h>
#include <plat/irvine-lcd.h>
#include <mach/pinmux.h>

#define BCM476X_CLCD_DEFAULT		0
#define BCM476X_CLCD_SND_SOURCE		1

static LIST_HEAD(clcd_list);

static inline int bcm476x_clcd_type(void)
{
	int res = BCM476X_CLCD_DEFAULT;

	if (gpio_request(TT_VGPIO_LCD_ID, "LCD_ID"))
	{
		pr_err("Can't request LCD_ID\n");
		goto err_req;
	}

	if (gpio_direction_input (TT_VGPIO_LCD_ID))
	{
		pr_err("Can't request LCD_ID direction input\n");
		goto err_dir;
	}

	if (gpio_get_value(TT_VGPIO_LCD_ID))
		res = BCM476X_CLCD_SND_SOURCE;

err_dir:
	gpio_free(TT_VGPIO_LCD_ID);

err_req:
	return res;
}

struct clcd_device *irvine_clcd_lookup(int index)
{
	struct clcd_paneldev *dev = NULL;
	struct clcd_paneldev *tmp;
	const char *screen;

	screen = fdt_get_string ("/features", "tft", "LMS430");

	/* Choose default panel */
	list_for_each_entry(tmp, &clcd_list, list)
	{
		if (!strcmp(tmp->panel->timings->mode.name, screen))
		{
			dev = tmp;
			break;
		}
	}
	
	if (dev == NULL) {
		pr_err("There is no default panel defined\n");
		return NULL;
	}

	if (screen && ( bcm476x_clcd_type() == BCM476X_CLCD_SND_SOURCE) )
	{
		struct clcd_paneldev *fdtdev;

		list_for_each_entry(fdtdev, &clcd_list, list)
		{
			if (!strcmp( fdtdev->panel->timings->mode.name, screen ))
			{
				dev = fdtdev;
				break;
			}
		}
	}

	writel((readl(IO_ADDRESS(CMU_R_PLA_PLL_CTL6_MEMADDR)) & ~CMU_F_PLA_M5DIV_MASK) | dev->lcd_mdiv,
		IO_ADDRESS(CMU_R_PLA_PLL_CTL6_MEMADDR));

	return dev->panel;
}

void irvine_clcd_register (struct clcd_paneldev *dev)
{
	list_add_tail(&dev->list, &clcd_list);
}

void irvine_clcd_unregister (struct clcd_paneldev *dev)
{
	list_del(&dev->list);
}

static int __init irvine_lcd_init (void)
{
	bcm476x_amba_lcd_init();
	bcm476x_clcd_lookup_register (irvine_clcd_lookup);

	return 0;
}
arch_initcall(irvine_lcd_init);
