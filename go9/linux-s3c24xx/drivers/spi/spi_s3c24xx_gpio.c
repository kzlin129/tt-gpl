/* linux/drivers/spi/spi_s3c24xx_gpio.c
 *
 * Copyright (c) 2006 Ben Dooks
 * Copyright (c) 2006 Simtec Electronics
 *
 * S3C24XX GPIO based SPI driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <asm/arch/regs-gpio.h>
#include <asm/arch/spi-gpio.h>
#include <asm/arch/hardware.h>

#ifdef CONFIG_MACH_TOMTOMGO
#include <barcelona/gopins.h>
#endif /* CONFIG_MACH_TOMTOMGO */


struct s3c2410_spigpio {
	struct spi_bitbang		 bitbang;

	struct s3c2410_spigpio_info	*info;
	struct platform_device		*dev;
};


static inline struct s3c2410_spigpio *spidev_to_sg(struct spi_device *spi)
{
	return spi->controller_data;
}

#ifdef CONFIG_MACH_TOMTOMGO

void tomtom_spigpio_set_cs(struct spi_device *spi, int value, int pol){
        /* Note : pol must be active high! */
        if(! pol)
                printk( "tomtom_spi_set_cs(): WARNING pol is not active high ! Mach config error\n");
        switch( value ){
                case BITBANG_CS_INACTIVE:
                        IOP_Deactivate( spi->chip_select );
                        break;
                case BITBANG_CS_ACTIVE:
                        IOP_Activate( spi->chip_select );
                        break;

                default:
                        printk( "tomtom_spi_set_cs(): ILLEGAL VALUE for value !\n");
        }
}
EXPORT_SYMBOL(tomtom_spigpio_set_cs);


static inline void setsck(struct spi_device *dev, int on)
{
	struct s3c2410_spigpio *sg = spidev_to_sg(dev);
	if ( on )
		IOP_Activate( (gopin_t) sg->info->pin_clk );
	else
		IOP_Deactivate( (gopin_t) sg->info->pin_clk );
}

static inline void setmosi(struct spi_device *dev, int on)
{
	struct s3c2410_spigpio *sg = spidev_to_sg(dev);
	if ( on )
		IOP_Activate( (gopin_t) sg->info->pin_mosi );
	else
		IOP_Deactivate( (gopin_t) sg->info->pin_mosi );
}

static inline u32 getmiso(struct spi_device *dev)
{
	struct s3c2410_spigpio *sg = spidev_to_sg(dev);
	return IOP_GetInput( (gopin_t) sg->info->pin_miso );
}
# else /* CONFIG_MACH_TOMTOMGO */

static inline void setsck(struct spi_device *dev, int on)
{
	struct s3c2410_spigpio *sg = spidev_to_sg(dev);
	s3c2410_gpio_setpin(sg->info->pin_clk, on ? 1 : 0);
}

static inline void setmosi(struct spi_device *dev, int on)
{
	struct s3c2410_spigpio *sg = spidev_to_sg(dev);
	s3c2410_gpio_setpin(sg->info->pin_mosi, on ? 1 : 0);
}

static inline u32 getmiso(struct spi_device *dev)
{
	struct s3c2410_spigpio *sg = spidev_to_sg(dev);
	return s3c2410_gpio_getpin(sg->info->pin_miso) ? 1 : 0;
}

#endif /* CONFIG_MACH_TOMTOMGO */

#define spidelay(x) ndelay(x)

#define	EXPAND_BITBANG_TXRX
#include <linux/spi/spi_bitbang.h>


static u32 s3c2410_spigpio_txrx_mode0(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, word, bits);
}

static u32 s3c2410_spigpio_txrx_mode1(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 0, word, bits);
}

static u32 s3c2410_spigpio_txrx_mode3(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 1, word, bits);
}

static void s3c2410_spigpio_chipselect(struct spi_device *dev, int value)
{
	struct s3c2410_spigpio *sg = spidev_to_sg(dev);

	if (sg->info && sg->info->chip_select)
#ifdef CONFIG_MACH_TOMTOMGO
		(sg->info->chip_select)(dev, value, (dev->mode&SPI_CPOL? 1 : 0) );
#else
		(sg->info->chip_select)(sg->info, value);
#endif /* MACH_TOMTOMGO */
}

static int s3c2410_spigpio_probe(struct device *dev)
{
	struct spi_master	*master;
	struct s3c2410_spigpio  *sp;
	int ret;
	int i;

	printk( "S3C2410 GPIO SPI Driver\n" );

	master = spi_alloc_master( dev, sizeof(struct s3c2410_spigpio));
	if (master == NULL) {
		dev_err( dev, "failed to allocate spi master\n");
		ret = -ENOMEM;
		goto err;
	}

	sp = spi_master_get_devdata(master);

	dev_set_drvdata( dev, sp );

	/* copy in the plkatform data */
	sp->info = dev->platform_data;

	/* setup spi bitbang adaptor */
	sp->bitbang.master = spi_master_get(master);
	sp->bitbang.chipselect = s3c2410_spigpio_chipselect;

	sp->bitbang.txrx_word[SPI_MODE_0] = s3c2410_spigpio_txrx_mode0;
	sp->bitbang.txrx_word[SPI_MODE_1] = s3c2410_spigpio_txrx_mode1;
	sp->bitbang.txrx_word[SPI_MODE_3] = s3c2410_spigpio_txrx_mode3;

#ifdef CONFIG_MACH_TOMTOMGO
	/* set state of spi pins */

	/* NOTE: on TomTom GO mach, these fields contain gopin_t */
	IOP_Deactivate( (gopin_t) sp->info->pin_clk );
	IOP_Deactivate( (gopin_t) sp->info->pin_mosi );

	IOP_SetFunction( (gopin_t) sp->info->pin_clk );
	IOP_SetFunction( (gopin_t) sp->info->pin_mosi );
	IOP_SetInput( (gopin_t) sp->info->pin_miso );
#else
	/* set state of spi pins */
	s3c2410_gpio_setpin(sp->info->pin_clk, 0);
	s3c2410_gpio_setpin(sp->info->pin_mosi, 0);

	s3c2410_gpio_cfgpin(sp->info->pin_clk, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_cfgpin(sp->info->pin_mosi, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_cfgpin(sp->info->pin_miso, S3C2410_GPIO_INPUT);
#endif /* CONFIG_MACH_TOMTOMGO  */

	ret = spi_bitbang_start(&sp->bitbang);
	if (ret)
		goto err_no_bitbang;

	/* register the chips to go with the board */

	for (i = 0; i < sp->info->board_size; i++) {
		//dev_info(dev, "registering %p: %s\n",
		printk("registering %p: %s\n",
			 &sp->info->board_info[i],
			 sp->info->board_info[i].modalias);

		sp->info->board_info[i].controller_data = sp;
		spi_new_device(master, sp->info->board_info + i);
	}

	return 0;

 err_no_bitbang:
	spi_master_put(sp->bitbang.master);
 err:
	printk("THERE WAS AN ERRROR WITH s3c2410_spigpio_probe()!");

	return ret;

}

static int s3c2410_spigpio_remove(struct device *dev)
{
	struct s3c2410_spigpio *sp = dev_get_drvdata(dev);

	spi_bitbang_stop(&sp->bitbang);
	spi_master_put(sp->bitbang.master);

	return 0;
}

/* all gpio should be held over suspend/resume, so we should
 * not need to deal with this
*/

#define s3c2410_spigpio_suspend NULL
#define s3c2410_spigpio_resume NULL


static struct device_driver s3c2410_spigpio_drv = {
	.probe		= s3c2410_spigpio_probe,
        .remove		= s3c2410_spigpio_remove,
        .suspend	= s3c2410_spigpio_suspend,
        .resume		= s3c2410_spigpio_resume,
	.bus            = &platform_bus_type,
   	.name		= "s3c24xx-spi-gpio",
	.owner		= THIS_MODULE,
};

static int __init s3c2410_spigpio_init(void)
{
	int result;
	result = driver_register(&s3c2410_spigpio_drv);
	return result;
}

static void __exit s3c2410_spigpio_exit(void)
{
        driver_unregister(&s3c2410_spigpio_drv);
}

module_init(s3c2410_spigpio_init);
module_exit(s3c2410_spigpio_exit);

MODULE_DESCRIPTION("S3C24XX SPI GPIO Driver");
MODULE_AUTHOR("Ben Dooks, <ben@simtec.co.uk>");
MODULE_LICENSE("GPL");
