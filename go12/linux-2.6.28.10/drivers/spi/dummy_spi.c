#include <linux/device.h>
#include <linux/spi/spi.h>

static int txrx = 3;
static int tsize = 10;
static int busmask = 3;

static void dummy_spi_output(struct spi_device *spi)
{
        struct spi_transfer     t = { };
        struct spi_message      m;
	int status, i;

	t.len = tsize;
	if (txrx & 2)
		t.tx_buf = kmalloc(4096, GFP_KERNEL);
	if (txrx & 1)
		t.rx_buf = kmalloc(4096, GFP_KERNEL);

	dev_info(&spi->dev, "dumping %d bytes %p %p\n", tsize, t.tx_buf, t.rx_buf);
        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);
	dev_info(&spi->dev, "status = %d\ndatain = ", status);
	if (t.rx_buf)
		for (i = 0; i < tsize; i++)
			printk("%d ", ((char *)t.rx_buf)[i]);
	printk("\ndataout = ");
	if (t.tx_buf)
		for (i = 0; i < tsize; i++)
			printk("%d ", ((char *)t.tx_buf)[i]);
	printk("\n");
	kfree(t.tx_buf);
	kfree(t.rx_buf);
}


static int dummy_spi_probe(struct spi_device *spi)
{
	if (busmask & (1 << spi->master->bus_num))
		dummy_spi_output(spi);
	return 0;
}

static int dummy_spi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver dummy_spi_driver = {
	.driver = {
		.name		= "dummy_spi",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},
	.probe			= dummy_spi_probe,
	.remove			= dummy_spi_remove,
};

static int __init dummy_spi_init(void)
{
	return spi_register_driver(&dummy_spi_driver);
}

static void __exit dummy_spi_exit(void)
{
	spi_unregister_driver(&dummy_spi_driver);
}

module_param(txrx, int, 0444);
module_param(tsize, int, 0444);
module_param(busmask, int, 0444);

module_init(dummy_spi_init);
module_exit(dummy_spi_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("dummy SPI driver");
