/* linux/drivers/spi/spi_s3c24xx.c
 *
 * Copyright (c) 2006 Ben Dooks
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/


//#define DEBUG
#define HERE()	{ printk("%s line %d says: HERE : %s\n", __FILE__, __LINE__, __FUNCTION__ ); }
#define TELL(X)	{ printk("%s line %d says: %s = %d ( 0x%02x) \n", __FILE__, __LINE__, #X, (int) (X), (int) (X)); }
#define YELL(X)	{ printk("%s line %d says: %s = \"%s\" \n", __FILE__, __LINE__, #X, X ); }
#define SAY(X)	{ printk("%s line %d says: \"%s\" \n", __FILE__, __LINE__, X ); }

#include <linux/config.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <asm/hardware/clock.h>
#include <linux/device.h>
//#include <linux/platform_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <asm/io.h>
#include <asm/dma.h>
#include <asm/hardware.h>

#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-spi.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/spi.h>

#ifdef CONFIG_MACH_TOMTOMGO
#include <barcelona/gopins.h>
#endif /* CONFIG_MACH_TOMTOMGO */

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
#include <linux/notifier.h>
#include <linux/cpufreq.h>
#include <barcelona/cpufreq_order.h>
#include <asm/io.h>
#endif

struct s3c24xx_spi {
	/* bitbang has to be first */
	struct spi_bitbang	 bitbang;
	struct completion	 done;

	void __iomem		*regs;
	int			 irq;
	int			 len;
	int			 nsend;
	int			 nrecv;

	/* these are needed for suspend/resume procedure */
	u8       sppin_save;
	u8       sppre_save;
	u16      spfic_save;
	u32      spcon_save;

	/* data buffers */
	const unsigned char	*tx;
	unsigned char		*rx;

	struct clk		*clk;
	struct resource		*ioarea;
	struct spi_master	*master;
	struct spi_device	*curdev;
	struct device		*dev;
	struct s3c2410_spi_info *pdata;
#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	struct notifier_block   freq_policy;
	struct notifier_block   freq_transition;
#endif
};

#define TXFIFO_TIMEOUT 1

/* Borrowed fromo linux 2.6.19 <asm/arch/regs-gpio.h>	*/
#define S3C2410_GPIO_OUTPUT  (0xFFFFFFF1)	

#define SPCON_DEFAULT (S3C2410_SPCON_MSTR     | S3C2410_SPCON_SMOD_INT | \
                       S3C2410_SPCON_RXFIFOEN | S3C2410_SPCON_TXFIFOEN )
#define SPFIC_DEFAULT (S3C2410_SPFIC_RXFIFONEMIE )
#define SPPIN_DEFAULT (S3C2410_SPPIN_KEEP)

#ifdef CONFIG_MACH_TOMTOMGO

#ifdef DEBUG
static void debug_spi_transfer(struct s3c24xx_spi *hw)
{
	unsigned char buffer[128];
	unsigned int ret = 0;
	unsigned int index;
	
	ret += snprintf(buffer+ret, sizeof(buffer)-ret-1, "tx: ");
	if (hw->tx)
	{
		for (index = 0; index < hw->len; index++) {
			ret += snprintf(buffer+ret, sizeof(buffer)-ret-1, "%02x ", hw->tx[index]);
		}
		if (hw->len == 1)
		{
		  ret += snprintf(buffer+ret, sizeof(buffer)-ret-1, "   ");
		}
	}
	else
	{
		ret += snprintf(buffer+ret, sizeof(buffer)-ret-1, "      ");
	}

	ret += snprintf(buffer+ret, sizeof(buffer)-ret-1, "rx: ");
	if (hw->rx)
	{
		for (index = 0; index < hw->len; index++) 		{
			ret += snprintf(buffer+ret, sizeof(buffer)-ret-1, "%02x ", hw->rx[index]);
		}
	}

	buffer[ret] = '\0';
	printk(KERN_DEBUG "SPI transfer: %s\n", buffer);
}

static void spi_show_reg( const char* name, void* addr, u32 mask ){
	u32 reg = ioread32( addr );
	printk( "%s = 0x%08x\n", name, reg & mask ); 
}

static void spi_dump_registers( struct s3c24xx_spi *hw ){
	spi_show_reg( "S3C2410_SPCON", hw->regs + S3C2410_SPCON , -1 );
	spi_show_reg( "S3C2410_SPSTA", hw->regs + S3C2410_SPSTA, -1 );
	spi_show_reg( "S3C2410_SPFIC", hw->regs + S3C2410_SPFIC, -1 );
	spi_show_reg( "S3C2410_SPTOV", hw->regs + S3C2410_SPTOV, -1 );
	spi_show_reg( "S3C2410_SPPIN", hw->regs + S3C2410_SPPIN, -1 );
	spi_show_reg( "S3C2410_SPPRE", hw->regs + S3C2410_SPPRE, -1 );
}
#endif /* DEBUG */


void tomtom_spi_set_cs(struct spi_device *spi, int value, int pol){
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
EXPORT_SYMBOL(tomtom_spi_set_cs);


static void tomtom_spi_hw_init( void ){
	IO_SetFunction( SPIMSI );
	IO_SetFunction( SPIMSO );
	IO_SetFunction( SPICLK );
}

static void tomtom_spi_hw_exit( void ){
	IO_SetInput( SPIMSI );
	IO_SetInput( SPIMSO );
	IO_SetInput( SPICLK );
}

#endif /* CONFIG_MACH_TOMTOMGO */

static inline struct s3c24xx_spi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static void s3c24xx_spi_chipsel(struct spi_device *spi, int value)
{
	struct s3c24xx_spi *hw = to_hw(spi);
	u32 cspol = spi->mode & SPI_CS_HIGH ? 1 : 0;
	u32 spcon;

	switch (value) {
	case BITBANG_CS_INACTIVE:
#ifdef CONFIG_MACH_TOMTOMGO
		if (hw->pdata && hw->pdata->chip_select)
			(hw->pdata->chip_select)(spi, value, cspol );
#else
		if (hw->pdata && hw->pdata->set_cs)
			//hw->pdata->set_cs(hw->pdata, value, cspol);
			hw->pdata->set_cs(spi, value, cspol);
		else
			s3c2410_gpio_setpin(hw->pdata->pin_cs, cspol ^ 1);
#endif /* CONFIG_MACH_TOMTOMGO */
		break;

	case BITBANG_CS_ACTIVE:
		spcon = ioread32(hw->regs + S3C2410_SPCON);

		if (spi->mode & SPI_CPHA)
			spcon |= S3C2410_SPCON_CPHA_FMTB;
		else
			spcon &= ~S3C2410_SPCON_CPHA_FMTB;

		if (spi->mode & SPI_CPOL)
			spcon |= S3C2410_SPCON_CPOL_HIGH;
		else
			spcon &= ~S3C2410_SPCON_CPOL_HIGH;

		spcon |= S3C2410_SPCON_ENSCK;

		/* write new configration */
		iowrite32(spcon, hw->regs + S3C2410_SPCON);

#ifdef CONFIG_MACH_TOMTOMGO
		if (hw->pdata && hw->pdata->chip_select)
			(hw->pdata->chip_select)(spi, value, cspol );
#else
		if (hw->pdata->set_cs)
			//hw->pdata->set_cs(hw->pdata, value, cspol);
			hw->pdata->set_cs(spi, value, cspol);
		else
			s3c2410_gpio_setpin(hw->pdata->pin_cs, cspol);
#endif /* CONFIG_MACH_TOMTOMGO */
		break;

	}
}

static int s3c24xx_spi_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct s3c24xx_spi *hw = to_hw(spi);
	unsigned int bpw;
	unsigned int hz;
	unsigned int div;

	bpw = t ? t->bits_per_word : spi->bits_per_word;
	hz  = t ? t->speed_hz : spi->max_speed_hz;

	if (bpw != 8) {
		dev_err(&spi->dev, "invalid bits-per-word (%d)\n", bpw);
		return -EINVAL;
	}

	div = clk_get_rate(hw->clk) / hz;

	/* is clk = pclk / (2 * (pre+1)), or is it
	 *    clk = (pclk * 2) / ( pre + 1) */

	div = (div / 2) - 1;

	if (div < 0)
		div = 1;

	if (div > 255)
		div = 255;

	dev_dbg(&spi->dev, "setting pre-scaler to %d (hz %d)\n", div, hz);
	iowrite8(div, hw->regs + S3C2410_SPPRE);

	spin_lock(&hw->bitbang.lock);
	if (!hw->bitbang.busy) {
		hw->bitbang.chipselect(spi, BITBANG_CS_INACTIVE);
		/* need to ndelay for 0.5 clocktick ? */
	}
	spin_unlock(&hw->bitbang.lock);

	return 0;
}

static int s3c24xx_spi_setup(struct spi_device *spi)
{
	int ret;

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	if ((spi->mode & SPI_LSB_FIRST) != 0)
		return -EINVAL;

	ret = s3c24xx_spi_setupxfer(spi, NULL);
	if (ret < 0) {
		dev_err(&spi->dev, "setupxfer returned %d\n", ret);
		return ret;
	}

	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n",
		__FUNCTION__, spi->mode, spi->bits_per_word,
		spi->max_speed_hz);

	return 0;
}

static inline unsigned int hw_txbyte(struct s3c24xx_spi *hw, int count)
{
	return hw->tx ? hw->tx[count] : 0xff;
}

static void s3c24xx_spi_tx_fifo(struct s3c24xx_spi *hw)
{
	unsigned int spsta = readb(hw->regs + S3C2410_SPSTA);

	while((hw->nsend < hw->len) && (spsta & S3C2410_SPSTA_TXFIFONFULL))
	{
	  iowrite8(hw_txbyte(hw, hw->nsend), hw->regs + S3C2410_SPTXFIFO);
		hw->nsend++;
		spsta = readb(hw->regs + S3C2410_SPSTA);
	}
}

static void s3c24xx_spi_rx_fifo(struct s3c24xx_spi *hw)
{
	unsigned int spsta = readb(hw->regs + S3C2410_SPSTA);

	while((hw->nrecv < hw->len) && (spsta & S3C2410_SPSTA_RXFIFONEMPTY))
	{
		u8 recv = readb(hw->regs + S3C2410_SPRXFIFO);
		if (hw->rx) hw->rx[hw->nrecv] = recv;
			hw->nrecv++;
		spsta = readb(hw->regs + S3C2410_SPSTA);
	}
}

static int s3c24xx_spi_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	struct s3c24xx_spi *hw = to_hw(spi);
	unsigned int sent = 0;

	dev_dbg(&spi->dev, "txrx: tx %p, rx %p, len %d\n",
		t->tx_buf, t->rx_buf, t->len);

	hw->tx    = t->tx_buf;
	hw->rx    = t->rx_buf;
	hw->len   = t->len;
	hw->nsend = 1;
	hw->nrecv = 0;

#ifdef DEBUG
	spi_dump_registers(hw);
#endif /* DEBUG */

	while (!sent)
	{
		unsigned int spsta = readb(hw->regs + S3C2410_SPSTA);
		if (spsta & S3C2410_SPSTA_TXFIFONFULL)
		{
			iowrite8(hw_txbyte(hw, 0), hw->regs + S3C2410_SPTXFIFO);
			sent = 1;
		}
		else
			msleep(TXFIFO_TIMEOUT);
	}
	wait_for_completion(&hw->done);

#ifdef DEBUG
	debug_spi_transfer(hw);
#endif /* DEBUG */

	return hw->nrecv;
}

static irqreturn_t s3c24xx_spi_irq(int irq, void *dev, struct pt_regs *regs)
{
	struct s3c24xx_spi *hw = dev;
	unsigned int spsta = readb(hw->regs + S3C2410_SPSTA);

	if (spsta & S3C2410_SPSTA_DCOL) {
		dev_dbg(hw->dev, "data-collision\n");
		complete(&hw->done);
		goto irq_done;
	}

	if (!(spsta & S3C2410_SPSTA_READY)) {
		dev_dbg(hw->dev, "spi not ready for tx?\n");
		complete(&hw->done);
		goto irq_done;
	}

	/* We only support S3C2410_SPSTA_RXFIFONEMPTY */
	if (spsta & S3C2410_SPSTA_RXFIFONEMPTY)
	{
		s3c24xx_spi_rx_fifo(hw);

		/* Send more data if possible */
		if (hw->nsend < hw->len)
			s3c24xx_spi_tx_fifo(hw);

		if(hw->nrecv == hw->len)
			complete(&hw->done);
	}

irq_done:
	return IRQ_HANDLED;
}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
static int spi_freq_transition(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs		*f = data;
	struct s3c24xx_spi		*spi=(struct s3c24xx_spi *) container_of( nb, struct s3c24xx_spi, freq_policy );  
	__u32				pclk_spiclk_div_min=2;
	__u32				pclk_spiclk_div_max=512;
	__u32				pclk_old;
	__u32				pclk_new;
	__u32				pclk_min=pclk_spiclk_div_min * 1;
	__u32				pclk_max=pclk_spiclk_div_max * spi->curdev->max_speed_hz;
	unsigned long			sppre;

	/* Get the PCLK values. */
	f->trans2pclk( f, &pclk_old, &pclk_new );

	/* If the device ain't open, no need to worry. */
	switch (val)
	{
		case CPUFREQ_PRECHANGE:
		{
			/* Is this a transition from a valid to an invalid frequency? */
			if( (pclk_new > pclk_max) || (pclk_new < pclk_min) )
			{
				if( (pclk_old < pclk_max) || (pclk_old > pclk_min) )
				{
					/* Disable the clock. This prevents any transmissions. */
					clk_disable(spi->clk);
					tomtom_spi_hw_exit();
					printk( KERN_WARNING "spi_s3c24xx: SPI Clock divider invalid. Disabled spi clock.\n" );
				}
			}
			else
			{
				/* Legal frequency. */
				if( pclk_new > pclk_old )
				{
					/* Get the current iicon value. */
					sppre=ioread8( spi->regs + S3C2410_SPPRE );

					/* Need to set the divider now. */
					/* prenew=(preold + 1)*pclknew/pclkold -1 */
					sppre=(((sppre + 1)*pclk_new)/pclk_old) - 1;
					if( sppre > 255 )
					{
						printk( KERN_WARNING "spi_s3c24xx: Can't get right clock divider. Assuming maximum.\n" );
						sppre=255;
					}

					/* Set the new frequency. */
					iowrite8( sppre, spi->regs + S3C2410_SPPRE );
				}
			}
			break;
		}

		case CPUFREQ_POSTCHANGE:
		{
			/* Set the new divider if the new frequency is lower than the old frequency. */
			if( ((pclk_new < pclk_max) || (pclk_new > pclk_min)) && (pclk_new <= pclk_old) )
			{
				/* Get the current iicon value. */
				sppre=ioread8( spi->regs + S3C2410_SPPRE );

				/* Need to set the divider now. */
				/* prenew=(preold + 1)*pclknew/pclkold -1 */
				sppre=(((sppre + 1)*pclk_new)/pclk_old) - 1;
				if( sppre > 255 )
				{
					printk( KERN_WARNING "spi_s3c24xx: Can't get right clock divider. Assuming maximum.\n" );
					sppre=255;
				}

				/* Set the new frequency. */
				iowrite8( sppre, spi->regs + S3C2410_SPPRE );
			}

			/* Is this a transition from a valid to an invalid frequency? */
			if( (pclk_old > pclk_max) || (pclk_old < pclk_min) )
			{
				if( (pclk_new < pclk_max) || (pclk_new > pclk_min) )
				{
					/* Reenable the clock. Transmissions can continue. */
					clk_enable(spi->clk);
					tomtom_spi_hw_init();
					printk( KERN_WARNING "spi_s3c24xx: I2C Clock divider valid again. Re-enabled spi clock.\n" );
				}
			}
			break;
		}
	}
	return 0;
}

static int spi_freq_policy( struct notifier_block *nb, unsigned long val, void *data )
{
	struct cpufreq_policy		*policy = data;
	struct s3c24xx_spi		*spi=(struct s3c24xx_spi *) container_of( nb, struct s3c24xx_spi, freq_policy );  
	__u32				pclk_spiclk_div_max=512;
	__u32				pclk_spiclk_div_min=2;
	__u32				pclk_min=pclk_spiclk_div_min * 1;
	__u32				pclk_max=pclk_spiclk_div_max * spi->curdev->max_speed_hz;
	__u32				pclk_low;
	__u32				pclk_high;

	policy->policy2pclk( policy, &pclk_low, &pclk_high );
	
	switch (val) {
		case CPUFREQ_ADJUST:
			policy->pclk2policy( policy, pclk_min, pclk_max );
			break;
		case CPUFREQ_INCOMPATIBLE:
			if( (pclk_low >= pclk_min) || (pclk_low <= pclk_max) )
				pclk_min=pclk_low;

			if( (pclk_high >= pclk_min) || (pclk_high <= pclk_max) )
				pclk_max=pclk_high;

			if( (pclk_high != pclk_max) || (pclk_low != pclk_min) )
				policy->pclk2policy( policy, pclk_min, pclk_max );
			break;
		case CPUFREQ_NOTIFY:
			/* Illegal values are handled in the transition notifier. */
			break;
	}
	return 0;
}
#endif

//static int s3c24xx_spi_probe(struct platform_device *pdev)
static int s3c24xx_spi_probe(struct device *dev)
{
	struct platform_device *pdev;
	struct s3c24xx_spi *hw;
	struct spi_master *master;
	struct spi_board_info *bi;
	struct resource *res;
	int err = 0;
	int i;

	master = spi_alloc_master(dev, sizeof(struct s3c24xx_spi));
	if (master == NULL) {
		dev_err(dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hw = spi_master_get_devdata(master);
	memset(hw, 0, sizeof(struct s3c24xx_spi));

	hw->master = spi_master_get(master);
	hw->pdata = dev->platform_data;
	hw->dev = dev;

	if (hw->pdata == NULL) {
		dev_err(dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_no_pdata;
	}

	dev_set_drvdata( dev, hw );
	init_completion(&hw->done);

	/* setup the state for the bitbang driver */

	hw->bitbang.master         = hw->master;
	hw->bitbang.setup_transfer = s3c24xx_spi_setupxfer;
	hw->bitbang.chipselect     = s3c24xx_spi_chipsel;
	hw->bitbang.txrx_bufs      = s3c24xx_spi_txrx;
	hw->bitbang.master->setup  = s3c24xx_spi_setup;

	dev_dbg(hw->dev, "bitbang at %p\n", &hw->bitbang);

	/* find and map our resources */
	pdev = to_platform_device(dev);
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_iores;
	}

	hw->ioarea = request_mem_region(res->start, (res->end - res->start)+1,
					pdev->name);

	if (hw->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		err = -ENXIO;
		goto err_no_iores;
	}

	hw->regs = ioremap(res->start, (res->end - res->start)+1);
	if (hw->regs == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}

	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}

	err = request_irq(hw->irq, s3c24xx_spi_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_no_irq;
	}

	hw->clk = clk_get(&pdev->dev, "spi");
	if (IS_ERR(hw->clk)) {
		dev_err(&pdev->dev, "No clock for device\n");
		err = PTR_ERR(hw->clk);
		goto err_no_clk;
	}
	
#ifdef CONFIG_MACH_TOMTOMGO
	tomtom_spi_hw_init();
	
	/* Just a test until better times	*/
	{
		u32 reg = ioread32( S3C2443_PCLKCON );
		iowrite32( reg | S3C2443_PCLKCON_SPI_0, S3C2443_PCLKCON );	
	}
#endif /* CONFIG_MACH_TOMTOMGO */

	/* for the moment, permanently enable the clock */
	clk_enable(hw->clk);

	/* program defaults into the registers */

	iowrite8(0xff, hw->regs + S3C2410_SPPRE);
	iowrite16(SPFIC_DEFAULT, hw->regs + S3C2410_SPFIC);
	iowrite8(SPPIN_DEFAULT, hw->regs + S3C2410_SPPIN);
	iowrite32(SPCON_DEFAULT, hw->regs + S3C2410_SPCON);

	/* setup any gpio we can */

	if (!hw->pdata->set_cs) {
		s3c2410_gpio_setpin(hw->pdata->pin_cs, 1);
		s3c2410_gpio_cfgpin(hw->pdata->pin_cs, S3C2410_GPIO_OUTPUT);
	}
	/* register our spi controller */

	err = spi_bitbang_start(&hw->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master\n");
		goto err_register;
	}

	dev_dbg(hw->dev, "shutdown=%d\n", hw->bitbang.shutdown);

	/* register all the devices associated */

	bi = &hw->pdata->board_info[0];
	for (i = 0; i < hw->pdata->board_size; i++, bi++) {
		dev_info(hw->dev, "registering %s\n", bi->modalias);

		bi->controller_data = hw;
		spi_new_device(master, bi);
	}

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
        /* Initialize the frequency transition and notifier structures. */
        memset( &hw->freq_transition, 0, sizeof( hw->freq_transition ) );
        memset( &hw->freq_policy, 0, sizeof( hw->freq_policy ) );
        hw->freq_transition.notifier_call = spi_freq_transition;
        hw->freq_transition.priority = CPUFREQ_ORDER_S3C24XX_SPI_PRIO;
        hw->freq_policy.notifier_call = spi_freq_policy;
        cpufreq_register_notifier(&hw->freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
        cpufreq_register_notifier(&hw->freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif
	return 0;

 err_register:
	clk_disable(hw->clk);
	clk_put(hw->clk);

 err_no_clk:
	free_irq(hw->irq, hw);

 err_no_irq:
	iounmap(hw->regs);

 err_no_iomap:
	release_resource(hw->ioarea);
	kfree(hw->ioarea);

 err_no_iores:
 err_no_pdata:
	spi_master_put(hw->master);;

 err_nomem:
	return err;
}

//static int s3c24xx_spi_remove(struct platform_device *dev)
static int s3c24xx_spi_remove(struct device *dev)
{
	//struct s3c24xx_spi *hw = platform_get_drvdata(dev);
	struct s3c24xx_spi *hw = dev_get_drvdata( dev );

#if defined CONFIG_CPU_FREQ && defined CONFIG_S3C24XX_DFS_CPUFREQ
	cpufreq_unregister_notifier(&hw->freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_unregister_notifier(&hw->freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif
	//platform_set_drvdata(dev, NULL);

	spi_unregister_master(hw->master);

	clk_disable(hw->clk);
	clk_put(hw->clk);

	free_irq(hw->irq, hw);
	iounmap(hw->regs);

	release_resource(hw->ioarea);
	kfree(hw->ioarea);

	spi_master_put(hw->master);

#ifdef CONFIG_MACH_TOMTOMGO
	tomtom_spi_hw_exit();
#endif /* CONFIG_MACH_TOMTOMGO */

	return 0;
}


#ifdef CONFIG_PM

//static int s3c24xx_spi_suspend(struct platform_device *pdev, pm_message_t msg)
static int s3c24xx_spi_suspend(struct device *dev, pm_message_t msg, u32 level)
{
	//struct s3c24xx_spi *hw = platform_get_drvdata(pdev);
	struct s3c24xx_spi *hw = dev_get_drvdata( dev );

	hw->sppre_save = ioread8(hw->regs + S3C2410_SPPRE);
	hw->spfic_save = ioread16(hw->regs + S3C2410_SPFIC);
	hw->sppin_save = ioread8(hw->regs + S3C2410_SPPIN);
	hw->spcon_save = ioread32(hw->regs + S3C2410_SPCON);

	clk_disable(hw->clk);

#ifdef CONFIG_MACH_TOMTOMGO
	tomtom_spi_hw_exit();
#endif /* CONFIG_MACH_TOMTOMGO */
	return 0;
}

//static int s3c24xx_spi_resume(struct platform_device *pdev)
static int s3c24xx_spi_resume(struct device *dev, u32 level )
{
	struct s3c24xx_spi *hw = dev_get_drvdata( dev );

#ifdef CONFIG_MACH_TOMTOMGO
	tomtom_spi_hw_init();
#endif /* CONFIG_MACH_TOMTOMGO */

	clk_enable(hw->clk);

	iowrite8(hw->sppre_save, hw->regs + S3C2410_SPPRE);
	iowrite16(hw->spfic_save, hw->regs + S3C2410_SPFIC);
	iowrite8(hw->sppin_save, hw->regs + S3C2410_SPPIN);
	iowrite32(hw->spcon_save, hw->regs + S3C2410_SPCON);

	return 0;
}

#else
#define s3c24xx_spi_suspend NULL
#define s3c24xx_spi_resume  NULL
#endif

static struct device_driver s3c24xx_spidrv = {
	.name		= "s3c2410-spi",
	.owner		= THIS_MODULE,
	.bus		= &platform_bus_type,
	.probe		= s3c24xx_spi_probe,
	.remove		= s3c24xx_spi_remove,
	.suspend	= s3c24xx_spi_suspend,
	.resume		= s3c24xx_spi_resume,
		
};


static int __init s3c24xx_spi_init(void)
{
	return driver_register(&s3c24xx_spidrv);
}

static void __exit s3c24xx_spi_exit(void)
{
	driver_unregister(&s3c24xx_spidrv);
}

module_init(s3c24xx_spi_init);
module_exit(s3c24xx_spi_exit);

MODULE_DESCRIPTION("S3C24XX SPI Driver");
MODULE_AUTHOR("Ben Dooks, <ben@simtec.co.uk>");
MODULE_LICENSE("GPL");

