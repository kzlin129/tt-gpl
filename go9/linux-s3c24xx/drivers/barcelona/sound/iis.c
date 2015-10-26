#include <linux/kernel.h>
#include <linux/err.h>

#include <asm/errno.h>
#include <asm/io.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-iis.h>
#include <asm/arch/regs-dyn.h>
#include <asm/hardware/clock.h>
#include <asm/arch/regs-clock.h>

#include <barcelona/Barc_snd.h>
#include <barcelona/gopins.h>
#include <barcelona/debug.h>

#include "iis.h"
#include "codec.h"

#define PFX "iis2410: "
#define PK_DBG PK_DBG_FUNC

struct iis_regs {
	unsigned long iismod;
	unsigned long iisfcon;
	unsigned long iispsr;
	unsigned long iiscon;
};

struct clk *iis_clock;
static struct iis_regs iis_regs;

unsigned current_samplerate;
unsigned current_samplesize;
unsigned iis_clockrate;
unsigned iis_clk_div;

#if defined(CONFIG_CPU_S3C2412) || defined( CONFIG_CPU_S3C2443 ) || defined( CONFIG_CPU_S3C2450 )
extern iis_hwintf_t iis_s3c2410_intf; /* see end of file */
extern iis_hwintf_t iis_s3c2412_intf; /* from iis-s3c2412.c */
extern iis_hwintf_t iis_s3c2443_intf; /* from iis-s3c2443.c */
extern iis_hwintf_t iis_s3c2450_intf; /* from iis-s3c2450.c */

iis_hwintf_t iis_hwintf; /* This will point to one of the above, depending on hardware we're running on; set in iis_init() */
#endif /* CONFIG_CPU_S3C2412 || CONFIG_CPU_S3C2443 || CONFIG_CPU_S3C2450 */

int iis_s3c2410_hw_init(void)
{
	unsigned long iiscon =	S3C2410_IISCON_PSCEN |
				S3C2410_IISCON_IISEN |
				S3C2410_IISCON_TXDMAEN |
				S3C2410_IISCON_RXDMAEN;

	unsigned long iisfcon = S3C2410_IISFCON_RXDMA |
				S3C2410_IISFCON_TXDMA |
				S3C2410_IISFCON_TXENABLE |
				S3C2410_IISFCON_RXENABLE;

	unsigned long iismod =	S3C2410_IISMOD_LR_RLOW |
				S3C2410_IISMOD_IIS |
				S3C2410_IISMOD_256FS |
				S3C2410_IISMOD_32FS |
				S3C2410_IISMOD_TXRXMODE;

	if (IO_GetCodecMaster() ) /* Codec is either internal or external Master	*/
		iismod |= S3C2410_IISMOD_SLAVE;

	//dump_iis_bus("Test1");
	writel(iismod,  S3C2410_IISMOD);
	writel(iisfcon, S3C2410_IISFCON);
	writel(iiscon,  S3C2410_IISCON);
	//dump_iis_bus("Test2");

	return 0;
}

void iis_s3c2410_hw_exit(void)
{
	/* This space was intentionally left blank :P */
}

void iis_s3c2410_showbus(const char *message)
{
	unsigned long mod,fcon,psr,con;

	mod  = readl(S3C2410_IISMOD);
	fcon = readl(S3C2410_IISFCON);
	psr  = readl(S3C2410_IISPSR);
	con  = readl(S3C2410_IISCON);


	printk("(%s) iismod: %lx, iisfcon: %lx, iiscon: %lx, iispsr: %lx\n",
	       message, mod, fcon, con, psr);
}

void iis_s3c2410_suspend(void)
{
	iis_regs.iismod  = readl(S3C2410_IISMOD);
	iis_regs.iisfcon = readl(S3C2410_IISFCON);
	iis_regs.iispsr  = readl(S3C2410_IISPSR);
	iis_regs.iiscon  = readl(S3C2410_IISCON);
	codec_suspend();

	clk_disable( iis_clock );
}

void iis_s3c2410_resume(void)
{
	clk_enable( iis_clock );

	codec_resume();
	writel(iis_regs.iismod,  S3C2410_IISMOD);
	writel(iis_regs.iisfcon, S3C2410_IISFCON);
	writel(iis_regs.iispsr,  S3C2410_IISPSR);
	writel(iis_regs.iiscon,  S3C2410_IISCON);

	/* IRA: Cold hard hack ; should not be needed! */
	if (IO_GetCodecMaster()) {
		IO_SetInput(CDCLK);
		IO_SetFunction(CDCLK_12MHZ);
	} else {
		IO_SetFunction(CDCLK);
		IO_SetInput(CDCLK_12MHZ);
	}
}

void iis_s3c2410_control_cmd(enum iis_cmd command)
{
	unsigned iiscon, iisfcon;

	switch (command) {
	case IIS_IDLE_TX:
		IO_Deactivate(I2SSDO);
		break;

	case IIS_ACTIVE_TX:
		IO_SetFunction(I2SSDO);
		break;

	case IIS_ALIGN_TX:
		if(readl(S3C2410_IISCON) & S3C2410_IISCON_LRINDEX) {
			iisfcon = readl(S3C2410_IISFCON);
			writel(iisfcon & (~S3C2410_IISFCON_TXDMA), S3C2410_IISFCON);
			writew(0, S3C2410_IISFIFO);
			writel(iisfcon, S3C2410_IISFCON);

	                /* wait for flip to correct channel to give more stable left / right selection (not full correct solution) */
			if (IO_GetCodecMaster()) {
		                while (readl(S3C2410_IISCON) & S3C2410_IISCON_LRINDEX)
		                	;
			}
		}
		break;


	case IIS_FLUSH_TX:
		iisfcon = readl(S3C2410_IISFCON);
		iisfcon &= ~S3C2410_IISFCON_TXENABLE;
		writel(iisfcon, S3C2410_IISFCON);
		iisfcon |=  S3C2410_IISFCON_TXENABLE;
		writel(iisfcon, S3C2410_IISFCON);
		break;

	case IIS_FLUSH_RX:
		iisfcon = readl(S3C2410_IISFCON);
		iisfcon &= ~S3C2410_IISFCON_RXENABLE;
		writel(iisfcon, S3C2410_IISFCON);
		iisfcon |=  S3C2410_IISFCON_RXENABLE;
		writel(iisfcon, S3C2410_IISFCON);
		break;

	case IIS_START:
		iiscon = readl(S3C2410_IISCON);
		iiscon |= S3C2410_IISCON_IISEN;
		writel(iiscon, S3C2410_IISCON);
		break;

	case IIS_STOP:
		iiscon = readl(S3C2410_IISCON);
		iiscon &= ~S3C2410_IISCON_IISEN;
		writel(iiscon, S3C2410_IISCON);
		break;

	default:
		break;

	}
}

static inline unsigned divrnd(unsigned nom, unsigned den)
{
	return (nom + den / 2) / den;
}

int iis_s3c2410_set_samplerate(unsigned rate)
{
	/* Formula is: div + 1 = iisclk / (fs * rate) */
	unsigned div1;
	unsigned div2;
	unsigned realrate1;
	unsigned realrate2;
	unsigned diff1;
	unsigned diff2;
	enum codec_fs fs;
	unsigned div;
	unsigned realrate;
	unsigned long flags;
	unsigned long iismod;
	
	switch(IO_GetCodecMaster()) {
	case GOCODECCFG_INTERNAL_MASTER:
		{
			unsigned short div = clk_get_rate(iis_clock) / INTERNAL_MASTER_CLOCK;
			local_irq_save(flags);
			--div;
			codec_set_fs(0,rate, iis_clockrate );
			writel((div << 5) | div, S3C2410_IISPSR);
			current_samplerate = rate;
			local_irq_restore(flags);
		}
		break;

	case GOCODECCFG_EXTERNAL_MASTER:
		#define rMISCCR     (*(volatile unsigned *)s3c24xx_misccr)
		#define rUPLLCON    (*(volatile unsigned *)S3C2410_UPLLCON)
	
		switch (IO_GetGpsType()) {
			/* GL type chips */
			case GOGPS_GL:
			case GOGPS_GL_INT_LNA:
			case GOGPS_GL_BCM4750:

				if (IO_HaveUsbHost()) {
					/* turn off UPLL */
					rUPLLCON = (1<<20);
					/* select xtal clock for codec */
					rMISCCR = (rMISCCR & (~(7 << 4)));
				} 
				else {
					switch (rate) {
						case 8000:
						case 12000:
						case 16000:
						case 24000:
						case 32000:
						case 48000:
						case 96000:
						/* 12.288 MHz codec master clock */
						rUPLLCON = 0xCD322;
						rMISCCR = (rMISCCR & (~(7 << 4))) | (1 << 4);
						break;
						
						default:
						/* 11.2896 MHz codec master clock */
						rUPLLCON = (135 << 12) | (36 << 4) | (2 << 0);
						rMISCCR = (rMISCCR & (~(7 << 4))) | (1 << 4);
						break;
					}
				}

				break;
			
			/* SiRF chips and Barracuda */
			case GOGPS_SIRF1:
			case GOGPS_SIRF2:
			case GOGPS_SIRF3:

				break;

			default:
				printk("%s: ERROR: GPS chip type needs to be set!\n", __FUNCTION__);
				return -1;
		}

		current_samplerate = rate;
		codec_set_fs(0,rate, iis_clockrate);
		break;
		
	case GOCODECCFG_SLAVE:
		IO_SetFunction(CDCLK);
                IO_SetInput(CDCLK_12MHZ);
		div1 = divrnd(iis_clockrate, FS256 * rate);
		div2 = divrnd(iis_clockrate, FS384 * rate);
		if (unlikely((div1 == 0 || div1 > 32) || (div2 == 0 || div2 > 32))) {
			PK_ERR("Input rate %u too %s.\n", rate, (div1 == 0 || div2 == 0) ? "low" : "high");
			return -EINVAL;
		}

		realrate1 = divrnd(iis_clockrate, FS256 * div1);
		realrate2 = divrnd(iis_clockrate, FS384 * div2);
		if (unlikely((realrate1 < 4000 || realrate1 > 100000) || (realrate2 < 4000 || realrate2 > 100000))) {
			PK_ERR("Weird output rates %u, %u for input rate %u.\n", realrate1, realrate2, rate);
			return -EINVAL;
		}
	
		/* Rate should be okay, store it */
		current_samplerate = rate;
	
		/* Calculate best rate */
		diff1 = (rate <= realrate1) ? realrate1 - rate : rate - realrate1;
		diff2 = (rate <= realrate2) ? realrate2 - rate : rate - realrate2;
	
		PK_DBG("wantrate %u, div1 %u, realrate1 %u, diff1 %u\n", rate, div1, realrate1, diff1);
		PK_DBG("wantrate %u, div2 %u, realrate2 %u, diff2 %u\n", rate, div2, realrate2, diff2);
	
		if (diff1 <= diff2) {
			fs = FS256;
			div = div1;
			realrate = realrate1;
		} else {
			fs = FS384;
			div = div2;
			realrate = realrate2;
		}
	
		div--; /* This is the /real/ divider */
		PK_DBG("Using divider %u, fs %u, wantrate %u, realrate %u\n", div, fs, rate, realrate);
	
		local_irq_save(flags);
		codec_set_fs(fs,0, iis_clockrate);
		iismod = readl(S3C2410_IISMOD);
		if (fs == FS256)
			iismod &= ~S3C2410_IISMOD_384FS;
		else
			iismod |= S3C2410_IISMOD_384FS;
		writel(iismod, S3C2410_IISMOD);
		writel((div << 5) | div, S3C2410_IISPSR);
		local_irq_restore(flags);
		break;
	}

	//iis_showbus("after set samplerate");

	return 0;
}

int iis_s3c2410_set_samplesize(unsigned size)
{
	unsigned long iismod = readl(S3C2410_IISMOD);

	if (size == 8) iismod &= ~S3C2410_IISMOD_16BIT;
	else if (size == 16) iismod |= S3C2410_IISMOD_16BIT;
	else return -EINVAL;

	current_samplesize = size;
	writel(iismod, S3C2410_IISMOD);
	//iis_showbus("after setsmp");
	return 0;
}

int iis_init( unsigned int divider )
{
	int result;

	IO_Deactivate(I2SSDO);
	IO_SetFunction(I2SSCLK);
	IO_SetFunction(I2SLRCK);
	IO_SetFunction(I2SSDI);

	if (IO_GetCodecMaster() == GOCODECCFG_EXTERNAL_MASTER) {
		iis_clock = clk_get(NULL, "12mhz_iis");
		IO_SetInput( CDCLK );
	}
	else{
		iis_clock = clk_get(NULL, "iis");
		IO_SetInput( CDCLK_12MHZ );
	}
	if (IS_ERR(iis_clock)) {
		iis_clock = NULL;
		PK_ERR("Failed to get apropriate IIS clock\n");
		return -ENOENT;
	}

	codec_init();

	clk_use(iis_clock);
	clk_enable(iis_clock);
	
	iis_clockrate = clk_get_rate(iis_clock);
	iis_clk_div = divider;	
	if (IO_GetCodecMaster() == GOCODECCFG_INTERNAL_MASTER) {
		iis_clockrate = iis_clockrate/ iis_clk_div; /* calculate frq of output signal to codec( iis clock / iiss divider )s	*/
	}
	
	
	PK_INFO("Current clock rate: %u\n", iis_clockrate);

#if defined(CONFIG_CPU_S3C2412) || defined( CONFIG_CPU_S3C2443 ) || defined( CONFIG_CPU_S3C2450 )
	switch( IO_GetCpuType() ){
		case GOCPU_S3C2412:
			 iis_hwintf = iis_s3c2412_intf;
			 break;
		case GOCPU_S3C2443:
			iis_hwintf = iis_s3c2443_intf;
			break;
		case GOCPU_S3C2450:
			iis_hwintf = iis_s3c2450_intf;
			break;
		default:
			iis_hwintf = iis_s3c2410_intf;
	}
#endif /* CONFIG_CPU_S3C2412 || CONFIG_CPU_S3C2443 || defined( CONFIG_CPU_S3C2450 ) */

	result = iis_hw_init();
	if (!result) {
		iis_set_samplerate(INITIAL_SAMPLERATE);
		iis_set_samplesize(16);
	}

	codec_init_post_iis_hw_init();

	return result;
}

void iis_exit(void)
{
	if (iis_clock != NULL) {
		clk_disable(iis_clock);
		clk_unuse(iis_clock);
		clk_put(iis_clock);
		iis_clock = NULL;
	}

	iis_hw_exit();
}

#ifdef CONFIG_CPU_S3C2412
iis_hwintf_t iis_s3c2410_intf = {
	.hw_init        = iis_s3c2410_hw_init,
	.hw_exit        = iis_s3c2410_hw_exit,
	.control_cmd    = iis_s3c2410_control_cmd,
	.set_samplerate = iis_s3c2410_set_samplerate,
	.set_samplesize = iis_s3c2410_set_samplesize,
	.showbus        = iis_s3c2410_showbus,
	.suspend        = iis_s3c2410_suspend,
	.resume         = iis_s3c2410_resume,
};
#endif /* CONFIG_CPU_S3C2412 */
