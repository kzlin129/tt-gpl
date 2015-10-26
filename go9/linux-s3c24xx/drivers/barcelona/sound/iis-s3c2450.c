#include <linux/config.h>

#define DEBUG

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <asm/errno.h>
#include <asm/io.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-iis.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-dyn.h>
#include <asm/hardware/clock.h>

#include <barcelona/Barc_snd.h>
#include <barcelona/gopins.h>
#include <barcelona/debug.h>

#include "iis.h"
#include "codec.h"

#if defined(CONFIG_CPU_S3C2450)

#define PFX "iis2450: "
#define PK_DBG PK_DBG_FUNC

#define S3C2450_IISPSR_MASK     0x7f
#define S3C2450_IISPSR_SHIFT    8

struct iis_regs {
	unsigned long iismod;
	unsigned long iisfic;
	unsigned long iispsr;
	unsigned long iiscon;
};

extern unsigned current_samplerate;
extern unsigned current_samplesize;
extern unsigned iis_clockrate;

/* Dirty export from arch/arm/mach-s3c2410/clock.{c,h}	*/
extern int s3c2450_epll_set_rate( unsigned long rate );

static void iis_2450_setup_epll_path( void )
{
	/* Select SELI2s == 0	*/
	unsigned long reg = readl( S3C2450_CLKSRC );
	reg &= ~(S3C2412_CLKSRC_SELEREF_MASK);
	reg |= (1<<6);	/* SELEPLL instead of EpllRefClk	*/
	writel(reg, S3C2450_CLKSRC); 
			
	reg = readl(S3C2450_SCLKCON);
	reg |= S3C2450_SCLKCON_IISCLK_0;
	writel( reg, S3C2450_SCLKCON );
}

static int iis_s3c2450_hw_init(void)
{
	int iismod = 0;
	
	switch( IO_GetCodecMaster() ){
		case GOCODECCFG_SLAVE:
			iismod &= ~S3C2450_IISMOD_CDCLKCON;	/* specify to output S3c generated clk signal	*/
			iismod |= S3C2412_IISMOD_EMASTER;

			s3c2450_epll_set_rate( 32769231 );
			iis_clockrate = 32769231;
			msleep( 10 );
			
			/* We already made sure that we own the EPLL in iis.c	*/
			/* because parent of iis clock is EPLL 					*/ 
			iis_2450_setup_epll_path();

			break;
		case GOCODECCFG_INTERNAL_MASTER:
			iismod &= ~S3C2450_IISMOD_CDCLKCON;	/* specify to output S3c generated clk signal	*/
			iismod |= S3C2412_IISMOD_SLAVE;
			break;
		case GOCODECCFG_EXTERNAL_MASTER:
		default:
			iismod |= S3C2450_IISMOD_CDCLKCON;	/* Codec is externally clocked        */
			iismod |= S3C2412_IISMOD_SLAVE;
			break;
	}

	writel(
		S3C2412_IISCON_I2SACTIVE |
	        S3C2412_IISCON_RXDMACTIVE |
	        S3C2412_IISCON_TXDMACTIVE,
	        S3C2412_IISCON);

	writel(
		S3C2412_IISMOD_LOCLKR |
        	S3C2412_IISMOD_FMTI2S |
        	S3C2412_IISMOD_256FS |
        	S3C2412_IISMOD_BFS32FS |
		S3C2412_IISMOD_XMITRECV |
		iismod,
		S3C2412_IISMOD);

	writel(0, S3C2412_IISFIC);

	writel(S3C2412_IISPSR_PSRAEN, S3C2412_IISPSR);

        /* FIXME: Bootloader incorrectly assigns PCM functions to these pins */
        writel( 0, S3C2450_GPESLPCON );
	return 0;
}

static void iis_s3c2450_hw_exit(void)
{
	/* This space was intentionally left blank :P */
}


static inline unsigned divrnd(unsigned nom, unsigned den)
{
	return (nom + den / 2) / den;
}

static int iis_s3c2450_set_samplerate(unsigned rate)
{
	unsigned long flags;
	
	PK_DBG("iis-2450@%u MHz (div=%u)\n", iis_clockrate, iis_clk_div);
	
	switch( IO_GetCodecMaster() ){
		case GOCODECCFG_INTERNAL_MASTER:			
			{
			local_irq_save(flags);
			codec_set_fs(0,rate, iis_clockrate);
			writel( (((iis_clk_div - 1) & S3C2450_IISPSR_MASK) << S3C2450_IISPSR_SHIFT ) | S3C2412_IISPSR_PSRAEN, S3C2412_IISPSR); 
			current_samplerate = rate;
			local_irq_restore(flags);
			}
			break;
		
		case GOCODECCFG_EXTERNAL_MASTER:
			current_samplerate = rate;
			local_irq_save(flags);
			writel( 0, S3C2412_IISPSR); 	/* No clock divider please!	*/
			codec_set_fs(0,rate, iis_clockrate);
			local_irq_restore(flags);
			break;
			
		case GOCODECCFG_SLAVE:
		default:
			{
			unsigned div;
			unsigned realrate1;
			unsigned realrate2;
			
			/* Formula is: div + 1 = iisclk / (fs * rate) */
			unsigned div1;
			unsigned div2;
		
			unsigned diff1;
			unsigned diff2;
			unsigned long iismod;
			enum codec_fs fs;
			unsigned realrate;

			switch (rate) {
			case 8000:
			case 16000:
			case 32000:	
				s3c2450_epll_set_rate( 32769231 );
				iis_clockrate = 32769231;
				break;

			case 11025:
			case 22050:
			case 44100:	
				s3c2450_epll_set_rate( 67714286 );
				iis_clockrate = 67714286;
				break;

			case 12000:
			case 24000:
			case 48000:	
				s3c2450_epll_set_rate( 73714286 );
				iis_clockrate = 73714286;
				break;
			}
			
			iis_2450_setup_epll_path( );
			
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
	
			iismod = readl(S3C2412_IISMOD);
			iismod &= ~S3C2412_IISMOD_RFSMASK;
			iismod |= (fs == FS256) ? S3C2412_IISMOD_256FS : S3C2412_IISMOD_384FS;
			writel(iismod, S3C2412_IISMOD);
			writel((div << S3C2450_IISPSR_SHIFT) | S3C2412_IISPSR_PSRAEN, S3C2412_IISPSR);
	
			local_irq_restore(flags);
			}
			break;
			
	}

	//iis_showbus("after set samplerate");

	return 0;
}

/**
 *  The S3C2450 I2S interface is basically a slightly modified S3C2412 interface.
 *  We inherit the basic stuff and override only the methods that need a different approach.
 */
extern void iis_s3c2412_control_cmd(enum iis_cmd command);
extern void iis_s3c2412_suspend(void);
extern void iis_s3c2412_resume(void);
extern int iis_s3c2412_set_samplesize(unsigned size);
extern void iis_s3c2412_showbus(const char *message);

iis_hwintf_t iis_s3c2450_intf = {
	.hw_init        = iis_s3c2450_hw_init,
	.hw_exit        = iis_s3c2450_hw_exit,
	.control_cmd    = iis_s3c2412_control_cmd,
	.set_samplerate = iis_s3c2450_set_samplerate,
	.set_samplesize = iis_s3c2412_set_samplesize,
	.showbus        = iis_s3c2412_showbus,
	.suspend        = iis_s3c2412_suspend,
	.resume         = iis_s3c2412_resume,
};

#endif /* CONFIG_CPU_S3C2412 && CONFIG_CPU_S3C2450 */
