
#include <linux/config.h>

#define DEBUG

#include <linux/kernel.h>
#include <linux/err.h>

#include <asm/errno.h>
#include <asm/io.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-iis.h>
#include <asm/arch/regs-dyn.h>
#include <asm/hardware/clock.h>

#include <barcelona/Barc_snd.h>
#include <barcelona/gopins.h>
#include <barcelona/debug.h>

#include "iis.h"
#include "codec.h"

#ifdef CONFIG_CPU_S3C2412

#define PFX "iis2412: "
#define PK_DBG PK_DBG_FUNC

struct iis_regs {
	unsigned long iismod;
	unsigned long iisfic;
	unsigned long iispsr;
	unsigned long iiscon;
};

static struct iis_regs iis_regs;

extern unsigned current_samplerate;
extern unsigned current_samplesize;
extern unsigned iis_clockrate;

int iis_s3c2412_hw_init(void)
{
	int iismod = 0;	
	switch( IO_GetCodecMaster() ){
		case GOCODECCFG_SLAVE:
		case GOCODECCFG_INTERNAL_MASTER:
			iismod &= ~S3C2412_IISMOD_SLAVE;
			break;
		case GOCODECCFG_EXTERNAL_MASTER:
		default:
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
        	S3C2412_IISMOD_XMIT |
		iismod,
		S3C2412_IISMOD);

	writel(0, S3C2412_IISFIC);

	writel(S3C2412_IISPSR_PSRAEN, S3C2412_IISPSR);

	return 0;
}

void iis_s3c2412_hw_exit(void)
{
	/* This space was intentionally left blank :P */
}

void iis_s3c2412_showbus(const char *message)
{
	unsigned long mod,fic,psr,con;

	mod  = readl(S3C2412_IISMOD);
	fic = readl(S3C2412_IISFIC);
	psr  = readl(S3C2412_IISPSR);
	con  = readl(S3C2412_IISCON);

	printk("(%s) iismod: %lx, iisfic: %lx, iiscon: %lx, iispsr: %lx\n",
	       message, mod, fic, con, psr);
}

void iis_s3c2412_suspend(void)
{
	iis_regs.iismod  = readl(S3C2412_IISMOD);
	iis_regs.iisfic  = readl(S3C2412_IISFIC);
	iis_regs.iispsr  = readl(S3C2412_IISPSR);
	iis_regs.iiscon  = readl(S3C2412_IISCON);
	codec_suspend();
	
	clk_disable( iis_clock );
}

void iis_s3c2412_resume(void)
{
	clk_enable( iis_clock );

	codec_resume();
	writel(iis_regs.iismod,  S3C2412_IISMOD);
	writel(iis_regs.iisfic,  S3C2412_IISFIC);
	writel(iis_regs.iispsr,  S3C2412_IISPSR);
	writel(iis_regs.iiscon,  S3C2412_IISCON);
}

void iis_s3c2412_control_cmd(enum iis_cmd command)
{
	switch (command) {
	case IIS_IDLE_TX:
		PK_DBG("IIS_IDLE_TX\n");
		IO_Deactivate(I2SSDO);
		break;

	case IIS_ACTIVE_TX:
		PK_DBG("IIS_ACTIVE_TX\n");
		IO_SetFunction(I2SSDO);
		break;

	case IIS_ALIGN_TX:
		PK_DBG("IIS_ALIGN_TX\n");
		/* This is a no-op for 3c2412, not supported */
		break;


	case IIS_FLUSH_TX:
		PK_DBG("IIS_FLUSH_TX\n");
		writel(readl(S3C2412_IISFIC) | S3C2412_IISFIC_TFLUSH, S3C2412_IISFIC);
		writel(readl(S3C2412_IISFIC) & (~S3C2412_IISFIC_TFLUSH), S3C2412_IISFIC);
		break;

	case IIS_FLUSH_RX:
		PK_DBG("IIS_FLUSH_RX\n");
		writel(readl(S3C2412_IISFIC) | S3C2412_IISFIC_RFLUSH, S3C2412_IISFIC);
		writel(readl(S3C2412_IISFIC) & (~S3C2412_IISFIC_RFLUSH), S3C2412_IISFIC);
		break;

	case IIS_START:
		PK_DBG("IIS_START\n");
		writel(readl(S3C2412_IISCON) | S3C2412_IISCON_I2SACTIVE, S3C2412_IISCON);
		break;

	case IIS_STOP:
		PK_DBG("IIS_STOP\n");
		writel(readl(S3C2412_IISCON) & (~S3C2412_IISCON_I2SACTIVE), S3C2412_IISCON);
		break;

	default:
		PK_DBG("IIS_UNKNOWN!\n");
		break;

	}
}

static inline unsigned divrnd(unsigned nom, unsigned den)
{
	return (nom + den / 2) / den;
}

int iis_s3c2412_set_samplerate(unsigned rate)
{
	/* Formula is: div + 1 = iisclk / (fs * rate) */
	unsigned div1;
	unsigned div2;
	unsigned realrate1;
	unsigned realrate2;
	unsigned diff1;
	unsigned diff2;
	enum codec_fs fs;
	
	unsigned realrate;
	unsigned long flags;
	unsigned long iismod;

#ifndef CONFIG_CPU_FREQ
	printk("iis-24nn\n");
#endif
	switch( IO_GetCodecMaster() ){
		case GOCODECCFG_INTERNAL_MASTER:		
			{
			local_irq_save(flags);
			codec_set_fs(0,rate, iis_clockrate );
			writel( (((iis_clk_div-1) &0x3ff)<< 8) | S3C2412_IISPSR_PSRAEN, S3C2412_IISPSR);
			current_samplerate = rate;
			local_irq_restore(flags);
			}
			break;
			
		case GOCODECCFG_EXTERNAL_MASTER:
			current_samplerate = rate;
			codec_set_fs(0,rate, iis_clockrate );
			break;
			
		case GOCODECCFG_SLAVE:
		default:
			{
			unsigned div;
			
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
			codec_set_fs(fs,0, iis_clockrate );
	
			iismod = readl(S3C2412_IISMOD);
			iismod &= ~S3C2412_IISMOD_RFSMASK;
			iismod |= (fs == FS256) ? S3C2412_IISMOD_256FS : S3C2412_IISMOD_384FS;
			writel(iismod, S3C2412_IISMOD);
			writel(div << 8 | S3C2412_IISPSR_PSRAEN, S3C2412_IISPSR);
	
			local_irq_restore(flags);
			}
			break;
	}

	//iis_showbus("after set samplerate");

	return 0;
}

int iis_s3c2412_set_samplesize(unsigned size)
{
	unsigned long iismod = readl(S3C2412_IISMOD);

	if (size == 16)
        	iismod &= ~S3C2412_IISMOD_8BIT;
	else if (size == 8)
        	iismod |= S3C2412_IISMOD_8BIT;
	else
		return -EINVAL;


	current_samplesize = size;
    	writel(iismod, S3C2412_IISMOD);
	//iis_showbus("after setsmp");

	return 0;
}

iis_hwintf_t iis_s3c2412_intf = {
	.hw_init        = iis_s3c2412_hw_init,
	.hw_exit        = iis_s3c2412_hw_exit,
	.control_cmd    = iis_s3c2412_control_cmd,
	.set_samplerate = iis_s3c2412_set_samplerate,
	.set_samplesize = iis_s3c2412_set_samplesize,
	.showbus        = iis_s3c2412_showbus,
	.suspend        = iis_s3c2412_suspend,
	.resume         = iis_s3c2412_resume,
};

#endif /* CONFIG_CPU_S3C2412 */
