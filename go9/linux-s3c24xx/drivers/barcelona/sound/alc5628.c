#include <linux/kernel.h>
#include <barcelona/debug.h>
#include <barcelona/gopins.h>

#include <linux/i2c.h>
#include "alc5628.h"
#include "alc5628-i2c.h"

#include "iis.h"
#include <asm/io.h>
#include <asm/arch/regs-iis.h>

#include <linux/delay.h>  /* for mdelay */

#define PFX "alc5628: "
#define PK_DBG PK_DBG_FUNC

#define REG_38_FS256 (0)
#define REG_38_FS384 (1)
#define REG_38_SEL_DAC_FILTER_CLK_MASK (4)
#define REG_38_SEL_DAC_FILTER_CLK_BITS (2)

#define REG_00 (0)
#define REG_02 (1)
#define REG_04 (2)
#define REG_0C (4)
#define REG_16 (5)
#define REG_1C (6)
#define REG_34 (7)
#define REG_38 (8)
#define REG_3A (9)
#define REG_3C (10)
#define REG_3E (11)
#define REG_40 (12)
#define REG_42 (13)
#define REG_44 (14)
#define REG_5C (16)
#define REG_68 (18)


static uint16_t alc5628_boot[29][2] = {
	{0x00, 0x0003},    /*  0, reg-00h */
	{0x02, 0x9F9F},    /*  1, reg-02h */
	{0x04, 0x9F9F},    /*  2, reg-04h */
	{0x0A, 0xC8C8},    /*  3, reg-0Ah */
	{0x0C, 0xFFFF},    /*  4, reg-0Ch */
	{0x16, 0x0009},    /*  5, reg-16h */
	{0x1C, 0x8004},    /*  6, reg-1Ch */
	{0x34, 0x8000},    /*  7, reg-34h */
	{0x38, 0x2000},    /*  8, reg-38h */
	{0x3A, 0x0000},    /*  9, reg-3Ah */
	{0x3C, 0x0000},    /* 10, reg-3Ch */
	{0x3E, 0x0000},    /* 11, reg-3Eh */
	{0x40, 0x0000},    /* 12, reg-40h */
	{0x42, 0x0000},    /* 13, reg-42h */
	{0x44, 0x0000},    /* 14, reg-44h */
	{0x5A, 0x0000},    /* 15, reg-5Ah */
	{0x5C, 0x0000},    /* 16, reg-5Ch */
	{0x5E, 0x0000},    /* 17, reg-5Eh */
	{0x68, 0x000B},    /* 18, reg-68h */
	{0x6A, 0x0000},    /* 19, reg-6Ah */
	{0x6C, 0x0000},    /* 20, reg-6Ch */
	{0x21, 0x0400},    /* 21, private-21h */
	{0x22, 0x0390},    /* 22, private-22h */
	{0x23, 0x0040},    /* 23, private-23h */
	{0x24, 0x03FF},    /* 24, private-24h */
	{0x25, 0x0400},    /* 25, private-25h */
	{0x39, 0x8800},    /* 26, private-39h */
	{0x7C, 0x10EC},    /* 27, reg-7Ch */
	{0x7E, 0x2700},    /* 28, reg-7Eh */
};

static uint16_t alc5628_current[29][2] = {
	{0x00, 0x0000},    /* reg-00h */
	{0x02, 0x0000},    /* reg-02h */
	{0x04, 0x0000},    /* reg-04h */
	{0x0A, 0x0000},    /* reg-0Ah */
	{0x0C, 0x0000},    /* reg-0Ch */
	{0x16, 0x0000},    /* reg-16h */
	{0x1C, 0x0000},    /* reg-1Ch */
	{0x34, 0x0000},    /* reg-34h */
	{0x38, 0x0000},    /* reg-38h */
	{0x3A, 0x0000},    /* reg-3Ah */
	{0x3C, 0x0000},    /* reg-3Ch */
	{0x3E, 0x0000},    /* reg-3Eh */
	{0x40, 0x0000},    /* reg-40h */
	{0x42, 0x0000},    /* reg-42h */
	{0x44, 0x0000},    /* reg-44h */
	{0x5A, 0x0000},    /* reg-5Ah */
	{0x5C, 0x0000},    /* reg-5Ch */
	{0x5E, 0x0000},    /* reg-5Eh */
	{0x68, 0x0000},    /* reg-68h */
	{0x6A, 0x0000},    /* reg-6Ah */
	{0x6C, 0x0000},    /* reg-6Ch */
	{0x21, 0x0000},    /* private-21h */
	{0x22, 0x0000},    /* private-22h */
	{0x23, 0x0000},    /* private-23h */
	{0x24, 0x0000},    /* private-24h */
	{0x25, 0x0000},    /* private-25h */
	{0x39, 0x0000},    /* private-39h */
	{0x7C, 0x0000},    /* reg-7Ch */
	{0x7E, 0x0000},    /* reg-7Eh */
};

static struct i2c_client *alc5628_i2c_client;

static inline void ALC5628_WRREG(uint16_t reg, uint16_t value)
{
	alc5628_current[reg][1] = value;
	/*printk("T%d: %2x = 0x%x\n", reg, alc5628_current[reg][0], value);*/

	if (!alc5628_i2c_client)
		PK_ERR("i2c control device has not been detected (yet)!\n");
	else
	{
		unsigned char buf[3] = { alc5628_current[reg][0] & 0xff, (char)(value >> 8), value & 0xff };
		if (3 > i2c_master_send(alc5628_i2c_client, buf, 3))
			PK_ERR("i2c_master_send() failed\n");
	}
}

static inline uint16_t ALC5628_RDREG(uint16_t reg)
{
	return alc5628_current[reg][1];
}

static void alc5628_init_registers(void)
{
	int i;

	/* set iis PSR divider based on EPLL of 32769231 that is set by the 
	 * i2s init. divide by 3 to get a 10.9 MHz 'MCLK' for the codec init.
	 */
	writel((1<<15) + ((3-1)<<8), S3C2412_IISPSR); 
	mdelay(1); 
	ALC5628_WRREG(REG_00, 0x0003); /* reset */
	mdelay(1); /* reset takes < 1msec before alc5628 codec is ready */

	/* fill in the current read values of the registers which are default reset values */
	for (i = 0; i < sizeof alc5628_boot / sizeof *alc5628_boot; i++) {
		/* a reset sets al registers to the default value, no need to write them again */
		alc5628_current[i][1] = alc5628_boot[i][1];
	}

	/* init */
	if (!IO_HaveRiderDock()) {
		ALC5628_WRREG(REG_3C, 0x2000); /* enable analog power */
		ALC5628_WRREG(REG_3E, 0x8000); /* enable mainbias */
		ALC5628_WRREG(REG_16, 0x0000); /* 1x SVSYNC (1/sample rate) time per soft volume step (1.5dB)*/
		ALC5628_WRREG(REG_5C, 0xaffe); /* disable soft mute and enable zero cross volume control */
		/*ALC5628_WRREG(REG_0C, 0x1010);*/ /* set DAC volume to 0dB and output all mixer */
		ALC5628_WRREG(REG_0C, 0x1818); /* set DAC volume to -6dB (for SPK mixer) and output all mixer */
		ALC5628_WRREG(REG_04, 0x8888); /* set HP vol to -12dB by default and mute */
		ALC5628_WRREG(REG_02, 0x8080); /* set speaker volume to 0dB and mute */
		ALC5628_WRREG(REG_40, 0x0b00); /* ratio 1 Vdd (SPKVDD/AVDD = >=3.3/3.3 = 1Vdd */
		ALC5628_WRREG(REG_3A, 0x8000); /* enable i2s */
		/* TODO: delay to avoid pop */
		/*ALC5628_WRREG(REG_3C, 0x67f0);*/ /* enable class d, dac -> mixer -> HP mixer */
		ALC5628_WRREG(REG_3C, 0x67c8); /* enable class d, dac -> mixer -> SPK mixer */
		ALC5628_WRREG(REG_3E, 0x9000); /* enable mainbias, power to speaker */
		/*ALC5628_WRREG(REG_1C, 0x0704);*/ /* enable HP and HP mixer volume output source, diff mode */
		ALC5628_WRREG(REG_1C, 0x0804); /* enable SPK mixer volume output source, diff mode */
		ALC5628_WRREG(REG_38, 0x0004); /* defaults (256Fs), i2s pre div to div 1 */
	} else {
		ALC5628_WRREG(REG_3C, 0x2000); /* enable analog power */
		ALC5628_WRREG(REG_3E, 0x8600); /* enable mainbias */
		ALC5628_WRREG(REG_16, 0x0000); /* 1x SVSYNC (1/sample rate) time per soft volume step (1.5dB)*/
		ALC5628_WRREG(REG_5C, 0xaafe); /* disable soft mute and enable zero cross volume control */
		/*ALC5628_WRREG(REG_0C, 0x1010);*/ /* set DAC volume to 0dB and output all mixer */
		ALC5628_WRREG(REG_0C, 0x1c1c); /* set DAC volume to -6dB (for SPK mixer) and output all mixer */
		ALC5628_WRREG(REG_04, 0x8080); /* set HP vol to 0dB by default and mute */
		ALC5628_WRREG(REG_02, 0x8080); /* set speaker volume to 0dB and mute */
		ALC5628_WRREG(REG_40, 0x0b00); /* ratio 1 Vdd (SPKVDD/AVDD = >=3.3/3.3 = 1Vdd */
		ALC5628_WRREG(REG_3A, 0x8020); /* enable i2s and enable H*/
		/* TODO: delay to avoid pop */
		/*ALC5628_WRREG(REG_3C, 0x67f0);*/ /* enable class d, dac -> mixer -> HP mixer */
		ALC5628_WRREG(REG_3C, 0x27f0); /* enable class d, dac -> mixer -> SPK mixer */
		ALC5628_WRREG(REG_3E, 0x8600); /* enable mainbias, power to speaker */
		/*ALC5628_WRREG(REG_1C, 0x0704);*/ /* enable HP and HP mixer volume output source, diff mode */
		ALC5628_WRREG(REG_1C, 0x0700); /* enable SPK mixer volume output source, diff mode */
		ALC5628_WRREG(REG_38, 0x0004); /* defaults (256Fs), i2s pre div to div 1 */
	}
}

void alc5628_init_i2c_control(struct i2c_client *cl)
{
	alc5628_i2c_client = cl;
	alc5628_init_registers();
}

static int alc5628_set_volume(unsigned int volume)
{
	/* set_volume function is based on another codec with 
	 * ~1dB per step and up to 127, 121 = 0dB. For new
	 * products set_volume_dB should be used. 
	 */

	PK_ERR("use set_volume_dB\n");
	return 0;
}

#define SPK_VOLUME_STEP (48) /* 1.5 * 32 */
static int alc5628_set_volume_dB(unsigned int volume)
{
	uint16_t value_reg_02 = 0x9F9F; /* mute */
	uint16_t attenuation = 0;
	int result = 0;

	/* the attenuation in dB comes in as a possitive value */
	/*printk("Setting volume to: -%d dB\n", volume);*/

	/* set correct SPKVDD/AVDD ratio */
	/* TODO: note: implementation for voices only, no dynamic switching during playback */
	if (IO_GetInput(ACPWR)) {
		/*ALC5628_WRREG(REG_40, 0x0900);*/ /* ratio 1.25 Vdd (SPKVDD/AVDD = ~4.85/3.3 = 1.25Vdd */
		ALC5628_WRREG(REG_40, 0x0700); /* ratio 1.5 Vdd (SPKVDD/AVDD = ~4.85/3.3 = 1.25Vdd so 1.5 gives distortion upto 4% THD when input is 0dBFS */
	} else {
		ALC5628_WRREG(REG_40, 0x0b00); /* ratio 1 Vdd (SPKVDD/AVDD = >=3.3/3.3 = 1Vdd */

		/* adjust volume for 3dB when on battery */
		if (volume >= 3)
			volume -= 3;
		else {
			/* result contains adjustment needed by audio engine when over 0dB */
			result = -(volume - 3);
			volume = 0;
		}
	}

	/* scale with 32 to get the 1.5 dB steps in calculation range */
	attenuation = (volume * 32) / SPK_VOLUME_STEP;
	if (attenuation <= 0x1f) /* no mute */
		value_reg_02 = (attenuation << 8) | attenuation; 

	/* printk("Setting volume reg to: %x\n", value_reg_02); */
	/* Write speaker volume and headphone volume at same time */
	if (!IO_HaveRiderDock())
		ALC5628_WRREG(REG_02, value_reg_02);
	else
		ALC5628_WRREG(REG_04, value_reg_02);
	return result;
}

static int alc5628_set_treble(unsigned int treble)
{
	/* Treble is not supported, so do nothing */
	return 0;
}

static int alc5628_set_bass(unsigned int bass)
{
	/* Bass is not supported, so do nothing */
	return 0;
}

static int alc5628_set_fs(enum codec_fs fs,unsigned int samplerate, unsigned int clockrate)
{
	/* only support codec in slave mode for now */
	uint16_t value_reg_38 = (ALC5628_RDREG(REG_38)) & ~REG_38_SEL_DAC_FILTER_CLK_MASK;
	switch (fs) {
	case FS256:
		value_reg_38 |= (REG_38_FS256 << REG_38_SEL_DAC_FILTER_CLK_BITS);
		break;
	case FS384:
		value_reg_38 |= (REG_38_FS384 << REG_38_SEL_DAC_FILTER_CLK_BITS);
		break;
	default:
		PK_ERR("Unknown fs %d\n", (int) fs);
		return -EINVAL;
	}
	ALC5628_WRREG(REG_38, value_reg_38);

	return 0;
}

static int alc5628_switch_line_in(unsigned int line_in)
{
	/* line in is not supported, so do nothing */
	return 0;
}

static int alc5628_set_register(unsigned int data)
{
	/* do not support set_register */
	return 0;
}

static void alc5628_powerdown(void)
{
	ALC5628_WRREG(REG_04, 0x8888); /* set HP vol to -12dB by default and mute */
	ALC5628_WRREG(REG_02, 0x8080); /* set speaker volume to 0dB and mute */

	/* go to idle mode */
	ALC5628_WRREG(REG_3A, 0x0000);
	ALC5628_WRREG(REG_3E, 0x8000);
	ALC5628_WRREG(REG_3C, 0x2000);

	mdelay(1);

	/* power down */
	ALC5628_WRREG(REG_3E, 0x0000); /* disable mainbias */
	ALC5628_WRREG(REG_3C, 0x0000); /* disable vref */
}

void alc5628_init(struct codec_ops *ops)
{
	ops->set_volume = alc5628_set_volume;
	ops->set_volume_internal = alc5628_set_volume;
	ops->set_volume_external = alc5628_set_volume;
	ops->set_register = alc5628_set_register;
	ops->resume = alc5628_init_registers;
	ops->suspend = alc5628_powerdown;
	ops->set_treble = alc5628_set_treble;
	ops->set_bass = alc5628_set_bass;
	ops->set_fs = alc5628_set_fs;
	ops->set_volume_external_left = alc5628_set_volume;
	ops->set_volume_external_right = alc5628_set_volume;
	ops->switch_line_in = alc5628_switch_line_in;
	ops->set_volume_dB = alc5628_set_volume_dB;

	alc5628_i2c_init();
}

/* EOF */
